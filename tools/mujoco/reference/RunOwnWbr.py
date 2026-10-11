"""用 ARBATOS WheelLegCore 控制固定版本 WBR 模型，记录传感器闭环结果。"""
from __future__ import annotations

import argparse
import csv
import ctypes
import json
import math
import os
from pathlib import Path
import subprocess
import sys

import mujoco
import numpy as np

from RunWbr import ROOT, SOURCE, REVISION, MOTORS, digest, inspect_state, speed_command


def load_core(output):
    sys.path.insert(0, str(ROOT / "tools/mujoco/wheelleg"))
    from RunWheelLeg import find_compiler
    compiler = find_compiler()
    if compiler is None:
        raise RuntimeError("缺少主机 C 编译器")
    executable, tags = compiler
    library = output / f"own_wbr_{os.getpid()}{'.dll' if os.name == 'nt' else '.so'}"
    source = Path(__file__).with_name("OwnWbrBridge.c")
    includes = [ROOT / p for p in ("shared/application/wheelleg", "shared/application/robot",
                                  "shared/components/support")]
    if "msvc" in tags:
        command = [executable, "/nologo", "/LD", "/O2"] + [f"/I{p}" for p in includes]
        command += [str(source), f"/Fe{library}"]
    else:
        command = [executable, "cc"] if "zig" in tags else [executable]
        command += ["-shared", "-O2", "-std=c99"] + [f"-I{p}" for p in includes]
        command += [str(source), "-o", str(library)]
        if os.name != "nt":
            command += ["-fPIC", "-lm"]
    subprocess.run(command, cwd=output, check=True)
    lib = ctypes.CDLL(str(library))
    ptr = ctypes.POINTER(ctypes.c_float)
    lib.OwnWbrCreate.argtypes = [ptr]
    lib.OwnWbrCreate.restype = ctypes.c_void_p
    lib.OwnWbrDestroy.argtypes = [ctypes.c_void_p]
    lib.OwnWbrStep.argtypes = [ctypes.c_void_p, ptr, ctypes.c_float, ptr]
    lib.OwnWbrStep.restype = ctypes.c_int
    return lib, command


def evaluate(rows, scenario, duration, failure):
    reasons = [failure] if failure else []
    # 所有接触/倾倒判定从释放时开始，稳态速度窗口与原控制对照保持一致。
    if any(r["other_floor_contacts"] for r in rows):
        reasons.append("body_or_leg_touched_floor")
    peak_pitch = max(abs(r["pitch_deg"]) for r in rows)
    peak_roll = max(abs(r["roll_deg"]) for r in rows)
    if max(peak_pitch, peak_roll) > 30:
        reasons.append("tilt_over_30deg")
    closure = max(r["closure_m"] for r in rows)
    if closure > .001:
        reasons.append("closure_over_1mm")
    peak_yaw = max(abs(r["yaw_deg"]) for r in rows)
    lateral_span = max(r["y_m"] for r in rows)-min(r["y_m"] for r in rows)
    if peak_yaw > 5 or lateral_span > .05:
        reasons.append("straight_line_drift")
    windows = [("stand", 8, duration, 0)] if scenario == "stand" else [
        ("stand", 8, 10, 0), ("forward", 12, 15, .25), ("stop1", 16, 18, 0),
        ("reverse", 20, 23, -.25), ("stop2", 25, duration, 0)]
    results = {}
    for name, start, end, target in windows:
        samples = [r for r in rows if start <= r["time_s"] < end]
        if not samples or samples[-1]["time_s"] < end - .03:
            reasons.append(f"{name}_incomplete")
            continue
        mean = float(np.mean([r["true_v_mps"] for r in samples]))
        rmse = float(np.sqrt(np.mean([(r["true_v_mps"]-target)**2 for r in samples])))
        results[name] = dict(target_mps=target, mean_true_mps=mean, rmse_mps=rmse,
                             x_start_m=samples[0]["x_m"], x_end_m=samples[-1]["x_m"])
        if rmse > .10 or (target and mean*target <= 0):
            reasons.append(f"{name}_speed_error")
    return dict(passed=not reasons, failures=reasons, scenario=scenario,
                simulated_s=rows[-1]["time_s"], peak_pitch_deg=peak_pitch,
                peak_roll_deg=peak_roll, max_closure_m=closure, windows=results,
                peak_yaw_deg=peak_yaw, lateral_span_m=lateral_span,
                nonwheel_contact_samples=sum(r["other_floor_contacts"] > 0 for r in rows),
                observer_rmse_mps=float(np.sqrt(np.mean([(r["odom_v_mps"]-r["true_v_mps"])**2
                                                        for r in rows]))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=SOURCE)
    parser.add_argument("--profile", type=Path, default=Path(__file__).with_name("OwnWbrProfile.json"))
    parser.add_argument("--output", type=Path, default=ROOT / "local/cache/wbr-own-20261003/drive")
    parser.add_argument("--scenario", choices=("stand", "drive"), default="drive")
    parser.add_argument("--duration", type=float, default=30)
    parser.add_argument("--pitch-offset-deg", type=float, default=0)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("时长必须为正数")
    if not math.isfinite(args.pitch_offset_deg):
        parser.error("初始倾角必须为有限数")
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    revision = subprocess.check_output(["git", "-C", str(args.source), "rev-parse", "HEAD"], text=True).strip()
    if revision != REVISION:
        parser.error("WBR 版本不符")
    if subprocess.run(["git", "-C", str(args.source), "diff", "--quiet", "HEAD", "--", "mjcf"]).returncode:
        parser.error("WBR 模型存在修改")
    profile = json.loads(args.profile.read_text(encoding="utf-8"))
    config = (ctypes.c_float * 20)(*np.asarray(profile["negative_k"]).flatten(),
        profile["theta0"], profile["tp0"], profile["wheel0"], profile["support_n"],
        profile["length_m"], profile["radius_m"], profile.get("length_kp", 1000), profile.get("length_kd", 60))
    lib, build_command = load_core(args.output)
    handle = lib.OwnWbrCreate(config)
    if not handle:
        raise RuntimeError("控制器初始化失败")
    model = mujoco.MjModel.from_xml_path(str(args.source / "mjcf/scene.xml"))
    model.opt.timestep = .001
    data = mujoco.MjData(model)
    data.qpos[:] = profile["qpos"]
    mujoco.mj_forward(model, data)
    if args.pitch_offset_deg:
        # 绕两轮轴中点旋转整机，保持轮高和腿内闭链，随后自由释放。
        angle = math.radians(args.pitch_offset_deg)
        pivot = .5 * (data.body("lwlink").xpos + data.body("rwlink").xpos)
        delta = np.array([math.cos(angle/2), 0, math.sin(angle/2), 0])
        rotation = np.array([[math.cos(angle), 0, math.sin(angle)], [0, 1, 0],
                             [-math.sin(angle), 0, math.cos(angle)]])
        data.qpos[:3] = pivot + rotation @ (data.qpos[:3] - pivot)
        quaternion = np.empty(4)
        mujoco.mju_mulQuat(quaternion, delta, data.qpos[3:7].copy())
        data.qpos[3:7] = quaternion
    mujoco.mj_forward(model, data)
    names = ["imu_quat", "imu_gyro", "imu_accel"]
    names += [f"{motor}_{kind}" for motor in MOTORS for kind in ("pos", "vel", "tor")]
    indices = np.concatenate([np.arange(model.sensor(n).adr[0], model.sensor(n).adr[0]+model.sensor(n).dim[0])
                              for n in names])
    sensor = (ctypes.c_float * 28)()
    output = (ctypes.c_float * 40)()
    command = np.zeros(7)
    rows, poses, times = [], [], []
    failure = None
    try:
        for tick in range(round(args.duration/.001)+1):
            command[1] = speed_command(data.time, args.scenario)
            if tick % 3 == 0:
                sensor[:] = data.sensordata[indices]
                error = lib.OwnWbrStep(handle, sensor, command[1], output)
                if error:
                    failure = f"core_error_{error}"
                    break
                data.ctrl[:] = output[:6]
            if tick % 10 == 0:
                row = inspect_state(model, data, output, command)
                del row["fsm"]
                row["core_valid"] = int(output[24])
                row["target_x_m"], row["ramped_v_mps"] = float(output[25]), float(output[26])
                row["left_theta_rad"], row["right_theta_rad"] = float(output[14]), float(output[20])
                row["left_tp_nm"], row["right_tp_nm"] = float(output[16]), float(output[22])
                row["left_f_n"], row["right_f_n"] = float(output[17]), float(output[23])
                rows.append(row)
                poses.append(data.qpos.copy())
                times.append(float(data.time))
                if max(abs(row["roll_deg"]), abs(row["pitch_deg"])) > 60:
                    failure = "fell_over"
                    break
            if tick % 1000 == 0:
                print(f"t={data.time:.1f} pitch={math.degrees(output[7]):+.2f} "
                      f"v={rows[-1]['true_v_mps']:+.3f} obs={output[10]:+.3f} "
                      f"cmd={command[1]:+.2f} z={rows[-1]['z_m']:.3f}", flush=True)
            if tick == round(args.duration/.001):
                break
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            if not np.isfinite(data.qpos).all() or any(w.number for w in data.warning):
                failure = "mujoco_warning_or_nonfinite"
                break
    finally:
        lib.OwnWbrDestroy(handle)
    if not rows:
        raise RuntimeError(f"未产生有效数据：{failure}")
    with (args.output / "trace.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    np.savez_compressed(args.output / "trajectory.npz", time=times, qpos=poses)
    report = evaluate(rows, args.scenario, args.duration, failure)
    report.update(controller="ARBATOS WheelLegCore", source_url="https://github.com/CosmosMount/wbr_mujoco",
        source_revision=revision, mujoco=mujoco.__version__, control_dt_s=.003, physics_dt_s=.001,
        initial_pose="independent closed-chain standing solve; no startup controller",
        initial_pitch_offset_deg=args.pitch_offset_deg,
        scope="Own kinematics, observer, common-mode LQR output, length/split PID, VMC, roll/yaw; model-specific host adapter",
        excluded="Zephyr tasks, CAN, original firmware state machine, stand-up sequence, upstream controller",
        total_mass_kg=float(model.body_mass.sum()), profile=profile,
        build_command=build_command,
        evaluation=dict(tilt_limit_deg=30, closure_limit_m=.001, speed_rmse_limit_mps=.10,
                        yaw_limit_deg=5, lateral_span_limit_m=.05, any_nonwheel_floor_contact_fails=True),
        source_sha256={str(p.relative_to(args.source)):digest(p) for p in (args.source / "mjcf").rglob("*") if p.is_file()},
        core_sha256={str(p.relative_to(ROOT)):digest(p) for folder in ("shared/application/wheelleg", "shared/components/support")
                     for p in (ROOT / folder).glob("*.h")},
        adapter_sha256={p.name:digest(p) for p in (Path(__file__), Path(__file__).with_name("OwnWbrBridge.c"),
                                               Path(__file__).with_name("DesignOwnWbr.py"), args.profile)},
        output_sha256={name:digest(args.output/name) for name in ("trace.csv", "trajectory.npz")})
    (args.output / "summary.json").write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps({k:report[k] for k in ("passed", "failures", "simulated_s", "peak_pitch_deg", "observer_rmse_mps", "windows")}, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
