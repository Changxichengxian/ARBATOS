"""运行固定版本的 RM WBR 原模型与控制核心，保存实际轨迹和评价结果。"""

from __future__ import annotations

import argparse
import csv
import ctypes
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / "local/reference/wbr_mujoco"
REVISION = "81a1035e66315a46abd1f0ca4c11ec40596b95c5"
CONFIG_HASH = "f29b286308b5e49f6a5ea7949ff339f9ce867a7a24177959ecf2da0bf7bcc7cb"
MOTORS = ("ljoint1", "ljoint4", "lwheel", "rjoint1", "rjoint4", "rwheel")
DT = 0.001


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_bridge(source: Path, output: Path, *, correct_spring_derivatives=False):
    # 沿用本项目已支持的主机编译器查找，不依赖嵌入式 ARM 工具链。
    sys.path.insert(0, str(ROOT / "tools/mujoco/wheelleg"))
    from RunWheelLeg import find_compiler
    compiler = find_compiler()
    if compiler is None:
        raise RuntimeError("需要 Zig、Clang、GCC 或 MSVC 主机编译器")
    executable, tags = compiler
    suffix = "_gas" if correct_spring_derivatives else ""
    library = output / f"wbr_bridge_{os.getpid()}{suffix}{'.dll' if os.name == 'nt' else '.so'}"
    sources = [Path(__file__).with_name("WbrBridge.cpp")]
    sources += [source / f"controller/src/{name}.cpp" for name in ("balance", "leg", "chassis_fsm")]
    if correct_spring_derivatives:
        # 原仓库保持不变。仅纠正气弹簧闭链求导的两个符号，临时源码随报告保留。
        leg_source = source / "controller/src/leg.cpp"
        content = leg_source.read_bytes()
        for expression in (b"cfg_.l1 * std::sin(u3_ - phi1_) * inv_l2s23",
                           b"cfg_.l1 * std::sin(phi4_ - u3_) * inv_l2s23"):
            original = b"= " + expression + b";"
            if content.count(original) != 1:
                raise RuntimeError("原作者气弹簧公式与固定版本不一致，拒绝自动修正")
            content = content.replace(original, b"= -" + expression + b";")
        corrected = output / "leg_spring_corrected.cpp"
        corrected.write_bytes(content)
        sources[2] = corrected
        (output / "spring-correction.json").write_text(json.dumps(dict(
            original_source=str(leg_source), original_sha256=digest(leg_source),
            corrected_source=str(corrected), corrected_sha256=digest(corrected),
            change="negate dphi2_dphi1 and dphi2_dphi4 in calc_spring_force only"), indent=2), encoding="utf-8")
    # 起身的可达下限单独配置，正常调腿高仍保留原下限；原 clone 不改写。
    include = output / "upstream-adapted-include"
    shutil.copytree(source / "controller/include", include, dirs_exist_ok=True)
    config_path = include / "controller/config.hpp"
    config_data = config_path.read_bytes()
    field = b"float lmin = 0.16f;"
    if config_data.count(field) != 1:
        raise RuntimeError("原作者起身配置与固定版本不一致")
    config_path.write_bytes(config_data.replace(field,
        field + b"\n    float rise_min_length = 0.16f;"))
    fsm_source = source / "controller/src/chassis_fsm.cpp"
    fsm_data = fsm_source.read_bytes()
    boundary = b"void chassis_fsm::step_normal("
    if fsm_data.count(boundary) != 1:
        raise RuntimeError("原作者起身状态机与固定版本不一致")
    rise_data, normal_data = fsm_data.split(boundary, 1)
    if rise_data.count(b"cfg_.lmin") != 6:
        raise RuntimeError("原作者起身腿高引用与固定版本不一致")
    adapted_fsm = output / "chassis_fsm_rise_length.cpp"
    adapted_fsm.write_bytes(rise_data.replace(b"cfg_.lmin", b"cfg_.rise_min_length") + boundary + normal_data)
    sources[3] = adapted_fsm
    (output / "rise-length-adaptation.json").write_text(json.dumps(dict(
        original_source=str(fsm_source), original_sha256=digest(fsm_source),
        adapted_source=str(adapted_fsm), adapted_sha256=digest(adapted_fsm),
        change="separate six initialization/standup length references from normal lmin",
        config_sha256=digest(config_path)), indent=2), encoding="utf-8")
    if "msvc" in tags:
        command = [executable, "/nologo", "/LD", "/O2", "/EHsc", "/std:c++17", f"/I{include}"]
        command += [str(path) for path in sources] + [f"/Fe{library}"]
    else:
        command = [executable, "c++"] if "zig" in tags else [executable]
        command += ["-shared", "-O2", "-std=c++17", f"-I{include}"]
        if os.name != "nt":
            command += ["-fPIC"]
        command += [str(path) for path in sources] + ["-o", str(library)]
    subprocess.run(command, cwd=output, check=True)
    lib = ctypes.CDLL(str(library))
    ptr = ctypes.POINTER(ctypes.c_float)
    lib.WbrCreate.restype = ctypes.c_void_p
    lib.WbrCreateWithRiseLength.argtypes = [ctypes.c_float]
    lib.WbrCreateWithRiseLength.restype = ctypes.c_void_p
    lib.WbrDestroy.argtypes = [ctypes.c_void_p]
    lib.WbrStep.argtypes = [ctypes.c_void_p, ptr, ptr, ptr]
    lib.WbrStep.restype = ctypes.c_int
    return lib


def speed_command(t, scenario):
    if scenario == "drive":
        if 10 <= t < 15:
            return 0.25
        if 18 <= t < 23:
            return -0.25
    return 0.0


def inspect_state(model, data, output, command):
    jac = np.zeros((3, model.nv))
    body = model.body("base_link").id
    mujoco.mj_jacBody(model, data, jac, None, body)
    velocity = jac @ data.qvel
    rotation = data.body(body).xmat.reshape(3, 3)
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    forward = velocity[0] * math.cos(yaw) + velocity[1] * math.sin(yaw)
    floor = model.geom("floor").id
    wheel_bodies = {model.body("lwlink").id, model.body("rwlink").id}
    wheel_contacts, other_contacts = 0, 0
    for contact in data.contact:
        if floor not in (contact.geom1, contact.geom2):
            continue
        other = contact.geom2 if contact.geom1 == floor else contact.geom1
        if model.geom_bodyid[other] in wheel_bodies:
            wheel_contacts += 1
        else:
            other_contacts += 1
    residual = max(float(np.linalg.norm(data.site(f"{side}link4_site{i}").xpos
                                        - data.site(f"{side}link1_site{i}").xpos))
                   for side in ("l", "r") for i in (1, 2))
    row = {
        "time_s": float(data.time), "command_v_mps": float(command[1]),
        "x_m": float(data.xpos[body, 0]), "y_m": float(data.xpos[body, 1]),
        "z_m": float(data.xpos[body, 2]), "true_v_mps": float(forward),
        "odom_v_mps": float(output[10]), "odom_x_m": float(output[9]),
        "roll_deg": math.degrees(output[6]), "pitch_deg": math.degrees(output[7]),
        "yaw_deg": math.degrees(output[8]), "fsm": int(output[24]),
        "left_leg_m": float(output[12]), "right_leg_m": float(output[18]),
        "wheel_contacts": wheel_contacts, "other_floor_contacts": other_contacts,
        "closure_m": residual,
    }
    for i, name in enumerate(MOTORS):
        row[f"{name}_requested_nm"] = float(output[i])
        row[f"{name}_previous_applied_nm"] = float(data.actuator_force[i])
    return row


def evaluate(rows, scenario, duration, failure):
    settled = [r for r in rows if r["time_s"] >= 8]
    reasons = [failure] if failure else []
    result = {"scenario": scenario, "simulated_s": rows[-1]["time_s"] if rows else 0}
    if not settled:
        reasons.append("no_settled_window")
    else:
        result.update(
            peak_pitch_deg=max(abs(r["pitch_deg"]) for r in settled),
            peak_roll_deg=max(abs(r["roll_deg"]) for r in settled),
            normal_fraction=sum(r["fsm"] == 4 for r in settled) / len(settled),
            max_closure_m=max(r["closure_m"] for r in settled),
            body_floor_fraction=sum(r["other_floor_contacts"] > 0 for r in settled) / len(settled),
        )
        if result["peak_pitch_deg"] > 30 or result["peak_roll_deg"] > 30:
            reasons.append("tilt_over_30deg")
        if result["normal_fraction"] < 0.95:
            reasons.append("not_in_normal_control")
        if result["body_floor_fraction"] > 0:
            reasons.append("body_or_leg_touching_floor")
        if result["max_closure_m"] > 0.01:
            reasons.append("closure_over_10mm")
    windows = [("stand", 8, duration, 0)] if scenario == "stand" else [
        ("stand", 8, 10, 0), ("forward", 12, 15, .25),
        ("stop1", 16, 18, 0), ("reverse", 20, 23, -.25), ("stop2", 25, duration, 0),
    ]
    result["windows"] = {}
    for name, start, end, target in windows:
        samples = [r for r in rows if start <= r["time_s"] < end]
        if not samples or samples[-1]["time_s"] < end - .02:
            reasons.append(f"{name}_incomplete")
            continue
        average = float(np.mean([r["true_v_mps"] for r in samples]))
        rmse = float(np.sqrt(np.mean([(r["true_v_mps"] - target) ** 2 for r in samples])))
        result["windows"][name] = {"target_mps": target, "mean_true_mps": average, "rmse_mps": rmse,
                                  "x_start_m": samples[0]["x_m"], "x_end_m": samples[-1]["x_m"]}
        if rmse > .10:
            reasons.append(f"{name}_speed_error")
        if target != 0 and average * target <= 0:
            reasons.append(f"{name}_wrong_direction")
    result.update(passed=not reasons, failures=reasons)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=SOURCE)
    parser.add_argument("--output", type=Path, default=ROOT / "local/cache/wbr-reference-20261003/run")
    parser.add_argument("--scenario", choices=("stand", "drive"), default="drive")
    parser.add_argument("--duration", type=float, default=30)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < (27 if args.scenario == "drive" else 10):
        parser.error("站立至少10秒；完整行驶场景至少27秒")
    args.source, args.output = args.source.resolve(), args.output.resolve()
    if not args.source.is_dir():
        parser.error("先克隆 https://github.com/CosmosMount/wbr_mujoco 到 --source 目录")
    revision = subprocess.check_output(["git", "-C", str(args.source), "rev-parse", "HEAD"], text=True).strip()
    if revision != REVISION or digest(args.source / "config/robots/wbr.yaml") != CONFIG_HASH:
        parser.error("源码或 YAML 与桥接接口固定版本不符，请核对参数后再运行")
    if subprocess.run(["git", "-C", str(args.source), "diff", "--quiet", "HEAD", "--",
                       "controller", "mjcf", "config"]).returncode != 0:
        parser.error("第三方控制、模型或配置存在修改；本工具要求使用固定版本原文件")
    args.output.mkdir(parents=True, exist_ok=True)
    lib = load_bridge(args.source, args.output)
    handle = lib.WbrCreate()
    if not handle:
        raise RuntimeError("WbrCreate failed")
    model = mujoco.MjModel.from_xml_path(str(args.source / "mjcf/scene.xml"))
    model.opt.timestep = DT  # 上游 YAML 覆盖 MJCF 的默认 2 ms。
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)
    sensors = ["imu_quat", "imu_gyro", "imu_accel"]
    sensors += [f"{motor}_{kind}" for motor in MOTORS for kind in ("pos", "vel", "tor")]
    sensor_indices = np.concatenate([np.arange(model.sensor_adr[model.sensor(name).id],
                                              model.sensor_adr[model.sensor(name).id] + model.sensor_dim[model.sensor(name).id])
                                     for name in sensors])
    sensor = (ctypes.c_float * 28)()
    command = (ctypes.c_float * 7)(0, 0, .16, 0, 0, 0, 0)
    output = (ctypes.c_float * 32)()
    rows, poses, pose_times = [], [], []
    failure = None
    try:
        for tick in range(math.ceil(args.duration / DT) + 1):
            command[1] = speed_command(data.time, args.scenario)
            command[5] = 1 if data.time >= .5 else 0
            command[0] = command[0] + command[1] * DT if output[31] > .5 else output[9]
            sensor[:] = data.sensordata[sensor_indices]
            if lib.WbrStep(handle, sensor, command, output) != 0:
                failure = "controller_error"
                break
            if tick % 10 == 0:
                row = inspect_state(model, data, output, command)
                rows.append(row)
                poses.append(data.qpos.copy())
                pose_times.append(float(data.time))
                if data.time > 8 and max(abs(row["roll_deg"]), abs(row["pitch_deg"])) > 60:
                    failure = "fell_over"
                    break
            if tick % 1000 == 0:
                print(f"t={data.time:.1f} fsm={int(output[24])} pitch={math.degrees(output[7]):+.2f} "
                      f"v={rows[-1]['true_v_mps']:+.3f} cmd={command[1]:+.2f} z={rows[-1]['z_m']:.3f}", flush=True)
            if tick == math.ceil(args.duration / DT):
                break
            data.ctrl[:] = output[:6]
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            if not np.isfinite(data.qpos).all() or any(w.number for w in data.warning):
                failure = "mujoco_warning_or_nonfinite"
                break
    finally:
        lib.WbrDestroy(handle)
    if not rows:
        raise RuntimeError(f"仿真未产生有效采样：{failure}")
    with (args.output / "trace.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    np.savez_compressed(args.output / "trajectory.npz", time=pose_times, qpos=poses)
    report = evaluate(rows, args.scenario, args.duration, failure)
    report.update(source_url="https://github.com/CosmosMount/wbr_mujoco", source_revision=revision,
                  mujoco=mujoco.__version__, control_dt_s=DT, physics_dt_s=DT,
                  total_mass_kg=float(sum(model.body_mass)), nq=model.nq, nv=model.nv,
                  initial_pose="upstream home keyframe", imu_mode="upstream bypass",
                  scope="Original controller .cpp, synchronous direct commands; eCAL/keyboard/threads excluded",
                  evaluation={"settle_until_s": 8, "tilt_limit_deg": 30,
                              "normal_fraction_min": .95, "closure_limit_m": .01,
                              "speed_rmse_limit_mps": .10, "other_floor_contacts_allowed": False},
                  output_sha256={name: digest(args.output / name) for name in ("trace.csv", "trajectory.npz")},
                  source_sha256={str(p.relative_to(args.source)): digest(p)
                                 for folder in ("controller", "mjcf", "config")
                                 for p in (args.source / folder).rglob("*") if p.is_file()},
                  adapter_sha256={p.name: digest(p) for p in (Path(__file__), Path(__file__).with_name("WbrBridge.cpp"))})
    (args.output / "summary.json").write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k not in ("source_sha256", "adapter_sha256")}, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
