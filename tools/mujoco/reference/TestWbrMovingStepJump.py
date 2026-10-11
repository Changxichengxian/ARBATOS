"""8009不同速度跳上200mm标准台：固定时间模拟按钮，真值只用于验收。"""
from __future__ import annotations

import argparse
import csv
import ctypes
import hashlib
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from RunWbr import MOTORS, inspect_state
from SweepWbrGasActions import Energy, contact_metrics, configure
from TestWbrImpactClimb import load_core, geom_world_aabb
from WbrGasSpring import load_model
from WbrKneeRange import knee_range_info
from WbrTerrain import ground_geom_ids

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
HIPS = ("ljoint1", "ljoint4", "rjoint1", "rjoint4")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest().upper()


def contacts(model, data, grounds, target):
    wheels = [model.body(s + "wlink").id for s in ("l", "r")]
    loads = [0., 0.]
    wheel_grounds = [[], []]
    target_contacts = []
    nonwheel, edge = [], []
    max_penetration = 0.
    for i, c in enumerate(data.contact):
        if c.geom1 in grounds:
            ground, other, nz = c.geom1, c.geom2, float(c.frame[2])
        elif c.geom2 in grounds:
            ground, other, nz = c.geom2, c.geom1, -float(c.frame[2])
        else:
            continue
        body = int(model.geom_bodyid[other])
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, force)
        if float(c.dist) > 0.:
            continue
        max_penetration = max(max_penetration, -float(c.dist))
        if body not in wheels:
            nonwheel.append([model.body(body).name, model.geom(ground).name])
        else:
            side = wheels.index(body)
            wheel_grounds[side].append(model.geom(ground).name)
            if ground == target:
                target_contacts.append([side, nz, float(c.dist), float(force[0]), *map(float, c.pos)])
                if nz >= .90:
                    loads[side] += max(0., float(force[0]) * nz)
                else:
                    edge.append([model.body(body).name, nz, float(force[0])])
    return dict(target_load_n=loads, nonwheel=nonwheel, edge=edge,
                wheel_grounds=wheel_grounds, target_contacts=target_contacts,
                max_ground_penetration_m=max_penetration)


def run(args):
    script_hash = digest(__file__)
    if not .5 <= args.speed <= 1.5:
        raise ValueError("本轮速度扫描范围为0.5～1.5m/s")
    dest = args.output.resolve()
    dest.mkdir(parents=True, exist_ok=True)
    snapshot = dest / "OwnWbrBridge.executed.c"
    snapshot.write_bytes(args.source.read_bytes())
    (dest / "OwnWbrStages.h").write_bytes((HERE / "OwnWbrStages.h").read_bytes())
    model = load_model(gas_force_n=args.gas, terrain="course", guide_rollers="passive",
        correct_link_collisions=True, knee_range="stop180", noslip_iterations=10)
    model.opt.timestep = .001
    stop = knee_range_info(model)
    for name in HIPS:
        aid, jid = model.actuator(name + "_actuator").id, model.joint(name).id
        model.actuator_ctrlrange[aid] = model.actuator_forcerange[aid] = [-40., 40.]
        model.jnt_actfrcrange[jid] = [-40., 40.]
    profile = json.loads((HERE / "OwnWbrProfile.json").read_text(encoding="utf-8"))
    data = mujoco.MjData(model)
    data.qpos[:len(profile["qpos"])] = profile["qpos"]
    data.qpos[0], data.qpos[1] = args.start_x, -17.
    mujoco.mj_forward(model, data)
    initial = inspect_state(model, data, np.zeros(40), np.zeros(7))
    if initial["closure_m"] > 1e-6:
        raise RuntimeError("起始闭链无效")
    grounds = ground_geom_ids(model)
    initial_penetration = max((-float(c.dist) for c in data.contact
        if c.geom1 in grounds or c.geom2 in grounds), default=0.)
    if initial_penetration > .0005:
        raise RuntimeError("起始地面穿透超过0.5mm")
    if contact_metrics(model, data)["other_joint_limit_constraints"]:
        raise RuntimeError("起始姿态触发关节长端或其他限位")
    target = model.geom("terrain_climb200_platform").id
    aabb = geom_world_aabb(model, data, target)
    if not np.allclose(aabb, [[9., -18.5, -.1], [11., -15.5, .2]], atol=1e-9):
        raise RuntimeError("目标200mm台阶几何发生变化")
    edge_x, far_x, top_z = float(aabb[0][0]), float(aabb[1][0]), float(aabb[1][2])
    radius = profile["radius_m"]
    lib, build = load_core(dest, snapshot)
    configure(lib)
    for name in ("OwnWbrSetStepJump", "OwnWbrSetStepClimb", "OwnWbrSetDriveMode"):
        fn = getattr(lib, name)
        fn.argtypes = [ctypes.c_void_p, ctypes.c_int]
        fn.restype = ctypes.c_int
    config = (ctypes.c_float * 20)(*np.asarray(profile["negative_k"]).flatten(),
        profile["theta0"], profile["tp0"], profile["wheel0"], profile["support_n"],
        .20, radius, profile["length_kp"], profile["length_kd"])
    handle = lib.OwnWbrCreate(config)
    if not handle:
        raise RuntimeError("创建控制器失败")
    for name, values in (("OwnWbrSetJointLimits", (20., 40.)), ("OwnWbrSetGasSpring", (args.gas,)),
        ("OwnWbrSetClimbFold", (0,)), ("OwnWbrSetJumpProfile", (3,)),
        ("OwnWbrSetStage", (3,)), ("OwnWbrEnableRise", (0,)),
        ("OwnWbrSetStepClimb", (0,)), ("OwnWbrSetStepJump", (1,)), ("OwnWbrSetDriveMode", (1,))):
        if getattr(lib, name)(handle, *values) != 1:
            raise RuntimeError(name + "配置被拒绝")
    sensor_names = ["imu_quat", "imu_gyro", "imu_accel"] + [f"{m}_{k}" for m in MOTORS for k in ("pos", "vel", "tor")]
    indices = np.concatenate([np.arange(model.sensor(n).adr[0], model.sensor(n).adr[0] + model.sensor(n).dim[0]) for n in sensor_names])
    sensor, command, output = (ctypes.c_float*28)(), (ctypes.c_float*3)(), (ctypes.c_float*40)()
    energy = Energy(model)
    rows, qtimes, qpos = [], [], []
    triggered = None
    failure = None
    try:
        for tick in range(round(args.seconds/.001)):
            t = float(data.time)
            if tick % 3 == 0:
                if triggered is None and t >= args.trigger_time:
                    triggered = t
                sensor[:] = data.sensordata[indices]
                command[:] = (args.speed if t >= 2 else 0., 0., float(triggered is not None and t-triggered < .09))
                code = lib.OwnWbrCommandStep(handle, sensor, command, output)
                if code:
                    failure = f"controller_error_{code}"
                    break
                data.ctrl[:] = output[:6]
            mujoco.mj_forward(model, data)
            row = inspect_state(model, data, output, np.zeros(7))
            row.update(contact_metrics(model, data))
            row.update(contacts(model, data, grounds, target))
            wheels = [data.body(s+"wlink").xpos.copy() for s in ("l", "r")]
            rotation = data.body("base_link").xmat.reshape(3, 3)
            row.update(time_s=t, jump_phase=int(output[29]), jump_count=int(output[30]),
                command_v_mps=float(command[0]), command_jump_pressed=float(command[2]),
                true_pitch_deg=math.degrees(math.atan2(-rotation[2, 0], math.hypot(rotation[0, 0], rotation[1, 0]))),
                true_roll_deg=math.degrees(math.atan2(rotation[2, 1], rotation[2, 2])),
                v_ref_mps=float(output[26]), joint_limit_nm=float(output[31]),
                xy_speed_mps=float(np.linalg.norm(data.qvel[:2])),
                body_rate_radps=float(np.linalg.norm(data.qvel[3:6])),
                left_x_m=float(wheels[0][0]), right_x_m=float(wheels[1][0]),
                left_y_m=float(wheels[0][1]), right_y_m=float(wheels[1][1]),
                left_bottom_m=float(wheels[0][2]-radius), right_bottom_m=float(wheels[1][2]-radius))
            rows.append(row)
            if tick % 5 == 0:
                qtimes.append(t)
                qpos.append(data.qpos.copy())
            energy.sample(data)
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            if not np.isfinite(data.qpos).all() or any(w.number for w in data.warning):
                failure = "nonfinite_or_mujoco_warning"
                break
            if max(abs(row["true_roll_deg"]), abs(row["true_pitch_deg"])) > 65:
                failure = "tilt_over_65deg"
                break
            if args.early_fail and (row["nonwheel"] or row["edge"] or
                    row["other_joint_limit_constraints"] or
                    (row["jump_phase"] and row["short_stop_constraints"])):
                # 扫描时只提前结束已经失败的样本，成功样本仍执行完整稳站验收。
                failure = "early_exit_after_contact_or_limit_failure"
                break
    finally:
        lib.OwnWbrDestroy(handle)
    with (dest/"trace.csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    energy.write(dest/"motor-trace.csv")
    np.savez_compressed(dest/"qpos-trajectory.npz", time_s=qtimes, qpos=qpos)
    reasons = [failure] if failure else []
    air_runs, start = [], None
    for r in rows:
        airborne = r["jump_phase"] in (2, 3, 4) and r["jump_count"] == 1 and not r["nonwheel"] and not any(r["wheel_grounds"])
        if airborne and start is None:
            start = r
        elif not airborne and start is not None:
            air_runs.append((start, r))
            start = None
    # 只验收推蹬后的第一次完整腾空，不把失败后翻滚或弹跳当作起跳。
    flight = next((pair for pair in air_runs if pair[0]["jump_phase"] in (2, 3)
        and pair[1]["time_s"]-pair[0]["time_s"] >= .12), None)
    if flight is None or flight[1]["time_s"]-flight[0]["time_s"] < .12:
        reasons.append("no_valid_flight")
    preparation = [r for r in rows if triggered is not None and flight and triggered <= r["time_s"] <= flight[0]["time_s"]]
    moving = bool(preparation) and min(r["true_v_mps"] for r in preparation) >= .70*args.speed
    moving = moving and .70*args.speed <= flight[0]["true_v_mps"] <= 1.30*args.speed
    if preparation:
        moving = moving and .90*args.speed <= preparation[0]["true_v_mps"] <= 1.10*args.speed
    if not moving:
        reasons.append("preparation_or_takeoff_not_moving_at_target")
    crossed = []
    flight_rows = [r for r in rows if flight and flight[0]["time_s"] <= r["time_s"] < flight[1]["time_s"]]
    center_clearance = []
    for side in ("left", "right"):
        # 圆轮用台角净空判定，不能把整个轮的矩形包络当作实体。
        centers = [(a, b) for a, b in zip(flight_rows, flight_rows[1:])
            if a[side+"_x_m"] <= edge_x <= b[side+"_x_m"] and b[side+"_x_m"] > a[side+"_x_m"]]
        clearance = None
        if centers:
            a, b = centers[0]
            fraction = (edge_x-a[side+"_x_m"])/(b[side+"_x_m"]-a[side+"_x_m"])
            clearance = a[side+"_bottom_m"]+fraction*(b[side+"_bottom_m"]-a[side+"_bottom_m"])-top_z
        center_clearance.append(clearance)
        overlap = [r for r in flight_rows if edge_x-radius <= r[side+"_x_m"] < edge_x]
        corner_clear = all((math.hypot(edge_x-r[side+"_x_m"], r[side+"_bottom_m"]+radius-top_z)
            if r[side+"_bottom_m"]+radius >= top_z else edge_x-r[side+"_x_m"]) >= radius for r in overlap)
        over_top = [r for r in flight_rows if r[side+"_x_m"] >= edge_x]
        crossed.append(clearance is not None and clearance >= .005 and corner_clear and
            bool(over_top) and min(r[side+"_bottom_m"] for r in over_top) >= top_z-.0005)
    if not all(crossed):
        reasons.append("wheel_did_not_clear_edge")
    takeoff_before_edge = bool(flight) and max(flight[0][side+"_x_m"]+radius for side in ("left", "right")) <= edge_x-.005
    first_per_wheel = [next((r for r in rows if any(c[0] == i for c in r["target_contacts"])), None) for i in range(2)]
    # 轮心及真实接触点进入顶面20mm才接受落台；后缘整轮净空另列更保守的诊断。
    landed_past_edge = all(r is not None and r[s+"_x_m"] >= edge_x+.020 and
        all(c[1] >= .90 and c[4] >= edge_x+.020 for c in r["target_contacts"] if c[0] == i)
        for i, (s, r) in enumerate(zip(("left", "right"), first_per_wheel)))
    rear_margins = [r[s+"_x_m"]-radius-edge_x if r else None for s, r in zip(("left", "right"), first_per_wheel)]
    clean_flight = takeoff_before_edge and landed_past_edge
    if not clean_flight:
        reasons.append("flight_did_not_span_entire_edge")
    first_target = next((r for r in rows if r["target_contacts"]), None)
    if first_target is None or not flight or abs(first_target["time_s"]-flight[1]["time_s"]) > .002 or any(c[1] < .90 for c in first_target["target_contacts"]):
        reasons.append("first_landing_not_target_top")
    if any(r["nonwheel"] for r in rows):
        reasons.append("body_or_leg_contact")
    if any(r["edge"] for r in rows):
        reasons.append("wheel_hit_platform_edge")
    if any(r["other_joint_limit_constraints"] for r in rows):
        reasons.append("other_joint_limit")
    if any(r["short_stop_constraints"] for r in rows if r["jump_phase"] != 0):
        reasons.append("jump_hit_short_stop")
    if max(max(r["stop_penetration_rad"]) for r in rows) > .01:
        reasons.append("stop_penetration")
    if max(r["closure_m"] for r in rows) > .001:
        reasons.append("closure_over_1mm")
    if max(r["max_ground_penetration_m"] for r in rows) > .015:
        reasons.append("ground_penetration_over_15mm")
    final = [r for r in rows if r["time_s"] >= args.seconds-2]
    stable = len(final) >= 1999 and all(min(r["target_load_n"]) > 5 and
        edge_x+radius+.10 <= min(r["left_x_m"],r["right_x_m"]) and max(r["left_x_m"],r["right_x_m"]) <= far_x-radius-.10 and
        all(aabb[0][1]+radius+.10 <= r[s+"_y_m"] <= aabb[1][1]-radius-.10 and abs(r[s+"_bottom_m"]-top_z) < .010 for s in ("left", "right")) and
        r["xy_speed_mps"] < .10 and r["body_rate_radps"] < .5 and r["jump_phase"] == 0 and
        max(abs(r["true_pitch_deg"]),abs(r["true_roll_deg"])) < 5 and
        all(set(g) == {"terrain_climb200_platform"} for g in r["wheel_grounds"]) for r in final)
    stable = stable and max(r["x_m"] for r in final)-min(r["x_m"] for r in final) < .05 and max(r["y_m"] for r in final)-min(r["y_m"] for r in final) < .02
    if not stable:
        reasons.append("not_stable_on_top_last_2s")
    if rows[-1]["jump_count"] != 1 or rows[-1]["jump_phase"] != 0:
        reasons.append("jump_sequence_incomplete")
    phase_sequence = [r["jump_phase"] for i, r in enumerate(rows) if not i or r["jump_phase"] != rows[i-1]["jump_phase"]]
    if phase_sequence != [0, 1, 2, 3, 4, 5, 0]:
        reasons.append("jump_phase_sequence_invalid")
    if digest(snapshot) != digest(args.source) or digest(__file__) != script_hash:
        reasons.append("source_changed_during_run")
    button_edges = sum(r["command_jump_pressed"] > .5 and (not i or rows[i-1]["command_jump_pressed"] <= .5) for i, r in enumerate(rows))
    button_duration = sum(r["command_jump_pressed"] > .5 for r in rows)*.001
    if button_edges != 1 or not .087 <= button_duration <= .096:
        reasons.append("button_pulse_invalid")
    if not preparation or min(r["v_ref_mps"] for r in preparation) < .7*args.speed or any(abs(r["command_v_mps"]-args.speed) > 1e-5 for r in preparation):
        reasons.append("preparation_command_braked")
    if any(abs(r[n+"_requested_nm"]) > min(40.,r["joint_limit_nm"])+1e-4 for r in rows for n in HIPS):
        reasons.append("motor_limit_violation")
    stats = energy.summary(args.seconds)
    budget = {}
    for name in HIPS:
        over = [abs(r[name+"_torque_nm"]) > 20.0001 for r in energy.rows]
        longest = current = 0
        for hit in over:
            current = current+1 if hit else 0
            longest = max(longest, current)
        budget[name] = dict(total_above_20nm_s=sum(over)*.001, longest_above_20nm_s=longest*.001)
        if longest*.001 > .5:
            reasons.append(name+"_peak_budget_over_0.5s")
        if stats["hips"][name]["peak_nm"] > 40.0001:
            reasons.append(name+"_actual_torque_over_40nm")
        if any(abs(r[name+"_torque_nm"]) > 20.0001 for r in energy.rows if r["time_s"] >= args.seconds-2):
            reasons.append(name+"_final_above_continuous_limit")
    report = dict(passed=not reasons, failures=reasons, gas_force_n=args.gas, speed_command_mps=args.speed,
        trigger_schedule_s=args.trigger_time, trigger_time_s=triggered,
        trigger_actual_x_m=next((r["x_m"] for r in rows if triggered is not None and r["time_s"] >= triggered), None),
        takeoff_time_s=flight[0]["time_s"] if flight else None,
        takeoff_actual_speed_mps=flight[0]["true_v_mps"] if flight else None,
        takeoff_x_m=flight[0]["x_m"] if flight else None,
        prep_min_actual_speed_mps=min((r["true_v_mps"] for r in preparation), default=None),
        airborne_s=flight[1]["time_s"]-flight[0]["time_s"] if flight else None,
        max_wheel_bottom_m=max((min(r["left_bottom_m"],r["right_bottom_m"]) for r in flight_rows), default=None),
        stable_on_platform=stable, wheel_clearance_over_edge=crossed,
        wheel_center_clearance_m=center_clearance, phase_sequence=phase_sequence,
        rear_edge_margin_at_first_top_contact_m=rear_margins,
        full_wheel_rear_clearance_5mm=all(m is not None and m >= .005 for m in rear_margins),
        button_pulses=button_edges, button_duration_s=button_duration,
        clean_flight_spanning_edge=clean_flight, hip_20nm_budget=budget,
        final_x_m=rows[-1]["x_m"], final_pitch_deg=rows[-1]["pitch_deg"],
        nonwheel_contact_samples=sum(bool(r["nonwheel"]) for r in rows),
        short_stop_contact_samples=sum(bool(r["short_stop_constraints"]) for r in rows),
        edge_contact_samples=sum(bool(r["edge"]) for r in rows),
        initial_closure_m=initial["closure_m"], initial_penetration_m=initial_penetration, physical_stop=stop,
        physics=dict(continuous_nm=20., peak_nm=40., step_s=.001, platform_aabb=[p.tolist() for p in aabb]),
        energy=stats, controller_sha256=digest(snapshot), source_path=str(args.source),
        test_script_sha256=script_hash,
        source_changed_during_run=digest(snapshot)!=digest(args.source), build=build,
        boundary="固定时间触发一次模拟按钮，位置/接触真值只验收；控制器只用IMU/编码器；独立直立开始，不包含起身；20/40仅输出边界，不是完整8009热/电压模型。")
    (dest/"summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps({k:v for k,v in report.items() if k not in ("energy","build","physical_stop")}, ensure_ascii=False))
    return int(bool(reasons))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--source", type=Path, default=HERE/"OwnWbrBridge.c")
    parser.add_argument("--gas", type=float, default=450.)
    parser.add_argument("--speed", type=float, default=1.)
    parser.add_argument("--start-x", type=float, default=4.5)
    parser.add_argument("--trigger-time", type=float, default=6.)
    parser.add_argument("--seconds", type=float, default=14.)
    parser.add_argument("--early-fail", action="store_true",
                        help="扫描时提前结束已撞击或触限的失败样本，成功样本仍跑完整时长")
    raise SystemExit(run(parser.parse_args()))
