"""从 WBR home 低姿态验收自有 Core 起身、常态运动和一次跳跃。"""
from __future__ import annotations

import argparse
import csv
import ctypes
import hashlib
import json
import math
from pathlib import Path
import subprocess

import mujoco
import numpy as np

from RunOwnWbr import load_core
from RunWbr import MOTORS, REVISION, ROOT, SOURCE, inspect_state
from WbrGasSpring import load_model


DT = .001
CONTROL_DT = .003
CONTINUOUS_NM = 10.0
PEAK_NM = 45.0
GAS_FORCE_N = 450.0
JUMP_PROFILE = 0
CORRECT_LINK_COLLISIONS = False
MAX_RISE_SECONDS = None
MAX_RISE_DISPLACEMENT_M = None


def configure_library(lib) -> None:
    pointer = ctypes.POINTER(ctypes.c_float)
    lib.OwnWbrCommandStep.argtypes = [ctypes.c_void_p, pointer, pointer, pointer]
    lib.OwnWbrCommandStep.restype = ctypes.c_int
    lib.OwnWbrSetJointPeak.argtypes = [ctypes.c_void_p, ctypes.c_float]
    lib.OwnWbrSetJointPeak.restype = ctypes.c_int
    lib.OwnWbrSetJointLimits.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float]
    lib.OwnWbrSetJointLimits.restype = ctypes.c_int
    lib.OwnWbrEnableRise.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.OwnWbrEnableRise.restype = ctypes.c_int
    lib.OwnWbrSetStage.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.OwnWbrSetStage.restype = ctypes.c_int
    lib.OwnWbrSetGasSpring.argtypes = [ctypes.c_void_p, ctypes.c_float]
    lib.OwnWbrSetGasSpring.restype = ctypes.c_int
    lib.OwnWbrSetJumpProfile.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.OwnWbrSetJumpProfile.restype = ctypes.c_int


def configure_model() -> tuple[mujoco.MjModel, np.ndarray]:
    model = load_model(SOURCE / "mjcf/scene.xml", gas_force_n=GAS_FORCE_N,
                       correct_link_collisions=CORRECT_LINK_COLLISIONS)
    model.opt.timestep = DT
    for actuator_name, joint_name in (
        ("ljoint1_actuator", "ljoint1"), ("ljoint4_actuator", "ljoint4"),
        ("rjoint1_actuator", "rjoint1"), ("rjoint4_actuator", "rjoint4"),
    ):
        actuator = model.actuator(actuator_name).id
        joint = model.joint(joint_name).id
        model.actuator_ctrlrange[actuator] = (-PEAK_NM, PEAK_NM)
        model.actuator_forcerange[actuator] = (-PEAK_NM, PEAK_NM)
        model.jnt_actfrcrange[joint] = (-PEAK_NM, PEAK_NM)
    names = ["imu_quat", "imu_gyro", "imu_accel"]
    names += [f"{motor}_{kind}" for motor in MOTORS for kind in ("pos", "vel", "tor")]
    indices = np.concatenate([
        np.arange(model.sensor(name).adr[0],
                  model.sensor(name).adr[0] + model.sensor(name).dim[0])
        for name in names
    ])
    return model, indices


def make_handle(lib, config, enable_rise: bool):
    handle = lib.OwnWbrCreate(config)
    if not handle:
        raise RuntimeError("控制器初始化失败")
    # 旧入口仍须接受 45 N·m；随后用新入口明确 10/45 两级限制。
    if not lib.OwnWbrSetJointPeak(handle, PEAK_NM):
        raise RuntimeError("OwnWbrSetJointPeak 未接受 45 N·m")
    if not lib.OwnWbrSetJointLimits(handle, CONTINUOUS_NM, PEAK_NM):
        raise RuntimeError("OwnWbrSetJointLimits 未接受 10/45 N·m")
    if not lib.OwnWbrSetGasSpring(handle, GAS_FORCE_N):
        raise RuntimeError("气弹簧补偿配置无效")
    if not lib.OwnWbrSetJumpProfile(handle, JUMP_PROFILE + (2 if CORRECT_LINK_COLLISIONS else 0)):
        raise RuntimeError("跳跃动作配置无效")
    if enable_rise and not lib.OwnWbrEnableRise(handle, 1):
        raise RuntimeError("OwnWbrEnableRise 启用失败")
    return handle


def command_after_rise(elapsed: float) -> tuple[float, float, float]:
    if 2.0 <= elapsed < 4.0:
        return .20, 0.0, 0.0
    if 5.0 <= elapsed < 7.0:
        return -.20, 0.0, 0.0
    if 8.0 <= elapsed < 10.0:
        return 0.0, .40, 0.0
    if 11.0 <= elapsed < 13.0:
        return 0.0, -.40, 0.0
    if 14.0 <= elapsed < 14.05:
        return 0.0, 0.0, 1.0
    return 0.0, 0.0, 0.0


def longest_run(times: list[float], flags: list[bool]) -> tuple[float, float, float]:
    best = (0.0, 0.0, 0.0)
    start = None
    previous = None
    for time_s, flag in zip(times, flags):
        if flag and start is None:
            start = time_s
        if flag:
            previous = time_s
        elif start is not None:
            best = max(best, (previous - start + DT, start, previous))
            start = None
    if start is not None:
        best = max(best, (previous - start + DT, start, previous))
    return best


def mean_window(rows: list[dict], key: str, begin: float, end: float) -> float:
    values = [row[key] for row in rows if begin <= row["after_rise_s"] < end]
    return float(np.mean(values))


def run_rise_trial(lib, config, trial: int, save_trace: bool) -> tuple[dict, list[dict], list, list]:
    model, sensor_indices = configure_model()
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    home_qpos = data.qpos.copy()
    mujoco.mj_forward(model, data)
    handle = make_handle(lib, config, True)
    sensor = (ctypes.c_float * 28)()
    command = (ctypes.c_float * 3)()
    output = (ctypes.c_float * 40)()
    rows: list[dict] = []
    poses, pose_times = [], []
    rise_done = None
    failure = None
    phase_transitions = []
    previous_phase = None
    hip_rpm_max = 0.0

    try:
        for tick in range(round(30.0 / DT) + 1):
            time_s = float(data.time)
            if rise_done is None:
                requested = (.25, .40, 1.0) if .50 <= time_s < 1.00 else (0.0, 0.0, 0.0)
            else:
                requested = command_after_rise(time_s - rise_done)
            if tick % 3 == 0:
                sensor[:] = data.sensordata[sensor_indices]
                command[:] = requested
                status = lib.OwnWbrCommandStep(handle, sensor, command, output)
                if status:
                    failure = f"core_error_{status}"
                    break
                data.ctrl[:] = output[:6]

            phase = int(round(output[34]))
            if phase != previous_phase:
                phase_transitions.append({"time_s": time_s, "phase": phase,
                                          "limit_nm": float(output[31])})
                previous_phase = phase
            if phase < 0:
                failure = f"rise_failed_{phase}"
                break
            if rise_done is None and phase == 0 and output[35] > 0.0:
                rise_done = time_s

            command_view = np.array([0.0, requested[0], 0.0, 0.0, 0.0, 0.0, requested[1]])
            row = inspect_state(model, data, output, command_view)
            del row["fsm"]
            row.update(
                trial=trial,
                rise_phase=phase,
                rise_elapsed_s=float(output[35]),
                after_rise_s=time_s - rise_done if rise_done is not None else -1.0,
                requested_yaw_rate_radps=requested[1],
                jump_button=requested[2],
                jump_phase=int(round(output[29])),
                jump_count=int(round(output[30])),
                joint_limit_nm=float(output[31]),
                joint_limit_constraints=int(np.count_nonzero(
                    data.efc_type == mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT)),
                ramped_v_mps=float(output[26]),
                ramped_yaw_rate_radps=float(output[28]),
                left_wheel_z_m=float(data.body("lwlink").xpos[2]),
                right_wheel_z_m=float(data.body("rwlink").xpos[2]),
            )
            rows.append(row)
            for name in ("ljoint1", "ljoint4", "rjoint1", "rjoint4"):
                speed = abs(float(data.qvel[model.joint(name).dofadr[0]])) * 60.0 / (2.0 * math.pi)
                hip_rpm_max = max(hip_rpm_max, speed)
            if save_trace and tick % 10 == 0:
                poses.append(data.qpos.copy())
                pose_times.append(time_s)

            if rise_done is not None and time_s - rise_done >= 20.0:
                break
            if tick == round(30.0 / DT):
                failure = "trial_timeout"
                break
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            if not np.isfinite(data.qpos).all() or any(warning.number for warning in data.warning):
                failure = "mujoco_warning_or_nonfinite"
                break
    finally:
        lib.OwnWbrDestroy(handle)

    failures = [failure] if failure else []
    if rise_done is None:
        failures.append("rise_did_not_complete")
        return ({"trial": trial, "passed": False, "failures": failures,
                 "phase_transitions": phase_transitions}, rows, poses, pose_times)

    after = [row for row in rows if row["after_rise_s"] >= 0.0]
    final = [row for row in after if row["after_rise_s"] >= 18.0]
    startup = [row for row in rows if .50 <= row["time_s"] < 1.00]
    # 增强跳落地前 0.18 秒也允许峰值，按实际限幅窗口统计持续力矩。
    normal_samples = [row for row in after if row["joint_limit_nm"] <= CONTINUOUS_NM + 1e-4]
    peak_samples = [row for row in rows if row["joint_limit_nm"] > CONTINUOUS_NM + 1e-4]
    torque_keys = ("ljoint1_requested_nm", "ljoint4_requested_nm",
                   "rjoint1_requested_nm", "rjoint4_requested_nm")
    torque_max = max(abs(row[key]) for row in rows for key in torque_keys)
    continuous_max = max(abs(row[key]) for row in normal_samples for key in torque_keys)
    above_flags = [max(abs(row[key]) for key in torque_keys) > CONTINUOUS_NM + 1e-4 for row in rows]
    above_run, above_start, above_end = longest_run([row["time_s"] for row in rows], above_flags)

    airborne = [row["jump_count"] > 0 and row["wheel_contacts"] == 0 for row in rows]
    air_time, air_start, air_end = longest_run([row["time_s"] for row in rows], airborne)
    air_rows = [row for row in rows if air_start <= row["time_s"] <= air_end] if air_time else []
    clearance = max((min(row["left_wheel_z_m"], row["right_wheel_z_m"])
                     for row in air_rows), default=.057997) - .057997

    forward = mean_window(after, "true_v_mps", 3.0, 4.0)
    reverse = mean_window(after, "true_v_mps", 6.0, 7.0)
    yaw_l0 = mean_window(after, "yaw_deg", 7.8, 8.0)
    yaw_l1 = mean_window(after, "yaw_deg", 9.8, 10.0)
    yaw_r0 = mean_window(after, "yaw_deg", 10.8, 11.0)
    yaw_r1 = mean_window(after, "yaw_deg", 12.8, 13.0)
    post_nonwheel = sum(row["other_floor_contacts"] > 0 for row in after)
    final_speed_rms = float(np.sqrt(np.mean([row["true_v_mps"] ** 2 for row in final]))) if final else None
    final_pitch = max((abs(row["pitch_deg"]) for row in final), default=None)
    rise_rows = [row for row in rows if row["time_s"] <= rise_done]
    def displacement(row):
        return math.hypot(row["x_m"] - rows[0]["x_m"], row["y_m"] - rows[0]["y_m"])
    rise_displacement = max(displacement(row) for row in rise_rows)
    if MAX_RISE_SECONDS is not None and rise_done > MAX_RISE_SECONDS:
        failures.append("rise_duration_exceeded")
    if MAX_RISE_DISPLACEMENT_M is not None and rise_displacement > MAX_RISE_DISPLACEMENT_M:
        failures.append("rise_displacement_exceeded")
    if startup and (max(abs(row["ramped_v_mps"]) for row in startup) > 1e-5 or
                    max(abs(row["ramped_yaw_rate_radps"]) for row in startup) > 1e-5 or
                    max(row["jump_count"] for row in startup) != 0):
        failures.append("startup_command_not_ignored")
    if (torque_max > PEAK_NM + 1e-4 or continuous_max > CONTINUOUS_NM + 1e-4 or
            any(abs(row[key]) > row["joint_limit_nm"] + 1e-4
                for row in rows for key in torque_keys)):
        failures.append("joint_torque_limit_exceeded")
    if post_nonwheel:
        failures.append("body_or_leg_contact_after_rise")
    post_joint_limits = sum(row["joint_limit_constraints"] > 0 for row in after)
    if JUMP_PROFILE == 1 and post_joint_limits:
        failures.append("joint_limit_after_rise")
    max_closure_after_rise = max(row["closure_m"] for row in after)
    if max_closure_after_rise > .001:
        failures.append("closure_over_1mm")
    if forward < .08 or reverse > -.08:
        failures.append("forward_or_reverse_response")
    if yaw_l1 - yaw_l0 < 10.0 or yaw_r1 - yaw_r0 > -10.0:
        failures.append("turn_response")
    if max(row["jump_count"] for row in rows) != 1 or air_time < .120 or clearance < .030:
        failures.append("jump_below_120ms_or_30mm")
    if final_pitch is None or final_speed_rms is None or final_pitch > 5.0 or final_speed_rms > .10:
        failures.append("post_action_not_stable")

    report = {
        "trial": trial,
        "passed": not failures,
        "failures": failures,
        "home_qpos_exact": bool(np.array_equal(home_qpos, model.key("home").qpos)),
        "rise_completed_s": rise_done,
        "rise_max_horizontal_displacement_m": rise_displacement,
        "rise_completion_displacement_m": displacement(after[0]),
        "rise_pitch_peak_deg": max(abs(row["pitch_deg"]) for row in rise_rows),
        "rise_last_nonwheel_contact_s": max((row["time_s"] for row in rise_rows
            if row["other_floor_contacts"]), default=None),
        "rise_last_joint_limit_s": max((row["time_s"] for row in rise_rows
            if row["joint_limit_constraints"]), default=None),
        "phase_transitions": phase_transitions,
        "startup_commands_ignored": "startup_command_not_ignored" not in failures,
        "joint_torque_abs_max_nm": torque_max,
        "continuous_window_torque_abs_max_nm": continuous_max,
        "peak_limit_active_s": len(peak_samples) * DT,
        "longest_actual_above_10nm_s": above_run,
        "actual_above_10nm_interval_s": [above_start, above_end],
        "hip_speed_abs_max_rpm": hip_rpm_max,
        "forward_mean_mps": forward,
        "reverse_mean_mps": reverse,
        "left_turn_delta_deg": yaw_l1 - yaw_l0,
        "right_turn_delta_deg": yaw_r1 - yaw_r0,
        "jump_airborne_s": air_time,
        "jump_clearance_m": clearance,
        "post_rise_nonwheel_contact_samples": post_nonwheel,
        "post_rise_joint_limit_samples": post_joint_limits,
        "home_initial_closure_m": rows[0]["closure_m"],
        "max_closure_after_rise_m": max_closure_after_rise,
        "final_2s_pitch_peak_deg": final_pitch,
        "final_2s_speed_rms_mps": final_speed_rms,
    }
    return report, rows, poses, pose_times


def run_stage_probe(lib, config, stage: int) -> dict:
    model, sensor_indices = configure_model()
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)
    handle = make_handle(lib, config, False)
    if not lib.OwnWbrSetStage(handle, stage):
        lib.OwnWbrDestroy(handle)
        raise RuntimeError(f"阶段 {stage} 配置失败")
    if not lib.OwnWbrEnableRise(handle, 1):
        lib.OwnWbrDestroy(handle)
        raise RuntimeError(f"阶段 {stage} 起身启用失败")
    sensor = (ctypes.c_float * 28)()
    command = (ctypes.c_float * 3)()
    output = (ctypes.c_float * 40)()
    completed = None
    failed_phase = None
    physics_fault = False
    post_pitch = []
    post_contacts = 0
    try:
        for tick in range(round(12.0 / DT) + 1):
            if tick % 3 == 0:
                sensor[:] = data.sensordata[sensor_indices]
                status = lib.OwnWbrCommandStep(handle, sensor, command, output)
                if status:
                    raise RuntimeError(f"阶段 {stage} core_error_{status}")
                data.ctrl[:] = output[:6]
            phase = int(round(output[34]))
            if phase < 0:
                failed_phase = phase
                break
            if completed is None and phase == 0 and output[35] > 0.0:
                completed = float(data.time)
            if completed is not None:
                post_pitch.append(abs(math.degrees(float(output[7]))))
                row = inspect_state(model, data, output, np.zeros(7))
                post_contacts += int(row["other_floor_contacts"] > 0)
                if data.time - completed >= 2.0:
                    break
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            if not np.isfinite(data.qpos).all() or any(w.number for w in data.warning):
                physics_fault = True
                break
    finally:
        lib.OwnWbrDestroy(handle)
    expected = (not physics_fault and completed is not None and failed_phase is None and post_contacts == 0 and
                max(post_pitch, default=180.0) < 8.0)
    return {
        "stage": stage,
        "passed": bool(expected),
        "completed_s": completed,
        "failed_phase": failed_phase,
        "mujoco_warning_or_nonfinite": physics_fault,
        "post_completion_pitch_peak_deg": max(post_pitch) if post_pitch else None,
        "post_completion_nonwheel_contact_samples": post_contacts,
        "expectation": "complete from home and remain upright for two seconds; failure must come from measured stability timeout",
    }


def run_hold10(lib, config) -> dict:
    model, sensor_indices = configure_model()
    data = mujoco.MjData(model)
    profile = json.loads((Path(__file__).with_name("OwnWbrProfile.json")).read_text(encoding="utf-8"))
    data.qpos[:] = profile["qpos"]
    mujoco.mj_forward(model, data)
    handle = make_handle(lib, config, False)
    sensor = (ctypes.c_float * 28)()
    command = (ctypes.c_float * 3)()
    output = (ctypes.c_float * 40)()
    lengths = [[], []]
    pitches, speeds, torques = [], [], []
    nonwheel_contacts = 0
    physics_fault = False
    try:
        for tick in range(round(15.0 / DT) + 1):
            if tick % 3 == 0:
                sensor[:] = data.sensordata[sensor_indices]
                status = lib.OwnWbrCommandStep(handle, sensor, command, output)
                if status:
                    raise RuntimeError(f"10 N·m 保持测试 core_error_{status}")
                data.ctrl[:] = output[:6]
            lengths[0].append(float(output[12]))
            lengths[1].append(float(output[18]))
            pitches.append(abs(math.degrees(float(output[7]))))
            speeds.append(float(data.qvel[0]))
            torques.append(max(abs(float(output[index])) for index in (0, 1, 3, 4)))
            row = inspect_state(model, data, output, np.zeros(7))
            nonwheel_contacts += int(row["other_floor_contacts"] > 0)
            if tick < round(15.0 / DT):
                mujoco.mj_step(model, data)
                mujoco.mj_forward(model, data)
                if not np.isfinite(data.qpos).all() or any(warning.number for warning in data.warning):
                    physics_fault = True
                    break
    finally:
        lib.OwnWbrDestroy(handle)
    final_slice = slice(-2000, None)
    final_lengths = [float(np.mean(values[final_slice])) for values in lengths]
    pitch_peak = max(pitches)
    speed_rms = float(np.sqrt(np.mean(np.square(speeds))))
    torque_max = max(torques)
    passed = (pitch_peak < 5.0 and speed_rms < .05 and torque_max <= CONTINUOUS_NM + 1e-4 and
              nonwheel_contacts == 0 and not physics_fault)
    return {
        "passed": passed,
        "simulated_s": 15.0,
        "target_length_m": .20,
        "final_2s_mean_leg_length_m": final_lengths,
        "target_within_5mm": all(abs(value - .20) <= .005 for value in final_lengths),
        "pitch_peak_deg": pitch_peak,
        "speed_rms_mps": speed_rms,
        "joint_torque_abs_max_nm": torque_max,
        "nonwheel_contact_samples": nonwheel_contacts,
        "mujoco_warning_or_nonfinite": physics_fault,
        "note": "10 N·m 可保持直立，但该项如为 false，表示不能严格保持两侧 0.20 m。",
    }


def run_long_hold(lib, config, early_jump: bool, destination: Path) -> dict:
    """起身后保持一分钟，覆盖短动作验收曾漏掉的缓慢侧倾。"""
    model, indices = configure_model()
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)
    handle = make_handle(lib, config, True)
    sensor, command, output = (ctypes.c_float * 28)(), (ctypes.c_float * 3)(), (ctypes.c_float * 40)()
    ready_at, failure = None, None
    rows = []
    try:
        for tick in range(round(75.0 / DT) + 1):
            elapsed = float(data.time) - ready_at if ready_at is not None else -1.0
            command[:] = (0.0, 0.0, float(early_jump and .03 <= elapsed < .08))
            if tick % 3 == 0:
                sensor[:] = data.sensordata[indices]
                error = lib.OwnWbrCommandStep(handle, sensor, command, output)
                if error or output[34] < 0:
                    failure = f"control_or_rise_failed_{error}_{int(output[34])}"
                    break
                data.ctrl[:] = output[:6]
            if ready_at is None and output[34] == 0 and output[35] > 0:
                ready_at = float(data.time)
                elapsed = 0.0
            row = inspect_state(model, data, output, np.zeros(7))
            row.update(after_rise_s=elapsed, jump_count=int(output[30]),
                       jump_phase=int(output[29]), joint_limit_nm=float(output[31]),
                       joint_limit_constraints=int(np.count_nonzero(
                           data.efc_type == mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT)))
            if tick % 10 == 0:
                rows.append(row)
            if ready_at is not None:
                if any(abs(row[name + "_requested_nm"]) > min(row["joint_limit_nm"], PEAK_NM) + 1e-4
                       for name in ("ljoint1", "ljoint4", "rjoint1", "rjoint4")):
                    failure = "active_torque_limit_exceeded"
                    break
                if row["other_floor_contacts"]:
                    failure = "nonwheel_contact_after_rise"
                    break
                if JUMP_PROFILE == 1 and row["joint_limit_constraints"]:
                    failure = "joint_limit_after_rise"
                    break
                if max(abs(row["pitch_deg"]), abs(row["roll_deg"])) > 15:
                    failure = "post_rise_tilt_over_15deg"
                    break
                if elapsed >= 60:
                    break
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            if not np.isfinite(data.qpos).all() or any(w.number for w in data.warning):
                failure = "mujoco_warning_or_nonfinite"
                break
    finally:
        lib.OwnWbrDestroy(handle)
    after = [row for row in rows if row["after_rise_s"] >= 0]
    normal = [row for row in after if row["joint_limit_nm"] <= CONTINUOUS_NM + 1e-4]
    names = ("ljoint1", "ljoint4", "rjoint1", "rjoint4")
    report = dict(early_jump=early_jump, failure=failure, rise_completed_s=ready_at,
                  post_rise_simulated_s=float(data.time) - ready_at if ready_at is not None else 0,
                  observed_jump_count=int(output[30]))
    report["passed"] = (failure is None and report["post_rise_simulated_s"] >= 60 and
                        int(output[30]) == int(early_jump) and int(output[29]) == 0)
    if after:
        report.update(pitch_peak_deg=max(abs(row["pitch_deg"]) for row in after),
                      roll_peak_deg=max(abs(row["roll_deg"]) for row in after),
                      speed_rms_mps=float(np.sqrt(np.mean([row["true_v_mps"] ** 2 for row in after]))))
    if normal:
        report["normal_motor_abs_mean_nm"] = {name: float(np.mean([
            abs(row[name + "_requested_nm"]) for row in normal])) for name in names}
        report["normal_motor_abs_max_nm"] = max(abs(row[name + "_requested_nm"])
            for row in normal for name in names)
        if report["normal_motor_abs_max_nm"] > CONTINUOUS_NM + 1e-4:
            report["passed"] = False
            report["failure"] = "continuous_torque_exceeded"
    if rows:
        filename = "long_early_jump.csv" if early_jump else "long_stand.csv"
        with (destination / filename).open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    return report


def main() -> int:
    global GAS_FORCE_N, JUMP_PROFILE, CORRECT_LINK_COLLISIONS
    global MAX_RISE_SECONDS, MAX_RISE_DISPLACEMENT_M
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "local/cache/wbr-own-rise/final")
    parser.add_argument("--trials", type=int, default=3)
    parser.add_argument("--gas-spring-n", type=float, choices=(0.0, 450.0), default=450.0)
    parser.add_argument("--jump-profile", type=int, choices=(0, 1), default=0)
    parser.add_argument("--correct-link-collisions", action="store_true")
    parser.add_argument("--max-rise-seconds", type=float)
    parser.add_argument("--max-rise-displacement-m", type=float)
    args = parser.parse_args()
    if args.trials < 1:
        parser.error("--trials 至少为 1，不能跳过起身验收")
    for value in (args.max_rise_seconds, args.max_rise_displacement_m):
        if value is not None and (not math.isfinite(value) or value <= 0):
            parser.error("起身时间和位移上限必须是正的有限数")
    GAS_FORCE_N = args.gas_spring_n
    JUMP_PROFILE = args.jump_profile
    CORRECT_LINK_COLLISIONS = args.correct_link_collisions
    MAX_RISE_SECONDS = args.max_rise_seconds
    MAX_RISE_DISPLACEMENT_M = args.max_rise_displacement_m
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    revision = subprocess.check_output(
        ["git", "-C", str(SOURCE), "rev-parse", "HEAD"], text=True).strip()
    if revision != REVISION:
        parser.error("WBR 版本不符")
    if subprocess.run(["git", "-C", str(SOURCE), "diff", "--quiet", "HEAD", "--", "mjcf"]).returncode:
        parser.error("WBR 模型存在修改")

    profile = json.loads((Path(__file__).with_name("OwnWbrProfile.json")).read_text(encoding="utf-8"))
    config = (ctypes.c_float * 20)(
        *np.asarray(profile["negative_k"]).flatten(), profile["theta0"], profile["tp0"],
        profile["wheel0"], profile["support_n"], profile["length_m"], profile["radius_m"],
        profile.get("length_kp", 1000), profile.get("length_kd", 60))
    lib, build_command = load_core(args.output)
    configure_library(lib)

    hold10 = run_hold10(lib, config)
    stage_probes = [run_stage_probe(lib, config, stage) for stage in (0, 1, 2)]
    long_holds = [run_long_hold(lib, config, early, args.output) for early in (False, True)]
    trial_reports = []
    first_rows = []
    first_poses = []
    first_times = []
    for trial in range(1, args.trials + 1):
        report, rows, poses, times = run_rise_trial(lib, config, trial, trial == 1)
        trial_reports.append(report)
        print(json.dumps(report, ensure_ascii=False), flush=True)
        if trial == 1:
            first_rows, first_poses, first_times = rows, poses, times

    summary = {
        "passed": (hold10["passed"] and all(report["passed"] for report in long_holds) and
                   all(report["passed"] for report in trial_reports) and
                   all(report["passed"] for report in stage_probes)),
        "source_revision": revision,
        "controller_sha256": hashlib.sha256(
            Path(__file__).with_name("OwnWbrBridge.c").read_bytes()).hexdigest().upper(),
        "build_command": build_command,
        "control_inputs": "IMU and six motor sensors only; qpos/contact/velocity truth is evaluation only",
        "initial_condition": "unmodified WBR home keyframe for every rise trial",
        "controller": "ARBATOS WheelLegCore with bridge rise sequence; no upstream controller or trajectory",
        "joint_limits_nm": {"continuous": CONTINUOUS_NM, "peak": PEAK_NM},
        "gas_spring_n_each": GAS_FORCE_N,
        "jump_profile": JUMP_PROFILE,
        "correct_link_collisions": CORRECT_LINK_COLLISIONS,
        "required_max_rise_seconds": MAX_RISE_SECONDS,
        "required_max_rise_displacement_m": MAX_RISE_DISPLACEMENT_M,
        "rated_speed_rpm": 200,
        "rated_speed_interpretation": "recorded as rated speed, not enforced as a maximum-speed curve",
        "peak_duration_status": "manufacturer peak-duration limit not yet verified; measured intervals are reported per trial",
        "hold10": hold10,
        "long_holds": long_holds,
        "stage_handoff_probes": stage_probes,
        "trials": trial_reports,
    }
    if first_rows:
        with (args.output / "trace.csv").open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(first_rows[0]))
            writer.writeheader()
            writer.writerows(first_rows)
        np.savez_compressed(args.output / "trajectory.npz", time=first_times, qpos=first_poses)
    (args.output / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
