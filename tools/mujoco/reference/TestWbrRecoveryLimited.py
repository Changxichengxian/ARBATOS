"""验证保留物理状态的单次受限自起、失败清零与保护边界。"""
from __future__ import annotations

import argparse
import csv
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[3]
REFERENCE = ROOT / "tools/mujoco/reference"
sys.path.insert(0, str(REFERENCE))
from RunWbr import MOTORS, SOURCE  # noqa: E402
from WbrGasSpring import load_model  # noqa: E402
import WbrGuideRollers  # noqa: E402
from WbrGuideRollers import guide_roller_info  # noqa: E402
from WbrLinkCollision import link_collision_info  # noqa: E402
from WbrTerrain import ground_geom_ids  # noqa: E402


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest().upper()


def float_array_sha256(values):
    return hashlib.sha256(np.asarray(values, dtype=np.float64).tobytes()).hexdigest().upper()


def model_data_sha256(model):
    """覆盖MjModel公开数组和关键选项，区分几何、导轮及编译参数。"""
    digest = hashlib.sha256()
    for name in sorted(dir(model)):
        try:
            value = getattr(model, name)
        except Exception:
            continue
        if isinstance(value, np.ndarray):
            array = np.ascontiguousarray(value)
            digest.update(name.encode("ascii"))
            digest.update(str(array.dtype).encode("ascii"))
            digest.update(np.asarray(array.shape, dtype=np.int64).tobytes())
            digest.update(array.tobytes())
    digest.update(np.asarray([
        model.opt.timestep, model.opt.gravity[0], model.opt.gravity[1], model.opt.gravity[2],
        model.opt.noslip_iterations,
    ], dtype=np.float64).tobytes())
    return digest.hexdigest().upper()


def load_core(source, output):
    sys.path.insert(0, str(ROOT / "tools/mujoco/wheelleg"))
    from RunWheelLeg import find_compiler
    executable, tags = find_compiler()
    library = output / f"recovery_{os.getpid()}.dll"
    includes = [ROOT / path for path in ("tools/mujoco/reference", "shared/application/wheelleg",
        "shared/application/robot", "shared/components/support")]
    command = [executable, "cc"] if "zig" in tags else [executable]
    command += ["-shared", "-O2", "-std=c99"] + [f"-I{path}" for path in includes]
    command += [str(source), "-o", str(library)]
    subprocess.run(command, cwd=output, check=True)
    lib = ctypes.CDLL(str(library))
    pointer = ctypes.POINTER(ctypes.c_float)
    bindings = {
        "OwnWbrCreate": ([pointer], ctypes.c_void_p),
        "OwnWbrDestroy": ([ctypes.c_void_p], None),
        "OwnWbrCommandStep": ([ctypes.c_void_p, pointer, pointer, pointer], ctypes.c_int),
        "OwnWbrSetStage": ([ctypes.c_void_p, ctypes.c_int], ctypes.c_int),
        "OwnWbrSetJointLimits": ([ctypes.c_void_p, ctypes.c_float, ctypes.c_float], ctypes.c_int),
        "OwnWbrSetGasSpring": ([ctypes.c_void_p, ctypes.c_float], ctypes.c_int),
        "OwnWbrSetJumpProfile": ([ctypes.c_void_p, ctypes.c_int], ctypes.c_int),
        "OwnWbrSetDriveMode": ([ctypes.c_void_p, ctypes.c_int], ctypes.c_int),
        "OwnWbrSetLegLength": ([ctypes.c_void_p, ctypes.c_float], ctypes.c_int),
        "OwnWbrEnableRise": ([ctypes.c_void_p, ctypes.c_int], ctypes.c_int),
    }
    if hasattr(lib, "OwnWbrRequestRecovery"):
        bindings["OwnWbrRequestRecovery"] = ([ctypes.c_void_p], ctypes.c_int)
    for name, (args, result) in bindings.items():
        function = getattr(lib, name)
        function.argtypes = args
        function.restype = result
    return lib, command


def create_handle(lib, profile, recovery, length_m=0.20):
    config = (ctypes.c_float * 20)(*np.asarray(profile["negative_k"]).flatten(),
        profile["theta0"], profile["tp0"], profile["wheel0"], profile["support_n"],
        profile["length_m"], profile["radius_m"], profile["length_kp"], profile["length_kd"])
    handle = lib.OwnWbrCreate(config)
    if not handle:
        raise RuntimeError("create failed")
    ok = [lib.OwnWbrSetStage(handle, 3), lib.OwnWbrSetJointLimits(handle, 10.0, 45.0),
          lib.OwnWbrSetGasSpring(handle, 450.0), lib.OwnWbrSetJumpProfile(handle, 2),
          lib.OwnWbrSetDriveMode(handle, 0), lib.OwnWbrSetLegLength(handle, length_m)]
    request = (lib.OwnWbrRequestRecovery(handle) if recovery and hasattr(lib, "OwnWbrRequestRecovery")
               else lib.OwnWbrEnableRise(handle, 1) if recovery else 1)
    if not all(ok) or not request:
        lib.OwnWbrDestroy(handle)
        raise RuntimeError("configuration failed")
    return handle


def quat_mul(a, b):
    return np.array([a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
        a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
        a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
        a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]])


GUIDE_ROLLERS = "legacy"
CORRECT_LINK_COLLISIONS = True


def make_model(profile, rotation, initial_state=None, hip_turns=0):
    terrain = "course" if initial_state is not None else "flat"
    model = load_model(SOURCE / "mjcf/scene.xml", gas_force_n=450.0,
                       correct_link_collisions=CORRECT_LINK_COLLISIONS,
                       terrain=terrain, noslip_iterations=10,
                       guide_rollers=GUIDE_ROLLERS)
    model.opt.timestep = .001
    hips = np.array([model.actuator(name).id for name in
        ("ljoint1_actuator", "ljoint4_actuator", "rjoint1_actuator", "rjoint4_actuator")])
    joints = model.actuator_trnid[hips, 0]
    model.actuator_ctrlrange[hips] = (-45.0, 45.0)
    model.actuator_forcerange[hips] = (-45.0, 45.0)
    model.jnt_actfrcrange[joints] = (-45.0, 45.0)
    data = mujoco.MjData(model)
    data.qpos[:len(profile["qpos"])] = profile["qpos"]
    if initial_state is not None:
        data.qpos[:] = initial_state["qpos"]
        data.qvel[:] = initial_state["qvel"]
        data.time = initial_state["sim_time_s"]
    elif rotation is not None:
        axis, degrees = rotation
        angle = math.radians(degrees)
        vector = dict(roll=(1, 0, 0), pitch=(0, 1, 0))[axis]
        rotate = np.array([math.cos(angle/2), *(math.sin(angle/2)*np.array(vector))])
        data.qpos[3:7] = quat_mul(rotate, data.qpos[3:7])
        data.qpos[2] = .65
        for name in ("ljoint1", "ljoint4", "rjoint1", "rjoint4"):
            data.qpos[model.joint(name).qposadr[0]] += hip_turns * 2.0 * math.pi
    mujoco.mj_forward(model, data)
    return model, data


def settle(model, data, seconds):
    data.ctrl[:] = 0.0
    for _ in range(round(seconds / model.opt.timestep)):
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)


def run_guard_check(lib, profile, name, sensor_index, value, expected_phase):
    model, data = make_model(profile, ("roll", 90))
    names = ["imu_quat", "imu_gyro", "imu_accel"] + [
        f"{motor}_{kind}" for motor in MOTORS for kind in ("pos", "vel", "tor")
    ]
    indices = np.concatenate([np.arange(model.sensor(item).adr[0],
        model.sensor(item).adr[0] + model.sensor(item).dim[0]) for item in names])
    sensor = (ctypes.c_float * 28)(*data.sensordata[indices])
    command = (ctypes.c_float * 3)()
    control = (ctypes.c_float * 40)()
    handle = create_handle(lib, profile, True)
    first_status = 0
    for _ in range(335):
        first_status = lib.OwnWbrCommandStep(handle, sensor, command, control)
    active_phase_before_injection = int(control[34])
    origin = float(sensor[sensor_index])
    sensor[sensor_index] = origin + value if name.startswith("travel") else value
    second_status = lib.OwnWbrCommandStep(handle, sensor, command, control)
    result = dict(name=name, sensor_index=sensor_index, injected_value=float(sensor[sensor_index]),
        expected_phase=expected_phase, actual_phase=int(control[34]),
        active_phase_before_injection=active_phase_before_injection,
        first_status=first_status, second_status=second_status,
        six_outputs_zero=all(abs(float(control[index])) <= 1e-7 for index in range(6)))
    result["passed"] = (active_phase_before_injection in (12, 13, 14, 15) and
        first_status == 0 and second_status == 0 and
        result["actual_phase"] == expected_phase and result["six_outputs_zero"])
    lib.OwnWbrDestroy(handle)
    return result


def run_settle_check(lib, profile, name, transient):
    model, data = make_model(profile, ("roll", 90))
    names = ["imu_quat", "imu_gyro", "imu_accel"] + [
        f"{motor}_{kind}" for motor in MOTORS for kind in ("pos", "vel", "tor")
    ]
    indices = np.concatenate([np.arange(model.sensor(item).adr[0],
        model.sensor(item).adr[0] + model.sensor(item).dim[0]) for item in names])
    sensor = (ctypes.c_float * 28)(*data.sensordata[indices])
    command = (ctypes.c_float * 3)()
    control = (ctypes.c_float * 40)()
    handle = create_handle(lib, profile, True)
    wait_zero = True
    sensor[17] = 5.0
    high_steps = 100 if transient else 501
    for _ in range(high_steps):
        lib.OwnWbrCommandStep(handle, sensor, command, control)
        wait_zero &= all(abs(float(control[index])) <= 1e-7 for index in range(6))
    if transient:
        sensor[17] = 0.0
        for _ in range(235):
            lib.OwnWbrCommandStep(handle, sensor, command, control)
        passed = int(control[34]) in (12, 13) and wait_zero
    else:
        passed = int(control[34]) == -18 and wait_zero and all(
            abs(float(control[index])) <= 1e-7 for index in range(6))
    result = dict(name=name, transient=transient, high_speed_radps=5.0,
        final_phase=int(control[34]), wait_outputs_zero=bool(wait_zero), passed=bool(passed))
    lib.OwnWbrDestroy(handle)
    return result


def run_peak_budget_check(lib, profile):
    model, data = make_model(profile, ("roll", 90))
    names = ["imu_quat", "imu_gyro", "imu_accel"] + [
        f"{motor}_{kind}" for motor in MOTORS for kind in ("pos", "vel", "tor")
    ]
    indices = np.concatenate([np.arange(model.sensor(item).adr[0],
        model.sensor(item).adr[0] + model.sensor(item).dim[0]) for item in names])
    sensor = (ctypes.c_float * 28)(*data.sensordata[indices])
    command = (ctypes.c_float * 3)()
    control = (ctypes.c_float * 40)()
    handle = create_handle(lib, profile, True)
    for _ in range(335):
        lib.OwnWbrCommandStep(handle, sensor, command, control)
    active_phase_before_check = int(control[34])
    samples = []
    for _ in range(500):
        lib.OwnWbrCommandStep(handle, sensor, command, control)
        samples.append(max(abs(float(control[index])) for index in (0, 1, 3, 4)))
    result = dict(name="peak_time_budget", active_phase_before_check=active_phase_before_check,
        peak_nm=max(samples), tail_peak_nm=max(samples[-50:]), final_phase=int(control[34]))
    result["passed"] = (active_phase_before_check in (12, 13) and
        result["peak_nm"] > 10.5 and result["peak_nm"] <= 12.0001 and
        result["tail_peak_nm"] <= 10.0001 and result["final_phase"] in (12, 13))
    lib.OwnWbrDestroy(handle)
    return result


def run_trial(lib, profile, name, rotation, output, initial_state=None, hip_turns=0,
              max_retries=0, force_first_failure=False):
    model, data = make_model(profile, rotation, initial_state, hip_turns)
    settle_before_recovery = not name.endswith("_immediate")
    if rotation is not None and initial_state is None and settle_before_recovery:
        settle(model, data, 3.0)
    names = ["imu_quat", "imu_gyro", "imu_accel"] + [
        f"{motor}_{kind}" for motor in MOTORS for kind in ("pos", "vel", "tor")]
    indices = np.concatenate([np.arange(model.sensor(item).adr[0],
        model.sensor(item).adr[0] + model.sensor(item).dim[0]) for item in names])
    sensor = (ctypes.c_float * 28)()
    command = (ctypes.c_float * 3)()
    control = (ctypes.c_float * 40)()
    if name == "crouch":
        prepare = create_handle(lib, profile, False, 0.18)
        for tick in range(5000):
            if tick % 3 == 0:
                sensor[:] = data.sensordata[indices]
                if lib.OwnWbrCommandStep(prepare, sensor, command, control):
                    raise RuntimeError("crouch preparation failed")
                data.ctrl[:] = control[:6]
            mujoco.mj_step(model, data)
        lib.OwnWbrDestroy(prepare)
    recovery_start_time = float(data.time)
    recovery_start_qpos = data.qpos.copy()
    recovery_start_qvel = data.qvel.copy()
    handle = create_handle(lib, profile, True, 0.18 if name == "crouch" else 0.20)
    recovery_request_preserved_state = (
        np.array_equal(data.qpos, recovery_start_qpos) and
        np.array_equal(data.qvel, recovery_start_qvel) and
        float(data.time) == recovery_start_time)
    if force_first_failure:
        if not lib.OwnWbrSetJointLimits(handle, 10.0, 10.0):
            raise RuntimeError("failed to select bounded first attempt")
    grounds = ground_geom_ids(model)
    wheels = {model.body("lwlink").id, model.body("rwlink").id}
    hip_actuators = np.array([model.actuator(name).id for name in
        ("ljoint1_actuator", "ljoint4_actuator", "rjoint1_actuator", "rjoint4_actuator")])
    hip_joints = model.actuator_trnid[hip_actuators, 0]
    hip_dofs = model.jnt_dofadr[hip_joints]
    peak_time_s = np.zeros(4)
    mechanical_work_j = np.zeros(4)
    peak_hip_speed = np.zeros(4)
    active_peak_hip_speed = np.zeros(4)
    rows = []
    trajectory_time = []
    trajectory_qpos = []
    trajectory_qvel = []
    trajectory_ctrl = []
    trajectory_phase = []
    command_samples = []
    error = None
    stable_since = None
    retry_count = 0
    for tick in range(60001):
        if tick % 3 == 0:
            sensor[:] = data.sensordata[indices]
            status = lib.OwnWbrCommandStep(handle, sensor, command, control)
            if status:
                error = f"core_error_{status}"
                break
            data.ctrl[:] = control[:6]
            command_samples.append((float(data.time), int(control[34]),
                max(abs(float(control[2])), abs(float(control[5])))))
        hip_torque = np.asarray(data.ctrl[hip_actuators], dtype=float)
        hip_speed = np.asarray(data.qvel[hip_dofs], dtype=float)
        peak_time_s += model.opt.timestep * (np.abs(hip_torque) > 10.0001)
        mechanical_work_j += model.opt.timestep * np.abs(hip_torque * hip_speed)
        peak_hip_speed = np.maximum(peak_hip_speed, np.abs(hip_speed))
        if 12 <= int(control[34]) <= 15:
            active_peak_hip_speed = np.maximum(active_peak_hip_speed, np.abs(hip_speed))
        if tick % 10 == 0:
            support = set()
            nonwheel = 0
            for contact in data.contact:
                if (contact.geom1 in grounds) == (contact.geom2 in grounds):
                    continue
                other = contact.geom2 if contact.geom1 in grounds else contact.geom1
                body = int(model.geom_bodyid[other])
                if body in wheels:
                    support.add(body)
                else:
                    nonwheel += 1
            row = dict(time_s=float(data.time), roll_deg=math.degrees(float(control[6])),
                pitch_deg=math.degrees(float(control[7])), phase=int(control[34]),
                wheel_support_count=len(support), nonwheel_contacts=nonwheel,
                max_hip_nm=max(abs(float(control[i])) for i in (0, 1, 3, 4)),
                max_wheel_nm=max(abs(float(control[2])), abs(float(control[5]))),
                max_abs_hip_position_rad=max(abs(float(sensor[i])) for i in (10, 13, 19, 22)),
                hip_q1=float(sensor[10]), hip_q4=float(sensor[13]),
                hip_r1=float(sensor[19]), hip_r4=float(sensor[22]),
                hip_v1=float(sensor[11]), hip_v4=float(sensor[14]),
                hip_rv1=float(sensor[20]), hip_rv4=float(sensor[23]),
                left_length=float(control[12]), left_d_length=float(control[13]),
                left_theta=float(control[14]), right_theta=float(control[20]),
                right_length=float(control[18]), right_d_length=float(control[19]),
                base_z=float(data.qpos[2]),
                base_linear_speed=float(np.linalg.norm(data.qvel[:3])),
                base_angular_speed=float(np.linalg.norm(data.qvel[3:6])),
                qvel_norm=float(np.linalg.norm(data.qvel)))
            rows.append(row)
            trajectory_time.append(float(data.time))
            trajectory_qpos.append(data.qpos.copy())
            trajectory_qvel.append(data.qvel.copy())
            trajectory_ctrl.append(data.ctrl.copy())
            trajectory_phase.append(row["phase"])
            stable = (row["phase"] == 0 and abs(row["roll_deg"]) < 5 and
                      abs(row["pitch_deg"]) < 5 and row["wheel_support_count"] == 2 and
                      row["nonwheel_contacts"] == 0 and
                      row["base_linear_speed"] < .10 and
                      row["base_angular_speed"] < .35 and
                      0.16 < float(control[12]) < 0.25 and
                      0.16 < float(control[18]) < 0.25)
            stable_since = row["time_s"] if stable and stable_since is None else (
                stable_since if stable else None)
            if stable_since is not None and row["time_s"] - stable_since >= 5.0:
                break
            if row["phase"] < 0:
                if (retry_count < max_retries and
                        lib.OwnWbrSetJointLimits(handle, 10.0, 45.0) and
                        lib.OwnWbrRequestRecovery(handle)):
                    retry_count += 1
                    stable_since = None
                else:
                    error = f"recovery_phase_{row['phase']}"
                    break
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all() or any(
                warning.number for warning in data.warning):
            error = "mujoco_warning_or_nonfinite"
            break
    lib.OwnWbrDestroy(handle)
    passed = error is None and stable_since is not None and rows[-1]["time_s"] - stable_since >= 5.0
    failure_phase = int(rows[-1]["phase"]) if rows else 0
    failure_zero_output = failure_phase < 0 and all(
        abs(float(trajectory_ctrl[-1][index])) <= 1e-7 for index in range(6))
    bounded_failure = (error in {f"recovery_phase_{phase}" for phase in range(-20, -14)} and
        failure_zero_output and max(row["max_hip_nm"] for row in rows) <= 12.0001 and
        max(row["max_wheel_nm"] for row in rows) <= 0.6001)
    position = np.asarray(trajectory_qpos)[:, :3]
    velocity = np.asarray(trajectory_qvel)[:, :3]
    active = np.asarray([12 <= int(row["phase"]) <= 15 for row in rows])
    active_indices = np.flatnonzero(active)
    if len(active_indices):
        active_position = position[active_indices]
        active_velocity = velocity[active_indices]
        active_peak_speed = float(np.linalg.norm(active_velocity, axis=1).max())
        active_peak_horizontal_speed = float(np.linalg.norm(active_velocity[:, :2], axis=1).max())
        active_net_horizontal_displacement = float(np.linalg.norm(
            active_position[-1, :2] - active_position[0, :2]))
        active_max_horizontal_displacement = float(np.linalg.norm(
            active_position[:, :2] - active_position[0, :2], axis=1).max())
    else:
        active_peak_speed = 0.0
        active_peak_horizontal_speed = 0.0
        active_net_horizontal_displacement = 0.0
        active_max_horizontal_displacement = 0.0
    completion_frame_wheel_nm = None
    release_window_peak_wheel_nm = None
    saw_phase_15 = False
    for index, (sample_time, phase, wheel_nm) in enumerate(command_samples):
        saw_phase_15 |= phase == 15
        if saw_phase_15 and phase == 0:
            completion_frame_wheel_nm = wheel_nm
            release_window_peak_wheel_nm = max(sample[2] for sample in command_samples[index:]
                if sample[0] <= sample_time + 0.20)
            break
    completion_frame_limit_passed = (completion_frame_wheel_nm is None or
        completion_frame_wheel_nm <= 0.6001)
    roller_metadata = guide_roller_info(model)
    collision_metadata = link_collision_info(model)
    result = dict(name=name, rotation=rotation, passed=passed,
        expected_outcome="recover_and_hold_or_bounded_zero_output",
        expected_outcome_passed=(passed and completion_frame_limit_passed) or bounded_failure,
        error=error,
        model_nq=model.nq, model_nv=model.nv,
        model_data_sha256=model_data_sha256(model),
        terrain="course" if initial_state is not None else "flat",
        noslip_iterations=10,
        recovery_retry_count=retry_count,
        recovery_request_preserved_qpos_qvel_time=recovery_request_preserved_state,
        bounded_failure=bounded_failure,
        failure_zero_output=failure_zero_output,
        guide_roller_count=roller_metadata["roller_count"],
        guide_roller_metadata=roller_metadata,
        link_collision_metadata=collision_metadata,
        initial_recording_seq=(initial_state.get("source_seq") if initial_state else None),
        initial_qpos_sha256=(float_array_sha256(initial_state["qpos"])
                             if initial_state else None),
        initial_qvel_sha256=(float_array_sha256(initial_state["qvel"])
                             if initial_state else None),
        recovery_request_timing=("immediate_after_pose_initialization" if
            rotation is not None and initial_state is None and not settle_before_recovery else
            "after_3s_passive_settle" if rotation is not None and initial_state is None else
            "immediate_from_preserved_recording_state" if initial_state is not None else
            "immediate_from_profile_state"),
        initial_time_s=recovery_start_time,
        duration_s=rows[-1]["time_s"] - recovery_start_time,
        initial_roll_deg=rows[0]["roll_deg"], initial_pitch_deg=rows[0]["pitch_deg"],
        final_roll_deg=rows[-1]["roll_deg"], final_pitch_deg=rows[-1]["pitch_deg"],
        phases=sorted(set(row["phase"] for row in rows)), stable_since_s=stable_since,
        peak_abs_roll_deg=max(abs(row["roll_deg"]) for row in rows),
        peak_abs_pitch_deg=max(abs(row["pitch_deg"]) for row in rows),
        peak_hip_nm=max(row["max_hip_nm"] for row in rows),
        peak_wheel_nm=max(row["max_wheel_nm"] for row in rows),
        peak_abs_hip_position_rad=max(row["max_abs_hip_position_rad"] for row in rows),
        per_hip_over_10_nm_s=peak_time_s.tolist(),
        per_hip_abs_mechanical_work_j=mechanical_work_j.tolist(),
        per_hip_peak_speed_radps=peak_hip_speed.tolist(),
        active_per_hip_peak_speed_radps=active_peak_hip_speed.tolist(),
        active_peak_base_speed_mps=active_peak_speed,
        active_peak_horizontal_speed_mps=active_peak_horizontal_speed,
        active_net_horizontal_displacement_m=active_net_horizontal_displacement,
        active_max_horizontal_displacement_m=active_max_horizontal_displacement,
        completion_frame_wheel_nm=completion_frame_wheel_nm,
        release_window_peak_wheel_nm=release_window_peak_wheel_nm,
        completion_frame_limit_passed=completion_frame_limit_passed,
        nonwheel_contact_samples=sum(row["nonwheel_contacts"] > 0 for row in rows))
    trial = output / name
    trial.mkdir(parents=True, exist_ok=True)
    with (trial / "trace.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    np.savez_compressed(trial / "trajectory.npz",
                        time=np.asarray(trajectory_time),
                        qpos=np.asarray(trajectory_qpos),
                        qvel=np.asarray(trajectory_qvel),
                        ctrl=np.asarray(trajectory_ctrl),
                        phase=np.asarray(trajectory_phase))
    (trial / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--guide-rollers", choices=("legacy", "fixed", "passive"),
                        default="passive")
    parser.add_argument("--legacy-link-collision", action="store_true")
    parser.add_argument("--roller-end-x", type=float)
    parser.add_argument("--case", action="append", dest="cases")
    args = parser.parse_args()
    global GUIDE_ROLLERS, CORRECT_LINK_COLLISIONS
    GUIDE_ROLLERS = args.guide_rollers
    CORRECT_LINK_COLLISIONS = not args.legacy_link_collision
    if args.roller_end_x is not None:
        WbrGuideRollers.ROLLER_X = tuple(float(x) for x in
                                           np.linspace(-args.roller_end_x, args.roller_end_x,
                                                       WbrGuideRollers.ROLLER_ROWS))
        WbrGuideRollers.ROLLER_CENTERS = tuple(
            (float(x), float(y), 0.0) for x in WbrGuideRollers.ROLLER_X
            for y in (-.11, .11))
    args.output.mkdir(parents=True, exist_ok=True)
    profile = json.loads((REFERENCE / "OwnWbrProfile.json").read_text(encoding="utf-8"))
    lib, build = load_core(args.source.resolve(), args.output.resolve())
    recording = ROOT / "local/cache/wbr-rollers-recovery-20261004/live-final/recordings/wbr-20261003-175654-6d66ee28.jsonl"
    recorded = {}
    with recording.open(encoding="utf-8") as stream:
        for line in stream:
            item = json.loads(line)
            if item.get("seq") in (5926, 6253):
                recorded[item["seq"]] = dict(item["frame"], sim_time_s=item["sim_time_s"],
                                               source_seq=item["seq"])
    trials = [
        ("standing", None, None, 0, 0),
        ("crouch", None, None, 0, 0),
        ("side_left", ("roll", 90), None, 0, 0),
        ("side_right", ("roll", -90), None, 0, 0),
        ("back", ("pitch", 90), None, 0, 0),
        ("front", ("pitch", -90), None, 0, 0),
        ("upside_down", ("roll", 180), None, 0, 0),
        ("upside_down_multiturn", ("roll", 180), None, 3, 0),
        ("upside_down_immediate", ("roll", 180), None, 0, 0),
        ("recorded_5926", None, recorded[5926], 0, 0),
        ("recorded_6253", None, recorded[6253], 0, 0),
    ]
    trials = [trial if len(trial) == 6 else (*trial, False) for trial in trials]
    if args.cases:
        trials = [trial for trial in trials if trial[0] in set(args.cases)]
    results = [run_trial(lib, profile, name, rotation, args.output, state, turns,
                         retries, force_failure)
               for name, rotation, state, turns, retries, force_failure in trials]
    guard_checks = [
        run_guard_check(lib, profile, "joint_speed_positive", 11, 2.51, -15),
        run_guard_check(lib, profile, "joint_speed_negative", 11, -2.51, -15),
        run_guard_check(lib, profile, "travel_positive", 10, 2.46, -16),
        run_guard_check(lib, profile, "travel_negative", 10, -2.46, -16),
        run_guard_check(lib, profile, "wheel_speed_positive", 17, 6.01, -17),
        run_guard_check(lib, profile, "wheel_speed_negative", 17, -6.01, -17),
        run_guard_check(lib, profile, "body_rate_positive", 4, 1.51, -20),
        run_guard_check(lib, profile, "body_rate_negative", 4, -1.51, -20),
        run_settle_check(lib, profile, "settle_transient_then_start", True),
        run_settle_check(lib, profile, "settle_timeout_zero", False),
        run_peak_budget_check(lib, profile),
    ]
    (args.output / "guard-checks.json").write_text(
        json.dumps(guard_checks, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    report = dict(controller_source=str(args.source.resolve()), controller_sha256=sha256(args.source),
                  guide_rollers=args.guide_rollers,
                  correct_link_collisions=CORRECT_LINK_COLLISIONS,
                  requested_roller_end_x_m=args.roller_end_x,
                  recording_source=str(recording.resolve()), recording_sha256=sha256(recording),
                  recorded_initial_states={str(seq): {
                      "sim_time_s": state["sim_time_s"],
                      "qpos_sha256": float_array_sha256(state["qpos"]),
                      "qvel_sha256": float_array_sha256(state["qvel"]),
                  } for seq, state in recorded.items()},
                  build_command=build,
                  acceptance_passed=all(item["expected_outcome_passed"] for item in results) and
                      all(item["recovery_request_preserved_qpos_qvel_time"] for item in results) and
                      all(item["recovery_retry_count"] == 0 for item in results) and
                      all(item["passed"] for item in guard_checks),
                  all_poses_recovered=bool(results) and all(item["passed"] for item in results),
                  recovered_count=sum(item["passed"] for item in results),
                  tested_pose_count=len(results),
                  guard_checks=guard_checks, trials=results)
    (args.output / "summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
