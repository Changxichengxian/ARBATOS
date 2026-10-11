"""从真实 home 起身，以自动触台模式验收 100 mm 或 200 mm 平台。"""
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

from RunWbr import MOTORS, ROOT, SOURCE, inspect_state
from WbrGasSpring import load_model
from WbrGuideRollers import MODES, guide_roller_info
from WbrKneeRange import knee_range_info
from WbrTerrain import LARGE_STEPS, STANDARD_CLIMB_PLATFORMS, ground_geom_ids, ground_height


def sha256(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest().upper()


def load_core(output: Path, source: Path):
    sys.path.insert(0, str(ROOT / "tools/mujoco/wheelleg"))
    from RunWheelLeg import find_compiler
    compiler = find_compiler()
    if compiler is None:
        raise RuntimeError("缺少主机 C 编译器")
    executable, tags = compiler
    library = output / f"impact_climb_{os.getpid()}{'.dll' if os.name == 'nt' else '.so'}"
    includes = [ROOT / path for path in ("shared/application/wheelleg",
        "shared/application/robot", "shared/components/support")]
    if "msvc" in tags:
        command = [executable, "/nologo", "/LD", "/O2"] + [f"/I{path}" for path in includes]
        command += [str(source), f"/Fe{library}"]
    else:
        command = [executable, "cc"] if "zig" in tags else [executable]
        command += ["-shared", "-O2", "-std=c99"] + [f"-I{path}" for path in includes]
        command += [str(source), "-o", str(library)]
        if os.name != "nt":
            command += ["-fPIC", "-lm"]
    subprocess.run(command, cwd=output, check=True)
    lib = ctypes.CDLL(str(library))
    pointer = ctypes.POINTER(ctypes.c_float)
    lib.OwnWbrCreate.argtypes = [pointer]
    lib.OwnWbrCreate.restype = ctypes.c_void_p
    lib.OwnWbrDestroy.argtypes = [ctypes.c_void_p]
    lib.OwnWbrDestroy.restype = None
    return lib, command


def joint_limit_names(model: mujoco.MjModel, data: mujoco.MjData) -> list[str]:
    names = []
    for index in np.flatnonzero(data.efc_type == mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT):
        joint_id = int(data.efc_id[index])
        names.append(model.joint(joint_id).name or f"joint_{joint_id}")
    return names


def geom_world_aabb(model: mujoco.MjModel, data: mujoco.MjData,
                    geom_id: int) -> tuple[np.ndarray, np.ndarray]:
    """返回实际编译几何在世界坐标中的轴对齐包围盒。"""
    rotation = data.geom_xmat[geom_id].reshape(3, 3)
    if model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_MESH:
        mesh_id = int(model.geom_dataid[geom_id])
        start = int(model.mesh_vertadr[mesh_id])
        count = int(model.mesh_vertnum[mesh_id])
        vertices = model.mesh_vert[start:start+count]
        world = data.geom_xpos[geom_id] + vertices @ rotation.T
        return world.min(axis=0), world.max(axis=0)
    center = data.geom_xpos[geom_id] + rotation @ model.geom_aabb[geom_id, :3]
    half_size = np.abs(rotation) @ model.geom_aabb[geom_id, 3:]
    return center-half_size, center+half_size


def split_nonwheel_contacts(contacts: list[tuple[str, str, str]], phase: int,
                            chassis_bodies: set[str],
                            target_contact_names: set[str]
                            ) -> tuple[list[tuple[str, str, str]],
                                       list[tuple[str, str, str]]]:
    allowed = [item for item in contacts if
               1 <= phase <= 6 and item[0] in chassis_bodies and
               item[2] in target_contact_names]
    violations = [item for item in contacts if item not in allowed]
    return allowed, violations


def contact_classification_self_check() -> None:
    chassis = {"base_link", "guide_roller_6"}
    target_family = {"terrain_rmul_1627", "terrain_rmul_0014"}
    cases = (
        (("guide_roller_6", "guide_roller_geom_6", "terrain_rmul_0014"), 2, True),
        (("guide_roller_6", "guide_roller_geom_6", "terrain_rmul_1627"), 4, True),
        (("guide_roller_6", "guide_roller_geom_6", "terrain_rmul_0015"), 2, False),
        (("llink2", "llink2_collision", "terrain_rmul_0014"), 2, False),
        (("guide_roller_6", "guide_roller_geom_6", "terrain_rmul_0014"), 7, False),
    )
    for contact, phase, should_allow in cases:
        allowed, violations = split_nonwheel_contacts(
            [contact], phase, chassis, target_family)
        if bool(allowed) != should_allow or bool(violations) == should_allow:
            raise RuntimeError(f"接触分类自检失败：phase={phase}, contact={contact}")


def main() -> int:
    test_hash = sha256(Path(__file__))
    contact_classification_self_check()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path,
                        default=Path(__file__).with_name("OwnWbrBridge.c"))
    parser.add_argument("--speed", type=float, default=2.0,
                        help="W 对应的前进速度，m/s")
    parser.add_argument("--start-x", type=float,
                        help="home 起点 x 坐标，m")
    parser.add_argument("--platform", choices=("100", "200", "rmul200"), default="100",
                        help="100 原窄台，200 独立宽台，rmul200 使用 CAD 场地入口的高台")
    parser.add_argument("--guide-rollers", choices=MODES, default="legacy",
                        help="底盘导轮模型；passive 与当前网页一致，legacy 保留旧实验")
    parser.add_argument("--seconds", type=float, default=22.0,
                        help="总仿真时间，s")
    parser.add_argument("--knee-range", choices=("upstream", "cad120"), default="upstream",
                        help="上游膝范围，或经网格抽样核对的 CAD 120 mm 收腿参考")
    parser.add_argument("--climb-assist", action="store_true",
                        help="站立就绪后按住 G，验证上台准备")
    parser.add_argument("--assist-delay", type=float, default=0.0,
                        help="站立就绪后延迟多少秒按 G")
    parser.add_argument("--input-delay", type=float, default=0.0,
                        help="高台站立就绪后延迟多少秒发送 W/G，复现网页输入时序")
    parser.add_argument("--assist-release-after", type=float,
                        help="按 G 后多少秒松开，用于验证准备撤销和动作锁存")
    parser.add_argument("--release-after", type=float,
                        help="阶段7开始后持续驻车多少秒再松开 W，0.75 s 后再次按 W")
    parser.add_argument("--hold-after-release", action="store_true",
                        help="松开 W 后不再前进，严格验收整个剩余驻车段且至少 5 s")
    parser.add_argument("--disable-drive-delay", type=float,
                        help="松开 W 后多少秒切到 drive_mode 0，模拟输入超时；需长期驻车")
    parser.add_argument("--jump-delay", type=float,
                        help="松开 W 后多少秒按空格 0.15 s；需实际起跳并在恢复后连续台顶稳定 5 s")
    parser.add_argument("--output", type=Path,
                        default=ROOT / "local/cache/wbr-impact-climb-20261003/formal")
    args = parser.parse_args()
    high_platform = args.platform != "100"
    start_y = {"100": -1.5, "200": -17.0, "rmul200": 0.0}[args.platform]
    if args.start_x is None:
        args.start_x = {"100": 6.0, "200": 6.5, "rmul200": 37.0}[args.platform]
    args.source = args.source.resolve()
    args.output = args.output.resolve()
    if not args.source.is_file():
        parser.error(f"控制器源文件不存在：{args.source}")
    if not math.isfinite(args.speed) or not .1 <= args.speed <= 3.0:
        parser.error("速度必须是 0.1–3.0 m/s 的有限数")
    max_start_x = 37.5 if args.platform == "rmul200" else 6.6
    if not math.isfinite(args.start_x) or not -10.0 <= args.start_x <= max_start_x:
        parser.error(f"起点 x 必须是 −10–{max_start_x} m 的有限数")
    if not math.isfinite(args.seconds) or args.seconds < 14.0:
        parser.error("总仿真时间至少 14 s")
    if args.release_after is not None and (not math.isfinite(args.release_after) or
            args.release_after < 5.0):
        parser.error("松开 W 前必须给台上稳定验收至少留 5 s")
    if args.hold_after_release and args.release_after is None:
        parser.error("长期驻车必须指定 --release-after")
    for name in ("disable_drive_delay", "jump_delay"):
        value = getattr(args, name)
        if value is not None and (not math.isfinite(value) or value < 0.0):
            parser.error(f"{name} 必须是非负有限秒数")
        if value is not None and not args.hold_after_release:
            parser.error(f"{name} 必须配合 --hold-after-release")
    args.output.mkdir(parents=True, exist_ok=True)

    for name in ("assist_delay", "assist_release_after", "input_delay"):
        value = getattr(args, name)
        if value is not None and (not math.isfinite(value) or value < 0.0):
            parser.error(f"{name} 必须是非负有限秒数")
    if not args.climb_assist and (args.assist_delay or args.assist_release_after is not None):
        parser.error("准备按键时序必须配合 --climb-assist")

    profile_path = Path(__file__).with_name("OwnWbrProfile.json")
    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    leg_length = .32 if high_platform else float(profile["length_m"])
    config = (ctypes.c_float * 20)(*np.asarray(profile["negative_k"]).flatten(),
        profile["theta0"], profile["tp0"], profile["wheel0"], profile["support_n"],
        profile["length_m"], profile["radius_m"], profile["length_kp"], profile["length_kd"])
    source_hash = sha256(args.source)
    lib, build_command = load_core(args.output, args.source)
    if sha256(args.source) != source_hash:
        raise RuntimeError("控制器源文件在编译期间发生变化，请重新验收")
    pointer = ctypes.POINTER(ctypes.c_float)
    for name, argtypes in (
        ("OwnWbrCommandStep", [ctypes.c_void_p, pointer, pointer, pointer]),
        ("OwnWbrSetJointLimits", [ctypes.c_void_p, ctypes.c_float, ctypes.c_float]),
        ("OwnWbrSetLegLength", [ctypes.c_void_p, ctypes.c_float]),
        ("OwnWbrSetGasSpring", [ctypes.c_void_p, ctypes.c_float]),
        ("OwnWbrSetStage", [ctypes.c_void_p, ctypes.c_int]),
        ("OwnWbrSetJumpProfile", [ctypes.c_void_p, ctypes.c_int]),
        ("OwnWbrSetStepJump", [ctypes.c_void_p, ctypes.c_int]),
        ("OwnWbrSetStepClimb", [ctypes.c_void_p, ctypes.c_int]),
        ("OwnWbrGetStepClimbPhase", [ctypes.c_void_p]),
        ("OwnWbrSetDriveMode", [ctypes.c_void_p, ctypes.c_int]),
        ("OwnWbrEnableRise", [ctypes.c_void_p, ctypes.c_int])):
        function = getattr(lib, name)
        function.argtypes = argtypes
        function.restype = ctypes.c_int

    handle = lib.OwnWbrCreate(config)
    if not handle:
        raise RuntimeError("控制器初始化失败")
    if hasattr(lib, "OwnWbrSetClimbFold"):
        lib.OwnWbrSetClimbFold.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.OwnWbrSetClimbFold.restype = ctypes.c_int
        if lib.OwnWbrSetClimbFold(handle, int(args.knee_range == "cad120")) != 1:
            lib.OwnWbrDestroy(handle)
            raise RuntimeError("收腿控制与模型范围配置失败")
    elif args.knee_range == "cad120":
        lib.OwnWbrDestroy(handle)
        raise RuntimeError("CAD 短腿模型需要支持 OwnWbrSetClimbFold 的控制器")
    configured = all((
        lib.OwnWbrSetJointLimits(handle, 10.0, 45.0) == 1,
        lib.OwnWbrSetLegLength(handle, leg_length) == 1,
        lib.OwnWbrSetGasSpring(handle, 450.0) == 1,
        lib.OwnWbrSetStage(handle, 3) == 1,
        lib.OwnWbrSetJumpProfile(handle, 2) == 1,
        lib.OwnWbrSetStepJump(handle, 0) == 1,
        lib.OwnWbrSetDriveMode(handle, 1) == 1,
        lib.OwnWbrEnableRise(handle, 1) == 1,
        lib.OwnWbrSetStepClimb(handle, 2) == 1,
    ))
    if not configured:
        lib.OwnWbrDestroy(handle)
        raise RuntimeError("自动触台模式配置失败；请确认 --source 支持 mode 2")
    model = load_model(SOURCE / "mjcf/scene.xml", gas_force_n=450.0,
                       correct_link_collisions=True, terrain="course",
                       noslip_iterations=10, guide_rollers=args.guide_rollers,
                       knee_range=args.knee_range)
    model.opt.timestep = .001
    hip_names = ("ljoint1_actuator", "ljoint4_actuator", "rjoint1_actuator", "rjoint4_actuator")
    hips = np.array([model.actuator(name).id for name in hip_names])
    joints = model.actuator_trnid[hips, 0]
    model.actuator_ctrlrange[hips] = (-45.0, 45.0)
    model.actuator_forcerange[hips] = (-45.0, 45.0)
    model.jnt_actfrcrange[joints] = (-45.0, 45.0)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    data.qpos[0] = args.start_x
    data.qpos[1] = start_y
    mujoco.mj_forward(model, data)

    names = ["imu_quat", "imu_gyro", "imu_accel"] + [
        f"{motor}_{kind}" for motor in MOTORS for kind in ("pos", "vel", "tor")]
    indices = np.concatenate([np.arange(model.sensor(name).adr[0],
        model.sensor(name).adr[0] + model.sensor(name).dim[0]) for name in names])
    grounds = ground_geom_ids(model)
    if args.platform == "rmul200":
        # 只限定验收区域；沿中心入口的 CAD 台顶到 40.35 m 后还有下一层障碍。
        target_name, target_x0, target_x1 = "terrain_rmul_1627", 38.5, 40.35
        target_y0, target_y1, target_height = -.35, .35, .20
    elif args.platform == "200":
        target_name, target_x0, target_x1, target_y0, target_y1, target_height = STANDARD_CLIMB_PLATFORMS[0]
    else:
        target_name, target_x0, target_x1, target_y, target_height = LARGE_STEPS[0]
        target_y0, target_y1 = target_y - .4, target_y + .4
    target = model.geom(target_name).id
    target_contact_names = {target_name}
    target_boundary_ids: set[int] = set()
    target_contact_family = {
        "top_geom": target_name,
        "transient_boundary_geoms": [],
        "allowed_phase_range": [1, 6],
        "allowed_bodies": "base_link_and_guide_rollers",
        "boundary_world_aabb_m": {},
    }
    if args.platform == "rmul200":
        boundary_name = "terrain_rmul_0014"
        boundary = model.geom(boundary_name).id
        boundary_bounds = geom_world_aabb(model, data, boundary)
        target_bounds = geom_world_aabb(model, data, target)
        expected_boundary = np.array([[38.499, -2.5, 0.0], [38.501, 4.0, 0.2]])
        expected_target = np.array([[38.5, -2.5, 0.198], [40.35, 4.0, 0.2]])
        actual_boundary = np.stack(boundary_bounds)
        actual_target = np.stack(target_bounds)
        if (not np.allclose(actual_boundary, expected_boundary, rtol=0.0, atol=2e-6) or
                not np.allclose(actual_target, expected_target, rtol=0.0, atol=2e-6)):
            raise RuntimeError("RMUL 目标台顶或前立面的世界包围盒与已审查 CAD 几何不符")
        target_contact_names.add(boundary_name)
        target_boundary_ids.add(boundary)
        target_contact_family["transient_boundary_geoms"] = [boundary_name]
        target_contact_family["boundary_world_aabb_m"] = {
            boundary_name: actual_boundary.tolist()}
    wheel_bodies = {model.body("lwlink").id: "lwlink",
                    model.body("rwlink").id: "rwlink"}
    chassis_bodies = {"base_link"} | {
        model.body(index).name for index in range(model.nbody)
        if model.body(index).name.startswith("guide_roller_")}
    sensor = (ctypes.c_float * 28)()
    command = (ctypes.c_float * 3)()
    output = (ctypes.c_float * 40)()
    rows: list[dict] = []
    qpos_times: list[float] = []
    qpos_samples: list[np.ndarray] = []
    home_ready_time = trigger_time = phase7_time = None
    trigger_true_speed = None
    stand_gate_start = stand_ready_time = None
    phase8_time = rearmed_time = None
    release_time = resume_time = None
    drive_disabled_time = jump_time = None
    jump_started_time = jump_finished_time = None
    jump_recovery_s = .75
    hold_ticks = hold_violation_ticks = 0
    hold_first_violation_time = None
    impact_time = preimpact_speed = None
    impact_wheels: list[str] = []
    previous_true_speed = float(data.qvel[0])
    core_error = None
    warning_counts = [0] * len(data.warning)
    post_ready_limit_ticks = 0
    post_ready_limit_names: set[str] = set()
    contact_violation_ticks = 0
    contact_violation_details: set[tuple[str, str, str]] = set()
    target_boundary_contact_ticks = 0
    allowed_target_boundary_contact_ticks = 0
    target_boundary_contact_details: set[tuple[str, str, str]] = set()
    allowed_target_boundary_contact_details: set[tuple[str, str, str]] = set()
    target_boundary_contact_stats: dict[str, dict[str, float | str]] = {}
    stable_start = None
    stable_longest = 0.0
    stable_longest_start = stable_longest_end = None
    climb_stable_start = None
    climb_stable_longest = 0.0
    climb_stable_longest_start = climb_stable_longest_end = None
    target_nonwheel_contact_stats: dict[str, dict[str, float]] = {}
    max_target_penetration_m = max_target_penetration_after_ready_m = 0.0
    hip_ctrl_indices = np.array([0, 1, 3, 4])
    wheel_ctrl_indices = np.array([2, 5])
    hip_peak_torque_nm = np.zeros(4, dtype=np.float64)
    wheel_peak_torque_nm = np.zeros(2, dtype=np.float64)
    hip_over_continuous_s = np.zeros(4, dtype=np.float64)
    climb_hip_over_continuous_s = np.zeros(4, dtype=np.float64)
    climb_hip_peak_speed = np.zeros(4, dtype=np.float64)
    hip_dof_indices = model.jnt_dofadr[joints]
    assist_api = hasattr(lib, "OwnWbrSetClimbAssist")
    if args.climb_assist and not assist_api:
        parser.error("所选控制器没有上台准备接口")
    if assist_api:
        lib.OwnWbrSetClimbAssist.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.OwnWbrSetClimbAssist.restype = ctypes.c_int
    assist_press_time = assist_release_time = None
    try:
        phase = 0
        for tick in range(round(args.seconds / model.opt.timestep) + 1):
            time_s = float(data.time)
            if args.release_after is not None and phase7_time is not None:
                release_time = phase7_time + args.release_after
                resume_time = None if args.hold_after_release else release_time + .75
            drive_ready = (stand_ready_time is not None and
                           time_s >= stand_ready_time + args.input_delay)
            drive = (args.speed if args.platform == "100" or drive_ready
                     else 0.0)
            if release_time is not None and release_time <= time_s and (
                    resume_time is None or time_s < resume_time):
                drive = 0.0
            jump = float(release_time is not None and args.jump_delay is not None and
                release_time + args.jump_delay <= time_s <
                release_time + args.jump_delay + .15)
            if tick % 3 == 0:
                assist_ready_time = stand_ready_time if high_platform else home_ready_time
                assist_held = (args.climb_assist and assist_ready_time is not None and
                    time_s >= assist_ready_time + args.input_delay + args.assist_delay and
                    (args.assist_release_after is None or assist_press_time is None or
                     time_s < assist_press_time + args.assist_release_after))
                if assist_held and assist_press_time is None:
                    assist_press_time = time_s
                if (not assist_held and assist_press_time is not None and
                    assist_release_time is None and args.assist_release_after is not None):
                    assist_release_time = time_s
                if assist_api and lib.OwnWbrSetClimbAssist(handle, int(assist_held)) != 1:
                    core_error = "climb_assist_input_failed"
                    break
                if (drive_disabled_time is None and release_time is not None and
                        args.disable_drive_delay is not None and
                        time_s >= release_time + args.disable_drive_delay):
                    if lib.OwnWbrSetDriveMode(handle, 0) != 1:
                        core_error = "disable_drive_mode_failed"
                        break
                    drive_disabled_time = time_s
                if jump and jump_time is None:
                    jump_time = time_s
                sensor[:] = data.sensordata[indices]
                command[:] = (drive, 0.0, jump)
                status = lib.OwnWbrCommandStep(handle, sensor, command, output)
                if status:
                    core_error = f"core_error_{status}"
                    break
                phase = lib.OwnWbrGetStepClimbPhase(handle)
                if args.jump_delay is not None and jump_time is not None:
                    if int(output[29]) != 0:
                        if jump_started_time is None:
                            jump_started_time = time_s
                        jump_finished_time = None
                    elif jump_started_time is not None and jump_finished_time is None:
                        jump_finished_time = time_s
                if int(output[34]) < 0:
                    core_error = f"rise_failed_{int(output[34])}"
                    break
                home_gate_ready = int(output[34]) == 0
                if high_platform:
                    home_gate_ready = (home_gate_ready and
                        min(float(output[12]), float(output[18])) > .30 and
                        abs(float(output[7])) < .0524 and
                        max(abs(float(value)) for value in sensor[4:7]) < .35)
                if home_gate_ready and home_ready_time is None:
                    home_ready_time = time_s
                if high_platform:
                    if home_gate_ready:
                        stand_gate_start = time_s if stand_gate_start is None else stand_gate_start
                        if time_s - stand_gate_start >= .30 and stand_ready_time is None:
                            stand_ready_time = time_s
                    else:
                        stand_gate_start = None
                if phase >= 1 and trigger_time is None:
                    trigger_time = time_s
                    trigger_true_speed = float(data.qvel[0])
                if phase == 7 and phase7_time is None:
                    phase7_time = time_s
                if phase == 8 and phase8_time is None:
                    phase8_time = time_s
                if phase8_time is not None and phase == 0 and rearmed_time is None:
                    rearmed_time = time_s
                data.ctrl[:] = output[:6]

            top_normals: dict[str, float] = {}
            nonwheel = []
            target_nonwheel_tick: dict[str, tuple[float, float]] = {}
            target_boundary_tick: dict[tuple[str, str, str], tuple[float, float]] = {}
            target_contact_wheels = set()
            for contact_index, contact in enumerate(data.contact):
                if contact.geom1 in grounds:
                    ground, robot = contact.geom1, contact.geom2
                    normal_z = float(contact.frame[2])
                elif contact.geom2 in grounds:
                    ground, robot = contact.geom2, contact.geom1
                    normal_z = -float(contact.frame[2])
                else:
                    continue
                body = int(model.geom_bodyid[robot])
                body_name = model.body(body).name
                ground_name = model.geom(ground).name
                robot_name = model.geom(robot).name
                if ground == target and body not in wheel_bodies:
                    force = np.zeros(6, dtype=np.float64)
                    mujoco.mj_contactForce(model, data, contact_index, force)
                    penetration = max(0.0, -float(contact.dist))
                    max_target_penetration_m = max(max_target_penetration_m, penetration)
                    if home_ready_time is not None:
                        max_target_penetration_after_ready_m = max(
                            max_target_penetration_after_ready_m, penetration)
                    previous = target_nonwheel_tick.get(body_name, (0.0, 0.0))
                    target_nonwheel_tick[body_name] = (
                        max(previous[0], penetration), max(previous[1], abs(float(force[0])))
                    )
                if ground in target_boundary_ids and body not in wheel_bodies:
                    force = np.zeros(6, dtype=np.float64)
                    mujoco.mj_contactForce(model, data, contact_index, force)
                    item = (body_name, robot_name, ground_name)
                    previous = target_boundary_tick.get(item, (0.0, 0.0))
                    target_boundary_tick[item] = (
                        max(previous[0], max(0.0, -float(contact.dist))),
                        max(previous[1], abs(float(force[0]))))
                if body in wheel_bodies:
                    if ground == target:
                        wheel_name = wheel_bodies[body]
                        target_contact_wheels.add(wheel_name)
                        top_normals[wheel_name] = max(top_normals.get(wheel_name, -1.0), normal_z)
                else:
                    nonwheel.append((body_name, robot_name, ground_name))
            for body_name, (penetration, normal_force) in target_nonwheel_tick.items():
                stats = target_nonwheel_contact_stats.setdefault(body_name, {
                    "duration_s": 0.0, "first_time_s": time_s, "last_time_s": time_s,
                    "max_penetration_m": 0.0, "peak_normal_force_n": 0.0,
                })
                stats["duration_s"] += model.opt.timestep
                stats["last_time_s"] = time_s
                stats["max_penetration_m"] = max(stats["max_penetration_m"], penetration)
                stats["peak_normal_force_n"] = max(stats["peak_normal_force_n"], normal_force)
            if home_ready_time is not None:
                hip_torque = np.abs(data.ctrl[hip_ctrl_indices])
                wheel_torque = np.abs(data.ctrl[wheel_ctrl_indices])
                hip_peak_torque_nm = np.maximum(hip_peak_torque_nm, hip_torque)
                wheel_peak_torque_nm = np.maximum(wheel_peak_torque_nm, wheel_torque)
                hip_over_continuous_s += (hip_torque > 10.0) * model.opt.timestep
                if 1 <= phase <= 6:
                    climb_hip_over_continuous_s += (hip_torque > 10.0) * model.opt.timestep
                    climb_hip_peak_speed = np.maximum(
                        climb_hip_peak_speed, np.abs(data.qvel[hip_dof_indices]))
            if impact_time is None and target_contact_wheels:
                impact_time = time_s
                preimpact_speed = previous_true_speed
                impact_wheels = sorted(target_contact_wheels)

            limits = joint_limit_names(model, data)
            if home_ready_time is not None:
                post_ready_limit_ticks += int(bool(limits))
                post_ready_limit_names.update(limits)
                allowed, violations = split_nonwheel_contacts(
                    nonwheel, phase, chassis_bodies, target_contact_names)
                boundary_allowed = [item for item in allowed if item[2] != target_name]
                if target_boundary_tick:
                    target_boundary_contact_ticks += 1
                    target_boundary_contact_details.update(target_boundary_tick)
                if boundary_allowed:
                    allowed_target_boundary_contact_ticks += 1
                    allowed_target_boundary_contact_details.update(boundary_allowed)
                for item, (penetration, normal_force) in target_boundary_tick.items():
                    key = " | ".join(item)
                    stats = target_boundary_contact_stats.setdefault(key, {
                        "body": item[0], "robot_geom": item[1], "terrain_geom": item[2],
                        "duration_s": 0.0, "first_time_s": time_s, "last_time_s": time_s,
                        "max_penetration_m": 0.0, "peak_normal_force_n": 0.0,
                    })
                    stats["duration_s"] += model.opt.timestep
                    stats["last_time_s"] = time_s
                    stats["max_penetration_m"] = max(
                        float(stats["max_penetration_m"]), penetration)
                    stats["peak_normal_force_n"] = max(
                        float(stats["peak_normal_force_n"]), normal_force)
                if violations:
                    contact_violation_ticks += 1
                    contact_violation_details.update(violations)

            wheel_positions = {name: data.xpos[body].copy()
                               for body, name in wheel_bodies.items()}
            inside = {name: target_x0 < float(pos[0]) < target_x1 and
                      target_y0 < float(pos[1]) < target_y1
                      for name, pos in wheel_positions.items()}
            both_top = all(inside.values()) and all(
                top_normals.get(name, -1.0) >= .80 for name in wheel_positions)
            stable_now = (phase in (0, 7, 8) and both_top and not nonwheel and not limits and
                min(float(output[12]), float(output[18])) >= .19 and
                abs(math.degrees(float(output[7]))) < 8.0 and
                abs(math.degrees(float(output[6]))) < 8.0 and
                abs(float(data.qvel[0])) < .12)
            climb_stable_now = (phase in (0, 7, 8) and both_top and not limits and
                min(float(output[12]), float(output[18])) >= .19 and
                abs(math.degrees(float(output[7]))) < 8.0 and
                abs(math.degrees(float(output[6]))) < 8.0 and
                abs(float(data.qvel[0])) < .12 and
                max(abs(float(value)) for value in data.ctrl) <= 45.0)
            if climb_stable_now:
                climb_stable_start = time_s if climb_stable_start is None else climb_stable_start
                climb_duration = time_s - climb_stable_start + model.opt.timestep
                if climb_duration > climb_stable_longest:
                    climb_stable_longest = climb_duration
                    climb_stable_longest_start = climb_stable_start
                    climb_stable_longest_end = time_s
            else:
                climb_stable_start = None
            if stable_now:
                stable_start = time_s if stable_start is None else stable_start
                stable_duration = time_s - stable_start + model.opt.timestep
                if stable_duration > stable_longest:
                    stable_longest = stable_duration
                    stable_longest_start = stable_start
                    stable_longest_end = time_s
            else:
                stable_start = None
            jump_recovering = (args.jump_delay is not None and jump_started_time is not None and
                (int(output[29]) != 0 or jump_finished_time is None or
                 time_s < jump_finished_time + jump_recovery_s))
            if (args.hold_after_release and release_time is not None and
                    time_s >= release_time and not jump_recovering):
                hold_ticks += 1
                if not stable_now:
                    hold_violation_ticks += 1
                    if hold_first_violation_time is None:
                        hold_first_violation_time = time_s

            if tick % 5 == 0:
                heights = {name: ground_height(model, float(pos[0]), float(pos[1]))
                           for name, pos in wheel_positions.items()}
                row = inspect_state(model, data, output,
                    np.r_[0.0, drive, 0.0, 0.0, 0.0, 0.0, 0.0])
                row.update(phase=phase, rise_phase=int(output[34]),
                    left_wheel_x_m=float(wheel_positions["lwlink"][0]),
                    left_wheel_y_m=float(wheel_positions["lwlink"][1]),
                    left_wheel_z_m=float(wheel_positions["lwlink"][2]),
                    right_wheel_x_m=float(wheel_positions["rwlink"][0]),
                    right_wheel_y_m=float(wheel_positions["rwlink"][1]),
                    right_wheel_z_m=float(wheel_positions["rwlink"][2]),
                    left_ground_z_m=heights["lwlink"], right_ground_z_m=heights["rwlink"],
                    left_target_normal_z=top_normals.get("lwlink", math.nan),
                    right_target_normal_z=top_normals.get("rwlink", math.nan),
                    left_center_inside_platform=inside["lwlink"],
                    right_center_inside_platform=inside["rwlink"],
                    both_wheels_top_supported=both_top,
                    nonwheel_ground_contacts=len(nonwheel),
                    nonwheel_contact_detail=repr(nonwheel),
                    joint_limit_names=repr(limits), stable_now=stable_now,
                    command_w_pressed=drive > .0, command_jump_pressed=bool(jump),
                    jump_phase=int(output[29]), jump_recovering=jump_recovering,
                    drive_mode=0 if drive_disabled_time is not None else 1)
                rows.append(row)
                qpos_times.append(time_s)
                qpos_samples.append(data.qpos.copy())
            warning_counts = [max(old, int(item.number))
                              for old, item in zip(warning_counts, data.warning)]
            previous_true_speed = float(data.qvel[0])
            if tick == round(args.seconds / model.opt.timestep):
                break
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            if not np.isfinite(data.qpos).all() or any(item.number for item in data.warning):
                core_error = "mujoco_warning_or_nonfinite"
                break
    finally:
        lib.OwnWbrDestroy(handle)

    if not rows:
        raise RuntimeError(core_error or "没有仿真记录")
    with (args.output / "trace.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    trajectory_path = args.output / "qpos-trajectory.npz"
    np.savez_compressed(trajectory_path,
        time_s=np.asarray(qpos_times, dtype=np.float32),
        qpos=np.asarray(qpos_samples, dtype=np.float32))

    warnings = {str(index): number for index, number in enumerate(warning_counts) if number}
    hold_duration = hold_ticks * model.opt.timestep
    terminal_stable_duration = (float(data.time) - stable_start + model.opt.timestep
                                if stable_start is not None else 0.0)
    climb_terminal_stable_duration = (float(data.time) - climb_stable_start + model.opt.timestep
                                      if climb_stable_start is not None else 0.0)
    hold_complete = (not args.hold_after_release or
        (hold_duration >= 5.0 and hold_violation_ticks == 0 and terminal_stable_duration >= 5.0))
    release_sequence_complete = args.release_after is None or (
        phase8_time is not None and rearmed_time is not None and
        ((args.hold_after_release and hold_complete) or
         (resume_time is not None and rows[-1]["time_s"] >= resume_time + .25)))
    optional_inputs_complete = (
        (args.disable_drive_delay is None or
         (drive_disabled_time is not None and rows[-1]["time_s"] >= drive_disabled_time + 5.0)) and
        (args.jump_delay is None or
         (jump_started_time is not None and jump_finished_time is not None and
          rows[-1]["time_s"] >= jump_finished_time + jump_recovery_s + 5.0 and
          terminal_stable_duration >= 5.0)))
    motor_budget_ok = bool(np.all(hip_peak_torque_nm <= 45.0 + 1e-6) and
                           np.all(wheel_peak_torque_nm <= 5.2 + 1e-6) and
                           np.all(climb_hip_over_continuous_s <= 3.505))
    report = {
        "passed": core_error is None and not warnings and trigger_time is not None and
            phase7_time is not None and impact_time is not None and preimpact_speed is not None and
            stable_longest >= 5.0 and post_ready_limit_ticks == 0 and
            contact_violation_ticks == 0 and release_sequence_complete and optional_inputs_complete,
        "home_ready_time_s": home_ready_time,
        "stand_ready_time_s": stand_ready_time,
        "home_ready_gate": ("rise_complete_and_min_leg_gt_0.30m_pitch_lt_0.0524rad_gyro_axes_lt_0.35rad_s"
            if high_platform else "rise_complete"),
        "stand_ready_gate": ("home_ready_gate_continuously_0.30s_then_enable_W"
            if high_platform else None),
        "impact_time_s": impact_time,
        "impact_contact_scope": "首次车轮接触目标台顶碰撞体；CAD 前立面可能属于其它碰撞体",
        "preimpact_true_speed_mps": preimpact_speed,
        "impact_wheels": impact_wheels,
        "trigger_time_s": trigger_time,
        "trigger_true_speed_mps": trigger_true_speed,
        "climb_assist_press_time_s": assist_press_time,
        "climb_assist_release_time_s": assist_release_time,
        "phase7_time_s": phase7_time,
        "stable_longest_s": stable_longest,
        "climb_completed": (climb_stable_longest >= 5.0 and
            post_ready_limit_ticks == 0 and motor_budget_ok and core_error is None and not warnings),
        "motor_budget_ok": motor_budget_ok,
        "motor_budget": {"hip_peak_abs_torque_nm": hip_peak_torque_nm.tolist(),
            "climb_hip_time_above_10nm_s": climb_hip_over_continuous_s.tolist(),
            "climb_hip_peak_abs_speed_rad_s": climb_hip_peak_speed.tolist(),
            "climb_peak_time_budget_s": 3.5,
            "peak_time_measurement_tolerance_s": .005,
            "hip_time_above_10nm_s": hip_over_continuous_s.tolist(),
            "wheel_peak_abs_torque_nm": wheel_peak_torque_nm.tolist(),
            "hip_peak_limit_nm": 45.0, "wheel_peak_limit_nm": 5.2},
        "climb_stable_longest_s": climb_stable_longest,
        "climb_stable_longest_window_s": [climb_stable_longest_start, climb_stable_longest_end],
        "climb_terminal_stable_duration_s": climb_terminal_stable_duration,
        "stable_longest_window_s": [stable_longest_start, stable_longest_end],
        "release_time_s": release_time,
        "resume_time_s": resume_time,
        "phase8_time_s": phase8_time,
        "rearmed_time_s": rearmed_time,
        "release_sequence_complete": release_sequence_complete,
        "hold_after_release": args.hold_after_release,
        "hold_duration_s": hold_duration,
        "hold_violation_ticks": hold_violation_ticks,
        "hold_first_violation_time_s": hold_first_violation_time,
        "hold_complete": hold_complete,
        "terminal_stable_duration_s": terminal_stable_duration,
        "drive_disabled_time_s": drive_disabled_time,
        "jump_command_time_s": jump_time,
        "jump_started_time_s": jump_started_time,
        "jump_finished_time_s": jump_finished_time,
        "jump_recovery_allowance_s": jump_recovery_s if args.jump_delay is not None else None,
        "optional_inputs_complete": optional_inputs_complete,
        "post_ready_joint_limit_ticks": post_ready_limit_ticks,
        "post_ready_joint_limit_names": sorted(post_ready_limit_names),
        "post_ready_contact_violation_ticks": contact_violation_ticks,
        "post_ready_contact_violation_details": sorted(contact_violation_details),
        "target_contact_family": target_contact_family,
        "post_ready_target_boundary_contact_ticks": target_boundary_contact_ticks,
        "post_ready_target_boundary_contact_details": sorted(target_boundary_contact_details),
        "post_ready_allowed_target_boundary_contact_ticks":
            allowed_target_boundary_contact_ticks,
        "post_ready_allowed_target_boundary_contact_details":
            sorted(allowed_target_boundary_contact_details),
        "target_boundary_contact_stats": target_boundary_contact_stats,
        "target_nonwheel_contact_stats": target_nonwheel_contact_stats,
        "max_target_penetration_m": max_target_penetration_m,
        "max_target_penetration_after_home_ready_m": max_target_penetration_after_ready_m,
        "sampled_closure_m": {"all": max(float(row["closure_m"]) for row in rows),
            "after_home_ready": max((float(row["closure_m"]) for row in rows
                if home_ready_time is not None and float(row["time_s"]) >= home_ready_time),
                default=0.0), "sample_rate_hz": 200},
        "platform_bounds_m": {"x": [target_x0, target_x1],
            "y": [target_y0, target_y1], "height": target_height,
            "platform": args.platform,
            "initial_xy_m": [args.start_x, start_y],
            "wheel_center_check": "x0 < wheel_x < x1 and y0 < wheel_y < y1"},
        "command": {"speed_mps": args.speed, "leg_length_m": leg_length, "drive_mode": 1,
            "step_climb_mode": 2, "release_after_phase7_s": args.release_after,
            "climb_assist": args.climb_assist,
            "climb_assist_delay_s": args.assist_delay,
            "input_delay_after_stand_s": args.input_delay,
            "climb_assist_release_after_s": args.assist_release_after,
            "release_duration_s": .75 if args.release_after is not None and
                not args.hold_after_release else None,
            "disable_drive_delay_after_release_s": args.disable_drive_delay,
            "jump_delay_after_release_s": args.jump_delay,
            "inputs": "W；可选松开 W 后驻车、模拟超时或按空格"},
        "physics": {"terrain": "course", "correct_link_collisions": True,
            "knee_range": knee_range_info(model),
            "guide_rollers": guide_roller_info(model),
            "noslip_iterations": 10, "gas_spring_n_each": 450.0,
            "joint_continuous_nm": 10.0, "joint_peak_nm": 45.0,
            "wheel_limit_nm": 5.2,
            "total_mass_kg": float(model.body_subtreemass[model.body("base_link").id]),
            "mass_scope": "base_link 子树，包括全部机器人连杆与导轮",
            "fixed_scene_mass_kg": float(model.body_mass.sum() -
                model.body_subtreemass[model.body("base_link").id]),
            "whole_model_mass_kg": float(model.body_mass.sum())},
        "control_boundary": "控制仅用IMU、电机编码器/反馈和用户W指令；位置、接触、法向和地形边界只用于验收",
        "core_error": core_error,
        "mujoco_warning_counts": warnings,
        "controller_source": str(args.source),
        "controller_sha256": source_hash,
        "controller_changed_during_run": sha256(args.source) != source_hash,
        "terrain_sha256": sha256(Path(__file__).with_name("WbrTerrain.py")),
        "test_sha256": test_hash,
        "test_changed_during_run": sha256(Path(__file__)) != test_hash,
        "build_command": [str(item) for item in build_command],
        "trajectory": {"path": str(trajectory_path), "fields": ["time_s", "qpos"],
            "sample_rate_hz": 200, "purpose": "仅用于实测轨迹复现渲染与核对"},
    }
    report["passed"] = (report["passed"] and not report["controller_changed_during_run"] and
                        not report["test_changed_during_run"])
    (args.output / "summary.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
