"""Run the ARBATOS wheel-leg control core in a minimal MuJoCo scene."""

from __future__ import annotations

import argparse
import ctypes
import math
import os
import re
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[3]
TOOLS_SIM = REPO_ROOT / "tools" / "sim"
if str(TOOLS_SIM) not in sys.path:
    sys.path.insert(0, str(TOOLS_SIM))

import robot_sim  # type: ignore  # noqa: E402


HERE = Path(__file__).resolve().parent
DEFAULT_MODEL = HERE / "wheelleg_minimal.xml"
BRIDGE_SOURCE = HERE / "wheelleg_core_bridge.c"
DEFAULT_NATIVE_DIR = REPO_ROOT / "tmp" / "mujoco_wheelleg"

SIDE_LEFT = 0
SIDE_RIGHT = 1
CORE_ACT_RIGHT_FRONT = 0
CORE_ACT_RIGHT_BACK = 1
CORE_ACT_RIGHT_WHEEL = 2
CORE_ACT_LEFT_FRONT = 3
CORE_ACT_LEFT_BACK = 4
CORE_ACT_LEFT_WHEEL = 5


class Geometry(ctypes.Structure):
    _fields_ = [
        ("l1_m", ctypes.c_float),
        ("l2_m", ctypes.c_float),
        ("l3_m", ctypes.c_float),
        ("l4_m", ctypes.c_float),
        ("l5_m", ctypes.c_float),
    ]


class PidConfig(ctypes.Structure):
    _fields_ = [
        ("kp", ctypes.c_float),
        ("ki", ctypes.c_float),
        ("kd", ctypes.c_float),
        ("max_out", ctypes.c_float),
        ("max_iout", ctypes.c_float),
    ]


Float4 = ctypes.c_float * 4
LqrPoly = Float4 * 12


class BridgeConfig(ctypes.Structure):
    _fields_ = [
        ("geometry", Geometry),
        ("wheel_radius_m", ctypes.c_float),
        ("lqr_poly", LqrPoly),
        ("support_bias_n", ctypes.c_float),
        ("min_leg_length_m", ctypes.c_float),
        ("max_leg_length_m", ctypes.c_float),
        ("default_leg_length_m", ctypes.c_float),
        ("max_wheel_torque_nm", ctypes.c_float),
        ("max_joint_torque_nm", ctypes.c_float),
        ("max_support_force_n", ctypes.c_float),
        ("observer_lpf", ctypes.c_float),
        ("lqr_wheel_torque_scale", ctypes.c_float),
        ("lqr_hip_torque_scale", ctypes.c_float),
        ("pitch_balance_offset_right_rad", ctypes.c_float),
        ("pitch_balance_offset_left_rad", ctypes.c_float),
        ("leg_length_pid", PidConfig),
        ("leg_split_pid", PidConfig),
        ("turn_pid", PidConfig),
        ("roll_pid", PidConfig),
    ]


FloatJoint2 = ctypes.c_float * 2


class CoreLegCalc(ctypes.Structure):
    _fields_ = [
        ("l1", ctypes.c_float),
        ("l2", ctypes.c_float),
        ("l3", ctypes.c_float),
        ("l4", ctypes.c_float),
        ("l5", ctypes.c_float),
        ("phi1", ctypes.c_float),
        ("phi2", ctypes.c_float),
        ("phi3", ctypes.c_float),
        ("phi4", ctypes.c_float),
        ("phi0", ctypes.c_float),
        ("alpha", ctypes.c_float),
        ("d_alpha", ctypes.c_float),
        ("length", ctypes.c_float),
        ("d_length", ctypes.c_float),
        ("dd_length", ctypes.c_float),
        ("theta", ctypes.c_float),
        ("d_theta", ctypes.c_float),
        ("dd_theta", ctypes.c_float),
        ("f0", ctypes.c_float),
        ("tp", ctypes.c_float),
        ("fn", ctypes.c_float),
        ("joint_torque", FloatJoint2),
        ("last_phi0", ctypes.c_float),
        ("last_length", ctypes.c_float),
        ("last_d_length", ctypes.c_float),
        ("last_d_theta", ctypes.c_float),
        ("first", ctypes.c_uint8),
        ("contact", ctypes.c_uint8),
    ]


class CoreObserver(ctypes.Structure):
    _fields_ = [
        ("x_m", ctypes.c_float),
        ("v_mps", ctypes.c_float),
    ]


class CorePid(ctypes.Structure):
    _fields_ = [
        ("kp", ctypes.c_float),
        ("ki", ctypes.c_float),
        ("kd", ctypes.c_float),
        ("max_out", ctypes.c_float),
        ("max_iout", ctypes.c_float),
        ("iout", ctypes.c_float),
        ("last_err", ctypes.c_float),
    ]


CoreLegArray = CoreLegCalc * 2
CorePidArray = CorePid * 2


class BridgeState(ctypes.Structure):
    _fields_ = [
        ("leg", CoreLegArray),
        ("observer", CoreObserver),
        ("leg_pid", CorePidArray),
        ("split_pid", CorePid),
        ("yaw_set", ctypes.c_float),
        ("yaw_inited", ctypes.c_uint8),
    ]


class BridgeInput(ctypes.Structure):
    _fields_ = [
        ("dt_s", ctypes.c_float),
        ("pitch_rad", ctypes.c_float),
        ("roll_rad", ctypes.c_float),
        ("yaw_rad", ctypes.c_float),
        ("gyro_x_radps", ctypes.c_float),
        ("gyro_y_radps", ctypes.c_float),
        ("gyro_z_radps", ctypes.c_float),
        ("right_front_pos_rad", ctypes.c_float),
        ("right_back_pos_rad", ctypes.c_float),
        ("right_wheel_vel_radps", ctypes.c_float),
        ("left_front_pos_rad", ctypes.c_float),
        ("left_back_pos_rad", ctypes.c_float),
        ("left_wheel_vel_radps", ctypes.c_float),
        ("target_v_mps", ctypes.c_float),
        ("target_leg_m", ctypes.c_float),
        ("target_foot_x_m", ctypes.c_float),
        ("target_yaw_rate_radps", ctypes.c_float),
        ("use_vmc", ctypes.c_uint8),
    ]


Float2 = ctypes.c_float * 2
Float2x2 = Float2 * 2
Float6 = ctypes.c_float * 6


class BridgeOutput(ctypes.Structure):
    _fields_ = [
        ("wheel_torque_nm", Float2),
        ("joint_torque_nm", Float2x2),
        ("leg_length_m", Float2),
        ("leg_theta_rad", Float2),
        ("observer_x_m", ctypes.c_float),
        ("observer_v_mps", ctypes.c_float),
        ("actuator_torque_nm", Float6),
        ("ok", ctypes.c_uint8),
    ]


def lib_suffix() -> str:
    if os.name == "nt":
        return ".dll"
    if sys.platform == "darwin":
        return ".dylib"
    return ".so"


def find_compiler() -> tuple[str, list[str]] | None:
    for name in ("gcc", "clang"):
        path = shutil.which(name)
        if path:
            return path, ["unix"]
    zig = shutil.which("zig")
    if zig:
        return zig, ["zig"]
    cl = shutil.which("cl")
    if cl:
        return cl, ["msvc"]
    return None


def bridge_library_path(native_dir: Path) -> Path:
    return native_dir / ("arbatos_wheelleg_core_bridge" + lib_suffix())


def source_dependencies() -> list[Path]:
    return [
        BRIDGE_SOURCE,
        REPO_ROOT / "shared" / "application" / "wheelleg" / "wheelleg_core.h",
        REPO_ROOT / "shared" / "application" / "robot" / "control_core.h",
        REPO_ROOT / "shared" / "application" / "robot" / "actuator_cmd.h",
        REPO_ROOT / "shared" / "application" / "wheelleg" / "wheelleg_msg.h",
    ]


def needs_rebuild(target: Path) -> bool:
    if not target.exists():
        return True
    target_mtime = target.stat().st_mtime
    return any(path.exists() and path.stat().st_mtime > target_mtime for path in source_dependencies())


def compile_bridge(native_dir: Path, rebuild: bool) -> Path:
    native_dir.mkdir(parents=True, exist_ok=True)
    target = bridge_library_path(native_dir)
    if not rebuild and not needs_rebuild(target):
        return target

    compiler = find_compiler()
    if compiler is None:
        raise SystemExit(
            "No C compiler found for the native wheelleg core bridge. "
            "Install gcc, clang, or MSVC Build Tools, then rerun this command."
        )

    compiler_path, tags = compiler
    include_dirs = [
        REPO_ROOT / "shared" / "application" / "wheelleg",
        REPO_ROOT / "shared" / "application" / "robot",
        REPO_ROOT / "shared" / "components" / "support",
    ]
    if "msvc" in tags:
        obj = native_dir / "wheelleg_core_bridge.obj"
        cmd = [compiler_path, "/nologo", "/LD", "/O2"]
        cmd += [f"/I{path}" for path in include_dirs]
        cmd += [str(BRIDGE_SOURCE), f"/Fo{obj}", f"/Fe{target}"]
    elif "zig" in tags:
        cmd = [compiler_path, "cc", "-shared", "-O2", "-std=c99", "-Wno-unused-function"]
        cmd += [f"-I{path}" for path in include_dirs]
        cmd += [str(BRIDGE_SOURCE), "-o", str(target)]
    else:
        cmd = [compiler_path, "-shared", "-O2", "-std=c99", "-Wno-unused-function"]
        cmd += [f"-I{path}" for path in include_dirs]
        cmd += [str(BRIDGE_SOURCE), "-o", str(target)]
        if os.name != "nt":
            cmd.append("-lm")

    result = subprocess.run(cmd, cwd=REPO_ROOT, text=True, capture_output=True)
    if result.returncode != 0:
        raise SystemExit(
            "Failed to build native wheelleg core bridge.\n"
            f"command: {' '.join(cmd)}\n"
            f"stdout:\n{result.stdout}\n"
            f"stderr:\n{result.stderr}"
        )
    return target


def load_bridge(native_dir: Path, rebuild: bool) -> ctypes.CDLL:
    lib_path = compile_bridge(native_dir, rebuild)
    lib = ctypes.CDLL(str(lib_path))
    lib.arbatos_wheelleg_bridge_version.restype = ctypes.c_uint32
    lib.arbatos_wheelleg_bridge_config_defaults.argtypes = [ctypes.POINTER(BridgeConfig)]
    lib.arbatos_wheelleg_bridge_state_init.argtypes = [
        ctypes.POINTER(BridgeState),
        ctypes.POINTER(BridgeConfig),
    ]
    lib.arbatos_wheelleg_bridge_home_pose.argtypes = [
        ctypes.POINTER(BridgeConfig),
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
    ]
    lib.arbatos_wheelleg_bridge_home_pose.restype = ctypes.c_uint8
    lib.arbatos_wheelleg_bridge_step.argtypes = [
        ctypes.POINTER(BridgeConfig),
        ctypes.POINTER(BridgeState),
        ctypes.POINTER(BridgeInput),
        ctypes.POINTER(BridgeOutput),
    ]
    lib.arbatos_wheelleg_bridge_step.restype = ctypes.c_uint8
    return lib


def parse_float(expr: str | None, fallback: float) -> float:
    if expr is None:
        return fallback
    cleaned = re_sub_casts(expr)
    match = re.search(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", cleaned)
    if not match:
        return fallback
    return float(match.group(0))


def re_sub_casts(expr: str) -> str:
    return re.sub(r"\([A-Za-z_][A-Za-z0-9_\s\*]*\)", "", expr).replace("f", "").replace("F", "")


def parse_number_list(text: str) -> list[float]:
    return [
        float(match.group(0))
        for match in re.finditer(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", re_sub_casts(text))
    ]


def read_wheelleg_config_body(project: str) -> str:
    config_path = REPO_ROOT / "Robotconfig" / project / "config.c"
    if not config_path.exists():
        raise SystemExit(f"Missing project config: {config_path}")
    text = robot_sim.strip_c_comments(config_path.read_text(encoding="utf-8", errors="ignore"))
    body = robot_sim.extract_initializer(text, "wheelleg_mit")
    if body is None:
        raise SystemExit(f"{config_path} has no wheelleg_mit initializer")
    return body


def load_project_config(config: BridgeConfig, project: str) -> None:
    body = read_wheelleg_config_body(project)
    scalar_fields = {
        "l1_m": ("geometry", "l1_m"),
        "l2_m": ("geometry", "l2_m"),
        "l3_m": ("geometry", "l3_m"),
        "l4_m": ("geometry", "l4_m"),
        "l5_m": ("geometry", "l5_m"),
        "wheel_radius_m": (None, "wheel_radius_m"),
        "support_bias_n": (None, "support_bias_n"),
        "min_leg_length_m": (None, "min_leg_length_m"),
        "max_leg_length_m": (None, "max_leg_length_m"),
        "default_leg_length_m": (None, "default_leg_length_m"),
        "max_wheel_torque_nm": (None, "max_wheel_torque_nm"),
        "max_joint_torque_nm": (None, "max_joint_torque_nm"),
        "max_support_force_n": (None, "max_support_force_n"),
        "observer_lpf": (None, "observer_lpf"),
        "lqr_wheel_torque_scale": (None, "lqr_wheel_torque_scale"),
        "lqr_hip_torque_scale": (None, "lqr_hip_torque_scale"),
        "pitch_balance_offset_right_rad": (None, "pitch_balance_offset_right_rad"),
        "pitch_balance_offset_left_rad": (None, "pitch_balance_offset_left_rad"),
    }
    for field_name, target in scalar_fields.items():
        parent, attr = target
        raw = robot_sim.extract_named_value(body, field_name)
        holder = getattr(config, parent) if parent else config
        setattr(holder, attr, parse_float(raw, float(getattr(holder, attr))))

    lqr_body = robot_sim.extract_initializer(body, "lqr_poly")
    if lqr_body:
        rows = robot_sim.split_top_level(lqr_body)
        for row_index, row in enumerate(rows[:12]):
            values = parse_number_list(row)
            for coeff_index, value in enumerate(values[:4]):
                config.lqr_poly[row_index][coeff_index] = value

    for pid_name in ("leg_length_pid", "leg_split_pid", "turn_pid", "roll_pid"):
        pid_body = robot_sim.extract_initializer(body, pid_name)
        if not pid_body:
            continue
        values = parse_number_list(pid_body)
        if len(values) >= 5:
            pid = getattr(config, pid_name)
            pid.kp, pid.ki, pid.kd, pid.max_out, pid.max_iout = values[:5]


def validate_model_xml(model_path: Path) -> None:
    try:
        ET.parse(model_path)
    except ET.ParseError as exc:
        raise SystemExit(f"Invalid MJCF XML {model_path}: {exc}") from exc


def quat_to_euler_wxyz(quat: Iterable[float]) -> tuple[float, float, float]:
    w, x, y, z = quat
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def import_mujoco():
    try:
        import mujoco  # type: ignore
    except ModuleNotFoundError as exc:
        raise SystemExit("Python package 'mujoco' is not installed. Run: python -m pip install mujoco") from exc
    return mujoco


def joint_qpos(data, name: str) -> float:
    return float(data.joint(name).qpos[0])


def joint_qvel(data, name: str) -> float:
    return float(data.joint(name).qvel[0])


def set_joint_qpos(data, name: str, value: float) -> None:
    data.joint(name).qpos[0] = value


def actuator_id(mujoco, model, name: str) -> int:
    idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if idx < 0:
        raise SystemExit(f"MJCF actuator not found: {name}")
    return int(idx)


def fill_bridge_input(data, args: argparse.Namespace) -> BridgeInput:
    base_qpos = data.joint("base_free").qpos
    base_qvel = data.joint("base_free").qvel
    roll, pitch, yaw = quat_to_euler_wxyz(base_qpos[3:7])
    return BridgeInput(
        dt_s=float(args.dt),
        pitch_rad=float(pitch),
        roll_rad=float(roll),
        yaw_rad=float(yaw),
        gyro_x_radps=float(base_qvel[3]),
        gyro_y_radps=float(base_qvel[4]),
        gyro_z_radps=float(base_qvel[5]),
        right_front_pos_rad=joint_qpos(data, "right_front_hinge"),
        right_back_pos_rad=joint_qpos(data, "right_back_hinge"),
        right_wheel_vel_radps=joint_qvel(data, "right_wheel_hinge"),
        left_front_pos_rad=joint_qpos(data, "left_front_hinge"),
        left_back_pos_rad=joint_qpos(data, "left_back_hinge"),
        left_wheel_vel_radps=joint_qvel(data, "left_wheel_hinge"),
        target_v_mps=float(args.target_v),
        target_leg_m=float(args.target_leg),
        target_foot_x_m=float(args.target_foot_x),
        target_yaw_rate_radps=float(args.target_yaw_rate),
        use_vmc=1 if args.vmc else 0,
    )


def apply_output(mujoco, model, data, actuator_ids: dict[int, int], output: BridgeOutput) -> None:
    for core_index, mj_index in actuator_ids.items():
        data.ctrl[mj_index] = output.actuator_torque_nm[core_index]


def run_step(lib, config: BridgeConfig, state: BridgeState, data, args: argparse.Namespace) -> BridgeOutput:
    bridge_input = fill_bridge_input(data, args)
    output = BridgeOutput()
    ok = lib.arbatos_wheelleg_bridge_step(
        ctypes.byref(config),
        ctypes.byref(state),
        ctypes.byref(bridge_input),
        ctypes.byref(output),
    )
    if ok == 0 or output.ok == 0:
        raise RuntimeError("wheelleg core bridge step failed")
    return output


def run_sim(args: argparse.Namespace) -> None:
    mujoco = import_mujoco()
    lib = load_bridge(Path(args.native_dir), rebuild=not args.no_rebuild)
    config = BridgeConfig()
    lib.arbatos_wheelleg_bridge_config_defaults(ctypes.byref(config))
    load_project_config(config, args.project)

    model = mujoco.MjModel.from_xml_path(str(args.model))
    model.opt.timestep = float(args.dt)
    data = mujoco.MjData(model)
    state = BridgeState()
    lib.arbatos_wheelleg_bridge_state_init(ctypes.byref(state), ctypes.byref(config))

    front_home = ctypes.c_float()
    back_home = ctypes.c_float()
    if lib.arbatos_wheelleg_bridge_home_pose(ctypes.byref(config), ctypes.byref(front_home), ctypes.byref(back_home)) == 0:
        front_home.value = -2.0
        back_home.value = -2.0

    set_joint_qpos(data, "right_front_hinge", front_home.value)
    set_joint_qpos(data, "right_back_hinge", back_home.value)
    set_joint_qpos(data, "left_front_hinge", front_home.value)
    set_joint_qpos(data, "left_back_hinge", back_home.value)
    mujoco.mj_forward(model, data)

    actuator_ids = {
        CORE_ACT_RIGHT_FRONT: actuator_id(mujoco, model, "right_front_motor"),
        CORE_ACT_RIGHT_BACK: actuator_id(mujoco, model, "right_back_motor"),
        CORE_ACT_RIGHT_WHEEL: actuator_id(mujoco, model, "right_wheel_motor"),
        CORE_ACT_LEFT_FRONT: actuator_id(mujoco, model, "left_front_motor"),
        CORE_ACT_LEFT_BACK: actuator_id(mujoco, model, "left_back_motor"),
        CORE_ACT_LEFT_WHEEL: actuator_id(mujoco, model, "left_wheel_motor"),
    }

    def step_once(step_index: int) -> None:
        start = time.perf_counter()
        output = run_step(lib, config, state, data, args)
        apply_output(mujoco, model, data, actuator_ids, output)
        mujoco.mj_step(model, data)
        if args.print_every > 0 and step_index % args.print_every == 0:
            roll, pitch, _yaw = quat_to_euler_wxyz(data.joint("base_free").qpos[3:7])
            print(
                f"t={data.time:6.3f}s pitch={pitch:+.3f} roll={roll:+.3f} "
                f"v={output.observer_v_mps:+.3f} "
                f"tau_wheel_r={output.wheel_torque_nm[SIDE_RIGHT]:+.3f} "
                f"tau_wheel_l={output.wheel_torque_nm[SIDE_LEFT]:+.3f}"
            )
        if args.realtime:
            elapsed = time.perf_counter() - start
            time.sleep(max(0.0, float(args.dt) - elapsed))

    steps = max(1, int(args.duration_s / args.dt))
    if args.viewer:
        import mujoco.viewer  # type: ignore

        with mujoco.viewer.launch_passive(model, data) as viewer:
            for step_index in range(steps):
                if not viewer.is_running():
                    break
                step_once(step_index)
                viewer.sync()
    else:
        for step_index in range(steps):
            step_once(step_index)


def parse_args(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", default="MINIWHEELEG-C")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--native-dir", type=Path, default=DEFAULT_NATIVE_DIR)
    parser.add_argument("--duration-s", type=float, default=5.0)
    parser.add_argument("--dt", type=float, default=0.003)
    parser.add_argument("--target-v", type=float, default=0.0)
    parser.add_argument("--target-leg", type=float, default=0.10)
    parser.add_argument("--target-foot-x", type=float, default=0.0)
    parser.add_argument("--target-yaw-rate", type=float, default=0.0)
    parser.add_argument("--vmc", action="store_true", help="Enable VMC joint torque output.")
    parser.add_argument("--viewer", action="store_true", help="Open the MuJoCo viewer.")
    parser.add_argument("--realtime", action="store_true", help="Sleep to roughly match wall time.")
    parser.add_argument("--print-every", type=int, default=100)
    parser.add_argument("--no-rebuild", action="store_true")
    parser.add_argument("--check", action="store_true", help="Validate files and config without importing MuJoCo.")
    return parser.parse_args(list(argv))


def main(argv: Iterable[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    validate_model_xml(args.model)
    body = read_wheelleg_config_body(args.project)
    if args.check:
        lqr_rows = robot_sim.extract_initializer(body, "lqr_poly")
        row_count = len(robot_sim.split_top_level(lqr_rows or ""))
        print(f"MuJoCo wheelleg check ok: project={args.project} model={args.model} lqr_rows={row_count}")
        return 0
    run_sim(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
