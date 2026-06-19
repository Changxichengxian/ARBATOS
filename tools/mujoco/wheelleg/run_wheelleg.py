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
BRIDGE_SOURCE = HERE / "WheelLegCoreBridge.c"
DEFAULT_NATIVE_DIR = REPO_ROOT / "tmp" / "mujoco_wheelleg"

SIM_TOTAL_MASS_KG = 2.25
SIM_BODY_MASS_KG = 1.81
SIM_WHEEL_END_MASS_TOTAL_KG = 0.320
SIM_LEG_MASS_TOTAL_KG = 0.120
SIM_BODY_LENGTH_M = 0.115
SIM_BODY_HEIGHT_M = 0.055
SIM_BODY_COM_TO_HIP_M = -0.004
SIM_TRACK_HALF_M = 0.090
SIM_WHEEL_WIDTH_M = 0.024

SIDE_LEFT = 0
SIDE_RIGHT = 1
CORE_ACT_RIGHT_FRONT = 0
CORE_ACT_RIGHT_BACK = 1
CORE_ACT_RIGHT_WHEEL = 2
CORE_ACT_LEFT_FRONT = 3
CORE_ACT_LEFT_BACK = 4
CORE_ACT_LEFT_WHEEL = 5
SIM_ACTUATOR_SIGNS_DIAMOND = {
    CORE_ACT_RIGHT_FRONT: 1.0,
    CORE_ACT_RIGHT_BACK: 1.0,
    CORE_ACT_RIGHT_WHEEL: 1.0,
    CORE_ACT_LEFT_FRONT: 1.0,
    CORE_ACT_LEFT_BACK: 1.0,
    CORE_ACT_LEFT_WHEEL: -1.0,
}
SIM_ACTUATOR_SIGNS_CORE = {
    CORE_ACT_RIGHT_FRONT: 1.0,
    CORE_ACT_RIGHT_BACK: -1.0,
    CORE_ACT_RIGHT_WHEEL: 1.0,
    CORE_ACT_LEFT_FRONT: 1.0,
    CORE_ACT_LEFT_BACK: -1.0,
    CORE_ACT_LEFT_WHEEL: -1.0,
}
WHEEL_ACTUATORS = {CORE_ACT_RIGHT_WHEEL, CORE_ACT_LEFT_WHEEL}


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
        ("support_only", ctypes.c_uint8),
        ("jump_force_n", ctypes.c_float),
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
    local_appdata = os.environ.get("LOCALAPPDATA")
    if local_appdata:
        winget_zig = Path(local_appdata) / "Microsoft" / "WinGet" / "Links" / "zig.exe"
        if winget_zig.exists():
            return str(winget_zig), ["zig"]
    cl = shutil.which("cl")
    if cl:
        return cl, ["msvc"]
    return None


def bridge_library_path(native_dir: Path) -> Path:
    return native_dir / f"arbatos_wheelleg_core_bridge_{os.getpid()}{lib_suffix()}"


def source_dependencies() -> list[Path]:
    return [
        BRIDGE_SOURCE,
        REPO_ROOT / "shared" / "application" / "wheelleg" / "WheelLegCore.h",
        REPO_ROOT / "shared" / "application" / "robot" / "ControlCore.h",
        REPO_ROOT / "shared" / "application" / "robot" / "LowCmd.h",
        REPO_ROOT / "shared" / "application" / "wheelleg" / "WheelLegMsg.h",
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
        obj = native_dir / "WheelLegCoreBridge.obj"
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
    config_path = REPO_ROOT / "Robotconfig" / project / "RobotConfig.c"
    if not config_path.exists():
        raise SystemExit(f"Missing project config: {config_path}")
    text = robot_sim.strip_c_comments(robot_sim.read_text_with_local_config_includes(config_path))
    body = robot_sim.extract_initializer(text, "WheelLegMit")
    if body is None:
        raise SystemExit(f"{config_path} has no WheelLegMit initializer")
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


def validate_fivebar_config(config: BridgeConfig) -> None:
    values = {
        "l1_m": config.geometry.l1_m,
        "l2_m": config.geometry.l2_m,
        "l3_m": config.geometry.l3_m,
        "l4_m": config.geometry.l4_m,
        "l5_m": config.geometry.l5_m,
        "wheel_radius_m": config.wheel_radius_m,
    }
    invalid = [name for name, value in values.items() if float(value) <= 0.0]
    if invalid:
        raise SystemExit(f"Invalid wheelleg geometry config: {', '.join(invalid)} must be positive")


def f6(value: float) -> str:
    return f"{float(value):.6f}"


def base_mode(args: argparse.Namespace) -> str:
    if getattr(args, "bench", False):
        return "bench"
    if getattr(args, "drive_plane", False):
        return "drive_plane"
    if getattr(args, "yaw_plane", False):
        return "yaw_plane"
    if getattr(args, "planar", False):
        return "planar"
    return "free"


def generated_model_path(native_dir: Path, project: str, mode: str, branch: str) -> Path:
    safe_project = re.sub(r"[^A-Za-z0-9_.-]+", "_", project)
    safe_branch = re.sub(r"[^A-Za-z0-9_.-]+", "_", branch)
    return native_dir / f"wheelleg_fivebar_{safe_project}_{mode}_{safe_branch}_{os.getpid()}.xml"


def clamped_leg_length(config: BridgeConfig, target_leg_m: float | None) -> float:
    length = float(target_leg_m if target_leg_m is not None and target_leg_m > 0.02 else config.default_leg_length_m)
    if length <= 0.02:
        length = float(config.min_leg_length_m if config.min_leg_length_m > 0.02 else 0.10)
    if config.min_leg_length_m > 0.02 and config.max_leg_length_m > config.min_leg_length_m:
        length = max(float(config.min_leg_length_m), min(length, float(config.max_leg_length_m)))
    return length


def read_project_scalar(project: str, field_name: str, fallback: float = float("nan")) -> float:
    raw = robot_sim.extract_named_value(read_wheelleg_config_body(project), field_name)
    return parse_float(raw, fallback)


def print_model_params(args: argparse.Namespace, config: BridgeConfig, model_path: Path) -> None:
    print(f"project={args.project}")
    print(f"model={model_path}")
    print(f"leg_branch={args.leg_branch}")
    print("control_core=shared/application/wheelleg/WheelLegCore.h")
    print("project_config=Robotconfig/{}/RobotConfig.c: WheelLegMit".format(args.project))
    print(
        "geometry_from_config="
        f"l1={float(config.geometry.l1_m):.5f}, "
        f"l2={float(config.geometry.l2_m):.5f}, "
        f"l3={float(config.geometry.l3_m):.5f}, "
        f"l4={float(config.geometry.l4_m):.5f}, "
        f"l5={float(config.geometry.l5_m):.5f}, "
        f"wheel_radius={float(config.wheel_radius_m):.5f}"
    )
    print(
        "control_from_config="
        f"default_leg={float(config.default_leg_length_m):.3f}, "
        f"leg_range={float(config.min_leg_length_m):.3f}..{float(config.max_leg_length_m):.3f}, "
        f"support_bias={float(config.support_bias_n):.2f}, "
        f"max_wheel_torque={float(config.max_wheel_torque_nm):.2f}, "
        f"max_joint_torque={float(config.max_joint_torque_nm):.2f}"
    )
    print(f"leg_mass_kg_from_config={read_project_scalar(args.project, 'leg_mass_kg'):.3f}")
    print("lqr_mass_source=tools/wheelleg_lqr/small_3510_lqr_report.md")
    print(
        "lqr_mass_assumption="
        f"total={SIM_TOTAL_MASS_KG:.3f}, "
        f"body={SIM_BODY_MASS_KG:.3f}, "
        f"wheel_end_total={SIM_WHEEL_END_MASS_TOTAL_KG:.3f}, "
        f"leg_total={SIM_LEG_MASS_TOTAL_KG:.3f}"
    )
    print(
        "sim_shape_assumption="
        f"track_half={SIM_TRACK_HALF_M:.3f}, "
        f"wheel_width={SIM_WHEEL_WIDTH_M:.3f}, "
        f"body_length={SIM_BODY_LENGTH_M:.3f}, "
        f"body_height={SIM_BODY_HEIGHT_M:.3f}"
    )
    try:
        mujoco = import_mujoco()
        model = mujoco.MjModel.from_xml_path(str(model_path))
        total_mass = sum(float(model.body_mass[i]) for i in range(1, model.nbody))
        print(f"generated_mujoco_body_mass_total={total_mass:.3f}")
    except Exception as exc:
        print(f"generated_mujoco_body_mass_total=unavailable ({exc})")


def build_fivebar_model_xml(config: BridgeConfig, base_height_m: float, mode: str = "free") -> str:
    validate_fivebar_config(config)
    l1 = float(config.geometry.l1_m)
    l2 = float(config.geometry.l2_m)
    l3 = float(config.geometry.l3_m)
    l4 = float(config.geometry.l4_m)
    l5 = float(config.geometry.l5_m)
    wheel_radius = float(config.wheel_radius_m)
    half_l5 = l5 * 0.5
    track_half = 0.090
    wheel_width = SIM_WHEEL_WIDTH_M
    crank_radius = 0.0045
    rod_radius = 0.0038
    pivot_radius = 0.0070
    carrier_mass = SIM_WHEEL_END_MASS_TOTAL_KG * 0.0625
    wheel_mass = SIM_WHEEL_END_MASS_TOTAL_KG * 0.4375
    upper_link_mass = SIM_LEG_MASS_TOTAL_KG * 0.1000
    lower_link_mass = SIM_LEG_MASS_TOTAL_KG * 0.1417
    pivot_mass = SIM_LEG_MASS_TOTAL_KG * 0.0083
    crossbar_mass = 0.015
    body_mass = SIM_BODY_MASS_KG - 2.0 * crossbar_mass
    body_half_x = SIM_BODY_LENGTH_M * 0.5
    body_half_y = 0.052
    body_half_z = SIM_BODY_HEIGHT_M * 0.5

    if mode == "bench":
        base_joints = ""
    elif mode == "drive_plane":
        base_joints = """<joint name="base_x" type="slide" axis="1 0 0" damping="0.02"/>
      <joint name="base_y" type="slide" axis="0 1 0" damping="0.02"/>
      <joint name="base_z" type="slide" axis="0 0 1" damping="0.02"/>
      <joint name="base_yaw" type="hinge" axis="0 0 1" damping="0.01"/>"""
    elif mode == "yaw_plane":
        base_joints = """<joint name="base_x" type="slide" axis="1 0 0" damping="0.02"/>
      <joint name="base_y" type="slide" axis="0 1 0" damping="0.02"/>
      <joint name="base_z" type="slide" axis="0 0 1" damping="0.02"/>
      <joint name="base_yaw" type="hinge" axis="0 0 1" damping="0.01"/>
      <joint name="base_pitch" type="hinge" axis="0 1 0" damping="0.01"/>"""
    elif mode == "planar":
        base_joints = """<joint name="base_x" type="slide" axis="1 0 0" damping="0.02"/>
      <joint name="base_z" type="slide" axis="0 0 1" damping="0.02"/>
      <joint name="base_pitch" type="hinge" axis="0 1 0" damping="0.01"/>"""
    else:
        base_joints = """<freejoint name="base_free"/>"""

    def side_xml(prefix: str, y: float, material: str) -> str:
        return f"""
      <body name="{prefix}_carrier" pos="0 {f6(y)} 0">
        <joint name="{prefix}_carrier_x" type="slide" axis="1 0 0" range="-0.090 0.090" damping="0.4"/>
        <joint name="{prefix}_carrier_z" type="slide" axis="0 0 1" range="-0.180 -0.030" damping="0.4"/>
        <geom name="{prefix}_carrier_geom" type="sphere" size="0.010" mass="{f6(carrier_mass)}"
              material="{material}" contype="0" conaffinity="0"/>
        <site name="{prefix}_foot_site" pos="0 0 0" size="0.004"/>
        <body name="{prefix}_wheel" pos="0 0 0">
          <joint name="{prefix}_wheel_hinge" type="hinge" axis="0 1 0" damping="0.004"/>
          <geom name="{prefix}_wheel_geom" type="cylinder" size="{f6(wheel_radius)} {f6(wheel_width)}"
                euler="1.5707963268 0 0" mass="{f6(wheel_mass)}" material="wheel_mat"
                friction="1.8 0.04 0.002"/>
        </body>
      </body>

      <body name="{prefix}_front_crank" pos="{f6(-half_l5)} {f6(y)} 0">
        <joint name="{prefix}_front_hinge" type="hinge" axis="0 1 0" range="-3.40 3.40" damping="0.02"/>
        <geom name="{prefix}_front_pivot_geom" type="sphere" size="{f6(pivot_radius)}" mass="{f6(pivot_mass)}"
              material="{material}" contype="0" conaffinity="0"/>
        <geom name="{prefix}_front_crank_geom" type="capsule" fromto="0 0 0 0 0 {f6(-l1)}"
              size="{f6(crank_radius)}" mass="{f6(upper_link_mass)}" material="{material}" contype="0" conaffinity="0"/>
        <body name="{prefix}_front_rod" pos="0 0 {f6(-l1)}">
          <joint name="{prefix}_front_knee" type="hinge" axis="0 1 0" range="-3.40 3.40" damping="0.01"/>
          <geom name="{prefix}_front_rod_geom" type="capsule" fromto="0 0 0 0 0 {f6(-l2)}"
                size="{f6(rod_radius)}" mass="{f6(lower_link_mass)}" material="{material}" contype="0" conaffinity="0"/>
          <site name="{prefix}_front_rod_end" pos="0 0 {f6(-l2)}" size="0.004"/>
        </body>
      </body>

      <body name="{prefix}_back_crank" pos="{f6(half_l5)} {f6(y)} 0">
        <joint name="{prefix}_back_hinge" type="hinge" axis="0 1 0" range="-3.40 3.40" damping="0.02"/>
        <geom name="{prefix}_back_pivot_geom" type="sphere" size="{f6(pivot_radius)}" mass="{f6(pivot_mass)}"
              material="{material}" contype="0" conaffinity="0"/>
        <geom name="{prefix}_back_crank_geom" type="capsule" fromto="0 0 0 0 0 {f6(-l4)}"
              size="{f6(crank_radius)}" mass="{f6(upper_link_mass)}" material="{material}" contype="0" conaffinity="0"/>
        <body name="{prefix}_back_rod" pos="0 0 {f6(-l4)}">
          <joint name="{prefix}_back_knee" type="hinge" axis="0 1 0" range="-3.40 3.40" damping="0.01"/>
          <geom name="{prefix}_back_rod_geom" type="capsule" fromto="0 0 0 0 0 {f6(-l3)}"
                size="{f6(rod_radius)}" mass="{f6(lower_link_mass)}" material="{material}" contype="0" conaffinity="0"/>
          <site name="{prefix}_back_rod_end" pos="0 0 {f6(-l3)}" size="0.004"/>
        </body>
      </body>"""

    return f"""<mujoco model="arbatos_wheelleg_fivebar">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.003" integrator="RK4" gravity="0 0 -9.81"/>

  <default>
    <joint armature="0.0002"/>
    <geom condim="4" friction="1.2 0.04 0.002" solref="0.01 1"/>
    <motor ctrllimited="true"/>
  </default>

  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.18 0.19 0.20" rgb2="0.23 0.24 0.25"/>
    <material name="floor_mat" texture="grid" texrepeat="10 10" reflectance="0.15"/>
    <material name="body_mat" rgba="0.22 0.35 0.50 1"/>
    <material name="right_mat" rgba="0.75 0.28 0.20 1"/>
    <material name="left_mat" rgba="0.24 0.58 0.35 1"/>
    <material name="wheel_mat" rgba="0.04 0.04 0.04 1"/>
  </asset>

  <worldbody>
    <light pos="0 -3 3" dir="0 1 -1" diffuse="0.8 0.8 0.8"/>
    <geom name="floor" type="plane" size="5 5 0.1" material="floor_mat"/>

    <body name="base" pos="0 0 {f6(base_height_m)}">
      {base_joints}
      <site name="imu" pos="0 0 0" size="0.006"/>
      <geom name="body" type="box" pos="0 0 {f6(body_half_z + 0.012)}" size="{f6(body_half_x)} {f6(body_half_y)} {f6(body_half_z)}"
            mass="{f6(body_mass)}" material="body_mat"/>
      <geom name="front_crossbar" type="capsule" fromto="{f6(-half_l5)} -0.100 0 {f6(-half_l5)} 0.100 0"
            size="0.0045" mass="{f6(crossbar_mass)}" material="body_mat" contype="0" conaffinity="0"/>
      <geom name="back_crossbar" type="capsule" fromto="{f6(half_l5)} -0.100 0 {f6(half_l5)} 0.100 0"
            size="0.0045" mass="{f6(crossbar_mass)}" material="body_mat" contype="0" conaffinity="0"/>
{side_xml("right", -track_half, "right_mat")}
{side_xml("left", track_half, "left_mat")}
    </body>
  </worldbody>

  <equality>
    <connect name="right_front_loop" site1="right_front_rod_end" site2="right_foot_site" solref="0.004 1"/>
    <connect name="right_back_loop" site1="right_back_rod_end" site2="right_foot_site" solref="0.004 1"/>
    <connect name="left_front_loop" site1="left_front_rod_end" site2="left_foot_site" solref="0.004 1"/>
    <connect name="left_back_loop" site1="left_back_rod_end" site2="left_foot_site" solref="0.004 1"/>
  </equality>

  <actuator>
    <motor name="right_front_motor" joint="right_front_hinge" ctrlrange="-3 3"/>
    <motor name="right_back_motor" joint="right_back_hinge" ctrlrange="-3 3"/>
    <motor name="right_wheel_motor" joint="right_wheel_hinge" ctrlrange="-0.45 0.45"/>
    <motor name="left_front_motor" joint="left_front_hinge" ctrlrange="-3 3"/>
    <motor name="left_back_motor" joint="left_back_hinge" ctrlrange="-3 3"/>
    <motor name="left_wheel_motor" joint="left_wheel_hinge" ctrlrange="-0.45 0.45"/>
  </actuator>
</mujoco>
"""


def resolve_model_path(args: argparse.Namespace, config: BridgeConfig) -> Path:
    if args.model is not None:
        validate_model_xml(args.model)
        return args.model

    validate_fivebar_config(config)
    length = clamped_leg_length(config, args.target_leg)
    base_height = length + float(config.wheel_radius_m) + 0.002
    mode = base_mode(args)
    model_path = generated_model_path(Path(args.native_dir), args.project, mode, args.leg_branch)
    model_path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = model_path.with_name(model_path.name + ".tmp")
    tmp_path.write_text(build_fivebar_model_xml(config, base_height, mode), encoding="utf-8")
    os.replace(tmp_path, model_path)
    validate_model_xml(model_path)
    return model_path


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


def angle_from_down(dx: float, dz: float) -> float:
    return math.atan2(-dx, -dz)


def wrap_pi(value: float) -> float:
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def fivebar_points(config: BridgeConfig, front_pos: float, back_pos: float) -> dict[str, tuple[float, float]]:
    l1 = float(config.geometry.l1_m)
    l2 = float(config.geometry.l2_m)
    l3 = float(config.geometry.l3_m)
    l4 = float(config.geometry.l4_m)
    l5 = float(config.geometry.l5_m)
    phi1 = math.pi * 0.5 + front_pos
    phi4 = math.pi * 0.5 + back_pos
    xb = l1 * math.cos(phi1)
    yb = l1 * math.sin(phi1)
    xd = l5 + l4 * math.cos(phi4)
    yd = l4 * math.sin(phi4)
    lbd = math.hypot(xd - xb, yd - yb)
    a0 = 2.0 * l2 * (xd - xb)
    b0 = 2.0 * l2 * (yd - yb)
    c0 = l2 * l2 + lbd * lbd - l3 * l3
    discr = a0 * a0 + b0 * b0 - c0 * c0
    if discr < 0.0 or abs(a0 + c0) < 1.0e-9:
        raise RuntimeError("home five-bar pose is not reachable")
    phi2 = 2.0 * math.atan2(b0 + math.sqrt(max(0.0, discr)), a0 + c0)
    xc = xb + l2 * math.cos(phi2)
    yc = yb + l2 * math.sin(phi2)
    half_l5 = l5 * 0.5
    return {
        "front_elbow": (xb - half_l5, -yb),
        "back_elbow": (xd - half_l5, -yd),
        "foot": (xc - half_l5, -yc),
    }


def inverse_fivebar_point(
    config: BridgeConfig,
    foot_x_m: float,
    foot_y_m: float,
    front_ref: float = -math.pi,
    back_ref: float = -math.pi,
    prefer_outward: bool = True,
) -> tuple[float, float]:
    l1 = float(config.geometry.l1_m)
    l2 = float(config.geometry.l2_m)
    l3 = float(config.geometry.l3_m)
    l4 = float(config.geometry.l4_m)
    l5 = float(config.geometry.l5_m)
    cx = l5 * 0.5 + float(foot_x_m)
    cy = float(foot_y_m)
    front_r = math.hypot(cx, cy)
    back_dx = cx - l5
    back_r = math.hypot(back_dx, cy)
    if front_r <= 0.001 or back_r <= 0.001:
        raise RuntimeError("initial five-bar target is too close to a pivot")

    front_cos = (l1 * l1 + front_r * front_r - l2 * l2) / (2.0 * l1 * front_r)
    back_cos = (l4 * l4 + back_r * back_r - l3 * l3) / (2.0 * l4 * back_r)
    if front_cos > 1.0001 or front_cos < -1.0001 or back_cos > 1.0001 or back_cos < -1.0001:
        raise RuntimeError("initial five-bar target is not reachable")

    front_cos = max(-1.0, min(1.0, front_cos))
    back_cos = max(-1.0, min(1.0, back_cos))
    front_base = math.atan2(cy, cx)
    back_base = math.atan2(cy, back_dx)
    front_q = math.acos(front_cos)
    back_q = math.acos(back_cos)
    front_candidates = [
        front_ref + wrap_pi(front_base + front_q - math.pi * 0.5 - front_ref),
        front_ref + wrap_pi(front_base - front_q - math.pi * 0.5 - front_ref),
    ]
    back_candidates = [
        back_ref + wrap_pi(back_base + back_q - math.pi * 0.5 - back_ref),
        back_ref + wrap_pi(back_base - back_q - math.pi * 0.5 - back_ref),
    ]

    best: tuple[float, float] | None = None
    best_score = float("inf")
    outward_best: tuple[float, float] | None = None
    outward_best_score = float("inf")
    for front in front_candidates:
        for back in back_candidates:
            points = fivebar_points(config, front, back)
            foot = points["foot"]
            err = abs(foot[0] - foot_x_m) + abs(-foot[1] - foot_y_m)
            if err > 0.003:
                continue
            score = abs(wrap_pi(front - front_ref)) + abs(wrap_pi(back - back_ref)) + err * 1000.0
            outward = points["front_elbow"][0] < -l5 * 0.5 and points["back_elbow"][0] > l5 * 0.5
            candidate = (wrap_pi(front), wrap_pi(back))
            if outward and (outward_best is None or score < outward_best_score):
                outward_best = candidate
                outward_best_score = score
            if best is None or score < best_score:
                best = candidate
                best_score = score
    if prefer_outward and outward_best is not None:
        return outward_best
    if best is None:
        raise RuntimeError("initial five-bar target has no matching branch")
    return best


def joint_exists(mujoco, model, name: str) -> bool:
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) >= 0


def set_joint_qpos_if_exists(mujoco, model, data, name: str, value: float) -> None:
    if joint_exists(mujoco, model, name):
        data.joint(name).qpos[0] = value


def initialize_leg_pose(mujoco, model, data, prefix: str, config: BridgeConfig, front_pos: float, back_pos: float) -> None:
    points = fivebar_points(config, front_pos, back_pos)
    front_elbow = points["front_elbow"]
    back_elbow = points["back_elbow"]
    foot = points["foot"]
    front_rod_world = angle_from_down(foot[0] - front_elbow[0], foot[1] - front_elbow[1])
    back_rod_world = angle_from_down(foot[0] - back_elbow[0], foot[1] - back_elbow[1])

    set_joint_qpos_if_exists(mujoco, model, data, f"{prefix}_front_hinge", front_pos)
    set_joint_qpos_if_exists(mujoco, model, data, f"{prefix}_back_hinge", back_pos)
    set_joint_qpos_if_exists(mujoco, model, data, f"{prefix}_front_knee", wrap_pi(front_rod_world - front_pos))
    set_joint_qpos_if_exists(mujoco, model, data, f"{prefix}_back_knee", wrap_pi(back_rod_world - back_pos))
    set_joint_qpos_if_exists(mujoco, model, data, f"{prefix}_carrier_x", foot[0])
    set_joint_qpos_if_exists(mujoco, model, data, f"{prefix}_carrier_z", foot[1])


def initialize_model_pose(mujoco, model, data, config: BridgeConfig, front_home: float, back_home: float) -> None:
    initialize_leg_pose(mujoco, model, data, "right", config, front_home, back_home)
    initialize_leg_pose(mujoco, model, data, "left", config, front_home, back_home)


def initial_home_pose(config: BridgeConfig, args: argparse.Namespace) -> tuple[float, float]:
    length = clamped_leg_length(config, args.target_leg)
    return inverse_fivebar_point(config, float(args.target_foot_x), length, prefer_outward=args.leg_branch == "diamond")


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


def base_orientation(mujoco, model, data) -> tuple[float, float, float, float, float, float]:
    if joint_exists(mujoco, model, "base_free"):
        base_qpos = data.joint("base_free").qpos
        base_qvel = data.joint("base_free").qvel
        roll, pitch, yaw = quat_to_euler_wxyz(base_qpos[3:7])
        return roll, pitch, yaw, float(base_qvel[3]), float(base_qvel[4]), float(base_qvel[5])
    yaw = float(data.joint("base_yaw").qpos[0]) if joint_exists(mujoco, model, "base_yaw") else 0.0
    gyro_z = float(data.joint("base_yaw").qvel[0]) if joint_exists(mujoco, model, "base_yaw") else 0.0
    pitch = float(data.joint("base_pitch").qpos[0]) if joint_exists(mujoco, model, "base_pitch") else 0.0
    gyro_y = float(data.joint("base_pitch").qvel[0]) if joint_exists(mujoco, model, "base_pitch") else 0.0
    if joint_exists(mujoco, model, "base_yaw") or joint_exists(mujoco, model, "base_pitch"):
        return 0.0, pitch, yaw, 0.0, gyro_y, gyro_z
    return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0


def actuator_id(mujoco, model, name: str) -> int:
    idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if idx < 0:
        raise SystemExit(f"MJCF actuator not found: {name}")
    return int(idx)


def fill_bridge_input(mujoco, model, data, args: argparse.Namespace) -> BridgeInput:
    roll, pitch, yaw, gyro_x, gyro_y, gyro_z = base_orientation(mujoco, model, data)
    return BridgeInput(
        dt_s=float(args.dt),
        pitch_rad=float(pitch),
        roll_rad=float(roll),
        yaw_rad=float(yaw),
        gyro_x_radps=float(gyro_x),
        gyro_y_radps=float(gyro_y),
        gyro_z_radps=float(gyro_z),
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
        support_only=1 if args.support_only else 0,
        jump_force_n=float(args.jump_force_n) if time.perf_counter() < float(args.jump_until_perf) else 0.0,
    )


def sim_actuator_torque(core_index: int, output: BridgeOutput, args: argparse.Namespace) -> float:
    signs = SIM_ACTUATOR_SIGNS_CORE if args.leg_branch == "core" else SIM_ACTUATOR_SIGNS_DIAMOND
    torque = float(output.actuator_torque_nm[core_index]) * signs.get(core_index, 1.0)
    if core_index in WHEEL_ACTUATORS:
        torque *= float(args.sim_wheel_scale)
    if args.support_only and core_index in WHEEL_ACTUATORS:
        torque = 0.0
    return torque


def apply_output(mujoco, model, data, actuator_ids: dict[int, int], output: BridgeOutput, args: argparse.Namespace) -> None:
    for core_index, mj_index in actuator_ids.items():
        data.ctrl[mj_index] = sim_actuator_torque(core_index, output, args)


def run_step(lib, config: BridgeConfig, state: BridgeState, mujoco, model, data, args: argparse.Namespace) -> BridgeOutput:
    bridge_input = fill_bridge_input(mujoco, model, data, args)
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


def applied_wheel_torque(output: BridgeOutput, side: int, args: argparse.Namespace) -> float:
    core_index = CORE_ACT_RIGHT_WHEEL if side == SIDE_RIGHT else CORE_ACT_LEFT_WHEEL
    return sim_actuator_torque(core_index, output, args)


def clamp_float(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(value, max_value))


def keyboard_callback(args: argparse.Namespace):
    def on_key(key: int) -> None:
        ch = chr(key).lower() if 0 <= key <= 255 else ""
        changed = True
        if key == 265:
            args.target_v = clamp_float(float(args.target_v) + 0.05, -0.45, 0.45)
        elif key == 264:
            args.target_v = clamp_float(float(args.target_v) - 0.05, -0.45, 0.45)
        elif key == 263:
            args.target_yaw_rate = clamp_float(float(args.target_yaw_rate) + 0.15, -1.20, 1.20)
        elif key == 262:
            args.target_yaw_rate = clamp_float(float(args.target_yaw_rate) - 0.15, -1.20, 1.20)
        elif ch == "q":
            args.target_leg = clamp_float(float(args.target_leg) - 0.005, 0.085, 0.120)
        elif ch == "e":
            args.target_leg = clamp_float(float(args.target_leg) + 0.005, 0.085, 0.120)
        elif ch == "z":
            args.target_foot_x = clamp_float(float(args.target_foot_x) - 0.005, -0.030, 0.030)
        elif ch == "c":
            args.target_foot_x = clamp_float(float(args.target_foot_x) + 0.005, -0.030, 0.030)
        elif ch == "x":
            args.target_v = 0.0
            args.target_yaw_rate = 0.0
            args.target_foot_x = 0.0
        elif ch == " ":
            args.vmc = True
            args.jump_until_perf = time.perf_counter() + float(args.jump_duration_s)
            print(f"jump force={float(args.jump_force_n):.1f}N duration={float(args.jump_duration_s):.2f}s")
        else:
            changed = False
        if changed:
            print(
                "target "
                f"v={float(args.target_v):+.2f}m/s "
                f"yaw_rate={float(args.target_yaw_rate):+.2f}rad/s "
                f"leg={float(args.target_leg):.3f}m "
                f"foot_x={float(args.target_foot_x):+.3f}m"
            )

    return on_key


def run_sim(args: argparse.Namespace) -> None:
    mujoco = import_mujoco()
    lib = load_bridge(Path(args.native_dir), rebuild=not args.no_rebuild)
    config = BridgeConfig()
    lib.arbatos_wheelleg_bridge_config_defaults(ctypes.byref(config))
    load_project_config(config, args.project)
    model_path = resolve_model_path(args, config)

    model = mujoco.MjModel.from_xml_path(str(model_path))
    model.opt.timestep = float(args.dt)
    data = mujoco.MjData(model)
    state = BridgeState()
    lib.arbatos_wheelleg_bridge_state_init(ctypes.byref(state), ctypes.byref(config))

    try:
        front_home_value, back_home_value = initial_home_pose(config, args)
    except RuntimeError:
        front_home = ctypes.c_float()
        back_home = ctypes.c_float()
        if lib.arbatos_wheelleg_bridge_home_pose(
            ctypes.byref(config), ctypes.byref(front_home), ctypes.byref(back_home)
        ) == 0:
            front_home.value = -2.0
            back_home.value = -2.0
        front_home_value = float(front_home.value)
        back_home_value = float(back_home.value)

    initialize_model_pose(mujoco, model, data, config, front_home_value, back_home_value)
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
        output = run_step(lib, config, state, mujoco, model, data, args)
        apply_output(mujoco, model, data, actuator_ids, output, args)
        mujoco.mj_step(model, data)
        if args.print_every > 0 and step_index % args.print_every == 0:
            roll, pitch, _yaw, _gyro_x, _gyro_y, _gyro_z = base_orientation(mujoco, model, data)
            print(
                f"t={data.time:6.3f}s pitch={pitch:+.3f} roll={roll:+.3f} "
                f"target_v={float(args.target_v):+.2f} target_yaw={float(args.target_yaw_rate):+.2f} "
                f"v={output.observer_v_mps:+.3f} "
                f"tau_wheel_r={applied_wheel_torque(output, SIDE_RIGHT, args):+.3f} "
                f"tau_wheel_l={applied_wheel_torque(output, SIDE_LEFT, args):+.3f}"
            )
        if args.realtime:
            elapsed = time.perf_counter() - start
            time.sleep(max(0.0, float(args.dt) - elapsed))

    steps = max(1, int(args.duration_s / args.dt))
    if args.viewer:
        import mujoco.viewer  # type: ignore

        key_callback = keyboard_callback(args) if args.keyboard else None
        with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
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
    parser.add_argument("--model", type=Path, default=None, help="Use a custom MJCF file instead of generated five-bar.")
    parser.add_argument("--native-dir", type=Path, default=DEFAULT_NATIVE_DIR)
    parser.add_argument("--duration-s", type=float, default=5.0)
    parser.add_argument("--dt", type=float, default=0.003)
    parser.add_argument("--target-v", type=float, default=0.0)
    parser.add_argument("--target-leg", type=float, default=0.10)
    parser.add_argument("--target-foot-x", type=float, default=0.0)
    parser.add_argument("--target-yaw-rate", type=float, default=0.0)
    parser.add_argument("--leg-branch", choices=("diamond", "core"), default="diamond")
    parser.add_argument("--vmc", action="store_true", help="Enable VMC joint torque output.")
    parser.add_argument("--sim-wheel-scale", type=float, default=1.0, help="Scale wheel torques applied in MuJoCo.")
    parser.add_argument("--support-only", action="store_true", help="Disable wheel torques and keep only VMC leg support.")
    parser.add_argument("--planar", action="store_true", help="Use a 2D base for pitch-plane standing tests.")
    parser.add_argument("--yaw-plane", action="store_true", help="Lock roll while allowing x/y/z, yaw, and pitch for driving tests.")
    parser.add_argument("--drive-plane", action="store_true", help="Lock roll and pitch while allowing x/y/z/yaw for interactive driving tests.")
    parser.add_argument("--bench", action="store_true", help="Fix the base to inspect PID/VMC output on a virtual bench.")
    parser.add_argument("--keyboard", action="store_true", help="Enable viewer keyboard control: arrows speed/turn, Q/E leg, Z/C foot, X stop, Space jump.")
    parser.add_argument("--jump-force-n", type=float, default=55.0, help="Extra VMC support force while Space jump is active.")
    parser.add_argument("--jump-duration-s", type=float, default=0.16, help="Space jump force duration.")
    parser.add_argument("--print-params", action="store_true", help="Print project and MuJoCo model parameter sources.")
    parser.add_argument("--viewer", action="store_true", help="Open the MuJoCo viewer.")
    parser.add_argument("--realtime", action="store_true", help="Sleep to roughly match wall time.")
    parser.add_argument("--print-every", type=int, default=100)
    parser.add_argument("--no-rebuild", action="store_true")
    parser.add_argument("--check", action="store_true", help="Validate files and config without importing MuJoCo.")
    return parser.parse_args(list(argv))


def main(argv: Iterable[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    args.jump_until_perf = 0.0
    if args.support_only:
        args.vmc = True
    config = BridgeConfig()
    load_project_config(config, args.project)
    model_path = resolve_model_path(args, config)
    body = read_wheelleg_config_body(args.project)
    if args.print_params:
        print_model_params(args, config, model_path)
        return 0
    if args.check:
        lqr_rows = robot_sim.extract_initializer(body, "lqr_poly")
        row_count = len(robot_sim.split_top_level(lqr_rows or ""))
        print(f"MuJoCo wheelleg check ok: project={args.project} model={model_path} lqr_rows={row_count}")
        return 0
    run_sim(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
