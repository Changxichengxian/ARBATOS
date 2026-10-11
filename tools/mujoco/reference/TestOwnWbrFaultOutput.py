"""回归检查 OwnWbrCommandStep 的故障返回和恢复辨认零输出约束。"""
from __future__ import annotations

import argparse
import ctypes
import hashlib
import json
import math
from pathlib import Path
import sys

import mujoco
import numpy as np

from RunOwnWbr import load_core
from RunWbr import MOTORS, ROOT, SOURCE


HERE = Path(__file__).resolve().parent
CONTROLLER_SOURCE = HERE / "OwnWbrBridge.c"
FLOAT_POINTER = ctypes.POINTER(ctypes.c_float)
Float3 = ctypes.c_float * 3
Float20 = ctypes.c_float * 20
Float28 = ctypes.c_float * 28
Float40 = ctypes.c_float * 40

SENSOR_NAMES = (
    [f"imu_quat.{axis}" for axis in "wxyz"]
    + [f"imu_gyro.{axis}" for axis in "xyz"]
    + [f"imu_accel.{axis}" for axis in "xyz"]
    + [f"{motor}.{kind}" for motor in MOTORS for kind in ("pos", "vel", "tor")]
)
COMMAND_NAMES = ["speed_mps", "yaw_rate_radps", "jump"]
FAULT_VALUES = [("nan", math.nan), ("+inf", math.inf), ("-inf", -math.inf)]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def bind(lib) -> None:
    bindings = {
        "OwnWbrCreate": ([FLOAT_POINTER], ctypes.c_void_p),
        "OwnWbrDestroy": ([ctypes.c_void_p], None),
        "OwnWbrCommandStep": (
            [ctypes.c_void_p, FLOAT_POINTER, FLOAT_POINTER, FLOAT_POINTER],
            ctypes.c_int,
        ),
        "OwnWbrSetStage": ([ctypes.c_void_p, ctypes.c_int], ctypes.c_int),
        "OwnWbrSetJointLimits": (
            [ctypes.c_void_p, ctypes.c_float, ctypes.c_float],
            ctypes.c_int,
        ),
        "OwnWbrSetGasSpring": ([ctypes.c_void_p, ctypes.c_float], ctypes.c_int),
        "OwnWbrSetJumpProfile": ([ctypes.c_void_p, ctypes.c_int], ctypes.c_int),
        "OwnWbrSetDriveMode": ([ctypes.c_void_p, ctypes.c_int], ctypes.c_int),
        "OwnWbrSetLegLength": ([ctypes.c_void_p, ctypes.c_float], ctypes.c_int),
        "OwnWbrEnableRise": ([ctypes.c_void_p, ctypes.c_int], ctypes.c_int),
        "OwnWbrRequestRecovery": ([ctypes.c_void_p], ctypes.c_int),
    }
    for name, (arguments, result) in bindings.items():
        function = getattr(lib, name)
        function.argtypes = arguments
        function.restype = result


def load_inputs(profile_path: Path) -> tuple[dict, Float20, list[float]]:
    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    config_values = list(np.asarray(profile["negative_k"], dtype=float).flatten())
    config_values += [
        profile["theta0"],
        profile["tp0"],
        profile["wheel0"],
        profile["support_n"],
        profile["length_m"],
        profile["radius_m"],
        profile.get("length_kp", 1000.0),
        profile.get("length_kd", 60.0),
    ]
    model = mujoco.MjModel.from_xml_path(str(SOURCE / "mjcf/scene.xml"))
    data = mujoco.MjData(model)
    data.qpos[:] = profile["qpos"]
    mujoco.mj_forward(model, data)
    names = ["imu_quat", "imu_gyro", "imu_accel"]
    names += [f"{motor}_{kind}" for motor in MOTORS for kind in ("pos", "vel", "tor")]
    indices = np.concatenate(
        [
            np.arange(model.sensor(name).adr[0], model.sensor(name).adr[0] + model.sensor(name).dim[0])
            for name in names
        ]
    )
    sensor = [float(value) for value in data.sensordata[indices]]
    if len(config_values) != 20 or len(sensor) != 28 or len(SENSOR_NAMES) != 28:
        raise RuntimeError("配置或传感器维数不符")
    return profile, Float20(*config_values), sensor


def create_handle(lib, config: Float20, length_m: float, recovery: bool) -> int:
    handle = lib.OwnWbrCreate(config)
    if not handle:
        raise RuntimeError("控制器初始化失败")
    configured = [
        lib.OwnWbrSetStage(handle, 3),
        lib.OwnWbrSetJointLimits(handle, 10.0, 45.0),
        lib.OwnWbrSetGasSpring(handle, 450.0),
        lib.OwnWbrSetJumpProfile(handle, 2),
        lib.OwnWbrSetDriveMode(handle, 0),
        lib.OwnWbrSetLegLength(handle, length_m),
    ]
    if not all(configured) or (recovery and not lib.OwnWbrRequestRecovery(handle)):
        lib.OwnWbrDestroy(handle)
        raise RuntimeError("控制器测试参数设置失败")
    return handle


def call_step(lib, handle, sensor_values, command_values, sentinel: float = 123.0):
    sensor = None if sensor_values is None else Float28(*sensor_values)
    command = None if command_values is None else Float3(*command_values)
    output = Float40(*([sentinel] * 40))
    status = lib.OwnWbrCommandStep(handle, sensor, command, output)
    return status, [float(value) for value in output]


def prepare_runtime(lib, config: Float20, length_m: float, state: str,
                    home_sensor: list[float]) -> int:
    recovery = state in ("recovery_balance", "recovery")
    handle = create_handle(lib, config, length_m, recovery)
    sensor = list(home_sensor)
    if state == "recovery":
        half_sqrt = math.sqrt(0.5)
        sensor[:4] = [half_sqrt, half_sqrt, 0.0, 0.0]
    if state == "home_rise" and not lib.OwnWbrEnableRise(handle, 1):
        lib.OwnWbrDestroy(handle)
        raise RuntimeError("home_rise 起身请求失败")
    expected_phase = {"normal": 0, "home_rise": 2,
                      "recovery_balance": 15, "recovery": 12}[state]
    # R 先经过零输出落稳等待；故障注入须确实进入被测动作阶段。
    for _ in range(600):
        status, output = call_step(lib, handle, sensor, [0.0, 0.0, 0.0], sentinel=0.0)
        phase = round(output[34])
        if status != 0:
            break
        if phase == expected_phase:
            break
        if phase < 0:
            break
    if status != 0 or phase != expected_phase:
        lib.OwnWbrDestroy(handle)
        raise RuntimeError(f"{state} 状态准备失败：status={status}, phase={output[34]}")
    return handle


def probe_runtime_states(lib, config: Float20, length_m: float,
                         home_sensor: list[float]) -> dict[str, int]:
    expected = {
        "normal": 0,
        "home_rise": 2,
        "recovery_balance": 15,
        "recovery": 12,
    }
    observed = {}
    for state, expected_phase in expected.items():
        handle = prepare_runtime(lib, config, length_m, state, home_sensor)
        sensor = list(home_sensor)
        if state == "recovery":
            half_sqrt = math.sqrt(0.5)
            sensor[:4] = [half_sqrt, half_sqrt, 0.0, 0.0]
        try:
            status, output = call_step(lib, handle, sensor, [0.0, 0.0, 0.0], sentinel=0.0)
        finally:
            lib.OwnWbrDestroy(handle)
        phase = round(output[34])
        if status != 0 or phase != expected_phase:
            raise RuntimeError(
                f"{state} 状态探针失败：status={status}, phase={output[34]}, "
                f"expected={expected_phase}"
            )
        observed[state] = phase
    return observed


def add_failure(failures: list[dict], **details) -> None:
    failures.append(details)


def test_runtime_faults(lib, config: Float20, length_m: float,
                        home_sensor: list[float], failures: list[dict]) -> int:
    cases = 0
    for state in ("normal", "home_rise", "recovery_balance", "recovery"):
        for input_kind, names in (("sensor", SENSOR_NAMES), ("command", COMMAND_NAMES)):
            for index, name in enumerate(names):
                for value_name, value in FAULT_VALUES:
                    handle = prepare_runtime(lib, config, length_m, state, home_sensor)
                    sensor = list(home_sensor)
                    if state == "recovery":
                        half_sqrt = math.sqrt(0.5)
                        sensor[:4] = [half_sqrt, half_sqrt, 0.0, 0.0]
                    command = [0.0, 0.0, 0.0]
                    if input_kind == "sensor":
                        sensor[index] = value
                    else:
                        command[index] = value
                    try:
                        status, output = call_step(lib, handle, sensor, command)
                    finally:
                        lib.OwnWbrDestroy(handle)
                    cases += 1
                    if status == 0 or any(item != 0.0 for item in output):
                        add_failure(
                            failures,
                            group="runtime_fault",
                            state=state,
                            input_kind=input_kind,
                            index=index,
                            name=name,
                            value=value_name,
                            status=status,
                            nonzero_output_indices=[i for i, item in enumerate(output) if item != 0.0],
                        )
    return cases


def test_recovery_identification(lib, config: Float20, length_m: float,
                                 home_sensor: list[float], failures: list[dict]) -> int:
    cases = 0
    for index, name in enumerate(SENSOR_NAMES):
        for value_name, value in FAULT_VALUES:
            handle = create_handle(lib, config, length_m, recovery=True)
            sensor = list(home_sensor)
            sensor[index] = value
            try:
                status, output = call_step(lib, handle, sensor, [0.0, 0.0, 0.0])
            finally:
                lib.OwnWbrDestroy(handle)
            cases += 1
            finite = all(math.isfinite(item) for item in output)
            motors_zero = all(item == 0.0 for item in output[:6])
            if status != 0 or round(output[34]) != -11 or not finite or not motors_zero:
                add_failure(
                    failures,
                    group="recovery_identification",
                    input_kind="sensor",
                    index=index,
                    name=name,
                    value=value_name,
                    status=status,
                    phase=output[34] if math.isfinite(output[34]) else "nonfinite",
                    all_output_finite=finite,
                    nonzero_motor_indices=[i for i, item in enumerate(output[:6]) if item != 0.0],
                )
    return cases


def test_null_inputs(lib, config: Float20, length_m: float,
                     home_sensor: list[float], failures: list[dict]) -> int:
    cases = 0
    status, output = call_step(lib, None, home_sensor, [0.0, 0.0, 0.0])
    cases += 1
    if status == 0 or any(item != 0.0 for item in output):
        add_failure(
            failures,
            group="null_input",
            name="handle",
            status=status,
            nonzero_output_indices=[i for i, item in enumerate(output) if item != 0.0],
        )

    handle = create_handle(lib, config, length_m, recovery=False)
    try:
        status, output = call_step(lib, handle, None, [0.0, 0.0, 0.0])
    finally:
        lib.OwnWbrDestroy(handle)
    cases += 1
    if status == 0 or any(item != 0.0 for item in output):
        add_failure(
            failures,
            group="null_input",
            name="sensor",
            status=status,
            nonzero_output_indices=[i for i, item in enumerate(output) if item != 0.0],
        )
    return cases


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profile",
        type=Path,
        default=HERE / "OwnWbrProfile.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "local/cache/wbr-fault-output",
    )
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)

    profile, config, home_sensor = load_inputs(args.profile.resolve())
    hash_before_build = sha256(CONTROLLER_SOURCE)
    lib, build_command = load_core(args.output)
    hash_after_build = sha256(CONTROLLER_SOURCE)
    if hash_before_build != hash_after_build:
        raise RuntimeError("编译期间控制器源码发生变化，请重新运行")
    bind(lib)

    failures: list[dict] = []
    state_probes = probe_runtime_states(lib, config, profile["length_m"], home_sensor)
    runtime_cases = test_runtime_faults(
        lib, config, profile["length_m"], home_sensor, failures
    )
    identification_cases = test_recovery_identification(
        lib, config, profile["length_m"], home_sensor, failures
    )
    null_cases = test_null_inputs(
        lib, config, profile["length_m"], home_sensor, failures
    )
    total_cases = runtime_cases + identification_cases + null_cases
    hash_after_test = sha256(CONTROLLER_SOURCE)
    summary = {
        "passed": not failures and hash_after_test == hash_before_build,
        "controller_source": str(CONTROLLER_SOURCE),
        "controller_sha256": hash_before_build,
        "source_unchanged_during_run": hash_after_test == hash_before_build,
        "build_command": [str(item) for item in build_command],
        "state_probes": state_probes,
        "case_counts": {
            "runtime_nonfinite": runtime_cases,
            "recovery_identification_nonfinite_sensor": identification_cases,
            "null_input": null_cases,
            "total": total_cases,
            "passed": total_cases - len(failures),
            "failed": len(failures),
        },
        "failures": failures,
    }
    summary_path = args.output / "fault_output_summary.json"
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    print(f"summary={summary_path}")
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
