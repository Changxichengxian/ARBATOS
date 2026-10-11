"""验证参考修正遇到原控制后段错误时只回滚它临时改动的参考和增益。"""
import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np


def build_library(snapshot, directory, manifest):
    """在新目录编译专用测试库；所有控制源码和头文件来自指定快照。"""
    directory = directory.resolve()
    directory.mkdir(parents=True, exist_ok=False)
    sys.path.insert(0, str(snapshot / "tools/mujoco/wheelleg"))
    from RunWheelLeg import find_compiler
    compiler = find_compiler()
    if compiler is None:
        raise RuntimeError("未找到主机 C 编译器")
    executable, tags = compiler
    harness = Path(__file__).with_name("TestWbrReferenceFailureBridge.c")
    source = directory / harness.name
    source.write_bytes(harness.read_bytes())
    library = directory / ("wbr_reference_failure.dll" if sys.platform == "win32"
                           else "wbr_reference_failure.so")
    includes = [snapshot / name for name in ("tools/mujoco/learning",
        "shared/application/wheelleg", "shared/application/robot", "shared/components/support")]
    if "msvc" in tags:
        command = [executable, "/nologo", "/LD", "/O2"] + [f"/I{p}" for p in includes]
        command += [str(source), f"/Fe{library}"]
    else:
        command = [executable, "cc"] if "zig" in tags else [executable]
        command += ["-shared", "-O2", "-std=c99"] + [f"-I{p}" for p in includes]
        command += [str(source), "-o", str(library)]
        if sys.platform != "win32":
            command += ["-fPIC", "-lm"]
    subprocess.run(command, cwd=directory, check=True)
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    record = dict(snapshot=str(snapshot), snapshot_library_sha256=manifest["library_sha256"],
        source_hashes={name: digest(snapshot / name) for name in manifest["files"]},
        harness_sha256=digest(source), test_script_sha256=digest(Path(__file__)),
        library_sha256=digest(library), build_command=command)
    (directory / "build.json").write_text(
        json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return library


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", required=True, type=Path)
    library_input = parser.add_mutually_exclusive_group(required=True)
    library_input.add_argument("--library", type=Path,
                        help="包含 TestWbrInjectLateFailure/TestWbrReadTuningState 的专用测试库")
    library_input.add_argument("--build-dir", type=Path,
                               help="在尚不存在的目录编译专用库，然后运行测试")
    args = parser.parse_args()
    snapshot = args.snapshot.resolve()
    sys.path.insert(0, str(snapshot / "tools/mujoco/learning"))
    from WbrLearningEnv import WbrLearningEnv, verify_snapshot

    manifest = verify_snapshot()
    library_path = (build_library(snapshot, args.build_dir, manifest) if args.build_dir
                    else args.library.resolve())
    if not library_path.is_file():
        raise FileNotFoundError(library_path)
    env = WbrLearningEnv(randomize=False, strict=True)
    handles = []
    checks = {}
    lib = None

    def require(value, message):
        if not value:
            raise AssertionError(message)

    try:
        env.reset(seed=101, options=dict(case="stop", speed=.5, push=12., delay=0,
                                         duration=8.))
        lib = ctypes.CDLL(str(library_path))
        ptr = ctypes.POINTER(ctypes.c_float)
        lib.TestWbrCreate.argtypes = [ptr]
        lib.TestWbrCreate.restype = ctypes.c_void_p
        lib.TestWbrDestroy.argtypes = [ctypes.c_void_p]
        lib.TestWbrDestroy.restype = None
        lib.TestWbrSetGasSpring.argtypes = [ctypes.c_void_p, ctypes.c_float]
        lib.TestWbrSetJointLimits.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float]
        lib.TestWbrCommandStep.argtypes = [ctypes.c_void_p, ptr, ptr, ptr]
        lib.TestWbrReferenceStep.argtypes = [ctypes.c_void_p, ptr, ptr, ptr, ptr]
        lib.TestWbrSplitVelocityStep.argtypes = [ctypes.c_void_p, ptr, ptr, ptr, ptr]
        lib.TestWbrInjectLateFailure.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float]
        lib.TestWbrReadTuningState.argtypes = [ctypes.c_void_p, ptr]
        for function in (lib.TestWbrSetGasSpring, lib.TestWbrSetJointLimits,
                         lib.TestWbrCommandStep, lib.TestWbrReferenceStep,
                         lib.TestWbrSplitVelocityStep, lib.TestWbrInjectLateFailure,
                         lib.TestWbrReadTuningState):
            function.restype = ctypes.c_int

        profile = env.profile
        config = (ctypes.c_float * 20)(
            *np.asarray(profile["negative_k"]).flatten(), profile["theta0"],
            profile["tp0"], profile["wheel0"], profile["support_n"], .2,
            profile["radius_m"], profile["length_kp"], profile["length_kd"])

        def make_handle():
            handle = lib.TestWbrCreate(config)
            require(handle, "创建测试控制器失败")
            handles.append(handle)
            require(lib.TestWbrSetGasSpring(handle, 450.) == 1, "气簧配置失败")
            require(lib.TestWbrSetJointLimits(handle, 20., 40.) == 1, "电机配置失败")
            require(lib.TestWbrInjectLateFailure(handle, .4, .125) == 1,
                    "注入后段故障失败")
            return handle

        command = (ctypes.c_float * 3)(0., 0., 0.)
        output = (ctypes.c_float * 40)()
        state = (ctypes.c_float * 14)()

        # 原控制在返回错误前已经推进参考，证明故障点位于参考更新之后。
        witness = make_handle()
        require(lib.TestWbrReadTuningState(witness, state) == 1, "读取原始状态失败")
        before = bytes(state)
        before_reference = (state[0], state[1])
        output[:] = [123.] * 40
        witness_code = lib.TestWbrCommandStep(witness, env.sensor, command, output)
        require(witness_code != 0 and not np.any(output), "未触发清零的原控制后段错误")
        require(lib.TestWbrReadTuningState(witness, state) == 1, "读取错误后状态失败")
        require(state[0] != before_reference[0] and state[1] != before_reference[1] and
                bytes(state) != before, "故障发生在速度/位置参考更新之前，测试条件无效")
        checks["failure_occurs_after_reference_update"] = True

        for label, function, action_values in (
                ("reference", lib.TestWbrReferenceStep, (1., .5, .5)),
                ("split_velocity", lib.TestWbrSplitVelocityStep, (1., .5, -.5))):
            handle = make_handle()
            require(lib.TestWbrReadTuningState(handle, state) == 1, "读取保存状态失败")
            saved = bytes(state)
            action = (ctypes.c_float * 3)(*action_values)
            output[:] = [123.] * 40
            code = function(handle, env.sensor, command, action, output)
            require(code == witness_code, f"{label}没有传播同一后段错误")
            require(not np.any(output), f"{label}错误输出未清零")
            require(lib.TestWbrReadTuningState(handle, state) == 1, "读取恢复状态失败")
            require(bytes(state) == saved, f"{label}未恢复v_ref、x_ref或k")
            checks[label + "_restores_v_ref_x_ref_k_and_clears_output"] = True

        print(json.dumps(dict(checks=checks, witness_error_code=witness_code,
                              scope="只检查新增参考和增益状态；不声称回滚原控制全部内部状态"),
                         ensure_ascii=False, indent=2))
    finally:
        if lib is not None:
            for handle in handles:
                lib.TestWbrDestroy(handle)
        env.close()


if __name__ == "__main__":
    main()
