"""运行真实MuJoCo步进，验证记录、停止、归档下载和姿态回放。"""
from __future__ import annotations

import csv
import io
import json
from pathlib import Path
import tempfile
import zipfile

import mujoco
import numpy as np

from WbrGasSpring import load_model
from WbrMotorTelemetry import WbrMotorTelemetry
from WbrRecorder import WbrReadRecording, WbrRecorder, WbrSourceMetadata


def main() -> int:
    model = load_model(correct_link_collisions=True)
    model.opt.timestep = 0.001
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    telemetry = WbrMotorTelemetry(model)
    # 对称转矩与角速度反号后机械功率仍相同，避免右侧坐标方向造成符号误读。
    data.ctrl[:] = [1, 2, 0.3, -1, -2, -0.3]
    for (_, _, dof, _) in telemetry.axes:
        data.qvel[dof] = 1
    data.qvel[telemetry.axes[5][2]] = -1
    mujoco.mj_forward(model, data)
    motors = telemetry.snapshot(data)
    assert abs(motors["wheels"][0]["mechanical_power_w"] - 0.3) < 1e-9
    assert abs(motors["wheels"][1]["mechanical_power_w"] - 0.3) < 1e-9
    assert motors["hips"][0]["torque_nm"] == 1
    assert any(abs(data.qfrc_passive) > 0), "需要真实气弹簧被动力以检验隔离"
    data.ctrl[2] = -0.3
    mujoco.mj_forward(model, data)
    assert telemetry.snapshot(data)["wheels"][0]["braking"]

    with tempfile.TemporaryDirectory(prefix="arbatos-wbr-record-") as folder:
        recorder = WbrRecorder(Path(folder))
        metadata = dict(motor=telemetry.metadata(), sources=WbrSourceMetadata([Path(__file__)]))
        recorder.start(metadata, float(data.time))
        for _ in range(1000):
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            recorder.sample(float(data.time), dict(qpos=data.qpos.tolist(), qvel=data.qvel.tolist(),
                                                 ctrl=data.ctrl.tolist(), motor=telemetry.snapshot(data),
                                                 contacts=int(data.ncon), input=dict(speed=0)))
        before = recorder.status()["samples"]
        recorder.event("reset", float(data.time), dict(reason="test"))
        mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
        mujoco.mj_forward(model, data)
        recorder.sample(0, dict(qpos=data.qpos.tolist(), qvel=data.qvel.tolist(), ctrl=data.ctrl.tolist(),
                               motor=telemetry.snapshot(data), contacts=int(data.ncon), input=dict(speed=0)))
        recorder.stop(float(data.time))
        status = recorder.wait()
        assert status["ready"] and not status["finalizing"] and not status["error"], status
        assert status["dropped"] == 0 and status["written"] == before + 1, status
        assert 95 <= before <= 101, before
        assert not recorder.sample(0, {})
        archive = recorder.downloadPath(status["filename"])
        try:
            recorder.downloadPath("../outside.jsonl")
        except FileNotFoundError:
            pass
        else:
            raise AssertionError("路径穿越未拒绝")
        with zipfile.ZipFile(archive) as bundle:
            assert len(bundle.namelist()) == 3
            csvName = next(name for name in bundle.namelist() if name.endswith(".csv"))
            rows = list(csv.DictReader(io.StringIO(bundle.read(csvName).decode("utf-8-sig"))))
            assert len(rows) == status["written"]
            jsonlName = next(name for name in bundle.namelist() if name.endswith(".jsonl"))
            records = [json.loads(line) for line in bundle.read(jsonlName).decode("utf-8").splitlines()]
        assert records[0]["hardware"] is False and records[-1]["event"] == "stop"
        frames = [item for item in records if item["type"] == "sample"]
        replay = mujoco.MjData(model)
        for item in frames[::10]:
            frame = item["frame"]
            replay.qpos[:] = frame["qpos"]
            replay.qvel[:] = frame["qvel"]
            replay.ctrl[:] = frame["ctrl"]
            mujoco.mj_forward(model, replay)
            actual = telemetry.snapshot(replay)
            assert np.allclose([row["torque_nm"] for row in actual["hips"]],
                               [row["torque_nm"] for row in frame["motor"]["hips"]])
        assert len(list(WbrReadRecording(Path(folder) / jsonlName))) == len(records)
        assert status["duration"] > 0.99
        # 用真实异步写入线程制造缓存饱和，停止事件仍由线程在末尾独立写入。
        overflow = WbrRecorder(Path(folder), queueSize=1)
        overflow.start(dict(purpose="overflow"))
        for index in range(10000):
            overflow.event("input", index * .001, dict(speed=index))
        overflow.stop(10)
        overflowStatus = overflow.wait()
        assert overflowStatus["ready"] and overflowStatus["dropped"] > 0, overflowStatus
        overflowPath = overflow.downloadPath(overflowStatus["filename"])
        with zipfile.ZipFile(overflowPath) as bundle:
            name = next(name for name in bundle.namelist() if name.endswith(".jsonl"))
            last = json.loads(bundle.read(name).decode("utf-8").splitlines()[-1])
            assert last["event"] == "stop" and last["detail"]["dropped"] == overflowStatus["dropped"]
        print(json.dumps(dict(checks="passed", samples=status["written"], dropped=status["dropped"],
                              zip_bytes=status["bytes"], replay_frames=len(frames[::10]),
                              overflow_dropped=overflowStatus["dropped"]), ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
