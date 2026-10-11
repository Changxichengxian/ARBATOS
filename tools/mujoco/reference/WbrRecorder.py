"""有界异步仿真记录。JSONL为仿真扩展，不能当作实车SDLog二进制文件。"""
from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import csv
import hashlib
import json
import math
from pathlib import Path
from queue import Empty, Full, Queue
import threading
import uuid
import zipfile


class WbrRecorder:
    def __init__(self, directory: Path, sampleHz: float = 100, queueSize: int = 2048):
        if not math.isfinite(sampleHz) or sampleHz <= 0 or queueSize < 1:
            raise ValueError("记录频率和缓存容量必须为正")
        self.directory = Path(directory).resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.period = 1.0 / sampleHz
        self.queueSize = queueSize
        self._lock = threading.RLock()
        self._queue = Queue(maxsize=queueSize)
        self._thread = None
        self._active = False
        self._stopping = False
        self._name = None
        self._completed = set()
        self._error = None
        self._samples = self._dropped = self._written = 0
        self._next = 0.0
        self._last = 0.0
        self._startTime = 0.0
        self._bytes = 0
        self._duration = 0.0

    def start(self, metadata: dict, simTime: float = 0.0) -> dict:
        with self._lock:
            if self._active or self._stopping:
                return self.status()
            self._name = datetime.now(timezone.utc).strftime("wbr-%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:8] + ".jsonl"
            self._error = None
            self._samples = self._dropped = self._written = 0
            self._bytes = 0
            self._duration = 0.0
            self._queue = Queue(maxsize=self.queueSize)
            self._next = self._last = float(simTime)
            self._startTime = float(simTime)
            self._active = True
            header = dict(type="metadata", format="arbatos-wbr-sim", version=1,
                          hardware=False, sample_hz=1.0/self.period,
                          created_utc=datetime.now(timezone.utc).isoformat(), parameters=deepcopy(metadata))
            self._thread = threading.Thread(target=self._write, args=(self._name, header), daemon=True)
            self._thread.start()
            self.event("start", simTime)
            return self.status()

    def _put(self, record: dict) -> bool:
        try:
            self._queue.put_nowait(record)
            return True
        except Full:
            self._dropped += 1
            return False

    def event(self, name: str, simTime: float, detail: dict | None = None) -> bool:
        with self._lock:
            if not self._active:
                return False
            return self._put(dict(type="event", event=str(name), sim_time_s=float(simTime),
                                  detail=deepcopy(detail or {})))

    def sample(self, simTime: float, frame: dict) -> bool:
        with self._lock:
            if not self._active:
                return False
            now = float(simTime)
            if not math.isfinite(now):
                raise ValueError("仿真时间必须为有限数")
            if now < self._last - 1e-9:
                self.event("reset", now, dict(previous_sim_time_s=self._last))
                self._next = now
            self._duration += max(0.0, now - self._last)
            self._last = now
            if now + 1e-9 < self._next:
                return False
            # 按仿真时间采样，暂停期间不重复记录；超出周期不补造中间物理状态。
            self._next = now + self.period
            record = dict(type="sample", seq=self._samples, sim_time_s=now, frame=deepcopy(frame))
            self._samples += 1
            return self._put(record)

    def stop(self, simTime: float | None = None) -> dict:
        with self._lock:
            if self._active:
                self._stopTime = self._last if simTime is None else float(simTime)
                self._active = False
                self._stopping = True
            return self.status()

    def wait(self, timeout: float | None = 5.0) -> dict:
        thread = self._thread
        if thread is not None:
            thread.join(timeout)
        return self.status()

    def status(self) -> dict:
        with self._lock:
            filename = Path(self._name).with_suffix(".zip").name if self._name else None
            return dict(active=self._active, finalizing=self._stopping, file=filename, filename=filename,
                        state="recording" if self._active else "finalizing" if self._stopping else
                              "error" if self._error else "stopped",
                        samples=self._samples, written=self._written, dropped=self._dropped,
                        bytes=self._bytes, duration=self._duration,
                        error=self._error, ready=filename in self._completed,
                        files=sorted(self._completed))

    def downloadPath(self, name: str) -> Path:
        with self._lock:
            # 只允许本实例成功关闭的文件，拒绝路径穿越和读取仍写入的半成品。
            if name not in self._completed or Path(name).name != name:
                raise FileNotFoundError("日志尚未完成或文件名无效")
            path = (self.directory / name).resolve()
            if path.parent != self.directory or not path.is_file():
                raise FileNotFoundError("日志路径无效")
            return path

    def _write(self, name: str, header: dict) -> None:
        try:
            with (self.directory / name).open("x", encoding="utf-8", newline="\n") as stream:
                def emit(record):
                    line = json.dumps(record, ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n"
                    stream.write(line)
                    with self._lock:
                        self._bytes += len(line.encode("utf-8"))
                emit(header)
                while True:
                    try:
                        record = self._queue.get(timeout=0.05)
                    except Empty:
                        with self._lock:
                            if self._stopping and self._queue.empty():
                                break
                        continue
                    emit(record)
                    with self._lock:
                        if record["type"] == "sample":
                            self._written += 1
                emit(dict(type="event", event="stop", sim_time_s=self._stopTime,
                          detail=dict(samples=self._samples, written=self._written, dropped=self._dropped)))
                stream.flush()
            archive = self._export(name, header)
            with self._lock:
                self._bytes = archive.stat().st_size
                self._completed.add(archive.name)
        except Exception as exc:
            with self._lock:
                self._error = str(exc)
                self._active = False
        finally:
            with self._lock:
                self._stopping = False

    def _export(self, name: str, header: dict) -> Path:
        # 导出也在写入线程中完成，物理线程只会提交快照和查询状态。
        jsonl = self.directory / name
        csvPath = jsonl.with_suffix(".csv")
        metadataPath = jsonl.with_suffix(".metadata.json")
        metadataPath.write_text(json.dumps(header, ensure_ascii=False, indent=2), encoding="utf-8")
        def flatten(value, prefix=""):
            result = {}
            if isinstance(value, dict):
                for key, item in value.items():
                    result.update(flatten(item, f"{prefix}.{key}" if prefix else key))
            elif isinstance(value, (tuple, list)):
                for index, item in enumerate(value):
                    result.update(flatten(item, f"{prefix}.{index}"))
            else:
                result[prefix] = value
            return result
        with csvPath.open("w", encoding="utf-8-sig", newline="") as stream:
            writer = None
            for record in WbrReadRecording(jsonl):
                if record["type"] != "sample":
                    continue
                row = dict(seq=record["seq"], sim_time_s=record["sim_time_s"], **flatten(record["frame"]))
                # 保留完整帧，后续阶段出现新增字段时不会因首帧CSV表头漏失数据。
                row["frame_json"] = json.dumps(record["frame"], ensure_ascii=False, separators=(",", ":"))
                if writer is None:
                    writer = csv.DictWriter(stream, fieldnames=list(row), extrasaction="ignore")
                    writer.writeheader()
                writer.writerow(row)
            if writer is None:
                csv.writer(stream).writerow(["seq", "sim_time_s", "frame_json"])
        archive = jsonl.with_suffix(".zip")
        with zipfile.ZipFile(archive, "x", compression=zipfile.ZIP_DEFLATED) as output:
            for path in (jsonl, csvPath, metadataPath):
                output.write(path, arcname=path.name)
        return archive


def WbrSourceMetadata(paths) -> dict:
    return {str(Path(path)): hashlib.sha256(Path(path).read_bytes()).hexdigest() for path in paths}


def WbrReadRecording(path: Path):
    """顺序读取已下载JSONL；frame.qpos/qvel用于精确重建采样时姿态。"""
    with Path(path).open(encoding="utf-8") as stream:
        for line in stream:
            yield json.loads(line)
