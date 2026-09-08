"""客户端上传文件的本地转换服务。

不接受路径或命令参数：页面只能提交文件名和 Base64 内容，所有格式定义复用
tools/sdlog 与副板音乐当前实现。结果只留在本次 RPC 响应中，由浏览器下载。
"""

from __future__ import annotations

import base64
import csv
import importlib.util
import io
import json
import math
import shutil
import struct
import subprocess
import sys
import tempfile
import wave
import zlib
import zipfile
from pathlib import Path


MAX_INPUT_BYTES = 24 * 1024 * 1024
MAX_OUTPUT_BYTES = 24 * 1024 * 1024
MAX_CURVE_POINTS = 1200
MAX_CURVES = 128
MAX_RECORDS = 500_000
MAX_UNKNOWN_KINDS = 20
MAX_DECODED_BYTES = 64 * 1024 * 1024
FFMPEG_TIMEOUT_SECONDS = 30
WAV_SUFFIXES = {".wav", ".wave"}
FFMPEG_SUFFIXES = {".mp3", ".aac", ".m4a", ".flac", ".ogg", ".opus"}


def _load_viewer(root: Path):
    spec = importlib.util.spec_from_file_location("arbatos_sdlog_viewer", root / "tools/sdlog/SdLogViewer.py")
    if spec is None or spec.loader is None:
        raise ValueError("找不到 SD 日志解析器")
    module = importlib.util.module_from_spec(spec)
    # SdLogViewer 的 dataclass 需要通过模块名找到自身。
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ConverterService:
    def __init__(self, root: Path):
        self.root = Path(root).resolve()
        self.viewer = _load_viewer(self.root)
        bundled = self.root / "tools/Mp3ToU8/ffmpeg.exe"
        self.ffmpeg = shutil.which("ffmpeg")
        self.mp3_only = not self.ffmpeg and sys.platform == "win32" and bundled.is_file()
        if self.mp3_only:
            self.ffmpeg = str(bundled)
        self.audio_suffixes = {".mp3"} if self.mp3_only else FFMPEG_SUFFIXES

    def dispatch(self, method, params):
        if not isinstance(params, dict):
            raise ValueError("请求参数必须是对象")
        handlers = {
            "tools.capabilities": self.capabilities,
            "tools.sdlog": self.sdlog,
            "tools.audio": self.audio,
        }
        handler = handlers.get(method)
        if handler is None:
            raise ValueError(f"未知工具方法: {method}")
        expected = set() if method == "tools.capabilities" else {"name", "data", "options"}
        if set(params) - expected:
            raise ValueError("请求包含不支持的参数")
        return handler(**params)

    def capabilities(self):
        return {
            "maxInputBytes": MAX_INPUT_BYTES,
            "audio": {
                "output": "无文件头、12 kHz、单声道、无符号 8 位 PCM (.u8)",
                "wav": True,
                "ffmpeg": bool(self.ffmpeg),
                "ffmpegSuffixes": sorted(self.audio_suffixes) if self.ffmpeg else [],
            },
            "sdlog": {"extensions": [".bin", ".lz4"], "maxCurvePoints": MAX_CURVE_POINTS},
        }

    @staticmethod
    def _file(name, data):
        if not isinstance(name, str) or not name or len(name) > 120:
            raise ValueError("文件名不能为空且不能超过 120 个字符")
        if Path(name).name != name or name in {".", ".."} or any(char in name for char in '/\\:') or any(ord(char) < 32 for char in name):
            raise ValueError("文件名不能包含路径或控制字符")
        if not isinstance(data, str):
            raise ValueError("文件内容必须是 Base64 字符串")
        if len(data) > ((MAX_INPUT_BYTES + 2) // 3) * 4:
            raise ValueError("文件超过 24 MiB 限制")
        try:
            raw = base64.b64decode(data, validate=True)
        except (ValueError, TypeError) as exc:
            raise ValueError("文件内容不是有效的 Base64") from exc
        if not raw:
            raise ValueError("文件不能为空")
        if len(raw) > MAX_INPUT_BYTES:
            raise ValueError("文件超过 24 MiB 限制")
        return name, raw

    @staticmethod
    def _download(name, mime, data):
        if len(data) > MAX_OUTPUT_BYTES:
            raise ValueError("转换结果超过 24 MiB，不能通过客户端下载")
        return {"name": name, "mime": mime, "data": base64.b64encode(data).decode("ascii"), "bytes": len(data)}

    @staticmethod
    def _downsample(ticks, values):
        def finite_number(value):
            return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)

        if len(ticks) <= MAX_CURVE_POINTS:
            return [[tick, value] for tick, value in zip(ticks, values) if finite_number(value)]
        step = (len(ticks) + MAX_CURVE_POINTS - 1) // MAX_CURVE_POINTS
        return [
            [tick, value]
            for tick, value in zip(ticks[::step], values[::step])
            if finite_number(value)
        ]

    def _dataset(self, name, raw):
        stream = io.BytesIO(raw)
        header = stream.read(16)
        if len(header) != 16:
            raise ValueError("SD 日志文件头不完整")
        magic, header_size, flags, boot_tick_ms, _reserved = struct.unpack("<IHHII", header)
        if magic != self.viewer.SDLOG_FILE_MAGIC:
            raise ValueError("不是当前 SD 日志格式（缺少 SDLG 文件头）")
        if header_size < 16 or flags != 0:
            raise ValueError("不支持的 SD 日志文件头")
        if len(stream.read(header_size - 16)) != header_size - 16:
            raise ValueError("SD 日志扩展文件头不完整")
        dataset = self.viewer.Dataset(source_path=name, boot_tick_ms=boot_tick_ms, file_header_size=header_size)
        parser = self.viewer.RecordStreamParser(boot_tick_ms)
        blocks = 0
        compressed = 0
        records = 0
        decoded_bytes = 0
        while True:
            block_header = stream.read(20)
            if not block_header:
                break
            if len(block_header) != 20:
                raise ValueError("SD 日志数据块头不完整")
            magic, block_flags, block_header_size, raw_len, data_len, stored_crc32 = struct.unpack(
                "<IHHIII", block_header
            )
            if magic != self.viewer.SDLOG_BLOCK_MAGIC or block_header_size < 20:
                raise ValueError("SD 日志数据块格式错误")
            if len(stream.read(block_header_size - 20)) != block_header_size - 20:
                raise ValueError("SD 日志数据块扩展头不完整")
            if raw_len > MAX_INPUT_BYTES or data_len > MAX_INPUT_BYTES:
                raise ValueError("SD 日志数据块超过允许大小")
            decoded_bytes += raw_len
            if decoded_bytes > MAX_DECODED_BYTES:
                raise ValueError("日志解压后超过 64 MiB，请使用独立日志解析工具处理")
            if block_flags & ~(self.viewer.SDLOG_BLOCK_FLAG_COMPRESSED | self.viewer.SDLOG_BLOCK_FLAG_CRC32):
                raise ValueError("SD 日志包含不支持的数据块标志")
            encoded = stream.read(data_len)
            if len(encoded) != data_len:
                raise ValueError("SD 日志数据块不完整")
            if block_flags & self.viewer.SDLOG_BLOCK_FLAG_COMPRESSED:
                decoded = self.viewer.lz4_decompress_block(encoded, raw_len)
                compressed += 1
            else:
                if len(encoded) != raw_len:
                    raise ValueError("未压缩 SD 日志数据块长度不匹配")
                decoded = encoded
            if block_flags & self.viewer.SDLOG_BLOCK_FLAG_CRC32:
                if (zlib.crc32(decoded) & 0xFFFFFFFF) != stored_crc32:
                    raise ValueError("SD 日志数据块 CRC32 校验失败")
            for tick_ms, tag, payload in parser.feed(decoded):
                records += 1
                if records > MAX_RECORDS:
                    raise ValueError("SD 日志记录数超过客户端解析上限")
                dataset.add_record(tick_ms, tag, payload)
            blocks += 1
        if parser._buf:
            raise ValueError("SD 日志最后一条记录不完整")
        return dataset, blocks, compressed, records

    def sdlog(self, name, data, options=None):
        if options not in (None, {}):
            raise ValueError("SD 日志暂不接受额外转换选项")
        name, raw = self._file(name, data)
        if Path(name).suffix.lower() not in {".bin", ".lz4"}:
            raise ValueError("SD 日志只接受 .bin 或 .lz4 文件")
        dataset, blocks, compressed, records = self._dataset(name, raw)
        # 多组日志按组导出宽表，避免每个字段重复整行名称让几 MB 日志膨胀成上百 MB。
        if len(dataset.series) > 1:
            archive = io.BytesIO()
            with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as bundle:
                for index, (key, series) in enumerate(sorted(dataset.series.items())):
                    safe_key = "".join(char if char.isascii() and (char.isalnum() or char in "-_") else "_" for char in key)
                    with bundle.open(f"{index + 1:02d}_{safe_key[:80]}.csv", "w") as entry:
                        with io.TextIOWrapper(entry, encoding="utf-8-sig", newline="") as text:
                            writer = csv.writer(text)
                            names = sorted(series.fields)
                            writer.writerow(["tick_ms", *names])
                            for row, tick in enumerate(series.ticks_ms):
                                writer.writerow([tick, *(series.fields[field][row] for field in names)])
                    if archive.tell() > MAX_OUTPUT_BYTES:
                        raise ValueError("日志导出超过 24 MiB，请使用独立日志解析工具处理")
            export_data = archive.getvalue()
            export_name, export_mime = f"{Path(name).stem}.zip", "application/zip"
        else:
            csv_text = io.StringIO(newline="")
            writer = csv.writer(csv_text)
            writer.writerow(["series", "tick_ms", "field", "value"])
            for key, series in sorted(dataset.series.items()):
                for field in sorted(series.fields):
                    for tick, value in zip(series.ticks_ms, series.fields[field]):
                        writer.writerow([key, tick, field, value])
                        if csv_text.tell() > MAX_OUTPUT_BYTES:
                            raise ValueError("CSV 超过 24 MiB，请使用独立日志解析工具处理")
            export_data = csv_text.getvalue().encode("utf-8")
            export_name, export_mime = f"{Path(name).stem}.csv", "text/csv;charset=utf-8"
        curves = []
        for key, series in sorted(dataset.series.items()):
            if len(dataset.series) > 1 and len(series.ticks_ms) < 2:
                continue
            numeric = [
                field for field, values in sorted(series.fields.items())
                if any(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
                       for value in values)
            ]
            for field in numeric:
                curves.append({"key": key, "name": series.name, "field": field,
                               "points": self._downsample(series.ticks_ms, series.fields[field])})
                if len(curves) >= MAX_CURVES:
                    break
            if len(curves) >= MAX_CURVES:
                break
        unknown = json.loads(dataset.tag_index_json().decode("utf-8"))["unknown"]
        result = self._download(export_name, export_mime, export_data)
        result.update({
            "format": "当前 SDLOG（SDLG/SDBK，已校验 LZ4 和 CRC32）",
            "inputBytes": len(raw), "blocks": blocks, "compressedBlocks": compressed, "records": records,
            "series": [{"key": key, "name": s.name, "count": len(s.ticks_ms), "fields": sorted(s.fields)}
                       for key, s in sorted(dataset.series.items())],
            "unknown": unknown[:MAX_UNKNOWN_KINDS],
            "unknownKinds": len(unknown),
            "unknownRecords": len(dataset.unknown_records),
            "curves": curves,
        })
        return result

    @staticmethod
    def _wav_to_u8(raw):
        try:
            with wave.open(io.BytesIO(raw), "rb") as source:
                if source.getcomptype() != "NONE":
                    raise ValueError("WAV 必须是未压缩 PCM")
                channels = source.getnchannels()
                width = source.getsampwidth()
                rate = source.getframerate()
                frames = source.getnframes()
                if channels not in {1, 2} or width not in {1, 2, 3, 4} or rate <= 0 or rate > 48000:
                    raise ValueError("WAV 只支持 1/2 声道、8/16/24/32 位、最高 48 kHz 的 PCM")
                samples = source.readframes(frames)
        except wave.Error as exc:
            raise ValueError("WAV 文件无法解码") from exc
        frame_size = channels * width
        if frames == 0:
            raise ValueError("WAV 文件没有音频数据")
        if len(samples) != frames * frame_size:
            raise ValueError("WAV 音频数据不完整")

        def sample_at(offset):
            value = int.from_bytes(samples[offset:offset + width], "little", signed=width != 1)
            return value - 128 if width == 1 else value / (1 << (width * 8 - 8))

        output_frames = max(1, round(frames * 12000 / rate))
        if output_frames > MAX_OUTPUT_BYTES:
            raise ValueError("音频转换结果超过 24 MiB")

        def mono_at(frame):
            offset = frame * frame_size
            value = sum(sample_at(offset + channel * width) for channel in range(channels)) / channels
            return max(-128.0, min(127.0, value))

        output = bytearray(output_frames)
        for index in range(output_frames):
            source_position = index * rate / 12000
            left = min(int(source_position), frames - 1)
            right = min(left + 1, frames - 1)
            fraction = source_position - left
            first = mono_at(left)
            value = first + (mono_at(right) - first) * fraction
            output[index] = max(0, min(255, round(value + 128)))
        return bytes(output)

    def _ffmpeg_to_u8(self, raw):
        if not self.ffmpeg:
            raise ValueError("本机未检测到 ffmpeg；请上传 WAV，或安装 ffmpeg 后再转换该格式")
        if self.mp3_only:
            # 仓库精简版只编入 file 协议；强制 MP3 解复用，不允许上传内容变成播放列表。
            with tempfile.TemporaryDirectory(prefix="arbatos-audio-") as directory:
                source = Path(directory) / "source.mp3"
                output = Path(directory) / "output.u8"
                source.write_bytes(raw)
                args = [self.ffmpeg, "-nostdin", "-hide_banner", "-loglevel", "error",
                        "-protocol_whitelist", "file", "-f", "mp3", "-i", str(source),
                        "-t", str(MAX_OUTPUT_BYTES / 12000 + 1), "-vn", "-ac", "1", "-ar", "12000",
                        "-af", "acompressor=threshold=-18dB:ratio=2:attack=5:release=120:makeup=6,alimiter=limit=0.95",
                        "-c:a", "pcm_u8", "-f", "u8", "-fs", str(MAX_OUTPUT_BYTES + 1), str(output)]
                self._run_ffmpeg(args)
                if not output.is_file() or output.stat().st_size > MAX_OUTPUT_BYTES:
                    raise ValueError("音频输出为空或超过 24 MiB")
                return output.read_bytes()
        # 禁止解码器跟随上传内容引用网络/本地文件，并在读取输出前限制最长时长。
        args = [self.ffmpeg, "-nostdin", "-hide_banner", "-loglevel", "error", "-protocol_whitelist", "pipe",
                "-i", "pipe:0", "-t", str(MAX_OUTPUT_BYTES / 12000 + 1), "-vn", "-ac", "1", "-ar", "12000",
                "-af", "acompressor=threshold=-18dB:ratio=2:attack=5:release=120:makeup=6,alimiter=limit=0.95",
                "-c:a", "pcm_u8", "-f", "u8", "pipe:1"]
        return self._run_ffmpeg(args, raw).stdout

    @staticmethod
    def _run_ffmpeg(args, raw=None):
        try:
            result = subprocess.run(args, input=raw, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    timeout=FFMPEG_TIMEOUT_SECONDS, check=False,
                                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        except subprocess.TimeoutExpired as exc:
            raise ValueError("音频转换超过 30 秒，已停止") from exc
        if result.returncode != 0:
            message = result.stderr.decode("utf-8", errors="replace").strip()
            raise ValueError(f"ffmpeg 无法解码该音频：{message[:240] or '未知错误'}")
        return result

    def audio(self, name, data, options=None):
        if options not in (None, {}):
            raise ValueError("音频转换不接受自定义 ffmpeg 参数")
        name, raw = self._file(name, data)
        suffix = Path(name).suffix.lower()
        if suffix in WAV_SUFFIXES and (not self.ffmpeg or self.mp3_only):
            output = self._wav_to_u8(raw)
            source = "WAV PCM 本机转换"
        elif suffix in WAV_SUFFIXES | self.audio_suffixes:
            output = self._ffmpeg_to_u8(raw)
            source = "ffmpeg 本机解码"
        else:
            raise ValueError("当前解码工具不支持此格式；仓库内置版支持 MP3 和 PCM WAV，其他格式需要完整 ffmpeg")
        result = self._download(f"{Path(name).stem}.u8", "application/octet-stream", output)
        result.update({
            "source": source,
            "format": "无文件头、12 kHz、单声道、无符号 8 位 PCM",
            "sampleRateHz": 12000,
            "durationSeconds": round(len(output) / 12000, 3),
        })
        return result
