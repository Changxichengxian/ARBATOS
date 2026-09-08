import base64
import importlib.util
import struct
import unittest
import wave
import zipfile
from unittest import mock
from io import BytesIO
from pathlib import Path


REPO = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("converters", REPO / "client/backend/converters.py")
CONVERTERS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CONVERTERS)


def encoded(value):
    return base64.b64encode(value).decode("ascii")


def varint(value):
    out = bytearray()
    while value >= 128:
        out.append((value & 127) | 128)
        value >>= 7
    out.append(value)
    return bytes(out)


class ConverterTest(unittest.TestCase):
    def setUp(self):
        self.service = CONVERTERS.ConverterService(REPO)

    def wav(self):
        output = BytesIO()
        with wave.open(output, "wb") as source:
            source.setnchannels(2)
            source.setsampwidth(2)
            source.setframerate(24000)
            source.writeframes(struct.pack("<8h", -32768, -32768, -16384, -16384, 0, 0, 16384, 16384))
        return output.getvalue()

    def test_wav_becomes_12khz_unsigned_u8(self):
        self.service.ffmpeg = None
        result = self.service.dispatch("tools.audio", {"name": "tone.wav", "data": encoded(self.wav())})
        data = base64.b64decode(result["data"])
        self.assertEqual(result["sampleRateHz"], 12000)
        self.assertEqual(result["name"], "tone.u8")
        self.assertEqual(len(data), 2)
        self.assertNotEqual(data, b"\x80\x80")

    def test_invalid_name_and_extra_options_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "路径"):
            self.service.dispatch("tools.audio", {"name": "../tone.wav", "data": encoded(self.wav())})
        with self.assertRaisesRegex(ValueError, "参数"):
            self.service.dispatch("tools.audio", {"name": "tone.wav", "data": encoded(self.wav()), "options": {"x": 1}})

    def test_sdlog_decodes_real_current_block_layout_and_csv(self):
        payload = struct.pack("<ff", 24.5, 76.0)
        record = varint(10) + varint(4) + varint(len(payload)) + payload
        crc = __import__("zlib").crc32(record) & 0xffffffff
        raw = struct.pack("<IHHII", 0x474C4453, 16, 0, 100, 0)
        raw += struct.pack("<IHHIII", 0x4B424453, 2, 20, len(record), len(record), crc) + record
        result = self.service.dispatch("tools.sdlog", {"name": "sdlog_0001.bin", "data": encoded(raw)})
        self.assertEqual(result["blocks"], 1)
        self.assertTrue(result["curves"])
        csv_data = base64.b64decode(result["data"]).decode("utf-8")
        self.assertIn("BATTERY", csv_data)
        self.assertIn("24.5", csv_data)

    def test_sdlog_rejects_invalid_magic(self):
        with self.assertRaisesRegex(ValueError, "SDLG"):
            raw = struct.pack("<IHHII", 0x12345678, 16, 0, 0, 0)
            self.service.dispatch("tools.sdlog", {"name": "bad.bin", "data": encoded(raw)})

    def test_multiple_series_export_full_wide_csv_in_zip(self):
        payloads = [(4, struct.pack("<ff", 24.5, 76)), (5, struct.pack("<HBB3f", 1, 0, 0, 3, 2, 1))]
        record = b"".join(varint(10) + varint(tag) + varint(len(payload)) + payload for tag, payload in payloads)
        raw = struct.pack("<IHHII", 0x474C4453, 16, 0, 100, 0)
        raw += struct.pack("<IHHIII", 0x4B424453, 2, 20, len(record), len(record),
                           __import__("zlib").crc32(record) & 0xffffffff) + record
        result = self.service.sdlog("multi.bin", encoded(raw))
        self.assertEqual(result["mime"], "application/zip")
        with zipfile.ZipFile(BytesIO(base64.b64decode(result["data"]))) as archive:
            self.assertIsNone(archive.testzip())
            self.assertEqual(len(archive.namelist()), 2)
            battery = archive.read(next(name for name in archive.namelist() if "BATTERY" in name)).decode("utf-8-sig")
            self.assertIn("tick_ms,percent,voltage", battery)
            self.assertIn("110,76.0,24.5", battery)

    def test_bundled_mp3_decoder_still_uses_pcm_wav_reader(self):
        self.service.mp3_only = True
        self.service.ffmpeg = "bundled-ffmpeg"
        self.service.audio_suffixes = {".mp3"}
        with mock.patch.object(CONVERTERS.subprocess, "run", side_effect=AssertionError("WAV sent to MP3 decoder")):
            result = self.service.audio("tone.wav", encoded(self.wav()))
        self.assertEqual(result["source"], "WAV PCM 本机转换")
        self.assertEqual(self.service.capabilities()["audio"]["ffmpegSuffixes"], [".mp3"])

    def test_lz4_expansion_cannot_exceed_declared_size(self):
        with self.assertRaisesRegex(ValueError, "exceed"):
            self.service.viewer.lz4_decompress_block(b"\x1fA\x01\x00\xff\x00", 8)

    def test_wav_output_size_checked_before_allocation(self):
        output = BytesIO()
        with wave.open(output, "wb") as source:
            source.setnchannels(1)
            source.setsampwidth(1)
            source.setframerate(1)
            source.writeframes(b"\x80" * 2200)
        self.service.ffmpeg = None
        with self.assertRaisesRegex(ValueError, "超过"):
            self.service.dispatch("tools.audio", {"name": "slow.wav", "data": encoded(output.getvalue())})

    def test_ffmpeg_cannot_open_network_or_files_from_uploaded_audio(self):
        self.service.ffmpeg = "ffmpeg"
        self.service.mp3_only = False
        with mock.patch.object(CONVERTERS.subprocess, "run", return_value=mock.Mock(returncode=0, stdout=b"\x80", stderr=b"")) as run:
            self.service.dispatch("tools.audio", {"name": "tone.mp3", "data": encoded(b"test")})
        args = run.call_args.args[0]
        self.assertEqual(args[args.index("-protocol_whitelist") + 1], "pipe")
        self.assertIn("-t", args)

    def test_minimal_decoder_forces_mp3_and_cleans_its_temp_files(self):
        self.service.mp3_only = True
        self.service.ffmpeg = "bundled-ffmpeg"
        def convert(args, **kwargs):
            self.assertEqual(args[args.index("-protocol_whitelist") + 1], "file")
            self.assertEqual(args[args.index("-f") + 1], "mp3")
            self.assertIn("-fs", args)
            Path(args[-1]).write_bytes(b"\x80\x90")
            return mock.Mock(returncode=0, stdout=b"", stderr=b"")
        with mock.patch.object(CONVERTERS.subprocess, "run", side_effect=convert) as run:
            result = self.service.audio("tone.mp3", encoded(b"test"))
        self.assertEqual(result["bytes"], 2)
        self.assertFalse(Path(run.call_args.args[0][-1]).parent.exists())


if __name__ == "__main__":
    unittest.main()
