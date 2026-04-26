#!/usr/bin/env python3
from __future__ import annotations

import argparse
import struct
import sys
import zlib


SDLOG_FILE_MAGIC = 0x474C4453  # 'SDLG'
SDLOG_BLOCK_MAGIC = 0x4B424453  # 'SDBK'
SDLOG_BLOCK_FLAG_COMPRESSED = 0x0001
SDLOG_BLOCK_FLAG_CRC32 = 0x0002


def _try_read_var_u32(buf: bytearray, off: int) -> tuple[int, int] | None:
    v = 0
    shift = 0
    i = off
    while True:
        if i >= len(buf):
            return None
        b = buf[i]
        i += 1
        v |= (b & 0x7F) << shift
        if (b & 0x80) == 0:
            break
        shift += 7
        if shift > 35:
            raise ValueError("varint too long")
    return v, i


class V3RecordStreamParser:
    def __init__(self, boot_tick_ms: int) -> None:
        self._buf = bytearray()
        self._tick_ms = int(boot_tick_ms)

    def feed(self, data: bytes) -> list[tuple[int, int, bytes]]:
        out: list[tuple[int, int, bytes]] = []
        self._buf.extend(data)
        while True:
            off = 0
            r = _try_read_var_u32(self._buf, off)
            if r is None:
                return out
            dt_ms, off = r

            r = _try_read_var_u32(self._buf, off)
            if r is None:
                return out
            tag, off = r

            r = _try_read_var_u32(self._buf, off)
            if r is None:
                return out
            payload_len, off = r

            if len(self._buf) < off + payload_len:
                return out

            payload = bytes(self._buf[off : off + payload_len])
            del self._buf[: off + payload_len]
            self._tick_ms += dt_ms
            out.append((self._tick_ms, tag, payload))


def lz4_decompress_block(src: bytes, raw_len: int) -> bytes:
    out = bytearray()
    i = 0

    while i < len(src):
        token = src[i]
        i += 1

        lit_len = token >> 4
        if lit_len == 15:
            while True:
                if i >= len(src):
                    raise ValueError("LZ4: truncated literal length")
                s = src[i]
                i += 1
                lit_len += s
                if s != 255:
                    break

        if i + lit_len > len(src):
            raise ValueError("LZ4: truncated literals")
        if lit_len:
            out.extend(src[i : i + lit_len])
            i += lit_len

        if i >= len(src):
            break  # last literals

        if i + 2 > len(src):
            raise ValueError("LZ4: truncated offset")
        offset = src[i] | (src[i + 1] << 8)
        i += 2
        if offset == 0 or offset > len(out):
            raise ValueError(f"LZ4: invalid offset {offset}")

        match_len = token & 0x0F
        if match_len == 15:
            while True:
                if i >= len(src):
                    raise ValueError("LZ4: truncated match length")
                s = src[i]
                i += 1
                match_len += s
                if s != 255:
                    break
        match_len += 4

        copy_start = len(out) - offset
        while match_len:
            out.append(out[copy_start])
            copy_start += 1
            match_len -= 1

    if len(out) != raw_len:
        raise ValueError(f"LZ4: raw_len mismatch (got {len(out)} expected {raw_len})")
    return bytes(out)


def decompress_file(in_path: str, out_path: str) -> None:
    with open(in_path, "rb") as f:
        hdr0 = f.read(16)
        if len(hdr0) != 16:
            raise ValueError("File too small for sdlog header")

        magic, version, header_size, boot_tick_ms, reserved = struct.unpack("<IHHII", hdr0)
        if magic != SDLOG_FILE_MAGIC:
            raise ValueError(f"Bad sdlog magic 0x{magic:08X}")
        if header_size < 16:
            raise ValueError(f"Bad sdlog header_size {header_size}")
        if header_size > 16:
            extra = f.read(header_size - 16)
            if len(extra) != (header_size - 16):
                raise ValueError("Truncated sdlog header extension")

        if version == 1:
            f.seek(0)
            with open(out_path, "wb") as out:
                while True:
                    chunk = f.read(1024 * 1024)
                    if not chunk:
                        break
                    out.write(chunk)
            return

        if version not in (2, 3):
            raise ValueError(f"Unsupported sdlog version {version}")

        with open(out_path, "wb") as out:
            out.write(struct.pack("<IHHII", SDLOG_FILE_MAGIC, 1, 16, boot_tick_ms, reserved))

            v3_parser = V3RecordStreamParser(boot_tick_ms) if version == 3 else None

            while True:
                bh = f.read(20)
                if not bh:
                    break
                if len(bh) != 20:
                    raise ValueError("Truncated sdlog block header")

                bmagic, flags, bhsz, raw_len, data_len, stored_crc32 = struct.unpack("<IHHIII", bh)
                if bmagic != SDLOG_BLOCK_MAGIC:
                    raise ValueError(f"Bad block magic 0x{bmagic:08X}")
                if bhsz < 20:
                    raise ValueError(f"Bad block header_size {bhsz}")
                if bhsz > 20:
                    extra = f.read(bhsz - 20)
                    if len(extra) != (bhsz - 20):
                        raise ValueError("Truncated block header extension")

                data = f.read(data_len)
                if len(data) != data_len:
                    raise ValueError("Truncated block data")

                if (flags & SDLOG_BLOCK_FLAG_COMPRESSED) != 0:
                    raw = lz4_decompress_block(data, raw_len)
                else:
                    if len(data) != raw_len:
                        raise ValueError(f"Raw block length mismatch (got {len(data)} expected {raw_len})")
                    raw = data

                if (flags & SDLOG_BLOCK_FLAG_CRC32) != 0:
                    calc = zlib.crc32(raw) & 0xFFFFFFFF
                    if calc != stored_crc32:
                        raise ValueError(f"CRC32 mismatch (calc 0x{calc:08X} stored 0x{stored_crc32:08X})")

                if version == 2:
                    out.write(raw)
                else:
                    assert v3_parser is not None
                    for tick_ms, tag, payload in v3_parser.feed(raw):
                        tick_ms_u32 = tick_ms & 0xFFFFFFFF
                        tag_u16 = tag & 0xFFFF
                        payload_len = len(payload)
                        if payload_len > 0xFFFF:
                            raise ValueError(f"v3 payload too large for v1: {payload_len}")
                        out.write(struct.pack("<IHH", tick_ms_u32, tag_u16, payload_len))
                        out.write(payload)
                        pad = (-payload_len) & 3
                        if pad:
                            out.write(b"\x00" * pad)


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description="Decompress HERO sdlog v2/v3 (*.bin) into v1 record stream (*.raw.bin).")
    ap.add_argument("input", nargs="+", help="Input sdlog_XXXX.bin")
    ap.add_argument("-o", "--output", help="Output file (only valid with a single input)")
    args = ap.parse_args(argv)

    if args.output and len(args.input) != 1:
        ap.error("--output requires a single input file")

    for in_path in args.input:
        out_path = args.output
        if not out_path:
            out_path = in_path + ".raw.bin"

        decompress_file(in_path, out_path)
        sys.stderr.write(f"ok: {in_path} -> {out_path}\n")

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
