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
            break

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


def _read_current_header(f) -> tuple[bytes, int, int]:
    hdr0 = f.read(16)
    if len(hdr0) != 16:
        raise ValueError("File too small for sdlog header")

    magic, header_size, file_flags, boot_tick_ms, _reserved = struct.unpack("<IHHII", hdr0)
    if magic != SDLOG_FILE_MAGIC:
        raise ValueError(f"Bad sdlog magic 0x{magic:08X}")
    if header_size < 16:
        raise ValueError(f"Bad sdlog header_size {header_size}")
    if file_flags != 0:
        raise ValueError(f"Unsupported sdlog file flags 0x{file_flags:04X}")

    extra = b""
    if header_size > 16:
        extra = f.read(header_size - 16)
        if len(extra) != (header_size - 16):
            raise ValueError("Truncated sdlog header extension")
    return hdr0 + extra, header_size, boot_tick_ms


def decompress_file(in_path: str, out_path: str) -> None:
    with open(in_path, "rb") as f, open(out_path, "wb") as out:
        header, _header_size, _boot_tick_ms = _read_current_header(f)
        out.write(header)

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

            calc = zlib.crc32(raw) & 0xFFFFFFFF
            if (flags & SDLOG_BLOCK_FLAG_CRC32) != 0 and calc != stored_crc32:
                raise ValueError(f"CRC32 mismatch (calc 0x{calc:08X} stored 0x{stored_crc32:08X})")

            out.write(
                struct.pack(
                    "<IHHIII",
                    SDLOG_BLOCK_MAGIC,
                    SDLOG_BLOCK_FLAG_CRC32,
                    20,
                    len(raw),
                    len(raw),
                    calc,
                )
            )
            out.write(raw)


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description="Strip LZ4 compression from current sdlog files.")
    ap.add_argument("input", nargs="+", help="Input sdlog_XXXX.bin")
    ap.add_argument("-o", "--output", help="Output file (only valid with a single input)")
    args = ap.parse_args(argv)

    if args.output and len(args.input) != 1:
        ap.error("--output requires a single input file")

    for in_path in args.input:
        out_path = args.output
        if not out_path:
            out_path = in_path + ".uncompressed.bin"

        decompress_file(in_path, out_path)
        sys.stderr.write(f"ok: {in_path} -> {out_path}\n")

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
