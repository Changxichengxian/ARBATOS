#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import io
import json
import struct
import sys
import threading
import urllib.parse
import webbrowser
import zlib
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Iterable


SDLOG_FILE_MAGIC = 0x474C4453  # 'SDLG'
SDLOG_BLOCK_MAGIC = 0x4B424453  # 'SDBK'
SDLOG_BLOCK_FLAG_COMPRESSED = 0x0001
SDLOG_BLOCK_FLAG_CRC32 = 0x0002


TAG_NAMES: dict[int, str] = {
    0x0000: "META",
    0x0001: "IMU",
    0x0002: "RC_CRSF",
    0x0003: "ACTUATOR_CURRENT",
    0x0004: "BATTERY",
    0x0005: "PID",
    0x0010: "GIMBAL_LOOP",
    0x0011: "CHASSIS_LOOP",
    0x0030: "CAN_RX",
    0x0031: "APP_WATCH",
    0x0032: "DETECT_STATUS",
    0x0033: "CHASSIS_POWER_LIMIT",
    0x0034: "GIMBAL_LIMIT",
    0x0040: "APP_CONFIG",
    0x0041: "SYS_STATS",
    0x0042: "EVENT",
    0x0043: "VISION_USB_RX",
    0x0044: "UART1_TUNE",
    0x0046: "MANUAL_INPUT_RAW",
    0x0047: "IMAGE_LINK_STATS",
    0x0048: "IMU_TRUST",
    0x0049: "CONTROL_SUMMARY",
    0x004A: "PITCH_CALI",
}

PID_NAMES: dict[int, str] = {
    1: "IMU_TEMP",
    10: "GIMBAL_YAW_ANGLE",
    11: "GIMBAL_YAW_SPEED",
    12: "GIMBAL_PITCH_ANGLE",
    13: "GIMBAL_PITCH_SPEED",
    20: "CHASSIS_M1_SPEED",
    21: "CHASSIS_M2_SPEED",
    22: "CHASSIS_M3_SPEED",
    23: "CHASSIS_M4_SPEED",
    24: "CHASSIS_FOLLOW",
    30: "SHOOT_FRIC1_SPEED",
    31: "SHOOT_FRIC2_SPEED",
    32: "SHOOT_FRIC3_SPEED",
    33: "SHOOT_FRIC4_SPEED",
    34: "SHOOT_TRIGGER",
}

PITCH_CALI_STATE_NAMES: dict[int, str] = {
    0: "IDLE",
    1: "WAIT_BULLET",
    2: "MOVE_TO_ANGLE",
    3: "HOLD_AVG",
    4: "BREAKAWAY_UP",
    5: "RECOVER_UP",
    6: "BREAKAWAY_DOWN",
    7: "RECOVER_DOWN",
    8: "SAVE",
    9: "DONE",
    10: "ERROR",
}


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


class RecordStreamParser:
    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> Iterable[tuple[int, int, bytes]]:
        self._buf.extend(data)
        while True:
            if len(self._buf) < 8:
                return
            tick_ms, tag, payload_len = struct.unpack_from("<IHH", self._buf, 0)
            pad = (-payload_len) & 3
            total = 8 + payload_len + pad
            if len(self._buf) < total:
                return
            payload = bytes(self._buf[8 : 8 + payload_len])
            del self._buf[:total]
            yield tick_ms, tag, payload


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


class VarintRecordStreamParser:
    def __init__(self, boot_tick_ms: int) -> None:
        self._buf = bytearray()
        self._tick_ms = int(boot_tick_ms)

    def feed(self, data: bytes) -> Iterable[tuple[int, int, bytes]]:
        self._buf.extend(data)
        while True:
            off = 0
            r = _try_read_var_u32(self._buf, off)
            if r is None:
                return
            dt_ms, off = r

            r = _try_read_var_u32(self._buf, off)
            if r is None:
                return
            tag, off = r

            r = _try_read_var_u32(self._buf, off)
            if r is None:
                return
            payload_len, off = r

            if payload_len < 0:
                raise ValueError("negative payload_len")
            if len(self._buf) < off + payload_len:
                return

            payload = bytes(self._buf[off : off + payload_len])
            del self._buf[: off + payload_len]

            self._tick_ms += dt_ms
            yield self._tick_ms, tag, payload


@dataclass
class Series:
    key: str
    name: str
    ticks_ms: list[int] = field(default_factory=list)
    fields: dict[str, list[Any]] = field(default_factory=dict)

    def add(self, tick_ms: int, values: dict[str, Any]) -> None:
        self.ticks_ms.append(tick_ms)
        row_idx = len(self.ticks_ms) - 1

        for field_name, col in list(self.fields.items()):
            col.append(values.get(field_name))

        for field_name, v in values.items():
            if field_name not in self.fields:
                self.fields[field_name] = [None] * row_idx + [v]


class Dataset:
    def __init__(self, source_path: str, boot_tick_ms: int, file_version: int) -> None:
        self.source_path = source_path
        self.boot_tick_ms = boot_tick_ms
        self.file_version = file_version
        self.series: dict[str, Series] = {}
        self.unknown_tag_counts: dict[int, int] = {}

    def add_record(self, tick_ms: int, tag: int, payload: bytes) -> None:
        extracted = extract_series(tag, payload)
        if extracted is None:
            self.unknown_tag_counts[tag] = self.unknown_tag_counts.get(tag, 0) + 1
            return

        for key, name, values in extracted:
            s = self.series.get(key)
            if s is None:
                s = Series(key=key, name=name)
                self.series[key] = s
            s.add(tick_ms, values)

    def tag_index_json(self) -> bytes:
        tags = []
        for key, s in sorted(self.series.items(), key=lambda kv: kv[1].name):
            tags.append(
                {
                    "key": key,
                    "name": s.name,
                    "count": len(s.ticks_ms),
                    "fields": sorted(s.fields.keys()),
                }
            )
        unknown = []
        for tag, cnt in sorted(self.unknown_tag_counts.items(), key=lambda kv: kv[0]):
            unknown.append({"tag": tag, "tag_name": TAG_NAMES.get(tag, f"0x{tag:04X}"), "count": cnt})

        return json.dumps(
            {
                "source_path": self.source_path,
                "boot_tick_ms": self.boot_tick_ms,
                "file_version": self.file_version,
                "tags": tags,
                "unknown": unknown,
            },
            ensure_ascii=False,
        ).encode("utf-8")


def sdlog_tag_name(tag: int) -> str:
    return TAG_NAMES.get(tag, f"0x{tag:04X}")


def _unpack_exact(fmt: str, payload: bytes) -> tuple[Any, ...] | None:
    size = struct.calcsize(fmt)
    if len(payload) != size:
        return None
    return struct.unpack(fmt, payload)


def _pid_fields_from_snapshot(
    kp: float,
    ki: float,
    kd: float,
    max_out: float,
    max_iout: float,
    set_: float,
    fdb: float,
    out: float,
    pout: float,
    iout: float,
    dout: float,
) -> dict[str, float]:
    return {
        "kp": kp,
        "ki": ki,
        "kd": kd,
        "max_out": max_out,
        "max_iout": max_iout,
        "set": set_,
        "fdb": fdb,
        "out": out,
        "pout": pout,
        "iout": iout,
        "dout": dout,
    }


def extract_series(tag: int, payload: bytes) -> list[tuple[str, str, dict[str, Any]]] | None:
    name = sdlog_tag_name(tag)

    if tag == 0x0000:  # META
        v = _unpack_exact("<III", payload)
        if v is None:
            return None
        boot_tick_ms, heap_free, heap_min_ever_free = v
        return [("META", "META", {"boot_tick_ms": boot_tick_ms, "heap_free": heap_free, "heap_min_ever_free": heap_min_ever_free})]

    if tag == 0x0001:  # IMU
        v = _unpack_exact("<15f", payload)
        if v is not None:
            (
                dt,
                qw,
                qx,
                qy,
                qz,
                angle_yaw,
                angle_roll,
                angle_pitch,
                gyro_x,
                gyro_y,
                gyro_z,
                accel_x,
                accel_y,
                accel_z,
                temp,
            ) = v
            return [
                (
                    name,
                    name,
                    {
                        "dt": dt,
                        "quat_w": qw,
                        "quat_x": qx,
                        "quat_y": qy,
                        "quat_z": qz,
                        "angle_yaw": angle_yaw,
                        "angle_roll": angle_roll,
                        "angle_pitch": angle_pitch,
                        "gyro_x": gyro_x,
                        "gyro_y": gyro_y,
                        "gyro_z": gyro_z,
                        "accel_x": accel_x,
                        "accel_y": accel_y,
                        "accel_z": accel_z,
                        "temp": temp,
                    },
                )
            ]

        v = _unpack_exact("<11f", payload)
        if v is None:
            return None
        qw, qx, qy, qz, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, temp = v
        return [
            (
                name,
                name,
                {
                    "quat_w": qw,
                    "quat_x": qx,
                    "quat_y": qy,
                    "quat_z": qz,
                    "gyro_x": gyro_x,
                    "gyro_y": gyro_y,
                    "gyro_z": gyro_z,
                    "accel_x": accel_x,
                    "accel_y": accel_y,
                    "accel_z": accel_z,
                    "temp": temp,
                },
            )
        ]

    if tag == 0x0002:  # RC_CRSF
        v = _unpack_exact("<16H22B", payload)
        if v is None:
            return None
        fields: dict[str, Any] = {}
        ch = v[:16]
        rc_bytes = v[16:]
        for i, x in enumerate(ch):
            fields[f"ch{i}"] = x
        for i, b in enumerate(rc_bytes):
            fields[f"rc_ctrl_{i:02d}"] = b
        return [(name, name, fields)]

    if tag == 0x0003:  # ACTUATOR_CURRENT
        v = _unpack_exact("<11h", payload)
        if v is None:
            return None
        fields = {f"chassis{i}": v[i] for i in range(4)}
        fields.update(
            {
                "yaw": v[4],
                "pitch": v[5],
                "trigger": v[6],
                "fric0": v[7],
                "fric1": v[8],
                "fric2": v[9],
                "fric3": v[10],
            }
        )
        return [(name, name, fields)]

    if tag == 0x0005:  # PID
        v = _unpack_exact("<HBB17f", payload)
        if v is not None:
            pid_id = int(v[0])
            mode = int(v[1])
            pid_name = PID_NAMES.get(pid_id, f"PID_{pid_id}")
            key = f"PID:{pid_id}"
            disp = f"PID/{pid_name}"

            floats = v[3:]
            (
                kp,
                ki,
                kd,
                max_out,
                max_iout,
                set_,
                fdb,
                out,
                pout,
                iout,
                dout,
                dbuf0,
                dbuf1,
                dbuf2,
                err0,
                err1,
                err2,
            ) = floats
            fields = {"pid_id": pid_id, "mode": mode}
            fields.update(_pid_fields_from_snapshot(kp, ki, kd, max_out, max_iout, set_, fdb, out, pout, iout, dout))
            fields.update(
                {
                    "dbuf0": dbuf0,
                    "dbuf1": dbuf1,
                    "dbuf2": dbuf2,
                    "error0": err0,
                    "error1": err1,
                    "error2": err2,
                }
            )
            return [(key, disp, fields)]

        v = _unpack_exact("<HBB3f", payload)
        if v is None:
            return None
        pid_id, mode, _reserved, set_, fdb, out = v
        pid_id = int(pid_id)
        pid_name = PID_NAMES.get(pid_id, f"PID_{pid_id}")
        key = f"PID:{pid_id}"
        disp = f"PID/{pid_name}"
        return [(key, disp, {"pid_id": pid_id, "mode": int(mode), "set": set_, "fdb": fdb, "out": out})]

    if tag == 0x0004:  # BATTERY
        v = _unpack_exact("<2f", payload)
        if v is None:
            return None
        return [(name, name, {"voltage": v[0], "percent": v[1]})]

    if tag == 0x0010:  # GIMBAL_LOOP
        if len(payload) not in (624, 160):
            return None
        if len(payload) == 624:
            loop_cnt, yaw_cur, pitch_cur, trig_cur, yaw_mode, pitch_mode, shoot_mode, _res = struct.unpack_from("<IhhhBBBB", payload, 0)
            off = 16

            def parse_gimbal_pid(prefix: str, p: bytes, o: int) -> tuple[dict[str, Any], int] | None:
                if o + 52 > len(p):
                    return None
                pid_id, _r, _kp, _ki, _kd, set_, get_, err, _max_out, _max_iout, _pout, _iout, _dout, out = struct.unpack_from(
                    "<HH12f", p, o
                )
                return (
                    {
                        f"{prefix}.pid_id": pid_id,
                        f"{prefix}.set": set_,
                        f"{prefix}.fdb": get_,
                        f"{prefix}.err": err,
                        f"{prefix}.out": out,
                    },
                    o + 52,
                )

            def parse_pid(prefix: str, p: bytes, o: int) -> tuple[dict[str, Any], int] | None:
                if o + 72 > len(p):
                    return None
                pid_id, mode, _r, *floats = struct.unpack_from("<HBB17f", p, o)
                (
                    _kp,
                    _ki,
                    _kd,
                    _max_out,
                    _max_iout,
                    set_,
                    fdb,
                    out,
                    pout,
                    iout,
                    dout,
                    *_rest,
                ) = floats
                return (
                    {
                        f"{prefix}.pid_id": pid_id,
                        f"{prefix}.mode": mode,
                        f"{prefix}.set": set_,
                        f"{prefix}.fdb": fdb,
                        f"{prefix}.out": out,
                        f"{prefix}.pout": pout,
                        f"{prefix}.iout": iout,
                        f"{prefix}.dout": dout,
                    },
                    o + 72,
                )
        else:
            loop_cnt, yaw_cur, pitch_cur, trig_cur, _res16, yaw_mode, pitch_mode, shoot_mode, _res8 = struct.unpack_from(
                "<IhhhH4B", payload, 0
            )
            off = 16

            def parse_gimbal_pid(prefix: str, p: bytes, o: int) -> tuple[dict[str, Any], int] | None:
                if o + 16 > len(p):
                    return None
                pid_id, mode, _r, set_, fdb, out = struct.unpack_from("<HBB3f", p, o)
                return (
                    {
                        f"{prefix}.pid_id": pid_id,
                        f"{prefix}.mode": mode,
                        f"{prefix}.set": set_,
                        f"{prefix}.fdb": fdb,
                        f"{prefix}.out": out,
                    },
                    o + 16,
                )

            parse_pid = parse_gimbal_pid

        fields: dict[str, Any] = {
            "loop_cnt": loop_cnt,
            "yaw_current": yaw_cur,
            "pitch_current": pitch_cur,
            "trigger_current": trig_cur,
            "yaw_mode": yaw_mode,
            "pitch_mode": pitch_mode,
            "shoot_mode": shoot_mode,
        }

        r = parse_gimbal_pid("yaw_angle_pid", payload, off)
        if r is None:
            return None
        d, off = r
        fields.update(d)
        r = parse_pid("yaw_speed_pid", payload, off)
        if r is None:
            return None
        d, off = r
        fields.update(d)
        r = parse_gimbal_pid("pitch_angle_pid", payload, off)
        if r is None:
            return None
        d, off = r
        fields.update(d)
        r = parse_pid("pitch_speed_pid", payload, off)
        if r is None:
            return None
        d, off = r
        fields.update(d)
        r = parse_pid("shoot_trigger_pid", payload, off)
        if r is None:
            return None
        d, off = r
        fields.update(d)

        for i in range(4):
            r = parse_pid(f"shoot_fric{i}_pid", payload, off)
            if r is None:
                return None
            d, off = r
            fields.update(d)

        return [(name, name, fields)]

    if tag == 0x0011:  # CHASSIS_LOOP
        if len(payload) not in (392, 112):
            return None

        loop_cnt, chassis_mode, last_chassis_mode, _res, vx, vy, wz, vx_set, vy_set, wz_set = struct.unpack_from(
            "<IBBH6f", payload, 0
        )
        off = 32

        def parse_pid(prefix: str, p: bytes, o: int) -> tuple[dict[str, Any], int] | None:
            if len(payload) == 392:
                if o + 72 > len(p):
                    return None
                pid_id, mode, _r, *floats = struct.unpack_from("<HBB17f", p, o)
                (
                    _kp,
                    _ki,
                    _kd,
                    _max_out,
                    _max_iout,
                    set_,
                    fdb,
                    out,
                    pout,
                    iout,
                    dout,
                    *_rest,
                ) = floats
                return (
                    {
                        f"{prefix}.pid_id": pid_id,
                        f"{prefix}.mode": mode,
                        f"{prefix}.set": set_,
                        f"{prefix}.fdb": fdb,
                        f"{prefix}.out": out,
                        f"{prefix}.pout": pout,
                        f"{prefix}.iout": iout,
                        f"{prefix}.dout": dout,
                    },
                    o + 72,
                )

            if o + 16 > len(p):
                return None
            pid_id, mode, _r, set_, fdb, out = struct.unpack_from("<HBB3f", p, o)
            return (
                {
                    f"{prefix}.pid_id": pid_id,
                    f"{prefix}.mode": mode,
                    f"{prefix}.set": set_,
                    f"{prefix}.fdb": fdb,
                    f"{prefix}.out": out,
                },
                o + 16,
            )

        fields: dict[str, Any] = {
            "loop_cnt": loop_cnt,
            "chassis_mode": chassis_mode,
            "last_chassis_mode": last_chassis_mode,
            "vx": vx,
            "vy": vy,
            "wz": wz,
            "vx_set": vx_set,
            "vy_set": vy_set,
            "wz_set": wz_set,
        }

        for i in range(4):
            r = parse_pid(f"m{i+1}_speed_pid", payload, off)
            if r is None:
                return None
            d, off = r
            fields.update(d)
        r = parse_pid("follow_pid", payload, off)
        if r is None:
            return None
        d, off = r
        fields.update(d)

        return [(name, name, fields)]

    if tag == 0x0031:  # APP_WATCH
        return [(name, name, {"size": len(payload)})]

    if tag == 0x0032:  # DETECT_STATUS
        if len(payload) == 448:
            enable_mask = 0
            lost_mask = 0
            data_error_mask = 0
            error_exist_mask = 0
            lost_count = 0
            data_error_count = 0
            error_count = 0
            display_toe = 0xFF
            best_priority = -1

            for i in range(14):
                off = i * 32
                (
                    _new_time,
                    _last_time,
                    _lost_time,
                    _work_time,
                    _set_offline_time_ms,
                    _set_online_time_ms,
                    enable,
                    priority,
                    error_exist,
                    is_lost,
                    data_is_error,
                    _reserved0,
                    _reserved1,
                    _reserved2,
                    _frequency,
                ) = struct.unpack_from("<IIIIHH8Bf", payload, off)

                bit = 1 << i
                if enable:
                    enable_mask |= bit
                if is_lost:
                    lost_mask |= bit
                    lost_count += 1
                    if priority > best_priority:
                        best_priority = int(priority)
                        display_toe = i
                if data_is_error:
                    data_error_mask |= bit
                    data_error_count += 1
                if error_exist:
                    error_exist_mask |= bit
                    error_count += 1

            return [
                (
                    name,
                    name,
                    {
                        "enable_mask": enable_mask,
                        "lost_mask": lost_mask,
                        "data_error_mask": data_error_mask,
                        "error_exist_mask": error_exist_mask,
                        "display_toe": display_toe,
                        "lost_count": lost_count,
                        "data_error_count": data_error_count,
                        "error_count": error_count,
                    },
                )
            ]

        v = _unpack_exact("<4H4B", payload)
        if v is None:
            return None
        (
            enable_mask,
            lost_mask,
            data_error_mask,
            error_exist_mask,
            display_toe,
            lost_count,
            data_error_count,
            error_count,
        ) = v
        return [
            (
                name,
                name,
                {
                    "enable_mask": enable_mask,
                    "lost_mask": lost_mask,
                    "data_error_mask": data_error_mask,
                    "error_exist_mask": error_exist_mask,
                    "display_toe": display_toe,
                    "lost_count": lost_count,
                    "data_error_count": data_error_count,
                    "error_count": error_count,
                },
            )
        ]

    if tag == 0x0030:  # CAN_RX
        v = _unpack_exact("<BBH8B", payload)
        if v is None:
            return None
        bus, dlc, std_id, *data = v
        fields = {"bus": bus, "dlc": dlc, "std_id": std_id}
        for i, b in enumerate(data):
            fields[f"data{i}"] = b
        return [(name, name, fields)]

    if tag == 0x0033:  # CHASSIS_POWER_LIMIT
        v = _unpack_exact("<5f4B", payload)
        if v is None:
            return None
        chassis_power, chassis_power_buffer, total_current, total_current_limit, current_scale, referee_offline, robot_id, _r0, _r1 = v
        return [
            (
                name,
                name,
                {
                    "chassis_power": chassis_power,
                    "chassis_power_buffer": chassis_power_buffer,
                    "total_current": total_current,
                    "total_current_limit": total_current_limit,
                    "current_scale": current_scale,
                    "referee_offline": referee_offline,
                    "robot_id": robot_id,
                },
            )
        ]

    if tag == 0x0034:  # GIMBAL_LIMIT
        v = _unpack_exact("<4B7f", payload)
        if v is None:
            return None
        axis, soft_limited, current_limited, _r, angle, angle_min, angle_max, gyro_set, current_before, current_after, current_limit = v
        return [
            (
                name,
                name,
                {
                    "axis": axis,
                    "soft_limited": soft_limited,
                    "current_limited": current_limited,
                    "angle": angle,
                    "angle_min": angle_min,
                    "angle_max": angle_max,
                    "gyro_set": gyro_set,
                    "current_before": current_before,
                    "current_after": current_after,
                    "current_limit": current_limit,
                },
            )
        ]

    if tag == 0x0040:  # APP_CONFIG
        return [
            (
                name,
                name,
                {
                    "size": len(payload),
                    "crc32": zlib.crc32(payload) & 0xFFFFFFFF,
                },
            )
        ]

    if tag == 0x0041:  # SYS_STATS
        v = _unpack_exact("<BBHIIIIIiIIIIIIIIHH", payload)
        if v is None:
            return None
        (
            sd_mounted,
            sdlog_active,
            _r16,
            sdlog_dropped,
            sdlog_ring_used,
            sdlog_ring_free,
            sdlog_bytes_flushed,
            sdlog_last_sync_ms,
            sdlog_last_error,
            heap_free,
            heap_ever_free,
            stack_gimbal,
            stack_chassis,
            stack_detect,
            stack_calibrate,
            gimbal_loop_cnt,
            chassis_loop_cnt,
            cpu_load_permille,
            _r16_2,
        ) = v
        return [
            (
                name,
                name,
                {
                    "sd_mounted": sd_mounted,
                    "sdlog_active": sdlog_active,
                    "sdlog_dropped": sdlog_dropped,
                    "sdlog_ring_used": sdlog_ring_used,
                    "sdlog_ring_free": sdlog_ring_free,
                    "sdlog_bytes_flushed": sdlog_bytes_flushed,
                    "sdlog_last_sync_ms": sdlog_last_sync_ms,
                    "sdlog_last_error": sdlog_last_error,
                    "heap_free": heap_free,
                    "heap_ever_free": heap_ever_free,
                    "stack_gimbal": stack_gimbal,
                    "stack_chassis": stack_chassis,
                    "stack_detect": stack_detect,
                    "stack_calibrate": stack_calibrate,
                    "gimbal_loop_cnt": gimbal_loop_cnt,
                    "chassis_loop_cnt": chassis_loop_cnt,
                    "cpu_load_permille": cpu_load_permille,
                },
            )
        ]

    if tag == 0x0042:  # EVENT
        v = _unpack_exact("<HHII", payload)
        if v is None:
            return None
        event_id, arg0_u16, arg1_u32, arg2_u32 = v
        return [(name, name, {"event_id": event_id, "arg0_u16": arg0_u16, "arg1_u32": arg1_u32, "arg2_u32": arg2_u32})]

    if tag == 0x0043:  # VISION_USB_RX
        v = _unpack_exact("<2sB6fH", payload)
        if v is None:
            return None
        head, mode, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc, crc16 = v
        return [
            (
                name,
                name,
                {
                    "head": head.decode("ascii", errors="replace"),
                    "mode": mode,
                    "yaw": yaw,
                    "yaw_vel": yaw_vel,
                    "yaw_acc": yaw_acc,
                    "pitch": pitch,
                    "pitch_vel": pitch_vel,
                    "pitch_acc": pitch_acc,
                    "crc16": crc16,
                },
            )
        ]

    if tag == 0x0044:  # UART1_TUNE
        if len(payload) < 4:
            return None
        (seq,) = struct.unpack_from("<I", payload, 0)
        cmd_b = payload[4:]
        nul = cmd_b.find(b"\0")
        if nul != -1:
            cmd_b = cmd_b[:nul]
        cmd = cmd_b.decode("utf-8", errors="replace")
        return [(name, name, {"seq": seq, "cmd": cmd})]

    if tag == 0x0048:  # IMU_TRUST
        v = _unpack_exact("<7f4B", payload)
        if v is None:
            return None
        (
            acc_norm_g,
            acc_ref_g,
            norm_err_g,
            angle_deg,
            trust,
            kp_gain,
            gyro_norm_dps,
            acc_healthy,
            acc_rejected,
            fusion_mode,
            _reserved,
        ) = v
        return [
            (
                name,
                name,
                {
                    "acc_norm_g": acc_norm_g,
                    "acc_ref_g": acc_ref_g,
                    "norm_err_g": norm_err_g,
                    "angle_deg": angle_deg,
                    "trust": trust,
                    "kp_gain": kp_gain,
                    "gyro_norm_dps": gyro_norm_dps,
                    "acc_healthy": acc_healthy,
                    "acc_rejected": acc_rejected,
                    "fusion_mode": fusion_mode,
                },
            )
        ]

    if tag == 0x0049:  # CONTROL_SUMMARY
        v = _unpack_exact("<5BbbB4h10f4h", payload)
        if v is None:
            return None
        (
            manual_source,
            chassis_mode,
            yaw_mode,
            pitch_mode,
            shoot_mode,
            rc_s0,
            rc_s1,
            _reserved0,
            rc0,
            rc1,
            rc2,
            rc3,
            chassis_vx_set,
            chassis_vy_set,
            chassis_wz_set,
            chassis_vx,
            chassis_vy,
            chassis_wz,
            yaw_set_deg,
            yaw_deg,
            pitch_set_deg,
            pitch_deg,
            yaw_current,
            pitch_current,
            trigger_current,
            fric_speed_set_rpm,
        ) = v
        return [
            (
                name,
                name,
                {
                    "manual_source": manual_source,
                    "chassis_mode": chassis_mode,
                    "yaw_mode": yaw_mode,
                    "pitch_mode": pitch_mode,
                    "shoot_mode": shoot_mode,
                    "rc_s0": rc_s0,
                    "rc_s1": rc_s1,
                    "rc_ch0": rc0,
                    "rc_ch1": rc1,
                    "rc_ch2": rc2,
                    "rc_ch3": rc3,
                    "chassis_vx_set": chassis_vx_set,
                    "chassis_vy_set": chassis_vy_set,
                    "chassis_wz_set": chassis_wz_set,
                    "chassis_vx": chassis_vx,
                    "chassis_vy": chassis_vy,
                    "chassis_wz": chassis_wz,
                    "yaw_set_deg": yaw_set_deg,
                    "yaw_deg": yaw_deg,
                    "pitch_set_deg": pitch_set_deg,
                    "pitch_deg": pitch_deg,
                    "yaw_current": yaw_current,
                    "pitch_current": pitch_current,
                    "trigger_current": trigger_current,
                    "fric_speed_set_rpm": fric_speed_set_rpm,
                },
            )
        ]

    if tag == 0x004A:  # PITCH_CALI
        v = _unpack_exact("<8B4H2Ii8f", payload)
        if v is None:
            return None
        (
            state,
            angle_idx,
            bullet_idx,
            angle_points,
            bullet_points,
            bullet_ready,
            is_stable,
            motor_raw_mode,
            target_bullet,
            bullet_now,
            completed_cells,
            grid_cells,
            state_elapsed_ms,
            stable_elapsed_ms,
            last_error,
            target_angle,
            cmd_angle,
            angle,
            gyro,
            current,
            hold_avg,
            raw_current_cmd,
            delta,
        ) = v
        return [
            (
                name,
                name,
                {
                    "state": state,
                    "state_name": PITCH_CALI_STATE_NAMES.get(state, f"STATE_{state}"),
                    "angle_idx": angle_idx,
                    "bullet_idx": bullet_idx,
                    "angle_points": angle_points,
                    "bullet_points": bullet_points,
                    "bullet_ready": bullet_ready,
                    "is_stable": is_stable,
                    "motor_raw_mode": motor_raw_mode,
                    "target_bullet": target_bullet,
                    "bullet_now": bullet_now,
                    "completed_cells": completed_cells,
                    "grid_cells": grid_cells,
                    "state_elapsed_ms": state_elapsed_ms,
                    "stable_elapsed_ms": stable_elapsed_ms,
                    "last_error": last_error,
                    "target_angle": target_angle,
                    "cmd_angle": cmd_angle,
                    "angle": angle,
                    "gyro": gyro,
                    "current": current,
                    "hold_avg": hold_avg,
                    "raw_current_cmd": raw_current_cmd,
                    "delta": delta,
                },
            )
        ]

    return None


def load_dataset(path: Path) -> Dataset:
    with path.open("rb") as f:
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

        ds = Dataset(source_path=str(path), boot_tick_ms=boot_tick_ms, file_version=version)

        if version == 1:
            parser = RecordStreamParser()
            while True:
                chunk = f.read(1024 * 1024)
                if not chunk:
                    break
                for tick_ms, tag, payload in parser.feed(chunk):
                    ds.add_record(tick_ms, tag, payload)
            return ds

        if version == 2:
            parser = RecordStreamParser()
        elif version == 3:
            parser = VarintRecordStreamParser(boot_tick_ms)
        else:
            raise ValueError(f"Unsupported sdlog version {version}")

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

            for tick_ms, tag, payload in parser.feed(raw):
                ds.add_record(tick_ms, tag, payload)

        return ds


def _downsample(t_ms: list[int], y: list[Any], max_points: int) -> tuple[list[int], list[Any]]:
    if max_points <= 0 or len(t_ms) <= max_points:
        return t_ms, y
    step = (len(t_ms) + max_points - 1) // max_points
    return t_ms[::step], y[::step]


def _load_html() -> bytes:
    html_path = Path(__file__).with_suffix(".html")
    return html_path.read_bytes()


class Handler(BaseHTTPRequestHandler):
    dataset: Dataset
    html_bytes: bytes
    server_version = "sdlog-viewer/0.1"

    def log_message(self, fmt: str, *args: Any) -> None:
        return

    def do_GET(self) -> None:  # noqa: N802 - stdlib signature
        url = urllib.parse.urlparse(self.path)
        path = url.path
        qs = urllib.parse.parse_qs(url.query)

        try:
            if path == "/":
                self._send_bytes(HTTPStatus.OK, "text/html; charset=utf-8", self.html_bytes)
                return

            if path == "/api/index":
                self._send_bytes(HTTPStatus.OK, "application/json; charset=utf-8", self.dataset.tag_index_json())
                return

            if path == "/api/series":
                key = (qs.get("key") or [""])[0]
                field = (qs.get("field") or [""])[0]
                max_points_s = (qs.get("max_points") or ["5000"])[0]
                max_points = int(max_points_s) if max_points_s.isdigit() else 5000

                s = self.dataset.series.get(key)
                if s is None:
                    self._send_text(HTTPStatus.NOT_FOUND, f"unknown key {key}")
                    return
                col = s.fields.get(field)
                if col is None:
                    self._send_text(HTTPStatus.NOT_FOUND, f"unknown field {field}")
                    return

                t_ms, y = _downsample(s.ticks_ms, col, max_points)
                payload = json.dumps({"t_ms": t_ms, "y": y}, ensure_ascii=False).encode("utf-8")
                self._send_bytes(HTTPStatus.OK, "application/json; charset=utf-8", payload)
                return

            if path == "/api/export.csv":
                key = (qs.get("key") or [""])[0]
                s = self.dataset.series.get(key)
                if s is None:
                    self._send_text(HTTPStatus.NOT_FOUND, f"unknown key {key}")
                    return
                self._send_csv_attachment(s)
                return

            if path == "/api/export_field.csv":
                key = (qs.get("key") or [""])[0]
                field = (qs.get("field") or [""])[0]
                s = self.dataset.series.get(key)
                if s is None:
                    self._send_text(HTTPStatus.NOT_FOUND, f"unknown key {key}")
                    return
                col = s.fields.get(field)
                if col is None:
                    self._send_text(HTTPStatus.NOT_FOUND, f"unknown field {field}")
                    return
                self._send_field_csv_attachment(s, field, col)
                return

            self._send_text(HTTPStatus.NOT_FOUND, "not found")
        except Exception as e:
            self._send_text(HTTPStatus.INTERNAL_SERVER_ERROR, f"{type(e).__name__}: {e}")

    def _send_bytes(self, status: HTTPStatus, content_type: str, data: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_text(self, status: HTTPStatus, msg: str) -> None:
        data = msg.encode("utf-8")
        self._send_bytes(status, "text/plain; charset=utf-8", data)

    def _send_csv_attachment(self, s: Series) -> None:
        filename = f"{s.name}.csv".replace("/", "_")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/csv; charset=utf-8")
        self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
        self.send_header("Cache-Control", "no-store")
        self.end_headers()

        text = io.TextIOWrapper(self.wfile, encoding="utf-8", newline="")
        w = csv.writer(text)
        field_names = sorted(s.fields.keys())
        w.writerow(["tick_ms", *field_names])
        for i, tick in enumerate(s.ticks_ms):
            row = [tick]
            for fn in field_names:
                row.append(s.fields[fn][i])
            w.writerow(row)
        text.flush()

    def _send_field_csv_attachment(self, s: Series, field: str, col: list[Any]) -> None:
        filename = f"{s.name}.{field}.csv".replace("/", "_")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/csv; charset=utf-8")
        self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
        self.send_header("Cache-Control", "no-store")
        self.end_headers()

        text = io.TextIOWrapper(self.wfile, encoding="utf-8", newline="")
        w = csv.writer(text)
        w.writerow(["tick_ms", field])
        for tick, v in zip(s.ticks_ms, col):
            w.writerow([tick, v])
        text.flush()

def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description="HERO sdlog web viewer (no extra deps).")
    ap.add_argument("input", help="Input sdlog_XXXX.bin (v1/v2)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=0, help="0 = choose a free port")
    ap.add_argument("--no-open", action="store_true", help="Do not auto-open the browser")
    args = ap.parse_args(argv)

    path = Path(args.input)
    if not path.exists():
        ap.error(f"file not found: {path}")

    print(f"[sdlog] loading: {path}")
    ds = load_dataset(path)
    print(f"[sdlog] parsed series: {len(ds.series)} (unknown tags: {len(ds.unknown_tag_counts)})")

    Handler.dataset = ds
    Handler.html_bytes = _load_html()

    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    host, port = httpd.server_address
    url = f"http://{host}:{port}/"
    print(f"[sdlog] serving: {url}")

    if not args.no_open:
        threading.Timer(0.2, lambda: webbrowser.open(url)).start()

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
