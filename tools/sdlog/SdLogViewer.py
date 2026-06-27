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
    0x0040: "CONFIG",
    0x0041: "SYS_STATS",
    0x0042: "EVENT",
    0x0043: "VISION_RX",
    0x0044: "AUX_TUNE",
    0x0046: "MANUAL_INPUT_RAW",
    0x0047: "IMAGE_LINK_STATS",
    0x0048: "IMU_TRUST",
    0x0049: "CONTROL_SUMMARY",
    0x004A: "PITCH_CALI",
    0x004B: "CHASSIS_BASE_STREAM",
    0x004C: "GIMBAL_BASE_STREAM",
    0x004D: "IMU_BASE_STREAM",
    0x004E: "RT_PROFILER",
    0x004F: "WHEELLEG_MIT_CONFIG",
    0x0050: "WHEELLEG_MIT_STATUS",
    0x0051: "BUILD_INFO",
    0x0052: "RUNTIME_DEVICE",
    0x0053: "WHEELLEG_MIT_MOTOR_DIAG",
}

RT_PROFILER_NAMES: dict[int, str] = {
    0: "GIMBAL_CONTROL_LOOP",
    1: "CHASSIS_CONTROL_LOOP",
    2: "CAN_COMMAND_TX_LOOP",
    3: "CAN_FEEDBACK_RX_WAKE",
    4: "SDLOG_WRITE",
    5: "SDLOG_COMPRESS",
    6: "SDLOG_BLOCK_WRITE",
    7: "SDLOG_SYNC",
    8: "WATCH_TASK_BEAT",
}

PROFILE_KIND_NAMES: dict[int, str] = {
    0: "unknown",
    1: "hero",
    2: "infantry",
    3: "wheelleg",
    4: "sentry",
    5: "carrier",
    6: "custom",
}

BOARD_KIND_NAMES: dict[int, str] = {
    0: "unknown",
    1: "stm32f407",
    2: "stm32f427",
    3: "stm32h7",
    4: "custom",
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

WHEELLEG_MODE_NAMES: dict[int, str] = {
    0: "DISABLED",
    1: "CALIBRATION",
    2: "STANDUP",
    3: "BALANCE",
    4: "JUMP",
    5: "AIRBORNE",
    6: "LAND",
    7: "RECOVERY",
    8: "FAULT",
    9: "BENCH",
    10: "LEG_POSITION",
}

WHEELLEG_MIT_MOTOR_ROLE_NAMES: dict[int, str] = {
    0: "LF",
    1: "LB",
    2: "LW",
    3: "RF",
    4: "RB",
    5: "RW",
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


class RecordStreamParser:
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
class UnknownRecord:
    tick_ms: int
    tag: int
    payload: bytes


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
    def __init__(self, source_path: str, boot_tick_ms: int, file_header_size: int) -> None:
        self.source_path = source_path
        self.boot_tick_ms = boot_tick_ms
        self.file_header_size = file_header_size
        self.series: dict[str, Series] = {}
        self.unknown_tag_counts: dict[int, int] = {}
        self.unknown_records: list[UnknownRecord] = []

    def add_record(self, tick_ms: int, tag: int, payload: bytes) -> None:
        extracted = extract_records(tick_ms, tag, payload)
        if extracted is None:
            self.unknown_tag_counts[tag] = self.unknown_tag_counts.get(tag, 0) + 1
            self.unknown_records.append(UnknownRecord(tick_ms=tick_ms, tag=tag, payload=payload))
            return

        for sample_tick_ms, key, name, values in extracted:
            s = self.series.get(key)
            if s is None:
                s = Series(key=key, name=name)
                self.series[key] = s
            s.add(sample_tick_ms, values)

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
                "file_header_size": self.file_header_size,
                "tags": tags,
                "unknown": unknown,
            },
            ensure_ascii=False,
        ).encode("utf-8")

    def unknown_csv(self) -> bytes:
        text = io.StringIO()
        w = csv.writer(text)
        w.writerow(["tick_ms", "tag", "tag_name", "len", "crc32", "payload_hex"])
        for rec in self.unknown_records:
            w.writerow(
                [
                    rec.tick_ms,
                    f"0x{rec.tag:04X}",
                    TAG_NAMES.get(rec.tag, f"0x{rec.tag:04X}"),
                    len(rec.payload),
                    f"0x{(zlib.crc32(rec.payload) & 0xFFFFFFFF):08X}",
                    rec.payload.hex(),
                ]
            )
        return text.getvalue().encode("utf-8")


def sdlog_tag_name(tag: int) -> str:
    return TAG_NAMES.get(tag, f"0x{tag:04X}")


def _unpack_exact(fmt: str, payload: bytes) -> tuple[Any, ...] | None:
    size = struct.calcsize(fmt)
    if len(payload) != size:
        return None
    return struct.unpack(fmt, payload)


def _cstr(raw: bytes) -> str:
    nul = raw.find(b"\0")
    if nul != -1:
        raw = raw[:nul]
    return raw.decode("utf-8", errors="replace")


def _stream_sample_tick_ms(start_tick_ms: int, period_us: int, sample_idx: int) -> int:
    return start_tick_ms + ((sample_idx * period_us + 500) // 1000)


def _unpack_stream_header(payload: bytes, expected_sample_size: int) -> tuple[int, int, int, int, int | None] | None:
    if len(payload) >= 8:
        start_tick_ms, period_us, sample_count, version = struct.unpack_from("<IHBB", payload, 0)
        if version == 1 and sample_count != 0 and len(payload) == 8 + sample_count * expected_sample_size:
            return 8, start_tick_ms, period_us, sample_count, None

    if len(payload) >= 20:
        version, sample_size, sample_count, _reserved, start_tick_ms, period_us, seq0 = struct.unpack_from("<HHHHIII", payload, 0)
        if version == 1 and sample_size == expected_sample_size and sample_count != 0 and len(payload) == 20 + sample_count * expected_sample_size:
            return 20, start_tick_ms, period_us, sample_count, seq0

    return None


def _unpack_imu_base_stream_header(payload: bytes) -> tuple[int, int, int, int, int | None, int, int] | None:
    sample_size_by_version = {
        1: 44,
        2: 22,
        3: 20,
    }

    if len(payload) >= 8:
        start_tick_ms, period_us, sample_count, version = struct.unpack_from("<IHBB", payload, 0)
        sample_size = sample_size_by_version.get(version)
        if sample_size is not None and sample_count != 0 and len(payload) == 8 + sample_count * sample_size:
            return 8, start_tick_ms, period_us, sample_count, None, version, sample_size

    if len(payload) >= 20:
        version, sample_size, sample_count, _reserved, start_tick_ms, period_us, seq0 = struct.unpack_from("<HHHHIII", payload, 0)
        if (
            sample_size_by_version.get(version) == sample_size
            and sample_count != 0
            and len(payload) == 20 + sample_count * sample_size
        ):
            return 20, start_tick_ms, period_us, sample_count, seq0, version, sample_size

    return None


def _extract_chassis_base_stream_records(payload: bytes) -> list[tuple[int, str, str, dict[str, Any]]] | None:
    header = _unpack_stream_header(payload, 28)
    if header is None:
        return None

    header_size, start_tick_ms, period_us, sample_count, seq0 = header

    out: list[tuple[int, str, str, dict[str, Any]]] = []
    for sample_idx in range(sample_count):
        off = header_size + sample_idx * 28
        chassis_mode, last_chassis_mode, _res16, *vals = struct.unpack_from("<BBH12h", payload, off)
        tick_ms = _stream_sample_tick_ms(start_tick_ms, period_us, sample_idx)
        fields: dict[str, Any] = {
            "sample_idx": sample_idx,
            "chassis_mode": chassis_mode,
            "last_chassis_mode": last_chassis_mode,
        }
        if seq0 is not None:
            fields["seq"] = seq0 + sample_idx
        for i in range(4):
            fields[f"wheel_rpm{i}"] = vals[i]
            fields[f"current_request{i}"] = vals[4 + i]
            fields[f"current_output{i}"] = vals[8 + i]
        out.append((tick_ms, "CHASSIS_BASE_STREAM", "CHASSIS_BASE_STREAM", fields))

    return out


def _extract_gimbal_base_stream_records(payload: bytes) -> list[tuple[int, str, str, dict[str, Any]]] | None:
    header = _unpack_stream_header(payload, 28)
    if header is None:
        return None

    header_size, start_tick_ms, period_us, sample_count, seq0 = header

    out: list[tuple[int, str, str, dict[str, Any]]] = []
    for sample_idx in range(sample_count):
        off = header_size + sample_idx * 28
        (
            gimbal_behaviour,
            test_mode,
            yaw_motor_mode,
            pitch_motor_mode,
            yaw_angle,
            pitch_angle,
            yaw_gyro,
            pitch_gyro,
            yaw_current_request,
            pitch_current_request,
            yaw_current_output,
            pitch_current_output,
        ) = struct.unpack_from("<4B4f4h", payload, off)
        tick_ms = _stream_sample_tick_ms(start_tick_ms, period_us, sample_idx)
        out.append(
            (
                tick_ms,
                "GIMBAL_BASE_STREAM",
                "GIMBAL_BASE_STREAM",
                {
                    "sample_idx": sample_idx,
                    "gimbal_behaviour": gimbal_behaviour,
                    "test_mode": test_mode,
                    "yaw_motor_mode": yaw_motor_mode,
                    "pitch_motor_mode": pitch_motor_mode,
                    "yaw_angle": yaw_angle,
                    "pitch_angle": pitch_angle,
                    "yaw_gyro": yaw_gyro,
                    "pitch_gyro": pitch_gyro,
                    "yaw_current_request": yaw_current_request,
                    "pitch_current_request": pitch_current_request,
                    "yaw_current_output": yaw_current_output,
                    "pitch_current_output": pitch_current_output,
                },
            )
        )
        if seq0 is not None:
            out[-1][3]["seq"] = seq0 + sample_idx

    return out


def _extract_imu_base_stream_records(payload: bytes) -> list[tuple[int, str, str, dict[str, Any]]] | None:
    header = _unpack_imu_base_stream_header(payload)
    if header is None:
        return None

    header_size, start_tick_ms, period_us, sample_count, seq0, version, sample_size = header

    out: list[tuple[int, str, str, dict[str, Any]]] = []
    for sample_idx in range(sample_count):
        off = header_size + sample_idx * sample_size
        values: dict[str, Any]
        if version == 1:
            qw, qx, qy, qz, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, temp = struct.unpack_from("<11f", payload, off)
            values = {
                "sample_idx": sample_idx,
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
            }
        else:
            if version == 2:
                qw, qx, qy, qz, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, temp = struct.unpack_from("<11h", payload, off)
            else:
                qw, qx, qy, qz, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z = struct.unpack_from("<10h", payload, off)
                temp = None
            values = {
                "sample_idx": sample_idx,
                "quat_w": qw / 32767.0,
                "quat_x": qx / 32767.0,
                "quat_y": qy / 32767.0,
                "quat_z": qz / 32767.0,
                "gyro_x": gyro_x / 512.0,
                "gyro_y": gyro_y / 512.0,
                "gyro_z": gyro_z / 512.0,
                "accel_x": accel_x / 128.0,
                "accel_y": accel_y / 128.0,
                "accel_z": accel_z / 128.0,
            }
            if temp is not None:
                values["temp"] = temp / 100.0

        tick_ms = _stream_sample_tick_ms(start_tick_ms, period_us, sample_idx)
        out.append(
            (
                tick_ms,
                "IMU",
                "IMU",
                values,
            )
        )
        if seq0 is not None:
            out[-1][3]["seq"] = seq0 + sample_idx

    return out


def extract_records(tick_ms: int, tag: int, payload: bytes) -> list[tuple[int, str, str, dict[str, Any]]] | None:
    if tag == 0x004B:
        return _extract_chassis_base_stream_records(payload)
    if tag == 0x004C:
        return _extract_gimbal_base_stream_records(payload)
    if tag == 0x004D:
        return _extract_imu_base_stream_records(payload)

    extracted = extract_series(tag, payload)
    if extracted is None:
        return None
    return [(tick_ms, key, name, values) for key, name, values in extracted]


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


def _add_pid_param_fields(fields: dict[str, Any], prefix: str, payload: bytes, off: int) -> int:
    kp, ki, kd, max_out, max_iout = struct.unpack_from("<5f", payload, off)
    fields[f"{prefix}_kp"] = kp
    fields[f"{prefix}_ki"] = ki
    fields[f"{prefix}_kd"] = kd
    fields[f"{prefix}_max_out"] = max_out
    fields[f"{prefix}_max_iout"] = max_iout
    return off + struct.calcsize("<5f")


def _extract_wheelleg_mit_config(name: str, payload: bytes) -> list[tuple[str, str, dict[str, Any]]] | None:
    if len(payload) != 392:
        return None

    off = 0
    version, enable_switch_pos, control_period_ms, rc_deadband, lqr_default_mask = struct.unpack_from("<BBHHH", payload, off)
    off += struct.calcsize("<BBHHH")
    actuator_id = struct.unpack_from("<6B", payload, off)
    off += 6
    joint_dir = struct.unpack_from("<4b", payload, off)
    off += 4
    off += 2
    joint_zero = struct.unpack_from("<4f", payload, off)
    off += struct.calcsize("<4f")

    scalar_names = [
        "l1_m",
        "l2_m",
        "l3_m",
        "l4_m",
        "l5_m",
        "wheel_radius_m",
        "default_leg_length_m",
        "min_leg_length_m",
        "max_leg_length_m",
        "support_bias_n",
        "leg_mass_kg",
        "max_wheel_torque_nm",
        "max_joint_torque_nm",
        "max_jump_joint_torque_nm",
        "max_support_force_n",
        "attitude_limit_rad",
        "observer_lpf",
        "pitch_balance_offset_right_rad",
        "pitch_balance_offset_left_rad",
        "max_v_mps",
        "max_yaw_rate_radps",
    ]
    scalar_values = struct.unpack_from(f"<{len(scalar_names)}f", payload, off)
    off += struct.calcsize(f"<{len(scalar_names)}f")

    fields: dict[str, Any] = {
        "version": version,
        "enable_switch_pos": enable_switch_pos,
        "control_period_ms": control_period_ms,
        "rc_deadband": rc_deadband,
        "lqr_default_mask": lqr_default_mask,
        "actuator_left_front_id": actuator_id[0],
        "actuator_left_back_id": actuator_id[1],
        "actuator_left_wheel_id": actuator_id[2],
        "actuator_right_front_id": actuator_id[3],
        "actuator_right_back_id": actuator_id[4],
        "actuator_right_wheel_id": actuator_id[5],
        "left_front_dir": joint_dir[0],
        "left_back_dir": joint_dir[1],
        "right_front_dir": joint_dir[2],
        "right_back_dir": joint_dir[3],
        "left_front_zero_rad": joint_zero[0],
        "left_back_zero_rad": joint_zero[1],
        "right_front_zero_rad": joint_zero[2],
        "right_back_zero_rad": joint_zero[3],
    }
    for k, v in zip(scalar_names, scalar_values):
        fields[k] = v

    off = _add_pid_param_fields(fields, "leg_length_pid", payload, off)
    off = _add_pid_param_fields(fields, "leg_split_pid", payload, off)
    off = _add_pid_param_fields(fields, "turn_pid", payload, off)
    off = _add_pid_param_fields(fields, "roll_pid", payload, off)

    for row in range(12):
        coe = struct.unpack_from("<4f", payload, off)
        off += struct.calcsize("<4f")
        fields[f"lqr_{row}_uses_default"] = 1 if (lqr_default_mask & (1 << row)) else 0
        fields[f"lqr_{row}_c0"] = coe[0]
        fields[f"lqr_{row}_c1"] = coe[1]
        fields[f"lqr_{row}_c2"] = coe[2]
        fields[f"lqr_{row}_c3"] = coe[3]

    return [(name, name, fields)]


def _extract_wheelleg_mit_status(name: str, payload: bytes) -> list[tuple[str, str, dict[str, Any]]] | None:
    if len(payload) != 200:
        return None

    off = 0
    (
        version,
        mode,
        last_mode,
        controller_active,
        fault_flags,
        feedback_faults,
        test_mode,
        manual_on,
        profile_on,
        _reserved8,
    ) = struct.unpack_from("<4BHH4B", payload, off)
    off += struct.calcsize("<4BHH4B")

    float_names = [
        "pitch_rad",
        "roll_rad",
        "yaw_rad",
        "gyro_x_radps",
        "gyro_y_radps",
        "gyro_z_radps",
        "target_v_mps",
        "target_yaw_rate_radps",
        "target_leg_length_m",
        "target_foot_x_m",
        "target_leg_theta_rad",
        "observer_x_m",
        "observer_v_mps",
        "leg_left_length_m",
        "leg_right_length_m",
        "leg_left_theta_rad",
        "leg_right_theta_rad",
        "leg_left_d_length_mps",
        "leg_right_d_length_mps",
        "leg_left_d_theta_radps",
        "leg_right_d_theta_radps",
        "leg_left_support_force_n",
        "leg_right_support_force_n",
        "leg_left_hip_torque_nm",
        "leg_right_hip_torque_nm",
        "leg_left_front_joint_torque_nm",
        "leg_left_back_joint_torque_nm",
        "leg_right_front_joint_torque_nm",
        "leg_right_back_joint_torque_nm",
        "wheel_left_pos_rad",
        "wheel_right_pos_rad",
        "wheel_left_vel_radps",
        "wheel_right_vel_radps",
        "wheel_left_torque_nm",
        "wheel_right_torque_nm",
        "lqr_right_theta_err_rad",
        "lqr_right_dtheta_radps",
        "lqr_x_err_m",
        "lqr_v_err_mps",
        "lqr_right_pitch_err_rad",
        "lqr_right_gyro_radps",
        "lqr_right_wheel_torque_nm",
        "lqr_left_wheel_torque_nm",
        "lqr_right_hip_torque_nm",
        "lqr_left_hip_torque_nm",
    ]
    float_values = struct.unpack_from(f"<{len(float_names)}f", payload, off)
    off += struct.calcsize(f"<{len(float_names)}f")
    contact_left, contact_right, motor_online_bits, _reserved8_2 = struct.unpack_from("<4B", payload, off)
    off += struct.calcsize("<4B")
    (overrun_count,) = struct.unpack_from("<I", payload, off)

    fields: dict[str, Any] = {
        "version": version,
        "mode": mode,
        "mode_name": WHEELLEG_MODE_NAMES.get(mode, f"MODE_{mode}"),
        "last_mode": last_mode,
        "last_mode_name": WHEELLEG_MODE_NAMES.get(last_mode, f"MODE_{last_mode}"),
        "controller_active": controller_active,
        "fault_flags": fault_flags,
        "feedback_faults": feedback_faults,
        "test_mode": test_mode,
        "manual_on": manual_on,
        "profile_on": profile_on,
    }
    for k, v in zip(float_names, float_values):
        fields[k] = v
    target_foot_y_sq = fields["target_leg_length_m"] ** 2 - fields["target_foot_x_m"] ** 2
    fields["target_foot_y_m"] = target_foot_y_sq ** 0.5 if target_foot_y_sq > 0.0 else 0.0

    fields.update(
        {
            "contact_left": contact_left,
            "contact_right": contact_right,
            "left_front_online": 1 if (motor_online_bits & (1 << 0)) else 0,
            "left_back_online": 1 if (motor_online_bits & (1 << 1)) else 0,
            "left_wheel_online": 1 if (motor_online_bits & (1 << 2)) else 0,
            "right_front_online": 1 if (motor_online_bits & (1 << 3)) else 0,
            "right_back_online": 1 if (motor_online_bits & (1 << 4)) else 0,
            "right_wheel_online": 1 if (motor_online_bits & (1 << 5)) else 0,
            "motor_online_bits": motor_online_bits,
            "overrun_count": overrun_count,
        }
    )

    return [(name, name, fields)]


def _extract_runtime_device(name: str, payload: bytes) -> list[tuple[str, str, dict[str, Any]]] | None:
    if len(payload) != 8:
        return None

    version, kind, group, group_index, enabled, bus, source_id = struct.unpack("<6BH", payload)
    return [
        (
            name,
            name,
            {
                "version": version,
                "kind": kind,
                "group": group,
                "group_index": group_index,
                "enabled": enabled,
                "bus": bus,
                "source_id": source_id,
            },
        )
    ]


def _extract_wheelleg_mit_motor_diag(name: str, payload: bytes) -> list[tuple[str, str, dict[str, Any]]] | None:
    if len(payload) not in (496, 528, 552):
        return None

    version, count, _reserved16 = struct.unpack_from("<BBH", payload, 0)
    off = 4
    can_fields: dict[str, Any] = {}
    if len(payload) >= 528:
        (
            can1_rx_count,
            can1_rx_drop,
            can1_tx_count,
            can1_tx_fail,
            can2_rx_count,
            can2_rx_drop,
            can2_tx_count,
            can2_tx_fail,
        ) = struct.unpack_from("<8I", payload, off)
        off += 32
        can_fields = {
            "version": version,
            "can1_rx_count": can1_rx_count,
            "can1_rx_drop": can1_rx_drop,
            "can1_tx_count": can1_tx_count,
            "can1_tx_fail": can1_tx_fail,
            "can2_rx_count": can2_rx_count,
            "can2_rx_drop": can2_rx_drop,
            "can2_tx_count": can2_tx_count,
            "can2_tx_fail": can2_tx_fail,
        }

    has_tx_id_count = len(payload) >= 552
    entry_fmt = "<18B4H6I9f" if has_tx_id_count else "<18B4H5I9f"
    entry_size = struct.calcsize(entry_fmt)
    rows: list[tuple[str, str, dict[str, Any]]] = []
    if can_fields:
        rows.append((f"{name}.CAN", f"{name}.CAN", can_fields))
    field_names = [
        "role",
        "actuator_id",
        "fresh",
        "fb_online",
        "cmd_active",
        "cmd_mode",
        "applied_active",
        "applied_mode",
        "applied_drive_state",
        "applied_flags",
        "fb_bus",
        "fb_rx_dlc",
        "fb_rx_data0",
        "fb_rx_data0_low4",
        "fb_rx_data0_high4",
        "fb_motor_id",
        "fb_state",
        "reserved8",
        "fb_rx_id",
        "applied_tx_id",
        "cmd_writer",
        "cmd_timeout_ms",
        "fb_rx_count",
        *([] if not has_tx_id_count else ["tx_id_count"]),
        "fb_last_rx_tick_ms",
        "cmd_seq",
        "cmd_tick_ms",
        "applied_tick_ms",
        "cmd_position_rad",
        "cmd_velocity_radps",
        "cmd_kp",
        "cmd_kd",
        "cmd_torque_nm",
        "applied_torque_nm",
        "fb_position_rad",
        "fb_velocity_radps",
        "fb_torque_nm",
    ]

    available = min(int(count), 6)
    for i in range(available):
        entry_off = off + i * entry_size
        values = struct.unpack_from(entry_fmt, payload, entry_off)
        fields = {"version": version, "count": count}
        fields.update(can_fields)
        fields.update({k: v for k, v in zip(field_names, values) if k != "reserved8"})
        role = int(fields.get("role", i))
        role_name = WHEELLEG_MIT_MOTOR_ROLE_NAMES.get(role, f"ROLE_{role}")
        fields["role_name"] = role_name
        fields["fb_rx_data0_hex"] = f"0x{int(fields['fb_rx_data0']):02X}"
        series_key = f"{name}.{role_name}"
        rows.append((series_key, series_key, fields))

    return rows


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

    if tag == 0x0040:  # CONFIG
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

    if tag == 0x0051:  # BUILD_INFO
        if len(payload) < 2:
            return None
        version = struct.unpack_from("<H", payload, 0)[0]
        if version >= 3:
            v = _unpack_exact("<4H2I4BI3B4BI32s32s16s12s9s", payload)
            if v is None:
                return None
            (
                version,
                header_size,
                schema_version,
                _flags,
                config_size,
                config_crc32,
                task_module_count,
                high_rate_div,
                compression_enabled,
                build_dirty,
                task_module_mask,
                runtime_device_count,
                motorInstCount,
                controller_count,
                profile_kind,
                board_kind,
                rtProfCount,
                board_can_bus_count,
                board_cpu_hz,
                target,
                board,
                git_sha,
                build_date,
                build_time,
            ) = v
        else:
            v = _unpack_exact("<4H2I4BI3B32s32s16s12s9s", payload)
            if v is None:
                return None
            (
                version,
                header_size,
                schema_version,
                _flags,
                config_size,
                config_crc32,
                task_module_count,
                high_rate_div,
                compression_enabled,
                build_dirty,
                task_module_mask,
                runtime_device_count,
                motorInstCount,
                controller_count,
                target,
                board,
                git_sha,
                build_date,
                build_time,
            ) = v
            profile_kind = 0
            board_kind = 0
            rtProfCount = 0
            board_can_bus_count = 0
            board_cpu_hz = 0
        return [
            (
                name,
                name,
                {
                    "version": version,
                    "header_size": header_size,
                    "schema_version": schema_version,
                    "config_size": config_size,
                    "config_crc32": f"0x{config_crc32:08X}",
                    "task_module_count": task_module_count,
                    "task_module_mask": f"0x{task_module_mask:08X}",
                    "high_rate_div": high_rate_div,
                    "compression_enabled": compression_enabled,
                    "build_dirty": build_dirty,
                    "runtime_device_count": runtime_device_count,
                    "motorInstCount": motorInstCount,
                    "controller_count": controller_count,
                    "profile_kind": profile_kind,
                    "profile_kind_name": PROFILE_KIND_NAMES.get(profile_kind, f"kind_{profile_kind}"),
                    "board_kind": board_kind,
                    "board_kind_name": BOARD_KIND_NAMES.get(board_kind, f"kind_{board_kind}"),
                    "rtProfCount": rtProfCount,
                    "board_can_bus_count": board_can_bus_count,
                    "board_cpu_hz": board_cpu_hz,
                    "target": _cstr(target),
                    "board": _cstr(board),
                    "git_sha": _cstr(git_sha),
                    "build_date": _cstr(build_date),
                    "build_time": _cstr(build_time),
                },
            )
        ]

    if tag == 0x0052:  # RUNTIME_DEVICE
        return _extract_runtime_device(name, payload)

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

    if tag == 0x004E:  # RT_PROFILER
        if len(payload) < 4:
            return None
        count = payload[0]
        off = 4
        rows: list[tuple[str, str, dict[str, Any]]] = []
        entry_size = struct.calcsize("<B3x6I")
        available = (len(payload) - off) // entry_size
        for _ in range(min(count, available)):
            profiler_id, sample_count, last_us, max_us, avg_us, budget_us, overrun_count = struct.unpack_from("<B3x6I", payload, off)
            off += entry_size
            profiler_name = RT_PROFILER_NAMES.get(profiler_id, f"ID_{profiler_id}")
            series_name = f"{name}.{profiler_name}"
            rows.append(
                (
                    series_name,
                    name,
                    {
                        "id": profiler_id,
                        "count": sample_count,
                        "last_us": last_us,
                        "max_us": max_us,
                        "avg_us": avg_us,
                        "budget_us": budget_us,
                        "overrun_count": overrun_count,
                    },
                )
            )
        return rows

    if tag == 0x004F:  # WHEELLEG_MIT_CONFIG
        return _extract_wheelleg_mit_config(name, payload)

    if tag == 0x0050:  # WHEELLEG_MIT_STATUS
        return _extract_wheelleg_mit_status(name, payload)

    if tag == 0x0053:  # WHEELLEG_MIT_MOTOR_DIAG
        return _extract_wheelleg_mit_motor_diag(name, payload)

    if tag == 0x0042:  # EVENT
        v = _unpack_exact("<HHII", payload)
        if v is None:
            return None
        event_id, arg0_u16, arg1_u32, arg2_u32 = v
        return [(name, name, {"event_id": event_id, "arg0_u16": arg0_u16, "arg1_u32": arg1_u32, "arg2_u32": arg2_u32})]

    if tag == 0x0043:  # VISION_RX
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

    if tag == 0x0044:  # AUX_TUNE
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

        magic, header_size, file_flags, boot_tick_ms, reserved = struct.unpack("<IHHII", hdr0)
        if magic != SDLOG_FILE_MAGIC:
            raise ValueError(f"Bad sdlog magic 0x{magic:08X}")
        if header_size < 16:
            raise ValueError(f"Bad sdlog header_size {header_size}")
        if file_flags != 0:
            raise ValueError(f"Unsupported sdlog file flags 0x{file_flags:04X}")
        if header_size > 16:
            extra = f.read(header_size - 16)
            if len(extra) != (header_size - 16):
                raise ValueError("Truncated sdlog header extension")

        ds = Dataset(source_path=str(path), boot_tick_ms=boot_tick_ms, file_header_size=header_size)

        parser = RecordStreamParser(boot_tick_ms)

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

            if path == "/api/unknown.csv":
                self._send_bytes(HTTPStatus.OK, "text/csv; charset=utf-8", self.dataset.unknown_csv())
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
    ap.add_argument("input", help="Input sdlog_XXXX.bin")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=0, help="0 = choose a free port")
    ap.add_argument("--no-open", action="store_true", help="Do not auto-open the browser")
    args = ap.parse_args(argv)

    path = Path(args.input)
    if not path.exists():
        ap.error(f"file not found: {path}")

    print(f"[sdlog] loading: {path}")
    ds = load_dataset(path)
    print(f"[sdlog] parsed series: {len(ds.series)} (unknown records: {len(ds.unknown_records)})")

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
