"""分析 CaptureBench.py 保存的 UART 基准日志，不访问串口或硬件。

日志字段：
  @IMU,ms,temp_mC,gx_urad_s,gy_urad_s,gz_urad_s,yaw_urad,roll_urad,
       pitch_urad,ax_mms2,ay_mms2,az_mms2,heater_pwm,gyro_cal_done
  @BIAS,ms,bx_urad_s,by_urad_s,bz_urad_s,gyro_calibrating
  @MCU,ms,temp_mC,battery_mV
  @ELRS,ms,port_active,state,lq,rc_frames,link_stats,rc_published,crc_errors,
        length_errors,overflows,restarts,stats_timeouts
"""

import argparse
import csv
import json
import math
import re
from pathlib import Path

import numpy as np


INT32_MIN = -2147483648
RAD_TO_DEG = 180.0 / math.pi
IMU_FIELDS = ("ms", "temp_mC", "gx_urad_s", "gy_urad_s", "gz_urad_s",
              "yaw_urad", "roll_urad", "pitch_urad", "ax_mms2", "ay_mms2",
              "az_mms2", "heater_pwm", "gyro_cal_done")
BIAS_FIELDS = ("ms", "bx_urad_s", "by_urad_s", "bz_urad_s", "gyro_calibrating")
MCU_FIELDS = ("ms", "temp_mC", "battery_mV")
ELRS_FIELDS = ("ms", "port_active", "state", "uplink_lq", "rc_frames",
               "link_stats", "rc_published", "crc_errors", "length_errors",
               "overflows", "restarts", "stats_timeouts")
FIELD_MAP = {
    "IMU": IMU_FIELDS, "BIAS": BIAS_FIELDS, "MCU": MCU_FIELDS, "ELRS": ELRS_FIELDS,
    "SD": ("ms", "active", "bytes_flushed", "dropped", "last_error"),
    "STACK": ("ms", "unused_bytes", "total_bytes"),
    "UART": ("ms", "irq_bytes", "errors", "overrun", "framing", "parity", "noise", "last_error"),
    "INPUT": ("ms", "online", "source", "active_mask", "ch0", "ch1", "ch2", "ch3", "ch4", "s0", "s1"),
    "RC": ("ms", "last_frame_ms", "valid", "sequence") + tuple(f"ch{i}" for i in range(1, 17)),
}


ANSI_ESCAPE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")


def parse_lines(lines):
    records = {name: [] for name in FIELD_MAP}
    rejected = 0
    for raw in lines:
        text = ANSI_ESCAPE.sub("", raw.decode("utf-8", errors="replace"))
        marker = text.find("@")
        if marker < 0:
            continue
        fields = text[marker + 1:].strip().split(",")
        if not fields or fields[0] not in FIELD_MAP or len(fields) != len(FIELD_MAP[fields[0]]) + 1:
            rejected += 1
            continue
        try:
            values = [int(value.strip()) for value in fields[1:]]
        except ValueError:
            rejected += 1
            continue
        records[fields[0]].append(dict(zip(FIELD_MAP[fields[0]], values)))
    return records, rejected


def parse_log(path):
    return parse_lines(path.read_bytes().splitlines())


def monotonic_seconds(rows, tag):
    """返回本次启动内的相对秒；日志混入重启时拒绝把它伪装成 49 天回绕。"""
    if not rows:
        return np.array([], dtype=float)
    stamps = np.asarray([row["ms"] for row in rows], dtype=np.uint64)
    backwards = np.flatnonzero(stamps[1:] < stamps[:-1])
    if backwards.size:
        index = int(backwards[0] + 1)
        raise ValueError(
            f"@{tag} 时间戳在记录 {index} 从 {stamps[index - 1]} 回退到 {stamps[index]}；"
            "日志包含重启或多个启动段，请按启动切分后分别分析。")
    return (stamps - stamps[0]).astype(float) / 1000.0


def interval_summary(times):
    if times.size < 2:
        return {"count": int(times.size), "duration_s": 0.0, "interval_ms": None}
    intervals = np.diff(times) * 1000.0
    return {
        "count": int(times.size),
        "duration_s": float(times[-1]),
        "interval_ms": {
            "min": float(np.min(intervals)), "max": float(np.max(intervals)),
            "mean": float(np.mean(intervals)), "median": float(np.median(intervals)),
        },
    }


def finite_range(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return None
    return {"min": float(np.min(values)), "max": float(np.max(values)), "mean": float(np.mean(values))}


def rolling_reference(values, samples):
    """居中滑动平均足够用于运动筛查；窗口长度由 IMU 实测间隔推导。"""
    if values.shape[0] == 0:
        return values
    samples = max(1, min(samples, values.shape[0]))
    kernel = np.full(samples, 1.0 / samples)
    result = np.empty_like(values, dtype=float)
    for axis in range(values.shape[1]):
        padded = np.pad(values[:, axis], (samples // 2, samples - 1 - samples // 2), mode="edge")
        result[:, axis] = np.convolve(padded, kernel, mode="valid")
    return result


def stationary_segments(times, include):
    """返回连续静止段，任何运动样本或明显采样断档都会切断拟合。"""
    include = np.asarray(include, dtype=bool)
    if not include.size:
        return []
    dt = np.diff(times)
    expected = float(np.median(dt[dt > 0])) if np.any(dt > 0) else 0.1
    max_gap = max(0.5, expected * 5.0)
    segments = []
    start = None
    for index, valid in enumerate(include):
        previous_connected = index == 0 or (times[index] - times[index - 1] <= max_gap)
        if valid and (start is None or not previous_connected):
            if start is not None:
                segments.append(np.arange(start, index))
            start = index
        elif not valid and start is not None:
            segments.append(np.arange(start, index))
            start = None
    if start is not None:
        segments.append(np.arange(start, include.size))
    return [segment for segment in segments if segment.size >= 2]


def linear_drift_deg_min(times, values_deg, include):
    usable = np.asarray(include, dtype=bool) & np.isfinite(times) & np.isfinite(values_deg)
    if np.count_nonzero(usable) < 2:
        return None
    slope_deg_s = np.polyfit(times[usable], values_deg[usable], 1)[0]
    return float(slope_deg_s * 60.0)


def longest_segment_drift(times, angles_deg, stationary):
    segments = stationary_segments(times, stationary)
    if not segments:
        return None, []
    segment = max(segments, key=len)
    include = np.zeros(times.size, dtype=bool)
    include[segment] = True
    return [linear_drift_deg_min(times, angles_deg[:, axis], include) for axis in range(3)], segments


def window_metrics(times, gyro_dps, angles_deg, moving, calibrated, width_s, begin_s, end_s):
    result = []
    start = begin_s
    while start < end_s:
        stop = min(start + width_s, end_s)
        selected = (times >= start) & ((times < stop) if stop < end_s else (times <= stop))
        stationary = selected & ~moving & calibrated
        gyro = gyro_dps[stationary]
        angles = angles_deg[stationary]
        item = {
            "start_s": float(start), "end_s": float(stop),
            "imu_records": int(np.count_nonzero(selected)),
            "stationary_records": int(np.count_nonzero(stationary)),
            "motion_records": int(np.count_nonzero(selected & moving)),
            "uncalibrated_records_excluded": int(np.count_nonzero(selected & ~calibrated)),
            "motion": bool(np.any(selected & moving)), "full_window_angle_drift_valid": False,
            "gyro_mean_dps": None, "gyro_std_dps": None,
            "angle_drift_deg_min": None, "longest_stationary_segment_angle_drift_deg_min": None,
        }
        if gyro.size:
            item["gyro_mean_dps"] = [float(value) for value in np.mean(gyro, axis=0)]
            item["gyro_std_dps"] = [float(value) for value in np.std(gyro, axis=0)]
            drift, segments = longest_segment_drift(times, angles_deg, stationary)
            item["longest_stationary_segment_angle_drift_deg_min"] = drift
            item["longest_stationary_segment_records"] = int(max((len(x) for x in segments), default=0))
            if not item["motion"] and item["uncalibrated_records_excluded"] == 0:
                item["full_window_angle_drift_valid"] = True
                item["angle_drift_deg_min"] = drift
        result.append(item)
        start += width_s
    return result


def analyze(records, rejected, gyro_motion_dps, accel_motion_mps2):
    report = {"rejected_tag_lines": rejected, "records": {}, "fields": {
        name: list(fields) for name, fields in FIELD_MAP.items()}}
    for name, rows in records.items():
        report["records"][name] = interval_summary(monotonic_seconds(rows, name))

    imu = records["IMU"]
    if not imu:
        report["imu"] = None
        return report, []

    times = monotonic_seconds(imu, "IMU")
    temp_c = np.asarray([row["temp_mC"] for row in imu], dtype=float) / 1000.0
    gyro_dps = np.asarray([[row[key] for key in ("gx_urad_s", "gy_urad_s", "gz_urad_s")]
                           for row in imu], dtype=float) * 1e-6 * RAD_TO_DEG
    angles_deg = np.unwrap(np.asarray([[row[key] for key in ("yaw_urad", "roll_urad", "pitch_urad")]
                                       for row in imu], dtype=float) * 1e-6, axis=0) * RAD_TO_DEG
    accel_mps2 = np.asarray([[row[key] for key in ("ax_mms2", "ay_mms2", "az_mms2")]
                              for row in imu], dtype=float) / 1000.0
    heater = np.asarray([row["heater_pwm"] for row in imu], dtype=float)
    cal_done = np.asarray([row["gyro_cal_done"] != 0 for row in imu])
    median_dt = float(np.median(np.diff(times))) if times.size > 1 else 0.1
    ref_samples = max(3, int(round(2.0 / max(median_dt, 0.001))))
    accel_delta = np.linalg.norm(accel_mps2 - rolling_reference(accel_mps2, ref_samples), axis=1)
    gyro_magnitude = np.linalg.norm(gyro_dps, axis=1)
    moving = (gyro_magnitude > gyro_motion_dps) | (accel_delta > accel_motion_mps2)

    last_start = max(0.0, float(times[-1] - 180.0))
    last = (times >= last_start) & ~moving & cal_done
    last_has_motion = bool(np.any((times >= last_start) & moving))
    last_has_uncalibrated = bool(np.any((times >= last_start) & ~cal_done))
    last_metrics = {
        "from_s": last_start, "to_s": float(times[-1]),
        "stationary_records": int(np.count_nonzero(last)),
        "motion_records_excluded": int(np.count_nonzero((times >= last_start) & moving)),
        "uncalibrated_records_excluded": int(np.count_nonzero((times >= last_start) & ~cal_done)),
        "gyro_mean_dps": None, "gyro_std_dps": None, "angle_drift_deg_min": None,
        "full_window_angle_drift_valid": not (last_has_motion or last_has_uncalibrated),
        "longest_stationary_segment_angle_drift_deg_min": None,
    }
    if np.any(last):
        last_metrics["gyro_mean_dps"] = [float(value) for value in np.mean(gyro_dps[last], axis=0)]
        last_metrics["gyro_std_dps"] = [float(value) for value in np.std(gyro_dps[last], axis=0)]
        drift, _ = longest_segment_drift(times, angles_deg, last)
        last_metrics["longest_stationary_segment_angle_drift_deg_min"] = drift
        if not (last_has_motion or last_has_uncalibrated):
            last_metrics["angle_drift_deg_min"] = drift

    mcu_temp_c = np.asarray([row["temp_mC"] for row in records["MCU"]], dtype=float) / 1000.0
    mcu_temp_c[mcu_temp_c == INT32_MIN / 1000.0] = np.nan
    done_indices = np.flatnonzero(cal_done)
    bias_dps = np.asarray([[row[key] for key in ("bx_urad_s", "by_urad_s", "bz_urad_s")]
                           for row in records["BIAS"]], dtype=float).reshape((-1, 3)) * 1e-6 * RAD_TO_DEG
    report["imu"] = {
        "angle_axes": ["yaw", "roll", "pitch"],
        "temperature_c": finite_range(temp_c), "heater_pwm": finite_range(heater),
        "gyro_cal_completed_at_s": float(times[done_indices[0]]) if done_indices.size else None,
        "gyro_cal_completed_device_ms": int(imu[done_indices[0]]["ms"]) if done_indices.size else None,
        "motion_thresholds": {"gyro_dps": gyro_motion_dps, "accel_delta_mps2": accel_motion_mps2},
        "motion_records": int(np.count_nonzero(moving)),
        "stationary_records": int(np.count_nonzero(~moving)),
        "last_180s_stationary": last_metrics,
        "windows_60s": window_metrics(times, gyro_dps, angles_deg, moving, cal_done, 60.0, 0.0, float(times[-1])),
    }
    report["mcu_temperature_c"] = finite_range(mcu_temp_c)
    report["gyro_bias_dps"] = {
        "range_per_axis": [finite_range(bias_dps[:, axis]) for axis in range(3)],
        "last": [float(value) for value in bias_dps[-1]] if bias_dps.size else None,
    }
    report["latest"] = {name: rows[-1] if rows else None for name, rows in records.items()}

    csv_rows = []
    for index, row in enumerate(imu):
        csv_rows.append({
            "time_s": times[index], "temp_c": temp_c[index], "gx_dps": gyro_dps[index, 0],
            "gy_dps": gyro_dps[index, 1], "gz_dps": gyro_dps[index, 2],
            "yaw_deg_unwrapped": angles_deg[index, 0], "roll_deg_unwrapped": angles_deg[index, 1],
            "pitch_deg_unwrapped": angles_deg[index, 2], "ax_mps2": accel_mps2[index, 0],
            "ay_mps2": accel_mps2[index, 1], "az_mps2": accel_mps2[index, 2],
            "heater_pwm": int(heater[index]), "gyro_cal_done": int(cal_done[index]),
            "gyro_magnitude_dps": gyro_magnitude[index], "accel_delta_mps2": accel_delta[index],
            "motion": int(moving[index]),
        })
    return report, csv_rows


def write_csv(path, rows):
    columns = (list(rows[0]) if rows else ["time_s", "temp_c", "gx_dps", "gy_dps", "gz_dps",
               "yaw_deg_unwrapped", "roll_deg_unwrapped", "pitch_deg_unwrapped", "ax_mps2",
               "ay_mps2", "az_mps2", "heater_pwm", "gyro_cal_done", "gyro_magnitude_dps",
               "accel_delta_mps2", "motion"])
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def run_self_test():
    """小型内存样本，覆盖日志清洗、掩码方向、运动切段、重启和空 BIAS。"""
    records, rejected = parse_lines([b"\x1b[32m@MCU,100,25000,12000\x1b[0m"])
    assert rejected == 0 and records["MCU"][0]["temp_mC"] == 25000
    records, rejected = parse_lines([b"@RC,12000,11999,1,100," + b",".join([b"992"] * 16)])
    assert rejected == 0 and records["RC"][0]["last_frame_ms"] == 11999
    assert records["RC"][0]["ch16"] == 992
    times = np.asarray([0.0, 1.0, 2.0, 3.0])
    assert abs(linear_drift_deg_min(times, times, np.array([True] * 4)) - 60.0) < 1e-9
    try:
        monotonic_seconds([{"ms": 1000}, {"ms": 10}], "IMU")
    except ValueError:
        pass
    else:
        raise AssertionError("时间戳重启必须拒绝")
    gyro_urad = int(2.0 / RAD_TO_DEG * 1e6)
    imu = []
    for index in range(8):
        imu.append(dict(zip(IMU_FIELDS, [index * 100, 25000, gyro_urad if index == 0 else 0, 0, 0,
                                          int(index * 0.1 / RAD_TO_DEG * 1e6), 0, 0,
                                          0, 0, 9807, 100, 1])))
    report, rows = analyze({"IMU": imu, "BIAS": [], "MCU": [], "ELRS": []}, 0, 1.0, 0.5)
    assert rows[0]["motion"] == 1
    assert report["imu"]["angle_axes"] == ["yaw", "roll", "pitch"]
    assert report["imu"]["last_180s_stationary"]["full_window_angle_drift_valid"] is False
    assert report["gyro_bias_dps"]["last"] is None
    print("AnalyzeBench self-test: PASS")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, nargs="?", help="CaptureBench.py 产生的原始 UART 日志")
    parser.add_argument("--csv", type=Path, help="IMU 换算和运动标记 CSV，默认与输入同名")
    parser.add_argument("--json", type=Path, help="汇总 JSON，默认与输入同名")
    parser.add_argument("--gyro-motion-dps", type=float, default=1.0,
                        help="陀螺模长超过此值视为运动，默认 1.0")
    parser.add_argument("--accel-motion-mps2", type=float, default=0.5,
                        help="相对两秒局部均值的加速度变化超过此值视为运动，默认 0.5")
    parser.add_argument("--self-test", action="store_true", help="运行内置合成样本，不读取文件")
    args = parser.parse_args()
    if args.self_test:
        run_self_test()
        return
    if args.input is None:
        parser.error("需要指定日志文件，或使用 --self-test")
    if not args.input.is_file():
        parser.error(f"日志不存在: {args.input}")
    if args.gyro_motion_dps <= 0 or args.accel_motion_mps2 <= 0:
        parser.error("运动阈值必须大于 0")
    csv_path = args.csv or args.input.with_suffix(".analysis.csv")
    json_path = args.json or args.input.with_suffix(".analysis.json")
    records, rejected = parse_log(args.input)
    report, rows = analyze(records, rejected, args.gyro_motion_dps, args.accel_motion_mps2)
    write_csv(csv_path, rows)
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"csv": str(csv_path), "json": str(json_path), "records": report["records"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
