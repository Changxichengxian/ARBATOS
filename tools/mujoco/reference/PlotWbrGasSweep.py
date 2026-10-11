"""汇总气弹簧扫描结果，绘制扭矩平方热负担而非电功率。"""
from __future__ import annotations

import argparse
import csv
import html
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np


MOTOR_NAMES = ("ljoint1", "ljoint4", "rjoint1", "rjoint4")
TITLE = r"四髋扭矩平方热负担 ($\sum_i \tau_{i,\mathrm{RMS}}^2$；Nm$^2$；非电功率)"
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


def load_reports(directories: list[Path]) -> list[dict]:
    """按case合并多个目录，后出现的目录覆盖较早结果。"""
    reports: dict[str, dict] = {}
    for directory in directories:
        summary_path = directory / "summary.json"
        if not summary_path.is_file():
            print(f"跳过缺少 summary.json 的目录：{directory}")
            continue
        summary = json.loads(summary_path.read_text(encoding="utf-8-sig"))
        for report in summary.get("results", []):
            case = report.get("case")
            if case:
                reports[str(case)] = {**report, "_source": str(summary_path)}
    return sorted(reports.values(), key=lambda item: (
        item.get("mode", ""), float(item.get("target_length_m", 0)),
        float(item.get("force_n", 0)), item.get("case", "")))


def limit_fraction(report: dict) -> float:
    values = report.get("hip_limit_fraction", [])
    return max((float(value) for value in values), default=0.)


def has_stop_load(report: dict) -> bool:
    fractions = report.get("stop_contact_fraction", [])
    reactions = report.get("stop_reaction_mean_nm", [])
    return max((float(value) for value in fractions), default=0.) > .01 or \
        max((float(value) for value in reactions), default=0.) > .01


def accepted(report: dict) -> bool:
    eligible = bool(report.get("stable") and report.get("height_within_5mm") and
                    np.isfinite(report.get("hip_heat_index_nm2", np.nan)))
    if report.get("mode") == "stop":
        eligible = eligible and bool(report.get("stop_seated"))
    elif report.get("mode") == "normal":
        eligible = eligible and not has_stop_load(report)
    return eligible and limit_fraction(report) <= .01


def status(report: dict) -> str:
    reasons = []
    if not report.get("stable"):
        reasons.append("失稳/未稳定")
    if not report.get("height_within_5mm"):
        reasons.append("高度不合格")
    if report.get("failure"):
        reasons.append(f"失败:{report['failure']}")
    if report.get("mode") == "stop" and report.get("stable") and not report.get("stop_seated"):
        reasons.append("未持续双侧挡块承重")
    if report.get("mode") == "normal" and has_stop_load(report):
        reasons.append("挡块承重（非自由站立）")
    if limit_fraction(report) > .01:
        reasons.append("无余量（长期平均限幅>1%）")
    return "通过" if not reasons else "、".join(dict.fromkeys(reasons))


def plot_heatmap(reports: list[dict], output: Path) -> None:
    normal = [row for row in reports if row.get("mode") == "normal"]
    forces = sorted({float(row["force_n"]) for row in normal})
    lengths = sorted({float(row["target_length_m"]) for row in normal})
    if not forces or not lengths:
        return
    values = np.full((len(lengths), len(forces)), np.nan)
    rejected = np.zeros(values.shape, dtype=bool)
    for row in normal:
        y = lengths.index(float(row["target_length_m"]))
        x = forces.index(float(row["force_n"]))
        if accepted(row):
            values[y, x] = float(row["hip_heat_index_nm2"])
        else:
            rejected[y, x] = True
    fig, ax = plt.subplots(figsize=(max(7, len(forces) * 1.15), max(4.2, len(lengths) * .8)))
    masked = np.ma.masked_invalid(values)
    cmap = plt.get_cmap("viridis").copy()
    cmap.set_bad("#dedede")
    mesh = ax.pcolormesh(np.arange(len(forces) + 1), np.arange(len(lengths) + 1),
                         masked, cmap=cmap, shading="flat")
    fig.colorbar(mesh, ax=ax, label=r"$\sum_i \tau_{i,\mathrm{RMS}}^2$ (Nm$^2$)")
    ax.set_xticks(np.arange(len(forces)) + .5, [f"{value:g}" for value in forces])
    ax.set_yticks(np.arange(len(lengths)) + .5, [f"{value * 1000:.0f}" for value in lengths])
    ax.set_xlabel("每根气弹簧恒推力（N）")
    ax.set_ylabel("目标腿长（mm）")
    ax.set_title("普通站立｜" + TITLE)
    ax.invert_yaxis()
    for y, length in enumerate(lengths):
        for x, force in enumerate(forces):
            row = next((item for item in normal if float(item["target_length_m"]) == length and
                        float(item["force_n"]) == force), None)
            if row is None:
                continue
            if accepted(row):
                ax.text(x + .5, y + .5, f"{values[y, x]:.1f}", ha="center", va="center", color="white")
            elif np.isfinite(row.get("hip_heat_index_nm2", np.nan)) and \
                    (limit_fraction(row) > .01 or has_stop_load(row)):
                ax.add_patch(patches.Rectangle((x, y), 1, 1, facecolor="#f3c27a", edgecolor="white", zorder=2))
                ax.text(x + .5, y + .5, f"{float(row['hip_heat_index_nm2']):.1f}!", ha="center", va="center",
                        color="#5a2c00", zorder=3)
            else:
                ax.text(x + .5, y + .5, "×", ha="center", va="center", color="#b00020", fontsize=16)
    ax.text(0, -0.22, "橙格带!：挡块承重或限幅无余量；红×：失稳/高度不合格；灰格：无数据。",
            transform=ax.transAxes, fontsize=9, color="#555")
    fig.tight_layout()
    fig.savefig(output / "普通站立热负担热图.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_stop_comparison(reports: list[dict], output: Path) -> None:
    selected = [row for row in reports if abs(float(row.get("target_length_m", 0)) - .18) < 1e-8]
    forces = sorted({float(row["force_n"]) for row in selected})
    if not forces:
        return
    fig, ax = plt.subplots(figsize=(9, 5.2))
    accepted_values = [float(row["hip_heat_index_nm2"]) for row in selected if accepted(row)]
    ax.set_ylim(0, max(accepted_values + [1.]) * 1.15)
    for mode, label, color, marker in (("normal", "普通控制", "#2463a6", "o"),
                                       ("stop", "180 mm 挡块控制", "#d36b16", "s")):
        rows = {float(row["force_n"]): row for row in selected if row.get("mode") == mode}
        xs: list[float] = forces.copy()
        ys: list[float] = []
        for force in forces:
            row = rows.get(force)
            if row is not None and accepted(row):
                ys.append(float(row["hip_heat_index_nm2"]))
            elif row is not None:
                ax.scatter([force], [0], marker="x", color="#b00020", s=65, zorder=4)
                ys.append(np.nan)
            else:
                ys.append(np.nan)
        if any(np.isfinite(ys)):
            ax.plot(xs, ys, label=label, color=color, marker=marker)
    ax.set_xlabel("每根气弹簧恒推力（N）")
    ax.set_ylabel(r"四髋 $\sum_i \tau_{i,\mathrm{RMS}}^2$（Nm$^2$）")
    ax.set_title("180 mm 挡块与普通控制热负担对比")
    ax.grid(True, alpha=.25)
    ax.legend()
    ax.text(0, -0.18, "红叉表示不合格工况，未纳入曲线", transform=ax.transAxes, fontsize=9, color="#555")
    fig.tight_layout()
    fig.savefig(output / "180毫米挡块与普通控制对比.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_motor_rms(reports: list[dict], output: Path) -> None:
    eligible = [row for row in reports if accepted(row)]
    if not eligible:
        return
    colors = ("#2463a6", "#d36b16", "#39844a", "#8254a1")
    groups: dict[tuple[str, float], list[dict]] = {}
    for row in eligible:
        groups.setdefault((str(row["mode"]), float(row["target_length_m"])), []).append(row)
    for (mode, length), rows in groups.items():
        rows.sort(key=lambda row: float(row["force_n"]))
        fig, ax = plt.subplots(figsize=(max(7, len(rows) * 1.15), 5.2))
        values = np.asarray([row.get("hip_rms_nm", [np.nan] * 4) for row in rows], dtype=float)
        x = np.arange(len(rows))
        bar_w = .19
        for motor_index, motor in enumerate(MOTOR_NAMES):
            ax.bar(x + (motor_index - 1.5) * bar_w, values[:, motor_index], bar_w,
                   color=colors[motor_index], label=motor)
        ax.set_xticks(x, [f"{float(row['force_n']):g} N" for row in rows])
        ax.set_ylabel("单电机扭矩 RMS（Nm）")
        ax.set_title(f"四电机扭矩 RMS 对比｜{mode}｜{length*1000:.0f} mm")
        ax.grid(axis="y", alpha=.25)
        ax.legend(ncol=4)
        fig.tight_layout()
        fig.savefig(output / f"四电机RMS对比-{mode}-{length*1000:.0f}mm.png", dpi=180, bbox_inches="tight")
        plt.close(fig)


def write_csv(reports: list[dict], output: Path) -> None:
    per_motor_columns = tuple(f"{metric}_{motor}" for metric in
        ("hip_mean_nm", "hip_rms_nm", "hip_peak_nm", "hip_mechanical_mean_w", "hip_limit_fraction")
        for motor in MOTOR_NAMES)
    wheel_columns = ("wheel_left_electrical_mean_estimate_w", "wheel_right_electrical_mean_estimate_w",
                     "wheel_electrical_total_mean_estimate_w")
    columns = ("case", "mode", "force_n", "target_length_m", "actual_length_mean_m",
               "stable", "height_within_5mm", "stop_seated", "failure", "status",
               "hip_heat_index_nm2", "hip_rms_nm", "hip_peak_nm", "pitch_peak_deg",
               "roll_peak_deg", "speed_rms_mps", "stop_contact_fraction", "stop_reaction_mean_nm",
               "hip_mechanical_mean_w", "hip_mechanical_abs_mean_w", "hip_heat_index_integral_nm2_s_per_hour",
               *per_motor_columns, *wheel_columns, "source")
    with (output / "气弹簧扫描汇总.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        for row in reports:
            flattened = {}
            for metric in ("hip_mean_nm", "hip_rms_nm", "hip_peak_nm", "hip_mechanical_mean_w", "hip_limit_fraction"):
                for motor, value in zip(MOTOR_NAMES, row.get(metric, [])):
                    flattened[f"{metric}_{motor}"] = value
            flattened["hip_heat_index_integral_nm2_s_per_hour"] = (
                float(row["hip_heat_index_nm2"]) * 3600 if row.get("hip_heat_index_nm2") is not None else "")
            wheel_power = row.get("wheel_electrical_mean_w", [])
            flattened["wheel_left_electrical_mean_estimate_w"] = wheel_power[0] if len(wheel_power) > 0 else ""
            flattened["wheel_right_electrical_mean_estimate_w"] = wheel_power[1] if len(wheel_power) > 1 else ""
            flattened["wheel_electrical_total_mean_estimate_w"] = sum(wheel_power) if wheel_power else ""
            values = {**row, **flattened, "status": status(row), "source": row.get("_source", "")}
            for name in columns:
                if isinstance(values.get(name), (list, dict)):
                    values[name] = json.dumps(values[name], ensure_ascii=False)
            writer.writerow(values)


def write_html(reports: list[dict], output: Path) -> None:
    headers = (
        ("case", "工况编号"), ("mode", "控制方式"), ("force_n", "气弹簧推力（N/根）"),
        ("target_length_m", "目标腿长（mm）"), ("actual_length_mean_m", "实际腿长均值（mm）"),
        ("status", "状态"), ("hip_heat_index_nm2", "四髋热负担（Nm²）"),
        ("hip_rms_nm", "四髋扭矩 RMS（Nm）"), ("hip_peak_nm", "四髋扭矩峰值（Nm）"),
        ("stop_contact_fraction", "左右挡块承重比例"),
        ("wheel_electrical_mean_w", "左右轮估算电功率均值（W）"),
        ("failure", "失败原因"), ("source", "来源文件"))
    def fmt(name: str, value) -> str:
        if value is None:
            return ""
        if name == "mode":
            return "挡块控制" if value == "stop" else "普通控制"
        if name in ("target_length_m", "actual_length_mean_m"):
            if isinstance(value, (list, tuple)):
                return "，".join(f"{float(item) * 1000:.1f}" for item in value)
            return f"{float(value) * 1000:.1f}"
        if isinstance(value, (list, tuple)):
            return "，".join(f"{float(item):.3f}" for item in value)
        if isinstance(value, float):
            if name == "force_n":
                return f"{value:.1f}"
            return f"{value:.3f}"
        return str(value)

    rows = []
    for report in reports:
        cells = []
        for name, label in headers:
            if name == "status":
                value = status(report)
            elif name == "source":
                value = report.get("_source", "")
            else:
                value = report.get(name, "")
            cells.append(f"<td>{html.escape(fmt(name, value))}</td>")
        css = "pass" if accepted(report) else "fail"
        mode_value = "挡块" if report.get("mode") == "stop" else "普通"
        length_value = f"{float(report.get('target_length_m', 0)) * 1000:.1f}"
        force_value = f"{float(report.get('force_n', 0)):g}"
        rows.append(f"<tr class='{css}' data-mode='{mode_value}' data-length='{length_value}' data-force='{force_value}'>" +
                    "".join(cells) + "</tr>")
    header_html = "".join(f"<th>{html.escape(label)}</th>" for _, label in headers)
    images = []
    for name, caption in (("普通站立热负担热图.png", "普通站立热负担热图"),
                          ("180毫米挡块与普通控制对比.png", "180 mm 挡块与普通控制对比")):
        if (output / name).is_file():
            images.append(f"<figure><img src='{html.escape(name)}' alt='{html.escape(caption)}'><figcaption>{html.escape(caption)}</figcaption></figure>")
    rms_details = []
    for image_path in sorted(output.glob("四电机RMS对比-*.png")):
        caption = image_path.stem.replace("四电机RMS对比-", "四电机 RMS：").replace("mm", " mm")
        rms_details.append(f"<details><summary>{html.escape(caption)}</summary><img src='{html.escape(image_path.name)}' alt='{html.escape(caption)}'></details>")
    modes = sorted({"挡块" if row.get("mode") == "stop" else "普通" for row in reports})
    lengths = sorted({f"{float(row.get('target_length_m', 0))*1000:.1f}" for row in reports}, key=float)
    forces = sorted({f"{float(row.get('force_n', 0)):g}" for row in reports}, key=float)
    def options(items):
        return "".join(f"<option value='{html.escape(item)}'>{html.escape(item)}</option>" for item in items)
    page = f"""<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>气弹簧扫描数据报告</title>
<style>body{{font:14px system-ui,sans-serif;margin:24px;color:#202124}}h1{{font-size:24px}}h2{{margin-top:32px}}p{{color:#555;line-height:1.6}}.charts{{display:grid;grid-template-columns:repeat(auto-fit,minmax(380px,1fr));gap:16px}}figure{{margin:0;padding:8px;border:1px solid #ddd;border-radius:6px}}figure img,details img{{display:block;max-width:100%;height:auto}}figcaption{{text-align:center;padding:6px}}table{{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums;display:block;overflow:auto;white-space:nowrap}}th,td{{border:1px solid #ccc;padding:6px 8px;text-align:left}}th{{position:sticky;top:0;background:#eee}}.pass{{background:#f2f8f2}}.fail{{background:#fff0f0}}.filters{{display:flex;gap:12px;flex-wrap:wrap;margin:12px 0}}select{{padding:5px}}details{{margin:8px 0;padding:8px;border:1px solid #ddd}}a.button{{display:inline-block;padding:8px 12px;background:#245b91;color:white;text-decoration:none;border-radius:4px}}</style></head><body>
<h1>气弹簧扫描数据报告</h1><p>本页展示扫描数据，不给出选簧结论。ΣτRMS² 单位为 Nm²，是扭矩平方热负担指标，不是电功率；每小时累计为 ΣτRMS²×3600（Nm²·s/h），也不是能量或瓦数。髋机械功率根据测得扭矩与角速度计算；髋电流和耗电待转矩常数与电流定义确认。轮功率为现有 3508 估算模型结果，不是电池实测。</p>
<p><a class="button" href="气弹簧扫描汇总.csv" download>下载 CSV 汇总</a></p><section class="charts">{''.join(images)}</section><h2>四电机 RMS 分腿长图</h2>{''.join(rms_details)}
<h2>逐工况明细</h2><div class="filters"><label>控制方式 <select id="mode"><option value="">全部</option>{options(modes)}</select></label><label>目标腿长（mm） <select id="length"><option value="">全部</option>{options(lengths)}</select></label><label>气弹簧推力（N/根） <select id="force"><option value="">全部</option>{options(forces)}</select></label></div>
<table><thead><tr>{header_html}</tr></thead><tbody>{''.join(rows)}</tbody></table>
<script>const filters=['mode','length','force'].map(id=>document.getElementById(id));function applyFilter(){{const [mode,length,force]=filters.map(el=>el.value);document.querySelectorAll('tbody tr[data-mode]').forEach(row=>{{row.hidden=!!((mode&&row.dataset.mode!==mode)||(length&&row.dataset.length!==length)||(force&&row.dataset.force!==force));}});}}filters.forEach(el=>el.addEventListener('change',applyFilter));</script></body></html>"""
    (output / "气弹簧扫描完整汇总.html").write_text(page, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, nargs="+", required=True,
                        help="包含 summary.json 的扫描目录，靠后的目录按case覆盖前者")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    reports = load_reports(args.input)
    if not reports:
        parser.error("所有输入目录都没有可汇总的 results")
    plot_heatmap(reports, args.output)
    plot_stop_comparison(reports, args.output)
    plot_motor_rms(reports, args.output)
    write_csv(reports, args.output)
    write_html(reports, args.output)
    print(f"汇总 {len(reports)} 个case，结果写入 {args.output.resolve()}")


if __name__ == "__main__":
    main()
