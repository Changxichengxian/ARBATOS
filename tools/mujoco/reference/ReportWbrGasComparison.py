"""汇总已复核的站立、扰动与动作记录，不混用热指标和电功率。"""
import html
import json
import os
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SWEEP = ROOT / "local/cache/wbr-gas-sweep-20261011"
ACTIONS = ROOT / "local/cache/wbr-gas-actions-20261012"
OUT = SWEEP / "report"
FORCES = (300, 350, 400, 450)
MOTORS = ("ljoint1", "ljoint4", "rjoint1", "rjoint4")
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


def read(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def link(path, label):
    if not path.is_file():
        raise FileNotFoundError(path)
    return '<a href="{}">{}</a>'.format(html.escape(Path(os.path.relpath(path, OUT)).as_posix()), html.escape(label))


def table(headers, rows):
    head = "".join(f"<th>{h}</th>" for h in headers)
    body = "".join("<tr>" + "".join(f"<td>{cell}</td>" for cell in row) + "</tr>" for row in rows)
    return f'<div class="scroll"><table><thead><tr>{head}</tr></thead><tbody>{body}</tbody></table></div>'


def heat(energy):
    return sum(axis["torque_squared_integral_nm2s"] for axis in energy["hips"].values())


def rms(values):
    return " / ".join(f"{value:.3f}" for value in values)


def plot(push, actions):
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), layout="constrained")
    x = np.arange(4)
    ax = axes[0, 0]
    for shift, direction, label, color in ((-.18, 20, "前向推力后", "#247ba0"), (.18, -20, "后向推力后", "#ed9b40")):
        bars = ax.bar(x + shift, [push[(f, direction)]["hip_heat_index_nm2"] for f in FORCES], .36, label=label, color=color)
        ax.bar_label(bars, fmt="%.2f", fontsize=9, padding=3)
    ax.set(title="180 mm 挡块卸载：扰动后持有", ylabel="四髋平均扭矩平方 H（Nm²）")
    ax.legend(frameon=False)
    for ax, action, title in ((axes[0, 1], "rise", "低趴起身：统一 22 秒窗口"), (axes[1, 0], "climb200", "200 mm 上台：统一 22 秒窗口")):
        bars = ax.bar(x, [heat(actions[(action, f)]["energy"]) for f in FORCES], .62, color="#247ba0")
        for bar, force in zip(bars, FORCES):
            if not actions[(action, force)]["passed"]:
                bar.set_hatch("///")
                bar.set_edgecolor("#a53b26")
        ax.bar_label(bars, fmt="%.0f", fontsize=10, padding=3)
        ax.set(title=title, ylabel="四髋热积分（Nm²·s）")
        ax.text(.02, .94, "斜纹：起身初段触限，完整流程未通过", transform=ax.transAxes, va="top", fontsize=9, color="#a53b26")
    ax = axes[1, 1]
    for shift, action, label, color in ((-.18, "jump100", "小跳（高度偏高）", "#ed9b40"), (.18, "jump250", "大跳", "#247ba0")):
        bars = ax.bar(x + shift, [heat(actions[(action, f)]["jump_event_energy"]) for f in FORCES], .36, label=label, color=color)
        ax.bar_label(bars, fmt="%.0f", fontsize=9, padding=3)
    ax.set(title="跳跃：启动至落稳的单次事件窗", ylabel="四髋热积分（Nm²·s）")
    ax.legend(frameon=False)
    for ax in axes.flat:
        ax.set_xticks(x, [str(f) for f in FORCES])
        ax.set_xlabel("每根气弹簧推力（N）")
        ax.set_ylim(0, ax.get_ylim()[1] * 1.2)
        ax.spines[["top", "right"]].set_visible(False)
        ax.grid(axis="y", alpha=.16)
        ax.set_axisbelow(True)
    fig.suptitle("同模型、同轨迹下的气弹簧比较\nH 与热积分用于相对铜耗比较，均不是电功率或电能", fontsize=17)
    path = OUT / "气弹簧工况总览.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    pushes = read(SWEEP / "approach-validation-summary.json")["results"]
    push = {(int(r["force_n"]), int(r["horizontal_push_n"])): r for r in pushes}
    raw = read(ACTIONS / "jumps-final/summary.json")["rows"] + read(ACTIONS / "rise-climb-final/summary.json")["rows"]
    actions = {(r["action"], int(r["gas_force_n_each"])): r for r in raw}
    rise_check = {r["force_n"]: r for r in read(ACTIONS / "起身稳定复核.json")["results"]}
    figure = plot(push, actions)
    push_rows, rise_rows, climb_rows, jump_rows, hip_rows = [], [], [], [], []
    for force in FORCES:
        for sign, direction in ((20, "向前"), (-20, "向后")):
            row = push[(force, sign)]
            push_rows.append([force, direction, rms(row["hip_rms_nm"]), f'{row["hip_heat_index_nm2"]:.3f}', "通过" if row["stable"] and row["stop_seated"] else "未通过"])
        rise = actions[("rise", force)]
        check = rise_check[force]
        rise_rows.append([force, f'{rise["rise_completed_s"]:.3f}', f'{check["first_two_second_stable_window_s"][0]:.3f}',
            f'{rise["rise_max_displacement_m"]*1000:.1f}', f'{check["full_22s_max_displacement_m"]*1000:.1f}',
            "通过本轮门槛" if rise["passed"] else "初段触伸腿端限位，待处理"])
        climb = actions[("climb200", force)]
        energy = climb["energy"]
        climb_rows.append([force, f'{climb["preimpact_true_speed_mps"]:.3f}', f'{climb["phase7_time_s"]-climb["impact_time_s"]:.3f}',
            f'{climb["climb_terminal_stable_duration_s"]:.2f}', "完成" if climb["climb_completed"] else "未完成",
            "通过" if climb["passed"] else "起身触限，上台阶段完成",
            f'{max(h["longest_above_10nm_s"] for h in energy["hips"].values()):.3f}'])
        for action, name in (("rise", "起身22秒"), ("climb200", "上台22秒"), ("jump100", "小跳事件窗"), ("jump250", "大跳事件窗")):
            row = actions[(action, force)]
            energy = row.get("jump_event_energy") or row["energy"]
            axes = [energy["hips"][m] for m in MOTORS]
            folder = "jumps-final" if action.startswith("jump") else "rise-climb-final"
            source = ACTIONS / folder / f"{action}-F{force}" / "action-summary.json"
            hip_rows.append([force, name, f'{energy["recorded_window_s"]:.3f}', rms([h["rms_nm"] for h in axes]),
                rms([h["peak_nm"] for h in axes]), f'{heat(energy):.1f}',
                f'{energy["wheel_estimated_positive_electrical_wh"]:.5f}', link(source, "原始摘要")])
        for action, name in (("jump100", "100 mm小跳"), ("jump250", "250 mm大跳")):
            row = actions[(action, force)]
            jump_rows.append([force, name, f'{row["jump_clearance_m"]*1000:.1f}',
                f'{row["jump_airborne_s"]*1000:.0f}', "完成并落稳" if row["action_completed_stable_budget_passed"] else "未完成",
                "通过" if row["height_target_band_passed"] else "超过90～110 mm目标带"])
    parts = ["""<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>气弹簧选型总览</title><style>
body{font:16px/1.7 system-ui,'Microsoft YaHei',sans-serif;color:#24333e;max-width:1280px;margin:40px auto;padding:0 24px;background:#fafbfc}h1{font-size:32px;margin-bottom:6px}h2{margin-top:36px;font-size:23px}p{max-width:1080px}a{color:#146686}.callout{padding:18px 22px;background:#eaf3f6;border-left:5px solid #247ba0;border-radius:4px}.warn{background:#fff3df;border-color:#c88629}.scroll{overflow:auto;margin:16px 0}table{border-collapse:collapse;min-width:100%;background:white;white-space:nowrap;font-size:14px;font-variant-numeric:tabular-nums}th,td{padding:10px 12px;text-align:left;border-bottom:1px solid #dce3e8}th{background:#edf1f4}tr:nth-child(even){background:#f7f9fb}img{width:100%;height:auto}small{color:#60717c}
</style><h1>气弹簧选型总览</h1><small>180 mm实体挡块 · 自有控制器 · 2026-10-12复核版</small>
<p class="callout"><b>已做动态试验的300～450 N中，下一轮优先试300 N/根，350 N留作对照。</b> 挡块真正分担重量时，两者久站负担相近；300 N完成起身、约1.5 m/s上200 mm标准高台和大跳，且本轮起身没有碰伸腿端限位。350～450 N能完成动作，但起身最初触限，完整流程不能标成通过。250 N只做了静态，尚未验证动态能力，不能宣称已排除。当前结果支持候选范围，不是唯一最优或实车定案。</p>
<p>共72组站立、8组收腿加前后扰动、16组动作。久站为主要用途，跳跃按用户描述为极少发生；未编造动作频率或将各项目等权平均。每档每动作只有一个确定性标准场景，不能推算随机地形成功率。</p>
<p class="callout warn"><b>不能只换弱弹簧，保留原来的承重控制。</b> 普通180 mm站立时，350 N的四髋热指标为22.94，450 N为0.119。弱簧的省力结果来自实验中的挡块卸载：维持平衡控制，但让挡块分担重量。正式控制器尚未加入卸载与动作之间的在线切换，各项测试独立初始化，网页与固件参数也未改。</p>""",
        f'<img src="{figure.name}" alt="气弹簧在久站和动作中的热负担比较图">',
        "<h2>180 mm挡块久站：受到前后推力以后</h2><p>从220 mm收至180 mm，15秒时施加20 N水平力0.2秒，统计20～80秒。四髋顺序为左1、左4、右1、右4。八组都稳住、双轮承载、双侧挡块受压。300～350 N的每髋有效扭矩约0.8～1.0 Nm；450 N有一对约2.1～2.2 Nm。同方向对照的四髋铜耗比例指标约降低60%～70%，不是整车电耗降低同样比例。</p>",
        table(['N/根','推力方向','四髋有效扭矩 Nm','四髋 H（Nm²）','挡块承重验证'],push_rows),
        "<h2>低趴起身：程序结束和收稳分开看</h2><p>起身程序约1.75秒退出，此时仍有约4°俯仰及0.15 m/s后移。低速稳定区间要求连续2秒姿态角小于5°、前后速度小于0.1 m/s、两轮法向载荷均大于5 N，横向漂移小于20 mm。该门槛仍允许缓慢移动；完整22秒最大位移28～31 cm，尚未实现此前希望的约1秒、小位移起身。</p>",
        table(['N/根','程序结束 s','低速稳定区间起点 s','程序内最大位移 mm','22秒最大位移 mm','完整起身检查'],rise_rows),
        "<h2>200 mm标准高台</h2><p>四档上台动作都能完成。完整流程将准备阶段的低趴起身一并检查；350～450 N的问题发生在起身初段，上台阶段没有非承重挡块触限。四档都达到过45 Nm髋峰值和5.2 Nm轮端峰值，没有峰值余量。3.5秒只是一项仿真动作预算，不是厂家保证的热安全时间；尚未验收结构冲击，也没有证明任意翻倒恢复或RMUL所有台阶都能通过。</p>",
        table(['N/根','碰撞前实际速度 m/s','碰撞到台顶阶段 s','末段台顶稳定 s','上台动作','含起身完整流程','最长连续髋扭矩>10 Nm s'],climb_rows),
        "<h2>跳跃：少用也要保留能力</h2><p>从独立200 mm直立闭链姿态开始，大跳目标带240～270 mm。小跳都能完成并落稳，但实际约115 mm，超过100±10 mm，严格高度验收不通过；这项偏差需要调整动作，不能靠换气弹簧当作已经解决。</p>",
        table(['N/根','动作','轮底净空 mm','双轮离地 ms','动作能力','高度目标'],jump_rows),
        "<h2>四髋扭矩与动作负担明细</h2><p>有效扭矩按同一窗口计算。起身和上台用完整22秒，包含准备、动作及等待；跳跃用启动至双轮落地后连续回稳2秒的事件窗。不能把22秒当成净单次动作能耗，失败场景的较小能耗也不能用于优选。</p>",
        table(['N/根','窗口','时长 s','四髋有效扭矩 Nm','四髋峰值 Nm','合计热积分 Nm²·s','两轮正功率估计 Wh','数据'],hip_rows),
        """<h2>耗电口径与本次更正</h2><p>H是四个电机有效扭矩的平方和，单位Nm²；动作热积分∫Hdt的单位是Nm²·s。相同电机、温度和电流定义下，它们用于比较铜耗。6216P输出扭矩常数及相电流/电阻定义未确认，目前不能可靠换算髋电机绝对W、Wh或温升。静止时机械功率接近零也会耗电。两轮Wh来自仓库3508拟合模型，不能与髋热指标相加，也不当作电池实测或回充。</p>
<p>保留180 mm物理挡块、参考1.8 Nm髋静摩擦、同质量与导轮；气弹簧按每腿一根恒推力近似，尚无实物推力随压缩量/温度变化的曲线。70秒站立统计不代表电机达到热平衡。动态仅在缓存控制器副本中放宽气簧参数准入，其他控制逻辑同源；正式控制器仍拒绝非450 N跳跃档，尚不能直接启用这些参数。</p>
<p>复核修正了上台积分计时：旧汇总在加载时缓存2 ms，实际运行1 ms，导致积分与持续时间多算一倍。现已按原始22,000条电机记录重算，轨迹、RMS、峰值和四档顺序不变。旧汇总保留为每组action-summary.before-audit.json。另补查了此前被排除的起身初段限位，350～450 N完整流程的通过标记已更正。</p><h2>完整数据与复跑入口</h2><p>""",
        " · ".join(link(path,label) for path,label in ((OUT/'气弹簧扫描完整汇总.html','所有高度与档位明细'),(OUT/'气弹簧扫描汇总.csv','静态CSV汇总'),(ACTIONS/'jumps-final/summary.json','跳跃汇总'),(ACTIONS/'rise-climb-final/summary.json','起身和上台复核汇总'),(ACTIONS/'起身稳定复核.json','起身稳定复核'),(ACTIONS/'起身限位复核.json','起身限位复核'),(ROOT/'tools/仿真说明.md','试验说明'))),
        "</p></html>"]
    output = OUT / "气弹簧选型总览.html"
    output.write_text("\n".join(parts), encoding="utf-8")
    print(output)
    print(figure)


if __name__ == "__main__":
    main()
