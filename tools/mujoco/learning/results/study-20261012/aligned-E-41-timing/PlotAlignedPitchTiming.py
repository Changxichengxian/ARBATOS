import hashlib
import json
import math
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

workspace=Path(__file__).resolve().parents[1]
base=workspace/"work/study-20261012/aligned-reference/pitch-timing-diagnostic"
read=lambda p:json.loads(p.read_text(encoding="utf-8"))
record=read(base/"时序核验.json")
baseline=read(base/"原控制重放.json")
learned=read(base/"最终策略重放.json")
font_manager.fontManager.addfont(r"C:\Windows\Fonts\msyh.ttc")
plt.rcParams["font.family"]=font_manager.FontProperties(fname=r"C:\Windows\Fonts\msyh.ttc").get_name()
plt.rcParams["axes.unicode_minus"]=False
fig,axes=plt.subplots(1,2,figsize=(12.8,5.5))
for row,label,color in ((baseline,"原控制","#648296"),(learned,"E41最终策略","#b36e46")):
    axes[0].plot([v["time_s"] for v in row["trace"]],
        [abs(math.degrees(v["pitch_rad"])) for v in row["trace"]],label=label,color=color,lw=1.6)
axes[0].axhline(record["sampled_threshold_deg"],color="#9e4242",ls="--",label="原1ms峰值＋0.2°容差")
first=record["first_sampled_exceedance"]
axes[0].axvline(first["time_s"],color="#c0a049",ls=":",label="首次可见超限 3.78s")
axes[0].axvline(record["terminal_time_s"],color="#4a5261",ls=":",label="末尾罚分 8.01s")
axes[0].set(xlabel="回合时间（秒）",ylabel="绝对俯仰角（度）",title="训练场景 push-008 的实际重放")
axes[0].legend(fontsize=9,loc="lower right")
lag=np.arange(0,535)
axes[1].semilogy(lag*.015,(.999*.97)**lag,label="当前 λ=0.97",color="#b36e46",lw=2)
axes[1].semilogy(lag*.015,.999**lag,label="λ=1 数值参照",color="#648296",lw=2)
axes[1].scatter([first["seconds_before_terminal_penalty"]],[first["isolated_direct_terminal_coefficient"]],color="#9e4242",zorder=5)
axes[1].annotate("超限到末尾相隔4.23秒\n直接系数约0.00014",xy=(4.23,first["isolated_direct_terminal_coefficient"]),
    xytext=(1.9,2e-6),fontsize=9,arrowprops=dict(arrowstyle="->",color="#9e4242"))
axes[1].set(xlabel="距末尾罚分的时间（秒）",ylabel="隔离单位信号的直接系数（对数轴）",title="实际训练库验证的直接误差传播")
axes[1].legend(fontsize=10,loc="upper right")
for ax in axes:
    ax.grid(alpha=.18)
    ax.spines[["top","right"]].set_visible(False)
fig.suptitle("罚分时序诊断：线索成立，尚未证明它导致性能退步",fontsize=16,y=.97)
fig.text(.02,.11,"选例仅使用训练日志；最终确定性策略重放，不等同于原随机训练回合，也不是独立性能验证。",fontsize=10)
fig.text(.02,.045,"图中姿态按15ms采样；验收仍使用1ms峰值。直接系数未包含价值预测及其他奖励，λ=1也可能增加波动。",fontsize=10,color="#8c4942")
fig.tight_layout(rect=(0,.18,1,.91))
output=workspace/"outputs/轮腿罚分时序诊断.png"
fig.savefig(output,dpi=160)
data=dict(source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in base.iterdir() if p.is_file()},
    figure_sha256=hashlib.sha256(output.read_bytes()).hexdigest(), plot_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
(base/"图表来源.json").write_bytes((json.dumps(data,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))
print(output)
