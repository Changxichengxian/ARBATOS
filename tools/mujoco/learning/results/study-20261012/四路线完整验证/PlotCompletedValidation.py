import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

workspace = Path(__file__).resolve().parents[1]
base = Path(r"D:\ARBATOS\tools\mujoco\learning\results\study-20261012")
rows = []
for label,relative in (("六维修正 · 41","screening-A-full-41-evaluated"),
                       ("六维修正 · 73","screening-A-full-73-evaluated"),
                       ("对称动作 · 41","screening-B-symmetric-41"),
                       ("对称动作 · 73","screening-B-symmetric-73"),
                       ("线性修正 · 41","linear-C-41-validation"),
                       ("线性修正 · 73","linear-C-73-validation"),
                       ("常数参数 · 41","reference-D-41-validation"),
                       ("常数参数 · 73","reference-D-73-validation")):
    path = base / relative / "summary.json"
    report = json.loads(path.read_text(encoding="utf-8"))
    checks = report["acceptance"]["checks"]
    if len(report["learned_results"]) != 45 or report["split"] != "validation":
        raise RuntimeError("必须使用完整45例独立验证")
    rows.append(dict(label=label,source=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        stop=100*checks["stop_mean_improvement"]["improvement_fraction"],
        push=100*checks["push_mean_improvement"]["improvement_fraction"],
        heat=100*checks["hip_heat_mean"]["change_fraction"]))
font_manager.fontManager.addfont(r"C:\Windows\Fonts\msyh.ttc")
plt.rcParams["font.family"] = font_manager.FontProperties(fname=r"C:\Windows\Fonts\msyh.ttc").get_name()
plt.rcParams["axes.unicode_minus"] = False
fig,axes = plt.subplots(1,3,figsize=(13.6,6.4),sharey=True)
colors = ["#607d9a","#607d9a","#5a9593","#5a9593","#8a79a8","#8a79a8","#ac784b","#ac784b"]
for axis,key,title in zip(axes,("stop","push","heat"),("停车误差降低（%）","受推误差降低（%）","髋部热指标增加（%）")):
    values = [row[key] for row in rows]
    axis.barh(np.arange(len(rows)),values,color=colors,height=.56)
    axis.axvline(0,color="#627079",lw=.9)
    axis.axvline(5,color="#bf514d",linestyle="--",lw=1.2)
    axis.set_xlim(min(-2.4,min(values)-1.7),max(15.2 if key == "heat" else 11.1,max(values)+1.7))
    axis.set_title(title,fontsize=12,pad=14)
    axis.grid(axis="x",alpha=.2)
    axis.spines[["top","right"]].set_visible(False)
    for index,value in enumerate(values):
        axis.text(value+(.18 if value >= 0 else -.18),index,f"{value:+.2f}",va="center",
                  ha="left" if value >= 0 else "right",fontsize=10)
axes[0].set_yticks(np.arange(len(rows)),[row["label"] for row in rows],fontsize=11)
axes[0].invert_yaxis()
fig.suptitle("已完成的45例独立验证：平均收益与髋部用力负担",fontsize=16,y=.975)
fig.text(.02,.145,"每行45例：停车/受推均值各用18个单项场景，9个组合场景参加逐例保护；热指标用全部45例。",fontsize=10)
fig.text(.02,.090,"虚线：停车、受推的5%筛选收益；热指标的5%最终上限。正式最终验收要求两项改善各至少10%。",fontsize=10)
fig.text(.02,.035,"平均收益还须同时满足逐例、姿态及用力负担保护；不能单凭图中均值采用。最终60例仍未使用。",fontsize=10,color="#8c4942")
fig.tight_layout(rect=(0,.205,1,.92))
target = workspace / "outputs/轮腿已完成独立验证对照.png"
fig.savefig(target,dpi=160)
data = workspace / "work/study-20261012/completed-validation-plot-data.json"
data.write_bytes((json.dumps(dict(rows=rows,figure_sha256=hashlib.sha256(target.read_bytes()).hexdigest()),ensure_ascii=False,indent=2)+"\n").encode("utf-8"))
print(str(target))
