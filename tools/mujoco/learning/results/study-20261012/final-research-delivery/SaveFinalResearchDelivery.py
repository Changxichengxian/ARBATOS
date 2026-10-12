"""完整12份策略实际复现通过后，保存最终负结果交付并暂存研究文件。"""
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import zipfile

workspace = Path(__file__).resolve().parents[1]
repo = Path(r"D:\ARBATOS")
learning = repo / "tools/mujoco/learning"
results = learning / "results/study-20261012"
package = workspace / "work/final-package"
read = lambda p: json.loads(p.read_text(encoding="utf-8"))
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def require(ok, message):
    if not ok:
        raise RuntimeError(message)


def git(*args, input=None):
    return subprocess.run(["git", "-C", str(repo), *args], input=input,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True).stdout


def save(path, value):
    path.write_bytes((json.dumps(value, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))


require(Path(git("rev-parse", "--show-toplevel").decode().strip()).resolve() == repo, "仓库不符")
require(git("branch", "--show-current").strip() == b"main" and not git("diff", "--cached", "--name-only").strip(), "分支或索引需要核查")
summary_root = results / "六方案筛选汇总"
summary = read(summary_root / "六方案筛选汇总.json")
require(summary["negative_branch_supported"] and not summary["final_dataset_used"]
    and summary["completed_control_records"] == 1008, "未达到负结果收尾条件")
require(all(sha(Path(name)) == value for name, value in summary["source_files"].items()), "最终汇总来源改变")
inventory = read(package / "交付清单.json")
proof = read(package / "复现输出/汇总.json")
environment = read(package / "实际执行环境.json")
require(inventory["include_trace"] and {r["label"] for r in inventory["policies"]} ==
    {f"{route}-{seed}" for route in "ABCDEF" for seed in (41, 73)}, "最终包策略集合不符")
require(proof["status"] == "passed" and proof["all_fields_exact"] and proof["trajectories"] == 72,
    "12份策略真实重放未全部通过")
require(proof["inventory_sha256"] == environment["inventory_sha256"] == sha(package / "交付清单.json"), "执行来源不符")
require(environment["exit_code"] == 0 and environment["different_python_environment"]
    and Path(environment["prefix"]).resolve() == (workspace / "work/package-clean-env").resolve(), "实际环境核验不符")
require(environment["source_sha256"] == sha(workspace / "work/RunFinalPackageReplay.py")
    and inventory["builder_sha256"] == sha(workspace / "work/BuildStudyDeliveryPackage.py"), "实际入口改变")
require(all(sha(package / name) == value for name, value in inventory["files"].items()), "重放后包内文件改变")
require(len(proof["result_files"]) == 12, "重放结果数量不符")
for name, value in proof["result_files"].items():
    path = package / "复现输出" / name
    row = read(path)
    require(sha(path) == value and row["package_inventory_sha256"] == sha(package / "交付清单.json")
        and row["status"] == "passed" and row["trajectories"] == 6
        and all(r["actual"] == r["expected"] and r["all_fields_exact"] for r in row["results"]), "重放逐字段证据不符")
require((package / "evidence/六方案筛选汇总/六方案筛选汇总.json").read_bytes() ==
    (summary_root / "六方案筛选汇总.json").read_bytes(), "打包的研究结论不符")
target = results / "final-research-delivery"
archive = workspace / "work/轮腿研究最终复现包.zip"
output = workspace / "outputs/轮腿研究最终复现包.zip"
require(not target.exists() and not archive.exists() and not output.exists(), "最终交付已经保存")
doc = "tools/仿真说明.md"
head = git("show", "HEAD:" + doc)
require(git("show", ":" + doc) == head, "说明索引已有其他修改")
working = (repo / doc).read_bytes()
marker = "#### 六方案负结果收尾与完整复现交付（2026-10-12）"
require(marker.encode("utf-8") not in working, "收尾说明已经存在")
entries = {name: package / name for name in inventory["files"]}
entries["交付清单.json"] = package / "交付清单.json"
entries["实际执行环境.json"] = package / "实际执行环境.json"
entries.update({"已执行核验/" + p.name: p for p in (package / "复现输出").glob("*.json")})
with zipfile.ZipFile(archive, "x", zipfile.ZIP_DEFLATED, compresslevel=9) as z:
    for name, source in sorted(entries.items()):
        z.write(source, name)
with zipfile.ZipFile(archive) as z:
    require(z.testzip() is None and len(z.namelist()) == len(entries), "压缩包损坏或数量不符")
    require(all(hashlib.sha256(z.read(name)).hexdigest() == sha(source) for name, source in entries.items()), "压缩包内容与实测目录不同")
target.mkdir()
for name in ("BuildStudyDeliveryPackage.py", "ReproduceStudyPackage.py", "RunFinalPackageReplay.py", "SaveFinalResearchDelivery.py"):
    shutil.copyfile(workspace / "work" / name, target / name)
for p in (package / "复现输出").glob("*.json"):
    shutil.copyfile(p, target / p.name)
for name in ("交付清单.json", "实际执行环境.json"):
    shutil.copyfile(package / name, target / name)
shutil.copyfile(archive, target / archive.name)
shutil.copyfile(archive, output)
require(sha(archive) == sha(output) == sha(target / archive.name), "保存后的归档不同")
now = datetime.now().astimezone().isoformat()
record = dict(schema=1, saved_at=now, status="passed", policy_count=12, replay_trajectories=72,
    all_fields_exact=True, independent_python_environment=True, same_computer=True,
    archive_sha256=sha(archive), archive_bytes=archive.stat().st_size, archive_entries=len(entries),
    inventory_sha256=sha(package / "交付清单.json"), replay_proof_sha256=sha(package / "复现输出/汇总.json"),
    environment_proof_sha256=sha(package / "实际执行环境.json"), source_script_sha256=sha(Path(__file__)),
    archive=(target / archive.name).relative_to(repo).as_posix(), final_dataset_used=False,
    conclusion="六类有限配置均未通过筛选，按原研究目标负结果分支交付",
    boundary="同机新Python环境和独立目录，每份策略三例及对应原控制。没有将全45例/24例重新跑一遍，也没有另一电脑或实车验收。")
save(target / "最终交付核验.json", record)
progress_path = learning / "研究进度.json"
progress_bytes = progress_path.read_bytes()
progress = json.loads(progress_bytes.decode("utf-8"))
progress.update(updated_at=now, status="research_complete_negative_result",
    active_process=dict(kind="none", queue_pid=None, trainer_pid=None, updated_at=now),
    final_dataset_used=False, final_delivery=record,
    next="研究已按负结果分支完成，停止定时跟进；后续新模型或新算法研究需另行安排。")
progress["completed"].append("六类方案筛选负结果已形成带来源总表。最终12份模型在全新独立Python环境中真实重放72条历史轨迹，逐字段一致；含暂停重跑、四锁、新24例全部记录及冻结环境的最终包逐项字节核验通过。")
require(progress_path.read_bytes() == progress_bytes, "进度文件被其他任务修改，停止写入")
save(progress_path, progress)
appendix = f"""{marker}

六类有限配置均未满足各自登记筛选条件，已有超过原目标要求的三类有区别方案对照，按负结果分支结束本轮研究。12份最终策略共14组比较、504次候选策略的场景评价和1008条原控制/候选记录；三套评价场景分开统计，不是504个独立场景。A/B/C额外运动质量未评价，不能写成通过。最终60例仍未使用，三个正式训练种子未运行；这里是筛选阶段负结果，不是最终验收失败，也不证明强化学习整体无效、已收敛或更大预算无效。

完整结果、失败来源、瓶颈和下一步见 `learning/results/study-20261012/六方案筛选汇总/`。冻结对象为参考机械模型与ARBATOS控制核心，不能直接当作实际MINIWHEELEG-M的实车效果。未接入正式控制、未连接实车。

在同机独立Python3.12.14环境安装35个固定依赖后，最终12份策略分别实际重放停车、受推、组合及对应原控制，共72条历史轨迹逐字段一致。该检查不替代全量独立评价，也未验证另一电脑。最终包含{len(inventory['files'])}份清单文件，以及清单、实际环境和13份运行核验记录；共{len(entries)}项、{archive.stat().st_size}字节，SHA256 `{sha(archive)}`。包内全部字节与实测目录一致；旧运行记录存于`已执行核验`，用户重新运行写入新的`复现输出`。

最终归档及核验见 `learning/results/study-20261012/final-research-delivery/`。其中保留F73暂停前42496步记录、原种子从零重跑证明、四份最终模型锁、新24例计划、共同原控制及全部完成记录。按包内使用说明可用相对路径运行，历史绝对路径仅记录来源。至此停止本轮训练和自动跟进。
"""
require((repo / doc).read_bytes() == working, "说明被其他任务修改，停止写入")
expected = working.rstrip(b"\r\n") + b"\n\n" + appendix.encode("utf-8")
(repo / doc).write_bytes(expected)
blob = git("hash-object", "-w", "--stdin", input=head.rstrip(b"\r\n") + b"\n\n" + appendix.encode("utf-8")).decode().strip()
git("update-index", "--cacheinfo", f"100644,{blob},{doc}")
paths = [p.relative_to(repo).as_posix() for directory in (target, summary_root) for p in directory.rglob("*") if p.is_file()]
paths.append(progress_path.relative_to(repo).as_posix())
git("add", "--pathspec-from-file=-", "--pathspec-file-nul", input=("\0".join(paths) + "\0").encode("utf-8"))
actual = set(git("diff", "--cached", "--name-only", "-z").decode("utf-8").strip("\0").split("\0"))
require(actual <= {doc, *paths} and doc in actual, "索引混入其他工作")
require(all(git("show", ":" + p) == (repo / p).read_bytes() for p in paths), "暂存文件不同")
git("-c", "core.whitespace=blank-at-eol,space-before-tab,cr-at-eol,-blank-at-eof", "diff", "--cached", "--check")
validation_text = f"\n最终交付已验证：同机独立Python环境，12份策略共72条历史轨迹逐字段一致。压缩包SHA-256：{sha(archive)}。\n"
(workspace / "outputs/轮腿研究完整结论.txt").write_bytes((summary_root / "研究结论与下一步.txt").read_bytes() + validation_text.encode("utf-8"))
print(git("-c", "core.quotepath=false", "diff", "--cached", "--stat").decode("utf-8"))
