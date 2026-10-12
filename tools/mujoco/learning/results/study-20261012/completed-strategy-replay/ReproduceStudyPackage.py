"""验证包内文件并实际重跑已评价过的三类场景，不选择或训练新策略。"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

root=Path(__file__).resolve().parent
read=lambda p:json.loads(p.read_text(encoding="utf-8"))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def require(ok,message):
    if not ok:
        raise RuntimeError(message)

def local(relative):
    path=(root/relative).resolve()
    require(path.is_relative_to(root),"包内路径越界")
    return path

def check_inventory():
    inventory=read(root/"交付清单.json")
    require(all(sha(local(name))==value for name,value in inventory["files"].items()),"包内源文件、模型或报告发生变化")
    return inventory

def replay(label):
    inventory=check_inventory()
    selected=[row for row in inventory["policies"] if row["label"]==label]
    require(len(selected)==1,"策略名称不存在或重复")
    spec=selected[0]
    snapshot=local(spec["snapshot"])
    sys.path.insert(0,str(snapshot/"tools/mujoco/learning"))
    from WbrLearningEnv import verify_snapshot,experiment_identity
    from EvaluateWbrStudy import load_cases,run_case
    from StudyWbrPolicy import StudyPolicy
    manifest=verify_snapshot()
    contract=read(local(spec["policy"])/"policy.json")
    require(contract["experiment_identity"]==experiment_identity(manifest)
        and contract["library_sha256"]==manifest["library_sha256"],"策略与冻结物理版本不符")
    policy=StudyPolicy(local(spec["policy"]))
    model_check=None
    if policy.kind=="ppo":
        import numpy as np
        import torch
        from stable_baselines3 import PPO
        torch.set_num_threads(1)
        model=PPO.load(local(spec["policy"])/"checkpoint.zip",device="cpu")
        require(model.num_timesteps==contract["training_steps"] and len(model.policy.optimizer.state_dict()["state"])==13,"实际模型步数或优化器不符")
        inputs=np.random.default_rng(712341).normal(0,.5,(128,93)).astype(np.float32)
        expected,_=model.predict(inputs,deterministic=True)
        actual=np.asarray([policy.predict(row) for row in inputs])
        error=float(np.max(np.abs(actual-expected)))
        require(np.isfinite(actual).all() and error<2e-5,"保存的网络与导出推理不符")
        model_check=dict(actual_steps=model.num_timesteps,optimizer_states=13,export_max_abs_error=error)
        del model
    cases=load_cases(local(spec["cases"]))
    require(cases["split"] in ("validation","aligned-monitor","trace-monitor"),"只重放已完成的验证或监控，不运行最终集")
    summary=read(local(spec["summary"]))
    require(summary["scenarios_sha256"]==cases["cases_sha256"] and summary["split"]==cases["split"],"保存报告与场景不符")
    require(summary["experiment_identity"]==experiment_identity(manifest) and summary["env_config"]==contract["env_config"],"保存报告与物理环境不符")
    chosen=[]
    for kind in ("stop","push","combined"):
        candidates=sorted([row for row in cases["cases"] if row["options"]["case"]==kind],key=lambda row:row["id"])
        randomized=[row for row in candidates if row["randomize"]]
        require(bool(candidates),"缺少一种场景")
        chosen.append((randomized or candidates)[0])
    require([row["id"] for row in chosen]==spec["replay_case_ids"],"重放选例不是登记的各类首个随机场景")
    results=[]
    started=time.monotonic()
    for scenario in chosen:
        for controller in ("baseline","learned"):
            rows={row["id"]:row for row in summary[controller+"_results"]}
            reference=rows[scenario["id"]].copy()
            require(reference.pop("controller")==controller,"保存结果控制器标签不符")
            actual=run_case(scenario,None if controller=="baseline" else policy,contract["env_config"])
            exact=actual==reference
            results.append(dict(controller=controller,id=scenario["id"],all_fields_exact=exact,actual=actual,expected=reference))
            print(json.dumps(dict(policy=label,case=scenario["id"],controller=controller,all_fields_exact=exact),ensure_ascii=False),flush=True)
    output=root/"复现输出"
    output.mkdir(exist_ok=True)
    target=output/(label+".json")
    require(not target.exists(),"已有这组复现结果；保留原记录后另选解压目录重跑")
    result=dict(schema=1,checked_at=datetime.now().astimezone().isoformat(),label=label,
        status="passed" if all(row["all_fields_exact"] for row in results) else "failed",
        model_check=model_check,scenes=len(chosen),trajectories=len(results),elapsed_s=time.monotonic()-started,
        original_summary_sha256=sha(local(spec["summary"])),package_inventory_sha256=sha(root/"交付清单.json"),
        results=results,boundary="同机换目录重放已评价场景，逐字段比较；不是新独立性能评价，也不是另一电脑或实车验证。")
    target.write_bytes((json.dumps(result,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))
    require(result["status"]=="passed","实际重放有字段不同，完整差异已保存")

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--one",help="仅重放指定已保存策略")
    args=parser.parse_args()
    if args.one:
        replay(args.one)
        return
    inventory=check_inventory()
    labels=[row["label"] for row in inventory["policies"]]
    require(len(labels)==len(set(labels)),"策略名称重复")
    require(not (root/"复现输出").exists(),"复现输出已存在；请使用新的解压目录保留本次运行边界")
    for label in labels:
        subprocess.run([sys.executable,str(Path(__file__)) ,"--one",label],check=True)
    rows=[read(root/"复现输出"/(label+".json")) for label in labels]
    require(all(row["status"]=="passed" for row in rows),"不是所有模型均复现通过")
    result=dict(schema=1,status="passed",checked_at=datetime.now().astimezone().isoformat(),
        policies=labels,trajectories=sum(row["trajectories"] for row in rows),
        all_fields_exact=True,inventory_sha256=sha(root/"交付清单.json"),
        result_files={label+".json":sha(root/"复现输出"/(label+".json")) for label in labels},
        boundary="只覆盖各策略已评价过的停车/受推/组合三例及对应原控制，不声称整套独立评价已在另一目录全部重跑。")
    (root/"复现输出/汇总.json").write_bytes((json.dumps(result,ensure_ascii=False,indent=2)+"\n").encode("utf-8"))
    print(json.dumps(result,ensure_ascii=False),flush=True)

if __name__=="__main__":
    main()
