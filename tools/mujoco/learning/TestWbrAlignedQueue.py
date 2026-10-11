"""不运行训练，检查对齐训练的检查点来源、完成判据与队列输入绑定。"""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from RunWbrStudyQueue import aligned_protocol, make_job_binding, training_checkpoint, is_complete


class AlignedQueueTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="wbr-aligned-queue-")
        self.root = Path(self.temporary.name)
        self.bank = self.root / "bank.json"
        self.registration = self.root / "registration.json"
        self.config = dict(reward_mode="dense",action_mode="reference",gas_force=450.)
        self.write(self.bank,dict(env_config=self.config))
        self.write(self.registration,dict(weights={"heat":4.},algorithm={"gamma":.999}))
        self.source = dict(library_sha256="test-library",experiment_identity="test-identity")
        self.hashes = {name:"test-"+name for name in ("TrainWbrAligned.py","WbrAlignedEnv.py","WbrAlignedReward.py","TrainWbrStudy.py","TrainWbrLearning.py")}
        self.arguments = ["--output",str(self.root / "target"),"--bank",str(self.bank),"--registration",str(self.registration),
                          "--seed","41","--steps","262144","--envs","2"]

    def tearDown(self):
        self.temporary.cleanup()

    def write(self,path,value):
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(value),encoding="utf-8")

    def checkpoint(self,name,steps=262144):
        directory = self.root / name
        directory.mkdir()
        # 仅测试字节绑定；这些占位字节不能被当作可运行的模型。
        for filename in ("checkpoint.zip","policy.npz"):
            (directory / filename).write_bytes(("binding-test-"+filename).encode())
        run = dict(seed=41,envs=2,env_config=self.config,requested_total_steps=262144,
            training_source_sha256=self.hashes["TrainWbrAligned.py"],training_protocol=aligned_protocol(self.arguments,self.hashes),
            promotion_sha256=None,**self.source)
        contract = dict(run,kind="ppo",training_steps=steps,
            checkpoint_sha256=hashlib.sha256((directory / "checkpoint.zip").read_bytes()).hexdigest(),
            policy_sha256=hashlib.sha256((directory / "policy.npz").read_bytes()).hexdigest())
        self.write(directory / "run.json",run)
        self.write(directory / "policy.json",contract)
        self.write(directory / "status.json",dict(status="completed",steps=steps))
        return directory

    def complete(self,directory):
        return is_complete("aligned_train",directory,self.arguments,self.source,self.hashes["TrainWbrAligned.py"],self.hashes)

    def test_complete_requires_actual_checkpoint_and_network_bytes(self):
        directory = self.checkpoint("complete")
        self.assertTrue(self.complete(directory))
        for filename in ("checkpoint.zip","policy.npz"):
            path = directory / filename
            original = path.read_bytes()
            path.write_bytes(b"changed")
            with self.subTest(filename=filename), self.assertRaisesRegex(RuntimeError,"权重内容"):
                self.complete(directory)
            path.write_bytes(original)

    def test_completed_status_cannot_hide_an_earlier_model(self):
        directory = self.checkpoint("too-early",16384)
        self.write(directory / "status.json",dict(status="completed",steps=262144))
        with self.assertRaisesRegex(RuntimeError,"步数"):
            self.complete(directory)

    def test_resume_uses_older_matching_bank_and_env_count(self):
        older = self.checkpoint("valid",16384)
        for name,key,value in (("wrong-bank","training_protocol",dict(aligned_protocol(self.arguments,self.hashes),bank_sha256="wrong")),
                               ("wrong-envs","envs",1)):
            path = self.checkpoint(name,32768)
            contract = json.loads((path / "policy.json").read_text(encoding="utf-8"))
            contract[key] = value
            self.write(path / "policy.json",contract)
        selected = training_checkpoint([older,self.root / "wrong-bank",self.root / "wrong-envs"],
            self.arguments,self.source,self.hashes["TrainWbrAligned.py"],self.hashes)
        self.assertEqual(selected,older)

    def test_changed_registration_rejects_existing_output(self):
        directory = self.checkpoint("different-rule")
        self.write(self.registration,dict(weights={"heat":0.},algorithm={"gamma":.999}))
        with self.assertRaisesRegex(RuntimeError,"training_protocol"):
            self.complete(directory)

    def test_queue_binds_companions_and_all_training_inputs(self):
        snapshot = self.root / "snapshot"
        learning = snapshot / "tools/mujoco/learning"
        learning.mkdir(parents=True)
        for name in self.hashes:
            (learning / name).write_bytes(name.encode())
        (snapshot / "test.dll").write_bytes(b"test")
        self.write(snapshot / "manifest.json",dict(library="test.dll",library_sha256=hashlib.sha256(b"test").hexdigest(),files={}))
        binding = make_job_binding("aligned_train",self.arguments,dict(learning_directory=str(learning)))
        self.assertEqual(set(binding["payload"]["source_hashes"]),set(self.hashes))
        self.assertEqual(set(binding["payload"]["inputs"]),{"--bank","--registration"})
        self.write(self.bank,dict(env_config=self.config,changed=True))
        changed = make_job_binding("aligned_train",self.arguments,dict(learning_directory=str(learning)))
        self.assertNotEqual(binding["sha256"],changed["sha256"])

    def test_duplicate_promotion_evidence_rejected_before_loading(self):
        from PromoteWbrAligned import evaluate_stage
        with self.assertRaisesRegex(RuntimeError,"两个不同"):
            evaluate_stage(self.registration,[self.root,self.root],self.bank,"screen_expand")

    def test_incomplete_physics_has_a_negative_gate_without_aggregate_numbers(self):
        from PromoteWbrAligned import gate
        report = dict(checks={key:{"pass":False} for key in ("physical_complete","all_case_guards","motion_quality",
            "stop_mean_improvement","push_mean_improvement","hip_heat_mean")})
        result = gate(report,dict(minimum_stop_improvement=.01,minimum_push_improvement=.01,mean_hip_heat_increase_max=.05))
        self.assertFalse(result["pass_all"])
        self.assertIsNone(result["stop_improvement"])


if __name__ == "__main__":
    unittest.main()
