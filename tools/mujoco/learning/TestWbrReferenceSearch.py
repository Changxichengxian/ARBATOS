"""确认搜索不会用平均收益抵消物理、热量、姿态或运动质量退步。"""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from EvaluateWbrStudy import policy_artifacts
from RunWbrStudyQueue import policy_artifacts as queue_artifacts, is_complete
from SearchWbrLinear import reference_score, state_hash, validate_rows
from StudyWbrPolicy import StudyPolicy, reference_action
from TestWbrStudyMetrics import result
from TestWbrMotionQuality import row as quality_row


class ReferenceSearchTests(unittest.TestCase):
    def setUp(self):
        self.base = [result("s","stop",.2,100.),result("p","push",.3,100.)]
        self.better = [result("s","stop",.17,104.),result("p","push",.33,104.)]
        for rows in (self.base,self.better):
            for row in rows:
                row["motion_quality"] = quality_row(row["id"],row["case"])["motion_quality"]

    def test_zero_and_complete_improvement(self):
        self.assertEqual(reference_score(self.base,deepcopy(self.base))[0],0.)
        score,report = reference_score(self.base,self.better)
        self.assertGreater(score,1.)
        self.assertTrue(report["overall_pass"])

    def test_safety_cannot_be_bought_by_mean_improvement(self):
        for violation in ("heat","roll","sideways","short_drive","missing_quality"):
            wrong = deepcopy(self.better)
            if violation == "heat":
                for row in wrong:
                    row["metrics"]["hip_heat_nm2s"] = 106.
            elif violation == "roll":
                wrong[0]["metrics"]["peak_roll_1ms_deg"] = 5.
            elif violation == "sideways":
                wrong[1]["motion_quality"]["max_lateral_m"] = .1
            elif violation == "short_drive":
                wrong[0]["motion_quality"]["progress_at_stop_m"] = .5
            else:
                wrong[0].pop("motion_quality")
            with self.subTest(violation=violation):
                score,report = reference_score(self.base,wrong)
                self.assertLess(score,0.)
                self.assertFalse(report["overall_pass"])

    def test_one_regression_cannot_beat_zero(self):
        wrong = deepcopy(self.better)
        wrong[0]["metrics"] = result("s","stop",.2000001,104.)["metrics"]
        score,report = reference_score(self.base,wrong)
        self.assertLess(score,0.)
        self.assertFalse(report["overall_pass"])

    def test_physical_failure_is_worst(self):
        wrong = deepcopy(self.better)
        wrong[0] = result("s","stop",0.,1.,passed=False)
        self.assertLess(reference_score(self.base,wrong)[0],-1e8)

    def test_policy_and_artifact_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            data = dict(kind="reference_constant",coefficients=[.08,-.5,.5],
                        env_config=dict(action_mode="reference"))
            (root/"policy.json").write_text(json.dumps(data),encoding="utf-8")
            policy = StudyPolicy(root)
            np.testing.assert_array_equal(policy.predict(np.zeros(93)),reference_action(data["coefficients"]))
            self.assertEqual(policy_artifacts(root),queue_artifacts(root)[1])
            data["env_config"]["action_mode"] = "symmetric"
            (root/"policy.json").write_text(json.dumps(data),encoding="utf-8")
            with self.assertRaises(ValueError):
                StudyPolicy(root)
        for invalid in ([-.1,0,0],[0,2,0],[0,float("nan"),0],[0,0]):
            with self.subTest(invalid=invalid),self.assertRaises(ValueError):
                reference_action(invalid)

    def test_resume_rejects_other_scenario(self):
        case = dict(id="x",seed=31,randomize=True,options=dict(case="stop",speed=.5))
        cached = dict(case="stop",**case)
        validate_rows([cached],[case])
        cached = dict(cached,seed=32)
        with self.assertRaises(RuntimeError):
            validate_rows([cached],[case])
        state = dict(generation=0,cache={"a":[1]})
        original = state_hash(state)
        state["state_sha256"] = original
        self.assertEqual(state_hash(state),original)
        state["cache"]["a"].append(2)
        self.assertNotEqual(state_hash(state),original)

    def test_queue_rejects_corrupt_state_or_different_export(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cases = root/"cases.json"
            cases.write_text(json.dumps(dict(cases_sha256="cases")),encoding="utf-8")
            arguments = ["--cases",str(cases),"--family","reference","--generations","1","--population","4"]
            source = dict(library_sha256="dll",experiment_identity="identity")
            binding = dict(seed=41,family="reference",scenarios_sha256="cases",
                           env_config=dict(reward_mode="dense",action_mode="reference",gas_force=450.),
                           generations=1,population=4,source_hashes={"search":"sha"},**source)
            state = dict(binding=binding,status="completed",generation=1,
                         best=dict(coefficients=[0.,-.5,0.],score=.01))
            state["state_sha256"] = state_hash(state)
            policy = dict(binding,kind="reference_constant",coefficients=[0.,-.5,0.],development_score=.01)
            def write(name,value):
                (root/name).write_text(json.dumps(value),encoding="utf-8")
            write("search-state.json",state)
            write("status.json",dict(status="completed"))
            write("policy.json",policy)
            self.assertTrue(is_complete("search",root,arguments,source,"sha",binding["source_hashes"]))
            state["best"]["coefficients"][0] = .1
            write("search-state.json",state)
            with self.assertRaisesRegex(RuntimeError,"哈希"):
                is_complete("search",root,arguments,source,"sha",binding["source_hashes"])
            state["state_sha256"] = state_hash(state)
            write("search-state.json",state)
            with self.assertRaisesRegex(RuntimeError,"导出策略"):
                is_complete("search",root,arguments,source,"sha",binding["source_hashes"])


if __name__ == "__main__":
    unittest.main()
