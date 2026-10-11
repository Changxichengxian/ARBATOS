"""拒绝推力前已不同的轨迹、错配缓存和不完整物理记录。"""
from copy import deepcopy
from pathlib import Path
import tempfile
import unittest

from DiagnoseWbrStopPush import no_push_case,pair_metrics,content_hash,check_cache,read_verified,digest


class StopPushTests(unittest.TestCase):
    def setUp(self):
        self.case = dict(id="development-combined-000",seed=31,randomize=True,
                         options=dict(case="combined",speed=.35,push=-12.,delay=0,duration=8.))
        self.forced = dict(metrics=dict(passed=True,hip_heat_nm2s=10.),trace=[
            dict(time_s=4.995,true_v_mps=.1,action=[0,0,0]),
            dict(time_s=5.01,true_v_mps=.3,action=[0,0,0]),
            dict(time_s=5.205,true_v_mps=.2,action=[0,0,0]),
            dict(time_s=7.2,true_v_mps=0.,action=[0,0,0])])
        self.unforced = deepcopy(self.forced)
        self.unforced["trace"][1]["true_v_mps"] = -.1
        self.unforced["trace"][2]["true_v_mps"] = -.2

    def test_only_force_and_id_change(self):
        original = deepcopy(self.case)
        derived = no_push_case(self.case)
        self.assertEqual(self.case,original)
        self.assertEqual(derived["options"]["push"],0.)
        derived["id"] = original["id"]
        derived["options"]["push"] = original["options"]["push"]
        self.assertEqual(derived,original)

    def test_paired_difference_uses_same_sample(self):
        result = pair_metrics(self.forced,self.unforced)
        self.assertTrue(result["complete"])
        self.assertAlmostEqual(result["paired_delta_peak_abs_mps"],.4)
        self.assertAlmostEqual(result["forced_peak_sample"]["unforced_v_mps"],-.1)
        self.assertAlmostEqual(result["paired_delta_rms_mps"],(.32/3.)**.5)

    def test_pre_push_or_clock_mismatch_is_rejected(self):
        for field,value in (("true_v_mps",.100001),("action",[0,.1,0]),("time_s",4.994)):
            other = deepcopy(self.unforced)
            other["trace"][0][field] = value
            with self.subTest(field=field),self.assertRaises(RuntimeError):
                pair_metrics(self.forced,other)

    def test_failure_is_not_an_improvement(self):
        other = deepcopy(self.unforced)
        other["metrics"].update(passed=False,failure="fall")
        result = pair_metrics(self.forced,other)
        self.assertFalse(result["complete"])
        self.assertEqual(result["unforced_failure"],"fall")
        self.assertNotIn("paired_delta_rms_mps",result)

    def test_cache_rejects_corruption_and_wrong_scenario(self):
        scenario = no_push_case(self.case)
        binding = dict(mode="zero",run="hash")
        result = dict(scenario,case="combined",trace=[])
        saved = dict(binding=binding,result=result)
        saved["sha256"] = content_hash(saved)
        self.assertEqual(check_cache(saved,binding,scenario),result)
        with self.assertRaises(RuntimeError):
            check_cache(saved,dict(binding,evaluator_sha256="different"),scenario)
        wrong = deepcopy(saved)
        wrong["result"]["seed"] += 1
        with self.assertRaises(RuntimeError):
            check_cache(wrong,binding,scenario)
        wrong["sha256"] = content_hash({key:value for key,value in wrong.items() if key != "sha256"})
        with self.assertRaises(RuntimeError):
            check_cache(wrong,binding,scenario)

    def test_original_source_is_verified_at_read(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"original.json"
            path.write_text('{"v": 1}',encoding="utf-8")
            expected = digest(path)
            self.assertEqual(read_verified(path,expected),dict(v=1))
            path.write_text('{"v": 2}',encoding="utf-8")
            with self.assertRaises(RuntimeError):
                read_verified(path,expected)


if __name__ == "__main__":
    unittest.main()
