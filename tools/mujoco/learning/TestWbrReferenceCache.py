"""用错放场景和跨模式结果验证诊断续跑拒绝混用。"""
from copy import deepcopy
import unittest

from DiagnoseWbrReference import encoded_hash, row_binding, validate_saved_row


class ReferenceCacheTests(unittest.TestCase):
    def setUp(self):
        self.scenario = dict(id="development-stop-000",seed=31000,randomize=False,
                             options=dict(case="stop",speed=.35,push=12.,delay=0,duration=8.))
        self.mode = dict(name="brake",action=[.25,0,0])
        self.run = dict(script_sha256="fixed-script",modes=[self.mode])
        row = dict(id=self.scenario["id"],seed=31000,randomize=False,case="stop",
                   options=deepcopy(self.scenario["options"]),metrics=dict(passed=True))
        self.saved = dict(binding=row_binding(self.scenario,self.mode,self.run),
                          result_sha256=encoded_hash(row),result=row)

    def test_exact_cache(self):
        self.assertEqual(validate_saved_row(self.saved,self.scenario,self.mode,self.run),
                         self.saved["result"])

    def test_cross_mode(self):
        wrong = dict(name="velocity",action=[0,0,.5])
        with self.assertRaisesRegex(RuntimeError,"参数、模式或运行身份"):
            validate_saved_row(self.saved,self.scenario,wrong,self.run)

    def test_other_run(self):
        with self.assertRaisesRegex(RuntimeError,"参数、模式或运行身份"):
            validate_saved_row(self.saved,self.scenario,self.mode,dict(script_sha256="other"))

    def test_changed_result_content(self):
        wrong = deepcopy(self.saved)
        wrong["result"]["metrics"]["passed"] = False
        with self.assertRaisesRegex(RuntimeError,"内容哈希"):
            validate_saved_row(wrong,self.scenario,self.mode,self.run)

    def test_other_scenario_even_with_rehashed_content(self):
        for key,value in (("seed",999),("randomize",True),("case","push"),
                          ("options",dict(case="stop",speed=-.35))):
            wrong = deepcopy(self.saved)
            wrong["result"][key] = value
            wrong["result_sha256"] = encoded_hash(wrong["result"])
            with self.subTest(key=key), self.assertRaisesRegex(RuntimeError,"实际场景字段"):
                validate_saved_row(wrong,self.scenario,self.mode,self.run)


if __name__ == "__main__":
    unittest.main()
