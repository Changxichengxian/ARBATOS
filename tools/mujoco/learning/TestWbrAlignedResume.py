"""只验证训练来源关系，不把测试用元信息当作实际训练权重。"""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest

from TrainWbrAligned import resume_lineage


class AlignedResumeTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="wbr-aligned-lineage-")
        self.root = Path(self.temporary.name)
        self.origin = self.root / "promoted-final"
        self.origin.mkdir()
        self.contract = dict(seed=41,training_steps=262144,checkpoint_sha256="unit-test-model-bytes")
        (self.origin / "policy.json").write_text(json.dumps(self.contract),encoding="utf-8")
        self.rule = dict(smoke_seed=9001,screening_first=dict(steps=262144),training_steps=[262144,1048576])
        self.promotion = dict(stage="screen_expand",results=[dict(seed=41,policy=str(self.origin))])

    def tearDown(self):
        self.temporary.cleanup()

    def test_expansion_cannot_start_from_zero(self):
        with self.assertRaisesRegex(RuntimeError,"必须从被晋级"):
            resume_lineage(self.rule,41,None,None,self.promotion,"promotion-hash")

    def test_same_seed_other_short_checkpoint_rejected(self):
        with self.assertRaisesRegex(RuntimeError,"不是从实际晋级"):
            resume_lineage(self.rule,41,self.root / "unreviewed",self.contract,self.promotion,"promotion-hash")

    def test_selected_final_and_its_later_checkpoint_are_accepted(self):
        lineage = resume_lineage(self.rule,41,self.origin,self.contract,self.promotion,"promotion-hash")
        previous = dict(training_lineage=lineage,requested_total_steps=1048576,training_steps=327680)
        actual = resume_lineage(self.rule,41,self.root / "later-checkpoint",previous,self.promotion,"promotion-hash")
        self.assertEqual(actual,lineage)
        wrong = deepcopy(previous)
        wrong["training_lineage"]["origin_checkpoint_sha256"] = "different-origin"
        with self.assertRaisesRegex(RuntimeError,"不是从实际晋级"):
            resume_lineage(self.rule,41,self.root / "other-lineage",wrong,self.promotion,"promotion-hash")

    def test_formal_seed_must_start_fresh_or_resume_same_formal_lineage(self):
        promotion = dict(stage="formal")
        lineage = resume_lineage(self.rule,101,None,None,promotion,"formal-promotion")
        self.assertEqual(lineage["stage"],"formal")
        self.assertEqual(resume_lineage(self.rule,101,self.root / "formal-checkpoint",dict(training_lineage=lineage),promotion,"formal-promotion"),lineage)
        with self.assertRaisesRegex(RuntimeError,"阶段或训练来源"):
            resume_lineage(self.rule,101,self.root / "screen-checkpoint",dict(training_lineage=dict(stage="screen_first",seed=101)),promotion,"formal-promotion")


if __name__ == "__main__":
    unittest.main()
