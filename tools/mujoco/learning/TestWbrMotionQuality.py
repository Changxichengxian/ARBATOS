"""用明确的反例验证少跑、侧移及偏航不会通过新增质量门。"""
from copy import deepcopy
import json

from EvaluateWbrStudy import motion_quality_acceptance
from SummarizeWbrStudy import screening_gate


def row(case_id="stop-1", case="stop"):
    quality = dict(max_lateral_m=.002,max_yaw_deg=.1,
        windows={case:dict(world_vx_rms_mps=.2,plane_speed_rms_mps=.201)})
    if case == "stop":
        quality.update(drive_tracking_rms_mps=.1,steady_drive_tracking_rms_mps=.05,
                       progress_at_stop_m=1.)
    return dict(id=case_id,case=case,metrics={"passed":True},motion_quality=quality)


def main():
    baseline = [row(),row("push-1","push")]
    assert motion_quality_acceptance(baseline,deepcopy(baseline))["pass"]
    for label,edit in (
        ("less_travel",lambda rows:rows[0]["motion_quality"].update(progress_at_stop_m=.9)),
        ("poor_drive",lambda rows:rows[0]["motion_quality"].update(drive_tracking_rms_mps=.12)),
        ("poor_steady_drive",lambda rows:rows[0]["motion_quality"].update(steady_drive_tracking_rms_mps=.07)),
        ("sideways",lambda rows:rows[1]["motion_quality"]["windows"]["push"].update(plane_speed_rms_mps=.3)),
        ("yaw",lambda rows:rows[1]["motion_quality"].update(max_yaw_deg=1.)),
        ("drift",lambda rows:rows[1]["motion_quality"].update(max_lateral_m=.01)),
    ):
        learned = deepcopy(baseline)
        edit(learned)
        assert not motion_quality_acceptance(baseline,learned)["pass"],label
    missing = deepcopy(baseline)
    missing[0].pop("motion_quality")
    assert not motion_quality_acceptance(baseline,missing)["pass"]
    from EvaluateWbrStudy import acceptance
    from TestWbrStudyMetrics import result
    original = [result("s","stop",.2,100.),result("p","push",.3,100.)]
    better = [result("s","stop",.17,104.),result("p","push",.33,104.)]
    primary_report = acceptance(original,better)
    assert primary_report["overall_pass"]
    assert not screening_gate(primary_report)["pass_all"]
    primary_report["checks"]["motion_quality"] = motion_quality_acceptance(baseline,deepcopy(baseline))
    assert screening_gate(primary_report)["pass_all"]
    print(json.dumps(dict(zero_change_pass=True, six_false_improvements_rejected=True,
                         missing_quality_evidence_rejected=True, promotion_requires_quality=True)))


if __name__ == "__main__":
    main()
