"""验证动作中人工停止锁存、旧复位不能解锁、新的显式复位才恢复。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import threading
import time

import numpy as np

from RunWbr import ROOT
from TestWbrViewerReset import ResetSimulation, close_thread, wait_state


class StopSimulation(ResetSimulation):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.stop_samples = []

    def _step_physics_locked(self):
        if self.motor_stop_latched:
            self.stop_samples.append((float(self.data.time), self.data.ctrl.copy()))
        super()._step_physics_locked()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'local/cache/wbr-output-stop')
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    sim = StopSimulation(args.output, terrain='course', guide_rollers='passive')
    sim.location = 'playground'
    sim.command([], client_id='driver', claim_control=True)
    thread = threading.Thread(target=sim.run_test, daemon=True)
    thread.start()
    try:
        wait_state(sim, lambda s:s.get('rise_phase')==2 and s.get('sim_time',0)>.5, seconds=15)
        before = sim.data.ctrl.copy()
        assert sim.command([], client_id='observer', emergency_stop=True)
        stopped = wait_state(sim, lambda s:s.get('motor_stop_latched') and not s.get('running'), seconds=5)
        assert np.max(np.abs(before)) > 1
        assert stopped['physics_running']
        sim.command(['w'], client_id='driver')
        wait_state(sim, lambda s:s.get('sim_time',0)>stopped['sim_time']+.4, seconds=5)
        try:
            sim.command([], reset=True, reset_mode='recover')
            raise AssertionError('旧复位请求不能解除停止')
        except ValueError:
            pass
        assert sim.motor_stop_latched
        assert not sim.command([], reset=True, reset_mode='global',
            clear_emergency_stop=True, stop_generation=sim.motor_stop_generation,
            client_id='observer')
        assert sim.motor_stop_latched
        old_generation = sim.motor_stop_generation
        sim.command([], emergency_stop=True)
        current_generation = sim.motor_stop_generation
        owner_before_rejected_recovery = sim.control_owner
        try:
            sim.command([], reset=True, controller='upstream', reset_mode='recover',
                        clear_emergency_stop=True, stop_generation=current_generation,
                        client_id='observer', claim_control=True)
            raise AssertionError('原作者控制器的 R 不能解除输出停止')
        except ValueError:
            pass
        assert sim.motor_stop_latched and sim.motor_stop_generation == current_generation
        assert sim.control_owner == owner_before_rejected_recovery
        assert not sim.reset_requested
        try:
            sim.command([], reset=True, reset_mode='global', clear_emergency_stop=True,
                        stop_generation=old_generation)
            raise AssertionError('再次停止后，迟到的解除请求不能恢复输出')
        except ValueError:
            pass
        assert sim.motor_stop_latched
        settled = sim.stop_samples
        assert settled and all(np.array_equal(values,np.zeros(6)) for _,values in settled)
        last_zero = settled[-1][0]
        sim.command([], reset=True, reset_mode='global', clear_emergency_stop=True,
                    stop_generation=sim.motor_stop_generation, client_id='driver')
        ready = wait_state(sim, lambda s:s.get('own_ready') and s.get('running') and
                           not s.get('motor_stop_latched'), seconds=15)
        assert ready.get('limited_recovery_supported') is True
        result = dict(passed=True, before_stop_peak_nm=float(np.max(np.abs(before))),
            checked_zero_frames=len(settled), zero_window_s=[settled[0][0],last_zero],
            physics_continued=True, stale_recovery_rejected=True, stale_clear_rejected=True,
            upstream_recovery_cannot_clear_stop=True, upstream_recovery_cannot_claim_control=True,
            nonowner_can_stop=True, nonowner_cannot_clear_without_claim=True,
            explicit_t_restored=ready['own_ready'],
            boundary='软件停止使六路执行器归零，重力和被动气弹簧仍按物理模型作用')
    finally:
        close_thread(sim, thread)
    (args.output/'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
