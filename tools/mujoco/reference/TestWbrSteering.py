"""真实网页物理线程：前进、倒车和制动过零时的 A/D 转向。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import threading
import time

from RunWbr import ROOT
from TestWbrViewerReset import ResetSimulation, close_thread, wait_state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'local/cache/wbr-steering-direction')
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    sim = ResetSimulation(args.output, terrain='course', guide_rollers='passive')
    sim.location, sim.drive_speed, sim.turn_speed = 'playground', .6, .8
    thread = threading.Thread(target=sim.run_test, daemon=True)
    thread.start()
    rows = []

    def run(keys, duration, label, stop=False):
        with sim.lock:
            start = sim.state['sim_time']
        deadline = time.monotonic()+duration*4+10
        while time.monotonic() < deadline:
            sim.command(keys, stop=stop)
            with sim.lock:
                state = dict(sim.state)
            if sim.thread_error or not state.get('running', True):
                raise AssertionError(sim.thread_error or state.get('status'))
            row = {k:state.get(k) for k in ('sim_time','v_mps','yaw_rate_radps',
                'command_yaw','steering_reverse','pitch_deg','roll_deg')}
            row['label'] = label
            # 状态约每 33 ms 发布一次；不把发键前的旧状态归到新动作。
            if state['sim_time']-start > .05:
                rows.append(row)
            if state['sim_time']-start >= duration:
                return
            time.sleep(.025)
        raise AssertionError(f'{label} 未按期完成')

    try:
        wait_state(sim, lambda s:s.get('own_ready') and s.get('sim_time',0)>3, seconds=20)
        run(['w'], 3.5, 'forward')
        run(['w','a'], 2.0, 'forward_a')
        run([], 3.0, 'stop', stop=True)
        run(['s'], 3.5, 'reverse')
        run(['s','a'], 2.0, 'reverse_a')
        run(['w','a'], 4.0, 'brake_to_forward')
        run([], 3.0, 'finish', stop=True)
    finally:
        close_thread(sim, thread)
        (args.output/'states.json').write_text(json.dumps(rows, ensure_ascii=False), encoding='utf-8')
    forward = [r for r in rows if r['label']=='forward_a' and r['v_mps']>.25]
    reverse = [r for r in rows if r['label']=='reverse_a' and r['v_mps']<-.25]
    braking_back = [r for r in rows if r['label']=='brake_to_forward' and r['v_mps']<-.25]
    braking_front = [r for r in rows if r['label']=='brake_to_forward' and r['v_mps']>.25]
    checks = dict(forward_a_left=bool(forward) and all(r['command_yaw']>0 for r in forward)
                  and max(r['yaw_rate_radps'] for r in forward)>.4,
        reverse_a_head_right=bool(reverse) and all(r['command_yaw']<0 for r in reverse)
                  and min(r['yaw_rate_radps'] for r in reverse)<-.4,
        brake_keeps_reverse=bool(braking_back) and all(r['command_yaw']<0 for r in braking_back),
        forward_after_zero=bool(braking_front) and all(r['command_yaw']>0 for r in braking_front))
    result = dict(passed=all(checks.values()), checks=checks,
        forward_yaw_max=max((r['yaw_rate_radps'] for r in forward), default=None),
        reverse_yaw_min=min((r['yaw_rate_radps'] for r in reverse), default=None),
        command='按实际估计行驶方向切换，低于0.03m/s时按当前前后请求选择，0.03–0.08m/s保持方向')
    (args.output/'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
