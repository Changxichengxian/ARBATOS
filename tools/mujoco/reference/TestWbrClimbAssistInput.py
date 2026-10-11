"""验证上台准备的输入归属、松键、超时和停止边界；不替代物理动作验收。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import threading
import time

from RunWbr import ROOT
from ViewOwnWbr import LiveSimulation
from TestWbrViewerReset import ResetSimulation, close_thread, wait_state


def check_physics(output):
    sim = ResetSimulation(output)
    delivered = []
    original_setter = sim.own_lib.OwnWbrSetClimbAssist

    def observe_setter(handle, pressed):
        result = original_setter(handle, pressed)
        delivered.append((float(sim.data.time), pressed, result))
        return result

    sim.own_lib.OwnWbrSetClimbAssist = observe_setter
    sim.command([], client_id='driver', claim_control=True)
    thread = threading.Thread(target=sim.run_test, daemon=True)
    thread.start()
    try:
        wait_state(sim, lambda s: s.get('own_ready') and s.get('running'), seconds=20)
        for release in ('keyup', 'timeout', 'stop', 'reset', 'x'):
            sim.command(['g'], client_id='driver')
            wait_state(sim, lambda s: s.get('climb_assist_applied') is True)
            marker = len(delivered)
            if release == 'keyup':
                sim.command([], client_id='driver')
            elif release == 'stop':
                sim.command([], client_id='driver', stop=True)
            elif release == 'reset':
                sim.command([], client_id='driver', reset=True)
            elif release == 'x':
                sim.command([], client_id='driver', emergency_stop=True)
            wait_state(sim, lambda s: s.get('climb_assist_applied') is False)
            assert any(value == 0 and result == 1 for _, value, result in delivered[marker:]), release
            if release == 'reset':
                wait_state(sim, lambda s: s.get('own_ready') and s.get('running'), seconds=20)
        assert sim.motor_stop_latched
        assert (sim.data.ctrl == 0).all()
    finally:
        close_thread(sim, thread)
    return dict(real_setter_calls=len(delivered), passed=True,
                release_paths=['keyup', 'timeout', 'stop', 'reset', 'x'],
                boundary='真实物理线程及真实 C 接口；原地按 G，不替代上台动作验收')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'local/cache/wbr-climb-assist-input')
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    sim = LiveSimulation(args.output, terrain='course', guide_rollers='passive')
    assert sim.climb_assist_api, '当前控制器尚未接入上台准备'
    # 这里只检查输入收集，用明确的已就绪状态绕过起身；物理动作由独立测试验收。
    sim.own_ready = True
    sim.command(['g', 'w'], client_id='driver', claim_control=True)
    sim.inputs()
    assert sim.climb_assist_requested and sim.keys == {'g', 'w'}
    sim.command(['g', 'w', ' '], client_id='driver')
    sim.inputs()
    assert sim.keys == {'g', 'w'} and sim.jump_until == 0
    assert not sim.command([], client_id='observer')
    assert sim.keys == {'g', 'w'}
    sim.command(['w'], client_id='driver')
    sim.inputs()
    assert not sim.climb_assist_requested and sim.keys == {'w'}
    sim.command(['g'], client_id='driver')
    sim.input_at = time.monotonic() - .36
    sim.inputs()
    assert not sim.climb_assist_requested and not sim.keys
    sim.command(['g'], client_id='driver', stop=True)
    sim.inputs()
    assert not sim.climb_assist_requested and not sim.keys
    sim.command(['g'], client_id='driver')
    sim.command([], client_id='observer', emergency_stop=True)
    sim.command(['g', 'w'], client_id='driver')
    sim.inputs()
    assert sim.motor_stop_latched and not sim.climb_assist_requested
    sim.command(['g'], reset=True, reset_mode='global', clear_emergency_stop=True,
                stop_generation=sim.motor_stop_generation, client_id='driver')
    sim.inputs()
    assert not sim.climb_assist_requested and not sim.keys
    sim.reset_in_progress = False
    for controller, stage, ready, location in (
            ('upstream', 3, True, 'start'), ('own', 2, True, 'start'),
            ('own', 3, False, 'start'), ('own', 3, True, 'step_climb'),
            ('own', 3, True, 'large_steps')):
        sim.controller, sim.stage, sim.own_ready, sim.location = controller, stage, ready, location
        sim.command(['g'], client_id='driver')
        sim.inputs()
        assert not sim.climb_assist_requested and 'g' not in sim.keys
    result = dict(passed=True, held_and_released=True, nonowner_cannot_clear=True,
                  timeout_clears=True, stop_clears=True, motor_stop_blocks=True,
                  reset_clears=True, unsupported_modes_discard=True,
                  simultaneous_jump_discarded=True,
                  boundary='仅输入语义，不代表能平稳上台或实车安全验收')
    result['physics_input'] = check_physics(args.output/'physics')
    (args.output/'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
