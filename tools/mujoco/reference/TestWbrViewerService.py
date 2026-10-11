"""通过网页接口验收实际前后转向、高清切换和仿真记录下载。"""
from __future__ import annotations

import argparse
import io
import json
from pathlib import Path
import time
import urllib.error
import urllib.request
import zipfile

from PIL import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', default='http://127.0.0.1:8773')
    parser.add_argument('--output', type=Path, default=Path('local/cache/wbr-playground-20261003/service-test'))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    def get(path):
        return urllib.request.urlopen(args.base + path, timeout=5).read()

    def state():
        return json.loads(get('/state'))

    def post(path, body):
        request = urllib.request.Request(args.base + path, data=json.dumps(body).encode(),
            headers={'Content-Type': 'application/json'})
        return json.load(urllib.request.urlopen(request, timeout=5))

    def command(keys, reset=False):
        return post('/command', dict(keys=keys, reset=reset, controller='own', stage=3,
            location='playground', drive_speed=.6, turn_speed=1.2, leg_length=.20,
            jump_profile=0, client_id='service-test', claim_control=True))

    rows = []

    def run(keys, seconds, label):
        start = None
        expected_v = .6 * (int('w' in keys) - int('s' in keys))
        turning = 'a' in keys or 'd' in keys
        deadline = time.monotonic() + max(30, seconds * 3)
        while time.monotonic() < deadline:
            command(keys)
            sample = state()
            if not sample['running']:
                raise AssertionError(sample['status'])
            # 命令和状态发布异步，先确认新输入已被物理循环采纳。
            if (abs(sample['command_v'] - expected_v) > .001 or
                    (turning and abs(sample['command_yaw']) < .1)):
                time.sleep(.04)
                continue
            if start is None:
                start = sample['sim_time']
            rows.append(dict(label=label, **{k: v for k, v in sample.items() if k != 'terrain_info'}))
            if sample['sim_time'] >= start + seconds:
                return
            time.sleep(.04)
        raise TimeoutError(label)

    def reset():
        command([], True)
        deadline = time.monotonic() + 20
        seen_reset = False
        while time.monotonic() < deadline:
            sample = state()
            seen_reset |= sample['sim_time'] < .6
            command([])
            if seen_reset and sample['own_ready']:
                return
            if not sample['running']:
                raise AssertionError(sample['status'])
            time.sleep(.03)
        raise TimeoutError('home rise')

    post('/record', {'action': 'start'})
    reset()
    run(['w'], 2.0, 'forward')
    run(['w', 'a'], 2.5, 'forward_left')
    reset()
    run(['s'], 2.0, 'reverse')
    run(['s', 'a'], 2.5, 'reverse_left')
    reset()
    run(['w'], 2.0, 'before_brake')
    run(['s', 'a'], 3.5, 'brake_to_reverse')
    command([])
    sizes = {}
    before = state()
    for quality, expected in [('standard', (800, 500)), ('ultra', (1920, 1200)), ('high', (1280, 800))]:
        post('/quality', {'quality': quality})
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            frame = get('/frame.jpg')
            size = Image.open(io.BytesIO(frame)).size
            if size == expected:
                sizes[quality] = size
                (args.output / f'{quality}.jpg').write_bytes(frame)
                break
            time.sleep(.05)
        assert sizes.get(quality) == expected
    after = state()
    assert after['sim_time'] >= before['sim_time'] and after['control_owner'] == before['control_owner']
    post('/record', {'action': 'stop'})
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        recording = state()['recorder']
        if recording['ready']:
            break
        if recording['error']:
            raise AssertionError(recording['error'])
        time.sleep(.05)
    assert recording['ready'] and recording['dropped'] == 0
    archive = get('/recordings/' + recording['filename'])
    (args.output / recording['filename']).write_bytes(archive)
    with zipfile.ZipFile(io.BytesIO(archive)) as package:
        jsonl = next(name for name in package.namelist() if name.endswith('.jsonl'))
        records = [json.loads(line) for line in package.read(jsonl).decode('utf-8').splitlines()]
        metadata = next(record['parameters'] for record in records if record['type'] == 'metadata')
        assert len(metadata['control_sensor_order']) == 28
        assert metadata['hip_motor_model'] == '8009'
        assert metadata['hip_motor_continuous_nm'] == 20 and metadata['hip_motor_peak_nm'] == 40
        assert metadata['knee_range']['mode'] == 'stop180'
        assert metadata['gas_spring_n_each'] == 450
        samples = [record for record in records if record['type'] == 'sample']
        assert len(samples) == recording['written'] and len(samples) > 1000
        assert len(samples[-1]['frame']['motor_telemetry']['wheels']) == 2
        assert len(samples[-1]['frame']['motor_telemetry']['hips']) == 4
        assert any(r.get('event') == 'scene_reset' for r in records)
        inputs = [sample['frame']['control_input'] for sample in samples
            if sample['frame'].get('control_input') is not None]
        assert len(inputs) > 1000
        assert all(len(item['sensor']) == 28 and len(item['command']) == 3 for item in inputs)
        assert all(value is not None for item in inputs for value in item['sensor'] + item['command'])
        assert all(sample['frame']['step_climb_mode'] == 2 for sample in samples)
        for sample in samples:
            control = sample['frame'].get('control_input')
            if control is not None:
                assert 0 <= sample['sim_time_s'] - control['time_s'] <= .0041
    forward = [r for r in rows if r['label'] == 'forward_left' and r['yaw_rate_radps'] > .6]
    reverse = [r for r in rows if r['label'] == 'reverse_left' and r['yaw_rate_radps'] < -.6]
    braking = [r for r in rows if r['label'] == 'brake_to_reverse']
    positive = [r for r in braking if r['v_mps'] > .2]
    negative = [r for r in braking if r['v_mps'] < -.2]
    passed = bool(forward and reverse and positive and negative and
        all(r['command_yaw'] > 0 for r in positive) and
        all(r['command_yaw'] < 0 for r in negative))
    summary = dict(passed=passed, forward_yaw_max=max((r['yaw_rate_radps'] for r in forward), default=None),
        reverse_yaw_min=min((r['yaw_rate_radps'] for r in reverse), default=None), quality_sizes=sizes,
        recorder=recording, samples_in_download=len(samples), control_inputs_in_download=len(inputs),
        archive_bytes=len(archive))
    (args.output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    (args.output / 'trace.json').write_text(json.dumps(rows, ensure_ascii=False), encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=True), flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
