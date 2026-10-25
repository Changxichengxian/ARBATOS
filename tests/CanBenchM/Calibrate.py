"""失能读取六台角度，或执行一次由主控限时的单轮短转。"""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import struct
import time

import CanBench


def angles(state):
    values = []
    for motor_id, motor in enumerate(state['motors'], 1):
        raw = struct.pack('<II', motor['raw0'], motor['raw1'])
        p = ((raw[1] << 8) | raw[2]) * 25.0 / 65535.0 - 12.5
        vmax = 30.0 if motor_id <= 4 else 280.0
        v = ((raw[3] << 4) | (raw[4] >> 4)) * 2 * vmax / 4095.0 - vmax
        values.append({'id': motor_id, 'position_rad': p, 'position_deg': math.degrees(p),
                       'velocity_rad_s': v, 'state': motor['state'], 'count': motor['count']})
    return values


def capture(link, action):
    command = {'sample': 4, 'left': 5, 'right': 6}[action]
    link.write(('\nCB1 ' + action + '\n').encode('ascii'))
    link.flush()
    try:
        state = CanBench.serial_report(link, command, timeout=8)
    except BaseException:
        link.write(b'!\n')
        link.flush()
        raise
    return state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('sample', 'left', 'right', 'watch'))
    parser.add_argument('--port', default='COM14')
    parser.add_argument('--label', required=True)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 0 < args.seconds <= 180:
        parser.error('观察时长必须在 0～180 秒内')
    args.output.mkdir(parents=True, exist_ok=True)
    records = []
    path = args.output / (datetime.now().strftime('%H%M%S') + '-' + args.label + '.json')
    with CanBench.serial_open(args.port) as link:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if b'ARBATOS_CAN_BENCH_V3_CAN1_1M_ID1_6' in link.readline():
                break
        else:
            raise RuntimeError('没有读到 V3 标定固件身份')
        link.write(b'\nCB1 status\n')
        before = CanBench.serial_report(link, 0, timeout=8)
        if before['ready'] != 1 or before['busy'] != 0:
            raise RuntimeError('主控未就绪或仍在运行其他操作')
        start = time.monotonic()
        while True:
            state = capture(link, 'sample' if args.action == 'watch' else args.action)
            row = {'time': datetime.now().isoformat(), 'label': args.label, 'action': args.action,
                   'angles': angles(state), 'raw': state}
            records.append(row)
            path.write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding='utf-8')
            if state['result'] != 1 or state['busy'] != 0:
                raise RuntimeError('主控未报告本次操作成功，原始记录：' + str(path))
            if any(motor['state'] != 0 or motor['count'] == 0 for motor in state['motors']):
                raise RuntimeError('缺少六台失能确认，原始记录：' + str(path))
            if any(new['count'] <= old['count'] for new, old in zip(state['motors'], before['motors'])):
                raise RuntimeError('反馈未全部刷新，不能当作当前姿态，记录：' + str(path))
            print(row['time'], ' '.join(f"ID{x['id']}={x['position_deg']:.2f}deg" for x in row['angles']),
                  'all_disabled=1', flush=True)
            before = state
            if args.action != 'watch' or time.monotonic() - start >= args.seconds:
                break
            time.sleep(0.2)
    print('记录：' + str(path), flush=True)


if __name__ == '__main__':
    main()
