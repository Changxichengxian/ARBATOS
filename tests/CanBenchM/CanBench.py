"""MC02 CAN 台架工具：build 不接硬件；flash 和 run 必须显式调用。"""
import argparse
import binascii
import csv
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import statistics
import struct
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BUILD = ROOT / 'local/build/can-bench-m-20261024'
HEADER = 'magic ready initError busy result trialCount windowCount tickMs txAttempts txAccepted txComplete txErrors rxFrames rxBitsMin rxBitsMax'.split()
TRIAL = 'id round command attempts accepted completed txErrors elapsedUs state outcome'.split()
WINDOW = 'rate durationUs txAttempts txAccepted txComplete txErrors rxFrames rxBitsMin rxBitsMax overruns rxErrors txBusErrors busState'.split()
IDENTITY = b'ARBATOS_CAN_BENCH_V3_CAN1_1M_ID1_6\0'
POSITIONS = {'1': '右后关节', '2': '右前关节',
             '3': '左后关节', '4': '左前关节',
             '5': '右轮', '6': '左轮'}


def build(args):
    env = os.environ.copy()
    scripts = ROOT / 'local/cache/zephyrproject/.venv/Scripts'
    env.update(ZEPHYR_BASE=str(ROOT / 'local/cache/zephyrproject/zephyr'),
               ZEPHYR_SDK_INSTALL_DIR=str(ROOT / 'local/cache/zephyr-sdk'),
               ZEPHYR_TOOLCHAIN_VARIANT='zephyr', CMAKE_BUILD_PARALLEL_LEVEL='2', PYTHONUTF8='1')
    env['PATH'] = str(scripts) + os.pathsep + env.get('PATH', '')
    command = [str(scripts / 'python.exe'), '-m', 'west', 'build', '-s', str(Path(__file__).parent),
               '-d', str(args.build), '-b', 'dm_mc02_h7', '-p', 'never', '--',
               '-DDTC_OVERLAY_FILE=' + (Path(__file__).parent / 'app.overlay').as_posix(),
               '-DZephyr-sdk_DIR=' + (ROOT / 'local/cache/zephyr-sdk/cmake').as_posix(),
               '-DCMAKE_MAKE_PROGRAM=' + (scripts / 'ninja.exe').as_posix()]
    subprocess.run(command, cwd=ROOT, env=env, check=True)


def symbols(build_dir):
    from elftools.elf.elffile import ELFFile
    with (build_dir / 'zephyr/zephyr.elf').open('rb') as stream:
        elf = ELFFile(stream)
        result = {s.name: (int(s['st_value']), int(s['st_size']))
                  for s in elf.get_section_by_name('.symtab').iter_symbols()}
    if result.get('CanBenchDiag', (0, 0))[1] != 5292:
        raise RuntimeError('CAN bench ELF layout mismatch')
    image = (build_dir / 'zephyr/zephyr.bin').read_bytes()
    if not 0 < len(image) <= 0xc0000:
        raise RuntimeError('Image is empty or overlaps calibration sectors')
    return result, image


def session_for(probe):
    from pyocd.core.helpers import ConnectHelper
    return ConnectHelper.session_with_chosen_probe(
        unique_id=probe, target_override='stm32h723xx', frequency=100000,
        options={'connect_mode': 'attach', 'resume_on_disconnect': False,
                 'cmsis_dap.limit_packets': True, 'cmsis_dap.deferred_transfers': False,
                 'cache.enable_memory': False, 'cache.enable_register': False})


def decode(data):
    if len(data) != 5292:
        raise ValueError('Incomplete RAM snapshot')
    header = dict(zip(HEADER, struct.unpack_from('<IIi12I', data)))
    if header['magic'] != 0x43424d31 or header['trialCount'] > 120 or header['windowCount'] > 6:
        raise ValueError('Invalid RAM snapshot')
    header['motors'] = [dict(zip('count cycle state raw0 raw1'.split(), struct.unpack_from('<5I', data, 60 + i * 20))) for i in range(6)]
    header['trials'] = [dict(zip(TRIAL, struct.unpack_from('<10I', data, 180 + i * 40))) for i in range(header['trialCount'])]
    header['windows'] = [dict(zip(WINDOW, struct.unpack_from('<13I', data, 4980 + i * 52))) for i in range(header['windowCount'])]
    return header


def summarize(records):
    result = {'id_to_position': POSITIONS.copy(),
              'mapping_source': '2026-10-24 本轮用户分组手动摆杆及左右轮分别短转确认',
              'transitions': [], 'load_windows': []}
    for record in records:
        for window in record['windows'] if record.get('test') in ('passive', 'load') else []:
            duration = window['durationUs']
            if not duration:
                continue
            # 1 Mbps 下，微秒数正好等于同期总线位容量。含帧间隔，不含错误帧及硬件重发。
            result['load_windows'].append({**window,
                'measured_tx_fps': window['txComplete'] * 1e6 / duration,
                'measured_rx_fps': window['rxFrames'] * 1e6 / duration,
                'observed_load_min_percent': (window['rxBitsMin'] + 111 * window['txComplete']) * 100 / duration,
                'observed_load_max_percent': (window['rxBitsMax'] + 135 * window['txComplete']) * 100 / duration})
        if record.get('test') != 'transitions':
            continue
        for motor in range(1, 7):
            for command, label in ((0xfc, '使能'), (0xfd, '失能')):
                rows = [r for r in record['trials'] if r['id'] == motor and r['command'] == command]
                if not rows:
                    continue
                ok = [r for r in rows if r['outcome'] == 1]
                values = sorted(r['elapsedUs'] for r in ok)
                result['transitions'].append({'id': motor, 'action': label, 'tests': len(rows), 'success': len(ok),
                    'attempts_max': max(r['attempts'] for r in rows),
                    'latency_median_us': statistics.median(values) if values else None,
                    'latency_p95_us': values[max(0, (95 * len(values) + 99) // 100 - 1)] if values else None,
                    'latency_max_us': max(values) if values else None})
    return result


def save(output, records):
    output.mkdir(parents=True, exist_ok=True)
    (output / '原始结果.json').write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding='utf-8')
    summary = summarize(records)
    (output / '统计结果.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    with (output / '使能失能明细.csv').open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=TRIAL)
        writer.writeheader()
        for record in records:
            if record.get('test') == 'transitions':
                writer.writerows(record['trials'])
    return summary


def serial_open(port):
    import serial
    link = serial.Serial(port=None, baudrate=115200, timeout=0.2, write_timeout=2)
    link.dtr = False
    link.rts = False
    link.port = port
    link.open()
    return link


def serial_report(link, command, timeout=45):
    deadline = time.monotonic() + timeout
    payload = None
    expected_crc = None
    pending = bytearray()
    while time.monotonic() < deadline:
        part = link.readline()
        pending.extend(part)
        if len(pending) > 16384:
            raise ValueError('UART line exceeded expected size')
        if not part.endswith(b'\n'):
            continue
        line = bytes(pending).strip()
        pending.clear()
        # 无线串口刚打开时可能接到半截状态行；报告仍必须完整通过长度和 CRC 校验。
        marker = line.find(b'CB1_BEGIN ')
        if marker >= 0:
            line = line[marker:]
        if line.startswith(b'CB1_BEGIN '):
            fields = line.split()
            if len(fields) != 4:
                raise ValueError('Invalid UART report header')
            if int(fields[1]) != command:
                payload = None
                continue
            if int(fields[2]) != 5292:
                raise ValueError('Unexpected UART report size')
            expected_crc = int(fields[3], 16)
            payload = bytearray()
        elif payload is not None and line == b'CB1_END':
            if len(payload) != 5292 or binascii.crc32(payload) != expected_crc:
                raise ValueError('UART report incomplete or CRC mismatch')
            return decode(payload)
        elif payload is not None and line:
            payload.extend(bytes.fromhex(line.decode('ascii')))
            if len(payload) > 5292:
                raise ValueError('UART report exceeded expected size')
    raise TimeoutError('UART report timed out')


def serial_run(args, output):
    records = []
    active = False
    tests = [(1, 'passive'), (2, 'transitions'), (3, 'load')]
    if args.test != 'all':
        tests = [item for item in tests if item[1] == args.test]
    with serial_open(args.port) as link:
        time.sleep(1)
        link.reset_input_buffer()
        link.write(b'\nCB1 status\n')
        link.flush()
        state = serial_report(link, 0, timeout=8)
        if state['ready'] != 1 or state['busy'] != 0:
            raise RuntimeError(f"Firmware not ready: init={state['initError']} busy={state['busy']}")
        try:
            for command, name in tests:
                print('开始：' + name, flush=True)
                link.write(('CB1 ' + name + '\n').encode('ascii'))
                active = True
                state = serial_report(link, command)
                active = False
                state.update(test=name, transport='UART8_115200', port=args.port)
                records.append(state)
                save(output, records)
                print(f"{name}: result={state['result']}, trials={state['trialCount']}, windows={state['windowCount']}", flush=True)
                if state['result'] != 1:
                    raise RuntimeError('Test stopped or disable confirmation incomplete; inspect raw result')
        finally:
            if active:
                link.write(b'!\n')
                link.flush()
                print('已通过串口请求停止；结果未确认前不计为成功。', flush=True)
            if records:
                save(output, records)
    print(json.dumps(summarize(records), ensure_ascii=False, indent=2), flush=True)
    print('结果目录：' + str(output), flush=True)


def serial_listen(args):
    with serial_open(args.port) as link:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            line = link.readline()
            if line:
                print(line.decode('utf-8', errors='replace'), end='', flush=True)


def flash(args, session, syms, image, output):
    from pyocd.flash.file_programmer import FileProgrammer
    from pyocd.core.target import Target
    target = session.target
    identity = bytes(target.read_memory_block8(syms['CanBenchIdentity'][0], len(IDENTITY)))
    if identity != IDENTITY and not args.sd_removed:
        raise RuntimeError('旧程序身份未知。沿用仓库烧录约束：确认 SD 已取下后加 --sd-removed，避免暂停时破坏旧日志。')
    # 默认保留完整备份；只有用户明确要求时才跳过，应用区始终使用扇区擦写。
    target.halt()
    backup = None
    if args.skip_backup:
        print('按用户要求跳过备份，直接按扇区烧录应用程序。', flush=True)
    else:
        print('已暂停旧程序，正在备份完整 Flash。', flush=True)
        backup = bytes(target.read_memory_block8(0x08000000, 0x100000))
        (output / '烧录前完整备份.bin').write_bytes(backup)
        print('完整备份已保存，开始烧录 CAN 测试固件。', flush=True)
    # attach 会保留旧程序中断/外设状态；先软件复位并暂停，避免擦除向量表后旧中断进入空地址。
    target.reset_and_halt(reset_type=Target.ResetType.SYSRESETREQ)
    FileProgrammer(session, chip_erase='sector', trust_crc=False, keep_unwritten=not args.skip_backup).program(
        str(args.build / 'zephyr/zephyr.bin'), base_address=0x08000000)
    if bytes(target.read_memory_block8(0x08000000, len(image))) != image:
        raise RuntimeError('Image readback mismatch; target remains halted')
    if backup is not None and bytes(target.read_memory_block8(0x080c0000, 0x40000)) != backup[0xc0000:]:
        raise RuntimeError('Calibration region changed; target remains halted')
    (output / '烧录核验.json').write_text(json.dumps({'sha256': hashlib.sha256(image).hexdigest(),
        'bytes': len(image), 'readback_match': True, 'calibration_unchanged': True if backup is not None else None,
        'backup_skipped_by_request': args.skip_backup,
        'backup_sha256': hashlib.sha256(backup).hexdigest() if backup is not None else None,
        'backup_bytes': len(backup) if backup is not None else 0}, indent=2), encoding='utf-8')
    target.reset(reset_type=Target.ResetType.SYSRESETREQ)
    target.resume()
    print('烧录并回读通过；上电只接收，尚未开始使能测试。', flush=True)


def run(args, session, syms, image, output):
    target = session.target
    address = syms['CanBenchIdentity'][0]
    if bytes(target.read_memory_block8(address, len(IDENTITY))) != IDENTITY:
        raise RuntimeError('Board is not running this CAN bench firmware')
    if bytes(target.read_memory_block8(0x08000000, len(image))) != image:
        raise RuntimeError('Board image does not match this build')
    base = syms['CanBenchDiag'][0]
    command_addr = syms['CanBenchCommand'][0]
    stop_addr = syms['CanBenchStop'][0]
    records = []
    active = False
    tests = [(1, 'passive'), (2, 'transitions'), (3, 'load')]
    if args.test != 'all':
        tests = [item for item in tests if item[1] == args.test]
    try:
        for command, name in tests:
            state = decode(bytes(target.read_memory_block8(base, 5292)))
            if state['ready'] != 1 or state['busy'] != 0 or target.read32(command_addr) != 0:
                raise RuntimeError('Firmware not ready or another test is active')
            print('开始：' + name, flush=True)
            target.write32(stop_addr, 0)
            target.write32(command_addr, command)
            active = True
            deadline = time.monotonic() + 40
            while time.monotonic() < deadline:
                time.sleep(0.1)
                if target.read32(base + 12) == 0 and target.read32(command_addr) == 0:
                    break
            else:
                raise TimeoutError('Firmware test exceeded 40 seconds')
            active = False
            state = decode(bytes(target.read_memory_block8(base, 5292)))
            state['test'] = name
            records.append(state)
            save(output, records)
            print(f"{name}: result={state['result']}, trials={state['trialCount']}, windows={state['windowCount']}", flush=True)
            if state['result'] != 1:
                raise RuntimeError('Test stopped or disable confirmation incomplete; inspect raw result')
    finally:
        if active:
            try:
                target.write32(stop_addr, 1)
                print('已请求终止；固件会补发失能，需读取结果确认。', flush=True)
            except Exception:
                print('连接中断；固件自身有超时和失能收尾，尚未确认结果。', flush=True)
        if records:
            save(output, records)
    print(json.dumps(summarize(records), ensure_ascii=False, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('build', 'flash', 'run', 'serial-run', 'serial-listen'))
    parser.add_argument('--build', type=Path, default=DEFAULT_BUILD)
    parser.add_argument('--probe', help='显式选择 DAPLink 序列号；build 不需要')
    parser.add_argument('--test', choices=('all', 'passive', 'transitions', 'load'), default='all')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--sd-removed', action='store_true', help='仅 flash 使用：确认 SD 已物理取下')
    parser.add_argument('--skip-backup', action='store_true', help='仅 flash 使用：用户明确要求跳过备份时使用，仍校验应用程序回读')
    parser.add_argument('--port', help='UART8 对应串口，例如 COM14；固定 115200 8N1')
    parser.add_argument('--seconds', type=float, default=8, help='serial-listen 只读观察时长')
    args = parser.parse_args()
    if args.action == 'build':
        build(args)
        return
    if args.action in ('serial-run', 'serial-listen'):
        if not args.port:
            parser.error('serial-run/serial-listen 必须给出 --port')
        if args.action == 'serial-listen':
            serial_listen(args)
        else:
            output = args.output or ROOT / 'local/cache/can-bench' / datetime.now().strftime('%Y%m%d-%H%M%S')
            output.mkdir(parents=True, exist_ok=False)
            serial_run(args, output)
        return
    if not args.probe:
        parser.error('flash/run 必须给出 --probe')
    syms, image = symbols(args.build)
    output = args.output or ROOT / 'local/cache/can-bench' / datetime.now().strftime('%Y%m%d-%H%M%S')
    output.mkdir(parents=True, exist_ok=False)
    with session_for(args.probe) as session:
        if session.target.read32(0x5c001000) & 0xfff != 0x483:
            raise RuntimeError('Unexpected MCU device ID')
        if args.action == 'flash':
            flash(args, session, syms, image, output)
        else:
            run(args, session, syms, image, output)


if __name__ == '__main__':
    main()
