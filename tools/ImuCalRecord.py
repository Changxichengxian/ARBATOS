"""将 A 板稳温静止快照转换为内部 Flash 校准记录；本工具不擦写硬件。"""
import argparse
import json
import math
from pathlib import Path
import struct
import zlib

MAGIC = 0x4D494341


def decode_record(data, uid):
    if len(data) < 44:
        raise ValueError('校准记录不完整')
    magic, version, seq, stored_uid, x, y, z, temp, crc = struct.unpack('<III12s4fI', data[:44])
    if magic != MAGIC or version not in (1, 2) or stored_uid != uid:
        raise ValueError('板号或校准格式不匹配')
    if crc != zlib.crc32(data[:40]):
        raise ValueError('校准 CRC 不匹配')
    if any(not math.isfinite(v) or abs(v) > .0873 for v in (x, y, z)):
        raise ValueError('零偏超出范围')
    if not math.isfinite(temp) or not 47 <= temp <= 49:
        raise ValueError('校准温度不在 A 板工作范围')
    return dict(sequence=seq, version=version, bias=[x, y, z], temperature=temp)


def from_snapshot(s, sequence):
    if not s['sd_quiesce_confirmed'] or s['g_robot_lifecycle']['output_allowed']:
        raise ValueError('必须先停机并同步关闭 SD')
    if not s['InsGyroCaliState']['boot_adjust_finished'] or not s['InsHeaterStable']:
        raise ValueError('稳温静止校准尚未完成')
    if abs(s['InsTemp'] - 48) > .35 or s['ImuPwmLastError']:
        raise ValueError('温度或加热器状态异常')
    # Zephyr STM32 hwinfo 使用倒序 UID 字和大端字节序。
    uid = struct.pack('>III', *reversed(s['uid_words']))
    body = struct.pack('<III12s4f', MAGIC, 1, sequence, uid,
                       *s['InsGyroOffset'], s['InsTemp'])
    record = body + struct.pack('<I', zlib.crc32(body)) + b'\xff' * 20
    decode_record(record, uid)
    return record


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('snapshot', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--sequence', required=True, type=int)
    args = p.parse_args()
    if not 1 <= args.sequence <= 0xffffffff:
        p.error('序号必须在 1..0xffffffff')
    s = json.loads(args.snapshot.read_text(encoding='utf-8'))
    args.output.write_bytes(from_snapshot(s, args.sequence))
    print(args.output)


if __name__ == '__main__':
    main()
