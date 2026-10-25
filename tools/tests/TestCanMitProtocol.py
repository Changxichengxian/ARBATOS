"""编译实际电机参数表和 MIT 驱动，执行已知字节回归，不访问硬件。"""
import argparse
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--zig', default=shutil.which('zig'))
    args = parser.parse_args()
    if not args.zig:
        parser.error('请用 --zig 指定 Zig 编译器')
    output = ROOT / 'local/build/host-tests/can-mit-protocol'
    output.mkdir(parents=True, exist_ok=True)
    config = output / 'generated'
    subprocess.run([sys.executable, '-X', 'utf8', str(ROOT / 'tools/config/RobotConfigGen.py'),
                    'generate', '--target', 'MINIWHEELEG-M', '--out', str(config)], check=True)
    includes = [ROOT / 'tools/tests/stubs', ROOT / 'Robotconfig/MINIWHEELEG-M', config,
                ROOT / 'shared/application/robot', ROOT / 'shared/application/motors',
                ROOT / 'shared/components/support', ROOT / 'shared/hal']
    exe = output / 'test.exe'
    subprocess.run([args.zig, 'cc', '-std=c99', '-Wall', '-Wextra', '-Werror',
                    *['-I' + str(p) for p in includes],
                    str(ROOT / 'tools/tests/CanMitProtocolRegression.c'),
                    str(ROOT / 'shared/application/motors/MotorModelDb.c'), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    main()
