"""由小车物理模型生成固件和仿真共用的 VMC 增益；不启用实车 VMC。"""
import argparse
from pathlib import Path
import RunMeasuredWheelLeg as sim


def export(output):
    profile = sim.load_profile()
    model = sim.mujoco.MjModel.from_xml_string(sim.model_xml(profile))
    cfg = sim.make_config(profile)
    core = sim.Core(profile, sim.ROOT / 'local/cache/measured-wheelleg-native')
    geometry = sim.geometry_checks(model, cfg, core, profile)
    lengths = sim.np.linspace(.06, .12, 25)
    samples = [sim.design(model, cfg, core, profile, float(h)) for h in lengths]
    gains = sim.np.array([item['negative_k'] for item in samples]).reshape(len(lengths), 12)
    coefficients = sim.np.polyfit(lengths, gains, 3).T
    predicted = sim.np.array([sim.np.polyval(coefficients.T, h) for h in lengths])
    error = sim.np.max(abs(predicted - gains), axis=0)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    content = '/* 由 tools/mujoco/wheelleg/ExportVmcModel.py 生成；模型仍含未辨识的质量和惯量假设。 */\n'
    content += '{\n' + ',\n'.join('    {' + ', '.join(f'{v:.9e}f' for v in row) + '}' for row in coefficients) + '\n}\n'
    output.write_text(content, encoding='utf-8', newline='\n')
    report = {'geometry': geometry, 'source': str(sim.PROFILE),
              'length_range_m': [.06, .12], 'fit_max_absolute_error': error.tolist(),
              'note': '局部模型设计与三次拟合不等于整车闭环验证，也不等于实车标定。'}
    sim.write_json(output.with_suffix('.json'), report)
    print(output)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    export(parser.parse_args().output)
