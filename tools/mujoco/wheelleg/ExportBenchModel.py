"""从当前小轮腿物理模型生成实物板凳轮端增益，保存来源和计算结果。"""
import hashlib
import json
from pathlib import Path
import RunMeasuredWheelLeg as sim

root = sim.ROOT
profile = sim.load_profile()
core = sim.Core(profile, root / 'local/cache/measured-wheelleg-native')
model = sim.mujoco.MjModel.from_xml_string(sim.model_xml(profile))
schedule = [sim.design_bench(sim.design(model, sim.make_config(profile), core, profile, .055+i*.005), profile)
            for i in range(15)]
target = root / 'shared/application/wheelleg/WheelLegBenchGains.inc'
rows = [f'/* 由 ExportBenchModel.py 生成；有限关节刚度 {profile["control"]["bench_joint_kp_nm_rad"]:g} N.m/rad，3 ms 周期。 */',
        'static const float BenchGains[][7] = {']
for row in schedule:
    values = [row['length_m'], *row['negative_k'][0]]
    rows.append('    {' + ', '.join(f'{v:.9e}f' for v in values) + '},')
rows += ['};', '']
target.write_text('\n'.join(rows), encoding='utf-8', newline='\n')
output = root / 'local/cache/wheelleg-terrain-20261030/板凳增益来源.json'
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(json.dumps({'profile_sha256': hashlib.sha256(sim.PROFILE.read_bytes()).hexdigest(),
    'model': profile, 'schedule': schedule}, ensure_ascii=False, indent=2), encoding='utf-8')
print('generated', target, 'max spectral radius', max(row['spectral_radius'] for row in schedule))
