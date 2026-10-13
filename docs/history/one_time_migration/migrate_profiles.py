"""Update strict production presets without touching the approved baseline."""
from pathlib import Path
import re
root=Path(__file__).resolve().parents[1]
assert root.name=='RRTstarDFPFF_paper_r2_float32_strict_20261007'
legacy=root/'benchmark/config/legacy_mixed/paper_r2'
legacy.mkdir(parents=True,exist_ok=True)
for p in (root/'benchmark/config').glob('*.yaml'):
    if p.stem not in ('uni_bugtrap','uni_wall','quad2d_bugtrap','quad2d_hole','quad3d_block','quad3d_window'):continue
    text=p.read_text(encoding='utf-8')
    old=text.replace('source: ../../../','source: ../../../../../').replace('model_file: ../models/','model_file: ../../../models/')
    old=re.sub(r'output_directory:.*',f'output_directory: ../../../../results/legacy_{p.stem}',old)
    (legacy/p.name).write_text('numeric: {profile: legacy_mixed}\n'+old,encoding='utf-8')
    text=text.replace('profile: paper_fair','profile: flask_strict\nnumeric: {profile: float32_production}')
    text=re.sub(r'  max_iterations:.*','  max_iterations: 0',text)
    text=re.sub(r'  max_nodes:.*','  max_nodes: 0\n  hard_safety_node_cap: 1000000',text)
    text=re.sub(r'  timeout:.*','  timeout: 120.0',text)
    text=re.sub(r'  stop_at_first_solution:.*','  stop_at_first_solution: true',text)
    text=text.replace('turn_limited_sampling: true','turn_limited_sampling: false')
    text=re.sub(r'  duration_factors:.*','  duration_factors: []',text)
    text=re.sub(r'  terminal_speed:.*','  terminal_speed: 0',text)
    text=text.replace('model_time_hints: true','model_time_hints: false')
    text=text.replace('terminal_velocity_hint: true','terminal_velocity_hint: false')
    text=re.sub(r'  runs:.*','  runs: 20',text)
    text=re.sub(r'  seed:.*','  seed: 41',text)
    text=re.sub(r'  output_directory:.*',f'  output_directory: ../../results/float32_{p.stem}',text)
    text+='\npostprocess:\n  shortcut: true\n  mode: flask\n  time_mode: original_duration\n'
    p.write_text(text,encoding='utf-8')
for p in (root/'benchmark/config/full_df').glob('*.yaml'):
    t=p.read_text(encoding='utf-8');p.write_text('numeric: {profile: legacy_mixed}\n'+t,encoding='utf-8')

