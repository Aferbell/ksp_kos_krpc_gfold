# diag9_top.py — 定位可行域【上边界】(alt~1758) 的紧约束
#
# 已知：alt=1884 infeasible，alt=1758 optimal（真实状态、真实质量、tf=30）。
# 已排除：滑翔锥(30°允许 dist/alt<=1.73，实际 0.72)、推力上界(需求 |a|~16 << a_net~58)。
# 逐个候选：① 滑翔锥角度 ② pcs_start ③ 速度上限 v_max ④ 末端推力竖直等式
#           ⑤ 节点数 N ⑥ 时间 tf ⑦ 起末位置同时约束(几何可达性)
import csv
import os
import numpy as np

# --- 路径自愈：analysis/ 下的脚本引用 solver/ 里的求解器 ---
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'solver'))
# --------------------------------------------------------
from gfold_terminal import solve_terminal

LOGDIR = r'D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script\log-B1040-9'


def load(name):
    with open(os.path.join(LOGDIR, name), 'r', encoding='utf-8-sig') as f:
        return list(csv.DictReader(f))


def g(r, k, d=0.0):
    try:
        return float(r[k])
    except (KeyError, ValueError):
        return d


rows = load('land_log_47822691.csv')
ign = next(i for i, r in enumerate(rows) if g(r, 'phase') >= 1)
# 取 alt 最接近 1884 的真实行
best = min((r for r in rows[ign:] if 1500 < g(r, 'alt') < 2200),
           key=lambda r: abs(g(r, 'alt') - 1884))
S = dict(alt=g(best, 'alt'), dist=g(best, 'dist_hz'), vz=g(best, 'vz'),
         vh=g(best, 'vh'), m=g(best, 'mass'))
print('测试状态（真实行）: alt=%.0f dist=%.0f vz=%.1f vh=%.1f mass=%.1ft'
      % (S['alt'], S['dist'], S['vz'], S['vh'], S['m']))

base = dict(alt=S['alt'], dist=S['dist'], vz=S['vz'], vh=S['vh'],
            mass=S['m'] * 1000, tf=30, N=50,
            gs_deg=30.0, pcs_start_deg=85.0, pcs_end_deg=5.0)

print('\n基线: %s' % solve_terminal(**base)['status'])

print('\n### 单变量放开')
tests = [
    ('gs 30 -> 20 (更宽锥)',  dict(gs_deg=20.0)),
    ('gs 30 -> 10',           dict(gs_deg=10.0)),
    ('pcs_start 85 -> 89.9',  dict(pcs_start_deg=89.9)),
    ('pcs_end 5 -> 30',       dict(pcs_end_deg=30.0)),
    ('v_max 400 -> 1000',     dict(v_max=1000.0)),
    ('tf 30 -> 60',           dict(tf=60)),
    ('tf 30 -> 120',          dict(tf=120)),
    ('N 50 -> 30',            dict(N=30)),
    ('N 50 -> 80',            dict(N=80)),
    ('throttle_max 1.0',      dict(throttle=(0.0, 1.0))),
    ('t_max 12.8 -> 19.2MN',  dict(t_max=19.2e6)),
]
for name, over in tests:
    kw = dict(base)
    kw.update(over)
    r = solve_terminal(**kw)
    print('  %-24s -> %s' % (name, r['status']))

print('\n### 组合放开（找最小充分集）')
combos = [
    ('gs20 + pcs89.9',        dict(gs_deg=20.0, pcs_start_deg=89.9)),
    ('gs20 + tf60',           dict(gs_deg=20.0, tf=60)),
    ('gs10',                  dict(gs_deg=10.0)),
    ('gs10 + pcs89.9',        dict(gs_deg=10.0, pcs_start_deg=89.9)),
    ('vmax1000 + gs20',       dict(v_max=1000.0, gs_deg=20.0)),
]
for name, over in combos:
    kw = dict(base)
    kw.update(over)
    r = solve_terminal(**kw)
    print('  %-24s -> %s' % (name, r['status']))
