# diag8_realstates.py — 用【真实】状态值（含真实质量）测 G-FOLD 可行域
#
# 更正前面测试里的一个错误：diag7 我用了臆造的 vz/vh（|v|=339），
# 而真实 alt=2549 处是 vz=-360 / vh=288.5 / |v|=461.3 / mass=194.2 t。
# 这里全部改用日志真值，重新确定可行域上边界。
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

states = []
for r in rows[ign:]:
    alt = g(r, 'alt')
    if 60 < alt < 6000:
        states.append((g(r, 't'), alt, g(r, 'dist_hz'), g(r, 'vz'),
                       g(r, 'vh'), g(r, 'mass')))

print('### 真实状态 + 真实质量，G-FOLD 可行域（tf 扫描）')
print('   alt    dist     vz      vh    |v|   mass   a_net  tf20 tf25 tf30 tf40 tf60')
feas_lo = None
for (t, alt, d, vz, vh, m) in states[::8]:
    G0, TMAX = 9.80665, 12.8e6
    a_net = TMAX / (m * 1000) - G0
    row = []
    for tf in [20, 25, 30, 40, 60]:
        r = solve_terminal(alt=alt, dist=d, vz=vz, vh=vh, mass=m * 1000,
                           tf=tf, N=50, gs_deg=30.0,
                           pcs_start_deg=85.0, pcs_end_deg=5.0)
        row.append('  OK ' if r['status'] == 'optimal' else '  -- ')
    print('  %5.0f %7.0f %7.1f %7.1f %6.0f %6.1f %6.1f %s'
          % (alt, d, vz, vh, np.hypot(vz, vh), m, a_net, ''.join(row)))

print()
print('### 用真实状态确定"最低可行交班高度"（tf=30, N=50）')
print('   alt    dist    |v|   -> status')
prev_ok = True
for (t, alt, d, vz, vh, m) in states[::4]:
    r = solve_terminal(alt=alt, dist=d, vz=vz, vh=vh, mass=m * 1000,
                       tf=30, N=50, gs_deg=30.0,
                       pcs_start_deg=85.0, pcs_end_deg=5.0)
    ok = r['status'] == 'optimal'
    mark = ''
    if ok != prev_ok:
        mark = '   <<< 边界切换'
        prev_ok = ok
    extra = ''
    if ok:
        extra = ' 落地%.0ft 末端倾角%.2f°' % (r['mass_land'] / 1000, r['end_tilt_deg'])
    print('  %5.0f %7.0f %6.0f  -> %-10s%s%s'
          % (alt, d, np.hypot(vz, vh), r['status'], extra, mark))
