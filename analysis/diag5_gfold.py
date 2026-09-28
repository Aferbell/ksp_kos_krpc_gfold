# diag5_gfold.py — 为什么真实剖面状态全部 infeasible（而人造状态可以）
#
# 对比：
#   人造 alt=2500 dist=700 vz=-120 vh=80  m=152t  -> OK (tf=25, N=60)
#   真实 alt=2549 dist=1880 vz=... vh=... m=185t  -> infeasible
# 差异候选：① tf 20 vs 25；② dist 大得多；③ mass 大得多；④ 滑翔锥 gs=60
# 逐项隔离。
import numpy as np
# --- 路径自愈：analysis/ 下的脚本引用 solver/ 里的求解器 ---
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'solver'))
# --------------------------------------------------------
from gfold_terminal import solve_terminal

print('### 1. 复现：人造状态在 gs=60 / tf=20 下是否还 OK')
for tf in [20, 25]:
    for gs in [25, 60]:
        r = solve_terminal(alt=2500, dist=700, vz=-120, vh=80, mass=152e3,
                           tf=tf, gs_deg=gs)
        print('  人造 tf=%2d gs=%2d -> %s' % (tf, gs, r['status']))

print()
print('### 2. 真实状态，单变量逼近：从人造状态一步步改成真实状态')
base = dict(alt=2500, dist=700, vz=-120, vh=80, mass=152e3, tf=25, N=60,
            gs_deg=60.0, pcs_start_deg=80.0, pcs_end_deg=5.0)
r = solve_terminal(**base)
print('  base(alt2500 d700 vz-120 vh80 m152t gs60 tf25) -> %s' % r['status'])

for label, over in [
    ('dist 700 -> 1880', dict(dist=1880)),
    ('mass 152 -> 185 t', dict(mass=185e3)),
    ('vh 80 -> 200', dict(vh=200)),
    ('vz -120 -> -269', dict(vz=-269)),
    ('tf 25 -> 20', dict(tf=20)),
    ('N 60 -> 40', dict(N=40)),
    ('pcs_start 80 -> 90', dict(pcs_start_deg=89.9)),
]:
    kw = dict(base); kw.update(over)
    r = solve_terminal(**kw)
    print('  %-20s -> %s' % (label, r['status']))

print()
print('### 3. 真实状态本身，扫描 tf 与 pcs_start')
real = dict(alt=2549, dist=1880, vz=-269.0, vh=207.0, mass=186e3,
            N=60, gs_deg=60.0, pcs_end_deg=5.0)
for tf in [15, 20, 25, 30, 40, 50]:
    row = []
    for ps in [80, 85, 89]:
        kw = dict(real); kw.update(tf=tf, pcs_start_deg=ps)
        r = solve_terminal(**kw)
        row.append('ps%2d:%s' % (ps, 'OK ' if r['status'] == 'optimal' else '-- '))
    print('  tf=%2d  %s' % (tf, '  '.join(row)))

print()
print('### 4. 若把 dist 缩小（相当于交接点选在更靠近目标处）')
for d in [1880, 1500, 1000, 700, 400, 200, 100]:
    r = solve_terminal(alt=2549, dist=d, vz=-269.0, vh=207.0, mass=186e3,
                       tf=25, N=60, gs_deg=60.0, pcs_start_deg=85.0)
    print('  dist=%5d -> %s' % (d, r['status']))
