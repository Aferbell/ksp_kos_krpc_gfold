# diag4_gfold.py — 剩余两个 case 仍 infeasible 的原因
#
# 现状（gfold_terminal.py 自测）：
#   锥内-高能  OK  末端倾角 1.75°
#   锥内-高横速 OK  末端倾角 0.00°
#   锥内典型   infeasible
#   锥内-低空   infeasible
# 有意思的是"高能/高横速"能解、"典型/低空"不能 —— 说明不是能量问题。
# 低空 case（alt=800, dist=150, tf=25）怀疑是【滑翔角锥】在低空过紧：
#   dist=150 要求 alt >= dist/cot(25°) = 150*tan(25°) = 70 m，看似够。
# 典型 case（alt=2000, dist=500）：500/cot25 = 233 m，也够。
# 那问题在哪？逐项扫参数定位。
import numpy as np
# --- 路径自愈：analysis/ 下的脚本引用 solver/ 里的求解器 ---
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'solver'))
# --------------------------------------------------------
from gfold_terminal import solve_terminal

CASES = [('锥内典型', 2000, 500, -80, 50, 150e3),
         ('锥内-低空', 800, 150, -50, 30, 148e3)]


def probe(nm, alt, d, vz, vh, m):
    print('=' * 74)
    print('%s: alt=%d dist=%d vz=%d vh=%d m=%.0ft' % (nm, alt, d, vz, vh, m / 1000))

    print(' -- 扫 tf（时间是否不够）--')
    for tf in [15, 20, 25, 30, 40, 60]:
        r = solve_terminal(alt=alt, dist=d, vz=vz, vh=vh, mass=m, tf=tf)
        print('    tf=%2d -> %s' % (tf, r['status']))

    print(' -- 扫滑翔角锥 gs_deg（位置约束是否过紧）--')
    for gs in [25, 40, 60, 75, 89]:
        r = solve_terminal(alt=alt, dist=d, vz=vz, vh=vh, mass=m, gs_deg=gs)
        print('    gs=%2d° -> %s' % (gs, r['status']))

    print(' -- 扫起始推力锥 pcs_start（分段时间是否不够）--')
    for ps in [75, 85, 89]:
        r = solve_terminal(alt=alt, dist=d, vz=vz, vh=vh, mass=m, pcs_start_deg=ps)
        print('    pcs_start=%2d° -> %s' % (ps, r['status']))

    print(' -- 扫节点数 N（离散精度）--')
    for N in [40, 60, 100]:
        r = solve_terminal(alt=alt, dist=d, vz=vz, vh=vh, mass=m, N=N)
        print('    N=%3d -> %s' % (N, r['status']))


for c in CASES:
    probe(*c)
