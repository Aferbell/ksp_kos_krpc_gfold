# timing_check.py — 检查求解耗时是否满足"逐帧重解(MPC)"要求
#
# demo3_gfold.py 的做法是【每个物理帧重解一次】。KSP 物理帧 = 0.02 s。
# 我们的 tf 二分（14 次求解）单次可达 ~0.3 s，远超一帧预算。
# 必须给出可行的调用策略。这里实测不同 N / 二分次数的耗时。
import time
import numpy as np
# --- 路径自愈：analysis/ 下的脚本引用 solver/ 里的求解器 ---
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'solver'))
# --------------------------------------------------------
from gfold_terminal import _solve_fixed_tf, solve_terminal

ALT, DIST, VZ, VH, M = 2000.0, 500.0, -120.0, 80.0, 160e3

print('=== 单次定 tf 求解耗时 vs N ===')
for N in [20, 30, 40, 50, 60]:
    t0 = time.time()
    r = _solve_fixed_tf(ALT, DIST, VZ, VH, M, tf=20, N=N)
    dt = time.time() - t0
    print('  N=%2d  %.3f s  (%s)' % (N, dt, r['status']))

print()
print('=== tf 自动搜索（14 次二分）总耗时 ===')
for N in [20, 30, 40]:
    t0 = time.time()
    r = solve_terminal(ALT, DIST, VZ, VH, M, N=N)
    dt = time.time() - t0
    print('  N=%2d  总 %.2f s  tf_min=%.2f  (%s)'
          % (N, dt, r.get('tf_min', float('nan')), r['status']))

print()
print('=== 结论导向：可行的调用策略 ===')
print('  KSP 物理帧 = 0.02 s。逐帧重解要求单次 < 0.02 s，当前达不到。')
print('  推荐三段式时间管理：')
print('   ① 交班时解一次（tf 自动搜索，~1 s）-> 得到参考轨迹 x*(t), u*(t)')
print('   ② 之后【每 0.5~1 s】用【固定 tf = 剩余时间】重解一次（单次 ~0.05 s @ N=20）')
print('      —— 不重跑二分，只做热启动的单次 QP/SOCP')
print('   ③ 帧间（0.02 s）沿参考轨迹做 PD 跟踪（demo3 的 find_nearest_index 法）')
