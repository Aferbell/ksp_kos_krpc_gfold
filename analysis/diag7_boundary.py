# diag7_boundary.py — 确定 G-FOLD 的可行域边界（这决定"从哪里交班"）
#
# 现象（gfold_handoff.py）：
#   alt >= 1922 全部 infeasible；alt <= 1727 全部 OK；alt <= 203 又 infeasible。
# 两个边界都要解释：
#   上边界（alt 高 / 能量大）：为什么高就不行？—— G-FOLD 要在给定 tf 内把
#     |v| 收到 0 且落到点，能量太大时 tf 不够。回头扫 tf 看能不能救。
#   下边界（alt 低）：alt=203 反而不行？—— 几乎没有时间/高度余量，
#     滑翔锥或推力上界在起始几帧就违反。
import numpy as np
# --- 路径自愈：analysis/ 下的脚本引用 solver/ 里的求解器 ---
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'solver'))
# --------------------------------------------------------
from gfold_terminal import solve_terminal

print('### 上边界：高能状态加大 tf（也加大 T_max 试）')
for (alt, d, vz, vh, m) in [(2549, 1880, -269, 207, 186e3),
                            (2996, 2242, -300, 250, 190e3),
                            (1922, 1386, -207, 190, 181e3)]:
    row = []
    for tf in [20, 30, 40, 60, 90]:
        r = solve_terminal(alt=alt, dist=d, vz=vz, vh=vh, mass=m, tf=tf, N=60,
                           gs_deg=30.0, pcs_start_deg=80.0, pcs_end_deg=5.0)
        row.append('tf%2d:%s' % (tf, 'OK ' if r['status'] == 'optimal' else '-- '))
    print('  alt=%4d |v|=%5.0f  %s' % (alt, np.hypot(vz, vh), ' '.join(row)))

print()
print('### 上边界：能量判据 |v| 与 a_net 的关系')
G0 = 9.80665; TMAX = 12.8e6
for (alt, d, vz, vh, m) in [(2549, 1880, -269, 207, 186e3),
                            (1922, 1386, -207, 190, 181e3),
                            (1727, 1236, -269, 190, 184e3)]:
    a_net = TMAX / m - G0
    v = np.hypot(vz, vh)
    tf = 20
    ah = vh / tf
    av = abs(vz) / tf + G0
    print('  alt=%4d m=%.0ft a_net=%5.1f |v|=%5.1f  need: a_h=%5.1f a_v=%5.1f |a|=%5.1f  %s'
          % (alt, m / 1000, a_net, v, ah, av, np.hypot(ah, av),
             'FEASIBLE' if np.hypot(ah, av) <= a_net else 'OVER-BUDGET'))

print()
print('### 下边界：低空为什么不行（alt 100~400，tf 扫）')
for alt in [400, 300, 254, 203, 163, 111]:
    d = alt * 0.75
    vz = -alt * 0.55
    vh = alt * 0.40
    row = []
    for tf in [8, 12, 20, 30]:
        r = solve_terminal(alt=alt, dist=d, vz=vz, vh=vh, mass=172e3, tf=tf, N=60,
                           gs_deg=30.0, pcs_start_deg=80.0, pcs_end_deg=5.0)
        row.append('tf%2d:%s' % (tf, 'OK ' if r['status'] == 'optimal' else '-- '))
    print('  alt=%4d dist=%4.0f vz=%6.1f vh=%5.1f  %s'
          % (alt, d, vz, vh, ' '.join(row)))

print()
print('### 下边界：是否由滑翔锥在起始帧违反造成（alt 低 dist/alt 比大）')
for alt in [400, 254, 203, 163]:
    d = alt * 0.75
    print('  alt=%4d dist=%6.1f  dist/alt=%.3f  gs须>=? 1/tan(gs)>=0.75 => gs<=%.1f°'
          % (alt, d, d / alt, np.degrees(np.arctan(1 / (d / alt)))))
    for gs in [30, 45, 60, 75]:
        r = solve_terminal(alt=alt, dist=d, vz=-alt * 0.55, vh=alt * 0.40,
                           mass=172e3, tf=12, N=60, gs_deg=gs,
                           pcs_start_deg=80.0, pcs_end_deg=5.0)
        print('      gs=%2d° -> %s' % (gs, r['status']))
