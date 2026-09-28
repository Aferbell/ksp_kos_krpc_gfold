# diag10_profile.py — 检查解出来的剖面是否有物理缺陷（贴地回弹 / 爬升）
#
# 观察：validate_final.py 打印的剖面出现
#   t=15.3 h=121.4  vz=-123
#   t=18.4 h= 33.5  vz=+45.8   <-- 高度触底后 vz 变正（爬升）
#   t=21.4 h=136.5  vz=+20.0
#   t=27.6 h= 80.5  vz=-38.6
# 这在物理上荒谬（落到 33 m 又爬回 152 m）。原因排查：
#   ① 末端 s[N-1]=0 强制末推力为 0；末速也必须为 0 -> 末段必须"无推力悬停"，
#      不可能；求解器于是用"俯冲-再爬"的畸形方式凑边界。
#   ② 缺少"不许爬升"约束（我们的竖直铁律）。
#   ③ 目标函数只有燃料项，对轨迹形状无惩罚。
# 修法：加 h >= 0（不穿地）与 vz <= 0 单调下降约束，并给末端留小推力。
import numpy as np
# --- 路径自愈：analysis/ 下的脚本引用 solver/ 里的求解器 ---
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'solver'))
# --------------------------------------------------------
from gfold_terminal import solve_terminal

r = solve_terminal(alt=5950, dist=4885, vz=-515.7, vh=476.8, mass=208.1e3,
                   tf=30, N=50, gs_deg=30.0, pcs_start_deg=85.0, pcs_end_deg=5.0)
print('status:', r['status'])
x, u = r['x'], r['u']
N = x.shape[1]
print('\n全部节点（细看有无穿地/爬升）:')
print('  i   t(s)   高度    垂速    推力幅值   s')
for i in range(N):
    ti = i * r['tf'] / (N - 1)
    flag = ''
    if x[0, i] < -0.5:
        flag += ' <穿地>'
    if x[3, i] > 1.0:
        flag += ' <爬升>'
    print('  %2d %5.1f %8.2f %8.1f %9.2f %8.3f%s'
          % (i, ti, x[0, i], x[3, i], np.linalg.norm(u[:, i]), r['s'][0, i], flag))

print('\n最低高度 = %.2f m' % x[0, :].min())
print('垂速最大值 = %.2f m/s ( >0 表示爬升)' % x[3, :].max())
print('末端 s = %.4f  (强制为 0)' % r['s'][0, -1])
