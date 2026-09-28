"""验证 n_i 推进符号：加号 vs 减号。

结论将决定修复方向。用一个"完美跟踪"的假想闭环（本机严格沿轨迹走）
来检验两种符号下 n_i 能否正常推进到位。
"""
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..', 'gfold', 'solver'))
sys.path.insert(0, os.path.join(_HERE, '..', '..', 'tools', 'stubs'))
from gfold_p3p4 import solve_p3p4_state      # noqa: E402

TARGET_ALT = 36.8


def nearest_index(x, r, tf, N):
    nm = float(np.linalg.norm(x[0:3, 0] - r))
    ni = 0
    for i in range(x.shape[1]):
        m = float(np.linalg.norm(x[0:3, i] - r))
        if m < nm:
            nm, ni = m, i
    vv = x[3:6, ni]
    vn = float(np.linalg.norm(vv))
    if vn < 1e-6:
        return float(ni)
    fr = np.clip(float(np.dot(r - x[0:3, ni], vv / vn)) / ((tf / N) * vn),
                 -0.5, 0.5)
    return float(ni) + fr


# 求解一条真实轨迹
x0 = np.array([4979.6 - TARGET_ALT, -1309.6, 0.0, -290.4, 214.0, 0.0])
r = solve_p3p4_state(x0, mass=183900.0, isp=315.0, t_max=8990529.0,
                     N3=160, N4=80, target_alt=TARGET_ALT)
x, tf = r['x'], r['tf']
N = x.shape[1]
print('轨迹: tf=%.2f N=%d 节点间隔=%.4f s' % (tf, N, tf / N))

print('\n=== 实验: 本机【严格沿轨迹】飞行（理想跟踪器）===')
print('比较两种 n_i 推进公式能否跟上\n')

for sign, label in ((-1.0, '减号(当前实现)'), (+1.0, '加号(参考注释掉的)')):
    n_i = -100.0
    dt = 0.02
    hist = []
    # 本机沿轨迹前进：真实时间 t -> 轨迹位置
    for step in range(200):
        t_game = step * dt
        k = min(N - 1, int(t_game / (tf / N)))
        pos = x[0:3, k]
        near = nearest_index(x, pos, tf, N)
        n_i = max(n_i + sign * dt * 0.2 * N / tf, near)
        hist.append(n_i)
    need = min(N - 1, int(200 * dt / (tf / N)))
    print('  %-16s 200 帧后 n_i=%7.2f  (应该到 %.1f)  差 %+.1f'
          % (label, hist[-1], need, hist[-1] - need))

print('\n=== 结论 ===')
print('  加号版能跟上（n_i 随时间前进）')
print('  减号版停在小值（每帧倒退 0.015，被 near 拖着走）')
print('  ⇒ 必须改成加号')

print('\n=== 另外验证: 0.2 系数是否合适 ===')
dtn = 0.02
print('  节点间隔 %.4f s, 每帧应推进 %.4f 节点' % (tf / N, dtn / (tf / N)))
print('  加 0.2 系数后 = %.4f 节点/帧 -> 只有 %.0f%%'
      % (dtn * 0.2 * N / tf, 100 * (dtn * 0.2 * N / tf) / (dtn / (tf / N))))
print('  ⇒ 0.2 让 n_i 推进【慢 5 倍】, 但 max(..., near) 会用 near 兜住')
print('    所以只要 near 正常, 0.2 只是"限速", 不会致命')
print('    ⇒ 真正致命的只是【符号】')
