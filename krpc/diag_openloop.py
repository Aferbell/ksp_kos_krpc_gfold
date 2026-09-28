# diag_openloop.py — 开环复现参考轨迹（无 PD、无 clamp）
# 目的：判定发散到底来自【跟踪律】还是【积分/执行】。
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..', 'gfold', 'solver'))
from gfold_p3p4 import solve_p3p4_state        # noqa: E402

G0, ISP, TMAX, TALT = 9.80665, 315.0, 8.99e6, 36.8


def main():
    alt0, dist0, vz0, vh0, m0 = 5031.8, 1285.0, -314.0, 361.5, 182800.0
    r = solve_p3p4_state([alt0 - TALT, -dist0, 0, vz0, vh0, 0], mass=m0,
                         isp=ISP, t_max=TMAX, N3=160, N4=80, target_alt=TALT)
    x, u, N, tf = r['x'], r['u'], r['u'].shape[1], r['tf']
    dt_plan = tf / N
    print('tf=%.2f N=%d dt_plan=%.4f' % (tf, N, dt_plan))

    # ---- 开环：直接按规划 dt 喂 u，无任何反馈 ----
    p = x[0:3, 0].copy()
    v = x[3:6, 0].copy()
    m = m0
    print('\n=== 开环：严格按 dt_plan 喂参考 u ===')
    print(' n   u=[up, y]        p=[h, y]            v=[vx, vy]')
    for n in range(N - 1):
        acc_t = u[:, n]
        acc = acc_t + np.array([-G0, 0, 0])
        v = v + acc * dt_plan
        p = p + v * dt_plan
        m = m - float(np.linalg.norm(acc_t)) * m / (ISP * G0) * dt_plan
        if n % 8 == 0 or n >= N - 3:
            print('%3d  [%7.2f %7.2f]  [%8.1f %8.1f]  [%7.1f %7.1f]'
                  % (n, u[0, n], u[1, n], p[0], p[1], v[0], v[1]))
    print('\n开环终点: alt=%.2f  y=%.2f  vx=%.2f  vy=%.2f'
          % (p[0] + TALT, p[1], v[0], v[1]))
    print('参考终点: alt=%.2f  y=%.2f  vx=%.2f  vy=%.2f'
          % (x[0, -1] + TALT, x[1, -1], x[3, -1], x[4, -1]))

    # ---- 半闭环：用参考的 x 做参考点，但按小步长积分 ----
    print('\n=== 半闭环：dt=0.02，参考点按时间索引，无 PD 修正 ===')
    p = x[0:3, 0].copy()
    v = x[3:6, 0].copy()
    m = m0
    dt = 0.02
    t = 0.0
    while p[0] > 0.5 and t < 60:
        j = int(min(N - 1, max(0, round(t / dt_plan))))
        acc_t = u[:, j]
        acc = acc_t + np.array([-G0, 0, 0])
        v = v + acc * dt
        p = p + v * dt
        m = m - float(np.linalg.norm(acc_t)) * m / (ISP * G0) * dt
        t += dt
    print('终点: alt=%.2f  y=%.2f  vx=%.2f  vy=%.2f  (t=%.1f)'
          % (p[0] + TALT, p[1], v[0], v[1], t))


if __name__ == '__main__':
    main()
