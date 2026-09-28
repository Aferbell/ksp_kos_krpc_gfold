# diag_loop.py — 逐步追踪闭环第一步，定位发散点
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..', 'gfold', 'solver'))
from gfold_p3p4 import solve_p3p4_state    # noqa: E402

G0 = 9.80665
ISP = 315.0
TMAX = 8.99e6
TARGET_ALT = 36.8
K_X, K_V = 0.5, 0.8
MAX_TILT = math.radians(25.0)


def main():
    alt0, dist0, vz0, vh0, m0 = 5031.8, 1285.0, -314.0, 361.5, 182800.0
    r = solve_p3p4_state([alt0 - TARGET_ALT, -dist0, 0.0, vz0, vh0, 0.0],
                         mass=m0, isp=ISP, t_max=TMAX, N3=160, N4=80,
                         target_alt=TARGET_ALT)
    x, u, N, tf = r['x'], r['u'], r['u'].shape[1], r['tf']
    print('tf=%.2f N=%d' % (tf, N))
    print('\n=== 参考轨迹采样（x[0]=高度 x[1]=y x[2]=z | u[0]=竖直 u[1]=y u[2]=z）===')
    for i in range(0, N, max(1, N // 10)):
        print('n=%2d  x=[%8.1f %8.1f %6.1f]  v=[%7.1f %7.1f %6.1f]  u=[%6.2f %6.2f %5.2f]'
              % (i, x[0, i], x[1, i], x[2, i], x[3, i], x[4, i], x[5, i],
                 u[0, i], u[1, i], u[2, i]))

    # 追踪前 5 步
    p = np.array([alt0 - TARGET_ALT, -dist0, 0.0])
    v = np.array([vz0, vh0, 0.0])
    m = m0
    dt = 0.02
    n_i = -100.0
    print('\n=== 闭环前 5 步 ===')
    print('step   n_i    |u|    thr    a_cmd=[up, y, z]           |a|/a_cap   p=[h,y,z] v=[..]')
    for step in range(5):
        # find_nearest_index
        nm = float(np.linalg.norm(x[0:3, 0] - p))
        ni = 0
        for i in range(N):
            mm = float(np.linalg.norm(x[0:3, i] - p))
            if mm < nm:
                nm, ni = mm, i
        vv = x[3:6, ni]
        vn = float(np.linalg.norm(vv))
        frac = np.clip(float(np.dot(p - x[0:3, ni], vv / max(1e-9, vn)))
                       / ((tf / N) * max(1e-9, vn)), -0.5, 0.5)
        n_i = max(n_i - dt * 0.2 * N / tf, ni + frac)
        idx = int(min(N - 1, max(0, math.floor(n_i)))) if n_i > 0 else 0
        us = u[:, idx]
        xs = x[:, idx]
        ta = us + (xs[3:6] - v) * K_V + (xs[0:3] - p) * K_X
        a_cap = TMAX / m
        print('%4d  %6.2f  %6.2f  %6.3f  [%8.2f %8.2f %6.2f]  %8.3f   p=[%7.1f %8.1f] v=[%7.1f %7.1f]'
              % (step, n_i, np.linalg.norm(us), np.linalg.norm(ta) / a_cap,
                 ta[0], ta[1], ta[2], np.linalg.norm(ta) / a_cap,
                 p[0], p[1], v[0], v[1]))
        # 简化积分：用未限幅指令（看趋势）
        acc = ta + np.array([-G0, 0, 0])
        v = v + acc * dt
        p = p + v * dt
        m = m - float(np.linalg.norm(ta)) * m / (ISP * G0) * dt


if __name__ == '__main__':
    main()
