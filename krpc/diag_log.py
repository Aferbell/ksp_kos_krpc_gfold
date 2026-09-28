# diag_log.py — 用最新实飞日志的实际数字，复现 track() 每一步
#
# 目的：确定 tilt_cmd=0 到底是哪条分支产生的，不再猜。
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..', 'gfold', 'solver'))
from gfold_p3p4 import solve_p3p4_state      # noqa: E402

G0, ISP, TMAX, TALT = 9.80665, 315.0, 8.99e6, 36.8
K_POS, K_VEL = 0.5, 0.8


def find_nearest_index(x, r, tf, N):
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
    return ni + fr


def sample_index(x, u, idx, N):
    if idx >= N - 1:
        return np.zeros(3), np.zeros(3), np.array([9.807, 0, 0])
    if idx <= 0:
        i, fr = 0, idx
    else:
        i, fr = int(math.floor(idx)), idx - math.floor(idx)
    xs = (1 - fr) * x[:, i] + fr * x[:, i + 1]
    us = (1 - fr) * u[:, i] + fr * u[:, i + 1]
    if idx < 0:
        us = u[:, 1].copy()
    return xs[0:3].copy(), xs[3:6].copy(), us.copy()


def conic_clamp(ta, min_mag, max_mag, max_tilt):
    hv = np.array([0.0, ta[1], ta[2]])
    ah = float(np.linalg.norm(hv))
    if ah < 1e-9:
        return np.array([min(max(ta[0], min_mag), max_mag), 0.0, 0.0]), 'ZERO_HOR'
    hd = hv / ah
    av = max(0.0, float(ta[0]))
    cap = av * math.tan(max_tilt)
    if ah > cap:
        ah = cap
    m2 = math.hypot(av, ah)
    if m2 < min_mag:
        if m2 < 1e-9:
            return np.array([min_mag, 0.0, 0.0]), 'MIN_PURE_VER'
        k = min_mag / m2
        av, ah = av * k, ah * k
    m3 = math.hypot(av, ah)
    if m3 > max_mag:
        k = max_mag / m3
        av, ah = av * k, ah * k
    return hd * ah + np.array([av, 0.0, 0.0]), ''


def main():
    # 日志首帧（交班点）
    alt0, dist0, vz0, vh0, m0 = 4994.0, 1314.0, -287.0, 221.0, 184200.0
    r = solve_p3p4_state([alt0 - TALT, -dist0, 0, vz0, vh0, 0], mass=m0,
                         isp=ISP, t_max=TMAX, N3=160, N4=80, target_alt=TALT)
    print('P3/P4:', r['status'], 'tf=%.2f' % r.get('tf', 0))
    x, u, N, tf = r['x'], r['u'], r['u'].shape[1], r['tf']
    print('N=%d  tf=%.2f' % (N, tf))

    p = np.array([alt0 - TALT, -dist0, 0.0])
    v = np.array([vz0, vh0, 0.0])
    m = m0
    dt = 0.02
    n_i = -100.0
    t = 0.0
    print('\n t     n_i    near   u_i            a_hor   tilt  thr   branch')
    while p[0] > 0.5 and t < 30:
        near = find_nearest_index(x, p, tf, N)
        n_i = max(n_i - dt * 0.2 * N / tf, near)
        xi, vi, ui = sample_index(x, u, n_i, N)
        step = min(1.5 * N / tf, float(np.linalg.norm(v)) / 50.0 * N / tf)
        _xi_, _vi_, _ui_ = sample_index(x, u, n_i + step, N)
        ta = ui + (vi - v) * K_VEL + (xi - p) * K_POS
        # 注：真实代码里 target_direction 用前瞻点 ta_（这里不画姿态，故略）
        a_cap = TMAX / m
        tilt_traj = math.degrees(math.atan2(
            float(np.linalg.norm(ui[1:3])), max(1e-9, float(ui[0]))))
        mt = math.radians(min(89.0, max(tilt_traj + 8.0, 25.0)))
        tac, br = conic_clamp(ta, 0.05 * a_cap, a_cap, mt)
        a_hor = float(np.linalg.norm(tac[1:3]))
        tilt = math.degrees(math.atan2(a_hor, max(1e-9, float(tac[0]))))
        thr = float(np.linalg.norm(tac)) / a_cap
        if int(t * 10) % 10 == 0 or br:
            print('%5.1f %7.2f %7.2f  [%6.2f %6.2f] %6.2f %5.1f %5.3f %s'
                  % (t, n_i, near, ui[0], ui[1], a_hor, tilt, thr, br))
        # 梯形积分（与求解器一致）
        acc_t = tac
        v_new = v + (acc_t + np.array([-G0, 0, 0])) * dt
        p = p + 0.5 * (v + v_new) * dt
        v = v_new
        t += dt
    print('\n终点 alt=%.1f dist=%.1f vz=%.1f vh=%.1f'
          % (p[0] + TALT, float(np.linalg.norm(p[1:3])), v[0],
             float(np.linalg.norm(v[1:3]))))


if __name__ == '__main__':
    main()
