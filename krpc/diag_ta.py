# diag_ta.py — 打印 target_a 的逐项分解，定位 thr 崩塌
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..', 'gfold', 'solver'))
from gfold_p3p4 import solve_p3p4_state        # noqa: E402

G0 = 9.80665
ISP, TMAX, TARGET_ALT = 315.0, 8.99e6, 36.8
K_X, K_V = 0.5, 0.8
MT0, MT1 = 85.0, 25.0


def clamp(n, mx, mn):
    return mx if n > mx else (mn if n < mn else n)


def conic(ta, min_mag, max_mag, mt):
    hv = np.array([0.0, ta[1], ta[2]])
    ah = float(np.linalg.norm(hv))
    av = float(ta[0])
    if ah < 1e-9:
        return np.array([clamp(av, max_mag, min_mag), 0.0, 0.0])
    hd = hv / ah
    if av < 0:
        av = 0.0
    cap = av * math.tan(mt)
    if ah > cap:
        ah = cap
    m2 = math.hypot(av, ah)
    if m2 < min_mag:
        if m2 < 1e-9:
            return np.array([min_mag, 0.0, 0.0])
        k = min_mag / m2
        av, ah = av * k, ah * k
    m3 = math.hypot(av, ah)
    if m3 > max_mag:
        k = max_mag / m3
        av, ah = av * k, ah * k
    return hd * ah + np.array([av, 0.0, 0.0])


def near(x, p, tf, N):
    nm = float(np.linalg.norm(x[0:3, 0] - p))
    ni = 0
    for i in range(x.shape[1]):
        m = float(np.linalg.norm(x[0:3, i] - p))
        if m < nm:
            nm, ni = m, i
    vv = x[3:6, ni]
    vn = float(np.linalg.norm(vv))
    if vn < 1e-6:
        return float(ni)
    return ni + clamp(float(np.dot(p - x[0:3, ni], vv / vn))
                      / ((tf / N) * vn), 0.5, -0.5)


def lerp(a, b, t):
    return t * b + (1 - t) * a


def samp(x, u, idx, N):
    if idx >= N - 1:
        return np.zeros(3), np.zeros(3), np.array([G0, 0, 0])
    if idx <= 0:
        i, fr = 0, idx
    else:
        i, fr = int(math.floor(idx)), idx - math.floor(idx)
    xs = lerp(x[:, i], x[:, i + 1], fr)
    us = lerp(u[:, i], u[:, i + 1], fr)
    if idx < 0:
        us = u[:, 1].copy()
    return xs[0:3].copy(), xs[3:6].copy(), us.copy()


def main():
    alt0, dist0, vz0, vh0, m0 = 5031.8, 1285.0, -314.0, 361.5, 182800.0
    r = solve_p3p4_state([alt0 - TARGET_ALT, -dist0, 0, vz0, vh0, 0],
                         mass=m0, isp=ISP, t_max=TMAX, N3=160, N4=80,
                         target_alt=TARGET_ALT)
    x, u, N, tf = r['x'], r['u'], r['u'].shape[1], r['tf']
    p = np.array([alt0 - TARGET_ALT, -dist0, 0.0])
    v = np.array([vz0, vh0, 0.0])
    m = m0
    dt, t, n_i = 0.02, 0.0, -100.0

    print('step    t     n_i    u_i                    PD                      ta(前clamp)          tilt  thr')
    for k in range(400):
        n_i = max(n_i - dt * 0.2 * N / tf, near(x, p, tf, N))
        xi, vi, ui = samp(x, u, n_i, N)
        st = min(1.5 * N / tf, float(np.linalg.norm(v)) / 50.0 * N / tf)
        xi_, vi_, ui_ = samp(x, u, n_i + st, N)
        pd = (vi - v) * K_V + (xi - p) * K_X
        ta = ui + pd
        prog = min(1.0, max(0.0, n_i / max(1, N - 1)))
        mt = math.radians(MT0 + (MT1 - MT0) * prog)
        a_cap = TMAX / m
        tac = conic(ta, 0.05 * a_cap, a_cap, mt)
        if n_i < 0:
            tac = np.array([G0, 0, 0]) + ui
        thr = float(np.linalg.norm(tac)) / a_cap
        if k % 25 == 0 or (0.0 < t < 3.0 and k % 5 == 0):
            print('%4d %5.2f %7.2f  [%7.2f %7.2f]  [%8.2f %8.2f]  [%8.2f %8.2f] %6.1f %5.3f'
                  % (k, t, n_i, ui[0], ui[1], pd[0], pd[1],
                     tac[0], tac[1], math.degrees(math.atan2(
                         abs(tac[1]), max(1e-9, tac[0]))), thr))
        acc_t = tac / max(1e-9, float(np.linalg.norm(tac))) \
            * (thr * a_cap)
        v = v + (acc_t + np.array([-G0, 0, 0])) * dt
        p = p + v * dt
        m = m - float(np.linalg.norm(acc_t)) * m / (ISP * G0) * dt
        t += dt
        if p[0] <= 0.5:
            break


if __name__ == '__main__':
    main()
