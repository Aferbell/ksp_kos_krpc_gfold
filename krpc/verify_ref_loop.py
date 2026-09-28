# verify_ref_loop.py — 用参考仓库结构做 3DoF 闭环（不连 KSP）
#
# 【目的】照抄参考仓库后，验证闭环能真的落地。
#   复现的环节（与 gfold_land.py 一致）：
#     · P3/P4 两级求解（gfold_p3p4）
#     · find_nearest_index + sample_index 索引跟踪
#     · conic_clamp 限幅
#     · PD: target_a = u_i + (v_i-vel)*k_v + (x_i-error)*k_x
#
# 【为什么要单独跑】实飞一次成本高；先用 3DoF 确认结构方向是对的。
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..', 'gfold', 'solver'))
from gfold_p3p4 import solve_p3p4_state   # noqa: E402

G0 = 9.80665
ISP = 315.0
TMAX = 8.99e6
TARGET_ALT = 36.8
K_X = 0.5          # 参考 params.txt
K_V = 0.8          # 参考 params.txt
MAX_TILT_DEG = 25.0
MAX_TILT_START_DEG = 85.0


def clamp(num, maxnum, minnum):
    if num > maxnum:
        return maxnum
    elif num < minnum:
        return minnum
    return num


def lerp(a, b, t):
    return t * b + (1 - t) * a


def conic_clamp(ta, min_mag, max_mag, max_tilt):
    """与 gfold_land.conic_clamp 一致（削水平、不抬竖直、保幅值）。"""
    a_hor_vec = np.array([0.0, ta[1], ta[2]])
    a_hor = float(np.linalg.norm(a_hor_vec))
    a_ver = float(ta[0])
    if a_hor < 1e-9:
        return np.array([clamp(a_ver, max_mag, min_mag), 0.0, 0.0])
    hor_dir = a_hor_vec / a_hor
    if a_ver < 0.0:
        a_ver = 0.0
    cap_hor = a_ver * math.tan(max_tilt)
    if a_hor > cap_hor:
        a_hor = cap_hor
    mag2 = math.hypot(a_ver, a_hor)
    if mag2 < min_mag:
        if mag2 < 1e-9:
            return np.array([min_mag, 0.0, 0.0])
        k = min_mag / mag2
        a_ver, a_hor = a_ver * k, a_hor * k
    mag3 = math.hypot(a_ver, a_hor)
    if mag3 > max_mag:
        k = max_mag / mag3
        a_ver, a_hor = a_ver * k, a_hor * k
    return hor_dir * a_hor + np.array([a_ver, 0.0, 0.0])


def find_nearest_index(x, r, tf, N):
    nearest_mag = float(np.linalg.norm(x[0:3, 0] - r))
    nearest_i = 0
    for i in range(x.shape[1]):
        mag = float(np.linalg.norm(x[0:3, i] - r))
        if mag < nearest_mag:
            nearest_mag = mag
            nearest_i = i
    vv = x[3:6, nearest_i]
    vn = float(np.linalg.norm(vv))
    if vn < 1e-6:
        return float(nearest_i)
    frac = clamp(float(np.dot(r - x[0:3, nearest_i], vv / vn))
                 / ((tf / N) * vn), 0.5, -0.5)
    return nearest_i + frac


def sample_index(x, u, index, N):
    if index >= N - 1:
        return np.zeros(3), np.zeros(3), np.array([G0, 0.0, 0.0])
    if index <= 0:
        i, frac = 0, index
    else:
        i = int(math.floor(index))
        frac = index - i
    xs = lerp(x[:, i], x[:, i + 1], frac)
    us = lerp(u[:, i], u[:, i + 1], frac)
    if index < 0:
        us = u[:, 1].copy()
    return xs[0:3].copy(), xs[3:6].copy(), us.copy()


def main():
    # 实飞交班点
    alt0, dist0, vz0, vh0, m0 = 5031.8, 1285.0, -314.0, 361.5, 182800.0
    EST_TIME = float(os.environ.get('EST_TIME', '0.5'))
    RESOLVE_DT = float(os.environ.get('RESOLVE_DT', '2.0'))
    print('=' * 76)
    print('参考仓库结构 3DoF 闭环验证')
    print('  入场 alt=%.0f dist=%.0f vz=%.0f vh=%.0f m=%.1ft'
          % (alt0, dist0, vz0, vh0, m0 / 1000))
    print('  est_time=%.2fs  resolve_dt=%.1fs' % (EST_TIME, RESOLVE_DT))
    print('=' * 76)

    x0 = [alt0 - TARGET_ALT, -dist0, 0.0, vz0, vh0, 0.0]
    r = solve_p3p4_state(x0, mass=m0, isp=ISP, t_max=TMAX, N3=160, N4=80,
                         target_alt=TARGET_ALT)
    if r['status'] != 'optimal':
        print('求解失败:', r['status'])
        return 1
    x, u, N, tf = r['x'], r['u'], r['u'].shape[1], r['tf']
    print('\n[P3/P4] optimal  tf_m=%.1f  tf=%.1f  落地 %.1ft  末倾 %.2f°  %.1fs'
          % (r['tf_m'], tf, r['mass_land'] / 1000, r['end_tilt_deg'],
             r['solve_time']))

    # 状态（目标系：x=上, y/z=水平，【三维带符号】，与参考仓库一致）
    p = np.array([alt0 - TARGET_ALT, -dist0, 0.0])
    v = np.array([vz0, vh0, 0.0])
    m = m0
    dt = 0.02
    t = 0.0
    n_i = -100.0
    log = []
    prev_v = None
    prev_acc = None
    last_resolve = -1e9
    resolve_n = 0
    print('\n   t     alt    dist     vz      vh    tilt   thr')
    while p[0] > 0.5 and t < 120:
        # ---- 状态外推 + 周期性重解（照抄参考仓库 line 440-458 / 174-175）----
        acc = (v - prev_v) / dt if prev_v is not None else np.zeros(3)
        prev_v = v.copy()
        if t - last_resolve > RESOLVE_DT:
            est = EST_TIME
            vel_est = v + acc * est
            pos_est = p + v * est + 0.5 * acc * est * est
            r2 = solve_p3p4_state([pos_est[0], pos_est[1], pos_est[2],
                                   vel_est[0], vel_est[1], vel_est[2]],
                                  mass=m, isp=ISP, t_max=TMAX, N3=160, N4=80,
                                  target_alt=TARGET_ALT)
            print('  [resolve] t=%.1f alt=%.0f vz=%.0f vh=%.0f -> %s'
                  % (t, pos_est[0] + TARGET_ALT, vel_est[0],
                     math.hypot(vel_est[1], vel_est[2]), r2['status']))
            if r2['status'] == 'optimal':
                x, u, N, tf = r2['x'], r2['u'], r2['u'].shape[1], r2['tf']
                r = r2
                n_i = -100.0
                resolve_n += 1
                last_resolve = t
        n_i = max(n_i - dt * 0.2 * N / tf, find_nearest_index(x, p, tf, N))
        x_i, v_i, u_i = sample_index(x, u, n_i, N)
        step = min(1.5 * N / tf, float(np.linalg.norm(v)) / 50.0 * N / tf)
        x_i_, v_i_, u_i_ = sample_index(x, u, n_i + step, N)

        ta = u_i + (v_i - v) * K_V + (x_i - p) * K_X
        ta_ = u_i_ + (v_i_ - v) * K_V + (x_i_ - p) * K_X

        a_cap = TMAX / m
        # 锥角跟随参考轨迹自己的倾角（+8° 裕度）—— 见 gfold_land.track() 说明
        tilt_traj = math.degrees(math.atan2(
            float(np.linalg.norm(u_i[1:3])), max(1e-9, float(u_i[0]))))
        mt_now = math.radians(max(tilt_traj + 8.0, MAX_TILT_DEG))
        ta = conic_clamp(ta, 0.05 * a_cap, 1.0 * a_cap, mt_now)
        ta_ = conic_clamp(ta_, 0.05 * a_cap, 1.0 * a_cap, mt_now)
        if n_i < 0:
            ta = np.array([G0, 0.0, 0.0]) + u_i

        thr = float(np.linalg.norm(ta)) / a_cap
        tilt = math.degrees(math.atan2(float(np.linalg.norm(ta[1:3])),
                                       max(1e-9, float(ta[0]))))
        # ---- 积分：必须与求解器【同一套动力学（梯形法）】----
        # 【踩过的坑】原先用欧拉法 `v += acc*dt; p += v*dt`，
        #   而求解器用的是梯形法
        #       v[n+1] = v[n] + dt/2*((u_n+g)+(u_{n+1}+g))
        #       x[n+1] = x[n] + dt/2*(v[n]+v[n+1])
        #   两者相差 dt/2*(u_{n+1}-u_n)，实测单步 ~0.04 m/s²，
        #   80 步累积 ~12.8 m/s —— 于是"参考轨迹本身"在欧拉积分下
        #   落点偏 107 m（开环复现证实），PD 就会去追这个【虚假误差】，
        #   表现为闭环发散。这不是控制律的问题，是仿真器口径不一致。
        acc_t = ta_ / max(1e-9, float(np.linalg.norm(ta_))) * (thr * a_cap)
        acc_new = acc_t + np.array([-G0, 0.0, 0.0])
        if prev_acc is None:
            prev_acc = acc_new
        v_new = v + 0.5 * (prev_acc + acc_new) * dt
        p = p + 0.5 * (v + v_new) * dt
        prev_acc = acc_new
        v = v_new
        m = m - float(np.linalg.norm(acc_t)) * m / (ISP * G0) * dt
        t += dt
        if int(t * 2) % 4 == 0:
            log.append((t, p[0] + TARGET_ALT, float(np.linalg.norm(p[1:3])),
                        v[0], float(np.linalg.norm(v[1:3])), tilt, thr))

    for row in log[::max(1, len(log) // 14)]:
        print('  %5.1f %7.1f %7.1f %7.2f %7.2f %6.2f %6.3f'
              % (row[0], row[1], row[2], row[3], row[4], row[5], row[6]))

    vz_e, vh_e = float(v[0]), float(np.linalg.norm(v[1:3]))
    d_e = float(np.linalg.norm(p[1:3]))
    h_e = float(p[0])
    print('\n  重解次数 = %d' % resolve_n)
    print('  退出: alt=%.2f  vz=%.2f  vh=%.2f  dist=%.2f  (t=%.1fs)'
          % (h_e + TARGET_ALT, vz_e, vh_e, d_e, t))

    ok = True
    print('\n' + '=' * 76)
    print('判据')
    print('=' * 76)
    for nm, cond, val in [
            ('触地/接近地面 alt < 2 m', h_e < 2.0, h_e),
            ('落地垂速 |vz| < 8 m/s', abs(vz_e) < 8.0, abs(vz_e)),
            ('落地横速 vh < 5 m/s', vh_e < 5.0, vh_e),
            ('落点偏差 < 10 m', d_e < 10.0, d_e)]:
        print('  [%s] %-26s 实测 %.2f' % ('OK ' if cond else 'FAIL', nm, val))
        ok = ok and cond
    print('\n结论:', '全部通过 —— 参考仓库结构可闭环落地' if ok else '有未通过项')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
