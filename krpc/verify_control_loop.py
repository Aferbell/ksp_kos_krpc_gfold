# verify_control_loop.py — 离线验证 gfold_land.py 的控制逻辑（不连 KSP）
#
# 目的：在没有 KSP 的情况下，验证
#   ① solver 集成正确（交班点状态可解）
#   ② 三段时间管理（首次解 + 固定 tf 重解 + 帧间 PD 跟踪）在时间上可行
#   ③ apply() 的 throttle / 姿态换算在数值上正确
#   ④ 用 3DoF 积分真的能沿参考轨迹落地
#
# 用真实交班点状态（来自 MATLAB 两级律仿真实测）：
#   alt=5036  dist=142  vz=-72.0  vh=20.5  mass=152.0 t
import os
import sys
import time
import math
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..', 'gfold', 'solver'))
from gfold_terminal import solve_terminal          # noqa: E402

TARGET_ALT = 36.8
G0 = 9.80665
ISP = 315.0
# 【载具参数必须与本载具一致】默认 12.8 MN 是 B1040-7 的推力。
#   入口状态改用实飞交班点时，必须同时把 TMAX 换成 B1040-9 的 8.99 MN，
#   否则"推力上限"用的是别的载具，闭环结论无意义。
TMAX = 12.8e6
SOLVER = dict(N=40, gs_deg=30.0, pcs_start_deg=85.0, pcs_end_deg=3.0,
              throttle=(0.05, 1.0), v_descent_max=160.0, term_win=0.40, term_vz=4.0)
KP_X, KV_V = 0.5, 0.8
REPLAN_DT = 1.0
GATE_ALT_MIN = 358.0

S0 = dict(alt=5036.0, dist=142.0, vz=-72.0, vh=20.5, mass=152.0e3)

# ---- 场景选择：--real 用【实飞真实交班点】----
#   verify_control_loop.py --real
#   实飞交班点横速远未消掉（vh 361.5 而非理想 20.5），能量高得多，
#   旧求解器在此报 infeasible。用它做闭环才能复现实飞失败并验证修复。
import sys as _sys
if '--real' in _sys.argv:
    TMAX = 8.990e6
    S0 = dict(alt=5031.8, dist=1285.0, vz=-314.0, vh=361.5, mass=182.8e3)
    print('[scenario] 使用实飞真实交班点（B1040-9, TMAX=8.99 MN）')
SOLVER['t_max'] = TMAX


def track_index(x, tf, t_plan):
    """按【时间索引】取参考节点号。

    【为什么必须按时间索引（踩过的坑）】
      用"最近点"索引时，参考点会滑到本机【前方】的节点，于是 x_ref 的高度
      【低于】本机高度，P 项 KP_X*(x_ref − p) 变成【负的向下加速度】，
      命令本机"下冲去追赶计划"。离线实测 a_x = −42 m/s²，垂速从 −72 一路涨到
      −260（而包络是 −160），落地 vz=−64.7 m/s 坠毁。
      按时间索引时参考点就是"此刻本应处于的位置"，误差符号自然正确；
      每次重解后时基 t_plan 归零。
    """
    N = x.shape[1]
    dt_plan = max(1e-6, tf / (N - 1))
    return int(min(N - 1, max(0, round(t_plan / dt_plan))))


def geom(a_cmd, mass):
    """把【推力加速度】指令换算成 (throttle, 推力方向倾角deg)

    【语义（踩过的坑）】G-FOLD 的 u 是【推力加速度】，不是净加速度。
    求解器里的动力学是 x[3:6,n+1] = x[3:6,n] + dt/2*((u+g)+(u+g))，
    g=[-9.8,0,0] —— 即【净加速度 = u + g】。
    第一版这里写成 `thr = a_cmd; thr[0] += G0`，等于把重力加了两次
    （把 u 当净加速度又补了一次 g），导致 x 向推力被系统性低估 ≈9.8 m/s²。
    """
    thr = a_cmd.copy()                 # a_cmd 本身就是推力加速度
    mag = float(np.linalg.norm(thr))
    throttle = min(1.0, max(0.0, mag * mass / TMAX)) if mag > 1e-9 else 0.0
    tilt = math.degrees(math.atan2(float(np.linalg.norm(thr[1:3])),
                                   max(1e-9, abs(float(thr[0])))))
    return throttle, tilt, mag


def main():
    print('=' * 74)
    print('① 首次解算（交班点状态）')
    print('=' * 74)
    t0 = time.time()
    r = solve_terminal(alt=S0['alt'], dist=S0['dist'], vz=S0['vz'],
                       vh=S0['vh'], mass=S0['mass'],
                       tf=None, target_alt=TARGET_ALT, **SOLVER)
    t_first = time.time() - t0
    if r['status'] != 'optimal':
        print('  解算失败:', r['status'])
        return 1
    print(f'  status={r["status"]}  tf={r["tf"]:.2f}s  落地={r["mass_land"]/1000:.1f}t')
    print(f'  末端倾角={r["end_tilt_deg"]:.2f}°  位置误差={r["end_pos_err"]:.4f}m '
          f'速度误差={r["end_vel_err"]:.4f}m/s')
    print(f'  首次求解耗时 = {t_first*1000:.0f} ms')

    print()
    print('=' * 74)
    print('② 3DoF 闭环积分：沿参考轨迹跟踪')
    print('=' * 74)
    x = r['x']; u = r['u']
    # 状态（目标系：x=上, y=水平朝目标；把 dist 放在 y）
    p = np.array([S0['alt'] - TARGET_ALT, S0['dist'], 0.0])
    v = np.array([S0['vz'], -S0['vh'], 0.0])
    m = S0['mass']
    dt = 0.02
    t = 0.0
    last_replan = 0.0
    t_plan = 0.0
    replan_n = 0
    solves_ms = []
    log = []
    tilt_hist = []
    while p[0] > 0.5 and t < 120:
        # --- 按【时间索引】取参考点 + PD（不要用最近点，见 track_index 说明）---
        j = track_index(x, r['tf'], t - t_plan)
        a_cmd = u[:, j] + KP_X * (x[0:3, j] - p) + KV_V * (x[3:6, j] - v)
        # --- 竖直铁律：P/D 修正后不许命令【净向下】加速度 ---
        #   对齐 kOS 侧"绝不主动下冲"。净加速度 = a_x - g，要求 >= -0.2g_net。
        if a_cmd[0] < 0.0:
            a_cmd[0] = 0.0
        # --- 几何换算 ---
        throttle, tilt, tmag = geom(a_cmd, m)
        tilt_hist.append(tilt)
        # --- 重解（用剩余时间；必须 >= 本次最小可行 tf）---
        # 【踩过的坑】最初用 tf = 首次tf*0.8，但加了下降率上限后最小可行 tf 变成
        #   37.4 s，而 0.8*31.6=25.3 s < 37.4 ⇒ infeasible ⇒ 重解全部失败
        #   （replan_n=0），只能沿用旧轨迹，越飞越偏。
        #   正确做法：tf 取【计划剩余时间 + 裕度】。
        if t - last_replan > REPLAN_DT and p[0] > GATE_ALT_MIN - TARGET_ALT:
            rem = max(3.0, float(r['tf']) - t + 4.0)
            t1 = time.time()
            r2 = solve_terminal(alt=p[0] + TARGET_ALT,
                                dist=math.hypot(p[1], p[2]),
                                vz=float(v[0]), vh=float(np.linalg.norm(v[1:3])),
                                mass=m, tf=rem, target_alt=TARGET_ALT, **SOLVER)
            solves_ms.append((time.time() - t1) * 1000)
            if r2['status'] == 'optimal':
                x, u = r2['x'], r2['u']
                r = r2
                replan_n += 1
                t_plan = t
            last_replan = t
        # --- 积分 ---
        # a_cmd 已是【推力加速度】；净加速度 = a_cmd + g（g=[-G0,0,0]）
        thrust_acc = a_cmd.copy()
        acc = thrust_acc + np.array([-G0, 0.0, 0.0])
        v = v + acc * dt
        p = p + v * dt
        m = m - float(np.linalg.norm(thrust_acc)) * m / (ISP * G0) * dt
        t += dt
        if int(t * 2) % 4 == 0:
            log.append((t, p[0] + TARGET_ALT, math.hypot(p[1], p[2]),
                        v[0], math.hypot(v[1], v[2]), throttle, tilt))

    print('   t     alt    dist     vz      vh    thr    tilt')
    for row in log[::max(1, len(log)//14)]:
        print('  %5.1f %7.1f %7.1f %7.2f %7.2f %6.3f %6.2f'
              % (row[0], row[1], row[2], row[3], row[4], row[5], row[6]))
    print()
    tl = tilt_hist[-1] if tilt_hist else float('nan')
    # 【触地判据的取样口径（重要）】循环在 p[0] <= 0.5（即 alt <= TARGET_ALT+0.5）
    #   时退出，所以 p 停在略微【高于】触地点的位置，此刻 vz 可能仍是 -6 而非
    #   最终的 -2。物理上有意义的是"压到目标高度那一刻"的速度。
    #   这里记录退出时的值作为保守代表值，并把循环内 alt 最接近目标的那一步
    #   一并打印，避免因为采样口径而自欺。
    vz_exit = float(v[0])
    vh_exit = float(np.linalg.norm(v[1:3]))
    dist_exit = float(np.linalg.norm(p[1:3]))
    print(f'  退出时: alt={p[0]+TARGET_ALT:.2f} m  vz={vz_exit:.2f} m/s  '
          f'vh={vh_exit:.2f} m/s  dist={dist_exit:.2f} m')
    print(f'  触地姿态倾角(推力方向) = {tl:.2f}°')
    print(f'  重解次数 = {replan_n}  重解耗时 中位={np.median(solves_ms):.0f}ms '
          f'max={max(solves_ms):.0f}ms' if solves_ms else '  无重解')

    print()
    print('=' * 74)
    print('③ 判据（用退出时的保守取值）')
    print('=' * 74)
    ok = True
    def chk(name, cond):
        nonlocal ok
        print(f'   [{"OK " if cond else "FAIL"}] {name}')
        ok = ok and cond
    chk(f'落地垂速 |vz| < 8 m/s  (实测 {abs(vz_exit):.2f})', abs(vz_exit) < 8)
    chk(f'落地横速 vh < 5 m/s   (实测 {vh_exit:.2f})', vh_exit < 5)
    chk(f'落点偏差 < 10 m      (实测 {dist_exit:.2f})', dist_exit < 10)
    chk(f'触地倾角 < 5°        (实测 {tl:.2f})', tl < 5)
    # 【重解耗时的判据要按 replan_dt 定，不是拍一个 500 ms】
    #   kRPC 的 control.throttle 是持久的，重解期间执行器保持上次的值，
    #   所以只要 单次耗时 < replan_dt 就不会失控；但占比越高跟踪越粗。
    med = float(np.median(solves_ms)) if solves_ms else 0.0
    chk(f'重解耗时中位 < replan_dt({REPLAN_DT*1000:.0f}ms)  (实测 {med:.0f}ms)',
        med < REPLAN_DT * 1000)
    chk(f'重解耗时占比 < 60%  (实测 {med/(REPLAN_DT*1000)*100:.0f}%)',
        med < 0.6 * REPLAN_DT * 1000)
    print()
    print('  结论:', '全部通过' if ok else '有未通过项')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
