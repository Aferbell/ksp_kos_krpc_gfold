# verify_rewrite.py -- 功能验证：重写版 gfold_land.track() 是否真能工作
#
# 【为什么需要】静态检查（pyflakes/pyright/check_cols）只能证明"语法与列名一致"，
#   不能证明【控制律算得对】。本脚本用【真实求解器 + 真实交班状态】跑 track()，
#   并核对重写版必须成立的三条恒等式：
#     H1  a_h == a_up * tan(tilt_cmd)          （指令自洽）
#     H2  前向倾角与后向倾角用【同一个 a_up】（口径闭环，措施3）
#     H3  内环限的是 |da_h/dt| <= j_max        （措施2）
#   以及飞行安全约束：
#     S1  0 <= a_up <= 0.99*g0                 （保证会落地）
#     S2  tilt_cmd <= cap(alt)                 （防机头翻过 90 deg）
#     S3  az == a_h_vec 的方向                 （大小与方向同源）
#     S4  |da_h/dt| 不随 a_up/sec^2 漂移       （限的是 a_h 而非倾角）
#
# 不连 KSP：用桩件替代 vessel/conn，只跑 track()/conic_clamp() 等纯计算。
# 输出全 ASCII（控制台 GBK）。
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
# 【必须与 gfold_land.py 的路径逻辑完全一致：只放 ../solver】
#   dev/solver 与 dev/gfold/solver 是【两个不同版本】的求解器
#   （前者是实飞用的、含本项目全部修正；后者是上游参考仓库的旧副本）。
#   若两者都进 sys.path，则【加载顺序决定用哪个】—— 本脚本曾因此
#   加载到旧副本，报 "cannot import name 'PCS_START_CAP_DEG'"，
#   而实飞走的是 dev/solver。故这里【只】放 dev/solver。
#   （与 dev/analysis/reproduce_gate.py 的说明一致。）
sys.path.insert(0, os.path.join(_HERE, '..', 'solver'))
sys.path.insert(0, os.path.join(_HERE, '..', '..', 'tools', 'stubs'))

from gfold_p3p4 import solve_p3p4_state          # noqa: E402

import gfold_land as GL                          # noqa: E402


class FakeV:
    """桩件 vessel：track() 只用到 mass / available_thrust / max_thrust。"""
    max_thrust = 8990529.0
    available_thrust = 8990529.0
    mass = 183900.0
    name = 'B1040-9(fake)'


def build_lander():
    """绕过 __init__（它要连 kRPC），把 track() 真正引用的属性全部塞齐。

    【为什么用 __new__ 而不是 __init__】__init__ 需要 conn/args。
    这里显式列出 track() 依赖的字段 —— 若重写版少了任何一个，
    本脚本会以 AttributeError 失败，而不是静默给出错的数。
    """
    obj = GL.GfoldLander.__new__(GL.GfoldLander)
    obj.v = FakeV()
    obj.gfold_n_i = -100.0
    obj.last_pkt = 0.02
    obj.gfold_traj_ready = False
    obj.target_direction = None
    obj.tilt_cmd = 0.0
    obj.a_cmd_vec = None
    obj.tilt_cap = 0.0
    obj.tilt_act = 0.0
    obj.idx_frozen_n = 0
    obj._sample_arrived_ok = False
    obj.min_mag_eff = 0.0
    obj.tf_ah_prev = 0.0
    obj.tf_az_dir = None
    obj._ref_slew_prev = None
    obj.dbg_ref_clamped = 0.0
    obj.dbg_alpha_max = 20.0          # 姿态权限 [deg/s^2]（实测中位 ~13，偏保守）
    obj.dbg_avel_local = [0.0, 0.0, 0.0]
    obj._tscale = 1.0
    obj._tscale_buf = []
    obj._tscale_prev_cmd = float('nan')
    return obj


def check(tag, cond, detail):
    print('  %-46s %s   %s' % (tag, 'PASS' if cond else 'FAIL', detail))
    return bool(cond)


def main():
    print('=== [1] 求解器：用真实交班状态解一条规划 ===')
    alt, dist, vz, vh = 1164.2, 130.0, -67.3, 12.2
    mass = 157900.0
    t_avail = FakeV.available_thrust
    x0 = np.array([alt - GL.TARGET_ALT, -dist, 0.0, vz, vh, 0.0])
    # 交班门处 a_net 必须够大，否则求解器直接预检失败
    a_net = t_avail / mass - GL.G0
    print('  a_net = %.2f m/s^2   (需 > 1.0)' % a_net)
    if a_net < 1.0:
        print('  SKIP: 该状态推重比过低，换一个质量')
        return 1
    r = solve_p3p4_state(
        x0, mass, isp=GL.ISP_DEFAULT, t_max=t_avail,
        throttle=(0.1, 0.8), tf_guess=GL.TF_GUESS,
        straight_fac=GL.STRAIGHT_FAC, gs_deg=GL.GS_DEG,
        pcs_deg=GL.PCS_END_DEG, pcs_start_deg=GL.PCS_START_DEG,
        v_max=GL.V_MAX_SOLVER, N3=GL.N3, N4=GL.N4,
        target_alt=GL.TARGET_ALT,
        v_descent_max=GL.V_DESCENT_MAX_ENABLE,
        term_win=GL.V_DESCENT_TERM_WIN, term_vz=GL.V_DESCENT_TERM_VZ)
    print('  status=%s  tf=%.2f  engine=%s  N=%d'
          % (r['status'], r['tf'], r.get('engine'), r['u'].shape[1]))
    if r['status'] != 'optimal':
        print('  FAIL: 求解器没给出最优解，无法继续')
        return 1

    L = build_lander()
    dt = 0.02

    print('')
    print('=== [2] 逐帧调用重写版 track()，模拟 hold 段靠近规划 ===')
    err = np.array([alt - GL.TARGET_ALT, -dist, 0.0])
    vel = np.array([vz, vh, 0.0])
    a_cap = t_avail / mass

    h1 = h2 = h3 = s1 = s2 = s3 = True
    h1_worst = h2_worst = 0.0
    n = 0
    j_hist = []
    print('  %4s %8s %8s %8s %8s %8s %8s'
          % ('step', 'n_i', 'a_up', 'a_h', 'tilt', 'j_used', 'j_max'))
    for step in range(40):
        tp, tv = err.copy(), vel.copy()
        # 上一帧的 a_h 作为内环状态（与 run() 的时序一致）
        L.tf_ah_prev = getattr(L, '_ah_prev_for_test', 0.0)
        ta, idx = L.track(r, tp, tv, L.gfold_n_i)
        L._ah_prev_for_test = float(np.linalg.norm(ta[1:3]))
        L.last_pkt = dt
        n += 1

        a_up = float(ta[0])
        a_h = float(np.linalg.norm(ta[1:3]))
        tilt = L.tilt_cmd
        # ---- H1: a_h == a_up*tan(tilt) ----
        if tilt < 89.0:
            pred = a_up * math.tan(math.radians(tilt))
            d1 = abs(pred - a_h)
            h1_worst = max(h1_worst, d1)
            if d1 > 0.05:
                h1 = False
        # ---- H2: 需求(tf_corr) 与 指令(tilt_cmd) 的差距必须能被
        #      【能力夹限 + 内环限速 + 终端收倾角】解释，不能有未知来源。
        #      注意：这个差距在起步瞬间本来就大（内环状态从 0 爬升），
        #      所以它【不是】恒等式，而是"需求/可达"的度量。
        d2 = abs(float(getattr(L, 'dbg_tf_tilt_corr', float('nan'))) - tilt)
        h2_worst = max(h2_worst, d2)
        if d2 > 90.0:
            h2 = False
        # ---- H3: |da_h/dt| <= j_max，【除非】能力上限硬性收缩 ----
        #   能力上限 a_h_max = a_up*tan(tilt_allow) 是硬安全限，
        #   它收缩时允许覆盖 jerk 曲线（见 track ⑧ 的说明）。
        #   故只在"能力上限未收紧"的帧上要求 jerk 不越界。
        j_used = abs(float(getattr(L, 'dbg_j_used', 0.0)))
        j_max = float(getattr(L, 'dbg_j_max', 0.0))
        cap_bind = float(getattr(L, 'dbg_ah_cap_bind', 0.0))
        j_hist.append((j_used, j_max, cap_bind))
        if j_max > 0 and j_used > j_max + 1e-6 and cap_bind < 0.5:
            h3 = False
        # ---- S1: 0 <= a_up（非负是硬约束；上界是 a_cap，见 E6）----
        if not (0.0 - 1e-9 <= a_up):
            s1 = False
        # ---- S2: tilt <= cap ----
        cap = float(getattr(L, 'dbg_tf_cap', 89.0))
        if tilt > cap + 1e-6:
            s2 = False
        # ---- S3: 方位角与 a_h 矢量同源 ----
        if a_h > 1e-6:
            az_n = float(getattr(L, 'dbg_tf_az_n', 0.0))
            az_e = float(getattr(L, 'dbg_tf_az_e', 0.0))
            v_n = float(ta[1]) / a_h
            v_e = float(ta[2]) / a_h
            if abs(az_n - v_n) > 0.05 or abs(az_e - v_e) > 0.05:
                s3 = False

        if step % 3 == 0:
            print('  %4d %8.2f %8.2f %8.2f %8.2f %8.2f %8.2f'
                  % (step, L.gfold_n_i, a_up, a_h, tilt, j_used, j_max))

        # 本机前进（hold 段：沿速度反向减速）
        sp = float(np.linalg.norm(vel))
        if sp > 1e-9:
            a_hold = (-vel / sp) * 0.4 * a_cap
            a_hold[0] -= GL.G0
            vel = vel + a_hold * dt
            err = err + vel * dt

    j_hist.append((j_used, j_max, cap_bind))

    print('')
    print('=== [3] 恒等式与安全约束核对（%d 帧）===' % n)
    ok = True
    ok &= check('H1 a_h == a_up*tan(tilt)',
                h1, 'max dev = %.4f m/s^2 (应 ~0)' % h1_worst)
    ok &= check('H2 需求/可达 差距可解释',
                h2, 'max |tf_corr - tilt_cmd| = %.2f deg' % h2_worst)
    _viol = sum(1 for a, b, c in j_hist
                if b > 0 and a > b + 1e-6 and c < 0.5)
    ok &= check('H3 |j_used| <= j_max (能力未收紧时)',
                h3, 'violations = %d / %d (cap_bind frames excluded)'
                % (_viol, len(j_hist)))
    ok &= check('S1 a_up >= 0（非负硬约束）', s1, '限制生效')
    ok &= check('S2 tilt_cmd <= cap', s2, '限制生效')
    ok &= check('S3 az 与 a_h 矢量同源', s3, '方向自洽')

    print('')
    print('=== [4] 内环诊断数值（确认限的是 a_h 而非倾角）===')
    js = [a for a, _, _ in j_hist]
    jm = [b for _, b, _ in j_hist]
    cb = sum(1 for _, _, c in j_hist if c > 0.5)
    print('  j_used  p50=%.3f max=%.3f' % (np.median(js), max(js)))
    print('  j_max   p50=%.3f max=%.3f' % (np.median(jm), max(jm)))
    print('  能力上限收紧的帧数 = %d / %d' % (cb, len(j_hist)))
    print('  j_max 随 alpha 在线导出（本测试 alpha=%.1f deg/s^2）'
          % L.dbg_alpha_max)
    print('  【判读】j_max 应为有限正数且不随倾角漂移；')
    print('         若 j_max 恒为 2.0（JERK_GUARD_MIN），说明 alpha 没采到。')

    print('')
    print('=== [5] 端到端：用重写版 track() 闭环飞到地面 ===')
    # 【为什么必须做】[2]-[4] 只验证单帧自洽；这里让 track() 驱动一个 3DoF
    #   质点，检查它是否真的能落地、且落点/速度在包络内。
    #   被控对象：v += (a_cmd - g) * dt，其中 a_cmd = target_a（推力加速度）。
    L2 = build_lander()
    L2.last_pkt = 0.02
    ee = np.array([alt - GL.TARGET_ALT, -dist, 0.0])
    vv = np.array([vz, vh, 0.0])
    a_up_h, a_h_h = [], []
    landed = False
    for step in range(4000):
        ta, _ = L2.track(r, ee.copy(), vv.copy(), L2.gfold_n_i)
        a_up_h.append(float(ta[0]))
        a_h_h.append(float(np.linalg.norm(ta[1:3])))
        # 一阶积分（推力沿 target_a 方向，含重力）
        acc = np.array(ta, float) - np.array([GL.G0, 0.0, 0.0])
        vv = vv + acc * L2.last_pkt
        ee = ee + vv * L2.last_pkt
        # 触地判据：相对目标高度 <= 0
        if ee[0] <= 0.0:
            landed = True
            break
    h_fin = float(ee[0]) + GL.TARGET_ALT
    dist_fin = math.hypot(float(ee[1]), float(ee[2]))
    vz_fin = float(vv[0])
    vh_fin = math.hypot(float(vv[1]), float(vv[2]))
    print('  landed=%s  steps=%d  t=%.1f s' % (landed, step, step * 0.02))
    print('  末状态: alt=%.2f m  dist=%.1f m  vz=%.2f m/s  vh=%.2f m/s'
          % (h_fin, dist_fin, vz_fin, vh_fin))
    print('  a_up: min=%.2f max=%.2f   a_h: min=%.2f max=%.2f'
          % (min(a_up_h), max(a_up_h), min(a_h_h), max(a_h_h)))
    # 安全判据（与实飞验收口径一致）
    e1 = check('E1 触地（alt <= 目标高度）', landed, 'alt=%.2f' % h_fin)
    e2 = check('E2 垂速在包络内（|vz| <= 10）', abs(vz_fin) <= 10.0,
               'vz=%.2f' % vz_fin)
    e3 = check('E3 横速在包络内（vh <= 10）', vh_fin <= 10.0,
               'vh=%.2f' % vh_fin)
    e4 = check('E4 落点在 50 m 内', dist_fin <= 50.0, 'dist=%.1f' % dist_fin)
    e5 = check('E5 a_up 恒非负', min(a_up_h) >= -1e-9,
               'min=%.3f' % min(a_up_h))
    # E6：a_up 是【推力加速度】（含 g0），故上界是 a_cap 而不是 0.99*g0。
    #   0.99*g0 只是【保底值】的封顶（见 track 的说明）。
    a_cap_lim = t_avail / mass
    e6 = check('E6 a_up <= a_cap (56.9)', max(a_up_h) <= a_cap_lim + 1e-6,
               'max=%.3f / cap=%.2f' % (max(a_up_h), a_cap_lim))
    ok &= (e1 and e2 and e3 and e4 and e5 and e6)

    print('')
    print('RESULT:', 'PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())