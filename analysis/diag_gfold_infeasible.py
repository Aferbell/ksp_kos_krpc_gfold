# diag_gfold_infeasible.py — 诊断 gfold_modern.py 为什么在锥内状态也 infeasible
#
# 假设：gfold_modern.py 第 40 行 u[:,0] == s[0,0]*[1,0,0] 把【初始推力方向】也钉成
#       竖直。对"需要横向制动"的下降段这是物理错误约束 —— 初始横向速度 vh=50~120
#       m/s 必须靠横向推力消掉，而该约束禁止了它 ⇒ 任何有横速的初值都 infeasible。
# 做法：把该约束改成只约束【末端】推力竖直（本来就是姿态约束的本意），重测。
import numpy as np
import cvxpy as cp

N = 40
tf = 20.0
dt = tf / N
ISP = 315.0
alpha = 1 / 9.80665 / ISP
g = np.array([-9.807, 0.0, 0.0])


def solve(alt=2000.0, dist=500.0, vz=-80.0, vh=50.0, mass=150e3,
          y_gs_deg=20.0, p_cs_deg=20.0, T_max=12.8e6, fix_initial_u=True):
    h = alt - 36.8
    x0 = np.array([h, dist, 0.0, vz, -vh, 0.0])
    y_gs_cot = 1 / np.tan(np.radians(y_gs_deg))
    p_cs_cos = np.cos(np.radians(p_cs_deg))

    x = cp.Variable((6, N))
    u = cp.Variable((3, N))
    z = cp.Variable((1, N))
    s = cp.Variable((1, N))

    con = [x[0:3, 0] == x0[0:3],
           x[3:6, 0] == x0[3:6],
           x[0:3, N - 1] == np.zeros(3),
           x[3:6, N - 1] == np.zeros(3),
           s[0, N - 1] == 0,
           u[:, N - 1] == s[0, N - 1] * np.array([1, 0, 0]),   # 末端推力竖直（姿态约束）
           z[0, 0] == np.log(mass)]
    if fix_initial_u:
        con += [u[:, 0] == s[0, 0] * np.array([1, 0, 0])]        # ← 可疑约束

    for n in range(N - 1):
        con += [x[3:6, n + 1] == x[3:6, n] + (dt * 0.5) * ((u[:, n] + g) + (u[:, n + 1] + g))]
        con += [x[0:3, n + 1] == x[0:3, n] + (dt * 0.5) * (x[3:6, n + 1] + x[3:6, n])]
        con += [cp.norm(x[1:3, n]) - y_gs_cot * x[0, n] <= 0]
        con += [cp.norm(x[3:6, n]) <= 400]
        z0 = np.log(mass - alpha * T_max * (n * dt))
        con += [z[0, n + 1] == z[0, n] - (alpha * dt * 0.5) * (s[0, n] + s[0, n + 1])]
        con += [cp.norm(u[:, n]) <= s[0, n]]
        con += [u[0, n] >= p_cs_cos * s[0, n]]
        if n > 0:
            mu1 = 0.0 * T_max / np.exp(z0)
            mu2 = 1.0 * T_max / np.exp(z0)
            con += [s[0, n] >= mu1 * (1 - (z[0, n] - z0))]
            con += [s[0, n] <= mu2 * (1 - (z[0, n] - z0))]

    obj = cp.Minimize(-z[0, N - 1] * N + cp.sum(cp.norm(x[1:3, :], axis=0)) * 0.1)
    prob = cp.Problem(obj, con)
    prob.solve(solver='CLARABEL')
    return prob.status, (x.value, u.value, np.exp(z.value) if z.value is not None else None)


for fix in (True, False):
    print('=' * 70)
    print('fix_initial_u =', fix, '(True = 复现现状；False = 只约束末端竖直)')
    for (alt, dist, vz, vh, mt, gs, tlt) in [
        (2000, 500, -80, 50, 150, 20, 20),
        (3000, 900, -150, 100, 160, 25, 30),
        (800, 150, -50, 30, 148, 20, 15),
        (1500, 600, -90, 120, 155, 25, 30),
    ]:
        try:
            st, res = solve(alt=alt, dist=dist, vz=vz, vh=vh, mass=mt * 1000,
                            y_gs_deg=gs, p_cs_deg=tlt, fix_initial_u=fix)
        except Exception as e:            # noqa: BLE001
            st, res = 'EXC: %s' % e, None
        line = 'alt=%4d dist=%4d vh=%3d -> %s' % (alt, dist, vh, st)
        if res is not None and res[2] is not None:
            xv, uv, mv = res
            line += '  m_land=%.1ft  u_end_tilt=%.1f°' % (
                mv[0, -1] / 1000,
                np.degrees(np.arctan2(np.linalg.norm(uv[1:3, -1]), abs(uv[0, -1]))))
        print(' ', line)
