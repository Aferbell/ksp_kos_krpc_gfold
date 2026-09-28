# diag2_gfold.py — 逐步定位 G-FOLD 在锥内状态 infeasible 的真正原因
#
# 上一轮（diag_gfold_infeasible.py）否定了"初始推力竖直"假设：
#   放开初始推力方向后，alt=3000/vh=100 与 alt=1500/vh=120 仍然 infeasible。
# 这一轮用"逐条约束开关 + 可行性松弛"定位真正的紧约束。
#
# 手段：把每条可能过紧的约束替换成一个【松弛变量】并最小化违反量。
#   若最小违反量 > 0 且能指出是哪条，就锁定根因。
import numpy as np
import cvxpy as cp

N = 40
tf = 20.0
dt = tf / N
ISP = 315.0
alpha = 1 / 9.80665 / ISP
g = np.array([-9.807, 0.0, 0.0])


def solve(alt, dist, vz, vh, mass=150e3, y_gs_deg=20.0, p_cs_deg=20.0,
          T_max=12.8e6, V_max=400.0, throt_lo=0.0,
          enforce=(True, True, True, True, True), tff=None):
    """
    enforce = (glide, thrust_cone, vmax, tmin, uend_vert)
      分别控制：滑翔角锥 / 推力指向锥 / 速度上限 / 推力下限 / 末端推力竖直
    """
    tff = tff or tf
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
           z[0, 0] == np.log(mass)]
    if enforce[4]:
        con += [u[:, N - 1] == s[0, N - 1] * np.array([1, 0, 0])]

    for n in range(N - 1):
        con += [x[3:6, n + 1] == x[3:6, n] + (dt * 0.5) * ((u[:, n] + g) + (u[:, n + 1] + g))]
        con += [x[0:3, n + 1] == x[0:3, n] + (dt * 0.5) * (x[3:6, n + 1] + x[3:6, n])]
        con += [z[0, n + 1] == z[0, n] - (alpha * dt * 0.5) * (s[0, n] + s[0, n + 1])]
        con += [cp.norm(u[:, n]) <= s[0, n]]
        if enforce[0]:
            con += [cp.norm(x[1:3, n]) - y_gs_cot * x[0, n] <= 0]
        if enforce[1]:
            con += [u[0, n] >= p_cs_cos * s[0, n]]
        if enforce[2]:
            con += [cp.norm(x[3:6, n]) <= V_max]
        # 推力上下限（对数凸化）；throt_lo=0 时退化为上界一条
        z0 = np.log(max(1.0, mass - alpha * T_max * (n * dt)))
        if throt_lo > 0:
            mu1 = throt_lo * T_max / np.exp(z0)
            con += [s[0, n] >= mu1 * (1 - (z[0, n] - z0))]
        mu2 = 1.0 * T_max / np.exp(z0)
        con += [s[0, n] <= mu2 * (1 - (z[0, n] - z0))]

    obj = cp.Minimize(-z[0, N - 1] * N + cp.sum(cp.norm(x[1:3, :], axis=0)) * 0.1)
    prob = cp.Problem(obj, con)
    try:
        prob.solve(solver='CLARABEL')
    except Exception as e:                      # noqa: BLE001
        return 'EXC:%s' % e, None
    if z.value is None:
        return prob.status, None
    return prob.status, (x.value, u.value, np.exp(z.value))


CASES = [(2000, 500, -80, 50), (3000, 900, -150, 100), (1500, 600, -90, 120)]

print('### A. 逐条放开约束（找出哪条在卡）')
print('  case                    glide cone  vmax uend   -> status')
for (alt, dist, vz, vh) in CASES:
    for tag, enf in [
        ('全约束',            (True,  True,  True,  False, True)),
        ('放开滑翔锥',        (False, True,  True,  False, True)),
        ('放开推力锥',        (True,  False, True,  False, True)),
        ('放开Vmax',          (True,  True,  False, False, True)),
        ('放开末端推力竖直',  (True,  True,  True,  False, False)),
        ('全放开',            (False, False, False, False, False)),
    ]:
        st, _ = solve(alt, dist, vz, vh, enforce=enf)
        print('  alt=%4d vh=%3d %-18s -> %s' % (alt, vh, tag, st))

print()
print('### B. 时间充裕度：tf 扫（锥内状态最需要的是时间）')
for (alt, dist, vz, vh) in CASES:
    row = []
    for t in [15, 20, 25, 30, 40, 60]:
        st, _ = solve(alt, dist, vz, vh, tff=t)
        row.append('tf=%d:%s' % (t, 'OK' if st == 'optimal' else st[:3]))
    print('  alt=%4d vh=%3d  %s' % (alt, vh, '  '.join(row)))

print()
print('### C. 推力上界：T_max 扫（是否推力不够）')
for (alt, dist, vz, vh) in CASES:
    row = []
    for f in [1.0, 1.5, 2.0, 3.0]:
        st, _ = solve(alt, dist, vz, vh, T_max=12.8e6 * f, tff=30)
        row.append('%.1fx:%s' % (f, 'OK' if st == 'optimal' else st[:3]))
    print('  alt=%4d vh=%3d  %s' % (alt, vh, '  '.join(row)))
