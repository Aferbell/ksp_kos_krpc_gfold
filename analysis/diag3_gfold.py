# diag3_gfold.py — 锁定根因：推力指向锥 u_x >= cos(p_cs)*|u| 对下降段过紧
#
# diag2 结论（铁证）：
#   · 放开【推力指向锥】 -> 三个 case 全部 optimal
#   · 放开滑翔锥/Vmax/末端竖直/推力上界/tf 加长 -> 全都仍然 infeasible
#   => 唯一紧约束是推力指向锥。
#
# 物理解释：该约束要求【每一帧】推力方向都落在"偏离竖直 ≤ p_cs"的锥内。
#   但下降段要消横向速度 vh，推力必须朝【斜后方】——偏离竖直角
#   atan(a_h / a_v) 在减速最猛时可能 >> 20~30°。
#   即：这个约束把"横向制动"整个禁掉了。
#
# 关键澄清（这正是"gfold 原生含姿态约束"的正确用法）：
#   p_cs 是【末端】姿态约束，不该全程施加。G-FOLD 原文的 p_cs 是
#   "thrust pointing constraint"，用于保证着陆器不翻倒，通常配
#   较小的值 + 【末端收紧】；全程套 20° 对高空大横速下降是不可行的。
#
# 验证：扫描 p_cs，看可行域边界在哪。
import numpy as np
import cvxpy as cp

N = 60
ISP = 315.0
alpha = 1 / 9.80665 / ISP
g = np.array([-9.807, 0.0, 0.0])


def solve(alt, dist, vz, vh, mass, tf=30.0, p_cs_deg=20.0, y_gs_deg=25.0,
          T_max=12.8e6, pcs_ramp=False, V_max=500.0):
    """pcs_ramp=True: p_cs 从 90° 线性收到 p_cs_deg（末端收紧，G-FOLD 正确用法）"""
    dt = tf / N
    h = alt - 36.8
    x0 = np.array([h, dist, 0.0, vz, -vh, 0.0])
    y_gs_cot = 1 / np.tan(np.radians(y_gs_deg))

    x = cp.Variable((6, N)); u = cp.Variable((3, N))
    z = cp.Variable((1, N)); s = cp.Variable((1, N))

    con = [x[0:3, 0] == x0[0:3], x[3:6, 0] == x0[3:6],
           x[0:3, N-1] == np.zeros(3), x[3:6, N-1] == np.zeros(3),
           s[0, N-1] == 0, z[0, 0] == np.log(mass)]

    for n in range(N - 1):
        con += [x[3:6, n+1] == x[3:6, n] + (dt*0.5)*((u[:, n]+g) + (u[:, n+1]+g))]
        con += [x[0:3, n+1] == x[0:3, n] + (dt*0.5)*(x[3:6, n+1] + x[3:6, n])]
        con += [cp.norm(x[1:3, n]) - y_gs_cot * x[0, n] <= 0]
        con += [cp.norm(x[3:6, n]) <= V_max]
        con += [z[0, n+1] == z[0, n] - (alpha*dt*0.5)*(s[0, n] + s[0, n+1])]
        con += [cp.norm(u[:, n]) <= s[0, n]]
        # 推力指向锥：全程固定 p_cs，或从宽到窄线性收紧
        if pcs_ramp:
            ang = np.radians(90.0 + (p_cs_deg - 90.0) * n / (N - 2))
        else:
            ang = np.radians(p_cs_deg)
        con += [u[0, n] >= np.cos(ang) * s[0, n]]
        z0 = np.log(max(1.0, mass - alpha*T_max*(n*dt)))
        con += [s[0, n] <= 1.0*T_max/np.exp(z0)*(1 - (z[0, n]-z0))]

    con += [u[:, N-1] == s[0, N-1]*np.array([1, 0, 0])]     # 末端推力竖直（真正的姿态约束）
    obj = cp.Minimize(-z[0, N-1]*N + cp.sum(cp.norm(x[1:3, :], axis=0))*0.1)
    prob = cp.Problem(obj, con)
    try:
        prob.solve(solver='CLARABEL')
    except Exception:                            # noqa: BLE001
        return 'EXC', None
    if z.value is None:
        return prob.status, None
    return prob.status, (x.value, u.value, np.exp(z.value))


CASES = [('锥内典型', 2000, 500, -80, 50, 150e3),
         ('锥内-高能', 3000, 900, -150, 100, 160e3),
         ('锥内-低空', 800, 150, -50, 30, 148e3),
         ('锥内-高横速', 1500, 600, -90, 120, 155e3)]

print('### 扫描 p_cs（全程固定推力指向锥半角）')
print('  case          ' + '  '.join('%5d°' % p for p in [15, 20, 30, 45, 60, 75, 85]))
for (name, alt, dist, vz, vh, m) in CASES:
    row = []
    for p in [15, 20, 30, 45, 60, 75, 85]:
        st, _ = solve(alt, dist, vz, vh, m, tf=30, p_cs_deg=p)
        row.append('  OK  ' if st == 'optimal' else '  --  ')
    print('  %-13s' % name + '  '.join(row))

print()
print('### 用【末端收紧】形式（p_cs 90°->目标，G-FOLD 正确用法）')
for (name, alt, dist, vz, vh, m) in CASES:
    for p in [10, 15, 20]:
        st, res = solve(alt, dist, vz, vh, m, tf=30, p_cs_deg=p, pcs_ramp=True)
        extra = ''
        if res is not None:
            xv, uv, mv = res
            ue = uv[:, -1]
            extra = '  落地%.1ft 末端倾角%.2f°' % (
                mv[0, -1]/1000,
                np.degrees(np.arctan2(np.linalg.norm(ue[1:3]), abs(ue[0]))))
        print('  %-13s p_cs末端=%2d° -> %-10s%s' % (name, p, st, extra))
