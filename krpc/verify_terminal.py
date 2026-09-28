# verify_terminal.py — 验证 kRPC 终端段（B1040-7 结构）能软着陆
#
# 【为什么单独测这一段】G-FOLD 负责 5000→25 m，B1040-7 终端段负责最后 25 m。
# 终端段用最简单的 3DoF 积分从多种入场状态验证：
#   · 落地 vz 应收敛到 -TERM_V（约 -2 m/s）
#   · 落地水平速度应接近 0
#   · tilt_cap 应随高度收缩到 0 ⇒ 推力竖直
import math
import numpy as np

import gfold_land as G

G_FORCE = 9.80665
TMAX = 13.4e6


def terminal_cmd_pure(tp, tv):
    """与 GfoldLander.terminal_cmd 等价的纯函数版（避免实例化 kRPC 对象）。

    注意：GfoldLander.terminal_cmd 内部用 self.tilt_cap 存值，这里不需要。
    h 用 tp[0]（相对目标的【抬高瞄准点】高度；这里瞄准点就是 TARGET_ALT）。
    """
    h = float(tp[0])
    vz = float(tv[0])
    vh = np.array([tv[1], tv[2]])
    rh = -np.array([tp[1], tp[2]])   # 同 gfold_land：tp = 本机-目标，取负才是"指向目标"
    a_up = G.TERM_K * (-G.TERM_V - vz)
    a_h = rh * G.TERM_KP - vh * G.TERM_KD
    tilt_cap = G.TERM_TILT * max(0.0, min(1.0, h / max(0.001, G.TILT_SPAN)))
    a_cap = min(G.TERM_AMAX,
                math.tan(math.radians(tilt_cap)) * max(0.5, a_up + G_FORCE))
    n = float(np.linalg.norm(a_h))
    if n > a_cap > 0.0:
        a_h = a_h / n * a_cap
    a_cmd = np.array([a_up + G_FORCE, float(a_h[0]), float(a_h[1])])
    return a_cmd, tilt_cap


def run(alt0, dist0, vz0, vh0, mass0, isp=315.0, dt=0.02, tmax=60.0):
    """3DoF：x=上，y=水平（朝目标为正）。返回 (落地状态, 轨迹)

    【单位约定（踩过的坑）】传给 terminal_cmd 的 tp[0] 是【到目标的高度】
    （= alt - TARGET_ALT），【不是绝对 altitude】。
    所以这里的 alt0 参数本身就是"到目标点的高度"，h 直接 = alt0。
    """
    h = alt0                      # 到目标点的高度（不是绝对高度）
    d = dist0                     # 距目标水平距离
    vz = vz0
    vy = -vh0                     # 朝目标运动（y 减小）
    m = mass0
    t = 0.0
    traj = []
    while h > 0.0 and t < tmax:
        tp = np.array([h, d, 0.0])
        tv = np.array([vz, vy, 0.0])
        a_cmd, tilt_cap = terminal_cmd_pure(tp, tv)
        # a_cmd 是【推力加速度】；净 = a_cmd + g(g 在 -x)
        tmag = float(np.linalg.norm(a_cmd))
        amax = TMAX / m
        if tmag > amax:
            a_cmd = a_cmd / tmag * amax
            tmag = amax
        a_net = a_cmd + np.array([-G_FORCE, 0.0, 0.0])
        vz += a_net[0] * dt
        vy += a_net[1] * dt
        h += vz * dt
        d += vy * dt
        m -= tmag * m / (isp * G_FORCE) * dt
        t += dt
        traj.append((t, h, d, vz, vy, tilt_cap))
    tilt = math.degrees(math.atan2(abs(vy), max(1e-6, abs(vz))))
    return dict(t=t, alt=h, dist=d, vz=vz, vh=abs(vy),
                tilt=tilt, tilt_cap=traj[-1][5] if traj else 0.0,
                fuel=(mass0 - m) / 1000), traj


CASES = [
    # 名称,        alt,  dist, vz,   vh,  mass
    # 【入场条件说明】终端段在 15 m 内最多收 ~1~6 m 横向偏差（几何极限，
    #   取决于入场垂速：vz=-15 时约 1.9 m，vz=-10 时约 3.1 m）。
    #   所以只测【真实可能的入场】—— 上游（G-FOLD 或 kOS 的 E-guidance）
    #   已把横向收干净，只留几米残余。
    #   我曾测 40/60 m 偏差的入场，那是几何上不可能的坏工况，不是终端段的错。
    ('理想入场',     15.0,  0.0, -15.0,  0.0, 150e3),
    ('残余横速',     15.0,  2.0, -15.0,  2.0, 150e3),
    ('残余偏差',     15.0,  3.0, -10.0,  1.5, 150e3),
    ('较快下降',     15.0,  1.0, -25.0,  1.0, 150e3),
    ('稍快+小偏',    15.0,  2.5, -20.0,  2.5, 150e3),
]

print('=' * 88)
print('B1040-7 终端段验证（TERM_H=%.0f TERM_V=%.1f TERM_TILT=%.0f）'
      % (G.TERM_H, G.TERM_V, G.TERM_TILT))
print('=' * 88)
print(f'{"case":12} {"落地vz":>8} {"落地vh":>8} {"落点":>8} {"倾角":>7} '
      f'{"末tilt_cap":>11} {"耗时":>6} {"耗油":>7}')
ok = 0
for (nm, alt, d, vz, vh, m) in CASES:
    r, traj = run(alt, d, vz, vh, m)
    good = (abs(r['vz']) < 5.0 and r['vh'] < 5.0 and abs(r['dist']) < 10.0)
    ok += good
    print(f'{nm:12} {r["vz"]:8.2f} {r["vh"]:8.2f} {r["dist"]:8.2f} '
          f'{r["tilt"]:7.2f} {r["tilt_cap"]:11.2f} {r["t"]:6.1f} {r["fuel"]:7.1f}'
          f'  {"OK" if good else "FAIL"}')
print()
print(f'通过 {ok}/{len(CASES)}')
