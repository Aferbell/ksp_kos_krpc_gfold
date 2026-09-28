"""验证：预热不再做错误的速度外推 + 过期一致性检查生效。"""
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, '..', 'gfold', 'solver'))
sys.path.insert(0, os.path.join(_HERE, '..', '..', 'tools', 'stubs'))
import gfold_land as GL                      # noqa: E402

TALT = GL.TARGET_ALT


class FakeV:
    max_thrust = 8990489.0
    mass = 184100.0


def mk():
    o = GL.GfoldLander.__new__(GL.GfoldLander)
    o.v = FakeV()
    o.last_pkt = 0.02
    o.gfold_n_i = -100.0
    o.target_direction = None
    o.tilt_cmd = 0.0
    o.tilt_dir_cmd = 0.0
    o.a_cmd_vec = None
    o.tilt_cap = 0.0
    o.tilt_act = 0.0
    return o


print('=== 1. 修正后的预热：用当前状态解，不外推速度 ===')
print('  阈值: alt %g m / vz %g m/s'
      % (GL.PREWARM_MAX_ALT_GAP, GL.PREWARM_MAX_V_GAP))

# 预热时刻状态（alt=6237, vz 来自日志量级）
pw_alt, pw_vz, pw_vh = 6237.0, -330.0, 260.0
pw_dist = (pw_alt - TALT) * 0.27
o = mk()
res = o.solve(np.array([pw_alt - TALT, -pw_dist, 0.0, pw_vz, pw_vh, 0.0]),
              FakeV.mass)
x = res['x']
print('\n  预热解 tf=%.2f  起点 alt=%.0f vz=%.1f'
      % (res['tf'], x[0, 0] + TALT, x[3, 0]))

# 到门时实际状态
hand_alt, hand_vz = 4966.25, -285.85
gap_alt = abs(float(x[0, 0]) - (hand_alt - TALT))
gap_v = abs(float(x[3, 0]) - hand_vz)
print('  到门时实际 alt=%.0f vz=%.1f' % (hand_alt, hand_vz))
print('  起点差: %.0f m / %.1f m/s' % (gap_alt, gap_v))
stale = gap_alt > GL.PREWARM_MAX_ALT_GAP or gap_v > GL.PREWARM_MAX_V_GAP
print('  判定: %s' % ('过期 -> 弃用并重解 (正确!)' if stale
                      else '可用'))

print('\n=== 2. 重解后（用实际状态）的指令应正常 ===')
o2 = mk()
# 【注意】dist 必须用【实际值 1344.16】，不是预热时的 1674
HAND_DIST = 1344.16
res2 = o2.solve(np.array([hand_alt - TALT, -HAND_DIST, 0.0, hand_vz, 225.66, 0.0]),
                FakeV.mass)
a_cap = FakeV.max_thrust / FakeV.mass
for k, (a_, d_, v_, h_) in enumerate([
        (4966.25, 1344.16, -285.85, 225.66),
        (4880.31, 1277.23, -286.74, 224.18),
        (4765.41, 1188.71, -287.76, 222.09)]):
    e = np.array([a_ - TALT, -d_, 0.0])
    v = np.array([v_, h_, 0.0])
    o2.track(res2, e, v, o2.gfold_n_i)
    ca = o2.a_cmd_vec
    hh = math.hypot(float(ca[1]), float(ca[2]))
    print('  step%d a_cmd=%s tilt=%5.2f thr=%.3f'
          % (k, np.round(ca, 2).tolist(),
             math.degrees(math.atan2(hh, max(1e-9, float(ca[0])))),
             float(np.linalg.norm(ca)) / a_cap))
