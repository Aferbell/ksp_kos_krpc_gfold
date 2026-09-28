"""复现实飞真实时序：预热用【预测状态】解，再用【实际状态】track。

这是实飞的真实流程（我之前的诊断脚本都漏了这一步，
所以总得到"正常"的结果，与实际日志不符）。
"""
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
GATE_ALT, GATE_VMAX = GL.GATE_ALT, GL.GATE_VMAX
PAF, PVF = GL.PREWARM_ALT_FACTOR, GL.PREWARM_V_FACTOR


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


# ---- 实飞预热时的状态（日志 prewarm start alt=6237 |v|=497.8）----
pw_alt, pw_vmag = 6237.0, 497.8
# 当时的分轴（按交班行比例推：vz/vh ≈ 285/226）
ratio = pw_vmag / math.hypot(285.85, 225.66)
pw_vz, pw_vh = -285.85 * ratio, 225.66 * ratio
pw_dist = (pw_alt - TALT) * 0.27
print('=== 预热状态 ===')
print('  alt=%.1f vz=%.1f vh=%.1f dist=%.1f' % (pw_alt, pw_vz, pw_vh, pw_dist))

# _start_prewarm 的预测：est_t 后到门
est_t = max(0.0, (pw_alt - GATE_ALT) / max(1.0, abs(pw_vz)))
pred_h = (pw_alt - TALT) - abs(pw_vz) * est_t
print('  预测 %.2f s 后到门: h=%.1f (alt=%.1f)'
      % (est_t, pred_h, pred_h + TALT))

o = mk()
print('\n=== 预热解算（用预测状态）===')
res = o.solve(np.array([pred_h, -pw_dist, 0.0, pw_vz, pw_vh, 0.0]),
              FakeV.mass)
print('  status=%s tf=%.2f  (日志 tf=20.75)' % (res['status'], res['tf']))
x, u, tf = res['x'], res['u'], res['tf']
N = u.shape[1]

# ---- 实际首帧状态（日志 t=0.04）----
print('\n=== 用【实际状态】track（这是实飞的真实情形）===')
a_cap = FakeV.max_thrust / FakeV.mass
print('  a_cap=%.2f min_mag=%.2f' % (a_cap, 0.05 * a_cap))
print('  %6s %8s | %-22s %8s %8s'
      % ('step', 'n_i', 'a_cmd', 'tilt', 'thr'))
seq = [(4966.25, 1344.16, -285.85, 225.66),
       (4880.31, 1277.23, -286.74, 224.18),
       (4765.41, 1188.71, -287.76, 222.09),
       (4630.59, 1092.27, -288.85, 219.80),
       (4456.01, 958.08, -290.44, 216.37)]
o2 = mk()
for k, (a_, d_, v_, h_) in enumerate(seq):
    e = np.array([a_ - TALT, -d_, 0.0])
    v = np.array([v_, h_, 0.0])
    o2.track(res, e, v, o2.gfold_n_i)
    ca = o2.a_cmd_vec
    hh = math.hypot(float(ca[1]), float(ca[2]))
    print('  %6d %8.3f | %-22s %8.2f %8.3f'
          % (k, o2.gfold_n_i, np.round(ca, 2).tolist(),
             math.degrees(math.atan2(hh, max(1e-9, float(ca[0])))),
             float(np.linalg.norm(ca)) / a_cap))
