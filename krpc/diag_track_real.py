"""直接调用 gfold_land.track()（真代码，非复刻）逐步打印，定位分支。

用假的 vessel/conn 桩件，只跑 track()/conic_clamp()/find_nearest_index()。
"""
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, '..', 'gfold', 'solver'))
sys.path.insert(0, os.path.join(_HERE, '..', '..', 'tools', 'stubs'))
sys.path.insert(0, os.path.join(_HERE, '..', '..', 'tools'))
from gfold_p3p4 import solve_p3p4_state      # noqa: E402

import gfold_land as GL                      # noqa: E402

TARGET_ALT = GL.TARGET_ALT


class FakeV:
    max_thrust = 8990529.0
    mass = 183900.0


def build_lander():
    """绕过 __init__（它要连 kRPC），手工塞齐 track() 需要的属性。"""
    obj = GL.GfoldLander.__new__(GL.GfoldLander)
    obj.v = FakeV()
    obj.gfold_n_i = -100.0
    obj.last_pkt = 0.02
    obj.target_direction = None
    obj.tilt_cmd = 0.0
    obj.a_cmd_vec = None
    obj.tilt_cap = 0.0
    obj.tilt_act = 0.0
    return obj


# 真实交班状态
alt, dist, vz, vh = 4979.6, 1309.6, -290.4, 214.0
h = alt - TARGET_ALT
x0 = np.array([h, -dist, 0.0, vz, vh, 0.0])
r = solve_p3p4_state(x0, mass=183900.0, isp=315.0, t_max=8990529.0,
                     N3=160, N4=80, target_alt=TARGET_ALT)
print('求解: %s tf=%.2f engine=%s' % (r['status'], r['tf'], r.get('engine')))
x, u, tf = r['x'], r['u'], r['tf']
N = u.shape[1]

L = build_lander()

print('\n=== 用真 track() 走 12 帧（本机按 hold 减速，模拟真实轨迹外推）===')
# 本机状态：沿速度反向减速（hold 段实际行为）
err = np.array([h, -dist, 0.0])
vel = np.array([vz, vh, 0.0])
a_cap = FakeV.max_thrust / FakeV.mass

for step in range(12):
    tp, tv = err.copy(), vel.copy()
    ta, idx = L.track(r, tp, tv, L.gfold_n_i)
    hh = math.hypot(float(ta[1]), float(ta[2]))
    print('  step%2d n_i=%8.3f  a=[%7.2f %7.2f]  tilt=%6.2f  thr=%.3f  '
          'err=[%7.1f %7.1f]  |v|=%.0f'
          % (step, L.gfold_n_i, ta[0], hh,
             math.degrees(math.atan2(hh, max(1e-9, float(ta[0])))),
             float(np.linalg.norm(ta)) / a_cap, err[0], err[1],
             float(np.linalg.norm(vel))))
    # 本机前进一小步（模拟 hold：沿速度反向减速）
    dt = 0.1
    a_hold = (-vel / max(1e-9, float(np.linalg.norm(vel)))) * 0.4 * a_cap
    a_hold[0] -= GL.G0
    vel = vel + a_hold * dt
    err = err + vel * dt
