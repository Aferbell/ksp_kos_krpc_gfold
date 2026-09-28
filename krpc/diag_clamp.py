"""逐步打印 conic_clamp 内部，看 a_h 为何归零。"""
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, '..', 'gfold', 'solver'))
sys.path.insert(0, os.path.join(_HERE, '..', '..', 'tools', 'stubs'))
from gfold_p3p4 import solve_p3p4_state      # noqa: E402
import gfold_land as GL                      # noqa: E402

TARGET_ALT = GL.TARGET_ALT


class FakeV:
    max_thrust = 8990529.0
    mass = 183900.0


o = GL.GfoldLander.__new__(GL.GfoldLander)
o.v = FakeV()

err = np.array([3413.0, -351.0, 0.0])
vel = np.array([-276.49, 180.70, 0.0])
ERR_AHEAD = np.array([3275.0, -261.0, 0.0])

r = solve_p3p4_state(np.array([ERR_AHEAD[0], ERR_AHEAD[1], 0.0,
                               vel[0], vel[1], 0.0]),
                     mass=183900.0, isp=315.0, t_max=8990529.0,
                     N3=160, N4=80, target_alt=TARGET_ALT)
x, u, tf = r['x'], r['u'], r['tf']
N = u.shape[1]

xi, vi, ui = o.sample_index(x, u, 0.0, tf, N)
print('=== n_i = 0 处的采样 ===')
print('  x_i     =', np.round(xi, 1).tolist())
print('  err     =', np.round(err, 1).tolist())
print('  x_i-err =', np.round(xi - err, 1).tolist())
print('  v_i     =', np.round(vi, 1).tolist())
print('  vel     =', np.round(vel, 1).tolist())
print('  v_i-vel =', np.round(vi - vel, 1).tolist())
print('  u_i     =', np.round(ui, 2).tolist())

ta = ui + (vi - vel) * GL.K_VEL + (xi - err) * GL.K_POS
print('\n=== target_a = u_i + (v_i-vel)*K_VEL + (x_i-err)*K_POS ===')
print('  K_POS=%.2f K_VEL=%.2f' % (GL.K_POS, GL.K_VEL))
print('  u_i 贡献        =', np.round(ui, 2).tolist())
print('  K_VEL 贡献      =', np.round((vi - vel) * GL.K_VEL, 2).tolist())
print('  K_POS 贡献      =', np.round((xi - err) * GL.K_POS, 2).tolist())
print('  target_a        =', np.round(ta, 2).tolist())

tilt_traj = math.degrees(math.atan2(float(np.linalg.norm(ui[1:3])),
                                    max(1e-9, float(ui[0]))))
tilt_now = min(max(tilt_traj + 8.0, GL.CONIC_TILT_DEG), 89.0)
a_cap = FakeV.max_thrust / FakeV.mass
print('\n  tilt_traj=%.1f  tilt_now=%.1f' % (tilt_traj, tilt_now))
print('  min_mag=%.3f  max_mag=%.3f' % (0.05 * a_cap, a_cap))

tc = o.conic_clamp(ta, 0.05 * a_cap, a_cap, tilt_now)
hh = float(np.linalg.norm(tc[1:3]))
print('  conic_clamp ->', np.round(tc, 2).tolist(),
      ' a_h=%.2f  tilt=%.2f' % (hh, math.degrees(math.atan2(hh, max(1e-9, tc[0])))))
