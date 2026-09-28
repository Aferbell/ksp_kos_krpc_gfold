"""对比 est_time 修正前后：重解后的位置误差与指令质量。

核心问题：重解用外推状态作 x0，若 est_time 过大，轨迹起点会超前于本机，
导致 K_POS 项把 target_a[0] 压成负数 -> conic_clamp 清零水平 -> 姿态归零。
"""
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


def mk():
    o = GL.GfoldLander.__new__(GL.GfoldLander)
    o.v = FakeV()
    o.gfold_n_i = -100.0
    o.last_pkt = 0.02
    o.target_direction = None
    o.tilt_cmd = 0.0
    o.a_cmd_vec = None
    o.tilt_cap = 0.0
    o.tilt_act = 0.0
    return o


def trial(est_over, label):
    """est_over = 外推超前量（秒），模拟 est_time 比实际求解耗时多出的部分。"""
    # 本机状态（日志 t=4.29 附近）
    err0 = np.array([3782.0 - TARGET_ALT, -459.39, 0.0])
    vel = np.array([-276.49, 180.70, 0.0])

    # 重解：用外推状态作 x0
    ahead = err0 + vel * est_over
    r = solve_p3p4_state(np.array([ahead[0], ahead[1], 0.0,
                                   vel[0], vel[1], 0.0]),
                         mass=183900.0, isp=315.0, t_max=8990529.0,
                         N3=160, N4=80, target_alt=TARGET_ALT)
    L = mk()
    err = err0.copy()
    a_cap = FakeV.max_thrust / FakeV.mass
    rows = []
    for step in range(8):
        L.track(r, err, vel, L.gfold_n_i)
        hh = math.hypot(float(L.a_cmd_vec[1]), float(L.a_cmd_vec[2]))
        rows.append((L.gfold_n_i, float(L.a_cmd_vec[0]), hh,
                     L.tilt_cmd,
                     float(np.linalg.norm(L.a_cmd_vec)) / a_cap))
        err = err + vel * 0.1
        err[0] -= abs(vel[0]) * 0.1

    print('\n=== %s（超前 %.2f s）===' % (label, est_over))
    print('  x0 与 本机 差: [%+.1f %+.1f] m'
          % (ahead[0] - err0[0], ahead[1] - err0[1]))
    print('  %5s %8s %8s %8s %7s' % ('step', 'n_i', 'a_up', 'a_h', 'tilt'))
    for i, (ni, au, ah, tl, th) in enumerate(rows):
        print('  %5d %8.3f %8.2f %8.2f %7.2f' % (i, ni, au, ah, tl))
    bad = sum(1 for _, _, ah, _, _ in rows if ah < 0.5)
    print('  a_h<0.5 的步数: %d / %d  %s'
          % (bad, len(rows), '<= 水平被清零!' if bad else '<= 正常'))


print('本机 err0=[%.0f %.0f] vel=[%.0f %.0f]'
      % (3782.0 - TARGET_ALT, -459.39, -276.49, 180.70))

trial(0.50, '修正前：est_time 下限 0.5 s')
trial(0.03, '修正后：est_time = 求解耗时×1.2')
