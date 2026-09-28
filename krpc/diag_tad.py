"""验证：target_a_ 是否被 conic_clamp 压成纯竖直（这就是 tilt_dir_cmd=0 的原因）。"""
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
    max_thrust = 8990196.0
    mass = 183300.0


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

# 交班后不久的真实状态（日志 t=2.31 附近：alt~4440, vz~-290, vh~213）
alt, dist, vz, vh = 4440.0, 1030.0, -290.0, 213.0
e = np.array([alt - TALT, -dist, 0.0])
v = np.array([vz, vh, 0.0])
res = o.solve(np.array([e[0], e[1], 0.0, v[0], v[1], 0.0]), FakeV.mass)
x, u, tf = res['x'], res['u'], res['tf']
N = u.shape[1]
a_cap = FakeV.max_thrust / FakeV.mass
print('tf=%.2f N=%d a_cap=%.2f' % (tf, N, a_cap))

print('\n=== 逐帧看 target_a 与 target_a_ 的竖直分量 ===')
print('%6s %8s | %-20s %7s | %-20s %7s'
      % ('step', 'n_i', 'target_a (节流用)', 'tilt', 'target_a_(姿态用)', 'tilt'))
n_i = -100.0
for k in range(8):
    # 本机沿轨迹前进
    frac = k / 8.0
    e = np.array([(alt - TALT) * (1 - frac), -dist * (1 - frac), 0.0])
    v = np.array([vz, vh * (1 - frac * 0.3), 0.0])
    o.track(res, e, v, o.gfold_n_i)
    n_i = o.gfold_n_i
    x_i, v_i, u_i = o.sample_index(x, u, n_i, tf, N)
    step = min(1.5 * N / tf, float(np.linalg.norm(v)) / 50.0 * N / tf)
    x_i2, v_i2, u_i2 = o.sample_index(x, u, n_i + step, tf, N)
    # 【用 track() 真正留下的结果】(它内部已算好 target_a / target_direction)
    ca = o.a_cmd_vec
    cb = o.target_direction * float(np.linalg.norm(o.a_cmd_vec))
    # 为对照，仍打印"如果误用 x_i_ 会得到什么"
    ta_wrong = u_i2 + (v_i2 - v) * GL.K_VEL + (x_i2 - e) * GL.K_POS

    def tl(z):
        return math.degrees(math.atan2(float(np.linalg.norm(z[1:3])),
                                       max(1e-9, float(z[0]))))
    print('%6d %8.2f | %-20s %7.2f | %-20s %7.2f'
          % (k, n_i, np.round(ca, 1).tolist(), tl(ca),
             np.round(cb, 1).tolist(), tl(cb)))
    print('        tilt_dir_cmd(日志)=%6.2f   对照:若误用x_i_ =%s (tilt %.1f)'
          % (o.tilt_dir_cmd, np.round(ta_wrong, 1).tolist(), tl(ta_wrong)))
