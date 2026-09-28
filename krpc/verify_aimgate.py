"""验证"瞄准点交班"的判据与可行性。"""
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, '..', 'gfold', 'solver'))
sys.path.insert(0, os.path.join(_HERE, '..', '..', 'tools', 'stubs'))
import gfold_land as GL                      # noqa: E402

TA = GL.TARGET_ALT
AIM = GL.AIM_ALT

print('=== 瞄准点交班判据 ===')
print('  aim_alt            = %.0f m  (目标点上方)' % AIM)
print('  瞄准点绝对高度     = %.1f + %.0f = %.1f m'
      % (TA, AIM, TA + AIM))
print('  GATE_AIM_H         = %.0f m  ("附近"的容差)' % GL.GATE_AIM_H)
print('  GATE_ALT (kRPC)    = %.1f m' % GL.GATE_ALT)
print('  kOS gfold_h(余量)  = 150 m  -> 交班 alt = %.1f m' % (TA + AIM + 150))
print('  GATE_VMAX          = %.0f m/s' % GL.GATE_VMAX)
print('  一致:', abs(GL.GATE_ALT - (TA + AIM + 150)) < 1e-6)

print('\n=== 旧判据的问题 ===')
print('  旧 GATE_ALT = 5000 -> 交班时 h_aim = 5000-36.8-3000 = %.1f m'
      % (5000 - TA - AIM))
print('  -> 一级离瞄准点还有 2000 m 就被交出，段1 来不及在瞄准点收速')

print('\n=== 新判据的可行性（用实飞交班点数据外推）===')
# log land_log_47883027.csv 交班点
alt0, vz0, vh0, dist0 = 5044.2, -288.07, 220.87, 1335.5
h_aim0 = alt0 - TA - AIM
print('  实飞交班点: alt=%.1f h_aim=%.1f vz=%.1f vh=%.1f dist=%.1f'
      % (alt0, h_aim0, vz0, vh0, dist0))
T = 2 * h_aim0 / max(math.sqrt(h_aim0), abs(vz0))
vmag = math.hypot(vz0, vh0)
a_net = 28.9
print('  段1 时标 T = 2·h_aim/max(sqrth_aim,|vz|) = %.1f s' % T)
print('  |v| = %.1f m/s, 需减速度 = %.1f m/s^2, 可用 a_net = %.1f'
      % (vmag, vmag / T, a_net))
print('  -> %s' % ('能收完（段1 可到瞄准点）' if vmag / T < a_net
                   else '收不完'))

print('\n=== 新交班点的预期状态（段1 收速后）===')
print('  alt  ≈ %.1f m' % (TA + AIM + 150))
print('  vz   ≈ 0（段1 律在瞄准点归零）')
print('  vh   ≈ 0')
print('  dist ≈ 0')
print('  |v|  < %.0f（GATE_VMAX）' % GL.GATE_VMAX)

print('\n=== 预热配套 ===')
print('  PREWARM_ALT_FACTOR = %.2f -> 预热门 %.1f m'
      % (GL.PREWARM_ALT_FACTOR, GL.GATE_ALT * GL.PREWARM_ALT_FACTOR))
print('  到门高度差 = %.1f m'
      % (GL.GATE_ALT * GL.PREWARM_ALT_FACTOR - GL.GATE_ALT))
print('  一致性阈值 = %.0f m  -> %s'
      % (GL.PREWARM_MAX_ALT_GAP,
         'OK（差值 < 阈值）'
         if (GL.GATE_ALT * (GL.PREWARM_ALT_FACTOR - 1)
             < GL.PREWARM_MAX_ALT_GAP) else '需再放宽'))

print('\n=== 求解可行性（瞄准点附近的状态）===')
class FakeV:
    max_thrust = 8990541.0
    mass = 183800.0


o = GL.GfoldLander.__new__(GL.GfoldLander)
o.v = FakeV()
for vmag in (20.0, 60.0, 110.0):
    h = 150.0
    vz = -vmag * 0.4
    vh = vmag * 0.9
    dist = 150.0 * 0.5
    r = o.solve(np.array([h, -dist, 0.0, vz, vh, 0.0]), FakeV.mass)
    st = r['status']
    extra = ('tf=%.1f 落地=%.1f t' % (r['tf'], r['mass_land'] / 1000)
             if st == 'optimal' else '')
    print('  到瞄准点 |v|=%.0f (vz=%.0f vh=%.0f) -> %s  %s'
          % (vmag, vz, vh, st, extra))
