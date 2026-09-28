"""验证预热解算逻辑（不连 KSP，纯逻辑 + 求解器）。

验证点：
  1. 预热门判据（alt < GATE_ALT*1.25 且 |v| < GATE_VMAX*1.15）
  2. 预测到交班门的状态是否合理
  3. 用预测状态求解是否可行、耗时多少
"""
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..', 'gfold', 'solver'))
sys.path.insert(0, os.path.join(_HERE, '..', '..', 'tools', 'stubs'))
from gfold_p3p4 import solve_p3p4_state      # noqa: E402

GATE_ALT, GATE_VMAX = 5000.0, 480.0
PAF, PVF = 1.25, 1.15
TALT = 36.8


def gate(alt, vmag):
    return (alt < GATE_ALT * PAF) and (vmag < GATE_VMAX * PVF)


def prewarm_state(alt, vz, vh, mass):
    """复刻 gfold_land._start_prewarm 的预测。"""
    est_t = max(0.0, (alt - GATE_ALT) / max(1.0, abs(vz)))
    h = alt - abs(vz) * est_t - TALT          # 预测到门时的离地高度
    return h, vz, vh, est_t


print('=== 1. 预热门判据 ===')
cases = [
    ('远高于门', 8000.0, 400.0, False),
    ('预热门内', 6000.0, 450.0, True),
    ('预热门内但太快', 6000.0, 600.0, False),
    ('正好交班门', 4900.0, 470.0, True),
    ('已过低速', 3000.0, 200.0, True),
]
ok = 0
for name, alt, vmag, want in cases:
    got = gate(alt, vmag)
    mark = 'OK ' if got == want else 'FAIL'
    ok += (got == want)
    print('  [%s] %-18s alt=%5.0f |v|=%4.0f -> %s (期望 %s)'
          % (mark, name, alt, vmag, got, want))
print('  判据 %d/%d 通过' % (ok, len(cases)))

print('\n=== 2. 预测到门的状态 ===')
# 实飞典型：预热门处 alt=6200, vz=-300, vh=230
for alt, vz, vh in [(6200.0, -300.0, 230.0), (5900.0, -290.0, 210.0)]:
    h, vzp, vhp, est_t = prewarm_state(alt, vz, vh, 183000.0)
    print('  当前 alt=%.0f vz=%.0f vh=%.0f' % (alt, vz, vh))
    print('    预测 %.2f s 后到门: 离地 %.0f m (即 alt=%.0f) vz=%.0f vh=%.0f'
          % (est_t, h, h + TALT, vzp, vhp))

print('\n=== 3. 用预测状态求解（可行性与耗时）===')
for alt, vz, vh in [(6000.0, -300.0, 230.0), (5500.0, -295.0, 215.0)]:
    h, vzp, vhp, est_t = prewarm_state(alt, vz, vh, 183000.0)
    dist = h * 0.27                       # 锥内水平偏差
    t0 = time.time()
    r = solve_p3p4_state([h, -dist, 0.0, vzp, vhp, 0.0], mass=183000.0,
                         N3=160, N4=80)
    dt = time.time() - t0
    st = r['status']
    extra = ''
    if st == 'optimal':
        extra = ('tf_m=%.1f 落地%.1ft 末倾%.2f° 末端误差%.4f'
                 % (r['tf_m'], r['mass_land'] / 1000,
                    r['end_tilt_deg'], r['end_pos_err']))
    print('  alt=%.0f -> 预测 h=%.0f: %-9s %.3fs (engine=%s)  %s'
          % (alt, h, st, dt, r.get('engine'), extra))

print('\n=== 4. 时间窗核算 ===')
vz = 300.0
win = (GATE_ALT * PAF - GATE_ALT) / vz
print('  预热门到交班门高度差 = %.0f m' % (GATE_ALT * PAF - GATE_ALT))
print('  以 vz=%.0f 计 -> 时间窗 %.2f s' % (vz, win))
print('  codegen 求解 ~0.1 s -> 余量充足')
