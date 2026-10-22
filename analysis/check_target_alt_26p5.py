import io, sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
sys.path.insert(0, 'dev/starship')
import numpy as np
import starship_config as C
import starship_p3p4 as G

print('=== 改前常量 ===')
print('  TARGET_ALT=%.1f TERM_CLEARANCE=%.1f FIN_ALT=%.1f LAND_ALT_EPS=%.1f' % (
    C.TARGET_ALT, C.TERM_CLEARANCE, C.FIN_ALT, C.LAND_ALT_EPS))
print()

p3 = G._load_cgen(3, 160)
p4 = G._load_cgen(4, 80)
print('  P3=%s P4=%s' % (p3 is not None, p4 is not None))
print()

mass = 1004100.0
# 实测交班：alt=929.3 vz=-73.6 vh=5.3
cases = [
    ('实测交班 929.3/-73.6/5.3', 929.3, -73.6, 5.3),
    ('172305 同点 921.9/-75.7/5.8', 921.9, -75.7, 5.8),
    ('低空 400/-50/3', 400.0, -50.0, 3.0),
]
print('  %-30s %-10s %-10s %-10s %s' % ('工况', 'TA=18', 'TA=26.5', 'tf差', '判定'))
for tag, alt, vz, vh in cases:
    out = []
    for ta in (18.0, 26.5):
        h_rel = alt - ta - C.TERM_CLEARANCE
        x0 = np.array([h_rel, 0.0, 0.0, vz, vh, 0.0])
        try:
            r = G.solve_p3p4_state(
                x0, mass, isp=315.0, t_max=C.TMAX_N,
                throttle=(0.1, 0.8), tf_guess=40.0, straight_fac=5.0,
                gs_deg=30.0, pcs_deg=5.0, pcs_start_deg=85.0,
                v_max=1200.0, N3=160, N4=80, target_alt=ta)
            out.append((r.get('status'), r.get('tf', float('nan'))))
        except Exception as e:
            out.append(('EXC:' + type(e).__name__, float('nan')))
    s18, tf18 = out[0]
    s265, tf265 = out[1]
    print('  %-30s %-10s %-10s %-10.2f %s' % (
        tag, s18, s265, tf265 - tf18,
        'OK' if s265 == 'optimal' else '**不可行**'))
