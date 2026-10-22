# -*- coding: utf-8 -*-
"""离线自检：用 gfold_log_20261020_132426 的真实交接状态跑新 solver。"""
import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import starship_config as C
import starship_p3p4 as S

# 日志交接实测（L14）：alt=662.4 vz=-67.4 vh=14.2 mass=977.5t
alt, vz, vh, mass = 662.4, 67.4, 14.2, 977500.0
h_rel = alt - C.TARGET_ALT
x0 = np.array([h_rel, 0.0, 0.0, -vz, vh, 0.0], float)
vdesc = C.vdesc_max(vz)

print("=" * 66)
print("starship 一级 solver 离线自检")
print("=" * 66)
print("交接状态: alt=%.1f vz=%.1f vh=%.1f mass=%.1ft" % (alt, vz, vh, mass/1000))
print("x0(rel)=%s  vdesc=%.1f  TMAX=%.1f MN" % (np.round(x0,1), vdesc, C.TMAX_N/1e6))
print()

for tag, m in (("交接 977.5t", mass), ("燃尽 628.5t", C.DRY_MASS_T*1000),
               ("中间 800t", 800000.0)):
    t0=time.time()
    r = S.solve_p3p4_state(x0, m, isp=C.ISP_VAC, t_max=C.TMAX_N,
                           throttle=(0.1,1.0), tf_guess=C.TF_GUESS,
                           straight_fac=5.0, gs_deg=30.0,
                           pcs_deg=5.0, pcs_start_deg=85.0, v_max=1200.0,
                           N3=160, N4=80, target_alt=C.TARGET_ALT,
                           v_descent_max=vdesc, term_win=0.40,
                           term_vz=C.TERM_DESCENT_VZ)
    dt=(time.time()-t0)*1000
    st=r.get('status')
    if st=='optimal':
        print("%-14s tf=%5.1f  %6.0f ms  pos_err=%.2f  vel_err=%.3f  fuel=%.1ft  %s" %
              (tag, r['tf'], dt, r['end_pos_err'], r['end_vel_err'],
               r['fuel']/1000, "OK" if r['end_pos_err']<C.POS_TOL_M else "偏"))
    else:
        print("%-14s %s  %6.0f ms" % (tag, st, dt))
