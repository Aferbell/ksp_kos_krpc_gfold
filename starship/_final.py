# -*- coding: utf-8 -*-
import io, sys, os, math, time
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
sys.path.insert(0, os.path.join(os.getcwd(),'dev','starship'))
sys.path.insert(0, os.path.join(os.getcwd(),'dev','solver'))
import numpy as np
import starship_land as L, starship_config as C

print("="*74)
print("最终参数总览")
print("="*74)
print("  引擎         : %d 台  TMAX=%.2f MN" % (len(C.ENGINE_TAGS_KEEP), C.TMAX_N/1e6))
print("  AIM_ALT      : %.0f m  GATE_ALT=%.0f m" % (C.AIM_ALT, C.GATE_ALT))
print("  REPLAN_DT    : %.1f s" % C.REPLAN_DT)
print("  PID          : kp=%.1f kd=%.1f (原 12/26)" % (C.CTRL_X_ROT_KP, C.CTRL_X_ROT_KD))
print("  调度         : ALPHA_REF=%.1f MAX=%.2f (原 10.16/1.60)"
      % (C.TAU_SCHED_ALPHA_REF, C.TAU_SCHED_MAX))
print("  倾角         : slope=%.1f LO=%.1f HI=%.1f TERM=%.0f (原 3/3/35/90)"
      % (C.TILT_CAP_SLOPE, C.TILT_CAP_LO, C.TILT_CAP_HI, C.TILT_TERM_ALT))
print("  油门下限     : MIN=%.2f MARGIN=%.2f RELEASE_FRAC=%.2f"
      % (C.GFOLD_THROTTLE_MIN, C.GFOLD_THROTTLE_MARGIN, C.GFOLD_THROTTLE_RELEASE_FRAC))
print()
print("="*74)
print("求解器验证（本次实飞交班状态）")
print("="*74)
rep13 = 6.62e6*13/7
class FV:
    mass=1005456.0; available_thrust=rep13; max_thrust=rep13
    thrust=0.0; name='starship'
ship = object.__new__(L.Ship)
ship.v=FV(); ship._tscale=2.500; ship._tscale_buf=[]
ship._tscale_prev_cmd=float('nan'); ship.solve_ms=0.0
ship.gfold_status='init'; ship.dbg_plan_tf_used=float('nan')
ship.dbg_vdesc=float('nan'); ship.dbg_plan_x0_up=float('nan')
x0 = np.array([932.3-C.TARGET_ALT, 0.0, 0.0, -72.1, 11.3, 0.0])
t0=time.time(); r=ship.solve(x0,1005456.0,allow_cvxpy_fallback=True); dt=(time.time()-t0)*1000
print("  status=%s  %.0f ms  tf=%.2f" % (ship.gfold_status, dt, (r or {}).get('tf',0)))
print()
print("="*74)
print("PID 检查（D 项已归一化：rate = RATE_D_GAIN*omega/omega_phys）")
print("="*74)
a = math.radians(47.59*0.321 - 3.936)     # 悬停权限 rad/s^2（13 台）
head = math.radians(C.REF_SLEW_HEADROOM_DEG)
wphys = math.sqrt(2.0*a*head)
print("  悬停 alpha=%.2f deg/s^2  =>  omega_phys=%.3f rad/s (%.1f deg/s)"
      % (math.degrees(a), wphys, math.degrees(wphys)))
print("  %-8s %-8s %-9s %-8s %-11s %-11s" %
      ("sched", "kp", "kd_eff", "wn", "D@0.2phys", "D@0.5phys"))
#   归一化后解析式：zeta = kd*G/(2*sqrt(2*headroom)) / sqrt(kp)
#     （推导：D=kd*G*w/wphys, wphys=sqrt(2*alpha*h)
#        zeta = kd_eff/2*sqrt(alpha/kp), kd_eff=kd*G/wphys
#             = kd*G/(2*sqrt(2h)) / sqrt(kp)  —— 与 alpha 只剩 -1/4 次幂）
_zz = C.CTRL_X_ROT_KD*C.RATE_D_GAIN/(2.0*math.sqrt(2.0*head))
for sc in (0.45, 1.0, 1.6, 1.9):
    kp = C.CTRL_X_ROT_KP*sc
    kde = C.CTRL_X_ROT_KD*C.RATE_D_GAIN*sc
    zeta = _zz/math.sqrt(kp)
    print("  %-8.2f %-8.1f %-9.2f %-8.2f %-11.3f %-11.3f  zeta=%.2f"
          % (sc, kp, kde, math.sqrt(a*kp), kde*0.2, kde*0.5, zeta))
print()
print("  归一化后 D 的上界只取决于 kd*G，与载具角速率峰值无关：")
print("    kd=%.1f G=%.2f => D_max=%.2f（omega>=omega_phys 时才可能 >=1）"
      % (C.CTRL_X_ROT_KD, C.RATE_D_GAIN, C.CTRL_X_ROT_KD*C.RATE_D_GAIN))
