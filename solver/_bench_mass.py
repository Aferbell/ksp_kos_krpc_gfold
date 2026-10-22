# -*- coding: utf-8 -*-
"""Why does codegen return infeasible for heavy vessels? Show z0i/z0l distortion."""
import numpy as np, importlib
import gfold_codegen as GC

alpha = 1.0/(315.0*9.80665)
r2 = GC.T_MAX*GC.THROTTLE[1]
frac = np.linspace(0.0, (GC.N_P3-1)/GC.N_P3, GC.N_P3)

print("MASS_GEN = %.0f kg   T_MAX=%.0f  THROTTLE=%s" % (GC.MASS_GEN, GC.T_MAX, GC.THROTTLE))
print("alpha=%.6e  r2=%.3e" % (alpha, r2))
print()
print("%10s %14s %14s" % ("mass_t", "z0_term[last]", "0.05*mass"))
for m in (152, 183, 500, 977, 1488, 3000):
    z0 = GC.MASS_GEN - alpha*r2*(GC.TF_GEN*frac)
    z0 = np.maximum(z0, 0.05*GC.MASS_GEN)
    print("%10.0f %14.1f %14.1f   (actual mass %.0f)" % (m, z0[-1], 0.05*GC.MASS_GEN, m*1000))
print()
print("=> z0_term 用 MASS_GEN=183t 算；实际 977t 时展开点差了 %.1fx" % (977/183.0))
print("   s_hi/s_lo 用真实质量算 => 与展开点不匹配 => infeasible")
