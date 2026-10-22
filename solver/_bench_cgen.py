# -*- coding: utf-8 -*-
import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import gfold_p3p4 as G

N3, N4 = 160, 80
def probe(tag, x0, mass, tf):
    out=[]
    for prog,N in ((3,N3),(4,N4)):
        m = G._load_cgen(prog, N)
        par=m.cpg_params(); upd=m.cpg_updated(); names=m.param_names
        vd=G.vdesc_cap_vector(N,float(x0[0]),float(x0[3]),mass,8.99e6,tf,isp=315.0,v_descent_max=None)
        vdl=[1.0e4]*N if vd is None else [float(max(x,1e-3)) for x in np.asarray(vd,float).reshape(-1)]
        val={'x0':[float(v) for v in x0],'m_wet_log':float(np.log(mass)),
             's_hi0':float(G.P_TMAX_DEFAULT*0.8/mass),'s_lo0':float(G.P_TMAX_DEFAULT*0.1/mass),
             'tf':float(tf),'vdesc_cap':vdl}
        for nm in names:
            v=val[nm]; setattr(par,nm,v if isinstance(v,list) else float(v)); setattr(upd,nm,1)
        m.set_solver_default_settings()
        t0=time.time(); res=m.solve(upd,par); dt=(time.time()-t0)*1000
        out.append("P%d st=%s %.0fms" % (prog,res.cpg_info.status,dt))
    print("  %-40s %s" % (tag, "  ".join(out)))

print("=== 质量对可行性/耗时的影响（同一几何 x0）===")
x0 = np.array([662.4, 0.0, 0.0, -67.4, 14.2, 0.0], float)
for mass_t in (152, 500, 977, 1488):
    probe("mass=%dt tf=11.8" % mass_t, x0, mass_t*1000.0, 11.8)

print("\n=== tf 扫描（mass=977t）===")
for tf in (11.8, 14.0, 18.0, 24.0, 30.0):
    probe("tf=%.1f" % tf, x0, 977500.0, tf)
