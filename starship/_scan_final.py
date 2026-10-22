# -*- coding: utf-8 -*-
"""最终交接门扫描：vdesc 随交班 vz 自适应（= max(80, vz*1.15)）。"""
import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'solver'))
import numpy as np
import starship_p3p4 as G
TA=36.8; MASS=977500.0; TMAX=G.P_TMAX_DEFAULT; N3,N4=160,80

def one(prog,N,x0,tf,vdm):
    m=G._load_cgen(prog,N)
    par=m.cpg_params(); upd=m.cpg_updated(); names=m.param_names
    vd=G.vdesc_cap_vector(N,float(x0[0]),float(x0[3]),MASS,TMAX,tf,isp=315.0,
                          v_descent_max=vdm,term_win=0.4,term_vz=4.0)
    vdl=[1.0e4]*N if vd is None else [float(max(x,1e-3)) for x in np.asarray(vd,float).reshape(-1)]
    val={'x0':[float(v) for v in x0],'m_wet_log':float(np.log(MASS)),
         's_hi0':float(TMAX*0.8/MASS),'s_lo0':float(TMAX*0.1/MASS),
         'tf':float(tf),'vdesc_cap':vdl}
    for nm in names:
        v=val[nm]; setattr(par,nm,v if isinstance(v,list) else float(v)); setattr(upd,nm,1)
    m.set_solver_default_settings()
    r=m.solve(upd,par)
    if int(r.cpg_info.status)!=0: return None
    return (np.array(r.cpg_prim.x).reshape((6,N),order='F'),
            np.array(r.cpg_prim.u).reshape((3,N),order='F'))

def plan(alt,vz,vh):
    h=alt-TA; x0=np.array([h,0,0,-vz,vh,0],float)
    vdm=max(80.0, vz*1.15)
    t0=time.time()
    r3=one(3,N3,x0,40.0,vdm)
    if r3 is None: return None,(time.time()-t0)*1000,vdm
    xs,_=r3; tf_m=None
    for i in range(xs.shape[1]):
        if np.linalg.norm(xs[0:3,i])+np.linalg.norm(xs[3:6,i])<0.1:
            tf_m=i/xs.shape[1]*40.0; break
    if tf_m is None: return None,(time.time()-t0)*1000,vdm
    for k in range(20):
        tf=tf_m+3.0+k
        r4=one(4,N4,x0,tf,vdm)
        if r4 is not None:
            xs4,us4=r4
            pe=float(np.linalg.norm(xs4[0:3,-1])); ve=float(np.linalg.norm(xs4[3:6,-1]))
            up=np.degrees(np.arccos(np.clip(us4[0,:]/np.maximum(np.linalg.norm(us4,axis=0),1e-9),-1,1)))
            return dict(tf=tf,tf_m=tf_m,pe=pe,ve=ve,pk=float(np.max(up))),(time.time()-t0)*1000,vdm
    return None,(time.time()-t0)*1000,vdm

print("mass=977.5t TMAX=17.5MN TWR=%.2f  (vdesc = max(80, vz*1.15))"%(TMAX/(MASS*9.81)))
print("%6s %5s | %6s %6s %8s %7s %6s %s"%("alt","vz","tf","tf_m","pos_err","peak°","ms","结果"))
res={}
for alt in (300,400,500,600,667,800,1000,1200,1500,1800,2200,2500):
    mx=None; tfs=[]
    for vz in (20,40,60,80,100,120,140):
        r,ms,vdm=plan(alt,vz,10)
        if r is None:
            print("%6d %5d | inf"%(alt,vz)); continue
        mx=vz; tfs.append(r['tf'])
        print("%6d %5d | %6.1f %6.1f %8.2f %7.1f %6.0f %s"%(alt,vz,r['tf'],r['tf_m'],r['pe'],r['pk'],ms,"OK"))
    res[alt]=(mx, max(tfs) if tfs else None)
print()
print("=== alt -> vz_max / tf_max ===")
for a,(mx,tf) in res.items(): print("  alt=%4d  vz_max=%s  tf_max=%s"%(a,mx,("%.1f"%tf) if tf else "-"))
