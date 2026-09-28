# -*- coding: utf-8 -*-
"""Final: booster landing inertia (cylinder-corrected), RCS & gimbal torques."""
import json, math

d = json.load(open(r'D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script\craft_parsed.json', encoding='utf-8'))

BASE = {
 'SSME':4.0,'LiquidEngineRE-J10':3.3,'restock-engine-375-corgi':5.25,'HotGasThruster-L':0.15,
 'restock-leg-3-rigid':0.1,'tailfin':0.125,'TPtank3mL05625':6.75,'TPtank2mL05625':3.0,
 'restock-fueltank-5-1':32.0,'restock-fueltank-adapter-375-5-1':4.0,'restock-adapter-hollow-25-375-1':1.0,
 'restock-drone-core-375-1':3.0,'probeStackLarge':1.0,'probeStackSmall':0.2,'KAL9000':0.05,
 'batteryPack':0.05,'batteryBank':0.1,'SurfAntenna':0.02,'RelayAntenna100':0.3,'Magnetometer':0.05,
 'solarPanels5':0.02,'TPmono1mL02850':0.5,'nfs-panel-static-truss-3':0.2,'fairingSize2':1.0,
 'RCSblock.01.small':0.02,'Separator.0':0.05,'Separator.2':0.2,'Separator.3':0.4,
}
RHO = {'LiquidFuel':0.005,'Oxidizer':0.005,'MonoPropellant':0.004,'ElectricCharge':0.0}

# approximate cylinder radius/length for booster tank parts (for inertia self-term)
# restock 5m tank: R~2.5 (5m dia), L=15 (nodes +-7.5). adapter: R~2.0, L=2.5.
GEOM = {'restock-fueltank-5-1':(2.5,15.0),'restock-fueltank-adapter-375-5-1':(2.0,2.5)}

parts=[]
for p in d:
    nm=p['part'].split('_')[0]
    dry=BASE.get(nm,0.1)+(p.get('modMass') or 0)
    res=p.get('resources',[])
    resmass=sum(RHO.get(r,0)*a for r,a,m in res)
    parts.append({'name':nm,'pos':p['pos'],'dry':dry,'resmass':resmass})

booster=[p for p in parts if p['pos'][1]<35.0]

RESID=0.05
for p in booster:
    p['land']=p['dry']+RESID*p['resmass']

M=sum(p['land'] for p in booster)
cx=sum(p['land']*p['pos'][0] for p in booster)/M
cy=sum(p['land']*p['pos'][1] for p in booster)/M
cz=sum(p['land']*p['pos'][2] for p in booster)/M
print(f"Landing mass={M:.2f} t  CoM=({cx:.3f},{cy:.3f},{cz:.3f})  (y up)")

# Inertia: point mass + cylinder self-term for tanks
Ip=Iyaw=Iroll=0.0
for p in booster:
    m=p['land']; dx=p['pos'][0]-cx; dy=p['pos'][1]-cy; dz=p['pos'][2]-cz
    Ip   += m*(dy*dy+dz*dz)
    Iyaw += m*(dy*dy+dx*dx)
    Iroll+= m*(dx*dx+dz*dz)
    if p['name'] in GEOM:
        R,L=GEOM[p['name']]
        # replace point contribution with cylinder: add self transverse inertia
        Iself=(1/12)*m*(3*R*R+L*L)
        Ip+=Iself; Iyaw+=Iself
        Iroll_self=0.5*m*R*R
        Iroll+=Iroll_self
print(f"Inertia (point+cyl self): I_pitch={Ip:,.0f}  I_yaw={Iyaw:,.0f}  I_roll={Iroll:,.0f} kg*m^2")
print(f"  -> about {Ip/1e6:.3f}e6 kg*m^2 pitch/yaw")

# Engine mount radius: SSME cluster
ssme=[p for p in booster if p['name']=='SSME']
import statistics
rs=[math.hypot(p['pos'][0]-cx,p['pos'][2]-cz) for p in ssme]
print(f"\nSSME n={len(ssme)}  radial dist from axis: min={min(rs):.2f} max={max(rs):.2f}")
eng_y=ssme[0]['pos'][1]
lever=cy-eng_y
print(f"Engine y={eng_y:.2f}  CoM y={cy:.2f}  gimbal lever arm = {lever:.2f} m")
# gimbal torque: 1 engine at 150% =1500kN, gimbal 10.5deg
T=1500e3; g=math.radians(10.5)
print(f"Single-engine gimbal torque = T*sin(g)*lever = {T*math.sin(g)*lever/1e6:,.2f} MN*m")
print(f"  (with all 9 thrusting, differential: up to ~{9*T*math.sin(g)*lever/1e6:.1f} MN*m, not used in landing)")

# RCS: HotGasThruster-L, thrusterPower=12 kN, positions relative CoM
rcs=[p for p in booster if p['name']=='HotGasThruster-L']
print(f"\nRCS HotGasThruster-L count in booster = {len(rcs)}  (12 kN each, LF/O)")
# top cluster (y~16.5-17) and bottom (y~2-2.5)
top=[p for p in rcs if p['pos'][1]>10]
bot=[p for p in rcs if p['pos'][1]<10]
print(f"  top ring y~{top[0]['pos'][1]:.1f}: {len(top)} thrusters, lever={top[0]['pos'][1]-cy:.1f} m")
print(f"  bot ring y~{bot[0]['pos'][1]:.1f}: {len(bot)} thrusters, lever={cy-bot[0]['pos'][1]:.1f} m")
# pitch/yaw torque from a ring firing tangentially: assume 2 opposing effective per axis
F=12e3
for ring,n in ((top,'top'),(bot,'bottom')):
    L=abs(ring[0]['pos'][1]-cy)
    # up to 4 thrusters aligned to push same tangential direction in this vernier cluster of 8
    print(f"  {ring} ring: 4 x 12kN x {L:.1f}m = {4*F*L/1e6:.2f} MN*m (optimistic, if 4 aligned)")
    print(f"           conservative 2 x 12kN x {L:.1f}m = {2*F*L/1e6:.2f} MN*m")
# roll torque: thrusters at radius ~2.4m firing tangentially
rr=[math.hypot(p['pos'][0]-cx,p['pos'][2]-cz) for p in top]
print(f"  roll: radius~{sum(rr)/len(rr):.2f} m, 4 tangential -> {4*F*2.4/1e3:.1f} kN*m")

# dry mass of booster, wet mass
dry=sum(p['dry'] for p in booster)
wet=sum(p['dry']+p['resmass'] for p in booster)
print(f"\nBooster: dry={dry:.2f}t  wet(launch)={wet:.2f}t  landing~{M:.2f}t")
lf=sum(p['resmass'] for p in booster)*9/11  # LF fraction of prop mass by ratio 0.9:1.1
print(f"prop total={sum(p['resmass'] for p in booster):.1f}t")
