# -*- coding: utf-8 -*-
"""Mass / CoM / inertia analysis of B1040-9 booster."""
import json, math

d = json.load(open(r'D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script\craft_parsed.json', encoding='utf-8'))

# part base stats: name -> (dry_mass_t, note)
# from cfg files
BASE = {
 'SSME': (4.0, 'Vector engine'),
 'LiquidEngineRE-J10': (3.3, 'Wolfhound'),
 'restock-engine-375-corgi': (5.25, 'Corgi upper-stage engine'),
 'HotGasThruster-L': (0.15, 'Vernier thruster (LF/O)'),
 'restock-leg-3-rigid': (0.1, 'landing leg'),
 'tailfin': (0.125, 'tail fin / grid fin proxy'),
 'TPtank3mL05625': (6.75, 'FTP tank'),
 'TPtank2mL05625': (3.0, 'FTP tank'),
 'restock-fueltank-5-1': (32.0, '5m tank'),
 'restock-fueltank-adapter-375-5-1': (4.0, 'adapter tank'),
 'restock-adapter-hollow-25-375-1': (1.0, 'hollow adapter est'),
 'restock-drone-core-375-1': (3.0, 'drone core est'),
 'probeStackLarge': (1.0, 'large probe core est'),
 'probeStackSmall': (0.2, 'small probe core est'),
 'KAL9000': (0.05, 'kOS computer est'),
 'batteryPack': (0.05, 'battery est'),
 'batteryBank': (0.1, 'battery bank est'),
 'SurfAntenna': (0.02, 'est'),
 'RelayAntenna100': (0.3, 'est'),
 'Magnetometer': (0.05, 'est'),
 'solarPanels5': (0.02, 'est'),
 'TPmono1mL02850': (0.5, 'monoprop tank est'),
 'nfs-panel-static-truss-3': (0.2, 'truss panel est'),
 'fairingSize2': (1.0, 'fairing est'),
 'RCSblock.01.small': (0.02, 'RCS block'),
 'Separator.0': (0.05, 'decoupler est'),
 'Separator.2': (0.2, 'decoupler est'),
 'Separator.3': (0.4, 'decoupler est'),
}

# modMass in the .craft ALREADY includes the TweakScale adjustment, so use base+modMass directly.
SCALE_MASS_MULT = {}
DEFAULT_MULT = 1.0

# Resource densities (t per unit): stock LF=0.005, OX=0.005, Mono=0.004, EC negligible
RHO = {'LiquidFuel':0.005, 'Oxidizer':0.005, 'MonoPropellant':0.004, 'ElectricCharge':0.0}

parts = []
for p in d:
    name = p['part'].split('_')[0]
    bm, note = BASE.get(name, (0.1, 'default est'))
    mult = SCALE_MASS_MULT.get(name, DEFAULT_MULT)
    dry = bm * mult + (p.get('modMass') or 0)
    res = p.get('resources', [])
    wet_add = sum(RHO.get(rn,0.0)*amt for rn,amt,mx in res)
    parts.append({
        'name': name, 'pos': p['pos'], 'dry': dry, 'wet': dry+wet_add,
        'resmass': wet_add, 'res': res, 'istg': p['istg'], 'dstg': p['dstg'], 'sepI': p['sepI'],
        'modules': p['modules'],
    })

def com(items, key):
    M = sum(i[key] for i in items)
    if M == 0: return (0,0,0), 0
    cx = sum(i[key]*i['pos'][0] for i in items)/M
    cy = sum(i[key]*i['pos'][1] for i in items)/M
    cz = sum(i[key]*i['pos'][2] for i in items)/M
    return (cx,cy,cz), M

# Full stack
for label, filt in [
    ('FULL STACK (all)', lambda i: True),
    ('BOOSTER only (dstg==0 or SSME group, y<35)', lambda i: i['pos'][1] < 35.0),
    ('STAGE2+payload (y>=35)', lambda i: i['pos'][1] >= 35.0),
]:
    items = [i for i in parts if filt(i)]
    (cw, Mw) = com(items, 'wet')
    (cd, Md) = com(items, 'dry')
    print(f"\n=== {label} ===")
    print(f"  parts={len(items)}  WET mass = {Mw:9.3f} t   DRY mass = {Md:9.3f} t")
    print(f"  CoM wet = ({cw[0]:.3f}, {cw[1]:.3f}, {cw[2]:.3f})   CoM dry = ({cd[0]:.3f}, {cd[1]:.3f}, {cd[2]:.3f})")

# Booster landing config: nearly empty tanks. Residual ~5% propellant in booster tanks.
booster = [i for i in parts if i['pos'][1] < 35.0]
# landing: dry + 5% of booster LF/OX
bl = []
for i in booster:
    land = i['dry'] + 0.05*i['resmass']
    bl.append(dict(i, land=land))
Mland = sum(i['land'] for i in bl)
cy_land = sum(i['land']*i['pos'][1] for i in bl)/Mland
cx_land = sum(i['land']*i['pos'][0] for i in bl)/Mland
cz_land = sum(i['land']*i['pos'][2] for i in bl)/Mland
print(f"\n=== BOOSTER LANDING (dry + 5% residual) ===")
print(f"  landing mass = {Mland:.3f} t  CoM=({cx_land:.3f},{cy_land:.3f},{cz_land:.3f})")

# Inertia estimate (pitch/yaw, about CoM) for landing config.
# Point-mass approx: I = sum m*(dy^2 + dr^2_lateral)
def inertia(items, key, c):
    cx,cy,cz = c
    I_pitch = 0.0; I_yaw = 0.0; I_roll = 0.0
    for i in items:
        m = i[key]
        dx = i['pos'][0]-cx; dy=i['pos'][1]-cy; dz=i['pos'][2]-cz
        # pitch about X axis: m*(dy^2+dz^2); yaw about Z: m*(dy^2+dx^2); roll about Y: m*(dx^2+dz^2)
        I_pitch += m*(dy*dy+dz*dz)
        I_yaw   += m*(dy*dy+dx*dx)
        I_roll  += m*(dx*dx+dz*dz)
    return I_pitch, I_yaw, I_roll

# But point-mass underestimates tank contribution; add cylinder model for big tanks.
# Booster main tanks: restock-fueltank-5-1 x2 (R=2.5m? actually 5m diameter => R=1.875 at scale.. check), h=15m each? node 7.5 => length 15m.
# We'll do point-mass (lower bound) and cylinder-corrected (upper bound).
Ip, Iy, Ir = inertia(bl, 'land', (cx_land,cy_land,cz_land))
print(f"  Inertia point-mass: I_pitch={Ip:,.0f}  I_yaw={Iy:,.0f}  I_roll={Ir:,.0f} kg*m^2")

# cylinder correction for the 2 big tanks + adapter: treat propellant+tank as hollow cylinder
# restock-fueltank-5-1: 5m diameter (R=2.5), length 15 m (nodes +/-7.5), dry 32t, full prop 51.2t -> landing 5% = 2.56t => m=34.56t each
R = 2.5; L = 15.0
for i in bl:
    if i['name'] == 'restock-fueltank-5-1':
        m = i['land']
        Ic = (1/12)*m*(3*R*R + L*L)   # solid cyl about transverse axis through own center
        dy = i['pos'][1]-cy_land
        Ic_total = Ic + m*(dy*dy)  # parallel axis (lateral CoM offset small)
        print(f"  tank@y={i['pos'][1]:.1f}: m={m:.1f}t  own-transverse-I={Ic:,.0f}  +parallel={Ic_total:,.0f}")

print("\n=== booster dry parts list ===")
for i in sorted(booster, key=lambda x:-x['wet']):
    print(f"  {i['name']:35s} dry={i['dry']:7.3f}t wet={i['wet']:8.3f}t res={i['resmass']:7.3f}t y={i['pos'][1]:7.2f}")

# totals of booster propellant
lf = sum(a for i in booster for rn,a,mx in i['res'] if rn=='LiquidFuel')
ox = sum(a for i in booster for rn,a,mx in i['res'] if rn=='Oxidizer')
print(f"\nBooster LF={lf} OX={ox}  prop mass={0.005*(lf+ox):.2f} t")
