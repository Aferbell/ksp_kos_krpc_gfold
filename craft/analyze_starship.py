# -*- coding: utf-8 -*-
"""analyze_starship.py -- starship.craft: mass, propellant, thrust, TWR, inertia,
attitude authority and the yaw (lateral) budget needed for the gravity turn.

CONVENTIONS (verified on this install)
  * craft has no 'mass =' lines. Each part stores modMass, and
        modMass = cfgMass * ((s/d)^3 - 1)
    so total part mass = cfgMass * (s/d)^3          [TweakScale 'stack' = volume]
    Verified against the stored LiquidFuel amounts: (9/5)^3 = 5.8320 exactly.
  * thrustPercentage is a per-instance limiter measured in PERCENT * 10:
        1000 = 100%,  250 = 250%,  500 = 500%,  1200 = 1200%
    i.e. actual thrust = cfg_maxThrust * thrustPercentage / 1000.
    (This craft runs the SSMEs at 250% and the caravels at 500%.)
"""
import re, os, math, collections

KSP = r"D:\SteamLibrary\steamapps\common\Kerbal Space Program"
CRAFT = os.path.join(KSP, r"saves\起源\Ships\VAB\starship.craft")
GAMEDATA = os.path.join(KSP, "GameData")
LF_DENS = 0.005
G0 = 9.80665
G_KERBIN = 9.81
G_MUN = 1.6285

text = open(CRAFT, encoding="utf-8", errors="replace").read()
parts = []
for m in re.finditer(r'^PART\s*\r?\n\{', text, re.M):
    i = m.start(); depth = 0; j = i
    while j < len(text):
        if text[j] == '{': depth += 1
        elif text[j] == '}':
            depth -= 1
            if depth == 0: break
        j += 1
    parts.append(text[i:j+1])

def g(b, pat, d=None):
    m = re.search(pat, b, re.M)
    return m.group(1) if m else d
def num(x, d=0.0):
    try: return float(x)
    except Exception: return d

_cfg = {}
def part_cfg(name):
    if name in _cfg: return _cfg[name]
    out = None
    for root, dirs, files in os.walk(GAMEDATA):
        for fn in files:
            if not fn.lower().endswith('.cfg'): continue
            p = os.path.join(root, fn)
            try: t = open(p, encoding='utf-8', errors='replace').read()
            except Exception: continue
            m = re.search(r'^\s*name\s*=\s*%s\s*$' % re.escape(name), t, re.M)
            if not m: continue
            st = t.rfind('\nPART', 0, m.start())
            out = t[max(0, st):m.start()+6000]
            break
        if out: break
    _cfg[name] = out
    return out

def part_props(name):
    b = part_cfg(name)
    if not b: return dict(mass=0.0, thrust=0.0, isp=300.0)
    mass = num(g(b, r'^\s*mass\s*=\s*([-\d.eE+]+)', '0'))
    mts = re.findall(r'^\s*maxThrust\s*=\s*([-\d.eE+]+)', b, re.M)
    th = max(num(x) for x in mts) if mts else 0.0
    isp = 300.0
    ac = re.search(r'atmosphereCurve\s*\r?\n\s*\{([^}]*)\}', b)
    if ac:
        for k in re.findall(r'key\s*=\s*([-\d.]+)\s+([-\d.]+)', ac.group(1)):
            if abs(num(k[0])) < 1e-9: isp = num(k[1]); break
    return dict(mass=mass, thrust=th, isp=isp)

dry = prop = 0.0
thrust = 0.0
thrust_by_pct = []
eng = collections.Counter()
pm = []; engpos = []
detail = collections.defaultdict(lambda: [0,0.0,0.0,0.0])

for b in parts:
    raw = g(b, r'^\s*part\s*=\s*(\S+)')
    base = re.sub(r'_\d+$', '', raw)
    ts = num(g(b, r'currentScale\s*=\s*([-\d.]+)'))
    dl = num(g(b, r'defaultScale\s*=\s*([-\d.]+)'))
    modm = num(g(b, r'^\s*modMass\s*=\s*([-\d.eE+]+)'))
    pct = num(g(b, r'^\s*thrustPercentage\s*=\s*([-\d.]+)'), 100.0)
    pr = part_props(base)
    m = pr['mass']*(ts/dl)**3 if (ts>0 and dl>0) else pr['mass']+max(0.0,modm)
    dry += m
    th = pr['thrust']*(pct/100.0)      # thrustPercentage is in percent (250 = 250%)
    if pr['thrust'] > 0:
        eng[(base, pct)] += 1
        thrust += th
    fu = 0.0
    for rm in re.finditer(r'RESOURCE\s*\r?\n\s*\{', b):
        i2=rm.start(); d2=0; j2=i2
        while j2 < len(b):
            if b[j2]=='{': d2+=1
            elif b[j2]=='}':
                d2-=1
                if d2==0: break
            j2+=1
        rb=b[i2:j2+1]
        if g(rb, r'^\s*name\s*=\s*(\S+)') in ('LiquidFuel','Oxidizer'):
            fu += num(g(rb, r'^\s*amount\s*=\s*([-\d.eE+]+)', '0'))
    prop += fu*LF_DENS
    detail[base][0]+=1; detail[base][1]+=m; detail[base][2]+=fu*LF_DENS; detail[base][3]+=th
    mm = re.search(r'^\s*pos\s*=\s*([-\d.eE+]+),([-\d.eE+]+),([-\d.eE+]+)', b, re.M)
    if mm:
        p3=(num(mm.group(1)),num(mm.group(2)),num(mm.group(3)))
        pm.append((p3, m+fu*LF_DENS))
        if th>0: engpos.append((p3, th))

wet = dry+prop
print('='*78)
print('starship.craft    parts=%d' % len(parts))
print('='*78)
print('dry mass   = %9.1f t' % dry)
print('propellant = %9.1f t' % prop)
print('WET mass   = %9.1f t' % wet)
print('')
print('%-26s %4s %8s %11s %12s' % ('engine','x','pct','kN/ea','kN total'))
for (k,p),n in sorted(eng.items()):
    i = part_props(k)
    print('%-26s %4d %7.0f%% %11.0f %12.0f' % (k, n, p, i['thrust']*p/100.0, n*i['thrust']*p/100.0))
print('TOTAL THRUST = %.0f kN' % thrust)
print('')
for lbl, gg in (('Kerbin', G_KERBIN), ('Mun', G_MUN)):
    print('%-7s TWR  wet=%.2f   dry=%.2f' % (lbl, thrust/(wet*gg), thrust/(dry*gg)))
ispw = sum(part_props(k)['isp']*n for (k,p),n in eng.items())/max(1,sum(eng.values()))
ve = ispw*G0
print('')
print('Isp(vac, avg) = %.1f s  ve = %.0f m/s  dV = %.0f m/s' % (ispw, ve, ve*math.log(wet/dry)))

if pm:
    M=sum(m for _,m in pm)
    cx=sum(p[0]*m for p,m in pm)/M; cy=sum(p[1]*m for p,m in pm)/M; cz=sum(p[2]*m for p,m in pm)/M
    Ixx=sum(m*((p[1]-cy)**2+(p[2]-cz)**2) for p,m in pm)
    Iyy=sum(m*((p[0]-cx)**2+(p[2]-cz)**2) for p,m in pm)
    Izz=sum(m*((p[0]-cx)**2+(p[1]-cy)**2) for p,m in pm)
    print('')
    print('COM=(%.2f,%.2f,%.2f)' % (cx,cy,cz))
    print('I_transverse ~ %.3e kg*m^2   I_axial ~ %.3e kg*m^2'
          % (max(Ixx,Izz)*1000, Iyy*1000))
    print('axial extent = %.1f m' % (max(p[1] for p,_ in pm)-min(p[1] for p,_ in pm)))

if engpos:
    M=sum(m for _,m in pm); cy=sum(p[1]*m for p,m in pm)/M
    n=len(engpos)
    lev=sum(abs(p[1]-cy) for p,_ in engpos)/n
    tb=sum(t*math.sin(math.radians(10.5))*abs(p[1]-cy) for p,t in engpos)
    I=max(Ixx,Izz)*1000
    print('')
    print('engines=%d  avg lever=%.1f m  gimbal torque=%.3e N*m' % (n, lev, tb*1000))
    print('angular accel = %.4f rad/s^2 = %.2f deg/s^2' % (tb*1000/I, math.degrees(tb*1000/I)))
