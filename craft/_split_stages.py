# -*- coding: utf-8 -*-
"""Split starship.craft at the separator (y ~= 73.24) and report each side."""
import re, os, math, collections
KSP = r"D:\SteamLibrary\steamapps\common\Kerbal Space Program"
CRAFT = os.path.join(KSP, r"saves\起源\Ships\VAB\starship.craft")
GAMEDATA = os.path.join(KSP, "GameData")
LF = 0.005

text = open(CRAFT, encoding="utf-8", errors="replace").read()
parts=[]
for m in re.finditer(r'^PART\s*\r?\n\{', text, re.M):
    i=m.start(); d=0; j=i
    while j < len(text):
        if text[j]=='{': d+=1
        elif text[j]=='}':
            d-=1
            if d==0: break
        j+=1
    parts.append(text[i:j+1])

def g(b,pat,dv=None):
    m=re.search(pat,b,re.M); return m.group(1) if m else dv
def num(x,dv=0.0):
    try: return float(x)
    except: return dv

cache={}
def cfg(name):
    if name in cache: return cache[name]
    out=None
    for r_,dn,fns in os.walk(GAMEDATA):
        for fn in fns:
            if not fn.lower().endswith('.cfg'): continue
            p=os.path.join(r_,fn)
            try: t=open(p,encoding='utf-8',errors='replace').read()
            except: continue
            m=re.search(r'^\s*name\s*=\s*%s\s*$'%re.escape(name),t,re.M)
            if not m: continue
            st=t.rfind('\nPART',0,m.start()); out=t[max(0,st):m.start()+6000]; break
        if out: break
    cache[name]=out; return out

def props(name):
    b=cfg(name)
    if not b: return dict(mass=0,thr=0,isp=300)
    mass=num(g(b,r'^\s*mass\s*=\s*([-\d.eE+]+)','0'))
    mts=re.findall(r'^\s*maxThrust\s*=\s*([-\d.eE+]+)',b,re.M)
    thr=max(num(x) for x in mts) if mts else 0
    isp=300
    ac=re.search(r'atmosphereCurve\s*\r?\n\s*\{([^}]*)\}',b)
    if ac:
        for k in re.findall(r'key\s*=\s*([-\d.]+)\s+([-\d.]+)',ac.group(1)):
            if abs(num(k[0]))<1e-9: isp=num(k[1]); break
    return dict(mass=mass,thr=thr,isp=isp)

print("=== all parts with y, mass, tag ===")
print("%-34s %8s %10s %8s %6s %s"%("part","y","dry_t","prop_t","istg","tag"))
info=[]
for b in parts:
    raw=g(b,r'^\s*part\s*=\s*(\S+)'); base=re.sub(r'_\d+$','',raw)
    ts=num(g(b,r'currentScale\s*=\s*([-\d.]+)')); dl=num(g(b,r'defaultScale\s*=\s*([-\d.]+)'))
    mm=num(g(b,r'^\s*modMass\s*=\s*([-\d.eE+]+)'))
    pct=num(g(b,r'^\s*thrustPercentage\s*=\s*([-\d.]+)'),100.0)
    istg=g(b,r'^\s*istg\s*=\s*(-?\d+)','?')
    tag=g(b,r'^\s*nameTag\s*=\s*(\S+)','')
    pm=re.search(r'^\s*pos\s*=\s*([-\d.eE+]+),([-\d.eE+]+),([-\d.eE+]+)',b,re.M)
    y=num(pm.group(2)) if pm else 0
    pr=props(base)
    m=pr['mass']*(ts/dl)**3 if (ts>0 and dl>0) else pr['mass']+max(0,mm)
    fu=0.0
    for rm in re.finditer(r'RESOURCE\s*\r?\n\s*\{',b):
        i2=rm.start(); d2=0; j2=i2
        while j2<len(b):
            if b[j2]=='{': d2+=1
            elif b[j2]=='}':
                d2-=1
                if d2==0: break
            j2+=1
        rb=b[i2:j2+1]
        if g(rb,r'^\s*name\s*=\s*(\S+)') in ('LiquidFuel','Oxidizer'):
            fu+=num(g(rb,r'^\s*amount\s*=\s*([-\d.eE+]+)','0'))
    info.append(dict(base=base,y=y,dry=m,prop=fu*LF,istg=istg,tag=tag,thr=pr['thr']*pct/100.0))

info.sort(key=lambda r:-r['y'])
for r in info:
    print("%-34s %8.1f %10.2f %8.1f %6s %s"%(r['base'][:34],r['y'],r['dry'],r['prop'],r['istg'],r['tag']))

sep_y = 73.24
upper=[r for r in info if r['y']>=sep_y]
lower=[r for r in info if r['y']<sep_y]
def agg(rs,lbl):
    print("\n=== %s  parts=%d ==="%(lbl,len(rs)))
    print("  dry  = %.1f t"%sum(r['dry'] for r in rs))
    print("  prop = %.1f t"%sum(r['prop'] for r in rs))
    print("  thrust= %.0f kN"%sum(r['thr'] for r in rs))
    c=collections.Counter(r['base'] for r in rs)
    for k,v in c.most_common(12): print("     %-30s x%d"%(k,v))
agg(upper,"UPPER (y>=73.24)")
agg(lower,"LOWER (y<73.24)")
