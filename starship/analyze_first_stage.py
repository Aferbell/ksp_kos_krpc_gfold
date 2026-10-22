# -*- coding: utf-8 -*-
"""analyze_first_stage.py -- starship.craft 的【一级】(分离器以下) 参数提取。

判据
  * 分离器 restock-separator-5-1 位于 y≈71.7 m（decoupler y≈73.2 在其上）。
  * 一级 = y < 71.7 的所有部件（含 2 个 fueltank-5-1、33 台 SSME、尾翼、着陆腿）。
  * 33 台 SSME 带 KOSNameTag e1..e33；回收时只留 e1,e2,e3,e5,e8,e10,e13 共 7 台。
  * thrustPercentage 是百分数（250 = 250%）。
  * TweakScale: mass = cfgMass*(s/d)^3。
"""
import re, os, math, json

KSP  = r"D:\SteamLibrary\steamapps\common\Kerbal Space Program"
CRAFT = os.path.join(KSP, r"saves\起源\Ships\VAB\starship.craft")
GAMEDATA = os.path.join(KSP, "GameData")
LF_DENS = 0.005
G0 = 9.80665
SEP_Y = 71.7                     # 分离面（separator 中心高度）
KEEP = {"e1","e2","e3","e5","e8","e10","e13"}   # 回收时保留的引擎

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
    except Exception: return dv

_c={}
def cfg(name):
    if name in _c: return _c[name]
    out=None
    for r_,_dn,fns in os.walk(GAMEDATA):
        for fn in fns:
            if not fn.lower().endswith('.cfg'): continue
            p=os.path.join(r_,fn)
            try: t=open(p,encoding='utf-8',errors='replace').read()
            except Exception: continue
            m=re.search(r'^\s*name\s*=\s*%s\s*$'%re.escape(name),t,re.M)
            if not m: continue
            st=t.rfind('\nPART',0,m.start()); out=t[max(0,st):m.start()+6000]; break
        if out: break
    _c[name]=out; return out

def props(name):
    b=cfg(name)
    if not b: return dict(mass=0.0,thr=0.0,isp=300.0,found=False)
    mass=num(g(b,r'^\s*mass\s*=\s*([-\d.eE+]+)','0'))
    mts=re.findall(r'^\s*maxThrust\s*=\s*([-\d.eE+]+)',b,re.M)
    thr=max(num(x) for x in mts) if mts else 0.0
    isp=300.0
    ac=re.search(r'atmosphereCurve\s*\r?\n\s*\{([^}]*)\}',b)
    if ac:
        for k in re.findall(r'key\s*=\s*([-\d.]+)\s+([-\d.]+)',ac.group(1)):
            if abs(num(k[0]))<1e-9: isp=num(k[1]); break
    return dict(mass=mass,thr=thr,isp=isp,found=True)

rows=[]
for b in parts:
    raw=g(b,r'^\s*part\s*=\s*(\S+)') or ''; base=re.sub(r'_\d+$','',str(raw))
    ts=num(g(b,r'currentScale\s*=\s*([-\d.]+)')); dl=num(g(b,r'defaultScale\s*=\s*([-\d.]+)'))
    mm=num(g(b,r'^\s*modMass\s*=\s*([-\d.eE+]+)'))
    pct=num(g(b,r'^\s*thrustPercentage\s*=\s*([-\d.]+)'),100.0)
    tag=g(b,r'^\s*nameTag\s*=\s*(\S+)','') or ''
    pm=re.search(r'^\s*pos\s*=\s*([-\d.eE+]+),([-\d.eE+]+),([-\d.eE+]+)',b,re.M)
    y=num(pm.group(2)) if pm else 0.0
    pr=props(base)
    m=pr['mass']*(ts/dl)**3 if (ts>0 and dl>0) else pr['mass']+max(0.0,mm)
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
    rows.append(dict(base=base, y=y, dry=m, prop=fu*LF_DENS,
                     thr=pr['thr']*pct/100.0, isp=pr['isp'], tag=tag,
                     is_engine=pr['thr']>0))

stage1=[r for r in rows if r['y'] < SEP_Y]
eng=[r for r in stage1 if r['is_engine']]
eng_keep=[r for r in eng if r['tag'] in KEEP]

dry  = sum(r['dry'] for r in stage1)
prop = sum(r['prop'] for r in stage1)
all_thr  = sum(e['thr'] for e in eng)
keep_thr = sum(e['thr'] for e in eng_keep)
wet = dry + prop

print("="*74)
print("starship 一级（分离器以下 y<%.1f m）" % SEP_Y)
print("="*74)
print("部件数        = %d  (其中引擎 %d)" % (len(stage1), len(eng)))
print("干重          = %9.1f t" % dry)
print("推进剂        = %9.1f t" % prop)
print("湿重          = %9.1f t" % wet)
print("")
print("全部引擎      = %d 台, 推力 %.1f MN" % (len(eng), all_thr/1000.0))
print("保留 e1/2/3/5/8/10/13 = %d 台, 推力 %.1f MN" % (len(eng_keep), keep_thr/1000.0))
print("关闭的引擎    = %d 台" % (len(eng)-len(eng_keep)))
print("单台推力(250%%) = %.0f kN" % (eng[0]['thr'] if eng else 0))

isp = (sum(e['isp'] for e in eng_keep)/len(eng_keep)) if eng_keep else 315.0
ve = isp*G0
print("")
print("Isp(vac)      = %.0f s -> ve=%.0f m/s" % (isp, ve))
for lbl, th in (("全部33台", all_thr), ("保留7台", keep_thr)):
    print("%-8s TWR Kerbin wet=%.2f dry=%.2f" % (lbl, th/(wet*9.81), th/(dry*9.81)))

# 力矩/惯量（一级）
M=sum(r['dry']+r['prop'] for r in stage1)
cy=sum(r['y']*(r['dry']+r['prop']) for r in stage1)/M
Ix=sum((r['dry']+r['prop'])*((r['y']-cy)**2) for r in stage1)
print("")
print("一级质心 y    = %.2f m  (相对 craft 原点)" % cy)
print("一级轴向范围  = %.1f .. %.1f m" % (min(r['y'] for r in stage1), max(r['y'] for r in stage1)))
print("I(横向,点质量)= %.3e kg*m^2" % (Ix*1000))
if eng_keep:
    lev=[abs(e['y']-cy) for e in eng_keep]
    tb=sum(e['thr']*math.sin(math.radians(10.5))*abs(e['y']-cy) for e in eng_keep)
    print("保留引擎平均力臂 = %.1f m" % (sum(lev)/len(lev)))
    print("gimbal 力矩(7台) = %.3e N*m" % (tb*1000))
    if Ix>0:
        print("角加速度 = %.3f deg/s^2" % math.degrees((tb*1000)/(Ix*1000)))

out = dict(sep_y=SEP_Y, parts=len(stage1), engines_all=len(eng), engines_keep=len(eng_keep),
           keep_tags=sorted(KEEP), dry_t=round(dry,1), prop_t=round(prop,1), wet_t=round(wet,1),
           thrust_all_kN=round(all_thr,0), thrust_keep_kN=round(keep_thr,0),
           isp_vac=round(isp,1), ve=round(ve,0),
           twr_wet_keep=round(keep_thr/(wet*9.81),2), twr_dry_keep=round(keep_thr/(dry*9.81),2),
           com_y=round(cy,2), I_transverse_kgm2=round(Ix*1000,0))
print("\nJSON:")
print(json.dumps(out, ensure_ascii=False, indent=2))
