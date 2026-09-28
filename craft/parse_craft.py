# -*- coding: utf-8 -*-
"""Parse B1040-9.craft: extract parts, positions, modules of interest, resources."""
import re, json, collections

PATH = r"D:\SteamLibrary\steamapps\common\Kerbal Space Program\saves\起源\Ships\VAB\B1040-9.craft"

text = open(PATH, encoding="utf-8", errors="replace").read()

# Split into PART blocks
parts = []
# find all "PART\n{" ... matching closing brace at same indent level (top-level)
idx = 0
n = len(text)
while True:
    m = re.search(r'^PART\s*$', text[idx:], re.M)
    if not m:
        break
    start = idx + m.end()
    # expect '{'
    m2 = re.search(r'^\{\s*$', text[start:], re.M)
    if not m2:
        break
    depth = 0
    i = start + m2.start()
    # scan lines
    lines = text[i:].split('\n')
    depth = 0
    block_lines = []
    for ln in lines:
        s = ln.strip()
        if s == '{':
            depth += 1
        elif s == '}':
            depth -= 1
        block_lines.append(ln)
        if depth == 0:
            break
    block = '\n'.join(block_lines)
    parts.append(block)
    idx = i + len(block)

print("TOTAL PARTS:", len(parts))

def parse_block(block):
    p = {}
    m = re.search(r'^\s*part = (\S+)', block, re.M)
    p['part'] = m.group(1) if m else '?'
    m = re.search(r'^\s*pos = ([-\d.Ee]+),([-\d.Ee]+),([-\d.Ee]+)', block, re.M)
    p['pos'] = [float(m.group(1)), float(m.group(2)), float(m.group(3))] if m else None
    m = re.search(r'^\s*istg = (-?\d+)', block, re.M)
    p['istg'] = int(m.group(1)) if m else None
    m = re.search(r'^\s*dstg = (-?\d+)', block, re.M)
    p['dstg'] = int(m.group(1)) if m else None
    m = re.search(r'^\s*sepI = (-?\d+)', block, re.M)
    p['sepI'] = int(m.group(1)) if m else None
    m = re.search(r'^\s*modMass = ([-\d.Ee]+)', block, re.M)
    p['modMass'] = float(m.group(1)) if m else 0.0
    # MODULE names + interesting keys
    modules = []
    for mm in re.finditer(r'MODULE\s*\{(.*?)\n\t\}', block, re.S):
        mb = mm.group(1)
        nm = re.search(r'name = (\S+)', mb)
        name = nm.group(1) if nm else '?'
        modules.append((name, mb))
    p['modules'] = modules
    # resources
    res = []
    for rm in re.finditer(r'RESOURCE\s*\{(.*?)\n\t\}', block, re.S):
        rb = rm.group(1)
        nm = re.search(r'name = (\S+)', rb)
        am = re.search(r'amount = ([-\d.Ee]+)', rb)
        mx = re.search(r'maxAmount = ([-\d.Ee]+)', rb)
        res.append((nm.group(1) if nm else '?', float(am.group(1)) if am else 0, float(mx.group(1)) if mx else 0))
    p['resources'] = res
    return p

parsed = [parse_block(b) for b in parts]

# Part name counts
cnt = collections.Counter(p['part'].split('_')[0] for p in parsed)
print("\n=== PART NAME COUNTS ===")
for k, v in sorted(cnt.items()):
    print(f"{v:4d}  {k}")

# Stage info
istgs = collections.Counter(p['istg'] for p in parsed)
print("\n=== istg distribution ===", dict(sorted(istgs.items())))

# Engine / gimbal / RCS modules details
print("\n=== PARTS WITH KEY MODULES ===")
for p in parsed:
    names = [m[0] for m in p['modules']]
    keys = {}
    for nm, mb in p['modules']:
        if nm in ('ModuleEnginesFX','ModuleEngines'):
            for key in ('thrustPercentage','engineType','throttleLocked','flameout','ignited'):
                km = re.search(rf'{key} = (\S+)', mb)
                if km: keys[key] = km.group(1)
        if nm == 'ModuleGimbal':
            for key in ('gimbalRange','gimbalResponseSpeed','useGimbalResponseSpeed','gimbalActive'):
                km = re.search(rf'{key} = (\S+)', mb)
                if km: keys['gimbal_'+key] = km.group(1)
        if nm == 'ModuleRCSFX':
            for key in ('thrustPercentage','useThrottle','fullThrust'):
                km = re.search(rf'{key} = (\S+)', mb)
                if km: keys['RCS_'+key] = km.group(1)
        if nm == 'ModuleWheelBase' or 'wheel' in nm.lower() or nm in ('ModuleLandingLeg','ModuleLandingGear','ModuleWheelDeployment'):
            keys['wheelmod'] = nm
        if nm == 'ModuleControlSurface' or 'Grid' in nm or nm == 'FARControlSys' or nm=='ModuleAeroSurface':
            for key in ('ctrlSurfaceRange','deploy'):
                km = re.search(rf'{key} = (\S+)', mb)
                if km: keys['cs_'+key] = km.group(1)
    interesting = any(x in names for x in ('ModuleEnginesFX','ModuleEngines','ModuleGimbal','ModuleRCSFX','ModuleControlSurface')) or keys.get('wheelmod')
    if interesting:
        print(f"{p['part']:45s} pos={p['pos']} istg={p['istg']} dstg={p['dstg']} sepI={p['sepI']} keys={keys}")

# TweakScale
print("\n=== TweakScale parts ===")
for p in parsed:
    for nm, mb in p['modules']:
        if nm == 'TweakScale':
            sc = re.search(r'currentScale = ([-\d.Ee]+)', mb)
            ty = re.search(r'type = (\S+)', mb)
            print(f"{p['part']:45s} scale={sc.group(1) if sc else '?'} type={ty.group(1) if ty else '?'}")
            break

# Resources totals
print("\n=== RESOURCES TOTALS (amount / maxAmount) ===")
tot = collections.defaultdict(lambda: [0.0, 0.0])
for p in parsed:
    for nm, am, mx in p['resources']:
        tot[nm][0] += am
        tot[nm][1] += mx
for k, (a, m) in tot.items():
    print(f"{k:25s} amount={a:12.2f}  max={m:12.2f}")

# per-part resources
print("\n=== PER-PART RESOURCES (nonzero) ===")
for p in parsed:
    if p['resources']:
        print(f"{p['part']:45s} pos={p['pos']} res={p['resources']}")

# Save parsed summary as JSON for later
out = []
for p in parsed:
    out.append({'part': p['part'], 'pos': p['pos'], 'istg': p['istg'], 'dstg': p['dstg'], 'sepI': p['sepI'], 'modMass': p['modMass'], 'modules': [m[0] for m in p['modules']], 'resources': p['resources']})
open(r"D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script\craft_parsed.json", 'w', encoding='utf-8').write(json.dumps(out, indent=1))
print("\nSaved craft_parsed.json")
