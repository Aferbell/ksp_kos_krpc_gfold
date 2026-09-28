# -*- coding: utf-8 -*-
import math
xs=[0.122,-1.718,-1.179,0.122,1.423,1.963,1.423,0.122,-1.179]
zs=[0.176,0.176,1.478,2.017,1.478,0.176,-1.125,-1.664,-1.125]
r=[math.hypot(x-0.122,z-0.176) for x,z in zip(xs,zs)]
print("engine radial dists (m):",[round(v,2) for v in r])
# booster extent
import json
d=json.load(open(r'D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script\craft_parsed.json',encoding='utf-8'))
b=[p for p in d if p['pos'][1]<35]
ys=[p['pos'][1] for p in b]
xs2=[p['pos'][0] for p in b]; zs2=[p['pos'][2] for p in b]
print(f"booster y range: {min(ys):.2f} .. {max(ys):.2f}  length~{max(ys)-min(ys):.1f} m")
print(f"booster x range: {min(xs2):.2f}..{max(xs2):.2f}  z range: {min(zs2):.2f}..{max(zs2):.2f}")
rr=[math.hypot(p['pos'][0]-0.122,p['pos'][2]-0.176) for p in b]
print(f"max radial extent (legs/fins): {max(rr):.2f} m -> diameter ~{2*max(rr):.1f} m")
