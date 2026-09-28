# -*- coding: utf-8 -*-
import json
d = json.load(open(r'D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script\craft_parsed.json', encoding='utf-8'))
for p in d:
    if any(k in p['part'] for k in ('Separator','decoupl','SSME','corgi','RE-J10','leg','tailfin','drone-core','probeStack','KAL9000','fueltank','TPtank','adapter','Magnetometer','Antenna','solar','truss','fairing','battery','mono')):
        print(f"{p['part']:45s} y={p['pos'][1]:8.3f} x={p['pos'][0]:8.3f} z={p['pos'][2]:8.3f} istg={p['istg']} dstg={p['dstg']} sepI={p['sepI']}")
