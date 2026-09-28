# -*- coding: utf-8 -*-
import json
d=json.load(open(r'D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script\craft_parsed.json',encoding='utf-8'))
for p in d:
    if 'leg' in p['part'] or 'tailfin' in p['part']:
        print(p['part'],'modMass=',p['modMass'],'modules=',[m for m in p['modules'] if 'Scale' in m or 'Wheel' in m or 'Leg' in m or 'Control' in m])
