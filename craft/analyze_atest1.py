# analyze_atest1.py -- 分析 atest1.craft（着陆器）的载具性质
#
# 【只读】本脚本只读取 .craft 并打印，不修改任何文件。
# 目的：在设计 Mun 非定点着陆程序之前，先弄清载具的真实参数：
#   · 发动机：型号、推力上限、Isp、万向节范围、级号
#   · RCS：型号、数量、可用推力、位置（相对 CoM 的力臂）
#   · 推进剂：种类、总量（决定 dv 预算）
#   · 质量：干重/湿重（决定 TWR）
#   · 起落架/腿：数量与位置
#   · 是否有 kOS 终端（用户说暂时没装）
#   · CoM 位置估计（用于估算 RCS 力臂与姿态权限）
import collections
import io
import json
import os
import re
import sys

CRAFT = (r"D:\SteamLibrary\steamapps\common\Kerbal Space Program"
         r"\saves\起源\Ships\VAB\atest1.craft")


def split_parts(text):
    """把 .craft 按顶层 PART 块切开。"""
    parts = []
    idx = 0
    while True:
        m = re.search(r'^PART\s*$', text[idx:], re.M)
        if not m:
            break
        start = idx + m.end()
        m2 = re.search(r'^\{\s*$', text[start:], re.M)
        if not m2:
            break
        i = start + m2.start()
        depth = 0
        block_lines = []
        for ln in text[i:].split('\n'):
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
    return parts


def fnum(m, key, block, default=0.0):
    mm = re.search(r'^\s*%s = ([-\d.Ee+]+)' % key, block, re.M)
    return float(mm.group(1)) if mm else default


def parse(block):
    p = {}
    m = re.search(r'^\s*part = (\S+)', block, re.M)
    p['part'] = m.group(1) if m else '?'
    m = re.search(r'^\s*pos = ([-\d.Ee]+),([-\d.Ee]+),([-\d.Ee]+)', block,
                  re.M)
    p['pos'] = ([float(m.group(1)), float(m.group(2)), float(m.group(3))]
                if m else [0.0, 0.0, 0.0])
    p['istg'] = int(fnum(None, 'istg', block, -99))
    p['dstg'] = int(fnum(None, 'dstg', block, -99))
    p['modMass'] = fnum(None, 'modMass', block, 0.0)
    p['scale'] = fnum(None, 'scale', block, 1.0)
    # MODULE 块
    mods = []
    for mm in re.finditer(r'MODULE\s*\{(.*?)\n\t\}', block, re.S):
        mb = mm.group(1)
        nm = re.search(r'name = (\S+)', mb)
        mods.append((nm.group(1) if nm else '?', mb))
    p['modules'] = mods
    # RESOURCE 块
    res = []
    for rm in re.finditer(r'RESOURCE\s*\{(.*?)\n\t\}', block, re.S):
        rb = rm.group(1)
        nm = re.search(r'name = (\S+)', rb)
        am = re.search(r'amount = ([-\d.Ee+]+)', rb)
        mx = re.search(r'maxAmount = ([-\d.Ee+]+)', rb)
        res.append((nm.group(1) if nm else '?',
                    float(am.group(1)) if am else 0.0,
                    float(mx.group(1)) if mx else 0.0))
    p['resources'] = res
    return p


def modkey(block, name, key, default=None):
    for nm, mb in [(None, block)]:
        pass
    return default


def get_mod(part, modname):
    for nm, mb in part['modules']:
        if nm == modname:
            return mb
    return None


def main():
    text = io.open(CRAFT, encoding='utf-8', errors='replace').read()
    blocks = split_parts(text)
    ps = [parse(b) for b in blocks]

    print('# atest1.craft 载具分析')
    print('')
    print('  文件: %s' % os.path.basename(CRAFT))
    print('  部件总数: %d' % len(ps))
    print('  ship 名: %s'
          % (re.search(r'^ship = (.+)$', text, re.M).group(1).strip()))
    print('  KSP 版本: %s'
          % (re.search(r'^version = (.+)$', text, re.M).group(1).strip()))
    print('')

    # ---- 部件统计 ----
    cnt = collections.Counter(p['part'] for p in ps)
    print('=== 部件清单（按出现次数）===')
    for k, v in sorted(cnt.items(), key=lambda x: (-x[1], x[0])):
        print('  %4d  %s' % (v, k))
    print('')

    # ---- kOS 终端？ ----
    print('=== kOS 终端检查 ===')
    kos = [p for p in ps if any('kOS' in nm for nm, _ in p['modules'])]
    if kos:
        for p in kos:
            print('  发现 kOS 部件: %s  pos=%s' % (p['part'], p['pos']))
    else:
        print('  未发现 kOS 部件（与用户说明一致：暂时没装 kOS 终端）')
    print('')

    # ---- 发动机 ----
    print('=== 发动机 ===')
    engines = []
    for p in ps:
        for nm, mb in p['modules']:
            if nm in ('ModuleEnginesFX', 'ModuleEngines', 'ModuleEnginesRF'):
                mx = re.search(r'maxThrust = ([-\d.Ee+]+)', mb)
                isp = re.findall(r'key = ([\d.]+) ([\d.]+)', mb)
                eng = re.search(r'engineID = (\S+)', mb)
                tp = re.search(r'thrustPercentage = ([-\d.Ee+]+)', mb)
                name = re.search(r'name = (\S+)', mb)
                engines.append(dict(
                    part=p['part'], pos=p['pos'], istg=p['istg'],
                    dstg=p['dstg'], module=nm,
                    maxThrust=float(mx.group(1)) if mx else 0.0,
                    isp=isp[:4], thrustPct=float(tp.group(1)) if tp else 100.0,
                    modmass=p['modMass'], scale=p['scale']))
                print('  %-38s pos=%-28s istg=%-3d maxThrust=%8.1f kN '
                      'thr%%=%5.1f' % (p['part'], p['pos'], p['istg'],
                                       float(mx.group(1)) if mx else 0.0,
                                       float(tp.group(1)) if tp else 100.0))
                if isp:
                    print('        Isp 曲线: %s' % isp[:4])
    print('')

    # ---- 万向节 ----
    print('=== 万向节 (ModuleGimbal) ===')
    for p in ps:
        g = get_mod(p, 'ModuleGimbal')
        if g:
            gr = re.search(r'gimbalRange = ([-\d.Ee+]+)', g)
            gs = re.search(r'gimbalResponseSpeed = ([-\d.Ee+]+)', g)
            ga = re.search(r'gimbalActive = (\w+)', g)
            print('  %-38s gimbalRange=%s  respSpeed=%s  active=%s'
                  % (p['part'], gr.group(1) if gr else '?',
                     gs.group(1) if gs else '?', ga.group(1) if ga else '?'))
    print('')

    # ---- RCS ----
    print('=== RCS ===')
    nrcs = 0
    for p in ps:
        for nm, mb in p['modules']:
            if 'RCS' in nm:
                nrcs += 1
                th = re.search(r'thrustPercentage = ([-\d.Ee+]+)', mb)
                ft = re.search(r'fullThrust = (\w+)', mb)
                print('  %-38s pos=%-28s module=%s thr%%=%s fullThrust=%s'
                      % (p['part'], p['pos'], nm,
                         th.group(1) if th else '?',
                         ft.group(1) if ft else '?'))
    print('  RCS 模块总数: %d' % nrcs)
    print('')

    # ---- 起落架/腿 ----
    print('=== 起落架 / 着陆腿 ===')
    for p in ps:
        for nm, mb in p['modules']:
            if 'LandingLeg' in nm or 'LandingGear' in nm or 'Wheel' in nm:
                print('  %-38s pos=%-28s module=%s' % (p['part'], p['pos'], nm))
    print('')

    # ---- 推进剂 ----
    print('=== 资源总量 ===')
    tot = collections.defaultdict(lambda: [0.0, 0.0])
    for p in ps:
        for nm, a, mx in p['resources']:
            tot[nm][0] += a
            tot[nm][1] += mx
    for k, (a, m) in sorted(tot.items()):
        print('  %-22s amount=%10.2f  max=%10.2f' % (k, a, m))
    print('')

    # ---- 质量 ----
    print('=== 质量 ===')
    modmass = sum(p['modMass'] for p in ps)
    print('  modMass 合计（部件空重）: %.3f t' % modmass)
    print('  （注意：modMass 不含推进剂；KSP 的 part.mass 在飞行中会加上燃料）')
    print('')

    # ---- 分级 ----
    istgs = collections.Counter(p['istg'] for p in ps)
    print('=== istg 分布 ===')
    for k in sorted(istgs):
        print('  istg=%3d : %d 个部件' % (k, istgs[k]))
    print('')

    # ---- 位置分布（用于估计 CoM 与尺寸）----
    print('=== 位置范围（用于估计尺寸与 CoM）===')
    xs = [p['pos'][0] for p in ps]
    ys = [p['pos'][1] for p in ps]
    zs = [p['pos'][2] for p in ps]
    print('  x: %.2f .. %.2f  (跨度 %.2f m)' % (min(xs), max(xs),
                                              max(xs) - min(xs)))
    print('  y: %.2f .. %.2f  (跨度 %.2f m)' % (min(ys), max(ys),
                                              max(ys) - min(ys)))
    print('  z: %.2f .. %.2f  (跨度 %.2f m)' % (min(zs), max(zs),
                                              max(zs) - min(zs)))
    #   质量加权中心（粗略：用 modMass 当权重，不含燃料）
    tw = sum(p['modMass'] for p in ps) or 1.0
    cx = sum(p['modMass'] * p['pos'][0] for p in ps) / tw
    cy = sum(p['modMass'] * p['pos'][1] for p in ps) / tw
    cz = sum(p['modMass'] * p['pos'][2] for p in ps) / tw
    print('  空重加权中心 (modMass): (%.2f, %.2f, %.2f)' % (cx, cy, cz))
    print('  【注意】这是空重加权，不是真实 CoM（真实 CoM 含燃料，需游戏内读）')
    print('')

    # ---- 保存 JSON 供后续使用 ----
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       '_atest1_parsed.json')
    with io.open(out, 'w', encoding='utf-8') as f:
        f.write(json.dumps([{k: v for k, v in p.items() if k != 'modules'}
                            for p in ps], indent=1, ensure_ascii=False))
    print('（部件明细已存 %s）' % os.path.basename(out))
    return 0


if __name__ == '__main__':
    sys.exit(main())
