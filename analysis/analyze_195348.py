import csv, io, os, sys, math, statistics
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
d = r'D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script\log-B1040-9-gfold'
def g(r, k):
    try:
        v = r.get(k)
        return float(v) if v not in (None, '', 'nan') else None
    except (TypeError, ValueError):
        return None
def med(v):
    v = [x for x in v if x is not None]
    return statistics.median(v) if v else float('nan')

rows = list(csv.DictReader([l for l in open(os.path.join(d,'starship_log_20261021_195348.csv'),encoding='utf-8') if not l.startswith('#')]))

print('=== att_err 的来源分解：倾角滞后 vs 方位错 ===')
print('  %-6s %-8s %-7s %-7s %-9s %-9s %-6s %s' % (
    't','attErr','tiltC','tiltA','若方位对','方位贡献','占比','az_err'))
for r in rows:
    t, ae = g(r,'t'), g(r,'att_err')
    tc, ta, az = g(r,'tilt_cmd'), g(r,'tilt_act'), g(r,'az_err')
    if None in (t, ae, tc, ta, az) or ae < 4.0:
        continue
    base = abs(ta - tc)
    contrib = math.sqrt(max(0.0, ae*ae - base*base))
    print('  %-6.2f %-8.2f %-7.2f %-7.2f %-9.2f %-9.2f %5.0f%% %8.1f' % (
        t, ae, tc, ta, base, contrib, 100*contrib/max(1e-6,ae), az))
print()
print('  若方位贡献占比高 => 是方位角在跳，不是倾角跟不上')
print('  若「若方位对」占比高 => 是倾角大小没跟上')
