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

rows = list(csv.DictReader([l for l in open(os.path.join(d, 'starship_log_20261021_205641.csv'), encoding='utf-8') if not l.startswith('#')]))

print('=== 结论：C 修正把方位角永久冻结，位置跟踪彻底崩掉 ===')
print()
rej = [(g(r, 't'), g(r, 'az_reject'), g(r, 'az_jump'), g(r, 'dist_hz'), g(r, 'vh'))
       for r in rows if g(r, 'az_reject') is not None]
print('  az_reject=1 的帧: %d/%d = %.0f%%' % (
    sum(1 for x in rej if x[1] > 0.5), len(rej),
    100*sum(1 for x in rej if x[1] > 0.5)/len(rej)))
print('  => 方位角被冻结在【同一个错误方向】上，永远不会更新')
print()

print('  %-7s %-8s %-9s %-9s %-8s %-8s' % ('t', 'alt', 'dist_hz', 'vh', 'az_rej', 'tilt_cmd'))
for r in rows[::8]:
    t = g(r, 't')
    if t is None:
        continue
    print('  %-7.2f %-8.1f %-9.2f %-9.2f %-8.0f %-8.2f' % (
        t, g(r, 'alt') or 0, g(r, 'dist_hz') or 0, g(r, 'vh') or 0,
        g(r, 'az_reject') or 0, g(r, 'tilt_cmd') or 0))

print()
print('=== 三趟关键指标对比 ===')
print('  %-14s %10s %10s %10s %10s %10s' % ('log', 'vh_max', 'dist_max', 'attE50', 'FIN入口vh', '结果'))
fmt = [
    ('195348', 'starship_log_20261021_195348.csv', '触地 vz=-1.08'),
    ('205641', 'starship_log_20261021_205641.csv', 'FIN timeout'),
]
for tag, ff, res in fmt:
    rr = list(csv.DictReader([l for l in open(os.path.join(d, ff), encoding='utf-8') if not l.startswith('#')]))
    vh = [g(r, 'vh') for r in rr if g(r, 'vh') is not None]
    ds = [g(r, 'dist_hz') for r in rr if g(r, 'dist_hz') is not None]
    ae = [g(r, 'att_err') for r in rr if g(r, 'att_err') is not None]
    notes = [l for l in open(os.path.join(d, ff), encoding='utf-8') if 'FIN enter' in l]
    fin_vh = 'nan'
    if notes:
        import re
        m = re.search(r'vh=([-\d.]+)', notes[0])
        if m:
            fin_vh = m.group(1)
    print('  %-14s %10.1f %10.1f %10.1f %10s %s' % (tag, max(vh), max(ds), med(ae), fin_vh, res))
