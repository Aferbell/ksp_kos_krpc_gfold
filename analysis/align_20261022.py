import csv, io, os, sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
d = r'D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script\log-B1040-9-gfold'

PAIRS = [
    ('starship_log_20261022_140648.csv', 15.60),
    ('starship_log_20261022_150231.csv', 26.72),
    ('starship_log_20261022_153233.csv', 14.74),
]
print('=== 起振窗口换算成视频时间 ===')
print('  %-10s %-10s %-10s %-10s' % ('log','起振 t=1~5s','姿态峰 t','落地 t'))
for f, off in PAIRS:
    rows = list(csv.DictReader([l for l in open(os.path.join(d, f), encoding='utf-8') if not l.startswith('#')]))
    if not rows:
        continue
    def g(r, k):
        try:
            v = r.get(k)
            return float(v) if v not in (None,'','nan') else None
        except: return None
    tc = [(g(r,'t'), g(r,'tilt_cmd')) for r in rows if g(r,'t') is not None and g(r,'tilt_cmd') is not None]
    pk = max(tc, key=lambda x: x[1])
    tl = float(rows[-1]['t'])
    print('  %-10s %5.2f~%5.2f s (视频 %5.2f~%5.2f)  峰@%.2f(视频%.2f)  末%.2f(视频%.2f)' % (
        f[13:27], 1.0, 5.0, off+1.0, off+5.0, pk[0], off+pk[0], tl, off+tl))
