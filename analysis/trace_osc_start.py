import csv, io, os, sys, math, statistics
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
d = r'D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script\log-B1040-9-gfold'

def g(r, k):
    try:
        v = r.get(k)
        return float(v) if v not in (None, '', 'nan') else None
    except (TypeError, ValueError):
        return None

def corr(a, b):
    n = len(a)
    if n < 3:
        return float('nan')
    ma, mb = sum(a)/n, sum(b)/n
    cov = sum((i-ma)*(j-mb) for i, j in zip(a, b))/n
    sa = math.sqrt(sum((i-ma)**2 for i in a)/n)
    sb = math.sqrt(sum((j-mb)**2 for j in b)/n)
    return cov/(sa*sb) if sa*sb > 1e-12 else float('nan')

print('=== 测 tilt_cmd -> tilt_act 的【纯滞后】（互相关取峰）===')
for f, tag in (('starship_log_20261022_131055.csv','131055'),
               ('starship_log_20261022_132426.csv','132426')):
    rows = list(csv.DictReader([l for l in open(os.path.join(d, f), encoding='utf-8') if not l.startswith('#')]))
    tc = [g(r,'tilt_cmd') for r in rows]
    ta = [g(r,'tilt_act') for r in rows]
    tt = [g(r,'t') for r in rows]
    pairs = [(a, b, t) for a, b, t in zip(tc, ta, tt) if None not in (a, b, t)]
    if len(pairs) < 10:
        continue
    tc = [x[0] for x in pairs]
    ta = [x[1] for x in pairs]
    print('  --- %s (%d 帧) ---' % (tag, len(pairs)))
    best = (-9, 0)
    for lag in range(0, 12):
        a = tc[:len(tc)-lag] if lag else tc
        b = ta[lag:] if lag else ta
        c = corr(a, b)
        if c == c and c > best[1]:
            best = (lag, c)
        # 估时间
        dt = statistics.median([pairs[i+1][2]-pairs[i][2] for i in range(len(pairs)-1)]) if len(pairs)>1 else 0
        if lag < 8:
            print('    lag=%d 帧 (%4.2f s)  corr=%+.3f' % (lag, lag*dt, c))
    print('    => 最佳滞后 %d 帧 = %.2f s' % (best[0], best[0]*dt))
    print()
