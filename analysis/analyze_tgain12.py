"""analyze_tgain12.py -- T_GAIN_MIN/MAX 8 -> 12 的实测对比（2026-10-22）

对比两组：
  T=8 : 131055 / 132426 / 224134 / 212521 / 195348
  T=12: 140648 / 150231 / 151716 / 153233

用法：
  python -X utf8 dev/analysis/analyze_tgain12.py
"""
import csv
import io
import math
import os
import re
import statistics
import sys

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
D = os.path.join(_ROOT, 'log-B1040-9-gfold')

OLD = ['starship_log_20261022_131055.csv', 'starship_log_20261022_132426.csv',
       'starship_log_20261021_224134.csv', 'starship_log_20261021_212521.csv',
       'starship_log_20261021_195348.csv']
NEW = ['starship_log_20261022_140648.csv', 'starship_log_20261022_150231.csv',
       'starship_log_20261022_151716.csv', 'starship_log_20261022_153233.csv']


def g(r, k):
    try:
        v = r.get(k)
        return float(v) if v not in (None, '', 'nan') else None
    except (TypeError, ValueError):
        return None


def med(v):
    v = [x for x in v if x is not None]
    return statistics.median(v) if v else float('nan')


def load(f):
    with open(os.path.join(D, f), encoding='utf-8') as fh:
        return list(csv.DictReader([l for l in fh if not l.startswith('#')]))


def notes(f):
    with open(os.path.join(D, f), encoding='utf-8') as fh:
        return [l[1:].strip() for l in fh if l.startswith('#')]


def stats(f):
    rows = load(f)
    if not rows:
        return None
    dd = [g(r, 'dist_hz') for r in rows if g(r, 'dist_hz') is not None]
    tc = [g(r, 'tilt_cmd') for r in rows if g(r, 'tilt_cmd') is not None]
    ae = [g(r, 'att_err') for r in rows if g(r, 'att_err') is not None]
    ahd = [g(r, 'ah_dem') for r in rows if g(r, 'ah_dem') is not None]
    tcr = [g(r, 'tilt_cmd_rate') for r in rows if g(r, 'tilt_cmd_rate') is not None]
    sz = [g(r, 'pid_sat_z') for r in rows if g(r, 'pid_sat_z') is not None]
    caT = [g(r, 'ca_T') for r in rows if g(r, 'ca_T') is not None]
    return dict(f=f[13:27], d0=g(rows[0], 'dist_hz'), vh0=g(rows[0], 'vh'),
                dmax=max(dd), tcmax=max(tc), tcp50=med(tc),
                aemax=max(ae), aep50=med(ae), ahdmax=max(ahd),
                tcrmax=max(tcr),
                szp=100 * sum(1 for x in sz if x > 0.5) / len(sz),
                caT=med(caT))


def lag(f):
    """tilt_cmd -> tilt_act 的互相关最佳滞后（秒）。"""
    rows = load(f)
    pr = [(g(r, 'tilt_cmd'), g(r, 'tilt_act'), g(r, 't')) for r in rows]
    pr = [x for x in pr if None not in x]
    if len(pr) < 10:
        return float('nan')
    tc = [x[0] for x in pr]
    ta = [x[1] for x in pr]
    dt = statistics.median([pr[i + 1][2] - pr[i][2] for i in range(len(pr) - 1)])
    best = (-9, -9.0)
    for L in range(0, 10):
        a = tc[:len(tc) - L] if L else tc
        b = ta[L:] if L else ta
        n = len(a)
        ma, mb = sum(a) / n, sum(b) / n
        cov = sum((i - ma) * (j - mb) for i, j in zip(a, b)) / n
        sa = math.sqrt(sum((i - ma) ** 2 for i in a) / n)
        sb = math.sqrt(sum((j - mb) ** 2 for j in b) / n)
        c = cov / (sa * sb) if sa * sb > 1e-12 else float('nan')
        if c == c and c > best[1]:
            best = (L, c)
    return best[0] * dt


def sec1():
    print('=' * 108)
    print('1. T=8 vs T=12 总体对比')
    print('=' * 108)
    print('  %-8s %6s %6s %7s %7s %7s %7s %7s %7s %6s %6s' % (
        'log', 'dist0', 'vh0', 'distMx', 'tiltMx', 'tilt50', 'attMx', 'att50',
        'ahdMx', 'satZ%', 'caT'))
    for grp in (OLD, NEW):
        acc = []
        for f in grp:
            s = stats(f)
            if not s:
                continue
            acc.append(s)
            print('  %-8s %6.2f %6.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %5.0f%% %6.2f' % (
                s['f'], s['d0'] or 0, s['vh0'] or 0, s['dmax'], s['tcmax'],
                s['tcp50'], s['aemax'], s['aep50'], s['ahdmax'], s['szp'], s['caT']))
        if acc:
            print('  %-8s %6.2f %6.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %5.0f%% %6.2f' % (
                '中位', med([x['d0'] for x in acc]), med([x['vh0'] for x in acc]),
                med([x['dmax'] for x in acc]), med([x['tcmax'] for x in acc]),
                med([x['tcp50'] for x in acc]), med([x['aemax'] for x in acc]),
                med([x['aep50'] for x in acc]), med([x['ahdmax'] for x in acc]),
                med([x['szp'] for x in acc]), med([x['caT'] for x in acc])))
        print()


def sec2():
    print('=' * 108)
    print('2. 落点精度与收敛比')
    print('=' * 108)
    print('  %-8s %8s %8s %9s %10s %10s %s' % (
        'log', 'dist0', 'distL', '收敛比', 'FIN入口vh', 'FIN入口vz', '结果'))
    for grp in (OLD, NEW):
        rats = []
        for f in grp:
            ns = notes(f)
            land = [x for x in ns if x.startswith('LANDED')]
            fin = [x for x in ns if x.startswith('FIN enter')]
            rows = load(f)
            d0 = g(rows[0], 'dist_hz') or 0
            m = re.search(r'dist=([-\d.]+)', land[0]) if land else None
            fvh = fvz = None
            if fin:
                a = re.search(r'vh=([-\d.]+)', fin[0])
                b = re.search(r'vz=([-\d.]+)', fin[0])
                fvh = float(a.group(1)) if a else None
                fvz = float(b.group(1)) if b else None
            if m:
                dL = float(m.group(1))
                rats.append(dL / max(0.1, d0))
                print('  %-8s %8.2f %8.2f %9.3f %10s %10s %s' % (
                    f[13:27], d0, dL, dL / max(0.1, d0),
                    '%.2f' % fvh if fvh is not None else '-',
                    '%.2f' % fvz if fvz is not None else '-', 'LANDED'))
            else:
                print('  %-8s %8.2f %8s %9s %10s %10s %s' % (
                    f[13:27], d0, '-', '-',
                    '%.2f' % fvh if fvh is not None else '-',
                    '%.2f' % fvz if fvz is not None else '-', '未落地'))
        if rats:
            print('  %-8s %8s %8s %9.3f  <- 中位收敛比' % ('中位', '', '', med(rats)))
        print()


def sec3():
    print('=' * 108)
    print('3. 姿态滞后（互相关取峰）')
    print('=' * 108)
    for grp, tag in ((OLD, 'T=8'), (NEW, 'T=12')):
        for f in grp:
            print('  %-8s %-6s 最佳滞后 %.2f s' % (f[13:27], tag, lag(f)))
        print()


def main():
    sec1()
    sec2()
    sec3()


if __name__ == '__main__':
    main()
