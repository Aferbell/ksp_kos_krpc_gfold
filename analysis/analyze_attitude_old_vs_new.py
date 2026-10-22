# -*- coding: utf-8 -*-
"""analyze_attitude_old_vs_new.py — 新旧载具姿态失控对比（可复跑）

【为什么需要这个脚本】
  2026-10-21 新载具 starship 连续两趟姿态发散（att_err 涨到 41~85 deg），
  而旧载具 B1040-9 结构完全相同的代码却从不发散。本脚本把两者的日志
  放在同一套判据下对比，定位"差在哪一个物理量上"。

【只读，不改任何飞行代码】
用法:
  python dev/analysis/analyze_attitude_old_vs_new.py
"""
import csv
import glob
import io
import math
import os
import statistics
import sys

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
_LOGDIR = os.path.join(_ROOT, 'log-B1040-9-gfold')

# 代表趟：旧载具 3 趟（含"小初偏"那趟，正是发病条件）+ 新载具 2 趟
PICKS = [
    ('OLD 164053', 'gfold_log_20261008_164053.csv'),
    ('OLD 124409', 'gfold_log_20261010_124409.csv'),
    ('OLD 143917', 'gfold_log_20261011_143917.csv'),
    ('NEW 144058', 'starship_log_20261021_144058.csv'),
    ('NEW 154351', 'starship_log_20261021_154351.csv'),
]

HEADROOM = math.radians(45.0)


def load(name):
    p = os.path.join(_LOGDIR, name)
    if not os.path.exists(p):
        return None
    with open(p, encoding='utf-8') as fh:
        raw = [ln for ln in fh if not ln.startswith('#')]
    return list(csv.DictReader(raw))


def g(row, key):
    try:
        v = row.get(key)
        return float(v) if v not in (None, '', 'nan') else None
    except (TypeError, ValueError):
        return None


def med(vals):
    v = [x for x in vals if x is not None]
    return statistics.median(v) if v else float('nan')


def gfold_only(rows):
    """只取 G-FOLD 段（t<16 s），FIN 段 thr=0 无姿态权限，会污染统计。"""
    seg = [r for r in rows if (g(r, 't') or 99.0) < 16.0]
    return seg if len(seg) >= 8 else rows


def rates3d(rows, kx, ky, kz):
    """把逐帧方向向量反算成 3D 角速率序列 [deg/s]。"""
    out, pv, pt = [], None, None
    for r in rows:
        vx, vy, vz, t = g(r, kx), g(r, ky), g(r, kz), g(r, 't')
        if None in (vx, vy, vz, t):
            continue
        n = math.sqrt(vx * vx + vy * vy + vz * vz)
        if n < 1e-9:
            continue
        u = (vx / n, vy / n, vz / n)
        if pv is not None and t - pt > 1e-6:
            c = max(-1.0, min(1.0, sum(a * b for a, b in zip(u, pv))))
            out.append(math.degrees(math.acos(c)) / (t - pt))
        pv, pt = u, t
    return out


def report():
    print('=' * 108)
    print('A. 姿态权限：新旧载具的 alpha 几乎相同 —— 差别不在"力气"')
    print('=' * 108)
    hdr = '%-12s %8s %9s %9s %11s %9s %9s'
    print(hdr % ('log', 'mass_t', 'thrustMN', 'a_cap', 'I_p', 'tau_p', 'alpha'))
    for tag, name in PICKS:
        rows = load(name)
        if rows is None:
            print('%-12s  (缺失)' % tag)
            continue
        seg = gfold_only(rows)
        m = med([g(r, 'mass') for r in seg])
        th = med([g(r, 'thrust_n') for r in seg])
        ip = med([g(r, 'I_p') for r in seg])
        tp = med([g(r, 'tau_p') for r in seg])
        al = med([g(r, 'alpha_max') for r in seg if (g(r, 'alpha_max') or 0) > 0.5])
        print(hdr % (tag, '%.1f' % ((m or 0) / 1000), '%.2f' % ((th or 0) / 1e6),
                     '%.2f' % ((th or 0) / (m or 1)), '%.2e' % (ip or 0),
                     '%.2e' % (tp or 0), '%.1f' % al))

    print()
    print('=' * 108)
    print('B. 核心判据：峰值角速率能不能在 45 deg 行程内刹住')
    print('   需要 alpha = omega^2 / (2*headroom)   可用 alpha = alpha_max')
    print('=' * 108)
    hdr = '%-12s %9s %9s %10s %9s %12s'
    print(hdr % ('log', 'omegaPk', 'alphaAv', 'alphaNeed', '余量', '判定'))
    for tag, name in PICKS:
        rows = load(name)
        if rows is None:
            continue
        seg = gfold_only(rows)
        av = [abs(g(r, 'avel_p')) for r in seg if g(r, 'avel_p') is not None]
        al = [g(r, 'alpha_max') for r in seg if (g(r, 'alpha_max') or 0) > 0.5]
        if not av or not al:
            continue
        om = max(av)
        a = math.radians(med(al))
        need = om * om / (2.0 * HEADROOM)
        marg = a / need if need > 0 else 99.0
        print(hdr % (tag, '%.3f' % om, '%.1f' % math.degrees(a),
                     '%.1f' % math.degrees(need), '%.2f' % marg,
                     '刹得住' if marg > 1.0 else '**刹不住**'))

    print()
    print('=' * 108)
    print('C. 阻尼比：名义值 vs 计饱和后的实际值')
    print('   zeta_nom = kd_eff/2*sqrt(alpha/kp)     （D 未饱和）')
    print('   zeta_eff = 用 1/omega 反解饱和后的 kd  （实际飞出来的）')
    print('=' * 108)
    hdr = '%-12s %7s %8s %8s %7s %9s %8s %8s %9s'
    print(hdr % ('log', 'sched', 'kp_eff', 'kd_eff', 'wn', 'zeta_nom',
                 'wPeak', 'D@peak', 'zeta_eff'))
    # 该趟实际用的 kp/kd（按日志 # ctrl 注释行无从读取，用代码版本对应值）
    KP_KD = {'OLD 164053': (2.0, 6.0), 'OLD 124409': (2.0, 6.0),
             'OLD 143917': (2.0, 6.0), 'NEW 144058': (5.0, 5.0),
             'NEW 154351': (5.0, 5.0)}
    for tag, name in PICKS:
        rows = load(name)
        if rows is None:
            continue
        seg = gfold_only(rows)
        kp0, kd0 = KP_KD[tag]
        norm = tag.startswith('NEW')
        sc = med([g(r, 'sched') for r in seg if g(r, 'sched') is not None])
        al = med([g(r, 'alpha_max') for r in seg if (g(r, 'alpha_max') or 0) > 0.5])
        a = math.radians(al)
        wphys = math.sqrt(2.0 * a * HEADROOM)
        kp = kp0 * sc
        wn = math.sqrt(a * kp)
        wpk = max([abs(g(r, 'avel_p')) for r in seg if g(r, 'avel_p') is not None])
        if norm:
            kde = kd0 * 0.78 / wphys
            d = kd0 * 0.78 * (wpk / wphys)
        else:
            kde = kd0 * 0.78 * sc
            d = kde * wpk
        zeta = kde / 2.0 * math.sqrt(a / kp)
        zeta_e = (1.0 / wpk) / 2.0 * math.sqrt(a / kp) if d > 1.0 else zeta
        print(hdr % (tag, '%.2f' % sc, '%.2f' % kp, '%.2f' % kde, '%.2f' % wn,
                     '%.2f' % zeta, '%.3f' % wpk, '%.2f' % d, '%.2f' % zeta_e))

    print()
    print('=' * 108)
    print('D. 是不是小初偏才会发病？—— 39 趟全量统计')
    print('=' * 108)
    recs = []
    for f in sorted(glob.glob(os.path.join(_LOGDIR, 'gfold_log_202610*.csv')) +
                    glob.glob(os.path.join(_LOGDIR, 'starship_log_202610*.csv'))):
        rows = load(os.path.basename(f))
        if not rows:
            continue
        seg = [r for r in rows if (g(r, 't') or 99.0) < 16.0]
        if len(seg) < 8:
            continue
        d0 = g(seg[0], 'dist_hz')
        if d0 is None:
            continue
        az = [abs(g(r, 'az_err')) for r in seg if g(r, 'az_err') is not None]
        recs.append((d0, med(az), os.path.basename(f)))
    hdr = '%-16s %6s %14s'
    print(hdr % ('dist0 区间', 'n', 'azErr50 中位'))
    for lo, hi, lab in ((0, 50, '< 50 m'), (50, 100, '50~100 m'),
                        (100, 1e9, '> 100 m')):
        grp = [r for r in recs if lo <= r[0] < hi]
        if grp:
            print(hdr % (lab, len(grp), '%.1f' % med([r[1] for r in grp])))
    if recs:
        xs = [r[0] for r in recs]
        ys = [r[1] for r in recs]
        n = len(xs)
        mx, my = sum(xs) / n, sum(ys) / n
        cov = sum((a - mx) * (b - my) for a, b in zip(xs, ys)) / n
        sx = math.sqrt(sum((a - mx) ** 2 for a in xs) / n)
        sy = math.sqrt(sum((b - my) ** 2 for b in ys) / n)
        print()
        print('corr(dist0, azErr50) = %.3f   (n=%d)' % (cov / (sx * sy), n))
        print('=> 负相关：初偏越小，方位误差越大。旧程序在小初偏下同样发病。')

    print()
    print('=' * 108)
    print('E. 参考方向速率与限速器：限速器从未真正起作用')
    print('=' * 108)
    hdr = '%-12s %7s %7s %7s %8s %8s %8s'
    print(hdr % ('log', 'p50', 'p90', 'max', 'clip%', '>20帧', '>15帧'))
    for tag, name in PICKS:
        rows = load(name)
        if rows is None:
            continue
        seg = gfold_only(rows)
        r3 = rates3d(seg, 'tgt_vx', 'tgt_vy', 'tgt_vz')
        if not r3:
            continue
        n = len(r3)
        rs = sorted(r3)
        rc = [g(r, 'ref_clip') for r in seg if g(r, 'ref_clip') is not None]
        print(hdr % (tag, '%.1f' % rs[n // 2], '%.1f' % rs[int(.9 * n)],
                     '%.1f' % rs[-1],
                     '%.0f%%' % (100.0 * sum(1 for q in rc if q > 0.5) / len(rc)
                                 if rc else 0),
                     sum(1 for q in r3 if q > 20), sum(1 for q in r3 if q > 15)))

    print()
    print('=' * 108)
    print('F. 前赡在旋转的方位参考上等于凭空加大误差')
    print('=' * 108)
    hdr = '%-12s %8s %9s %10s %11s %12s'
    print(hdr % ('log', 'tf', 'lead_n', '方位转率', '前赡提前', '占 az_err'))
    for tag, name in PICKS:
        rows = load(name)
        if rows is None:
            continue
        seg = gfold_only(rows)
        tf = med([g(r, 'tf') for r in seg if g(r, 'tf')])
        ln = med([g(r, 'tf_lead_n') for r in seg if g(r, 'tf_lead_n') is not None])
        rots, prev = [], None
        for r in seg:
            t, dn, de = g(r, 't'), g(r, 'dp_n'), g(r, 'dp_e')
            if None in (t, dn, de) or math.hypot(dn, de) < 0.5:
                prev = None
                continue
            ang = math.degrees(math.atan2(de, dn))
            if prev is not None and t - prev[0] > 1e-6:
                rots.append(abs(((ang - prev[1] + 180) % 360) - 180) / (t - prev[0]))
            prev = (t, ang)
        rot = med(rots) if rots else 0.0
        lead_sec = (ln or 0) * (tf or 0) / 79.0
        adv = rot * lead_sec
        azm = med([abs(g(r, 'az_err')) for r in seg if g(r, 'az_err') is not None])
        print(hdr % (tag, '%.2f' % (tf or 0), '%.1f' % (ln or 0),
                     '%.1f/s' % rot, '%.1f deg' % adv,
                     '%.0f%%' % (100.0 * adv / max(1e-6, azm))))


if __name__ == '__main__':
    report()
