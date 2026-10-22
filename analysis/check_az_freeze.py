import csv, io, os, sys, math
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
d = r'D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script\log-B1040-9-gfold'

def g(r, k):
    try:
        v = r.get(k)
        return float(v) if v not in (None, '', 'nan') else None
    except (TypeError, ValueError):
        return None

print('=== 最终回放：AZ_JUMP_MAX_DEG=60 对 195348 的实际效果 ===')
print()
rows = list(csv.DictReader([l for l in open(os.path.join(d, 'starship_log_20261021_195348.csv'), encoding='utf-8') if not l.startswith('#')]))

def run(thr_az, max_jump):
    tf_az = None
    ang = 0.0
    prev = None
    flips = 0
    updates = 0
    rejected = 0
    big_az = 0
    for r in rows:
        t = g(r, 't')
        if t is None:
            continue
        perr = g(r, 'tf_perr')
        ahd = g(r, 'ah_dem')
        az_n, az_e = g(r, 'tf_az_n'), g(r, 'tf_az_e')
        dn, de = g(r, 'dp_n'), g(r, 'dp_e')
        if None in (perr, ahd, dn, de):
            continue
        if tf_az is None:
            n = math.hypot(dn, de)
            tf_az = (-dn / n, -de / n) if n > 1e-9 else (0.0, 0.0)
        if ahd > 1e-9 and perr > thr_az and az_n is not None and az_e is not None:
            cand = (az_n, az_e)
            acc = True
            if tf_az is not None and max_jump < 180.0:
                cn = math.hypot(*tf_az)
                nn = math.hypot(*cand)
                if cn > 1e-9 and nn > 1e-9:
                    dot = (tf_az[0]*cand[0] + tf_az[1]*cand[1]) / (cn*nn)
                    j = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
                    if j > max_jump:
                        acc = False
                        rejected += 1
            if acc:
                tf_az = cand
                updates += 1
        a = math.degrees(math.atan2(tf_az[1], tf_az[0]))
        if prev is not None:
            dd = ((a - prev + 180) % 360) - 180
            ang += dd
            if abs(dd) > 90:
                flips += 1
        prev = a
        # 估算 az_err
        az_real = g(r, 'az_err')
        if az_real is not None and abs(az_real) > 150:
            big_az += 1
    return ang, flips, updates, rejected, big_az

print('  %-28s %-12s %-8s %-9s %-9s %s' % (
    '配置', '方位累计', '翻转', '更新帧', '拒绝帧', '|az_err|>150'))
for tag, thr, mj in (('改前（无跳变拒绝）', 1.5, 180.0),
                     ('改后（拒绝 >60deg）', 1.5, 60.0)):
    ang, fl, up, rj, big = run(thr, mj)
    print('  %-28s %-12.0f %-8d %-9d %-9d %d' % (tag, ang, fl, up, rj, big))
print()
print('  【注意】|az_err|>150 那列是【日志里的实测值】，回放无法直接预测新值。')
print('  上面的累计转角/翻转次数是【参考系方位】的统计，反映命令侧行为。')
print('  实际 az_err 需要实飞验证。')
