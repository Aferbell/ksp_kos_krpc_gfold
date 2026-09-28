# eguidance_validate.py — 段1（E-guidance 主段）验证：用真实日志界定其任务
#
# 段1 的任务【不是】自己着陆，而是把火箭从点火状态带到 G-FOLD 的可行域入口。
# 已由 gfold 侧的验证确定：G-FOLD 可行域 = alt 358~5950 m（真实状态）。
# 所以段1 只需把火箭送到"alt ≈ 5~6 km、锥内、|v| 已降到 G-FOLD 可接"。
#
# 本脚本用真实日志回答三个问题：
#   Q1 真实飞行在 alt=5950 时是什么状态？(= G-FOLD 可行域上沿)
#   Q2 从点火到那里的能量/时间预算是多少？
#   Q3 E-guidance 需要的"回锥"目标角是多少？（真实轨迹的位置矢量角）
import csv
import os
import numpy as np

LOGDIR = r'D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script\log-B1040-9'
TGT_ALT = 36.8
G0 = 9.80665
TMAX = 12.8e6

with open(os.path.join(LOGDIR, 'land_log_47822691.csv'), 'r', encoding='utf-8-sig') as f:
    rows = list(csv.DictReader(f))


def g(r, k, d=0.0):
    try:
        return float(r[k])
    except (KeyError, ValueError):
        return d


ign = next(i for i, r in enumerate(rows) if g(r, 'phase') >= 1)
t0 = g(rows[ign], 't')


def profile():
    out = []
    for r in rows[ign:]:
        out.append(dict(t=g(r, 't') - t0, alt=g(r, 'alt'), dist=g(r, 'dist_hz'),
                        vz=g(r, 'vz'), vh=g(r, 'vh'), m=g(r, 'mass'),
                        phase=int(g(r, 'phase'))))
    return out


P = profile()
i0 = P[0]

print('=' * 78)
print('Q1. 点火状态 与 G-FOLD 可行域上沿（alt≈5950）的状态')
print('=' * 78)


def at(alt):
    return min(P, key=lambda p: abs(p['alt'] - alt))


for a in [i0['alt'], 5950, 2000, 400]:
    p = at(a)
    an = TMAX / (p['m'] * 1000) - G0
    print('  alt=%6.0f  t=+%5.1f s  dist=%6.0f  vz=%7.1f  vh=%6.1f  |v|=%6.1f'
          '  m=%5.1ft  a_net=%5.1f'
          % (p['alt'], p['t'], p['dist'], p['vz'], p['vh'],
             np.hypot(p['vz'], p['vh']), p['m'], an))

print()
print('=' * 78)
print('Q2. 段1 的能量/时间预算（点火 -> alt 5950）')
print('=' * 78)
p_hi = at(5950)
dv = np.hypot(i0['vz'], i0['vh']) - np.hypot(p_hi['vz'], p_hi['vh'])
dm = i0['m'] - p_hi['m']
print('  点火   |v|=%.1f  高度 %.0f m  质量 %.1f t'
      % (np.hypot(i0['vz'], i0['vh']), i0['alt'], i0['m']))
print('  交班点 |v|=%.1f  高度 %.0f m  质量 %.1f t'
      % (np.hypot(p_hi['vz'], p_hi['vh']), p_hi['alt'], p_hi['m']))
print('  段1 需消速度 Δv = %.1f m/s   耗燃料 = %.1f t   真实耗时 = %.1f s'
      % (dv, dm, p_hi['t']))
print('  => 段1 平均减速率 = %.1f m/s²（a_net 在该段约 %.1f~%.1f）'
      % (dv / p_hi['t'], TMAX / (i0['m'] * 1000) - G0, TMAX / (p_hi['m'] * 1000) - G0))
print('  => 真实平均减速率 / a_net = %.2f  （<1 说明没打满推力，靠 FAR 阻力补）'
      % ((dv / p_hi['t']) / (TMAX / ((i0['m'] + p_hi['m']) / 2 * 1000) - G0)))

print()
print('=' * 78)
print('Q3. 段1 的回锥目标角（G-FOLD 的滑翔角锥语义）')
print('=' * 78)
print('  约束: dist <= (1/tan(gs))·alt  ；gs 从【水平】量起，越大越陡越紧')
print('  真实轨迹的位置矢量角 = atan(dist/alt)，从竖直量起：')
print('    alt      dist    dist/alt  离竖直角  所需的 gs 上界(=atan(1/(d/a)))')
for a in [12358, 8000, 5950, 4000, 2000, 1000, 358]:
    p = at(a)
    if p['alt'] <= 0:
        continue
    r_ = p['dist'] / p['alt']
    print('   %6.0f %7.0f %9.3f %8.1f° %10.1f°'
          % (p['alt'], p['dist'], r_, np.degrees(np.arctan(r_)),
             np.degrees(np.arctan(1 / r_)) if r_ > 0 else 90.0))
print()
print('  => 要使全程都在锥内，需 gs <= %.1f°（最紧处 alt=8000, dist/alt=%.3f）'
      % (np.degrees(np.arctan(1 / max(p['dist'] / p['alt']
                                      for p in P if p['alt'] > 500))),
         max(p['dist'] / p['alt'] for p in P if p['alt'] > 500)))
print('  => 但 G-FOLD 只从 alt<=5950 接手，故锥只需覆盖交班点之后：')
sub = [p for p in P if 358 <= p['alt'] <= 5950]
print('     该段最大 dist/alt = %.3f -> gs <= %.1f°'
      % (max(p['dist'] / p['alt'] for p in sub),
         np.degrees(np.arctan(1 / max(p['dist'] / p['alt'] for p in sub)))))
print('\n  【结论】kOS 段1 的滑翔角锥应取 gs≈30~50°，而不是现在的 60°')
print('         （60° 对应 dist/alt<=0.577，比真实轨迹的 0.72~0.91 紧，会一直触发回锥偏置）')
