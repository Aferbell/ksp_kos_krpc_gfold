# eguidance_main.py — 段1（主段）E-guidance 可行性验证（真实日志驱动）
#
# 目的：确定段1 交给 G-FOLD 的【交接点】是否存在、在哪，以及段1 的律要满足什么。
# 方法：先用真实日志（land_log_47822691，唯一一趟飞完整剖面的）提取"可行剖面"，
#      再检查在该剖面上，各时刻是否满足 G-FOLD 的进入条件（锥内 + 能量可收）。
#
# 关键结论预期：
#   · 真实飞行靠 FAR 阻力 + 满推力，能在 ~28 s 内从 12.3 km/979 m/s 刹到
#     alt≈358 m / dist≈217 m / |v|≈204 m/s（实测剖面）。这是【可达性上界】。
#   · 纯 3DoF 无阻力模型达不到这个（a_net 只有 47~75 m/s²，而真实平均更高），
#     所以段1 的仿真必须【含阻力】，否则会得出"不可行"的错误结论。
import csv
import glob
import os
import numpy as np

LOGDIR = r'D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script\log-B1040-9'
TARGET_ALT = 36.8


def load(name):
    with open(os.path.join(LOGDIR, name), 'r', encoding='utf-8-sig') as f:
        return list(csv.DictReader(f))


def g(row, k, d=0.0):
    try:
        return float(row[k])
    except (KeyError, ValueError):
        return d


def analyse(name='land_log_47822691.csv'):
    rows = load(name)
    ign = next(i for i, r in enumerate(rows) if g(r, 'phase') >= 1)
    print('=== %s 点火后剖面（真实飞行，含 FAR 阻力）===' % name)
    print('  t      alt    dist_hz    vz      vh     |v|    mass   phase  cone25')
    gs = np.radians(25.0)
    data = []
    for r in rows[ign:]:
        alt = g(r, 'alt'); d = g(r, 'dist_hz')
        vz = g(r, 'vz'); vh = g(r, 'vh'); m = g(r, 'mass')
        t = g(r, 't') - g(rows[ign], 't')
        in_cone = d <= alt * np.tan(gs)
        data.append(dict(t=t, alt=alt, dist=d, vz=vz, vh=vh,
                         v=np.hypot(vz, vh), mass=m, cone=in_cone,
                         phase=g(r, 'phase')))
    # 抽样打印
    shown = set()
    for dd in data:
        s = int(dd['t'] // 2)
        if s not in shown:
            shown.add(s)
            print('  %5.1f %7.0f %8.0f %8.1f %7.1f %7.1f %7.1f %5d   %s'
                  % (dd['t'], dd['alt'], dd['dist'], dd['vz'], dd['vh'], dd['v'],
                     dd['mass'], dd['phase'], 'IN' if dd['cone'] else 'out'))

    # ---- 找交接点：锥内 + 速度进入 G-FOLD 可处理范围 ----
    print('\n=== 交接点判定（G-FOLD 需要：锥内 + 能量可收）===')
    print('  条件：dist <= alt*tan(25°)  且  |v| <= 260 m/s（末段 25 s 内可收）')
    cand = [d for d in data if d['cone'] and d['v'] <= 260 and d['alt'] > TARGET_ALT + 60]
    if cand:
        c = cand[0]
        print('\n  >>> 推荐交接点: t=+%.1f s  alt=%.0f  dist=%.0f  vz=%.1f  vh=%.1f  |v|=%.1f  mass=%.1ft'
              % (c['t'], c['alt'], c['dist'], c['vz'], c['vh'], c['v'], c['mass']))
    else:
        print('\n  未找到（真实轨迹进锥很晚）')

    # ---- 进锥时刻 ----
    inc = [d for d in data if d['cone']]
    if inc:
        c0 = inc[0]
        print('  首次进锥: t=+%.1f s  alt=%.0f  dist=%.0f  |v|=%.1f  mass=%.1ft'
              % (c0['t'], c0['alt'], c0['dist'], c0['v'], c0['mass']))
    # ---- 该趟真实终止状态 ----
    last = data[-1]
    print('  真实最后一行: alt=%.0f dist=%.0f vz=%.1f vh=%.1f'
          % (last['alt'], last['dist'], last['vz'], last['vh']))
    return data


def energy_check(data):
    """检查：段1 在【无阻力】3DoF 下能不能到达交接点？用 ASCII 输出避免 GBK 编码问题。"""
    print('\n=== 段1 可达性核算（a_net 随质量变化）===')
    TMAX = 12.8e6
    G0 = 9.81
    d0 = data[0]
    m0 = d0['mass'] * 1000
    v0 = d0['v']
    print('  ignition: |v|=%.1f m/s  a_net(initial)=%.1f m/s2' % (v0, TMAX / m0 - G0))
    m_end = 172e3
    a_avg = TMAX / ((m0 + m_end) / 2) - G0
    print('  mean a_net (%.0f t -> %.0f t) = %.1f m/s2'
          % (m0 / 1000, m_end / 1000, a_avg))
    print('  pure-3DoF time to kill |v|=%.0f : %.1f s' % (v0, v0 / a_avg))
    t358 = next((x['t'] for x in data if x['alt'] < 400), float('nan'))
    print('  real flight time (ignition -> alt<400 m) = %.1f s' % t358)
    print('  => real flight is FASTER than pure 3DoF by %.1f s;'
          % (v0 / a_avg - t358))
    print('     that difference IS the FAR drag contribution (model must include it).')


if __name__ == '__main__':
    d = analyse()
    energy_check(d)
