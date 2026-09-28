# report_terminal.py — 用真实日志数据验证 G-FOLD 末段解，并打印完整剖面
#
# 数据来源：log-B1040-9/land_log_47822691.csv 的真实点火帧与飞行剖面。
# 目的：① 确认 G-FOLD 在末段域可解；② 打印剖面看是否物理合理（不穿地、推力连续、
#       末端速度/姿态达标）；③ 找出"从哪里交给 G-FOLD"（段1→段2 交接点）。
import csv
import glob
import os
import numpy as np

# --- 路径自愈：analysis/ 下的脚本引用 solver/ 里的求解器 ---
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'solver'))
# --------------------------------------------------------
from gfold_terminal import solve_terminal

LOGDIR = r'D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script\log-B1040-9'
TARGET_ALT = 36.8
TARGET_LAT = -0.0972060951675948
TARGET_LON = -74.5576822740041
RE = 600000.0


def load(logname):
    path = os.path.join(LOGDIR, logname)
    with open(path, 'r', encoding='utf-8-sig') as f:
        return list(csv.DictReader(f))


def f(row, k, d=0.0):
    try:
        return float(row[k])
    except (KeyError, ValueError):
        return d


def main():
    logs = sorted(glob.glob(os.path.join(LOGDIR, 'land_log_*.csv')))
    print('可用日志：')
    for p in logs:
        print('   ', os.path.basename(p))

    rows = load('land_log_47822691.csv')
    print('\n=== land_log_47822691 点火帧起的真实剖面（每 ~2 s 抽一行）===')
    ign = None
    for i, r in enumerate(rows):
        if f(r, 'phase') >= 1:
            ign = i
            break
    print('点火行 idx=%d  t=%.1f  alt=%.0f  dist_hz=%.0f  vz=%.1f  vh=%.1f  mass=%.1ft'
          % (ign, f(rows[ign], 't'), f(rows[ign], 'alt'), f(rows[ign], 'dist_hz'),
             f(rows[ign], 'vz'), f(rows[ign], 'vh'), f(rows[ign], 'mass')))
    print('\n  t      alt    dist_hz    vz      vh     mass    phase')
    for r in rows[ign:]:
        t = f(r, 't')
        if int(t) % 4 == 0 and abs(t - round(t)) < 0.06:
            print('  %5.1f %7.0f %8.0f %8.1f %7.1f %7.1f %5d'
                  % (t, f(r, 'alt'), f(r, 'dist_hz'), f(r, 'vz'), f(r, 'vh'),
                     f(r, 'mass'), f(r, 'phase')))

    # ---- 找"进入锥内"的时刻：dist <= alt*tan(gs)，gs=25° ----
    print('\n=== 找段1→段2 交接点（进入滑翔角锥 gs=25° 且 |v| 已降到末段可处理）===')
    gs = np.radians(25.0)
    pick = None
    for r in rows[ign:]:
        alt = f(r, 'alt')
        d = f(r, 'dist_hz')
        vz = f(r, 'vz')
        vh = f(r, 'vh')
        if alt <= TARGET_ALT + 50:
            break
        in_cone = d <= alt * np.tan(gs)
        slow = np.hypot(vz, vh) < 260
        if in_cone and slow and pick is None:
            pick = (f(r, 't'), alt, d, vz, vh, f(r, 'mass'))
            break
    if pick is None:
        print('  未找到满足条件的交接点，改用扫描表：')
        for r in rows[ign:]:
            alt = f(r, 'alt')
            if alt < 3500:
                d = f(r, 'dist_hz'); vz = f(r, 'vz'); vh = f(r, 'vh')
                print('    alt=%6.0f dist=%6.0f |v|=%6.1f in_cone=%s'
                      % (alt, d, np.hypot(vz, vh), d <= alt * np.tan(gs)))
    else:
        print('  交接点: t=%.1f alt=%.0f dist=%.0f vz=%.1f vh=%.1f mass=%.1ft'
              % pick)

    # ---- 对若干"真实可达的末段状态"跑 G-FOLD ----
    print('\n=== G-FOLD 末段求解（输入取自真实剖面的可达状态）===')
    tests = [
        ('早交班 alt2500', 2500, 700, -120, 80, 152e3),
        ('中交班 alt1800', 1800, 450, -90, 60, 148e3),
        ('晚交班 alt1000', 1000, 250, -70, 40, 145e3),
        ('很低  alt500', 500, 120, -50, 25, 143e3),
    ]
    print('  %-16s %-6s %8s %9s %9s %8s %8s' %
          ('case', 'status', 'tf(s)', '落地(t)', '耗油(t)', '末端倾角', '耗时(s)'))
    for (nm, alt, d, vz, vh, m) in tests:
        r = solve_terminal(alt=alt, dist=d, vz=vz, vh=vh, mass=m, tf=25, N=60)
        if r['status'] == 'optimal':
            print('  %-16s %-6s %8.0f %9.1f %9.1f %7.2f° %8.2f'
                  % (nm, 'OK', r['tf'], r['mass_land'] / 1000,
                     r['fuel'] / 1000, r['end_tilt_deg'], r['solve_time']))
        else:
            print('  %-16s %-6s' % (nm, r['status']))

    # ---- 打印一个解的完整剖面，检查物理合理性 ----
    print('\n=== 剖面检查：alt2500 解（x=竖直向上，y=水平朝目标）===')
    r = solve_terminal(alt=2500, dist=700, vz=-120, vh=80, mass=152e3, tf=25, N=60)
    if r['status'] == 'optimal':
        x, u = r['x'], r['u']
        N = x.shape[1]
        print('  t(s)   高度    水平偏差   垂速    横速   推力     推力倾角')
        for i in range(0, N, 5):
            ti = i * r['tf'] / (N - 1)
            tilt = np.degrees(np.arctan2(np.linalg.norm(u[1:3, i]), abs(u[0, i])))
            print('  %5.1f %7.1f %9.1f %8.1f %7.1f %7.2f %8.1f°'
                  % (ti, x[0, i], np.linalg.norm(x[1:3, i]), x[3, i],
                     np.linalg.norm(x[4:6, i]), np.linalg.norm(u[:, i]), tilt))
        # 物理合理性检查
        hmin = x[0, :].min()
        print('\n  最低高度 = %.2f m（应 >= 0，不穿地）' % hmin)
        print('  末端位置误差 = %.4f m  末端速度 = %.4f m/s'
              % (np.linalg.norm(x[0:3, -1]), np.linalg.norm(x[3:6, -1])))
        # 滑翔角锥检查
        gs = np.radians(25.0)
        viol = [(i, np.linalg.norm(x[1:3, i]) - np.tan(gs) * x[0, i])
                for i in range(N)]
        worst = max(viol, key=lambda p: p[1])
        print('  滑翔角锥最大越界 = %.3f m (节点 %d)' % (worst[1], worst[0]))


if __name__ == '__main__':
    main()
