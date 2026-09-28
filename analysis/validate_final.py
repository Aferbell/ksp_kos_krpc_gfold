# validate_final.py — 最终验证：用【真实日志状态 + 真实质量】确定混合架构可行域
#
# 这是交付给用户看的权威结果。修正了三个自造的人为 infeasible 源：
#   ① 推力指向锥全程固定（应按时序收紧）      —— 见 gfold_terminal.py 头注
#   ② z0 质量参考用满推力烧（应取参考减速率） —— 同上
#   ③ v_max=400 < 真实 |v|（应放宽到 1200）   —— 见 diag9_top.py
import csv
import os
import numpy as np

# --- 路径自愈：analysis/ 下的脚本引用 solver/ 里的求解器 ---
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'solver'))
# --------------------------------------------------------
from gfold_terminal import solve_terminal

LOGDIR = r'D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script\log-B1040-9'


def load(name):
    with open(os.path.join(LOGDIR, name), 'r', encoding='utf-8-sig') as f:
        return list(csv.DictReader(f))


def g(r, k, d=0.0):
    try:
        return float(r[k])
    except (KeyError, ValueError):
        return d


def main():
    rows = load('land_log_47822691.csv')
    ign = next(i for i, r in enumerate(rows) if g(r, 'phase') >= 1)
    states = []
    for r in rows[ign:]:
        alt = g(r, 'alt')
        if 60 < alt < 6000:
            states.append((alt, g(r, 'dist_hz'), g(r, 'vz'), g(r, 'vh'), g(r, 'mass')))

    print('=' * 78)
    print('G-FOLD 末段可行域（真实日志 land_log_47822691 + 真实质量，tf 自动搜索）')
    print('=' * 78)
    print('   alt    dist     vz      vh    |v|   mass   结果')
    wins = []
    for (alt, d, vz, vh, m) in states:
        r = solve_terminal(alt=alt, dist=d, vz=vz, vh=vh, mass=m * 1000,
                           N=50, gs_deg=30.0,
                           pcs_start_deg=85.0, pcs_end_deg=5.0)
        if r['status'] == 'optimal':
            wins.append((alt, d, vz, vh, m, r))
    if wins:
        hi = max(w[0] for w in wins)
        lo = min(w[0] for w in wins)
        print('  >>> 可行域: alt %.0f m  ~  %.0f m   共 %d / %d 个真实状态可解'
              % (lo, hi, len(wins), len(states)))
        print('\n  抽样（每 8 个取 1）:')
        print('   alt    dist    |v|   tf(s)  落地(t)  末端倾角  耗油(t)  h_min  爬升?')
        for w in wins[::8]:
            alt, d, vz, vh, m, r = w
            x = r['x']
            print('  %5.0f %7.0f %6.0f %6.1f %8.1f %8.2f° %8.1f %7.2f  %s'
                  % (alt, d, np.hypot(vz, vh), r['tf'], r['mass_land'] / 1000,
                     r['end_tilt_deg'], r['fuel'] / 1000, x[0, :].min(),
                     'YES' if x[3, :].max() > 0.5 else 'no'))
        print('\n  末端倾角全部 <= %.2f°  （指标要求 < 5°）'
              % max(w[5]['end_tilt_deg'] for w in wins))
        print('  末端位置误差 max = %.4f m' % max(w[5]['end_pos_err'] for w in wins))
        print('  末端速度误差 max = %.4f m/s' % max(w[5]['end_vel_err'] for w in wins))
        print('  穿地(h<0) 的解: %d 个' % sum(1 for w in wins if w[5]['x'][0, :].min() < -0.5))
        print('  爬升的解:       %d 个' % sum(1 for w in wins if w[5]['x'][3, :].max() > 0.5))
        print('  单次求解耗时 max = %.2f s（tf 二分 14 次，可离线/低频调用）'
              % max(w[5]['solve_time'] for w in wins))

    print()
    print('=' * 78)
    print('推荐交班点（在可行域内、且高度尽量高以留给末段余量）')
    print('=' * 78)
    if wins:
        best = max(wins, key=lambda w: w[0])
        alt, d, vz, vh, m, r = best
        print('  alt=%.0f  dist=%.0f  vz=%.1f  vh=%.1f  |v|=%.1f  mass=%.1ft'
              % (alt, d, vz, vh, np.hypot(vz, vh), m))
        print('  解: tf=%.0fs 落地%.1ft 耗油%.1ft 末端倾角%.2f° 耗时%.2fs'
              % (r['tf'], r['mass_land'] / 1000, r['fuel'] / 1000,
                 r['end_tilt_deg'], r['solve_time']))
        # 打印剖面
        x, u = r['x'], r['u']
        N = x.shape[1]
        print('\n  剖面:')
        print('   t(s)   高度   水平偏差   垂速    横速   推力    推力倾角')
        for i in range(0, N, 5):
            ti = i * r['tf'] / (N - 1)
            tilt = np.degrees(np.arctan2(np.linalg.norm(u[1:3, i]),
                                         max(1e-9, abs(u[0, i]))))
            print('  %5.1f %7.1f %9.1f %8.1f %7.1f %7.2f %8.1f°'
                  % (ti, x[0, i], np.linalg.norm(x[1:3, i]), x[3, i],
                     np.linalg.norm(x[4:6, i]), np.linalg.norm(u[:, i]), tilt))


if __name__ == '__main__':
    main()
