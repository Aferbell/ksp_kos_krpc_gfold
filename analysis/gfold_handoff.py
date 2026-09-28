# gfold_handoff.py — 用真实轨迹的【真实交接点】验证 G-FOLD 可接手
#
# 真实数据（land_log_47822691）：交接点 alt=163 / dist=74 / vz=-119.5 / vh=96.2 /
#                                |v|=153.4 / mass=172.4 t
#
# 重要修正（推翻 handoff §6.2 的表述）：
#   handoff 说"点火状态 45° 斜插，不在锥内，所以 G-FOLD infeasible"。
#   实测：真实轨迹【全程】的"位置矢量偏离竖直"都≈42°，只有到 alt=163 才降到 24°。
#   也就是说，滑翔角锥 γ_gs 必须 ≥42° 才能容下真实轨迹 —— 25°/20° 都太窄。
#   但这【不是】G-FOLD infeasible 的真正原因（diag2 已证明：放开滑翔锥仍然
#   infeasible）。真正原因是【推力指向锥】+【z0 质量参考】两个建模问题。
#   两个都已修好（gfold_terminal.py）。
#
# 本脚本用真实交接点状态喂修正后的求解器，确认末段可接手。
import csv
import os
import numpy as np

# --- 路径自愈：analysis/ 下的脚本引用 solver/ 里的求解器 ---
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'solver'))
# --------------------------------------------------------
from gfold_terminal import solve_terminal

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


def main():
    rows = load('land_log_47822691.csv')
    ign = next(i for i, r in enumerate(rows) if g(r, 'phase') >= 1)
    # 提取真实剖面中 alt 在 100~3000 m 之间的所有状态，作为"候选交接点"
    cands = []
    for r in rows[ign:]:
        alt = g(r, 'alt')
        if 100 < alt < 3000:
            cands.append((g(r, 't') - g(rows[ign], 't'), alt, g(r, 'dist_hz'),
                          g(r, 'vz'), g(r, 'vh'), g(r, 'mass')))
    print('=== 真实剖面中的候选交接点（alt 100~3000 m）===')
    print('  t(s)    alt    dist     vz      vh     |v|    mass')
    for (t, alt, d, vz, vh, m) in cands[::4]:
        print('  %5.1f %7.0f %7.0f %8.1f %7.1f %7.1f %7.1f'
              % (t, alt, d, vz, vh, np.hypot(vz, vh), m))

    print('\n=== 对每个候选交接点跑修正后的 G-FOLD ===')
    print('  alt   dist   |v|    -> status   tf  落地(t)  末端倾角  峰值倾角  耗时')
    # 【滑翔角锥语义修正】约束是 norm(x_h) <= (1/tan(gs)) * x_v，即 gs 从【水平】量起。
    #   gs 越大锥越【陡】越紧。真实轨迹 dist/alt≈0.75（37° 离竖直），
    #   需要 1/tan(gs) >= 0.75  =>  gs <= atan(1/0.75) = 53°。
    #   所以 gs 应取【较小的值】来放宽（之前我传 60° 反而把它夹死，是我的错）。
    for (t, alt, d, vz, vh, m) in cands[::6]:
        r = solve_terminal(alt=alt, dist=d, vz=vz, vh=vh, mass=m * 1000,
                           tf=20, N=60, gs_deg=30.0,
                           pcs_start_deg=80.0, pcs_end_deg=5.0)
        if r['status'] == 'optimal':
            print('  %5.0f %6.0f %6.1f  -> OK      %2.0f  %7.1f  %7.2f°  %7.1f°  %.2fs'
                  % (alt, d, np.hypot(vz, vh), r['tf'], r['mass_land'] / 1000,
                     r['end_tilt_deg'], r['peak_tilt_deg'], r['solve_time']))
        else:
            print('  %5.0f %6.0f %6.1f  -> %s' % (alt, d, np.hypot(vz, vh), r['status']))

    # ---- 完整闭环：从真实交接点 alt=163 解到地面 ----
    print('\n=== 交班点 alt=163 完整解（剖面）===')
    r = solve_terminal(alt=163, dist=74, vz=-119.5, vh=96.2, mass=172.4e3,
                       tf=20, N=60, gs_deg=30.0,
                       pcs_start_deg=85.0, pcs_end_deg=5.0)
    if r['status'] == 'optimal':
        x, u = r['x'], r['u']
        N = x.shape[1]
        print('  t(s)   高度   水平偏差   垂速    横速   推力     推力倾角')
        for i in range(0, N, 4):
            ti = i * r['tf'] / (N - 1)
            tilt = np.degrees(np.arctan2(np.linalg.norm(u[1:3, i]), abs(u[0, i])))
            print('  %5.1f %7.1f %9.1f %8.1f %7.1f %7.2f %8.1f°'
                  % (ti, x[0, i], np.linalg.norm(x[1:3, i]), x[3, i],
                     np.linalg.norm(x[4:6, i]), np.linalg.norm(u[:, i]), tilt))
        print('\n  最低高度=%.2f m  末端位置误差=%.4f m  末端速度=%.4f m/s  末端倾角=%.2f°'
              % (x[0, :].min(), np.linalg.norm(x[0:3, -1]),
                 np.linalg.norm(x[3:6, -1]), r['end_tilt_deg']))
    else:
        print('  %s' % r['status'])


if __name__ == '__main__':
    main()
