# validate_key.py — 聚焦验证：只用关键状态（快），给出权威结论
#
# validate_final.py 扫全部 ~50 个状态 * 14 次二分，太慢。这里只取关键点：
#   高/中/低三档 + 找不到可行域的确认点，足以界定边界。
import csv
import os
import numpy as np

# --- 路径自愈：analysis/ 下的脚本引用 solver/ 里的求解器 ---
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'solver'))
# --------------------------------------------------------
from gfold_terminal import solve_terminal

LOGDIR = r'D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script\log-B1040-9'

with open(os.path.join(LOGDIR, 'land_log_47822691.csv'), 'r', encoding='utf-8-sig') as f:
    rows = list(csv.DictReader(f))


def g(r, k, d=0.0):
    try:
        return float(r[k])
    except (KeyError, ValueError):
        return d


ign = next(i for i, r in enumerate(rows) if g(r, 'phase') >= 1)
allst = []
for r in rows[ign:]:
    alt = g(r, 'alt')
    if 60 < alt < 6500:
        allst.append((alt, g(r, 'dist_hz'), g(r, 'vz'), g(r, 'vh'), g(r, 'mass')))

# 目标高度序列（对数分布，覆盖全域）
targets = [6000, 5000, 4000, 3000, 2500, 2000, 1500, 1200, 1000, 800,
           600, 450, 350, 280, 230, 190, 160, 130, 100]
picked = []
for t in targets:
    if not allst:
        break
    best = min(allst, key=lambda s: abs(s[0] - t))
    if all(abs(best[0] - p[0]) > 5 for p in picked):
        picked.append(best)

print('=' * 80)
print('G-FOLD 末段可行性（真实日志状态 + 真实质量；tf 自动搜索；N=40）')
print('=' * 80)
print('   alt    dist     vz      vh   |v|  mass  -> 结果      tf   末端倾角  h_min 爬升')
ok_list = []
for (alt, d, vz, vh, m) in sorted(picked, reverse=True):
    r = solve_terminal(alt=alt, dist=d, vz=vz, vh=vh, mass=m * 1000,
                       N=40, gs_deg=30.0, pcs_start_deg=85.0, pcs_end_deg=5.0)
    if r['status'] == 'optimal':
        x = r['x']
        ok_list.append(alt)
        print('  %5.0f %7.0f %7.1f %7.1f %5.0f %6.1f  -> optimal  %5.1f %7.2f° %7.2f  %s'
              % (alt, d, vz, vh, np.hypot(vz, vh), m, r['tf'],
                 r['end_tilt_deg'], x[0, :].min(),
                 'YES' if x[3, :].max() > 0.5 else 'no'))
    else:
        print('  %5.0f %7.0f %7.1f %7.1f %5.0f %6.1f  -> %s'
              % (alt, d, vz, vh, np.hypot(vz, vh), m, r['status']))

print()
if ok_list:
    print('可行域高度区间: %.0f m ~ %.0f m' % (min(ok_list), max(ok_list)))
print('可解 %d / %d' % (len(ok_list), len(picked)))
