# diag6_tilt.py — 为什么末端倾角在 0° 与 89° 之间跳变
#
# 现象：gfold_terminal 的自测（人造状态）末端倾角 0.00°，但真实剖面状态
#       大量出现 89.33° / 88.96° / 86.15° —— 即末端推力几乎【水平】。
#
# 根因推断：我在末端只加了 u[:,N-1] == s[0,N-1]*[1,0,0]，同时 s[0,N-1]==0。
#   当 s→0 时这条等式两边都→0，它退化成 "u末 = 0"（对 u 没有方向约束）。
#   只要 s 足够小，u 可以是任意小量而不违反。于是"末端推力竖直"名存实亡。
#   真正的做法：约束 u[1:3, N-1] == 0（水平分量恒为 0），并与 s 解耦。
#
# 同时末端倾角统计口径也要改：应该看 |u_h| / u_v，而不是 atan2(|u_h|, |u_v|)
#   在 u_v≈0 时的数值噪声。
import numpy as np
# --- 路径自愈：analysis/ 下的脚本引用 solver/ 里的求解器 ---
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'solver'))
# --------------------------------------------------------
from gfold_terminal import solve_terminal


def tilt_profile(r):
    u, s = r['u'], r['s']
    N = u.shape[1]
    out = []
    for i in range(N):
        uh = np.linalg.norm(u[1:3, i])
        uv = abs(u[0, i])
        out.append((np.degrees(np.arctan2(uh, max(1e-12, uv))), uh, uv, s[0, i]))
    return out


print('### 现状：末端 s 与 u 的关系（暴露约束退化）')
for (nm, alt, d, vz, vh, m) in [
    ('人造-典型', 2000, 500, -80, 50, 150e3),
    ('真实-1727', 1727, 1236, -269, 190, 184e3),
    ('真实-1038', 1038, 718, -250, 161, 183e3),
]:
    r = solve_terminal(alt=alt, dist=d, vz=vz, vh=vh, mass=m, tf=20, N=60,
                       gs_deg=30.0, pcs_start_deg=80.0, pcs_end_deg=5.0)
    if r['status'] != 'optimal':
        print('  %-12s %s' % (nm, r['status']))
        continue
    tp = tilt_profile(r)
    print('  %-12s 末端: tilt=%.2f° |uh|=%.4f uv=%.4f s=%.6f'
          % (nm, tp[-1][0], tp[-1][1], tp[-1][2], tp[-1][3]))
    print('               倒数第5节点: tilt=%.2f° |uh|=%.4f uv=%.4f s=%.6f'
          % (tp[-5][0], tp[-5][1], tp[-5][2], tp[-5][3]))

print()
print('### 判据：pcs 收紧到 5° 时，末端前一节点应满足 |u_h| <= tan(5°)*u_v')
for (nm, alt, d, vz, vh, m) in [
    ('人造-典型', 2000, 500, -80, 50, 150e3),
    ('真实-1727', 1727, 1236, -269, 190, 184e3),
]:
    r = solve_terminal(alt=alt, dist=d, vz=vz, vh=vh, mass=m, tf=20, N=60,
                       gs_deg=30.0, pcs_start_deg=80.0, pcs_end_deg=5.0)
    if r['status'] != 'optimal':
        continue
    u = r['u']
    N = u.shape[1]
    print('  %s' % nm)
    for i in [N-1, N-2, N-3, N-5, N-8]:
        uh = np.linalg.norm(u[1:3, i]); uv = abs(u[0, i])
        allowed = np.tan(np.radians(5.0)) * uv
        print('    节点 %2d: |uh|=%9.5f  uv=%8.4f  allowed=%8.5f  %s'
              % (i, uh, uv, allowed, 'OK' if uh <= allowed + 1e-6 else 'VIOLATE'))
