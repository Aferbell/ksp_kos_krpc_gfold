# verify_real_handoff.py — 用【实飞真实交班点】跑离线闭环
#
# 【为什么单独写这个文件】
#   verify_control_loop.py 用的入场状态是早期 MATLAB 仿真的理想值
#   （alt 5036 / dist 142 / vz -72 / vh 20.5 / 152 t），横速很小、能量很低。
#   而实飞 kOS 交班实际给出的是：
#       alt 5031.8  dist 1285  |v| 478.8  (vz -314  vh 361.5)  mass 182.8 t
#   横向远远没消掉（vh 361.5 而非 20.5），能量显著更高。
#   旧版求解器（v_descent_max 包络与初速自相矛盾）在这个状态下报 infeasible，
#   导致实飞接管失败。本文件就是复现该工况并验证修复。
import os
import sys
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..', 'gfold', 'solver'))
from gfold_terminal import solve_terminal          # noqa: E402

TARGET_ALT = 36.8
G0 = 9.80665
ISP = 315.0
# 【载具参数必须对齐真实载具】B1040-9 实测最大推力 8.99 MN（日志 thrust=8.99 MN）。
#   求解器默认 t_max=12.8e6 是 B1040-7 的推力，直接拿来做 B1040-9 的
#   可实现性判定会得出错误结论。
TMAX = 8.990e6
SOLVER = dict(N=40, gs_deg=30.0, pcs_start_deg=85.0, pcs_end_deg=3.0,
              throttle=(0.05, 1.0), v_descent_max=160.0, term_win=0.40,
              term_vz=4.0, t_max=TMAX)

# 实飞交班点（最新日志）
S0 = dict(alt=5031.8, dist=1285.0, vz=-314.0, vh=361.5, mass=182.8e3)


def main():
    print('=' * 78)
    print('实飞交班点闭环验证')
    print('  入场: alt=%.0f dist=%.0f vz=%.0f vh=%.0f mass=%.1f t'
          % (S0['alt'], S0['dist'], S0['vz'], S0['vh'], S0['mass'] / 1000))
    print('  可用推力加速度 T/m = %.1f m/s2'
          % (TMAX / S0['mass']))
    print('=' * 78)

    # ---- ① 单次解算（旧版此处 infeasible）----
    r = solve_terminal(alt=S0['alt'], dist=S0['dist'], vz=S0['vz'],
                       vh=S0['vh'], mass=S0['mass'],
                       target_alt=TARGET_ALT, **SOLVER)
    print('\n[①] 首次解算: %s' % r['status'])
    if r['status'] != 'optimal':
        print('    FAIL — 修复无效')
        return 1
    print('    tf = %.1f s   落地质量 %.1f t   燃料 %.1f t'
          % (r['tf'], r['mass_land'] / 1000, r['fuel'] / 1000))
    print('    末端位置误差 %.4f m   末端速度误差 %.4f m/s'
          % (r['end_pos_err'], r['end_vel_err']))
    print('    末端倾角 %.2f deg   峰值倾角 %.1f deg'
          % (r['end_tilt_deg'], r['peak_tilt_deg']))

    # ---- ② 参考轨迹形状体检（是否可跟踪）----
    x, u, tf = r['x'], r['u'], r['tf']
    N = x.shape[1]
    vz_prof = x[3, :]
    print('\n[②] 参考下降剖面（垂速 m/s / 高度 m）:')
    for i in range(0, N, max(1, N // 8)):
        print('    t=%5.1fs  alt=%7.1f  vz=%8.1f  |u|=%6.2f'
              % (i * tf / (N - 1), x[0, i] + TARGET_ALT, vz_prof[i],
                 np.linalg.norm(u[:, i])))
    vmax_desc = float(np.max(np.abs(vz_prof)))
    print('    最大下降率 %.1f m/s   末段 vz=%.2f m/s'
          % (vmax_desc, vz_prof[-1]))

    # ---- ③ 推力是否越界 ----
    # 【口径很重要（踩过的坑）】不能拿"初质量"算上限 T/m0 去卡整条轨迹：
    #   燃料烧掉后质量变小，真上限 T/m(t) 是【逐节点升高】的。
    #   用 T/m0 当恒定上限会把后段合法推力误判成越界
    #   （实测 T/m0=49.2，而末段真上限已升到 62.4，误报 "max=59.6 越界"）。
    #   正确做法：逐节点用【该节点真实质量】算上限，与求解器自报的
    #   thrust_over_ratio 交叉核对。
    amag = np.linalg.norm(u, axis=0)
    mv = r['m'][0]
    a_lim = TMAX / mv
    ratio = float(np.max(amag / a_lim))
    print('\n[③] 推力体检:')
    print('    初质量上限 T/m0 = %.2f   末段真上限 = %.2f'
          % (TMAX / S0['mass'], TMAX / mv[-1]))
    print('    |u| max = %.2f   逐节点越界比 max = %.4f  (求解器自报 %.4f)'
          % (amag.max(), ratio, r['thrust_over_ratio']))
    if ratio > 1.001:
        print('    FAIL — 推力越界（参考轨迹物理不可实现）')
        return 1
    print('    OK — 参考轨迹物理可实现')

    print('\n结论: 实飞交班点【可解】，修复有效。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
