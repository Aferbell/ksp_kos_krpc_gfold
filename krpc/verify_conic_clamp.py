# verify_conic_clamp.py — 验证 conic_clamp（照抄参考仓库）确实能挡住横飞
#
# 【背景】2026-09-26 实飞（gfold_log_20260926_113321.csv）火箭"横飞"：
#     t=0.004  a_cmd_mag=1290.7 而可用推力仅 49.28（超 26 倍）
#     vh: 186 -> 873 m/s，alt 几乎不降
#   根因：参考仓库 demo3_gfold.py 在 PD 之后有一步 conic_clamp（line 389-390），
#   我自行设计时【漏掉了】，于是 PD 修正量可以无限大。
#
# 本测试用实飞首帧的真实数字，验证限幅后的输出落在物理可执行范围内。
import math
import sys

import numpy as np

G0 = 9.80665


def clamp(num, maxnum, minnum):
    """逐行照抄参考仓库的 clamp（注意参数顺序是 max 在前）。"""
    if num > maxnum:
        return maxnum
    elif num < minnum:
        return minnum
    return num


def conic_clamp(target_a, min_mag, max_mag, max_tilt, hor_dir_div_guard=False):
    """参考仓库 demo3_gfold.py 的 conic_clamp（逐行移植）。"""
    a_mag = float(np.linalg.norm(target_a))
    hor_dir = np.array([0.0, target_a[1], target_a[2]])
    n = float(np.linalg.norm(hor_dir))
    if n < 1e-9:
        if hor_dir_div_guard:
            return np.array([clamp(target_a[0], max_mag, min_mag), 0.0, 0.0])
        # 参考原版会在这里产生 nan —— 记录但不崩
        return np.array([float('nan')] * 3)
    hor_dir = hor_dir / n
    a_hor = float(np.linalg.norm(target_a[1:3]))
    a_ver = float(target_a[0])

    if a_hor < min_mag * math.sin(max_tilt):
        a_ver_min = math.sqrt(max(0.0, min_mag ** 2 - a_hor ** 2))
    else:
        a_ver_min = math.cos(max_tilt) * min_mag

    if a_hor < max_mag * math.sin(max_tilt):
        a_ver_max = math.sqrt(max(0.0, max_mag ** 2 - a_hor ** 2))
    else:
        a_ver_max = math.cos(max_tilt) * max_mag

    a_ver = clamp(a_ver, a_ver_max, a_ver_min)
    a_hor = min(a_hor, a_ver * math.tan(max_tilt))
    _ = a_mag
    return hor_dir * a_hor + np.array([a_ver, 0.0, 0.0])


def main():
    # 实飞首帧真实数字
    mass = 182414.9
    tmax = 8.99e6
    a_cap = tmax / mass
    a_cmd_raw = np.array([1288.694, 72.564, 0.0])       # a_up, a_h, 0
    print('=' * 76)
    print('conic_clamp 验证（用实飞首帧真实数字）')
    print('=' * 76)
    print('  mass=%.1f t  T_max=%.2f MN  ->  可用 a=%.2f m/s2'
          % (mass / 1000, tmax / 1e6, a_cap))
    print('  原始指令 |a| = %.2f  (超 %.1f 倍)'
          % (np.linalg.norm(a_cmd_raw), np.linalg.norm(a_cmd_raw) / a_cap))

    ok = True

    # ① 修复前的行为：throttle 饱和
    thr_raw = np.linalg.norm(a_cmd_raw) / a_cap
    print('\n[修复前] throttle = |a|/(T/m) = %.2f  -> 饱和到 1.0' % thr_raw)
    print('         竖直 %.1f 主导，水平 72.6 相对失控 => 火箭横飞' % a_cmd_raw[0])

    # ② 施加 conic_clamp
    for tilt_deg in (25.0, 12.0):
        tilt = math.radians(tilt_deg)
        out = conic_clamp(a_cmd_raw, 0.05 * a_cap, 1.0 * a_cap, tilt,
                          hor_dir_div_guard=True)
        mag = float(np.linalg.norm(out))
        thr = mag / a_cap
        hor = float(np.linalg.norm(out[1:3]))
        tilt_out = math.degrees(math.atan2(hor, max(1e-9, out[0])))
        print('\n[修复后] max_tilt=%.0f°' % tilt_deg)
        print('   a = [%8.2f, %7.2f, %7.2f]   |a| = %.2f'
              % (out[0], out[1], out[2], mag))
        print('   throttle = %.3f   推力轴倾角 = %.2f°'
              % (thr, tilt_out))
        c1 = thr <= 1.0 + 1e-9
        c2 = tilt_out <= tilt_deg + 1e-6
        print('   [%s] throttle <= 1' % ('OK ' if c1 else 'FAIL'))
        print('   [%s] 倾角 <= max_tilt' % ('OK ' if c2 else 'FAIL'))
        ok = ok and c1 and c2

    # ③ 边界：水平分量为 0 时不应产生 nan（参考原版此处会 nan）
    print('\n[边界] 水平分量为 0:')
    out0 = conic_clamp(np.array([30.0, 0.0, 0.0]), 0.05 * a_cap, a_cap,
                       math.radians(25.0), hor_dir_div_guard=True)
    c3 = not np.any(np.isnan(out0))
    print('   out = %s   [%s] 无 nan' % (np.round(out0, 3), 'OK ' if c3 else 'FAIL'))
    ok = ok and c3

    # ④ 物理：任何输入下 |a| 都不超过 a_cap
    print('\n[随机扫] 1000 组输入，检查 |a| <= T/m 且倾角 <= max_tilt')
    rng = np.random.default_rng(0)
    worst_mag, worst_tilt = 0.0, 0.0
    for _ in range(1000):
        t = rng.normal(0, 500, 3)
        t[0] = abs(t[0])
        o = conic_clamp(t, 0.05 * a_cap, a_cap, math.radians(25.0),
                        hor_dir_div_guard=True)
        m = float(np.linalg.norm(o))
        h = float(np.linalg.norm(o[1:3]))
        worst_mag = max(worst_mag, m / a_cap)
        worst_tilt = max(worst_tilt,
                         math.degrees(math.atan2(h, max(1e-9, o[0]))))
    c4 = worst_mag <= 1.0 + 1e-6 and worst_tilt <= 25.0 + 1e-6
    print('   最大 |a|/(T/m) = %.4f   最大倾角 = %.2f°   [%s]'
          % (worst_mag, worst_tilt, 'OK ' if c4 else 'FAIL'))
    ok = ok and c4

    print('\n结论:', '全部通过 —— conic_clamp 能把指令压回物理可执行范围'
          if ok else '有未通过项')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
