# verify_frames.py — 参考系口径回归测试（不连 KSP，用实飞日志数据）
#
# 【为什么需要这个文件】
#   2026-09-26 实飞失败（log-B1040-9-gfold/gfold_log_20260926_103746.csv）
#   的根因是【速度参考系传错】：target_frame 的 create_hybrid(velocity=body_frame)
#   把天体自转线速度混进了 vh。症状极具欺骗性 ——
#     · 位置 dist 完全正确
#     · 竖直速度 vz 完全正确
#     · 只有水平速度 vh 恒多出 ~175 m/s（= Kerbin 该纬度自转 174.9）
#   于是求解器一路 infeasible、跟踪律把 throttle 顶到 1.0 把火箭往上推，
#   而所有"检查"都看不出来。这个测试用【位置位移】这个独立口径去校验
#   【速度】，任何再犯的参考系错误都会被当场抓住。
import math
import os
import sys

CSV = os.path.join(
    r'D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script',
    'log-B1040-9-gfold', 'gfold_log_20260926_103746.csv')

# Kerbin 自转在回收场纬度处的线速度标尺
KERBIN_ROT_PERIOD = 21549.425      # s（6 小时）
KERBIN_RADIUS = 600000.0           # m
LAT_DEG = -0.095                   # 交接点纬度（日志 kOS handoff 行）


def v_rotation():
    return (2 * math.pi / KERBIN_ROT_PERIOD) * KERBIN_RADIUS \
        * math.cos(math.radians(LAT_DEG))


def load():
    rows = []
    with open(CSV, encoding='utf-8', errors='replace') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or line.startswith('t,'):
                continue
            p = line.split(',')
            if len(p) < 7:
                continue
            try:
                rows.append(dict(t=float(p[0]), alt=float(p[2]),
                                 dist=float(p[3]), vz=float(p[4]),
                                 vh=float(p[5])))
            except ValueError:
                continue
    return rows


def main():
    rows = load()
    if len(rows) < 10:
        print('FAIL: 日志样本不足，无法做口径回归')
        return 1
    vrot = v_rotation()
    print('=' * 74)
    print('参考系口径回归（数据源：实飞日志 %s）' % os.path.basename(CSV))
    print('  样本 %d 条   自转标尺 = %.1f m/s' % (len(rows), vrot))
    print('=' * 74)

    # 用位置差分独立反推真实速率（位置口径不受速度参考系影响）
    vh_meas, vz_meas = [], []
    for a, b in zip(rows, rows[1:]):
        dt = b['t'] - a['t']
        if dt <= 1e-3:
            continue
        vh_meas.append(abs(b['dist'] - a['dist']) / dt)
        vz_meas.append((b['alt'] - a['alt']) / dt)
        _ = b
    vh_log = [abs(r['vh']) for r in rows[1:]]
    vz_log = [abs(r['vz']) for r in rows[1:]]

    def med(x):
        s = sorted(x)
        return s[len(s) // 2]

    m_vh, l_vh = med(vh_meas), med(vh_log)
    m_vz, l_vz = med(vz_meas), med(vz_log)

    print('\n%-14s %10s %10s %10s' % ('分量', '位移反推', '日志', '差'))
    print('%-14s %10.2f %10.2f %10.2f' % ('水平 vh', m_vh, l_vh, l_vh - m_vh))
    print('%-14s %10.2f %10.2f %10.2f' % ('竖直 vz', m_vz, l_vz, l_vz - m_vz))

    ok = True
    # ① 竖直应吻合（证明位置与竖直口径都对）
    d_vz = abs(l_vz - m_vz)
    c1 = d_vz < max(2.0, 0.15 * max(1.0, m_vz))
    print('\n[%s] ① 竖直速度吻合 (差 %.2f m/s)' % ('OK ' if c1 else 'FAIL', d_vz))
    ok = ok and c1

    # ② 水平【不应】恰好差一个自转速度（这正是被污染的特征）
    d_vh = l_vh - m_vh
    near_rot = abs(d_vh - vrot) < 0.25 * vrot
    c2 = not near_rot
    print('[%s] ② 水平速度未被自转污染 (差 %.1f，自转标尺 %.1f)'
          % ('OK ' if c2 else 'FAIL', d_vh, vrot))
    ok = ok and c2

    # ③ 水平速度量级应可信（不应比位移反推大 5 倍以上）
    c3 = l_vh < max(5.0, 5.0 * max(1.0, m_vh + 1.0))
    print('[%s] ③ 水平速度量级可信 (日志 %.1f / 反推 %.1f)'
          % ('OK ' if c3 else 'FAIL', l_vh, m_vh))
    ok = ok and c3

    print('\n结论:', '全部通过' if ok else '存在参考系口径错误')
    if not ok:
        print('\n提示：若 ② 失败，检查 gfold_land.py 的 target_frame()：')
        print('      create_hybrid 的 velocity 参数必须是 tgt_frame，')
        print('      不能是 body_frame（那会把天体自转线速度混进 vh）。')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
