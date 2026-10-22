# -*- coding: utf-8 -*-
"""check_new_gates.py - 验证门槛链（TARGET_ALT=26.5 / TC=20 / FIN_ALT=10）"""
import io
import os
import sys

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(_ROOT, 'dev', 'starship'))

TA, TC, FA, LE = 26.5, 20.0, 10.0, 1.0
TOUCHDOWN = 24.2   # 184650 实测的真触地 surface_altitude


def sec1():
    print("=" * 92)
    print("1. 新门槛链")
    print("=" * 92)
    print("  TARGET_ALT     = %6.1f m" % TA)
    print("  TERM_CLEARANCE = %6.1f m  => 规划零速点 alt=%.1f m" % (TC, TA + TC))
    print("  FIN_ALT        = %6.1f m  => FIN 接管门 alt=%.1f m" % (FA, TA + FA))
    print("  LAND_ALT_EPS   = %6.1f m  => 非FIN触地判据 alt=%.1f m" % (LE, TA + LE))
    print()
    print("  alt=%.1f m  规划末节点 / 末段律接管" % (TA + TC))
    print("  alt=%.1f m  FIN 接管" % (TA + FA))
    print("  alt=%.1f m  非FIN判已落地" % (TA + LE))
    print("  alt=%.1f m  规划归零点 = 腿触点" % TA)
    print()
    print("  末段律窗口 = %.1f m" % (TC - FA))
    print("  FIN 窗口   = %.1f m" % FA)
    ok = True
    if TC <= FA:
        print("  X TERM_CLEARANCE 必须 > FIN_ALT")
        ok = False
    if FA <= LE:
        print("  X FIN_ALT 必须 > LAND_ALT_EPS")
        ok = False
    if ok:
        print("  OK 门槛链自洽")
    print()
    return ok


def sec2():
    print("=" * 92)
    print("2. FIN_ALT=10 m 能兜住多大入口垂速")
    print("=" * 92)
    a_cap, g, tgt = 30.42, 9.80665, 1.0
    a_net = a_cap - g
    print("  a_net = %.2f - %.2f = %.2f m/s^2" % (a_cap, g, a_net))
    vmax = (2 * a_net * FA + tgt ** 2) ** 0.5
    print("  名义 FIN_ALT = %.1f m  => 最大入口速度 %.1f m/s" % (FA, vmax))
    real_travel = (TA + FA) - TOUCHDOWN
    vreal = (2 * a_net * real_travel + tgt ** 2) ** 0.5
    print("  真实可用行程 = FIN门(%.1f) - 真触地(%.1f) = %.1f m" % (
        TA + FA, TOUCHDOWN, real_travel))
    print("  => 实际能兜住最大入口速度 = %.1f m/s" % vreal)
    print()
    print("  %-12s %-14s %-12s" % ("入口vz", "刹车需高度", "真实行程够不够"))
    for vz in (5, 10, 15, 20, 22, 25, 30, 40, 60):
        h = (vz * vz - tgt * tgt) / (2.0 * a_net)
        print("  %-12s %-14.1f %-12s" % (
            "-%.0f m/s" % vz, h, "OK" if h <= real_travel else "不够"))
    print()


def sec3():
    print("=" * 92)
    print("3. TERM_CLEARANCE 5->20 让求解器起点降 15 m，是否仍 feasible")
    print("=" * 92)
    try:
        import numpy as np
        import starship_config as C
        import starship_p3p4 as G
    except Exception as e:
        print("  导入失败: %s" % e)
        return
    p3 = G._load_cgen(3, 160)
    p4 = G._load_cgen(4, 80)
    print("  P3=%s  P4=%s" % (p3 is not None, p4 is not None))
    print()
    mass = 1010000.0
    cases = [
        ("交班 976m vz-70 vh3", 976.0, -70.0, 3.0),
        ("中段 700m vz-60 vh5", 700.0, -60.0, 5.0),
        ("低空 400m vz-50 vh3", 400.0, -50.0, 3.0),
    ]
    print("  %-22s %-10s %-10s %-9s %s" % ("工况", "TA=18", "TA=26.5", "tf差", "判定"))
    for tag, alt, vz, vh in cases:
        out = []
        for tc in (18.0, 26.5):
            h_rel = alt - TA - tc
            x0 = np.array([h_rel, 0.0, 0.0, vz, vh, 0.0])
            try:
                r = G.solve_p3p4_state(
                    x0, mass, isp=315.0, t_max=C.TMAX_N,
                    throttle=(0.1, 0.8), tf_guess=40.0, straight_fac=5.0,
                    gs_deg=30.0, pcs_deg=5.0, pcs_start_deg=85.0,
                    v_max=1200.0, N3=160, N4=80, target_alt=0.0)
                out.append((r.get("status"), r.get("tf", float("nan"))))
            except Exception as e:
                out.append(("EXC:" + type(e).__name__, float("nan")))
        s5, tf5 = out[0]
        s20, tf20 = out[1]
        verdict = "OK" if s20 == "optimal" else "不feasible"
        print("  %-22s %-10s %-10s %-9.1f %s" % (tag, s5, s20, tf20 - tf5, verdict))
    print()


def sec4():
    print("=" * 92)
    print("4. 改 TARGET_ALT 的连锁影响")
    print("=" * 92)
    ta_old, ta_new, aim, gh = 18.0, 26.5, 800.0, 150.0
    print("  %-38s %-12s %-12s" % ("项", "改前", "改后"))
    print("  %-38s %-12.1f %-12.1f" % ("TARGET_ALT", ta_old, ta_new))
    print("  %-38s %-12.1f %-12.1f" % ("GATE_ALT = T+aim+gfold_h",
                                       ta_old + aim + gh, ta_new + aim + gh))
    print("  %-38s %-12.1f %-12.1f" % ("规划零速点 = T+CLEARANCE",
                                       ta_old + 20.0, ta_new + 20.0))
    print("  %-38s %-12.1f %-12.1f" % ("FIN 门 = T+FIN_ALT",
                                       ta_old + 10.0, ta_new + 10.0))
    print("  %-38s %-12.1f %-12.1f" % ("FIN 真实行程（减真触地 24.2）",
                                       ta_old + 10.0 - TOUCHDOWN,
                                       ta_new + 10.0 - TOUCHDOWN))
    print()
    print("  必须同步的文件：")
    print("    dev/starship/starship_config.py  TARGET_ALT 18 -> 26.5")
    print("    boot/superheavy.ks               local target_alt is 26.5")
    print("    GATE_ALT 由 TARGET_ALT 推导，自动跟着变（968 -> 976.5）")
    print()


def main():
    ok = sec1()
    sec2()
    sec3()
    sec4()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
