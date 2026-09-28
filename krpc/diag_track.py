# diag_track.py — 实机诊断 track() 的坐标/符号一致性
#
# 【背景】实飞日志（gfold_log_20260926_105737.csv）里 a_cmd 高达 847 m/s²
# （可用推力只有 38.7），throttle 恒饱和 1.0。参考系自转污染已修（vh 从
# 176 降到 70，是合理值），所以这一次是【另一个】问题。
#
# track() 里：
#     cur     = [tp[0], tp[1], tp[2]]          # kRPC 实测位置
#     x_ref   = x[0:3, j]                       # 求解器参考位置
#     pos_err = x_ref - cur
#     vel_err = v_ref - np.array(tv)
#     a_cmd   = u_ref + KP_X*pos_err + KV_V*vel_err
#
# 而 gfold_terminal 的初值约定是：
#     x0 = [h, dist, 0, vz, -vh, 0]
# 即【水平位置 y = +dist（恒正）】、【水平速度 vy = -vh（朝目标）】。
#
# 但 kRPC 的 tp/tv 是【本机相对目标】的带符号偏移/速度。
# 两者是否同号，取决于火箭此刻在目标的哪一侧：
#   · 在"北"侧 → tp[1] 与求解器 +dist 同号
#   · 在"南"侧 → tp[1] 与求解器 +dist 【反号】⇒ pos_err 凭空多出 2*dist
# 且水平速度方向也可能反号 ⇒ vel_err 多出 2*vh。
#
# 本脚本把这三者并列打印，直接看符号是否一致。
#
# 用法：python diag_track.py
import math
import sys

import krpc
import numpy as np

TARGET_LAT = -0.0972060951675948
TARGET_LON = -74.5576822740041
TARGET_ALT = 36.8
KP_X = 0.5
KV_V = 0.8


def main():
    conn = krpc.connect(name='diag_track')
    sc = conn.space_center
    v = sc.active_vessel
    body = v.orbit.body
    body_frame = body.reference_frame

    tgt = np.array(body.surface_position(TARGET_LAT, TARGET_LON, body_frame))
    n = float(np.linalg.norm(tgt))
    tgt = tgt + tgt / n * TARGET_ALT
    tgt_frame = sc.ReferenceFrame.create_relative(
        body_frame, position=tuple(float(x) for x in tgt))
    tf = sc.ReferenceFrame.create_hybrid(
        position=tgt_frame, rotation=v.surface_reference_frame,
        velocity=tgt_frame)

    tp = np.array(v.position(tf))
    tv = np.array(v.velocity(tf))
    fl = v.flight(tf)

    print('=' * 78)
    print('track() 坐标一致性诊断   vessel=%r' % v.name)
    print('=' * 78)
    print('\n[当前位置 tp = v.position(tgt_frame)]')
    print('  tp[0] (上)   = %10.2f   (应 ≈ alt - TARGET_ALT)' % tp[0])
    print('  tp[1] (北)   = %10.2f' % tp[1])
    print('  tp[2] (东)   = %10.2f' % tp[2])
    print('  水平距离     = %10.2f m' % math.hypot(tp[1], tp[2]))
    print('  由 alt 推      = %10.2f m' % (fl.surface_altitude - TARGET_ALT))

    print('\n[当前速度 tv = v.velocity(tgt_frame)]')
    print('  tv[0] (上)   = %10.2f' % tv[0])
    print('  tv[1] (北)   = %10.2f' % tv[1])
    print('  tv[2] (东)   = %10.2f' % tv[2])
    print('  水平速率     = %10.2f m/s' % math.hypot(tv[1], tv[2]))
    print('  flight.vertical_speed = %10.2f  (应与 tv[0] 同)' % fl.vertical_speed)

    # ---- 求解器初值约定 ----
    dist = math.hypot(tp[1], tp[2])
    vh = math.hypot(tv[1], tv[2])
    print('\n[求解器初值约定 x0 = [h, dist, 0, vz, -vh, 0]]')
    print('  h      = %10.2f  (= tp[0])' % tp[0])
    print('  dist   = %10.2f  (恒正，与 tp 的【带符号】偏移不同)' % dist)
    print('  vz     = %10.2f  (= tv[0])' % tv[0])
    print('  -vh    = %10.2f' % (-vh))

    # ---- 关键：把 tp 投影到"朝目标"方向，看符号 ----
    print('\n' + '=' * 78)
    print('符号一致性判定')
    print('=' * 78)
    if dist > 1e-6:
        # 单位向量：从本机指向目标 = -tp_h / |tp_h|
        u_hat = -np.array([tp[1], tp[2]]) / dist
        # 本机水平速度在"朝目标"方向的投影（正=正在接近目标）
        v_close = float(tv[1] * u_hat[0] + tv[2] * u_hat[1])
        print('  朝目标单位向量 (北,东) = (%+.4f, %+.4f)' % (u_hat[0], u_hat[1]))
        print('  水平速度在朝目标方向的投影 = %+.2f m/s' % v_close)
        print('    正 = 正在接近目标；负 = 正在远离')
        print()
        print('  求解器用 vy = -vh 表示"以 vh 朝目标飞" ⇒ 隐含 v_close = +vh')
        print('  实测 v_close = %+.2f  (vh = %.2f)' % (v_close, vh))
        if abs(v_close + vh) < 0.3 * max(1.0, vh) and vh > 1.0:
            print('  >>> 警告：实测水平速度与求解器约定【反号】！')
            print('      track() 的 vel_err 会凭空多出 2*vh = %.1f m/s' % (2 * vh))
        else:
            print('  >>> 符号一致（本机确实在朝目标飞）')

    # ---- 复现 track() 的 a_cmd ----
    print('\n' + '=' * 78)
    print('复现 a_cmd（用 j=0，即参考轨迹起点）')
    print('=' * 78)
    print('  若 j=0，x_ref 应等于当前状态 ⇒ pos_err/vel_err 都≈0')
    print('  pos_err = [%+.2f, %+.2f, %+.2f]'
          % (tp[0] - tp[0], dist - tp[1], 0.0 - tp[2]))
    print('  注意第 2 项：求解器的 x[1,0]=+dist，而 cur[1]=tp[1]=%+.2f' % tp[1])
    print('  ⇒ 凭空多出 %+.2f m 的"位置误差"' % (dist - tp[1]))
    print('  ⇒ KP_X×该值 = %+.2f m/s²' % (KP_X * (dist - tp[1])))
    print()
    print('  这就是 a_cmd 爆掉的来源：**求解器与 kRPC 的水平系符号不统一**。')


if __name__ == '__main__':
    sys.exit(main())
