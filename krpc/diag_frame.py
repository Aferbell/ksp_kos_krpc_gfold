# diag_frame.py — 诊断参考系语义（连 KSP 实机跑，不猜）
#
# 【背景】实飞日志出现两个物理上不可能的读数：
#   ① 下落 313 m 期间水平只走 163 m，但日志 vh 报 201~243 m/s
#      （用相邻两点位移反推，真实水平速率只有 23.3 m/s —— 差 8.6 倍）
#   ② vh 从 243 单调收敛到 168，而 Kerbin 在该纬度的自转线速度是
#      174.9 m/s —— 即"真实水平速度 →0"时 vh 收敛到【自转速度】
#   ⇒ 怀疑 v.velocity(target_frame) 里混进了天体自转分量。
#
# 本脚本把同一时刻的多种口径并列打印，用【自转速度 174.9】当标尺判定：
#   哪一口径在火箭悬停时趋近 0，哪一口径就是"相对目标点"的正确速度。
#
# 用法：python diag_frame.py
import math
import sys

import krpc
import numpy as np

TARGET_LAT = -0.0972060951675948
TARGET_LON = -74.5576822740041
TARGET_ALT = 36.8


def main():
    conn = krpc.connect(name='diag_frame')
    sc = conn.space_center
    v = sc.active_vessel
    body = v.orbit.body
    print(f'vessel = {v.name!r}  mass = {v.mass/1000:.1f} t')

    # 天体自转标尺
    omega = 2 * math.pi / body.rotation_period
    r_here = body.equatorial_radius
    v_rot = omega * r_here * math.cos(math.radians(v.flight().latitude))
    print(f'body = {body.name}  rotation_period = {body.rotation_period:.1f} s')
    print(f'该纬度自转线速度 = {v_rot:.2f} m/s   <<< 判定标尺')
    print()

    body_frame = body.reference_frame
    surf = v.surface_reference_frame
    orb = v.orbital_reference_frame

    # 目标系（与 gfold_land.py 完全一致）
    tgt = np.array(body.surface_position(TARGET_LAT, TARGET_LON, body_frame))
    n = float(np.linalg.norm(tgt))
    tgt = tgt + tgt / n * TARGET_ALT
    tgt_frame = sc.ReferenceFrame.create_relative(
        body_frame, position=tuple(float(x) for x in tgt))
    hybrid_body = sc.ReferenceFrame.create_hybrid(
        position=tgt_frame, rotation=v.surface_reference_frame,
        velocity=body_frame)
    # 候选修法：hybrid 的 velocity 传 tgt_frame 自己（原点系）
    hybrid_tgt = sc.ReferenceFrame.create_hybrid(
        position=tgt_frame, rotation=v.surface_reference_frame,
        velocity=tgt_frame)
    # 候选修法：velocity 传 surface 系
    hybrid_surf = sc.ReferenceFrame.create_hybrid(
        position=tgt_frame, rotation=v.surface_reference_frame,
        velocity=surf)

    frames = [
        ('body_frame           ', body_frame),
        ('surface_reference    ', surf),
        ('orbital_reference    ', orb),
        ('tgt_frame(relative)  ', tgt_frame),
        ('hybrid(vel=body) 现用', hybrid_body),
        ('hybrid(vel=tgt)      ', hybrid_tgt),
        ('hybrid(vel=surf)     ', hybrid_surf),
    ]

    print(f'{"参考系":<24} {"|pos|水平":>10} {"x(上)":>10} '
          f'{"水平速度":>10} {"vy":>9} {"vz":>9}')
    print('-' * 84)
    rows = {}
    for name, fr in frames:
        p = v.position(fr)
        vel = v.velocity(fr)
        hz = math.hypot(p[1], p[2])
        vh = math.hypot(vel[1], vel[2])
        rows[name] = (p, vel)
        print(f'{name:<24} {hz:10.1f} {p[0]:10.1f} {vh:10.2f} '
              f'{vel[1]:9.2f} {vel[2]:9.2f}')

    print()
    print('=== 判定 ===')
    for name, (p, vel) in rows.items():
        vh = math.hypot(vel[1], vel[2])
        if vh < 1.0:
            print(f'  {name}: 水平速度 {vh:.2f} m/s  <-- 几乎静止于目标?')
        else:
            d = abs(vh - v_rot)
            tag = ' <<< 疑似含自转' if d < 5.0 else ''
            print(f'  {name}: 水平速度 {vh:.2f} m/s (与自转差 {d:.1f}){tag}')

    print()
    print('提示：本机若在高速平飞，"水平速度"不会接近 0。')
    print('真正可靠的判据是【竖向速度】与 flight().vertical_speed 对照，')
    print('以及下面这个恒等式检查：')
    fl = v.flight(body_frame)
    print(f'  flight(body_frame).speed        = {fl.speed:.2f}')
    print(f'  flight(surface).speed           = '
          f'{v.flight(v.surface_reference_frame).speed:.2f}')
    print(f'  flight(orbital).speed           = '
          f'{v.flight(v.orbital_reference_frame).speed:.2f}')
    print(f'  flight(body_frame).vertical_speed = {fl.vertical_speed:.2f}')
    print()
    print('对照：kOS 的 ship:verticalspeed 与 krpc flight.vertical_speed 同口径。')


if __name__ == '__main__':
    sys.exit(main())
