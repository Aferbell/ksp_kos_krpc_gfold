#!/usr/bin/env python3
# gfold_land.py — B1040-9 末段 G-FOLD 着陆控制端（kRPC）
#
# ============================================================================
# 【本文件的结构全部照抄参考仓库 dev/gfold/demo3_gfold.py（GFOLD_KSP）】
#
#   【教训（坑㉖）】我此前自行发明了 tf 二分搜索、时间索引跟踪、投影对齐、
#   auto_pilot 接管等结构，连错三次（㉑㉒㉕），包括一次"火箭横飞"坠毁。
#   参考仓库是【唯一验证过的 G-FOLD 实飞实现】，能用它的就用它的。
#   现在逐项对应关系：
#       参考 demo3_gfold.py            本文件
#       ─────────────────────────      ──────────────────────────────
#       GFOLD_direct_exec.py (P3/P4) → dev/gfold/solver/gfold_p3p4.py
#       ref_target (line 224-225)    → target_frame()
#       find_nearest_index (258-270) → find_nearest_index()
#       sample_index (273-290)       → sample_index()
#       conic_clamp (292-314)        → conic_clamp()
#       G-FOLD 跟随 (360-412)        → track()
#       final 末段 (413-427)         → run() 里的 nav_mode=='final'
#       PID 姿态 (461-477)           → PID 类 + apply()
#       params.txt                   → 文件顶部常量（逐项标注"参考"）
#
# ======================= 它在整个架构里的位置 =======================
#   段1（12.4 km → ~5 km）  : kOS  boot/B1040-9.ks 两级律，负责把火箭带进可行域
#   段2（~5 km → 地面）     : 本程序（kRPC + G-FOLD 凸优化），负责精确落点 + 姿态
#
#   交接机制见 dev/krpc/HANDOFF.md。kOS 侧满足交班门后打印交接行、
#   unlock 并【直接结束脚本】；本程序检测到状态满足门限后接管
#   control.throttle 与 control.pitch/yaw/roll（自写 PID，不用 auto_pilot）。
#
# ======================= 已验证的事实（非猜测）=======================
#   · kRPC API 全部对照 GameData/kRPC/KRPC.SpaceCenter.xml 与官方 stub 核实；
#     并用 tools/check_krpc_api.py 做飞行前静态核对（坑㉓ 的教训）
#   · P3/P4 两级求解实测：交班点 tf_m≈20.5 s、落地 137 t、末端倾角 0.00°、
#     耗时 ~3.5 s
#
# ======================= 控制策略（照抄参考仓库）=======================
#   ① 交班瞬间：P3 估落地时刻 tf_m → P4 在 tf 上求燃料最优（~3.5 s）
#   ② 之后：沿参考轨迹做 PD 跟随，索引由 find_nearest_index 给出
#   ③ 进入目标圆柱区（final_radius/final_height）后切 final 段直接 PID
#   姿态：自写 PID 直接写 control.pitch/yaw/roll（参考仓库做法，不用 auto_pilot）
#
# ======================= 日志 =======================
#   根目录 log-B1040-9-gfold/ ，每次运行一个 CSV，供离线分析。
#
# 用法：
#   python gfold_land.py                # 正常接管
#   python gfold_land.py --dry-run      # 只解算+画线，不接管控制（推荐首飞）
#   python gfold_land.py --no-autopilot # 只接管节流，姿态手动
import argparse
import csv
import math
import os
import re
import sys
import threading
import time
from typing import TYPE_CHECKING
from datetime import datetime

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
# 【2026-09-28 求解器路径变更】
#   原先指向 ../gfold/solver —— 那是【上游参考仓库的克隆目录】，
#   而 gfold_p3p4.py / gfold_codegen.py 其实是【本项目自己写的】
#   （上游 GFOLD_KSP 并未跟踪它们，见 dev/.gitignore 说明）。
#   为了让它们能被版本控制，已复制到 dev/solver/，这里同步改路径。
#   【两个位置都可用的写法】先找 ../solver（本项目仓库内），
#   找不到再回退 ../gfold/solver（旧的克隆目录），保证两者都能跑。
for _cand in (os.path.join(_HERE, '..', 'solver'),
              os.path.join(_HERE, '..', 'gfold', 'solver')):
    if os.path.isdir(_cand):
        sys.path.insert(0, _cand)

from gfold_p3p4 import solve_p3p4_state        # noqa: E402 参考仓库结构

# ------------------------- 常量（来自项目文档，非猜测）-------------------------
# 【标准重力加速度】2026-09-26 修：此前本文件用了 4 处 G0，但 G0 是随
#   `from gfold_terminal import ...` 一起被删掉的（改成 gfold_p3p4 后没补），
#   实飞直接 NameError 崩溃（line 1429 hold_up）。
#   参考仓库 demo3_gfold.py 里 G0 来自 params['g0']=9.807；求解器 gfold_p3p4
#   内部用 9.80665。这里统一用求解器同值，避免控制侧与规划侧口径不一致。
G0 = 9.80665

TARGET_LAT = -0.0972060951675948      # 回收场（与 kOS 侧 tgt_lat 一致）
TARGET_LON = -74.5576822740041
TARGET_ALT = 36.8                     # 着陆腿触点高度（雷达高度基准）
TARGET_HEADING_DEG = 90.0             # 机头朝东（与 kOS 发射段一致）

# 交班门（与 boot/B1040-9.ks 的 gfold_h / gfold_vmax / gfold_cone 对齐）
# ============ 【2026-09-27 按用户要求：交接对齐"瞄准点"】============
# 【用户要求】"瞄准点是在目标点上3000米，在一级到达瞄准点附近时开始交接"
#
# 【流程】
#   瞄准点绝对高度 = TARGET_ALT + AIM_ALT = 36.8 + 500 = 536.8 m（2026-09-28 起）
#   段1（kOS E-guidance）的律 a = r_tgt·(6/T²) − v·(4/T) 的意图就是
#   【在瞄准点把位置与速度同时归零】⇒ "到达瞄准点附近"≡ h_aim ≤ GATE_AIM_H
#
# 【旧判据的错】用 alt < 5000 ⇒ 交班时 h_aim 还有 2013 m，一级离瞄准点
#   还差 2 km 就被交出去，段1 来不及收速 ⇒ kRPC 接手时仍带 363 m/s、
#   水平偏差 1335 m ⇒ PD 位置项压过轨迹指令 ⇒ target_a 竖直分量变负
#   ⇒ conic_clamp 返回纯竖直 ⇒ a_cmd_h=0 ⇒ **完全不跟踪**
#
# 【新判据】alt < AIM_ALT + TARGET_ALT + GATE_AIM_H  （2026-09-28 起 = 686.8 m）
#   【可行性】段1 时标 T = 2·h_aim/|vz| 在交班点很短（h_aim≈150, |vz|≈70
#   ⇒ T≈4.3 s），需减速度 < 可用 a_net ≈ 28.9 m/s² ⇒ 收速可行。
#   【2026-09-28 旧值】原 AIM_ALT=3000 ⇒ 门 3186.8 m、T=14.0 s、需 26.0 m/s²。
# 【2026-09-28 第三次调整：AIM_ALT 3000 -> 500】
#   【必须与 boot/B1040-9.ks 的 aim_alt 保持一致】两边都描述同一个瞄准点，
#   改一边不改另一边会让交班高度与 kOS 的 h_aim 判据错位。
#   【理由】原 3000 m 假设"段1 在瞄准点把速度收到 0"；实飞证明段1 交班时
#   横速仍有 35~45 m/s，该假设不成立。降到 500 m 后：
#     · G-FOLD 入口高度 ~687 m（原 ~3180 m），tf 24.5 -> 10.2 s
#     · 入口动能更低，风扰/自转漂移累积时间更短
#     · 离线实测落地剩余质量 154.7 t（原 144.2 t，多剩 10 t）
#   【注意】GATE_ALT_MIN=358 保持不变；新门 686.8 m 距它只有 328.8 m 裕度，
#     而段1 现在还要额外消掉 35 m/s 横速（详见 kOS 侧 vh_gate 注释）。
#     这是本次改动的最大风险点。
AIM_ALT = 500.0                       # 瞄准点抬高量 [m]（与 kOS aim_alt 一致）
GATE_AIM_H = 150.0                    # "到达瞄准点附近"的容差 [m]
GATE_ALT = TARGET_ALT + AIM_ALT + GATE_AIM_H      # = 686.8 m
GATE_VMAX = 120.0                     # 段1 在瞄准点已把速度收到 0，
                                      #   故门槛收紧（旧 480 太松）
GATE_CONE_DEG = 48.0                  # 保持 48°
GATE_ALT_MIN = 358.0                  # 可行域下沿（再低 G-FOLD 无解）

# G-FOLD 求解参数（**逐项对照参考仓库 dev/gfold/params.txt**）
#   参考仓库用 params.txt + GFOLD_params.py，这里把它们落成常量。
#   凡是标注"参考"的就是原值；标注"本项目"的是为 B1040-9 改的。
#   （原 SOLVER dict 与 gfold_terminal 的 term_* 参数已随"照抄参考仓库"
#     一并删除 —— 现在求解走 gfold_p3p4.solve_p3p4，参数即下方这几项。）

# ---- 参考仓库 params.txt 对应项 ----
ISP_DEFAULT = 315.0        # 比冲 [s]
TF_GUESS = 40.0            # 参考 tf=20；本项目需更大（见 gfold_p3p4 文档）
STRAIGHT_FAC = 5.0         # 参考同值
GS_DEG = 30.0              # 参考 y_gs=30
PCS_START_DEG = 85.0       # 本项目补充（参考用固定 p_cs；见坑①）
PCS_END_DEG = 30.0         # 参考 p_cs = max_tilt*0.85 ≈ 21；本项目 30
V_MAX_SOLVER = 1200.0      # 参考 V_max=150（不适配本项目入场速度）
N3 = 160                   # 参考 GFOLD_params.py
N4 = 80                    # 参考 GFOLD_params.py
# 参考 params.txt 的 start_altitude：低于该高度才允许规划/重解。
#   【2026-09-27】设为交班门（GATE_ALT，现为 686.8 m）—— 它只是"允许重解"的
#   附加条件，而现在默认已不重解（--replan-dt 0）；设为 GATE_ALT 保持语义一致。
START_ALTITUDE = GATE_ALT

# 是否画参考轨迹线（对齐参考仓库 params.txt 的 debug_lines，它默认 True）。
#   【2026-09-26 修正】旧版只在 --dry-run 时画，导致正常飞行看不到线。
DEBUG_LINES = True


LOG_DIR = os.path.join(
    r'D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script',
    'log-B1040-9-gfold')

# 跟踪增益（**数值直接来自参考仓库 params.txt**，非自创）
#   params.txt:  k_x = 0.5   k_v = 0.8
#   对应参考仓库 demo3_gfold.py line 378:
#       target_a = u_i + (v_i - vel) * k_v + (x_i - error) * k_x
#
# 【命名的教训】旧名 KP_X / KV_V 有歧义（P 到底是位置的 P 还是别的），
#   我因此把两个增益【接反】用了很久（速度项 0.5 / 位置项 0.8），
#   导致欠阻尼（ζ 从 1.13 掉到 0.56）和姿态乱动。
#   改名成 K_POS（位置项）/ K_VEL（速度项），与参考仓库 k_x / k_v 一一对应，
#   名字里就写明"它乘在谁身上"，杜绝再次接反。
K_POS = 0.5     # = 参考 params.txt 的 k_x  —— 乘在【位置误差】上
K_VEL = 0.8     # = 参考 params.txt 的 k_v  —— 乘在【速度误差】上
# 旧名保留为别名，避免遗漏的引用直接 NameError（下个版本删）
KP_X = K_POS
KV_V = K_VEL
REPLAN_DT = 1.0

# 【参考仓库的推力锥限幅】demo3_gfold.py 用 params.txt 的 max_tilt = 25°。
#   这个角的语义是"推力轴相对竖直的最大倾角"，用于 conic_clamp 把 PD 修正后的
#   加速度指令限幅回物理可执行的锥内（参考仓库 line 389-390）。
#   本项目取 25°（与参考 txt 一致），而不是我此前自己拍的 12°（那是 B1040-7
#   终端段的 term_tilt，属于【另一个阶段】的量，不可混用）。
CONIC_TILT_DEG = 25.0

# ---- 推力锥限幅的锥角来源（2026-09-27 澄清：不是"时序收紧"）----
# 【曾经的误解】此处的注释一度写成"按时序从 85° 收到 25°"，并定义了
#   常量 CONIC_TILT_START_DEG = 85.0。**该常量从未被任何代码读取**，
#   那个"时序锥"实现并不存在 —— 属于注释与实现不符，已在 2026-09-27 删除，
#   以免后人以为存在这条逻辑。
#
# 【实际实现（正确且更自洽）】见 track() 里：
#       tilt_traj = atan2(|u_i[1:3]|, u_i[0])      # 参考轨迹【当前节点】的倾角
#       tilt_now  = max(tilt_traj + 8.0, CONIC_TILT_DEG)
#   —— 即限幅锥角 = 【参考轨迹自己的倾角】+ 8° 裕度，且不小于末端姿态要求
#   CONIC_TILT_DEG(25°)。
#
# 【为什么这样做才对】参考仓库能用固定 25°，是因为它的轨迹由【同一个 25° 锥】
#   解出来（p_cs = max_tilt·0.85），轨迹倾角天然 ≤ 25°，锥与轨迹自洽。
#   本项目为了让"大横速高空入场"可行，求解器用了 pcs_start=85°→pcs_end=30°
#   的时变锥，于是轨迹倾角也时变。若限幅仍用固定 25°，就会与轨迹自相矛盾：
#   轨迹要求 70°、限幅只给 25° ⇒ 水平制动被砍 58% ⇒ 横飞。
#   ⇒ 限幅必须跟随【轨迹本身】，而不是跟随某个虚构的时序。


# ---- 姿态 PID（**数值直接来自参考仓库 params.txt**）----
#   ctrl_x_rot.kp = 5   ctrl_x_rot.kd = 2.5     (pitch)
#   ctrl_z_rot.kp = 5   ctrl_z_rot.kd = 2.5     (yaw)
#   ctrl_y_avel_kp = 2                          (roll，直接消除角速度)
CTRL_X_ROT_KP = 5.0
CTRL_X_ROT_KD = 2.5
CTRL_Z_ROT_KP = 5.0
CTRL_Z_ROT_KD = 2.5
CTRL_Y_AVEL_KP = 2.0

# ================================================================
# ---- 等待首解期间的减速保持（hold 段）----
# 用户要求："瞄准速度反方向加油门继续减速，但不能太大以至于反冲"。
# 公式取自参考仓库 demo3_gfold.py 的 simple 模式（line 428-435），
# 外加两道【反冲保护】：
HOLD_UP_FRAC = 0.4    # 竖直推力分量上限 = g0 + 0.4·(a_cap - g0)
                      #   【为什么是 0.4】首解要 ~3.6 s，hold 期间烧的油
                      #   直接吃掉落地质量（总预算约 42 t）。实测权衡：
                      #     f=0.75 -> 4 s 烧 9.5 t，占总燃料 22%（太多）
                      #     f=0.5  -> 烧 5.5 t
                      #     f=0.4  -> 烧 ~4.4 t，竖直净减 ~23 m/s²，
                      #               且【水平也在减速】（旧版水平完全不管）
HOLD_TAU = 1.0        # 净竖直减速 <= |vz| / HOLD_TAU
                      #   （即"至少留 1 s 才把垂速消到 0"，防止 vz 穿越 0 反冲）

# ---- 接管校验（非阻塞采样）----
# 旧版是阻塞 0.5 s（sleep 0.05×10）且跑在主循环之前 ⇒ 那 0.5 s 无推力。
# 改成逐帧采样：不占额外时间。前 12 帧跳过（写入尚未生效），
# 共采 TAKEOVER_SAMPLES 个有效样本，不符比例 >=60% 判为未接管。
TAKEOVER_SAMPLES = 40

# ---- 最终降落段（**数值直接来自参考仓库 params.txt**）----
FINAL_RADIUS = 20.0      # 触发区域半径 [m]
FINAL_HEIGHT = 200.0     # 触发区域高度 [m]
FINAL_THROTTLE = 0.8
FINAL_KP = 1.0
# 【2026-09-28 本项目新增：final 段【油门下限】—— 参考仓库没有这个量】
#   【为什么必须加】参考仓库的 final 油门下限沿用 throttle_limit_ctrl[0]=0.05，
#   它假设载具在 final 段靠【RCS / 反作用轮】保持姿态（与主油门无关）。
#   本载具（B1040-9，一级 155 t）不是这样：
#     · 52 个 ModuleRCSFX + 5 个反作用轮确实存在，但按 I≈1.16e7 kg·m^2
#       估算，RCS 总力矩 ~3000 N·m 只能给 ~0.01 deg/s^2 —— 远远不够。
#     · 实测姿态权限【与油门强相关】（两趟实飞一致）：
#         油门 <=0.1 : 中位角速率 6.7 ~  9.6 deg/s
#         油门 >=0.7 : 中位角速率 30.8 ~ 54.3 deg/s      (慢 8 倍)
#       ⇒ 本载具实际靠【发动机摆动(gimbal)】获得姿态权限，
#         而 gimbal 力矩正比于油门：thr=0.05 时只有满油门的 1/20。
#   【硬约束：下限不能让载具无法下降】
#       下限给出一个【最小净加速度】 a_min = floor*T/m - g0。
#       必须 a_min < 0，否则光靠下限就把火箭往上推，根本无法下降。
#       floor < g0*m/T。final 段实际质量 149~154 t：
#           m=154.4 t -> floor_max = 0.1684
#           m=149.0 t -> floor_max = 0.1625      <- 全程最紧
#       ⇒ 下限必须 < 0.16，且要留裕度（燃油烧掉后 m 更小、更紧）。
#   · 曾误取 0.30：在 154 t 时 a_min = +7.66 m/s^2，
#     【下限本身就在把火箭往上推】。离线仿真在细时间步下直接失控
#     （floor>=0.34 时落地 vz 变成 +350~+730 m/s，火箭反冲上天）。已改正。
#   · 【本次取值 0.12】：a_min = -2.82 m/s^2（154 t）/ -2.19（149 t），
#     全程保持"能下降"，同时把低油门带的最低权限抬出 0.05 档。
FINAL_THROTTLE_MIN = 0.12   # final 段油门下限（参考仓库为 throttle_limit_ctrl[0]=0.05）

# ---- 终端下降段：**逐项照抄 boot/B1040-7.ks**，只改载具相关的量 ----
TERM_H = 15.0         # 终端门高度 [m]（B1040-7: 15 → 照抄）
TERM_V = 2.0          # 触地下降率 [m/s]（B1040-7: 1.5 → 本项目指标 -2）
TERM_K = 2.0          # 竖直 P 增益（时间常数 1/TERM_K）
TERM_KP = 0.8         # 水平位置环 P（B1040-7: 0.8 → 照抄）
TERM_KD = 1.8         # 水平速度环 D（B1040-7: 1.8 → 配 kp 得 ζ≈1）
TERM_AMAX = 20.0      # 终端段可用加速度上限 [m/s2]
TERM_TILT = 12.0      # 终端段推力轴倾角上限 [deg]
TILT_SPAN = 25.0      # 锥角收缩跨度 [deg]
EG_TMIN = 4.0         # B1040-7: 4 → 照抄（时标下限，防 1/T² 末端炸掉）

def _clamp(num, maxnum, minnum):
    """原版 clamp()。注意【参数顺序是 max 在前】，移植时保持原样以免搞错。"""
    if num > maxnum:
        return maxnum
    elif num < minnum:
        return minnum
    return num


def _lerp(vec1, vec2, t):
    """原版 lerp()。"""
    return t * vec2 + (1 - t) * vec1


def _sgn(f):
    """原版 sgn()。"""
    if f > 0:
        return 1
    elif f < 0:
        return -1
    return 0


def _normalize(vec):
    """原版 normalize()。"""
    return vec / np.linalg.norm(vec)


def _angle_around_axis(v1, v2, axis):
    """原版 angle_around_axis()（demo3_gfold.py line 79-84）。"""
    axis = _normalize(axis)
    v1 = _normalize(np.cross(v1, axis))
    v2 = _normalize(np.cross(v2, axis))
    direction = _sgn(np.dot(np.cross(v1, v2), axis))
    return direction * math.acos(max(-1.0, min(1.0, float(np.dot(v1, v2)))))


def _rotation_mat(q):
    """原版 rotation_mat()（demo3_gfold.py line 67-73）。q=(x,y,z,w)。"""
    (x, y, z, w) = q
    return np.array([
        [1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y + 2 * w * z,
         2 * x * z - 2 * w * y],
        [2 * x * y - 2 * w * z, 1 - 2 * x ** 2 - 2 * z ** 2,
         2 * y * z + 2 * w * x],
        [2 * x * z + 2 * w * y, 2 * y * z - 2 * w * x,
         1 - 2 * x ** 2 - 2 * y ** 2],
    ])


def _transform(vec, mat):
    """原版 transform()（demo3_gfold.py line 75-77）。"""
    res = np.asarray(vec) @ mat
    return np.array([res[0], res[1], res[2]])

class PID:
    """逐行照抄参考仓库 demo3_gfold.py 的 PID 类（line 100-137）。

    原版行为：
        update(error, dt):
            首帧只记录 error_prev；第二帧算 diff
            integral += error*dt*ki, 并用 integral_limit 限幅
            diff = lerp(diff, (error-error_prev)/dt, 1-sd)
            p = -error*kp ; i = -integral ; d = -diff*kd
            result = p*(ep) + i*(ei) + d*(ed)
    注意原版返回的 result 【已经带了负号】，调用方又取负（control_pitch = -clamp(...)），
    所以净效果是"正误差 -> 正控制量"。移植时【保持原样】，不做简化。
    """

    def __init__(self):
        self.ep = True
        self.ei = True
        self.ed = True
        # 【pyright 报的类型问题】原版写 kp=1 / kd=1（int），但外部会赋
        #   CTRL_X_ROT_KP = 5.0 这类 float。Python 本身不在乎，
        #   但把初值写成 float 才能让类型检查通过、也避免"整数除法"之类的隐患。
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 1.0
        self.sd = 0.0
        self.diff = 0.0
        self.integral = 0.0
        self.integral_limit = 1.0
        self.error_prev = 0.0
        self.first = True
        self.second = True
        self.result = 0.0

    def update(self, error, dt):
        if self.first:
            self.first = False
            self.error_prev = error
        elif self.second:
            self.second = False
            self.diff = (error - self.error_prev) / max(1e-9, dt)

        self.integral += error * dt * self.ki
        self.integral = _clamp(self.integral, self.integral_limit,
                               -self.integral_limit)
        self.diff = _lerp(self.diff, (error - self.error_prev) / max(1e-9, dt),
                          1 - self.sd)
        p = -error * self.kp
        i = -self.integral
        d = -self.diff * self.kd
        self.result = (p * (1 if self.ep else 0) + i * (1 if self.ei else 0)
                       + d * (1 if self.ed else 0))
        self.error_prev = error
        return self.result

class Logger:
    """写 CSV 到 log-B1040-9-gfold/，列固定，便于离线分析。"""

    COLS = ['t', 'wall', 'alt', 'dist_hz', 'vz', 'vh', 'vmag', 'mass',
            'thr_cmd', 'thr_act', 'a_cmd_up', 'a_cmd_h', 'a_cmd_mag',
            'tilt_cmd', 'tilt_dir_cmd', 'tilt_act', 'att_err',
            'gnc_phase', 'replan_n',
            'solve_ms', 'gfold_status', 'tf', 'tgo', 'h_min_plan',
            'x_err', 'v_err',
            # ---- ① 真实姿态（目标系分量）----
            'dir_vx', 'dir_vy', 'dir_vz',
            # ---- ② 姿态目标方向（目标系分量）----
            'tgt_vx', 'tgt_vy', 'tgt_vz',
            # ---- ③ 参考轨迹采样点 + 真实跟踪误差 ----
            'n_i', 'plan_alt', 'plan_vz', 'plan_vh',
            'trk_pos', 'trk_pos_up', 'trk_pos_h', 'trk_vel',
            # ---- ④ 姿态权限与循环时标 ----
            'loop_dt', 'rpc_ms',
            'tau_p', 'tau_r', 'tau_y', 'I_p', 'I_r', 'I_y',
            'alpha_max', 'avel_mag', 'tilt_rate',
            # ---- ⑤ PID 内部量 ----
            'pid_err_x', 'pid_err_z',
            'pid_px', 'pid_dx', 'pid_ix',
            'pid_out_x', 'pid_out_z', 'pid_out_r',
            'avel_p', 'avel_r', 'avel_y',
            # ---- ⑥ 姿态限速器 ----
            'slew_alpha', 'slew_wmax', 'slew_act', 'slew_err',
            'note']

    def __init__(self, enabled=True, name=None):
        self.enabled = enabled
        self.rows = []
        self.notes = []
        os.makedirs(LOG_DIR, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(LOG_DIR, name or f'gfold_log_{stamp}.csv')

    def note(self, msg, echo=True):
        """记录一条注释行。

        【2026-09-28 加 echo 开关（用户要求不要刷屏）】
          note() 默认【既写日志又打控制台】。但有些注释是【批量/周期性】的
          （例如 log_vessel_table 每 5 s 写 3 行候选船），打控制台会把终端
          刷满、淹没真正的告警。这类调用传 echo=False 只落盘。
        """
        self.notes.append(f'# {msg}')
        if echo:
            print(f'[note] {msg}')

    def row(self, **kw):
        if not self.enabled:
            return
        self.rows.append({c: kw.get(c, '') for c in self.COLS})

    def flush(self):
        if not self.enabled:
            return
        try:
            with open(self.path, 'w', newline='', encoding='utf-8') as f:
                f.write('\n'.join(self.notes) + '\n')
                w = csv.DictWriter(f, fieldnames=self.COLS)
                w.writeheader()
                w.writerows(self.rows)
            print(f'[log] {len(self.rows)} 行 -> {self.path}')
        except OSError as e:
            print(f'[log] 写入失败: {e}')

def geodetic_to_local(lat, lon, tgt_lat, tgt_lon, body_radius):
    """把 (lat,lon) 相对目标的偏移换算成当地 ENU 的 (east, north) [m]"""
    dlat = math.radians(lat - tgt_lat)
    dlon = math.radians(lon - tgt_lon)
    north = dlat * body_radius
    east = dlon * math.cos(math.radians(tgt_lat)) * body_radius
    return east, north


def great_circle_m(lat1, lon1, lat2, lon2, body_radius=600000.0):
    """球面大圆距离 [m]（Kerbin 半径 600 km，与 kOS 侧口径一致）"""
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = (math.sin(dp / 2) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2)
    return 2.0 * body_radius * math.asin(min(1.0, math.sqrt(a)))

class GfoldLander:
    # 【类型标注（pyright 排查后补）】pyright 默认把 `self.v = None` 推成
    #   "永远是 None"，于是后面 42 处 `self.v.xxx` 全报
    #   reportOptionalMemberAccess —— 真正的问题（PID int/float、plan 可能
    #   为 None）被淹没在噪音里。
    #   这里用"运行时为 None、类型上是 krpc 对象"的标准写法：
    #   先声明类型，再在 __init__ 里赋 None 并标 ignore。
    if TYPE_CHECKING:
        from krpc.services.spacecenter import (
            CelestialBody as _Body, ReferenceFrame as _Frame, Vessel as _Vessel)
        v: _Vessel
        body: _Body
        ref: _Frame

    def __init__(self, conn, args):
        self.conn = conn
        self.sc = conn.space_center
        self.args = args
        self.log = Logger(enabled=not args.no_log)

        # 【2026-09-25 空引用崩溃修复】此前 v/body/ref/plan 都是在【用到时】
        #   才由 pick_vessel()/bind() 赋值，但 __init__ 之后、bind() 之前
        #   （例如日志初始化、异常路径）就可能被访问，导致脚本当场崩溃。
        #   这里全部显式初始化为 None，让"未绑定"是一个明确状态。
        self.v = None             # type: ignore[assignment] 由 pick_vessel() 绑定
        self.body = None          # type: ignore[assignment]
        self.ref = None           # type: ignore[assignment]
        self.plan = None
        self.replan_n = 0
        self.last_replan = 0.0
        self.engaged = False
        self.solve_ms = 0.0
        self.gfold_status = 'none'
        self.tilt_cap = 0.0
        self.line = None
        # 自转污染自检用的上一帧状态
        self._last_pos = None
        self._last_t = 0.0
        self._rot_warned = False
        self.vrot_est = 0.0
        self.ap_engaged = False
        self.att_err = float('nan')
        # 【2026-09-27 新增】真实姿态向量 / 姿态目标向量 / 真实姿态夹角。
        #   全部在 surface 参考系下；用于回答"姿态是没跟上还是转反了"。
        self.dbg_dir = None          # 机头真实方向（surface 系，单位向量）
        self.dbg_tgt = None          # 目标方向（surface 系，单位向量）
        self.dbg_aim_err = float('nan')
        # 【2026-09-27 新增】参考轨迹采样点与真实跟踪误差（见 track() 赋值）。
        self.dbg_n_i = float('nan')
        self.dbg_plan_alt = float('nan')
        self.dbg_plan_vz = float('nan')
        self.dbg_plan_vh = float('nan')
        self.dbg_trk_pos = float('nan')
        self.dbg_trk_pos_up = float('nan')
        self.dbg_trk_pos_h = float('nan')
        self.dbg_trk_vel = float('nan')
        # 【2026-09-28 新增】姿态权限与循环时标（见 Logger.COLS ④ 的说明）。
        self.dbg_loop_dt = float('nan')
        self.dbg_rpc_ms = float('nan')
        self.dbg_tau_p = float('nan')
        self.dbg_tau_r = float('nan')
        self.dbg_tau_y = float('nan')
        self.dbg_I_p = float('nan')
        self.dbg_I_r = float('nan')
        self.dbg_I_y = float('nan')
        self.dbg_alpha_max = float('nan')
        self.dbg_avel_mag = float('nan')
        self.dbg_tilt_rate = float('nan')
        # ---- ⑤ PID 内部量 ----
        self.dbg_pid_err_x = float('nan')
        self.dbg_pid_err_z = float('nan')
        self.dbg_pid_px = float('nan')
        self.dbg_pid_dx = float('nan')
        self.dbg_pid_ix = float('nan')
        self.dbg_pid_out_x = float('nan')
        self.dbg_pid_out_z = float('nan')
        self.dbg_pid_out_r = float('nan')
        self.dbg_avel_local = None
        self._prev_tilt_act = float('nan')
        self._att_warned = False
        self.last_pkt = 0.02      # 上一帧游戏时间（供 track 的索引推进用）
        # 状态外推时长 [s]（参考仓库 vessel_profile1 的 est_time=0.5）。
        self.est_time = 0.5
        self.debug_lines = bool(DEBUG_LINES and not args.no_debug_lines)
        self.drawn = []
        self._lines_N = None
        self._lines_fr = None
        self._draw_warned = False
        self.prev_vel = None
        self._solve_thread = None
        self._pending_plan = None
        self._pending_msg = None
        self.gfold_n_i = -100     # 参考仓库的路径索引 n_i（初始 -100 表示未定位）
        # 【轨迹是否可信】等价于参考仓库的 `n_i < 0` 判据（见 track() 说明）。
        self.gfold_traj_ready = False
        self.target_direction = None
        self.tilt_cmd = 0.0
        # ---- ⑥ 姿态限速器状态（此机制已按用户要求移除，字段保留兼容）----
        self._slew_prev_dir = None
        self._slew_alpha = None
        self._slew_active = False
        self._slew_wmax = float('nan')
        self._slew_err = 0.0
        # 【姿态目标的真实倾角】tilt_cmd 是节流向量 target_a 的倾角，
        #   而姿态追的是前瞻向量 target_a_ —— 两者不同，必须分开记。
        self.tilt_dir_cmd = 0.0
        self.nav_mode = 'none'    # 参考仓库的三态: none / gfold / final

        # ---- PID（参考 params.txt: ctrl_x_rot/ctrl_z_rot kp=5 kd=2.5）----
        self.ctrl_x_rot = PID()
        self.ctrl_x_rot.kp = CTRL_X_ROT_KP
        self.ctrl_x_rot.kd = CTRL_X_ROT_KD
        self.ctrl_z_rot = PID()
        self.ctrl_z_rot.kp = CTRL_Z_ROT_KP
        self.ctrl_z_rot.kd = CTRL_Z_ROT_KD

    def bind(self, v):
        """绑定到某艘船（设置 body/ref）"""
        self.v = v
        self.body = v.orbit.body
        self.ref = self.body.reference_frame

    def describe(self, v):
        """读一艘船的关键量：(name, mass, thrust, a_net, alt, vspeed, type, situation)"""
        body = v.orbit.body
        fl = v.flight(body.reference_frame)
        ms = v.mass
        th = v.max_thrust
        a_net = (th / ms - 9.80665) if ms > 1 else float('-inf')
        return dict(v=v, name=v.name, mass=ms, thrust=th, a_net=a_net,
                    alt=fl.surface_altitude, vspeed=fl.vertical_speed,
                    type=v.type.name, situation=v.situation.name)

    def log_vessel_table(self):
        """把场景内所有船写进日志（诊断用，便于事后定位选错的原因）。

        【2026-09-28 按用户要求：不刷控制台】
          本函数在等待交班期间【每 5 s 调用一次】，每次写 3 行。
          原先走 note() 默认会 print 到控制台 ⇒ 终端被候选表刷满，
          真正的告警（如接管失败）被淹没。改成 echo=False 只落盘；
          需要看候选表时用 --list-vessels。
        """
        for v in list(self.sc.vessels):
            try:
                d = self.describe(v)
                self.log.note(
                    f"vessel-cand {d['name']!r} type={d['type']} "
                    f"sit={d['situation']} mass={d['mass']/1000:.1f}t "
                    f"thrust={d['thrust']/1e6:.2f}MN a_net={d['a_net']:.1f} "
                    f"alt={d['alt']:.0f} vs={d['vspeed']:.1f}",
                    echo=False)
            except Exception:                      # noqa: BLE001
                pass

    @staticmethod
    def _reject(d):
        """返回排除原因（None = 保留）。**只用物理量，不用 type/name。**

        【为什么彻底不用 type（2026-09-26 实飞踩坑，两次）】
          第一次我按 type 排除了 debris，但没排除 relay；
          第二次我加了"排除 relay"，结果把【真正的一级】也排除了 ——
          实飞证据：
            kOS 交班行: type=Relay name=B1040-9 mass=192.7 t alt=4941.1 v=478.8
            kRPC 候选:  type=relay        mass=185.7 t alt=4256   vs=-288.4
          质量差 7 t、高度差 685 m —— 完全符合"下落中持续烧油"，
          ⇒ 这就是同一艘船。**玩家在 VAB 里能把一级的 type 设成 Relay**，
          所以 type 完全不可信。
        【结论】判据只用：
          · 质量：> 20 t（空壳/碎片/探针排除；一级 220→150 t）
          · 高度：100 ~ 22000 m（排除在轨卫星与地面）
          · 下降：vspeed <= 5 m/s
          · a_net 上界 80（排除推重比异常的）
          · 质量下限不再卡 a_net：因为中继卫星型的"一级"也可能 a_net 偏低，
            真正的区分交给【交班点匹配】做。
        """
        if d['mass'] < 20000.0:                    # 20 t
            return f"mass={d['mass']/1000:.1f}t too light"
        if not (100.0 < d['alt'] < 22000.0):
            return f"alt={d['alt']:.0f} out of (100,22000)"
        if d['vspeed'] > 5.0:
            return f"vspeed={d['vspeed']:.1f} (climbing)"
        if d['a_net'] > 80.0:
            return f"a_net={d['a_net']:.1f} > 80"
        return None

    def pick_vessel(self, verbose=False, handoff=None):
        """选出【正在回收的一级】。优先级：交班点匹配 > 物理特征筛选。

        【为什么不能用 active_vessel，也不能只按名字】
          坑①：一级分离后 active_vessel 变成二级 —— 拿到 726~761 t 的整箭/
               二级，a_net ≈ 1.3~1.9 m/s²，G-FOLD 必然 infeasible。
          坑②：按名字也不行。--list-vessels 显示场景里
               'B1040-9'、'B1040-9 中继卫星'、'B1040-9的残骸' 互相包含；
               且 KSP 让二级/载荷继承原 vessel id，真正的回收一级是新 vessel。
          坑③：kRPC 常在火箭【还没分离】时启动，那一刻选到的只是"整箭"。

        【所以两条路并用】
          1) 若有 kOS 交班行（含精确经纬度）→ 按位置匹配（最可靠）
          2) 否则按物理特征筛：a_net 15~80、正在下降、高度 100~25000 m、非 debris
        """
        if handoff:
            v = self.match_by_handoff(handoff)
            if v is not None:
                return v
            if verbose:
                print('[vessel] 交班点匹配失败，回退物理特征筛选')
        cands = []
        for v in list(self.sc.vessels):
            try:
                d = self.describe(v)
            except Exception:                      # noqa: BLE001
                continue
            if self._reject(d) is not None:
                continue
            cands.append(d)

        if verbose:
            print('[vessel] 候选（满足 a_net/高度/下降 三条）：')
            for d in cands:
                print(f"    {d['name']!r:26} mass={d['mass']/1000:7.1f} t "
                      f"thrust={d['thrust']/1e6:5.2f} MN a_net={d['a_net']:5.1f} "
                      f"alt={d['alt']:8.0f} vs={d['vspeed']:7.1f}")
        if not cands:
            return None
        # 多个候选时取 a_net 最接近典型一级的（约 50）
        cands.sort(key=lambda d: abs(d['a_net'] - 50.0))
        return cands[0]['v']

    def match_by_handoff(self, hs):
        """按 kOS 交班行里的经纬度/高度，选【离该点最近】的船。

        【这是主路径（不再是"备用"）】它不依赖名字、不依赖 vessel id 继承、
        也不依赖 type。只用两个物理量：
          · 位置：三维距离（经纬度 + 高度）
          · 质量：kOS 广播的 mass 与候选当前质量之差（下落中持续烧油，
                  所以允许候选【比广播值轻】，且随时间单调变小）
        【为什么不用 type（2026-09-26 实飞踩坑）】我上一版把 relay 排除了，
        结果把【真正的一级】也排除了 —— 实飞证据：
            kOS 交班行:  type=Relay name=B1040-9 mass=192.7 t alt=4941.1
            kRPC 候选:   type=relay         mass=185.7 t alt=4256   a_net=37.2
        质量差 7 t、高度差 685 m，都在"下落中烧油"的合理范围内 ⇒ 同一艘船。
        玩家在 VAB 里可以把一级的 vessel type 设成 Relay，所以 type 不可信。
        """
        try:
            tlat = float(hs['lat']); tlon = float(hs['lon'])
            talt = float(hs['alt'])
            tmass = float(hs.get('mass') or 0.0) * 1000.0   # kOS 报的是吨
        except (KeyError, TypeError, ValueError):
            return None
        # ================================================================
        # 【2026-09-28 按用户要求：不再刷屏】
        # ----------------------------------------------------------------
        # 【问题】本函数在主循环里【每次轮询都调用】（gate 循环 0.05 s 一次），
        #   而它每轮都把整张候选表 print 出来。
        #   实测一次等待交班要轮询几十~上百次 ⇒ 终端被同一张表刷满，
        #   真正的告警（如 ROTATION CONTAMINATION）被淹没。
        # 【改法】只保留【状态变化时】的摘要输出：
        #   · 匹配结果（成功/失败/换船）变化时才打
        #   逐候选的明细不再打印（需要时用 --list-vessels 查看）。
        # ================================================================
        best, best_score = None, None
        n_cand = 0
        for v in list(self.sc.vessels):
            try:
                d = self.describe(v)
                if d['mass'] < 1000.0:          # 空壳/碎片
                    continue
                g = v.flight(self.body.reference_frame)
                dist = great_circle_m(g.latitude, g.longitude, tlat, tlon)
                dh = abs(d['alt'] - talt)
                tot = math.hypot(dist, dh)
                # 质量检查：候选应 <= 广播质量（烧油变轻）且不能轻太多
                dm = (tmass - d['mass']) if tmass > 0 else 0.0
                mass_ok = (tmass <= 0) or (-0.05 * tmass <= dm <= 0.35 * tmass)
                n_cand += 1
                if not mass_ok or tot >= 3000.0:
                    continue
                score = tot + abs(dm) * 0.1
                if best_score is None or score < best_score:
                    best_score, best = score, d
            except Exception:                      # noqa: BLE001
                continue
        key = (best['name'] if best is not None else None, n_cand)
        if key != getattr(self, '_match_report_key', None):
            self._match_report_key = key
            if best is not None:
                print(f"[vessel] 交班点匹配: {best['name']!r} "
                      f"mass={best['mass']/1000:.1f} t a_net={best['a_net']:.1f} "
                      f"alt={best['alt']:.0f}  (候选 {n_cand})")
            else:
                print(f'[vessel] 交班点匹配: 暂无合格候选（候选 {n_cand}）')
        return best['v'] if best is not None else None

    def state(self):
        """读当前状态，返回 (alt, dist_hz, vz, vh, vmag, mass, tp, tv)"""
        # 【2026-09-26 pyflakes 清理】原先这里有两行没用的读取
        #   pos = self.v.position(self.ref) / vel = self.v.velocity(self.ref)
        #   它们的结果从未被使用（下面全部改用 target_frame 口径），
        #   每帧白白多两次 RPC 往返 —— 而本项目对帧时间敏感。已删。
        fl = self.v.flight(self.ref)
        # 用 flight 的海拔（与 kOS 的 alt:radar 口径最接近）
        alt = fl.surface_altitude
        # 目标系：用 hybrid 参考系把位置/速度转到"目标点静止、x 朝上"
        tgt_frame = self.target_frame()
        tp = self.v.position(tgt_frame)
        tv = self.v.velocity(tgt_frame)
        # 在 target_frame 里：x = 上，y/z 水平
        dist_hz = math.hypot(tp[1], tp[2])
        vz = tv[0]
        vh = math.hypot(tv[1], tv[2])
        # ---- 自转污染自检（2026-09-26 实飞事故后新增）----
        # 【为什么必须查】Vessel.velocity(frame) 会混入 frame 自身的线速度。
        #   曾把 hybrid 的 velocity 传成 body_frame，导致 vh 恒多出天体自转
        #   线速度（实测 175.0 vs 174.9 m/s），而【位置 dist 却是对的】。
        #   这个错误不会抛异常：求解器只会一路 infeasible，跟踪律只会把
        #   a_cmd 顶到几百 m/s²，看上去"很努力"，实际是把火箭往上推。
        #   判据：本机水平速率若【接近所在纬度的自转线速度】且与实测位移
        #   不符，就是被污染了。这里用"相邻帧位移"做独立交叉验证 ——
        #   位移是位置口径，天然不受速度参考系影响。
        v_rot = self.body_rotation_speed()
        self.vrot_est = v_rot
        # 用与上一帧的位置差反推真实水平速率（位置口径，独立可信）
        now = time.time()
        if self._last_pos is not None:
            dtp = now - self._last_t
            if dtp > 0.05:
                dp = math.hypot(tp[1] - self._last_pos[1],
                                tp[2] - self._last_pos[2])
                vh_meas = dp / dtp
                # 日志 vh 远大于实测位移速率，且差值≈自转速度 ⇒ 污染
                if (vh > 20.0 and vh_meas < 0.5 * vh
                        and abs(vh - vh_meas - v_rot) < 0.25 * max(1.0, v_rot)):
                    if not self._rot_warned:
                        self._rot_warned = True
                        print(f'[WARN] 水平速度疑似被天体自转污染: '
                              f'vh={vh:.1f} 但位移反推仅 {vh_meas:.1f} '
                              f'(自转标尺 {v_rot:.1f} m/s)')
                        print('[WARN] 位置口径正常、速度口径异常 —— '
                              '检查 target_frame 的 velocity 参数')
                        self.log.note(
                            f'ROTATION CONTAMINATION suspected: vh={vh:.1f} '
                            f'vh_meas={vh_meas:.1f} v_rot={v_rot:.1f}')
        self._last_pos = tp
        self._last_t = now
        vmag = math.sqrt(vz * vz + vh * vh)
        return alt, dist_hz, vz, vh, vmag, self.v.mass, tp, tv

    def body_rotation_speed(self):
        """本机所在纬度处、天体自转的线速度 [m/s]（用作污染判定标尺）。

        【API 名称已核实】本 kRPC 构建的 CelestialBody【没有 rotation_period】，
        正确名称是 **rotational_period**（易错，与 KSP 自身的
        `body.rotationPeriod` 拼写不同）。依据：
          · D:/Applications/Python/Lib/site-packages/krpc/services/
            spacecenter.py → CelestialBody 成员表
          · GameData/kRPC/KRPC.SpaceCenter.xml
        曾写成 rotation_period，会在 state() 里抛 AttributeError；
        因为包在 try 里只返回 0.0，会让"自转污染自检"【静默失效】——
        比崩溃更危险，所以这里用 getattr 双名兜底并显式记录。
        """
        try:
            o = 2.0 * math.pi / float(self.body.rotational_period)
            lat = float(self.v.flight(self.ref).latitude)
            return abs(o * float(self.body.equatorial_radius)
                       * math.cos(math.radians(lat)))
        except Exception as e:                  # noqa: BLE001
            # 【不要把失败吞掉】这个值只用于"自转污染自检"。若它静默返回 0，
            #   自检会因为"标尺=0"而永不触发 ⇒ 保护形同虚设。
            #   这里显式告警一次，让"API 拼错"这类问题当场暴露。
            if not getattr(self, '_vrot_warned', False):
                self._vrot_warned = True
                print(f'[WARN] 无法取得天体自转速度（{type(e).__name__}: {e}）')
                print('[WARN] 自转污染自检将不可用 —— 请检查 CelestialBody '
                      '成员名（正确为 rotational_period）')
                try:
                    self.log.note(f'v_rot unavailable: {type(e).__name__}: {e}')
                except Exception:               # noqa: BLE001
                    pass
            return 0.0

    def target_frame(self):
        """目标系：原点 = 着陆点（抬高 TARGET_ALT），x=天顶, y=北, z=东。

        【API 语义，对照 KRPC.SpaceCenter.xml + 官方 python stub 逐条核实（非猜测）】
          · Vessel.SurfaceReferenceFrame：
              "origin at the center of mass of the vessel"，x=zenith(上), y=north, z=east
              ⇒ 旋转正是我们要的（x 朝上），但原点在【飞船质心】。
          · ReferenceFrame.CreateRelative(reference_frame, position, rotation, velocity,...)
              position = 原点偏移【位置矢量】；rotation = 【四元数 (x,y,z,w)】。
          · ReferenceFrame.CreateHybrid(position, rotation, velocity, angular_velocity)
              四个参数都是【参考系】，不是矢量：
              "The reference frame providing the position of the origin."

        ⇒ 两步构造（全部用已核实的 API）：
          ① 用 create_relative 在【天体固连系】里建一个原点=目标点的系（旋转不变）；
          ② 用 create_hybrid 把它与 surface 系（提供 x=天顶 的旋转）混合，
             速度继承天体固连系（目标点随天体自转）→ 目标点在系内静止。
        """
        body_frame = self.body.reference_frame
        # ① 目标点在天体固连系中的位置（surface_position 返回位置矢量，含地形高度）
        tgt = np.array(self.body.surface_position(TARGET_LAT, TARGET_LON, body_frame),
                       dtype=float)
        # 沿径向抬高 TARGET_ALT（着陆腿触点高度）
        n = float(np.linalg.norm(tgt))
        if n > 1e-6:
            tgt = tgt + tgt / n * TARGET_ALT
        tgt_frame = self.sc.ReferenceFrame.create_relative(
            body_frame, position=tuple(float(x) for x in tgt))
        # ② 混合：位置/速度都锚在目标点系，旋转用飞船 surface 系（x=天顶）
        #
        # 【2026-09-26 实飞重大修正 —— 速度参考系传错，vh 被天体自转污染】
        #   原实现此处传 velocity=body_frame，注释里的推理是错的。
        #   按官方文档（KRPC.SpaceCenter.xml:10422），CreateHybrid 的 velocity
        #   含义是 "The reference frame providing the linear velocity of the
        #   frame" —— 即【新系自身在世界中的线速度】由该参考系给出。
        #   传 body_frame 等于声明"本系原点以天体固连系的线速度运动"，
        #   而天体固连系随天体自转、自身就有 ~174.9 m/s 线速度
        #   ⇒ 原点被赋予虚假平移速度 ⇒ v.velocity(hybrid) 恒多出自转分量。
        #
        #   【实飞日志铁证】
        #     log-B1040-9-gfold/gfold_log_20260926_103746.csv
        #     用相邻两点实际位移反推真实水平速率，与日志 vh 对照：
        #         真实 vh 中位 = 1.5 m/s    日志 vh 中位 = 176.4 m/s
        #         差值   中位 = 175.0 m/s  ←→ 该纬度自转线速度 174.9 m/s（差 0.06%）
        #     竖直则完全吻合（真实 vz −0.5 / 日志 −0.4）
        #     ⇒ 位置口径正确，只有速度多了自转分量。
        #     另：vh 从 243 单调收敛到 168 —— 真实横速已→0，vh 却收敛到自转速度。
        #
        #   【后果（那次实飞失败的真正原因）】
        #     火箭其实已悬停在目标点上方（真实 dist≈0、vh≈0），程序却以为
        #     "dist≈0 且 vh=176"：
        #       · G-FOLD 被要求"在 0 米内消掉 176 m/s" ⇒ 全程 infeasible
        #       · 跟踪律 vel_err 带入 176 m/s 假速度 ⇒ a_cmd 冲到 817 m/s²
        #         （可用仅 47.6）⇒ throttle 饱和 1.0
        #       · 结果：满油门把火箭【推着往上爬】（日志末尾 alt=3006、vz=+5.3）
        #
        #   【修正】velocity 改传 tgt_frame —— 新系原点与目标点同速，
        #   目标点在系内速度为 0，不再混入天体自转线速度。
        #   旋转仍由 surface 系提供（保证 x=天顶）。
        return self.sc.ReferenceFrame.create_hybrid(
            position=tgt_frame,
            rotation=self.v.surface_reference_frame,
            velocity=tgt_frame)

    def vessel_profile1(self, error, vel, acc, mass):
        """二阶外推出求解器要用的初状态 x0（6 维：位置+速度）。

        【为什么必须做（2026-09-26 补）】
          规划不是瞬间完成的：本项目的 P3/P4 要跑 ~3.2 s。若把"此刻"的状态
          直接喂给求解器，等解出来时火箭已经飞出 3 s 了 —— 参考轨迹的起点
          与实际执行时刻的状态【系统性错位】，表现为跟踪一开始就欠账。
          参考仓库用固定 est_time=0.5 s 做二阶外推来补偿这段延迟。

        【2026-09-26 重要修正：外推必须用【预测的加速度】，不能用 0】
          旧实现在【首次解算】时传 `acc = np.zeros(3)`，于是：
              pos_est = error + vel*est_time          （只有一阶项）
              vel_est = vel                            （速度完全不外推）
          而本项目是【带 300 m/s 垂速 + 持续减速】入场：
            · 一阶外推把位置沿直线推出去，而实际是减速曲线
            · 速度不外推 ⇒ 给求解器一个"偏快"的初速度
          实测偏差：解算 3.6 s 内火箭下降约 970 m，
            而 est_time=0.5 s 的一阶外推只推了约 150 m
          ⇒ 轨迹起点比实际位置【高出约 800 m】
          ⇒ 跟踪器一上来就有巨大位置误差（P 项 K_POS·800 ≈ 400 m/s²，
             远超可用 49 m/s²）⇒ 指令饱和、姿态乱摆。

        【本修正】外推量由调用方给出【真实的预测加速度】acc：
            · 首次解算：用 hold 段的预测加速度（沿速度反向减速的净加速度）
            · 周期性重解：用实测差分加速度（主循环里已经在算）
          这样 pos_est / vel_est 都沿真实的减速曲线外推。

        返回 6 维 ndarray：[px, py, pz, vx, vy, vz]（目标系）。
        """
        est_time = float(getattr(self, 'est_time', 0.5))
        vel_est = np.asarray(vel, float) + np.asarray(acc, float) * est_time
        pos_est = (np.asarray(error, float)
                   + np.asarray(vel, float) * est_time
                   + 0.5 * np.asarray(acc, float) * est_time * est_time)
        return np.array([pos_est[0], pos_est[1], pos_est[2],
                         vel_est[0], vel_est[1], vel_est[2]])

    def _predict_est_time(self):
        """预测本次解算要多久，作为外推时长。

        【为什么要预测而不是用固定 0.5 s】
          参考仓库的 est_time=0.5 s 是"执行延迟"，不是"求解耗时"
          （它求解跑在后台线程，主循环继续飞，所以耗时不入外推）。
          但本项目的跟踪器是【用解出来的轨迹去追当前位置】：
          轨迹起点必须尽量对准"解出来的那一刻"的火箭位置，
          否则一进入 gfold 就有巨大位置误差。

        【取值】上次实测耗时 ×1.2 裕度。
        【2026-09-27 修正：下限由 0.5 s 降到 0.05 s】
          旧版写成 max(0.5, last*1.2)，本意是"至少外推 0.5 s"。
          但 0.5 s 是【参考仓库为"执行延迟"设的常数】（它求解在后台跑，
          耗时不计入外推），而本项目用它代表"求解耗时"——
          两处语义不同，直接套用就成了误差源：
            codegen 后实测求解只要 149 ms
            est_time = max(0.5, 0.149*1.2) = 0.5 s   ← 被下限硬拉到 0.5
            ⇒ 外推【超前 0.35 s】
            ⇒ 轨迹起点比本机靠前 0.35×360 ≈ 126 m
            ⇒ find_nearest_index 给出 fr=-0.5 ⇒ n_i 变负
            ⇒ 触发 track() 的 n_i<0 覆盖分支 ⇒ tilt_cmd 跳变
          ⇒ 下限改为 0.05 s（只防 0 除/零值），让外推时长真实跟随求解耗时。
        """
        last = getattr(self, 'solve_ms', 0.0) / 1000.0
        if last <= 0.05:
            return float(self.args.est_time)
        return float(min(6.0, max(0.05, last * 1.2)))

    def hold_accel(self, vel, mass):
        """预测 hold 段的【净加速度】（供首次解算的外推使用）。

        与主循环里 hold 段实际下发的指令保持同一套公式，
        这样"外推出的状态"与"火箭真实会走的状态"一致。

        返回净加速度 [ax, ay, az]（含重力）。
        """
        a_cap = self.v.max_thrust / max(1.0, mass)
        vmag = float(np.linalg.norm(vel))
        if vmag <= 1e-06:
            return np.array([0.0, 0.0, 0.0])
        rev = np.asarray(vel, float) * (-1.0) / vmag
        up_cap = G0 + HOLD_UP_FRAC * max(0.0, a_cap - G0)
        a_mag = min(a_cap, up_cap / max(1e-06, float(rev[0])))
        vz_net_cap = abs(float(vel[0])) / max(0.1, HOLD_TAU)
        a_net_now = a_mag * float(rev[0]) - G0
        if a_net_now > vz_net_cap:
            need_up = vz_net_cap + G0
            a_mag = min(a_mag, need_up / max(1e-06, float(rev[0])))
        a_mag = max(0.0, min(a_mag, a_cap))
        thrust = rev * a_mag
        return thrust + np.array([-G0, 0.0, 0.0])

    def solve(self, x0_state, mass):
        """调用参考仓库结构的求解器（P3/P4）。返回 dict 或 None。

        【接口变更（2026-09-26）】参数由 (alt, dist, vz, vh, tf) 改为
        (x0_state, mass)，其中 x0_state 是【三维带符号】的
        [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z]（目标系：x=上, y/z=水平）。
        这与参考仓库 GFOLD_run.solver 的 self.x0 口径一致，
        也是 PD 能直接做 `x_i - error` 相减的前提（详见 gfold_p3p4.py 说明）。
        """
        a_net = self.v.max_thrust / mass - 9.80665
        alt = float(x0_state[0]) + TARGET_ALT
        dist = math.hypot(float(x0_state[1]), float(x0_state[2]))
        vz = float(x0_state[3])
        vh = math.hypot(float(x0_state[4]), float(x0_state[5]))
        if a_net < 1.0:
            print(f'[G-FOLD] 预检失败：a_net = {a_net:.2f} m/s²（推重比过低）')
            print(f'          mass={mass/1000:.1f} t  max_thrust='
                  f'{self.v.max_thrust/1e6:.2f} MN')
            print('          这艘船物理上无法着陆 —— 多半是【选错了 vessel】')
            print(f'          当前 vessel={self.v.name!r}；'
                  '用 --vessel-name 指定回收一级')
            self.gfold_status = 'precheck: a_net too low'
            return None
        t0 = time.time()
        # 【求解参数全部来自文件顶部常量区】逐项对应参考仓库 params.txt。
        #   throttle=(0.1, 0.8) 是【求解器】的节流界（参考同值）；
        #   控制器侧用的是 [0.05, 1.0]，两套并存是原仓库设计。
        r = solve_p3p4_state(
            x0_state, mass,
            isp=ISP_DEFAULT,
            t_max=self.v.max_thrust,
            throttle=(0.1, 0.8),
            tf_guess=TF_GUESS,
            straight_fac=STRAIGHT_FAC,
            gs_deg=GS_DEG,
            pcs_deg=PCS_END_DEG,
            pcs_start_deg=PCS_START_DEG,
            v_max=V_MAX_SOLVER,
            N3=N3, N4=N4, target_alt=TARGET_ALT)
        self.solve_ms = (time.time() - t0) * 1000.0
        self.gfold_status = r.get('status', 'err')
        if r.get('status') != 'optimal':
            # 失败时打印"分轴能量需求"，供判断是能量不够还是约束太紧
            h_gnd = max(1.0, alt - TARGET_ALT)
            a_v = vz * vz / (2 * h_gnd)
            a_h = vh * vh / (2 * max(1.0, dist))
            a_need = math.hypot(a_v, a_h)
            print(f'[G-FOLD] solver={self.gfold_status}  a_net={a_net:.1f}')
            print(f'[G-FOLD] 分轴需求: 垂直{vz:+.0f}需{a_v:5.1f} | '
                  f'水平{vh:.0f}需{a_h:5.1f} => 矢量合成 {a_need:.1f} m/s²')
            print(f'[G-FOLD] 可用推力加速度 {a_net:.1f} m/s²  '
                  f'(比值 {a_need/max(1e-6, a_net):.2f})')
            if a_need > a_net * 0.95:
                print('[G-FOLD] >> 交班点能量偏高：所需 > 可用。'
                      '应在更早/更高处交班，或由 kOS 侧多消横向速度。')
            else:
                print('[G-FOLD] >> 能量本身够，是求解器约束或轨迹形状问题'
                      '（看 v_descent_max 包络）。')
            return None
        return r

    def find_nearest_index(self, x, r, v, tf, N):
        """参考仓库 demo3_gfold.py 的 find_nearest_index（逐行移植）。

        原版：
            nearest_mag = npl.norm(x[0:3, 0] - r)
            nearest_i = 0
            for i in range(x.shape[1]):
                mag = npl.norm(x[0:3, i] - r)
                if mag < nearest_mag:
                    nearest_mag = mag
                    nearest_i = i
            v = x[3:6, nearest_i]
            v_norm = npl.norm(v)
            v_dir = v / v_norm
            frac = clamp(np.dot(r - x[0:3, nearest_i], v_dir)
                         / (tf / N * v_norm), 0.5, -0.5)
            return nearest_i + frac

        【本项目补充的除零保护】原版 v_norm 为 0（轨迹末点速度=0）时会
        除零产生 inf/nan。这里在 v_norm 过小时直接返回 nearest_i。
        """
        nearest_mag = float(np.linalg.norm(x[0:3, 0] - r))
        nearest_i = 0
        for i in range(x.shape[1]):
            mag = float(np.linalg.norm(x[0:3, i] - r))
            if mag < nearest_mag:
                nearest_mag = mag
                nearest_i = i
        vv = x[3:6, nearest_i]
        v_norm = float(np.linalg.norm(vv))
        if v_norm < 1e-06:
            return float(nearest_i)
        v_dir = vv / v_norm
        denom = (tf / N) * v_norm
        if abs(denom) < 1e-12:
            return float(nearest_i)
        frac = _clamp(float(np.dot(r - x[0:3, nearest_i], v_dir)) / denom,
                      0.5, -0.5)
        _ = v
        return nearest_i + frac

    def sample_index(self, x, u, index, tf, N):
        """参考仓库 demo3_gfold.py 的 sample_index（逐行移植）。

        原版：
            if index >= N-1:
                return (v3(0,0,0), v3(0,0,0), v3(9.807,0,0))
            elif index <= 0:
                i = 0; frac = index
            else:
                i = math.floor(index); frac = index - i
            x_i_s = lerp(x[:, i], x[:, i+1], frac)
            u_i_s = lerp(u[:, i], u[:, i+1], frac)
            if index < 0:
                u_i_s = u[:, 1].copy()
            return (x_i_s[0:3].copy(), x_i_s[3:6].copy(), u_i_s.copy())

        【逐字保留的两个细节】
          · index >= N-1 时返回 (0,0,0)/(0,0,0)/(g0,0,0) —— 即"已到目标，
            只留重力补偿"。这是原版的到达处理。
          · index < 0 时 u 用 u[:,1] —— 原版的开头处理。
        """
        _ = tf, N
        if index >= N - 1:
            return (np.zeros(3), np.zeros(3), np.array([9.807, 0.0, 0.0]))
        if index <= 0:
            i = 0
            frac = index
        else:
            i = int(math.floor(index))
            frac = index - i
        x_i_s = _lerp(x[:, i], x[:, i + 1], frac)
        u_i_s = _lerp(u[:, i], u[:, i + 1], frac)
        if index < 0:
            u_i_s = u[:, 1].copy()
        return (x_i_s[0:3].copy(), x_i_s[3:6].copy(), u_i_s.copy())

    def track(self, r, tp, tv, n_i):
        """参考仓库 demo3_gfold.py 的 G-FOLD 跟随（逐行移植）。

        原版（line 360-405）：
            (tf,x,u,m,s,z) = gfold_path
            n_i = max(n_i - game_delta_time * 0.2 * N/tf,
                      find_nearest_index(x, error, vel))
            (x_i,  v_i,  u_i)  = sample_index(n_i)
            (x_i_, v_i_, u_i_) = sample_index(n_i + min(1.5*N/tf,
                                                        norm(vel)/50*N/tf))
            target_a  = u_i  + (v_i  - vel)*k_v + (x_i  - error)*k_x
            target_a_ = u_i_ + (v_i_ - vel)*k_v + (x_i_ - error)*k_x
            max_throttle_ctrl = throttle_limit_ctrl[1] * (max_thrust/mass)
            min_throttle_ctrl = throttle_limit_ctrl[0] * (max_thrust/mass)
            target_a  = conic_clamp(target_a,  min, max, max_tilt)
            target_a_ = conic_clamp(target_a_, min, max, max_tilt)
            if n_i < 0:
                target_a = [g0,0,0] + u_i
            target_direction = target_a_ / norm(target_a_)   # 【姿态用 target_a_】
            target_throttle  = norm(target_a) / (max_thrust/mass)  # 【节流用 target_a】
            vessel.control.throttle = target_throttle

        【关键差异（我此前全部搞错的地方）】
          · 位置/速度【直接用 error/vel 相减】，不做任何投影 —— 因为
            ref_target 的坐标轴就是 (上, 北, 东)，而求解器的 x 也是
            (上, 水平, 水平)，两者【本来就同构】。（原版 line 344/346）
          · 节流用 |target_a|，【姿态用 target_a_**（前瞻点）—— 两者不同！
          · 索引由 find_nearest_index 给出，不是时间索引。
        """
        x = r['x']
        u = r['u']
        N = x.shape[1]
        tf = r['tf']
        error = np.array(tp)
        vel = np.array(tv)

        # 索引推进（原版 line 363）
        dt_game = getattr(self, 'last_pkt', 0.02)
        # 【n_i 必须夹到非负】详见下方长注释。
        _raw_near = self.find_nearest_index(x, error, vel, tf, N)
        self.gfold_traj_ready = _raw_near > -0.05
        self.gfold_n_i = max(
            n_i - dt_game * 0.2 * N / tf,
            _raw_near)
        self.gfold_n_i = max(0.0, self.gfold_n_i)

        x_i, v_i, u_i = self.sample_index(x, u, self.gfold_n_i, tf, N)
        step = min(1.5 * N / tf, float(np.linalg.norm(vel)) / 50.0 * N / tf)
        # 【x_i_ 采样但不使用 —— 与参考仓库一致，不是笔误】
        #   参考 demo3_gfold.py：
        #       line 369  (x_i_, v_i_, u_i_) = sample_index(n_i + step)   # 采样了
        #       line 378  target_a  = u_i  + ... + (x_i - error)*k_x
        #       line 379  target_a_ = u_i_ + ... + (x_i - error)*k_x      # 用 x_i!
        #   即【前瞻点只用于 u_、v_，位置误差一律用当前点 x_i】。
        _x_i_, v_i_, u_i_ = self.sample_index(x, u, self.gfold_n_i + step, tf, N)

        # 参考仓库 demo3_gfold.py line 378：
        #     target_a = u_i + (v_i - vel) * k_v + (x_i - error) * k_x
        # 即【速度项用 K_VEL=0.8、位置项用 K_POS=0.5】。
        target_a = u_i + (v_i - vel) * K_VEL + (x_i - error) * K_POS
        target_a_ = u_i_ + (v_i_ - vel) * K_VEL + (x_i - error) * K_POS

        a_cap = self.v.max_thrust / max(1.0, self.v.mass)
        min_mag = 0.05 * a_cap
        max_mag = 1.00 * a_cap
        # ---- 锥角：跟随【参考轨迹自己的倾角】----
        #   参考仓库用固定 25° 锥，因为它的轨迹由【同一个 25° 锥】解出来，
        #   轨迹倾角天然 <= 25°，固定锥与轨迹自洽。本项目为了让大横速入场
        #   可行，求解器用了 pcs_start=85°→pcs_end=30° 的【时变】锥，
        #   于是轨迹倾角也时变 ⇒ 限幅必须用【与求解器同一条锥曲线】，
        #   否则就是自相矛盾。这里直接取参考轨迹在当前节点 u_i 的倾角。
        tilt_traj = math.degrees(math.atan2(
            float(np.linalg.norm(u_i[1:3])), max(1e-09, float(u_i[0]))))
        tilt_now = max(tilt_traj + 8.0, CONIC_TILT_DEG)
        tilt_now = min(tilt_now, 89.0)
        target_a = self.conic_clamp(target_a, min_mag, max_mag, tilt_now)
        target_a_ = self.conic_clamp(target_a_, min_mag, max_mag, tilt_now)

        # 【参考仓库 line 391-392 的降级分支】
        #   原版判据是 `if n_i < 0`（n_i 允许为负）。本实现把 n_i 夹到了
        #   >=0，所以改用等价的独立标志 gfold_traj_ready：
        #   当 find_nearest_index 的原始投影 < -0.05 时，说明本机落在轨迹
        #   起点之外、PD 的位置/速度项不可信 ⇒ 只跟随 u_i，不做修正。
        #   降级分支也要过锥限幅（本项目 u_i 是求解器输出、可能超出当前锥角）。
        if not self.gfold_traj_ready:
            _deg = np.array([G0, 0.0, 0.0]) + u_i
            target_a = self.conic_clamp(_deg, min_mag, max_mag, tilt_now)
            target_a_ = target_a

        self.target_direction = target_a_ / max(1e-09, float(np.linalg.norm(target_a_)))
        self.a_cmd_vec = target_a
        self.tilt_cmd = math.degrees(math.atan2(
            float(np.linalg.norm(target_a[1:3])), max(1e-09, float(target_a[0]))))
        # 【记录【真正的姿态目标】倾角】日志里的 tilt_cmd 是【节流向量
        #   target_a】的倾角，但【姿态】追的是【前瞻向量 target_a_】——
        #   两者是【不同的向量】，倾角也不同！必须分开记。
        #   （tilt_dir_cmd 的赋值已挪到 apply()，见那里的说明）
        self.tilt_cap = tilt_now

        # 【参考轨迹采样点 + 真实跟踪误差】
        _dpos = x_i - error
        self.dbg_n_i = float(self.gfold_n_i)
        self.dbg_plan_alt = float(x_i[0]) + TARGET_ALT
        self.dbg_plan_vz = float(v_i[0])
        self.dbg_plan_vh = float(np.linalg.norm(v_i[1:3]))
        self.dbg_trk_pos = float(np.linalg.norm(_dpos))
        self.dbg_trk_pos_up = float(_dpos[0])
        self.dbg_trk_pos_h = float(np.linalg.norm(_dpos[1:3]))
        self.dbg_trk_vel = float(np.linalg.norm(v_i - vel))
        return target_a, self.gfold_n_i

    def conic_clamp(self, target_a, min_mag, max_mag, tilt_deg=None):
        """把推力加速度指令限幅到推力锥内。

        【逐行照抄参考仓库】dev/gfold/demo3_gfold.py 的 conic_clamp()
        （原仓库 xiaodes/GFOLD_KSP，本项目唯一验证过的 G-FOLD 实飞实现）。
        原文：

            def conic_clamp(target_a, min_mag, max_mag, max_tilt):
                a_mag = npl.norm(target_a)
                hor_dir = v3(0, target_a[1], target_a[2])
                hor_dir /= npl.norm(hor_dir)
                a_hor = npl.norm(target_a[1:3])
                a_ver = target_a[0]
                if (a_hor < min_mag * math.sin(max_tilt)):
                    a_ver_min = math.sqrt(min_mag**2 - a_hor**2)
                else:
                    a_ver_min = math.cos(max_tilt) * min_mag
                if (a_hor < max_mag * math.sin(max_tilt)):
                    a_ver_max = math.sqrt(max_mag**2 - a_hor**2)
                else:
                    a_ver_max = math.cos(max_tilt) * max_mag
                a_ver = clamp(a_ver, a_ver_max, a_ver_min)
                a_hor = min(a_hor, a_ver * math.tan(max_tilt))
                return hor_dir * a_hor + v3(a_ver, 0, 0)

        【本项目的参数对应】（参考仓库用 params.txt，语义一一对应）
            min_mag = throttle_limit_ctrl[0] * (max_thrust/mass)
            max_mag = throttle_limit_ctrl[1] * (max_thrust/mass)
            max_tilt = max_tilt（参考 txt 默认 25°，本项目用 CONIC_TILT_DEG）
        唯一改动：参考里 `hor_dir /= npl.norm(hor_dir)` 在水平分量为 0 时
        会除零（numpy 给 nan）。这里加了零向量保护，其余逐行一致。
        """
        a_cap = self.v.max_thrust / max(1.0, self.v.mass)
        min_mag = 0.05 * a_cap
        max_mag = 1.0 * a_cap
        max_tilt = math.radians(CONIC_TILT_DEG if tilt_deg is None else tilt_deg)
        a_hor = float(np.linalg.norm(target_a[1:3]))
        a_ver = float(target_a[0])
        if a_hor < 1e-09:
            # 纯竖直指令：直接把竖直分量夹到 [min_mag, max_mag]
            return np.array([max(min(a_ver, max_mag), min_mag), 0.0, 0.0])
        hor_dir = np.array([0.0, float(target_a[1]), float(target_a[2])]) / a_hor
        if a_hor < min_mag * math.sin(max_tilt):
            a_ver_min = math.sqrt(max(0.0, min_mag ** 2 - a_hor ** 2))
        else:
            a_ver_min = math.cos(max_tilt) * min_mag
        if a_hor < max_mag * math.sin(max_tilt):
            a_ver_max = math.sqrt(max(0.0, max_mag ** 2 - a_hor ** 2))
        else:
            a_ver_max = math.cos(max_tilt) * max_mag
        # 注意 clamp 的参数顺序：这里是 max(min(...)) 形式，等价于 _clamp
        a_ver = max(a_ver_min, min(a_ver_max, a_ver))
        a_hor = min(a_hor, a_ver * math.tan(max_tilt))
        return hor_dir * a_hor + np.array([a_ver, 0.0, 0.0])

    def apply(self, target_a, target_direction=None, game_dt=0.02):
        """逐行照抄参考仓库 demo3_gfold.py 的执行部分（line 397-405, 460-477）。

        原版：
            target_direction = target_a_ / npl.norm(target_a_)
            target_throttle  = npl.norm(target_a) / (max_thrust / mass)
            vessel.control.throttle = target_throttle

            # 变换到机体坐标系计算姿态控制
            target_direction_local = transform(target_direction, rotation_srf2local)
            avel_local = transform(avel, rotation_srf2local)
            control_pitch = -clamp(ctrl_x_rot.update(angle_around_axis(
                target_direction_local, v3(0,1,0), v3(1,0,0)), dt), 1, -1)
            control_yaw   = -clamp(ctrl_z_rot.update(angle_around_axis(
                target_direction_local, v3(0,1,0), v3(0,0,1)), dt), 1, -1)
            control_roll  = clamp(avel_local[1] * ctrl_y_avel_kp, 1, -1)
            vessel.control.pitch = control_pitch
            vessel.control.yaw   = control_yaw
            vessel.control.roll  = control_roll

        【为什么不用 auto_pilot（坑㉒/㉓ 的最终结论）】
          参考仓库【完全不用 auto_pilot】，而是自己写 PID 直接写
          control.pitch/yaw/roll。我此前用 auto_pilot 一路踩了三个坑：
          never-engaged、错误的 engage() 名称、以及与 kOS 抢 control 通道。
          照抄参考仓库即可全部避开。

        【节流与姿态用不同的向量】
          节流用 target_a（当前点），姿态用 target_a_（前瞻点）——
          这是原版的设计（line 397-398），移植时保留。
        """
        # ---- 节流（原版 line 398, 405）----
        mag = float(np.linalg.norm(target_a))
        a_cap = self.v.max_thrust / max(1.0, self.v.mass)
        throttle = mag / a_cap if a_cap > 1e-9 else 0.0
        throttle = _clamp(throttle, 1.0, 0.0)
        self.v.control.throttle = throttle
        self.thr_cmd = throttle

        # ---- 姿态（原版 line 461-477）----
        if target_direction is None:
            target_direction = target_a / max(1e-9, mag)
        try:
            # 机体系 <- 地面系 的旋转（原版 line 348-349）
            q = self.v.rotation(self.v.surface_reference_frame)
            rot_l2s = _rotation_mat(tuple(q))
            rot_s2l = np.linalg.inv(rot_l2s)
            avel = np.array(self.v.angular_velocity(self.v.surface_reference_frame))
            # ============================================================
            # 【2026-09-28 姿态限速器已【整体移除】—— 它不起作用】
            # ------------------------------------------------------------
            # 曾在此处插入 _slew_limit()：把【指令方向的变化速率】限到
            #   w_max = sqrt(2*alpha*headroom)，意图是"不要求飞不出来的姿态"。
            # 【实飞证明无效】log ..._001430：69 帧里只有 1 帧真正被限速，
            #   slew_err 全程为 0。原因是限速器限制的是【设定值的速率】，
            #   而失败时设定值是【静止的】（25 deg 挂了 6 秒，没有速率可限）；
            #   真正失控的是【载具角速度】冲到 50~73 deg/s（限速上限的 4~8 倍）。
            #   ⇒ 只有约束【机身角速度本身】才有效，而那是 PID 的职责。
            # 故整段删除；姿态回路回到与参考仓库一致的纯角度 PID 形式。
            # ============================================================
            tgt_local = _transform(target_direction, rot_s2l)
            avel_local = _transform(avel, rot_s2l)
            # ============================================================
            # 【2026-09-28 新增：把 PID 的【输入/三项/输出】全部落日志】
            # ------------------------------------------------------------
            # 【为什么必须记】用户要调参，但此前日志【看不到 PID 内部】：
            #   · 只记了 att_err（机体系与 +y 的夹角），那不是 PID 的输入
            #   · PID 真正的输入是 angle_around_axis(...)，单位【弧度】，
            #     且【带符号】（正=要往正方向转）
            #   · 输出 control.pitch/yaw 被 clamp 到 ±1。
            #     「是否饱和」是本次过冲的核心问题，此前只能【推算】
            #     （kp*e>1），没有实测值 —— 这里补上。
            #   · 还要分开记 p/i/d 三项，才知道该调 kp 还是 kd。
            # 【口径】全部在【机体系】下，与 PID 内部一致。
            # ============================================================
            err_x = _angle_around_axis(
                tgt_local, np.array([0.0, 1.0, 0.0]), np.array([1.0, 0.0, 0.0]))
            err_z = _angle_around_axis(
                tgt_local, np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0]))
            out_x = self.ctrl_x_rot.update(err_x, game_dt)
            # 记录 p/i/d 分解（PID.update 里 result = p+i+d，已带符号）
            self.dbg_pid_px = float(-err_x * CTRL_X_ROT_KP)
            self.dbg_pid_dx = float(-self.ctrl_x_rot.diff * CTRL_X_ROT_KD)
            self.dbg_pid_ix = float(-self.ctrl_x_rot.integral)
            cp_ = -_clamp(out_x, 1, -1)
            out_z = self.ctrl_z_rot.update(err_z, game_dt)
            cy_ = -_clamp(out_z, 1, -1)
            cr_ = _clamp(float(avel_local[1]) * CTRL_Y_AVEL_KP, 1, -1)
            # 落日志：PID 输入(rad/deg) + 三项 + 最终控制量
            self.dbg_pid_err_x = math.degrees(err_x)
            self.dbg_pid_err_z = math.degrees(err_z)
            self.dbg_pid_out_x = float(cp_)
            self.dbg_pid_out_z = float(cy_)
            self.dbg_pid_out_r = float(cr_)
            # 【类型标注】avel_local 来自 _transform()，其返回被标注为
            #   Tuple[float,float,float]；逐个 float(...) 会被 Pylance 判为
            #   「Tuple 传给 float」（见本文件其它同类处说明）。
            #   先整体转 list[float] 让类型明确。
            self.dbg_avel_local = [float(x) for x in avel_local]
            self.v.control.pitch = float(cp_)
            self.v.control.yaw = float(cy_)
            self.v.control.roll = float(cr_)
            self.ap_engaged = True
            self.att_err = math.degrees(math.acos(max(-1.0, min(1.0, float(
                np.dot(_normalize(tgt_local), np.array([0.0, 1.0, 0.0])))))))
            # ---- 记录【本机机头相对天顶的真实倾角】----
            # 【2026-09-26 修正】旧版把日志的 tilt_act 直接写成 self.tilt_cmd
            #   （`tilt_act = self.tilt_cmd`），于是 tilt_act 列【只是命令的回声】，
            #   完全看不出姿态跟没跟上 —— 排查时被这一列误导过。
            #   这里改成从【机体轴在世界系的方向】算真实倾角：
            #   机头 = 机体 +y（与 target_direction 的约定一致），
            #   转到地面系后与天顶求夹角。
            try:
                # flight().direction 是机头在地面系中的单位向量；
                # surface 系 x 轴 = 天顶。两者夹角即真实倾角。
                nose_srf = np.array(self.v.flight(
                    self.v.surface_reference_frame).direction)
                zen = np.array([1.0, 0.0, 0.0])
                c = float(np.dot(_normalize(nose_srf), zen))
                self.tilt_act = math.degrees(math.acos(max(-1.0, min(1.0, c))))
                # 【2026-09-28 新增】实测倾角变化率 [deg/s]。
                #   用于判断"指令要变多快 vs 载具实际能变多快"。
                #   与 avel_mag 互补：avel_mag 是绕轴角速度模，
                #   tilt_rate 只反映"倾角"这一维的变化速度。
                if (self._prev_tilt_act == self._prev_tilt_act
                        and game_dt > 1e-6):
                    self.dbg_tilt_rate = (
                        (self.tilt_act - self._prev_tilt_act) / game_dt)
                self._prev_tilt_act = self.tilt_act
                # ============================================================
                # 【2026-09-27 新增：记录真实姿态【向量】与姿态目标【向量】】
                # ============================================================
                # 【为什么必须记向量而不是只记角度】此前只有 tilt_act(标量)
                #   和 tilt_dir_cmd(标量)，**无法判断方向对不对**：
                #   例如"倾角 24°"既可能是朝东 24°、也可能是朝西 24°，
                #   而本载体要给横向制动，方向反了就是加速而不是减速。
                #   实飞 log ..._164450 帧 0：att_err=47.95°，
                #   但完全看不出载具是"没转到位"还是"转反了"。
                #   ⇒ 记两组三分量向量（都在 surface 系下）：
                #       dir_v*  = 载具机头真实方向
                #       tgt_v*  = 我们要求它指向的方向（target_direction）
                #     两者点积 < 0 = 方向反了；夹角大 = 没转到位。
                #   同时记 aim_err_deg = 真实夹角（这才是"姿态误差"的定义，
                #   与 att_err 不同：att_err 是机体系下与 +y 的夹角，
                #   而控制目标 angle_around_axis 是绕轴旋转量，三者不可混用）。
                tgt_srf = np.array(target_direction, dtype=float)
                tgt_srf = tgt_srf / max(1e-9, float(np.linalg.norm(tgt_srf)))
                # ============================================================
                # 【2026-09-28 修复：tilt_dir_cmd 在 final 段是【陈旧值】】
                # ============================================================
                # 【故障现象】log ..._193223 帧 45..53（整个 final 段）
                #   tilt_dir_cmd 恒为 7.70，而同期 a_cmd_h 在 0.55~24 之间
                #   大幅变化 —— 一个"姿态目标"不可能同时给出恒定倾角和
                #   变化的水平指令。
                # 【根因】tilt_dir_cmd 此前只在 track()（gfold 分支）里赋值
                #   （原 1392-1396 行）。final 分支【从不更新它】，
                #   于是日志把进入 final 前最后一帧的旧值一路打印下去。
                #   ⇒ 这不是控制故障，而是【日志列的陈旧值】，
                #     但它会严重误导排障（本次就差点被误导）。
                # 【修法】把它挪到 apply() —— 这里是【两个分支共同的出口】，
                #   target_direction 在此已定稿，因此该列必定反映
                #   "本帧真正命令的姿态方向"，与走哪个分支无关。
                #   倾角定义与原来一致：acos(dir[0])，dir 在目标系 x=天顶。
                try:
                    self.tilt_dir_cmd = math.degrees(math.acos(
                        max(-1.0, min(1.0, float(tgt_srf[0])))))
                except Exception:                    # noqa: BLE001
                    self.tilt_dir_cmd = float('nan')
                self.dbg_dir = _normalize(nose_srf)
                self.dbg_tgt = tgt_srf
                self.dbg_aim_err = math.degrees(math.acos(max(-1.0, min(1.0,
                    float(np.dot(self.dbg_dir, self.dbg_tgt))))))
            except Exception:                       # noqa: BLE001
                self.tilt_act = float('nan')
        except Exception as e:                      # noqa: BLE001
            if not getattr(self, '_att_warned', False):
                self._att_warned = True
                print(f'[WARN] 姿态控制异常: {type(e).__name__}: {e}')
        return throttle

    def _dbg_cols(self):
        """【2026-09-27 新增】产出"真实姿态 + 参考轨迹点 + 真实跟踪误差"各列。

        【为什么单独抽出来】两个日志点（hold 段 / gfold 段）都要写这些列，
        抽成一个方法避免两处漂移（列名与顺序由 Logger.COLS 强制检查）。

        返回值：dict，键名与 Logger.COLS 中的新增列一一对应。
        未就绪/无数据的量写 ''（空），便于离线脚本用 float() 失败即跳过。
        """
        def _f(v, nd=3):
            """float 安全格式化：NaN -> ''（CSV 里留空，不写 nan）。"""
            try:
                x = float(v)
            except (TypeError, ValueError):
                return ''
            return round(x, nd) if x == x else ''

        def _v(vec, idx):
            """取向量某分量；向量为 None 时返回 ''。"""
            if vec is None:
                return ''
            try:
                return _f(vec[idx])
            except (TypeError, IndexError):
                return ''

        def _av_c(idx):
            """取【机体系角速度】的第 idx 个分量。

            【为什么单独写】self.dbg_avel_local 初值是 None、跑起来是
            list[float]。直接用 getattr(...)[idx] 会让类型检查器推断成
            Optional 下标而报错；这里显式判空并返回 '' 兜底，语义也更清楚。
            """
            _a = getattr(self, 'dbg_avel_local', None)
            if not _a:
                return ''
            try:
                return _f(_a[idx])
            except (TypeError, IndexError):
                return ''

        return dict(
            dir_vx=_v(self.dbg_dir, 0),
            dir_vy=_v(self.dbg_dir, 1),
            dir_vz=_v(self.dbg_dir, 2),
            tgt_vx=_v(self.dbg_tgt, 0),
            tgt_vy=_v(self.dbg_tgt, 1),
            tgt_vz=_v(self.dbg_tgt, 2),
            n_i=_f(self.dbg_n_i, 2),
            plan_alt=_f(self.dbg_plan_alt, 2),
            plan_vz=_f(self.dbg_plan_vz, 3),
            plan_vh=_f(self.dbg_plan_vh, 3),
            trk_pos=_f(self.dbg_trk_pos, 2),
            trk_pos_up=_f(self.dbg_trk_pos_up, 2),
            trk_pos_h=_f(self.dbg_trk_pos_h, 2),
            trk_vel=_f(self.dbg_trk_vel, 3),
            # ---- ④ 姿态权限与循环时标（2026-09-28 用户要求）----
            loop_dt=_f(getattr(self, 'dbg_loop_dt', None), 4),
            rpc_ms=_f(getattr(self, 'dbg_rpc_ms', None), 1),
            tau_p=_f(getattr(self, 'dbg_tau_p', None), 0),
            tau_r=_f(getattr(self, 'dbg_tau_r', None), 0),
            tau_y=_f(getattr(self, 'dbg_tau_y', None), 0),
            I_p=_f(getattr(self, 'dbg_I_p', None), 0),
            I_r=_f(getattr(self, 'dbg_I_r', None), 0),
            I_y=_f(getattr(self, 'dbg_I_y', None), 0),
            alpha_max=_f(getattr(self, 'dbg_alpha_max', None), 2),
            avel_mag=_f(getattr(self, 'dbg_avel_mag', None), 2),
            tilt_rate=_f(getattr(self, 'dbg_tilt_rate', None), 2),
            # ---- ⑤ PID 内部量 ----
            pid_err_x=_f(getattr(self, 'dbg_pid_err_x', None), 3),
            pid_err_z=_f(getattr(self, 'dbg_pid_err_z', None), 3),
            pid_px=_f(getattr(self, 'dbg_pid_px', None), 4),
            pid_dx=_f(getattr(self, 'dbg_pid_dx', None), 4),
            pid_ix=_f(getattr(self, 'dbg_pid_ix', None), 4),
            pid_out_x=_f(getattr(self, 'dbg_pid_out_x', None), 4),
            pid_out_z=_f(getattr(self, 'dbg_pid_out_z', None), 4),
            pid_out_r=_f(getattr(self, 'dbg_pid_out_r', None), 4),
            # 【写法】先取一次局部量再索引，避免 Pylance 对
            #   "getattr(..., default)[i]" 推断成 Any/Optional 而报错。
            avel_p=_av_c(0), avel_r=_av_c(1), avel_y=_av_c(2),
            # ---- ⑥ 姿态限速器 ----
            slew_alpha=_f(getattr(self, '_slew_alpha', None), 2),
            slew_wmax=_f(getattr(self, '_slew_wmax', None), 2),
            slew_act=(1 if getattr(self, '_slew_active', False) else 0),
            slew_err=_f(getattr(self, '_slew_err', None), 2),
        )

    def verify_takeover_step(self):
        """接管校验（**非阻塞**）：每帧采一个样本，够数后判定。

        【为什么必须验证】2026-09-26 实飞出现过最隐蔽的一种失败：
        kRPC 每帧都成功写入 control.throttle，日志里 thr_cmd 记的也是
        自己写的值（恒 1.0），但实际 thr_act 只有 0~0.5 —— 因为 kOS 侧
        `lock throttle` 仍然存在，kOS 的 lock 每物理帧重算、优先级高于
        kRPC 的写入，所以真正执行的是 kOS 的值。kRPC "以为"自己接管了。
        判据：把写入值与【实际读数】比较，连续若干帧对不上就是没接管。

        【2026-09-26 时序修正】旧版是个阻塞函数（sleep 0.05×10 = 0.5 s），
        且跑在主循环【之前】⇒ 那 0.5 s 火箭无推力自由落体。
        改成逐帧采样：不占额外时间，结论同样可靠（样本仍来自真实执行器）。
        """
        if self.takeover_ok is not None:
            return                      # 已判定过，不再重复
        if self.args.dry_run:
            self.takeover_ok = True     # dry-run 不写执行器，无需校验
            return
        cmd = getattr(self, 'thr_cmd', 0.0)
        try:
            act = float(self.v.control.throttle)
        except Exception:               # noqa: BLE001
            return
        # ================================================================
        # 【2026-09-28 修正：本校验的两个缺陷（实飞 log ..._231236 暴露）】
        # ----------------------------------------------------------------
        # 【缺陷 1：读回的是【我们自己写的值】】
        #   act = self.v.control.throttle 读的就是上一帧 kRPC 写入的同一个
        #   字段。KSP 只是把它回显，因此正常情况下 cmd 与 act 必然相等，
        #   本校验【永远通不出问题】——它只能发现"有别人覆盖了该字段"。
        #   （原注释说的 kOS lock 覆盖确实属于这一类，所以保留该判据。）
        # 【缺陷 2（本次失败的直接原因）：游戏暂停时会【误报】】
        #   实飞中 KSP 被暂停 ⇒ 物理不前进 ⇒ control.throttle 停在旧值，
        #   而我们的 cmd 一直在变 ⇒ 连续 28/40 不符 ⇒ 误判为"未被接管"，
        #   随后【放弃控制】。真因是暂停，与接管无关。
        # 【修法】把"状态是否冻结"作为前置：冻结时不采样、不判负，
        #   由主循环的冻结检测统一处理（那里会提示用户取消暂停）。
        #   同时把阈值说明清楚：这里的比较是"回显一致性"，不是执行器实测。
        # ================================================================
        if getattr(self, '_frozen_n', 0) >= 3:
            # 状态冻结中，样本不可信 —— 不计入、也不判负
            return
        if self.takeover_n < 12:        # 先跳过前几帧（写入还没生效）
            self.takeover_n += 1
            return
        if abs(cmd - act) > 0.15:
            self.takeover_bad += 1
        self.takeover_n += 1
        if self.takeover_n >= TAKEOVER_SAMPLES:
            self.takeover_ok = (self.takeover_bad < TAKEOVER_SAMPLES * 0.6)
            if self.takeover_ok:
                print(f'[kRPC] 接管校验通过：{self.takeover_n} 帧中不符 '
                      f'{self.takeover_bad} 帧')
                self.log.note(f'takeover OK: mismatch '
                              f'{self.takeover_bad}/{self.takeover_n}')
            else:
                print(f'[kRPC] >> 节流未被真正接管！{self.takeover_n} 帧中 '
                      f'{self.takeover_bad} 帧不符')
                print('[kRPC] >> 多半是 kOS 侧 lock throttle 仍存在'
                      '（kOS 的 lock 优先级高于 kRPC 写入）。')
                print('[kRPC] >> 请确认 boot/B1040-9.ks 交班时已 unlock throttle。')
                self.log.note(f'takeover FAILED: '
                              f'{self.takeover_bad}/{self.takeover_n}')

    # ------------------------- 后台重解（参考仓库 line 458）-------------------------
    def _solve_thread_alive(self):
        return self._solve_thread is not None and self._solve_thread.is_alive()

    def _start_resolve(self, x0_state, mass):
        """在【后台线程】里重解（照抄参考仓库的 Thread(target=solve_gfold)）。

        【为什么用线程】P3/P4 单次 ~3.2 s。若同步执行，主循环停摆 3.2 s，
        期间不写执行器 ⇒ 失控。参考仓库的做法是起线程，主循环继续按
        【旧轨迹】飞，解好后原子替换。
        【线程安全】只让线程写 self._pending_plan（单一赋值的原子引用），
        主循环在下一帧检查并接管，不做任何加锁 —— 与参考仓库一致。
        """
        def _work(x0, m):
            try:
                res = self.solve(x0, m)
                if res is not None and res.get('status') == 'optimal':
                    self._pending_plan = res
                    self._pending_msg = f'replan OK tf_m={res.get("tf_m"):.1f}'
                else:
                    self._pending_msg = f'replan fail: {self.gfold_status}'
            except Exception as e:                  # noqa: BLE001
                self._pending_msg = f'replan EXC: {type(e).__name__}: {e}'

        self._solve_thread = threading.Thread(
            target=_work, args=(np.array(x0_state, float), mass), daemon=True)
        self._pending_plan = None
        self._pending_msg = None
        self._solve_thread.start()

    def _collect_resolve(self):
        """主循环每帧调用：若后台解好了就把 self.plan 设为新轨迹。"""
        if self._pending_plan is not None:
            self.plan = self._pending_plan
            self._pending_plan = None
            self.replan_n += 1
            # 参考仓库 solve_gfold 里解好后 n_i = -100（重新定位到新轨迹起点）
            self.gfold_n_i = -100.0
            # 新轨迹尚未定位 ⇒ 先当作"不可信"，由 track() 下一帧重新判定。
            self.gfold_traj_ready = False
            msg = self._pending_msg or ''
            print(f'[replan] #{self.replan_n} 采纳新轨迹  {msg}')
            self.log.note(f'replan #{self.replan_n} {msg}')
            return
        # ---- 首次解算失败的情况 ----
        #   旧版同步解算失败会直接 return 3 退出（那时火箭没人管）。
        #   现在改成非阻塞：若后台解失败，self.plan 仍是 None，
        #   由 nav_mode=='hold' 分支继续稳住，并在下面这里报一次错，
        #   等后续周期性重解成功后再进入 gfold。
        if (self._pending_msg and self.plan is None
                and not getattr(self, '_first_fail_reported', False)):
            self._first_fail_reported = True
            print(f'[G-FOLD] 首次解算失败（{self._pending_msg}），'
                  f'保持 hold 并等待重解')
            self.log.note(f'first solve failed: {self._pending_msg}')

    # ------------------------- 主循环 -------------------------
    def run(self):
        # 【2026-09-28】把 vh 一并记下：kOS 侧交班门已加"横速"判据
        #   （boot/B1040-9.ks 的 vh_gate，2026-09-28 起 = 10），这里记录 kRPC 侧观察到的 vh，
        #   供实飞后判断"段1 是否真的把横速收干净了"。
        self.log.note(f'GATE alt<{GATE_ALT} |v|<{GATE_VMAX} cone<{GATE_CONE_DEG}deg '
                      f'(kOS vh_gate=10 hard gate, AIM_ALT={AIM_ALT})')
        self.log.note(f'dry_run={self.args.dry_run} no_autopilot={self.args.no_autopilot}')

        # 【2026-09-28 按用户要求：不再刷屏】3 行横幅压成 1 行。
        print('[wait] 等待交班门（kOS 段1 正在飞行）…'
              '（按 kOS 交班行匹配，否则按物理特征筛选）')
        t_start = time.time()
        handoff = None
        while True:
            # ---- 每轮重新选 vessel（优先用 kOS 交班点匹配）----
            hs = find_handoff_line(max_age_s=300.0)
            if hs and hs != handoff:
                handoff = hs
                print(f"[handoff] 读到 kOS 交班行: lat={hs['lat']} lon={hs['lon']} "
                      f"alt={hs['alt']} v={hs.get('v')} type={hs.get('type')} "
                      f"name={hs.get('name','')}")
                self.log.note(f"kOS handoff: lat={hs['lat']} lon={hs['lon']} "
                              f"alt={hs['alt']} v={hs.get('v')} "
                              f"mass={hs.get('mass')} name={hs.get('name','')}")
            v = self.pick_vessel(verbose=False, handoff=handoff)
            if v is None:
                # 【2026-09-28 按用户要求：不再刷屏】
                #   原先每 5 s 打一行 + 调一次 log_vessel_table。
                #   log_vessel_table 只写日志文件（不刷屏），保留；
                #   但控制台提示改为【只在第一次】打，避免长等待刷屏。
                # 【2026-09-28 二修：连日志也只写一次】
                #   上一版把 log_vessel_table 改成 echo=False（不刷控制台），
                #   但它仍每 5 s 往日志里追加 3 行 ⇒ 日志前 200 多行全是候选表
                #   （实飞 log ..._231236 就是如此，把真正的故障记录挤到最后）。
                #   候选表用于"事后定位选错船"，等待期间写一次足够；
                #   真正需要反复看时用 --list-vessels。
                if not getattr(self, '_wait_reported', False):
                    self._wait_reported = True
                    print('[wait] 暂无符合条件的回收一级（需要 a_net 15~80、'
                          '正在下降、高度 100~25000 m）；'
                          '后续同类提示不再重复打印')
                    self.log_vessel_table()
                if time.time() - t_start > self.args.gate_timeout:
                    print('[gate] 超时未找到回收一级，退出')
                    self.log.note('gate timeout: no candidate vessel')
                    self.log.flush()
                    return 2
                time.sleep(0.2)
                continue
            if self.v is None or v != self.v:
                self.bind(v)
                d = self.describe(v)
                print(f"[vessel] 锁定 {d['name']!r} mass={d['mass']/1000:.1f} t "
                      f"thrust={d['thrust']/1e6:.2f} MN a_net={d['a_net']:.1f} m/s²")
                self.log.note(f"vessel={d['name']} mass={d['mass']:.0f}kg "
                              f"max_thrust={d['thrust']:.0f}N a_net={d['a_net']:.1f}")
                self.log_vessel_table()

            alt, dist, vz, vh, vmag, mass, tp, tv = self.state()
            cone_ok = dist <= (alt - TARGET_ALT) / math.tan(math.radians(GATE_CONE_DEG))

            # 【2026-09-27 预热已删除】此处原有 _start_prewarm 触发。
            #   删除理由见文件头 "预热机制已整体删除" 注释块：
            #   预热轨迹起点比交接点高 112 m / 快 24 m/s，导致
            #   vterm=-23.2 把 a_ver 压到 0、conic_clamp 返回纯竖直、
            #   111 帧完全不跟踪。
            #   现在改为：门满足后【用门处的真实状态直接解算】。

            if (alt < GATE_ALT and vmag < GATE_VMAX and cone_ok
                    and alt > GATE_ALT_MIN):
                # ---- 能量可行性预检（交班门前置，仅告警不拦截）----
                # 【为什么加】旧版只看"锥内 + 高度 + 速度上限"就放行，但
                #   实飞交班点（alt 5031 / |v| 478.8 / vz -314 / vh 362）
                #   分轴合成需求 51.8 m/s² 略高于可用 49.2 —— 属于【能量偏紧】
                #   的边界工况。这个数放进日志，配合 infeasible 一看就知道
                #   该"等更高处交班"还是"让 kOS 多消横向速度"。
                #
                # 【重要：不拦截】实测分轴比值 1.05 时 G-FOLD 仍能解出
                #   optimal（tf 21.2 s、落地 141.6 t、末倾 0.00°）——因为
                #   垂直与水平需求不会全程同时满打，分轴比值只是【保守上界】。
                #   曾把它做成硬门（>0.95 就等），会把本可解算的状态挡在门外。
                #   所以这里只打印 + 记日志，交给求解器做最终裁决。
                h_gnd = max(1.0, alt - TARGET_ALT)
                a_avail = self.v.max_thrust / max(1.0, mass)
                a_need = math.hypot(vz * vz / (2 * h_gnd),
                                    vh * vh / (2 * max(1.0, dist)))
                ratio = a_need / max(1e-6, a_avail)
                print(f'[gate] 能量预检: 需{a_need:.1f} / 可用{a_avail:.1f} '
                      f'= {ratio:.2f}  (垂直{vz:+.0f} 水平{vh:.0f})'
                      + ('  [偏紧]' if ratio > 0.95 else ''))
                self.log.note(f'gate energy: need={a_need:.1f} '
                              f'avail={a_avail:.1f} ratio={ratio:.2f}')
                print(f'[gate] 交班门满足: alt={alt:.0f} dist={dist:.0f} '
                      f'|v|={vmag:.1f} vh={vh:.1f} vz={vz:.1f}')
                break
            if time.time() - t_start > self.args.gate_timeout:
                print('[gate] 超时未等到交班门，退出')
                self.log.note('gate timeout')
                self.log.flush()
                return 2
            time.sleep(0.05)

        # ---- 首次解算（含状态外推）----
        alt, dist, vz, vh, vmag, mass, tp, tv = self.state()
        # 【2026-09-26 修正】这里过去传 acc=0，导致外推只有一阶项、速度完全
        #   不外推 ⇒ 轨迹起点比实际位置高约 800 m（详见 vessel_profile1 说明）。
        #   现在用 hold_accel() 预测的【真实减速加速度】，并且 est_time 覆盖
        #   【整个求解耗时】（解算期间火箭一直在动，必须一路推过去）。
        game_prev_time = self.sc.ut
        acc_pred = self.hold_accel(np.array(tv), mass)
        self.est_time = self._predict_est_time()
        x0_state = self.vessel_profile1(np.array(tp), np.array(tv),
                                        acc_pred, mass)
        print(f'[G-FOLD] 状态外推 est_time={self.est_time:.1f}s '
              f'(含预测求解耗时)  起点 alt={x0_state[0]+TARGET_ALT:.0f} '
              f'vz={x0_state[3]:.1f} (实测 vz={vz:.1f})')
        # ---- 首次解算：改为【非阻塞】（2026-09-26 关键修正）----
        # 【实飞证据】旧版首次解算是同步的，实测 solve_ms = 3898.8 ms（3.9 s）。
        #   而 kOS 交班后已 unlock 并结束脚本 ⇒ 这 3.9 s 内【完全无人控制】。
        #   日志铁证：kOS 交班 alt=5034.4 / v=363.5
        #             kRPC 首帧 alt=3638.7 / vz=−310.9
        #   ⇒ 无控自由落体 1396 m（≈4.5 s，正好是 3.9 s 解算 + 0.6 s 其他）
        #   ⇒ 等解完时状态已恶化到分轴能量比 1.29（需 63.7 / 可用 49.3），
        #     一开头就【超能量】，后面再怎么控都救不回来。
        #
        # 【参考仓库怎么做的（demo3_gfold.py）】
        #   · line 436-438：规划出来之前 nav_mode='none'，仍然【每帧写执行器】：
        #         target_direction = -vel ; vessel.control.throttle = 0
        #   · line 440-458：满足 start_altitude 才【起线程】求解，主循环不阻塞
        #   · line 391-392：n_i < 0 时 target_a = [g0,0,0] + u_i（保持姿态）
        #   ⇒ 它【从不阻塞】也【从不停止写执行器】。
        #
        # 【本实现的等价做法】先起后台线程解算，主循环立刻开始跑；
        #   解出来之前进入 nav_mode='hold'：姿态保持竖直、节流给到"抵消重力"
        #   的水平（不做机动，只防止自由落体继续加速）。
        #   这样 kRPC 从交接后【第一帧】就握住执行器，不再有真空期。
        self.engaged = True
        self.replan_n = 0
        # ================= 交接时【直接重解】（预热已删除）=================
        # 【用户指定做法】"交接时直接重解"。
        # 【为什么这样可以】codegen 路径实测 0.08~0.12 s（cvxpy 需 2~3.6 s）。
        #   预热机制存在的唯一理由是"cvxpy 要 3.6 s，来不及"，而 codegen
        #   把耗时压到 0.1 s 量级后，这个理由已不成立；
        #   而预热带来的代价（起点错位 112 m / 24 m/s ⇒ 完全不跟踪）是致命的。
        #   ⇒ 用【门处的真实状态】解，起点与载具严格一致，零错位。
        #
        # 【求解期间做什么】仍然【非阻塞】：起后台线程，主循环立刻跑 hold，
        #   从第一帧就握住执行器（不让 kRPC 交接后有真空期）。
        #   解好后下一帧 _collect_resolve() 原子替换、切 nav_mode='gfold'。
        #   这正是参考仓库 demo3_gfold.py line 440-458 的结构。
        self.nav_mode = 'hold'
        print('[G-FOLD] 交接点直接解算（已提交后台线程，非阻塞）；'
              '解算期间按 hold 反向减速')
        self.log.note(f'handoff solve at gate: alt={alt:.1f} vz={vz:.1f} '
                      f'vh={vh:.1f} mass={mass/1000:.1f}t (prewarm removed)')
        self._start_resolve(x0_state, mass)

        # ---- 接管校验：确认真接管（**不再阻塞主循环**）----
        # 【2026-09-26 时序修正】旧版在这里同步调用 verify_takeover()，
        #   它内部 `time.sleep(0.05)` × 10 = **阻塞 0.5 s**。
        #   而这 0.5 s 发生在【主循环启动之前】⇒ hold 一帧都还没写
        #   ⇒ 火箭在 kOS 放权后的 0.5 s 内【完全没有推力】(自由落体)。
        #
        #   本项目的整个设计目标就是"交接后立刻握住执行器"，这 0.5 s 真空
        #   与目标直接矛盾。改为【在主循环里做】：
        #     · 第 1 帧就写执行器（hold）
        #     · 之后每帧采样 (thr_cmd, thr_act)，累计到 TAKEOVER_SAMPLES 帧
        #       再判定 —— 采样分散在正常控制循环中，不额外占用任何时间。
        self.takeover_ok = None          # None=未判定, True/False=已判定
        self.takeover_bad = 0
        self.takeover_n = 0

        # ---- 闭环（结构照抄参考仓库 demo3_gfold.py 的 while True）----
        t0 = time.time()
        game_prev_time = self.sc.ut
        while True:
            time.sleep(self.args.dt)
            # 参考仓库 line 337-340：游戏时间没前进就跳过（等于"每物理帧一次"）
            ut = self.sc.ut
            game_dt = ut - game_prev_time
            if game_dt < 0.01:
                continue
            self.last_pkt = game_dt

            # ================================================================
            # 【2026-09-28 新增：姿态权限与循环时标测量（用户要求）】
            # ----------------------------------------------------------------
            # 【为什么要测】用户判断"姿态跟不上"可能来自三者之一：
            #   (a) 控制循环周期太慢（指令更新频率不够）
            #   (b) 姿态权限不足（力矩不够，尤其在低油门时）
            #   (c) 指令变化太快（律给出的目标倾角跳变）
            #   此前日志无法区分。这里每帧采一次，落进 ④ 那几列。
            # 【耗时】available_torque / moment_of_inertia / angular_velocity
            #   是 3 次轻量 RPC（各约 0.3~1 ms）。为不拖慢循环，
            #   只在【非 dry_run】时读，并用 _t_rpc 计入 rpc_ms 便于核对。
            # ================================================================
            _t_rpc = time.time()
            self.dbg_loop_dt = game_dt
            try:
                # ------------------------------------------------------------
                # 【类型标注（2026-09-28 修 Pylance 报错）】
                #   本文件里 self.v 在类型上是 Optional（见 __init__ 注释与
                #   TYPE_CHECKING 声明），所以每个 self.v.xxx 都会触发
                #   reportOptionalMemberAccess —— 这是本文件【既有的】噪声
                #   类别。用一次局部别名 + 各自标注来收敛。
                #   【另一类报错】kRPC 的 stub 把 available_torque /
                #   moment_of_inertia 标成 Tuple[float,float,float]，
                #   而内置 float() 的参数在 Pylance 眼里是 ConvertibleToFloat，
                #   于是 float(_tau[0]) 被判为「Tuple 传给 float」——
                #   纯属类型推断冲突，运行时完全正常。
                #   这里先整体转成 list[float]，让类型明确，报错随之消失。
                # ------------------------------------------------------------
                _ves = self.v  # type: ignore[assignment]
                # 【2026-09-28 修正：available_torque 是【嵌套】二元组！】
                #   stub 标注为 Tuple[Tuple[float,float,float],
                #                    Tuple[float,float,float]]，
                #   即 (正向力矩三元组, 反向力矩三元组)，各自对应
                #   (pitch, roll, yaw)。
                #   【上一版写错了】直接 for x in available_torque 会拿到两个
                #   tuple，float(tuple) 抛 TypeError，又被下面的
                #   except Exception: pass 静默吞掉 —— 实飞表现为
                #   tau_*/I_*/alpha_max 全是 0 或空（log ..._233003 就是）。
                #   这里取【正向】三元组作为该轴最大可用力矩。
                _tau_pair = _ves.available_torque    # type: ignore[union-attr]
                _tau_l = [float(x) for x in _tau_pair[0]]
                _I_l = [float(x) for x in _ves.moment_of_inertia]  # type: ignore[union-attr]
                _av = np.array(_ves.angular_velocity(              # type: ignore[union-attr]
                    _ves.surface_reference_frame), dtype=float)    # type: ignore[union-attr]
                self.dbg_tau_p = _tau_l[0]
                self.dbg_tau_r = _tau_l[1]
                self.dbg_tau_y = _tau_l[2]
                self.dbg_I_p = _I_l[0]
                self.dbg_I_r = _I_l[1]
                self.dbg_I_y = _I_l[2]
                # 取【三轴里最弱的那一轴】的角加速度：姿态控制受最弱轴限制
                _alphas = []
                for _tq, _in in zip(_tau_l, _I_l):
                    if _in > 1e-6:
                        _alphas.append(abs(_tq) / _in)
                self.dbg_alpha_max = (math.degrees(min(_alphas))
                                      if _alphas else float('nan'))
                self.dbg_avel_mag = math.degrees(float(np.linalg.norm(_av)))
            except Exception as _e:                 # noqa: BLE001
                # 【2026-09-28 改成"首次失败要可见"】
                #   原先这里是裸 pass，导致上面的 TypeError 被静默吞掉，
                #   实飞日志里只看到一列 0，完全查不出原因。
                #   现在首次失败打印一次并写日志，之后不再重复。
                if not getattr(self, '_auth_warned', False):
                    self._auth_warned = True
                    print(f'[WARN] 姿态权限采样失败: {type(_e).__name__}: {_e}')
                    self.log.note(f'auth sample failed: {type(_e).__name__}: {_e}')

            alt, dist, vz, vh, vmag, mass, tp, tv = self.state()
            tt = time.time() - t0
            self.dbg_rpc_ms = (time.time() - _t_rpc) * 1000.0

            # ================================================================
            # 【2026-09-28 新增：检测【游戏被暂停】导致的状态冻结】
            # ----------------------------------------------------------------
            # 【实飞证据】log ..._231236：loop_dt(=game_dt) 从 0.02 一路涨到
            #   1.38 后【恒为 1.3800】不再变化；同时 alt/vz/mass 逐字节相同
            #   （连 mass 都不掉 ⇒ 发动机实际没在工作）。而 wall 时间 t 仍在
            #   增长 ⇒ 是 KSP 被 ESC 暂停 / 失焦，kRPC 一直返回同一帧状态。
            # 【后果（本次失败的直接原因）】控制循环在"空转"：每帧照常算、
            #   照常写执行器，但物理不前进。接管校验把"写进去的 vs 读回来的"
            #   比出 28/40 不符，于是【误报成"节流未被接管"】并放弃控制。
            #   实际根因是游戏暂停，不是 kOS 抢油门。
            # 【判据】用【质量 + 高度连续多帧完全不变】。暂停时二者都冻结；
            #   正常飞行时质量每帧都在掉（油耗），高度也在变。
            # 【为什么不只用 game_dt】暂停时 ut 仍返回旧值，game_dt 会稳定在
            #   某个非零常数，靠它判定不出"冻结"；而质量/高度是硬指标。
            # ================================================================
            _sig = (round(alt, 3), round(mass, 1))
            if _sig == getattr(self, '_frozen_sig', None):
                self._frozen_n = getattr(self, '_frozen_n', 0) + 1
            else:
                self._frozen_n = 0
            self._frozen_sig = _sig
            if self._frozen_n >= 10:
                if not getattr(self, '_frozen_warned', False):
                    self._frozen_warned = True
                    print('[kRPC] >> 检测到游戏状态连续 10 帧完全不变'
                          '（高度与质量都不变）—— 多半是 KSP 被【暂停】。')
                    print('[kRPC] >> 请取消暂停后重试；本程序不再继续写执行器。')
                    self.log.note('GAME FROZEN: alt/mass unchanged for 10 frames'
                                  ' - likely paused')
                    self.log.flush()
                time.sleep(0.2)
                continue
            error = np.array(tp)
            vel = np.array(tv)
            # 采纳后台重解结果（若有）
            self._collect_resolve()
            # 接管校验（非阻塞，逐帧采样；判定为失败则立刻放手）
            self.verify_takeover_step()
            if self.takeover_ok is False:
                print('[kRPC] 接管失败，放开执行器（避免与 kOS 抢控制）')
                self.log.flush()
                break
            # 加速度差分（参考仓库 demo3 line 353: (vel - prev_vel)/game_delta_time）
            if self.prev_vel is not None and game_dt > 1e-6:
                acc = (vel - self.prev_vel) / game_dt
            else:
                acc = np.zeros(3)
            self.prev_vel = vel.copy()

            # ================= 周期性重解（参考仓库 line 440-458）=================
            #   原版：只要 error[0] < start_altitude 就【开后台线程】重解，
            #        用 vessel_profile1() 的外推状态作为新初值。
            #
            # 【2026-09-27 用户要求：默认【不重解】，只跟最初轨迹】
            #   用户原话："我认为不需要重复重算，只让一级跟踪最初的轨迹就好。"
            #   【理由（已从日志验证）】每次重解都会：
            #     · 用外推状态解出新轨迹，起点与本机位置总有偏差
            #       （实测外推超前 138 m → 后修正到 8 m，但不会为 0）
            #     · 重解后 _collect_resolve 把 gfold_n_i 重置为 -100
            #       ⇒ 索引重新定位 ⇒ 采样点跳变 ⇒ 指令跳变
            #     · 日志证据：每次 replan 后 a_cmd_h 都出现一次跳变
            #       （39.67→0.00 / 41.04→0 / 42.91→0 / 44.59→0 …）
            #   ⇒ 重解带来的是【扰动】，而不是精度提升 —— 尤其在本项目
            #     求解已快（codegen ~0.17 s）、单条轨迹质量已被验证的前提下。
            #
            #   【怎么关】`--replan-dt 0`（或设很大）。见下方判据的 `> 0`。
            #   保留重解能力是因为：若实测发现单条轨迹确实跟不上（例如
            #   大扰动、发动机推力偏差），还能一行参数切回来。
            #
            #   【与旧实现的区别】旧版是【同步】重解（阻塞主循环 0.4~0.7 s），
            #     导致跟踪帧被饿死；这里照抄参考仓库用【线程】，
            #     主循环不阻塞，解好了再原子替换 self.plan。
            if (self.args.replan_dt > 0.0
                    and self.nav_mode == 'gfold'
                    and not self._solve_thread_alive()
                    and tt - self.last_replan > self.args.replan_dt
                    and float(error[0]) < START_ALTITUDE):
                # 【2026-09-26 修正】外推时长同样要覆盖本次求解耗时，
                #   否则重解出来的轨迹起点又会落在过去（同样的错位问题）。
                self.est_time = self._predict_est_time()
                x0_rs = self.vessel_profile1(error, vel, acc, mass)
                self._start_resolve(x0_rs, mass)
                self.last_replan = tt

            # ================= 参考仓库的三态导航 =================
            #   gfold : 跟随参考轨迹
            #   final : 进入目标圆柱区后切直接 PID
            #   hold  : 首次解算进行中（只稳住，不机动）
            #   none  : 尚未开始
            if self.nav_mode == 'hold':
                # ---- 等待首次解算：只稳住，不机动 ----
                #   姿态竖直、节流 ≈ 抵消重力（不加速下坠，也不做横向机动）。
                #   对应参考仓库 line 436-438 的"规划未就绪"处理，
                #   目的是【交接后立刻握住执行器】，消除 3.9 s 无控真空。
                if self.plan is not None:
                    self.nav_mode = 'gfold'
                    self.gfold_n_i = -100.0
                    print(f'[G-FOLD] 首次解算完成 tf={self.plan["tf"]:.1f}s '
                          f'落地={self.plan["mass_land"]/1000:.1f}t '
                          f'耗时={self.solve_ms:.0f}ms -> nav_mode=gfold')
                    self.log.note(f'first solve OK tf={self.plan["tf"]:.1f} '
                                  f'solve_ms={self.solve_ms:.0f}')
                else:
                    # ================= 等待首解期间的减速保持 =================
                    # 【2026-09-26 按用户要求改进】
                    #   用户原话："接管后计算好之前不应该不做任何事，应该瞄准
                    #   速度反方向加油门继续减速，但不能太大以至于反冲。"
                    #
                    # 【为什么改】旧实现是【纯竖直】+ 只给"抵消重力 + 一半余量"
                    #   （实测 thr=0.613，净竖直仅 20.3 m/s²）：
                    #     · 竖直减速弱 → 3.6 s 解算期间白白掉高度
                    #     · 水平【完全不管】→ vh 靠重力转向自然衰减，很慢
                    #   而交班时横速可达 221 m/s，不主动消掉等于把难题留给后面。
                    #
                    # 【参考仓库本来就有这个模式】demo3_gfold.py line 428-435：
                    #     elif nav_mode == 'simple':
                    #         target_a = (-vel) * k_v*3 + (-error) * k_x*0
                    #                    + v3(g0, 0, 0)
                    #         a_mag = npl.norm(target_a)
                    #         target_throttle = a_mag / (max_thrust / mass)
                    #         target_direction = target_a / a_mag
                    #         target_direction[0] = max(target_direction[0],
                    #                                   min_tilt_cos)
                    #         target_direction[1:3] *= (1/norm(target_direction[1:3]))
                    #                                * sqrt(1 - target_direction[0]**2)
                    #   即"瞄准速度反方向 + 补重力 + 限制最大倾角"。
                    #   本实现沿用它的公式与结构，只加一道【反冲保护】。
                    #
                    # 【反冲保护（用户明确要求"不能太大以至于反冲"）】
                    #   纯反推的竖直分量 = |a|·|vz|/|v|，在 vz 接近 0 时若仍
                    #   满推力，净竖直可达 +39 m/s² → vz 由负变正 → 火箭上冲。
                    #   两道限制：
                    #     ① 竖直分量上限 = g0 + HOLD_UP_FRAC·(a_cap - g0)
                    #        （只取可用"抬升余量"的一部分，不把竖直用满）
                    #     ② 与 |vz| 挂钩：净减速不超过 |vz|/HOLD_TAU
                    #        （即"至少留 HOLD_TAU 秒才把垂速消到 0"，
                    #          避免在 vz 很小时仍猛推）
                    #   两者取小 ⇒ vz 单调趋近 0 但绝不穿越。
                    a_cap = self.v.max_thrust / max(1.0, mass)
                    vmag_now = float(np.linalg.norm(vel))
                    if vmag_now > 1e-6:
                        # ① 瞄准速度反方向（参考仓库 simple 模式的核心）
                        rev = -vel / vmag_now
                        # ② 竖直分量限制（防反冲的上限）
                        up_cap = G0 + HOLD_UP_FRAC * max(0.0, a_cap - G0)
                        # 若反方向本身就给出较小竖直分量，则不用满推力
                        a_mag = min(a_cap, up_cap / max(1e-6, float(rev[0])))
                        # ③ 与 |vz| 挂钩的净减速上限
                        vz_net_cap = abs(vz) / max(0.1, HOLD_TAU)
                        a_net_now = a_mag * float(rev[0]) - G0
                        if a_net_now > vz_net_cap:
                            # 需要把竖直分量压下来（减少幅值或抬平方向）
                            need_up = vz_net_cap + G0
                            a_mag = min(a_mag, need_up / max(1e-6, float(rev[0])))
                        a_mag = max(0.0, min(a_mag, a_cap))
                        hold = rev * a_mag
                    else:
                        hold = np.array([G0, 0.0, 0.0])
                    hold_dir = hold / max(1e-9, float(np.linalg.norm(hold)))
                    if not self.args.dry_run:
                        self.apply(hold, hold_dir, game_dt)
                    self.tilt_cmd = math.degrees(math.atan2(
                        float(np.linalg.norm(hold[1:3])),
                        max(1e-9, float(hold[0]))))
                    # 姿态实测（供日志，与 gfold 段同口径）
                    if not self.args.dry_run:
                        _fv = self.v.flight(self.v.surface_reference_frame)
                        _c = float(np.dot(_normalize(np.array(_fv.direction)),
                                          np.array([1.0, 0.0, 0.0])))
                        self.tilt_act = math.degrees(math.acos(
                            max(-1.0, min(1.0, _c))))
                    self.log.row(
                        t=round(tt, 3), wall=round(time.time(), 2),
                        alt=round(alt, 2), dist_hz=round(dist, 2),
                        vz=round(vz, 3), vh=round(vh, 3), vmag=round(vmag, 3),
                        mass=round(mass, 1), thr_cmd=round(self.thr_cmd, 4),
                        thr_act=round(self.v.control.throttle, 4),
                        a_cmd_up=round(float(hold[0]), 3),
                        a_cmd_h=round(float(np.linalg.norm(hold[1:3])), 3),
                        a_cmd_mag=round(float(np.linalg.norm(hold)), 3),
                        tilt_cmd=round(self.tilt_cmd, 2),
                        tilt_dir_cmd=round(
                            getattr(self, 'tilt_dir_cmd', float('nan')), 2)
                        if getattr(self, 'tilt_dir_cmd', float('nan')) ==
                        getattr(self, 'tilt_dir_cmd', float('nan')) else '',
                        tilt_act=round(getattr(self, 'tilt_act', float('nan')), 2),
                        att_err='',
                        gnc_phase=0, replan_n=0,
                        solve_ms=round(self.solve_ms, 1),
                        gfold_status='hold(decel, waiting 1st solve)',
                        tf=0.0, tgo=0.0, h_min_plan=0.0,
                        x_err=round(float(np.linalg.norm(error)), 2),
                        v_err=round(vmag, 3),
                        **self._dbg_cols())
                    if alt <= TARGET_ALT + 1.0:
                        print('[done] 等待首次解算期间已触地')
                        break
                    # 【2026-09-26 新增】hold 期间也画一个"待解算"标记，
                    #   否则整个解算窗口（数秒）里屏幕上什么都没有。
                    #   参考仓库是"先建线、后更新"，这里用一个短竖线
                    #   标出【目标点】，提示程序在跑、且目标是哪里。
                    if self.debug_lines:
                        self.draw_target_marker()
                    continue

            if self.nav_mode == 'gfold':
                # 进入 final 段的判据（原版 line 409）
                if (float(np.linalg.norm(error[1:3])) < FINAL_RADIUS
                        and float(np.linalg.norm(error[0])) < FINAL_HEIGHT):
                    print('[nav] 进入 final 段（目标圆柱区内）')
                    self.log.note('nav_mode -> final')
                    self.nav_mode = 'final'

            if self.nav_mode == 'gfold':
                # 原版 line 361-405
                assert self.plan is not None      # hold 分支已保证
                target_a, idx = self.track(self.plan, tp, tv, self.gfold_n_i)
                target_direction = self.target_direction
                plan_x = self.plan['x']
                # 放起落架（原版 line 407-408）
                if (plan_x.shape[1] - self.gfold_n_i) \
                        * self.plan['tf'] / plan_x.shape[1] < 10:
                    self.v.control.gear = True
            else:
                # 原版 line 413-427：final 段
                a_cap = self.v.max_thrust / max(1.0, mass)
                max_acc = 1.0 * a_cap - G0
                max_acc_low = 1.0 * FINAL_THROTTLE * a_cap - G0
                est_h = error[0] - vel[0] ** 2 / (2 * max(1e-9, max_acc))
                est_h_low = error[0] - vel[0] ** 2 / (2 * max(1e-9, max_acc_low))
                denom = (est_h - est_h_low)
                frac = (-est_h_low / denom * (1 + FINAL_KP)) if abs(denom) > 1e-9 else 0.0
                # 【2026-09-28】下限由硬编码 0.05 改为 FINAL_THROTTLE_MIN(0.12)，
                #   理由见该常量的长注释（本载具姿态权限靠 gimbal，正比于油门）。
                #   上限仍为 1.0，本次不动。
                thr = _clamp(_lerp(1.0 * FINAL_THROTTLE, 1.0, frac),
                             1.0, FINAL_THROTTLE_MIN)
                self.v.control.throttle = thr
                self.thr_cmd = thr
                error_hor = np.array([0.0, error[1], error[2]])
                vel_hor = np.array([0.0, vel[1], vel[2]])
                ctrl_hor = -error_hor * 0.03 - vel_hor * 0.06
                td = ctrl_hor + np.array([1.0, 0.0, 0.0])
                td = td / max(1e-9, float(np.linalg.norm(td)))
                # ============================================================
                # 【2026-09-27 修正：补回原版的锥限幅】
                # ============================================================
                # 【原版 demo3_gfold.py:427】
                #     target_direction = conic_clamp(target_direction, 1, 1, max_tilt)
                #   注意参数是 (1, 1, max_tilt)：min_mag=max_mag=1，即
                #   【只做方向限幅、不改幅值】（此时 target_direction 已是单位向量）。
                # 【我此前漏了这一行】于是 final 段可以命令一个超出 max_tilt 的
                #   姿态 ⇒ 触地前姿态可能越界（本项目指标要求触地倾角 < 5°）。
                #   ⇒ 补上。这里直接对单位向量做锥限幅：先把水平分量压到
                #     a_ver*tan(max_tilt) 以内，再归一化。
                # ============================================================
                _t = math.radians(CONIC_TILT_DEG)
                _v = float(td[0])
                _h = float(np.linalg.norm(td[1:3]))
                if _h > 1e-9:
                    _cap = max(0.0, _v) * math.tan(_t)
                    if _h > _cap:
                        td = np.array([_v, td[1] * _cap / _h, td[2] * _cap / _h])
                        td = td / max(1e-9, float(np.linalg.norm(td)))
                # ============================================================
                # 【2026-09-28 修复：final 段油门被 apply() 覆盖成 100%】
                # ============================================================
                # 【故障现象】log ..._193223 帧 44->45（进入 final 段那一帧）：
                #     thr_cmd 0.050 -> 1.000，a_cmd_mag 2.76 -> 55.20
                #   随后 vz 从 -74.1 一路升到 +11.76（【火箭往上飞】），
                #   alt 从 186 m 回升到 187 m 后彻底失控。
                #   【实飞不是能量不够】进入 final 时 189.5 m 高度、
                #   满推力只需 56 m 就能停住 —— 高度是够的。
                #
                # 【根因】本段上面 2147-2156 算出的自杀点火油门 thr
                #   先写进了 control.throttle / self.thr_cmd，
                #   但紧接着 2187 调用 self.apply(target_a, ...)，
                #   而 apply() 内部（1556-1561 行）会【重新用 |target_a| 算油门】：
                #       throttle = |target_a| / a_cap
                #   由于这里 target_a = td * a_cap（td 是单位向量），
                #   ⇒ throttle 恒等于 a_cap/a_cap = 【1.0】
                #   ⇒ 2147-2156 那一段算得再对也被【同帧覆盖】，成为死代码。
                #   ⇒ final 段实际上一直在【满油门】，与自杀点火律无关。
                #
                # 【参考仓库为什么没这个问题】demo3_gfold.py 的 final 分支
                #   （413-427 行）【只算 target_direction，不调用任何写油门的
                #   函数】；真正写 throttle 的是【gfold 分支】的 405 行。
                #   本移植把 apply() 加进了 final 分支，才把这个覆盖引进来。
                #
                # 【修法】让 target_a 的【幅值】等于本段算出的 thrust（而不是
                #   a_cap），这样 apply() 复算的 |target_a|/a_cap 恰好还原 thr。
                #   姿态方向仍用 td，语义与参考仓库一致。
                target_a = td * thr * a_cap
                target_direction = td
                idx = self.gfold_n_i
                if not self.args.dry_run:
                    # final 段姿态仍用同一套 PID
                    #   （油门已由上面的 target_a 幅值承载，apply() 会还原它）
                    self.apply(target_a, target_direction, game_dt)

            if self.args.dry_run:
                # dry-run 不写执行器（apply 里才更新 thr_cmd）
                self.thr_cmd = 0.0
            else:
                if not self.engaged:
                    break
                if self.nav_mode == 'gfold':
                    self.apply(target_a, target_direction, game_dt)
                    # apply() 内部会写 self.thr_cmd / self.tilt_act

            # 画参考轨迹（对齐参考仓库 line 326-327：解完就更新线，
            #   与 dry_run / nav_mode 无关）。用 --debug-lines 控制开关。
            # 【2026-09-26 修正】plan 为 None 时不能画（旧版会 TypeError 被
            #   except 吞掉，表现为"完全看不到线"且毫无提示）。这里显式判断。
            if self.debug_lines and self.plan is not None:
                self.draw(self.plan)

            # 姿态实测
            # 【2026-09-26 修正】旧版这里写 `tilt_act = self.tilt_cmd`，
            #   于是日志的 tilt_act 列【只是命令的回声】，看不出姿态有没有跟上。
            #   现在 tilt_act 是 apply() 里从机头方向（flight().direction）
            #   实测的真实倾角。
            att_err = self.att_err
            tilt_act = getattr(self, 'tilt_act', float('nan'))

            # ---- 公共日志 ----
            # 【pyright 提示的风险】这行读 self.plan['tf'] / self.plan['x']。
            #   hold 分支带 continue（不会走到这里），gfold/final 分支都要求
            #   plan 已就绪 —— 所以正常情况下安全。但为防将来改动引入
            #   "final 段在 plan 就绪前进入"这类路径，这里显式兜底，
            #   避免一个 TypeError 让整个 kRPC 进程退出（火箭失控）。
            _plan = self.plan
            _tf = float(_plan['tf']) if _plan is not None else 0.0
            _hmin = (float(_plan['x'][0, :].min())
                     if _plan is not None else 0.0)
            self.log.row(
                t=round(tt, 3), wall=round(time.time(), 2),
                alt=round(alt, 2), dist_hz=round(dist, 2),
                vz=round(vz, 3), vh=round(vh, 3), vmag=round(vmag, 3),
                mass=round(mass, 1), thr_cmd=round(getattr(self, 'thr_cmd', 0), 4),
                thr_act=round(self.v.control.throttle, 4),
                a_cmd_up=round(float(target_a[0]), 3),
                a_cmd_h=round(float(np.linalg.norm(target_a[1:3])), 3),
                a_cmd_mag=round(float(np.linalg.norm(target_a)), 3),
                tilt_cmd=round(self.tilt_cmd, 2),
                tilt_dir_cmd=round(getattr(self, 'tilt_dir_cmd',
                                           float('nan')), 2)
                if getattr(self, 'tilt_dir_cmd', float('nan')) ==
                getattr(self, 'tilt_dir_cmd', float('nan')) else '',
                tilt_act=round(tilt_act, 2),
                att_err=round(att_err, 2) if att_err == att_err else '',
                gnc_phase=idx, replan_n=self.replan_n,
                solve_ms=round(self.solve_ms, 1), gfold_status=self.gfold_status,
                tf=round(_tf, 2),
                tgo=round(max(0.0, _tf
                              * (1 - max(0.0, idx)
                                 / max(1, (_plan['x'].shape[1] - 1)
                                       if _plan is not None else 1))), 2),
                h_min_plan=round(_hmin, 2),
                x_err=round(float(np.linalg.norm(error)), 2),
                v_err=round(vmag, 3),
                **self._dbg_cols())


            if int(tt * 2) % 4 == 0:
                print(f'  t={tt:5.1f} alt={alt:7.1f} dist={dist:6.1f} '
                      f'vz={vz:7.2f} vh={vh:6.2f} thr={getattr(self, "thr_cmd", 0):.3f} '
                      f'tilt={tilt_act:5.1f}')

            # 结束条件
            if alt <= TARGET_ALT + 1.0 or self.v.situation.name == 'landed':
                print(f'[done] alt={alt:.2f} vz={vz:.2f} vh={vh:.2f} '
                      f'dist={dist:.2f} tilt={tilt_act:.2f}')
                self.log.note(f'LANDED alt={alt:.2f} vz={vz:.2f} vh={vh:.2f} '
                              f'dist={dist:.2f} tilt={tilt_act:.2f}')
                break
            if vz > 5.0:
                print('[abort] 检测到爬升，退出')
                break
            time.sleep(self.args.dt)

        if not self.args.dry_run:
            self.v.control.throttle = 0.0
        self.log.flush()
        return 0

    def _ensure_lines(self, N):
        """【启动时一次性】创建线对象（照抄参考仓库 line 233-234）。

        【2026-09-26 第二次修正：为什么仍然看不到线】
          上一版我每次 draw() 都【先 remove 再 add_line】。参考仓库不是这样：
              line 234：启动时一次性 add_line() 建好 N-1 条线
              line 248-255：update_lines() 只更新 .start / .end
          在 KSP 里每帧"删掉再新建"线对象会导致它们根本来不及显示
          （对象生命周期只有一帧），表现为"完全看不到线"。
          ⇒ 必须改成【建一次、之后只改端点】。
        """
        if getattr(self, '_lines_N', None) == N and self.drawn:
            return
        # 清掉旧的（只在轨迹点数变化时重建）
        for ln in getattr(self, 'drawn', []):
            try:
                ln.remove()
            except Exception:                        # noqa: BLE001
                pass
        self.drawn = []
        fr = self.target_frame()
        try:
            for _ in range(max(1, N - 1)):
                ln = self.conn.drawing.add_line((0, 0, 0), (0, 0, 0), fr)
                ln.color = (255, 60, 60)
                ln.width = 2.0
                self.drawn.append(ln)
            self._lines_N = N
            self._lines_fr = fr
            print(f'[draw] 已创建 {len(self.drawn)} 条轨迹线')
        except Exception as e:                       # noqa: BLE001
            if not getattr(self, '_draw_warned', False):
                self._draw_warned = True
                print(f'[WARN] 创建轨迹线失败: {type(e).__name__}: {e}')
                print('[WARN] 检查 kRPC 的 drawing 服务是否可用')

    def draw_target_marker(self):
        """在目标点画一个竖直短线（hold 期间的可见标记）。

        【为什么需要】首解要数秒，那段时间还没有轨迹可画 ——
        屏幕上什么都没有，无法判断"程序是不是在跑"。这条短线标出
        目标点位置，起"我还活着 + 目标在哪"的作用。

        【实现】沿用参考仓库"建一次、只改端点"的做法（line 233-234）：
        线对象只在第一次创建，之后只改 start/end，不反复 add/remove。
        """
        try:
            fr = self.target_frame()
            if getattr(self, '_marker', None) is None:
                ln = self.conn.drawing.add_line((0, 0, 0), (0, 0, 0), fr)
                ln.color = (60, 200, 60)         # 绿色：目标标记
                ln.width = 3.0
                self._marker = ln
                self._marker_fr = fr
                print('[draw] 已创建目标点标记线')
            # 从目标点向上 30 m（目标点在原点，x=上）
            self._marker.start = (0.0, 0.0, 0.0)
            self._marker.end = (30.0, 0.0, 0.0)
        except Exception as e:                       # noqa: BLE001
            if not getattr(self, '_marker_warned', False):
                self._marker_warned = True
                print(f'[WARN] 目标标记线创建失败: {type(e).__name__}: {e}')

    def draw(self, r):
        """更新参考轨迹线（【建一次、只改端点】，照抄参考仓库 update_lines）。

        【为什么以前"看不到画线"—— 两次修正】
          ① 旧实现把 self.draw() 放在 `if self.args.dry_run:` 里，
             正常飞行（dry_run=False）根本不调用 ⇒ 一个点都不画。
          ② 修掉①后仍然看不到：因为每次都是"remove 旧线 + add_line 新线"，
             对象只活一帧。参考仓库是【启动建一次，之后只改 start/end】。
        """
        try:
            if r is None or r.get('x') is None:
                return                       # 还没解出轨迹，无可画
            x = r['x']
            N = x.shape[1]
            self._ensure_lines(N)
            if not self.drawn:
                return
            step = max(1, N // 40)
            idxs = list(range(0, N, step))
            if idxs[-1] != N - 1:
                idxs.append(N - 1)
            # 只更新端点（不删不建）
            for k in range(len(idxs) - 1):
                if k >= len(self.drawn):
                    break
                i0, i1 = idxs[k], idxs[k + 1]
                self.drawn[k].start = (float(x[0, i0]), float(x[1, i0]),
                                       float(x[2, i0]))
                self.drawn[k].end = (float(x[0, i1]), float(x[1, i1]),
                                     float(x[2, i1]))
            # 多出来的线收成一点（不可见），避免残留旧轨迹
            for k in range(len(idxs) - 1, len(self.drawn)):
                self.drawn[k].start = (0.0, 0.0, 0.0)
                self.drawn[k].end = (0.0, 0.0, 0.0)
        except Exception as e:                       # noqa: BLE001
            if not getattr(self, '_draw_warned', False):
                self._draw_warned = True
                print(f'[WARN] draw() 异常: {type(e).__name__}: {e}')


def list_vessels(conn):
    """列出场景内所有 vessel，并标出【哪个会被选中】，然后退出。

    【为什么需要这个】两次实飞选错船（kOS 侧一级 224.6 t，kRPC 侧拿到
    726~761 t 的整箭/二级）。定位必须看到场景内全部船及其物理量。
    这个模式【不用等交班门】，进游戏后随时可跑：
        python gfold_land.py --list-vessels
    建议在【一级分离之后】跑一次，那时才能看到真正的回收一级。
    """
    sc = conn.space_center
    body = sc.active_vessel.orbit.body
    print('=' * 92)
    print('场景内所有 vessel（★ = 会被自动选中）')
    print('=' * 92)
    print(f'{"":2} {"name":26} {"type":8} {"situation":11} {"mass(t)":>9} '
          f'{"thr(MN)":>8} {"a_net":>7} {"alt(m)":>9} {"vs(m/s)":>8}')
    rows = []
    for v in sc.vessels:
        try:
            fl = v.flight(body.reference_frame)
            ms = v.mass
            th = v.max_thrust
            a_net = (th / ms - 9.80665) if ms > 1 else float('-inf')
            rows.append(dict(v=v, name=v.name, type=v.type.name,
                             situation=v.situation.name, mass=ms, thrust=th,
                             a_net=a_net, alt=fl.surface_altitude,
                             vspeed=fl.vertical_speed))
        except Exception as e:                     # noqa: BLE001
            print(f'   <读取失败: {e}>')
    # 与 pick_vessel 相同的判据
    best = None
    for d in rows:
        d['reason'] = GfoldLander._reject(d)
        d['ok'] = d['reason'] is None
    cands = [d for d in rows if d['ok']]
    if cands:
        cands.sort(key=lambda d: abs(d['a_net'] - 50.0))
        best = cands[0]
    for d in rows:
        mark = '★' if (best is not None and d is best) else ' '
        flag = '' if d['ok'] else ('  排除: ' + str(d['reason']))
        print(f'{mark:2} {d["name"]:26} {d["type"]:8} {d["situation"]:11} '
              f'{d["mass"]/1000:9.1f} {d["thrust"]/1e6:8.2f} '
              f'{d["a_net"]:7.1f} {d["alt"]:9.0f} {d["vspeed"]:8.1f}{flag}')
    print()
    if best is not None:
        print(f'将被选中: {best["name"]!r}  mass={best["mass"]/1000:.1f} t  '
              f'a_net={best["a_net"]:.1f} m/s²   alt={best["alt"]:.0f} m')
        print('判定依据：a_net 15~80、正在下降、高度 100~25000 m、非 debris')
    else:
        print('没有符合条件的回收一级 —— 请确认：')
        print('  · 一级是否已分离？（分离前应选到"整箭"，a_net 只有 1~3）')
        print('  · 是否在回收下降段？（上升段 vspeed > 0，不会被选）')
        print('  · 高度是否在 100~25000 m？')


# ------------------------- kOS 交班行解析 -------------------------
# kOS 侧在交班时打印一行（格式固定，见 boot/B1040-9.ks）：
#   GFOLD_HANDOFF lat=.. lon=.. alt=.. t=.. dist=.. v=.. vz=.. vh=.. mass=.. id=.. name=..
#
# 【为什么要这一行（备用方案）】kRPC 侧按名字/active_vessel 选船都失败过：
#   · active_vessel 在一级分离后变成二级；
#   · 场景里 'B1040-9'、'B1040-9 中继卫星'、'B1040-9的残骸' 名字互相包含；
#   · KSP 让二级/载荷继承原 vessel id，真正的回收一级是【新 vessel】。
#   ⇒ 改由 kOS【广播自己的精确位置】，kRPC 选"离该点最近的那艘船"。
#      这条路径不依赖名字，也不依赖 vessel id 继承规则。
HANDOFF_RE = re.compile(
    r'GFOLD_HANDOFF\s+lat=(?P<lat>[-0-9.]+)\s+lon=(?P<lon>[-0-9.]+)'
    r'\s+alt=(?P<alt>[-0-9.]+)\s+t=(?P<t>[-0-9.]+)'
    r'\s+dist=(?P<dist>[-0-9.]+)\s+v=(?P<v>[-0-9.]+)'
    r'\s+vz=(?P<vz>[-0-9.]+)\s+vh=(?P<vh>[-0-9.]+)'
    r'\s+mass=(?P<mass>[-0-9.]+)'
    # 【不要期望 id=】kOS 的 Vessel 没有 ID 后缀（用了会终止脚本），
    #   所以广播行里只有 type= 和 name=。定位完全靠 lat/lon/alt。
    r'(?:\s+type=(?P<type>\S+))?'
    r'(?:\s+name=(?P<name>.*))?')

# kOS 的 Archive 卷在本机的落盘位置（用来读它的打印输出）
KOS_ARCHIVE_DIRS = [
    r'D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script',
    r'D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script\boot',
]


def find_handoff_line(max_age_s=120.0):
    """在 kOS 的 Archive 卷里找最近的 GFOLD_HANDOFF 行。

    返回 dict（含 lat/lon/alt/.../id/name）或 None。
    kOS 的 LOG ... TO 只写 Archive 卷；print 只到终端。为了不依赖"从终端
    复制粘贴"，这里约定 kOS 同时把该行【写进一个固定的状态文件】——
    见 boot/B1040-9.ks 的 gfold_state 写入逻辑。
    """
    cand = []
    for d in KOS_ARCHIVE_DIRS:
        try:
            for fn in os.listdir(d):
                if fn.startswith('gfold_state'):
                    cand.append(os.path.join(d, fn))
        except OSError:
            continue
    best = None
    for p in cand:
        try:
            if time.time() - os.path.getmtime(p) > max_age_s:
                continue
            with open(p, 'r', encoding='utf-8', errors='replace') as f:
                for line in f:
                    m = HANDOFF_RE.search(line)
                    if m:
                        best = m.groupdict()
        except OSError:
            continue
    return best


def main():
    ap = argparse.ArgumentParser(description='B1040-9 末段 G-FOLD 着陆（kRPC）')
    ap.add_argument('--list-vessels', action='store_true',
                    help='只列出场景内所有 vessel 后退出（诊断用，不用等交班门）')
    ap.add_argument('--dry-run', action='store_true', help='只解算+画线，不接管控制')
    ap.add_argument('--no-autopilot', action='store_true', help='不控姿态')
    ap.add_argument('--no-log', action='store_true', help='不写日志')
    ap.add_argument('--no-debug-lines', action='store_true',
                    help='不画参考轨迹线（默认画，与参考仓库 debug_lines=True 一致）')
    ap.add_argument('--dt', type=float, default=0.02, help='循环周期 [s]')
    # 【2026-09-27 用户要求】默认【不重解】(0 = 关闭)，只跟最初那条轨迹。
    #   理由：日志验证每次重解都会让 a_cmd_h 跳变（39.67→0 / 41.04→0 /
    #   42.91→0 / 44.59→0 …），带来的是扰动而非精度。
    #   想恢复周期重解：`--replan-dt 2.0`。
    ap.add_argument('--replan-dt', type=float, default=0.0,
                    help='重解周期 [s]；0 = 不重解（默认），只跟踪最初轨迹。'
                         '设为正数则按该周期重解（参考仓库行为是持续重解）。')
    ap.add_argument('--est-time', type=float, default=0.5,
                    help='状态外推时长 [s]（参考仓库 vessel_profile1 的 est_time=0.5）。'
                         '用于补偿"规划耗时 + 执行延迟"：把 x0 外推到解出来那一刻。')
    ap.add_argument('--gate-timeout', type=float, default=600.0, help='等交班门超时 [s]')
    ap.add_argument('--vessel-name', default='B1040-9',
                    help='回收一级的名字（一级分离后 active_vessel 会变成二级，'
                         '所以必须按名字选）')
    ap.add_argument('--address', default='127.0.0.1')
    ap.add_argument('--rpc-port', type=int, default=50000)
    ap.add_argument('--stream-port', type=int, default=50001)
    args = ap.parse_args()

    import krpc
    print(f'[conn] {args.address}:{args.rpc_port} …')
    conn = krpc.connect(name='B1040-9 GFOLD', address=args.address,
                        rpc_port=args.rpc_port, stream_port=args.stream_port)
    # 【防御】这只是打印服务器版本，绝不该因为取不到就让整个程序退出。
    #   pyright 认为 conn.krpc 可能为 None（krpc 的 stub 把它标成 KRPC | None），
    #   实际 krpc.connect() 成功后必然有 .krpc；这里仍用 try 兜住，
    #   属于"分析器与运行时约定不一致"，故加针对性 ignore 而非改逻辑。
    try:
        _st = conn.krpc.get_status()          # type: ignore[union-attr]
        _ver = getattr(_st, 'version', '?') if _st is not None else '?'
    except Exception as e:                          # noqa: BLE001
        _ver = f'<取版本失败: {type(e).__name__}>'
    print('[conn] server version =', _ver)
    try:
        if args.list_vessels:
            list_vessels(conn)
            return 0
        return GfoldLander(conn, args).run()
    finally:
        conn.close()


if __name__ == '__main__':
    sys.exit(main())
