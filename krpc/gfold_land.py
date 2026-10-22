#!/usr/bin/env python3
# gfold_land.py — B1040-9 末段 G-FOLD 着陆控制端（kRPC）
#
# ============================================================================
# 【2026-10-08 重写版】单一 tilt-first 控制律，无旧律、无开关
# ----------------------------------------------------------------------------
# 旧版（gfold_land_old.py，6076 行）积累了 18 个编号改动步，问题有：
#   · 两条并行律：旧律（照抄参考仓库）与新律（倾角第一性）并存，
#     由 TILT_FIRST_ENABLE 切换，另有 try/except 回退分支
#   · 同一个物理量有多个变量名（_amag/av_plan、tilt_cmd/tf_cmd/tf_tilt_prev…）
#   · 竖直指令不是"竖直律"，而是 |target_a|*cos(tilt) —— 幅值取自旧律 PD 向量，
#     于是【水平 PD 修正的幅值被当成竖直推力发出去】（实测恒等式
#     a_cmd_h == a_cmd_mag*sin(tilt)，偏差 < 0.003；去掉 conic_clamp 托底后
#     竖直 PD 原始值在 58.5% 的帧为负）
#   · 149 列日志里约 22 列恒空/恒定
#
# 本重写版的结构：
#   ┌ 外环：G-FOLD 规划给出 x/v/u，采样当前点与前赡点
#   │ 水平律：a_h_vec = u_h + kv*dve + kp*dp,  T = clamp(2|dp|/|dve|, T_MIN, T_MAX)
#   │        kv = 4/T, kp = 6/T^2  =>  zeta = 0.816 与 T 无关
#   ├ 竖直律：【真正的竖直 PD】 u_r[0] + K_VEL*dvz + K_POS*dalt
#   │        再受三层约束：a_ver_need 上界 / 正下限 / < g0
#   └ 内环：状态是 a_h（不是倾角），限【加加速度】而非角速率
#           j_max = a_up*sec^2(theta)*sqrt(2*alpha*headroom)   <- 由权限在线导出
#           tilt = atan2(a_h, a_up)  <- 由 a_h 反解，前后同一个 a_up
#
# 【措施2 与措施3 在此合一】
#   措施2（内环限加加速度）：状态量从 tilt 改为 a_h，限 |da_h/dt| <= j_max。
#     依据：现行"限倾角速率"给出的等效 jerk 只有 1.40 m/s^3，而制导需求
#     2.20（59.3% 的帧超限）；物理上限 7.10，尚有 3.2 倍余量。
#   措施3（口径闭环）：tilt 由【同一个 a_up】反解，不再有
#     "前向用 a_up_est、后向用 av_up" 的往返失配（实测该失配 p10=0.518 /
#     p90=1.721，单独修会让 p90 恶化 46%，故必须与措施2 同时做）。
#
# ============================================================================
# 【本文件的其余部分仍逐行沿用参考仓库 dev/gfold/demo3_gfold.py（GFOLD_KSP）】
#   参考仓库是【唯一验证过的 G-FOLD 实飞实现】，凡是它有的都照抄，
#   只改与载具数值相关的参数。对应关系：
#       demo3_gfold.py                本文件
#       ─────────────────────────    ──────────────────────────────
#       GFOLD_direct_exec (P3/P4)  → dev/solver/gfold_p3p4.py
#       ref_target (224-225)       → target_frame()
#       find_nearest_index(258-270)→ find_nearest_index()
#       sample_index(273-290)      → sample_index()
#       conic_clamp (292-314)      → conic_clamp()
#       G-FOLD 跟随 (360-412)      → track()
#       PID 姿态 (461-477)         → PID 类 + apply()
#
# 【架构位置】
#   段1（12.4 km → ~5 km）: kOS  boot/B1040-9.ks 两级律
#   段2（~5 km → 地面）   : 本程序（kRPC + G-FOLD 凸优化）
#
# 【为什么不用 auto_pilot】参考仓库自己写 PID 直接写 control.pitch/yaw/roll。
#   曾用 auto_pilot 踩过三个坑：never-engaged、错误的 engage() 名称、
#   与 kOS 抢 control 通道。照抄参考仓库即可全部避开。
#
# 用法：
#   python gfold_land.py                # 正常接管
#   python gfold_land.py --dry-run      # 只解算+画线，不接管控制（推荐首飞）
#   python gfold_land.py --no-autopilot # 只接管节流，姿态手动
# ============================================================================


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
#   【两个位置都可用的写法】优先 ../solver（本项目仓库内），
#   不存在时回退 ../gfold/solver（上游克隆目录），保证两者都能跑。
#   【为什么不用 for 循环】tools/check_names.py 对模块级 for 的循环变量
#   会先检查 load 再登记绑定，导致 _cand 被误报为"未定义"。
#   这里改用显式两次判断，语义等价且静态检查干净。
_SOLVER_DIR = os.path.join(_HERE, '..', 'solver')
if not os.path.isdir(_SOLVER_DIR):
    _SOLVER_DIR = os.path.join(_HERE, '..', 'gfold', 'solver')
sys.path.insert(0, _SOLVER_DIR)

from gfold_p3p4 import solve_p3p4_state        # noqa: E402 参考仓库结构


# ------------------------- 常量（来自项目文档，非猜测）-------------------------
# 【标准重力加速度】用求解器同值，避免控制侧与规划侧口径不一致。
G0 = 9.80665

# ---- 着陆点（Kerbin，与 kOS 侧一致）----
TARGET_LAT = -0.0972060951675948
TARGET_LON = -74.5576822740041
TARGET_ALT = 17            # 着陆腿触点高度 [m]
TARGET_HEADING_DEG = 90.0

# ---- 【2026-10-08 改动 2】末段抬高量 [m] ----
#   【为什么需要】原规划把"速度归零"点放在 x[0]=0 <=> alt=TARGET_ALT，
#   而控制器的停机判据是 alt <= TARGET_ALT+1 —— 两者几乎重合。
#   实测六趟的落地垂速 -1.6 ~ -10.8 m/s：载具在规划"刹停"之前就触地了，
#   继承的是规划倒数第二节点的速度（plan_vz 末帧仍有 -2.2 ~ -9.3）。
#   ⇒ 这是"刹车距离太短"，不是"规划没考虑减速"。
#
#   【做法】让求解器把归零点抬高 TERM_CLEARANCE：
#     交给求解器前把高度减 TERM_CLEARANCE，取回规划后加回来。
#     于是规划末节点对应 alt = TARGET_ALT + TERM_CLEARANCE，
#     而跟踪用的坐标系【不变】（仍是相对 TARGET_ALT）。
#   【配套】最后 TERM_CLEARANCE 米由【末段下降律】接管
#     （否则规划要求 v=0，载具会在抬高点【悬停】而不是落地）。
TERM_CLEARANCE = 5.0
#   末段下降律的目标垂速 [m/s]（正数表示向下）。与 AVER_TGT_VZ 同口径。
TERM_DESCENT_VZ = 2.0

# ============================================================================
# 【2026-10-09 P3：FIN 触地保持段 —— 停机后不再"完全无控"】
# ----------------------------------------------------------------------------
# 【要解决的问题（用户实测指出 + 视频证实）】
#   原停止条件（run() 末尾）是：
#       if alt <= TARGET_ALT + 1.0 or situation == 'landed':
#           ... break            <- 跳出主循环
#       self.v.control.throttle = 0.0     <- 油门清零
#       return 0                          <- main() 的 finally 关闭 kRPC 连接
#   即【姿态与油门全部撒手】，程序退出。视频证据（两趟真帧）：
#     · 引擎在停机后 ~0.1 s 内熄灭（V1 火焰像素 844->426->69->3）
#     · 之后载具【静止约 1 s】，随后【缓慢单向倾覆】，直到操作者用
#       VesselMover 接管（V1 绿椭圆 36.53 s；倾覆始于 33.3 s => 属自由物理）
#   日志证据：末帧还有残余横速与倾角
#       154838: vh=3.77 tilt=5.52   (plan_vh 要求 0.01)
#       160001: vh=1.10 tilt=6.00
#   而【既有末段律 _term_phase 明确"水平不动"】
#     （track() 内注释原文："水平】不动 —— 规划要求 vh=0，
#        继续由水平律收敛"）=> 它并不主动消横速。
#   ⇒ 带 1.1~3.8 m/s 横速 + 5.5~6.0° 残余倾角触地 => 滑移 => 缓慢倾倒。
#
# 【本段做什么】
#   把"停机即撒手"改成【进入 FIN 触地保持段】，继续写到真正触地
#   (situation == 'landed') 为止：
#     · 姿态：瞄准【天顶】（径向向外，即 tilt -> 0），压住残余倾角
#     · 油门：P-D 环把 vz 压到 FIN_DESCENT_VZ（=2 m/s 软着陆）
#     · 水平：继续用同一 P-D 消 vh（补齐 _term_phase 缺失的那一环）
#   用【最简单的 PID】（用户原话："用最简单的 pid 控制"）。
#
# 【物理边界（必须知道，别指望它救急）】
#   实测姿态权限里有一个【与油门无关】的兵底（反作用轮）：
#       alpha = k*thr + b，b 在 4 趟里恒为正（5.29/5.97/6.75/7.65 deg/s^2）
#   而 20~40 m 高箭体的【重力倾覆率】约 21~42 deg/s^2（per sin(tilt)），
#   远大于该兵底。
#   ⇒ 一旦倾角越过支撑多边形，反作用轮【物理上救不回来】。
#   ⇒ 本段是【预防性】的（把落地时的残余横速/倾角压小），
#     不是"倒了能扶正"。
# ============================================================================
FIN_ENABLE = True            # False = 退回旧行为（停机即撒手）
FIN_ALT = 1.0                # 进入 FIN 段的相对高度 [m]（TARGET_ALT 之上）
#   【用户指定"沿用"原停止条件】原判据是 alt <= TARGET_ALT + 1.0
#     （TARGET_ALT=17 => 18.0 m），故这里取 1.0，与原判据【逐字一致】。
#   ⚠️【必须知道的后果（离线实测）】在 18 m 处只剩 1.0 m 行程，
#     以 vz≈-3.8 m/s 计约 0.26 s 就触地。即本段的【主动控制权限很短】。
#     它的价值在两处：
#       ① 不再"油门清零 + 程序退出"（姿态保持到真正触地，
#          而不是让载具静止 1 s 后自行缓慢倾倒）
#       ② 对"TARGET_ALT 是否真的等于腿触点"这一【未解疑点】做保险：
#          日志显示 [done] 时 alt=17.24~17.26，而视频显示引擎熄灭后
#          载具仍继续下沉约 2 m、历时约 0.8 s。若真实腿触点低于 17 m，
#          本段就会持续工作到 situation=='landed' 为止。
#     【若想让它作用更久】把本常量改为 5.0（= TARGET_ALT+5 = 22 m，
#       即与 TERM_CLEARANCE 的规划归零面重合）。离线实测该处
#       可把落地垂速从 -3.8 收到 -2.7、并多出约 0.5 s 消横速。
FIN_TIMEOUT = 20.0           # FIN 段最长持续 [s]（防止卡死）
FIN_TAKEOFF_GUARD = 3.0      # FIN 段内 vz > 该值持续上升即判定"反冲"，退出
FIN_DESCENT_VZ = 2.0         # FIN 段目标下降率 [m/s]（正数=向下）
FIN_KP = 0.8                 # 预留的 P 增益（当前未叠加，见 fin_control 的说明）
FIN_KD = 0.0                 # D 阻尼增益
#   ⚠️【为什么 KD = 0（实测，反直觉但确凿）】
#     本律的竖直指令取 a_need（"在剩余高度内恰好收到目标垂速"）。
#     解析上它已在 h=0 时把 vz 精确收到 -FIN_DESCENT_VZ：
#         d(vz^2)/dh = (vz^2 - vt^2)/h  =>  vz^2 - vt^2 = K*h
#         => h=0 时 vz^2 = vt^2  => |vz| = vt。
#     而减速过程中 d(vz)/dt > 0，叠加 "-FIN_KD*d(vz)/dt" 会【减少刹车】。
#     离线积分实测（5 个工况，落地 |vz| 目标 2.0）：
#         KD=0.00 -> -2.68/-2.66/-3.07/-3.69/-7.43  sum|err|= 9.54
#         KD=0.05 -> -2.80/-2.76/-3.27/-3.98/-7.51  sum|err|=10.32
#         KD=0.10 -> -2.92/-2.88/-3.34/-4.30/-7.59  sum|err|=11.04
#         KD=0.45 -> -3.70/-3.62/-3.90/-6.35/-10.21 sum|err|=17.79
#     => KD 越大越差，KD=0 最优。故置 0。
FIN_VH_KP = 0.35             # vh -> 水平加速度的 P 增益
FIN_VH_KD = 0.30             # vh -> 水平加速度的 D 增益
FIN_TILT_MAX_DEG = 8.0       # FIN 段允许的最大倾角指令 [deg]（近竖直）
FIN_UP_MIN = 0.35            # FIN 段竖直推力下限 = 该比例 * 悬停加速度
#   【为什么必须有下限】油门为 0 时 gimbal 不产生力矩；虽然反作用轮
#     提供了 ~6.5 deg/s^2 的兵底，但保留一点油门能显著改善姿态保持。
#     0.35 是"远小于悬停（不会反冲）"与"仍有力矩"的折中。
FIN_TILT_HOLD = True         # 姿态瞄准天顶（径向向外）

# ---- 交班门 ----
#   【16(d)/17(a)】AIM_ALT 把瞄准点抬高，与 kOS 的 aim_alt 对齐。
#   【必须与 boot/B1040-9.ks 的 aim_alt 一致】两边描述同一个瞄准点。
#
#   ================================================================
#   【2026-10-09 P2：AIM_ALT 1000 -> 500（用户实测指出预瞄点太高）】
#   ----------------------------------------------------------------
#   【为什么降】AIM_ALT 直接决定 kOS 的交班高度：
#       GATE_ALT = TARGET_ALT + AIM_ALT + GATE_AIM_H
#       1000 -> 交班 1167 m ; 500 -> 交班 667 m
#   交班越高，kOS 段1 就要多飞约 500 m 的纯竖直距离才肯交班，
#   而这段时间里载具【继续朝目标点水平漂移】——即用户看到的"浪费距离"。
#
#   【收益（离线实测，4 个真实交班状态）】
#     交班 1167 -> 667 m，gfold 段缩短，tf 与燃料都省：
#       160001: tf 22.50 -> 11.75 s, 落地 188.0 -> 190.6 t
#       150743: tf 22.50 -> 12.50 s, 落地 142.0 -> 146.9 t
#     且在大横速下仍可行：vh=45 时 aim=500 依然 optimal（tf=14.25~20.50）。
#
#   ⚠️【必须知道并接受的风险（kOS 里记录的历史否决理由）】
#     交班高度降到 667 m 后，到 kRPC 兜底门 GATE_ALT_MIN=358 只剩 309 m。
#     若 kOS 段1 交班时横速仍达 35~45 m/s（历史上确实出现过：实测
#     154838 交班 vh=36.2、20261007 若干趟 vh=42~48），
#     则段1 消横速所需的下坠距离会吃掉大部分余量。
#     历史结论（boot/B1040-9.ks 第 238~248 行的长注释）：
#       "aim_alt=500 ⇒ 交班 686.8 m，扣掉 160~310 m 只剩 ~380 m，
#        距 GATE_ALT_MIN=358 只有 22 m 余量" —— 因此当年否决了 500。
#     ⇒ 本次【按用户要求改到 500】，但这条风险【未消除】。
#       若实飞出现"交班太晚/落不下来"，第一优先级是把 AIM_ALT 调回
#       700~1000，而不是去调包络或增益。
#   ================================================================
AIM_ALT = 500.0
GATE_AIM_H = 150.0
GATE_ALT = TARGET_ALT + AIM_ALT + GATE_AIM_H
GATE_VMAX = 120.0
GATE_CONE_DEG = 48.0
GATE_ALT_MIN = 358.0
START_ALTITUDE = GATE_ALT

# ---- 求解器参数（逐项对应参考仓库 params.txt）----
ISP_DEFAULT = 315.0        # SSME 海平面 Isp
TF_GUESS = 40.0
STRAIGHT_FAC = 5.0
GS_DEG = 30.0
PCS_START_DEG = 85.0
PCS_END_DEG = 5.0
V_MAX_SOLVER = 1200.0
N3 = 160
N4 = 80
# 下降率包络（必须与 codegen 期一致，见 gfold_codegen.py）
V_DESCENT_MAX_ENABLE = 80.0
V_DESCENT_TERM_WIN = 0.40
V_DESCENT_TERM_VZ = 4.0

# ---- 参考方向斜率限制（物理判据）----
#   allowed_slew = sqrt(2*alpha*headroom)，headroom = 允许的角行程。
#   物理含义：从静止加速到 slew、再同等减速停住，总角行程正好 = headroom。
REF_SLEW_ENABLE = True
REF_SLEW_HEADROOM_DEG = 45.0
REF_SLEW_MIN = 8.0             # 下限（避免权限极低时参考冻结）
REF_SLEW_MAX = 45.0            # 上限（避免高油门时参考过快）

# ---- 姿态环（照抄参考仓库 PID 形式）----
CTRL_X_ROT_KP = 2.0
CTRL_X_ROT_KD = 6.0
CTRL_Z_ROT_KP = 2.0
CTRL_Z_ROT_KD = 6.0
CTRL_Y_AVEL_KP = 2.0
RATE_D_GAIN = 0.78        # D 项用真实角速度后的阻尼增益
RATE_D_ENABLE = True      # False = 回到对误差差分
FF_ENABLE = True          # 参考方向速率前馈
FF_GAIN = 0.60
FF_CLAMP_RAD = 0.12       # 前馈限幅（= 0.30 权限占比 / kd 2.5）
TAU_SCHED_ENABLE = True   # 增益随权限调度
TAU_SCHED_MIN = 0.35
TAU_SCHED_MAX = 1.60

# ---- 倾角上限（随高度）----
#   cap(alt) = clamp(alt/TILT_CAP_SLOPE, TILT_CAP_LO, TILT_CAP_HI)
#   高空允许大倾角换水平权限；低空收紧以保证触地姿态。
TILT_CAP_LO = 5.0          # 地面附近倾角上限 [deg]
TILT_CAP_HI = 35.0         # 高空倾角上限 [deg]
TILT_CAP_SLOPE = 3.0       # alt/3
TILT_TERM_ALT = 90.0       # 终端收倾角的起始高度 [m]
TILT_AZ_MIN_ERR = 1.5      # 误差小于该值 [m] 时保持方位角不再更新

# ---- 水平 eguidance 时间尺度 ----
#   kv = 4/T, kp = 6/T^2  =>  zeta = 4/(2*sqrt(6)) = 0.816 与 T 无关。
T_GAIN_MIN = 8.0           # = CA_T_MIN（相位裕度定）
T_GAIN_MAX = 8.0
TILT_GAIN_CAP_ENABLE = True
TILT_GAIN_MAX = 1.25       # kv 封顶（防深度不稳定）

# ---- 前赡点 ----
#   实测机身相对倾角指令滞后 1.2~2.2 s，取规划前方 t_lead 秒的参考以抵消。
TILT_LEAD_ENABLE = True
TILT_LEAD_SEC = 1.5
TILT_LEAD_MAX_FRAC = 0.30

# ---- 锥角（conic_clamp 的兜底上限；正常路径用 _solver_cone_deg）----
#   参考仓库用固定 25 度锥，因为它的轨迹由同一个锥解出。
#   本项目求解器用 pcs_start=85 -> pcs_end=5 的【时变】锥，控制器
#   解析复现同一条曲线（_solver_cone_deg）。本常量仅作无 plan 时的兜底。
CONIC_TILT_DEG = 25.0

# ---- 内环：a_h 的加加速度限幅（重写核心）----
#   j_max 由【本帧姿态权限 alpha】在线导出，不是固定常数：
#       sec^2(theta) = 1 + (a_h/a_up)^2
#       j = a_up * sec^2(theta) * sqrt(2*alpha*headroom)
#   实测：物理上限 p50=7.10 m/s^3，制导需求 p50=2.20，故 J_MIN 取 2.0。
JERK_GUARD_MIN = 2.0       # 下限：低油门端仍保留基本跟踪能力 [m/s^3]
JERK_GUARD_MAX = 60.0      # 上限：防止 alpha 异常时给过大指令

# 【2026-10-10 P1】内环：只许用掉物理 slew 的一部分
#   【为什么】原判据 j = a_up*sec^2*sqrt(2*alpha*headroom) 只问
#     "机身物理上能不能转"，其等效倾角速率上限约 39~44 deg/s，
#     而实际指令速率 p50 只有 3.7~10.6 deg/s
#     ==> 余量 4~10 倍，【限速器从不生效】
#     （实测 j_used/j_max p50 仅 0.20、ah_cap_bind=0%）。
#   而指令速率翻倍（新两趟 10.6 vs 旧 3.7~5.5 deg/s）后，
#     姿态滞后升到 16~20 deg、偏航饱和 27~33%
#     ==> 需要一个【有约束力】的上限。
#   【改法】不另立模型，直接把物理上限乘比例系数：
#       j_max = min(j_phys, JERK_SLEW_FRAC * j_phys)
#     等效倾角速率上限降到 FRAC * sqrt(2*alpha*headroom)。
#   【为什么用比例而非绝对 deg/s】物理上限正比 sqrt(alpha)，而 alpha
#     随油门变化 8 倍；比例收紧能自动跟随权限，低油门端不会过紧。
#   【取值 0.50】实测：等效上限约 20 deg/s、只限 11% 的帧、
#     限后 p90 由 23.5 降到 18.8 deg/s ==> 只削峰，不压死正常跟踪。
JERK_SLEW_FRAC = 0.50      # 允许用掉物理 slew 的比例（1.0 = 原行为）

TILT_RATE_MAX_DEG = 8.0    # 倾角速率硬上限 [deg/s]（final 兜底，防爆）

# ---- 竖直通道 ----
K_POS = 0.5                # 位置增益（= 参考 params.txt 的 k_x）
K_VEL = 0.8                # 速度增益（= 参考 params.txt 的 k_v）
AVER_TGT_VZ = 2.0          # 目标触地垂速 [m/s]（a_ver_need 用）
AVER_H_FLOOR = 0.5         # a_ver_need 的分母下限 [m]
AVER_FLOOR = 5.0           # 竖直正向下限 [m/s^2]（必须 < g0）
AVER_FLOOR_ENABLE = True
AVER_CLIMB_K = 1.0         # 上升保护的比例增益
AVER_CLIMB_MARGIN = 0.5

# ---- 油门下限（gfold 段）----
#   下限 = min(GFOLD_THROTTLE_MIN, GFOLD_THROTTLE_MARGIN * 悬停油门)。
#   硬约束：下限必须 < 悬停油门，否则下限自己把火箭往上推。
GFOLD_THROTTLE_MARGIN = 0.96
GFOLD_THROTTLE_MIN = 0.16
GFOLD_THROTTLE_RELEASE = 0.05   # 规划要下降时放开到的非零小值

# ---- hold 段（等待首解期间的反向减速）----
HOLD_UP_FRAC = 0.4         # 竖直推力分量上限 = g0 + 0.4*(a_cap - g0)
HOLD_TAU = 1.0             # 净竖直减速 <= |vz| / HOLD_TAU（防反冲）

# ---- 索引推进 ----
IDX_VEL_WEIGHT = 0.2       # find_nearest_index 里速度项权重
IDX_MONOTONIC = True       # 索引单调不可回退

# ---- 其它 ----
TAKEOVER_SAMPLES = 40
REPLAN_DT = 1.0
MAX_TILT_CMD_DEG = 30.0    # 旧律兜底路径的姿态硬限幅（重写后仅 apply 兜底用）
DEBUG_LINES = False
LOG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))),
    'log-B1040-9-gfold')

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

def limit_tilt_dir(vec, max_tilt_deg):
    """把【方向向量】vec 的倾角限到 max_tilt_deg，保持水平方位角与模长。

    【2026-09-29 第六步 6(b)】给姿态指令加硬限幅，防止机头翻到地平线
    以下（实飞 185950 实测 142.7 deg）。详见 MAX_TILT_CMD_DEG 处说明。

    【为什么不能只限第一个分量】vec[0] 与 vec[1:3] 的模必须一起改，
    否则水平方位角会被拧曲。按三角关系反解：
        保持 |vec| 与方位角 => vec[0] = |vec|*cos(t)，水平模 = |vec|*sin(t)

    【边界】|vec| ~ 0 时返回原值（无方向可言）；水平分量为 0 时也无需
    限幅（已经是竖直方向）。
    """
    v = np.asarray(vec, dtype=float)
    n = float(np.linalg.norm(v))
    if not (n > 1e-9):
        return v
    h = float(np.linalg.norm(v[1:3]))
    if not (h > 1e-12):
        return v                       # 已经是竖直方向，无需限幅
    tilt = math.degrees(math.atan2(h, float(v[0])))
    if tilt <= max_tilt_deg:
        return v
    t = math.radians(max_tilt_deg)
    out = np.array(v, dtype=float)
    out[0] = n * math.cos(t)
    out[1:3] = v[1:3] * (n * math.sin(t) / h)
    return out

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

    def update(self, error, dt, rate=None):
        """rate: 【可选】直接用实测角速度当 D 项输入，跳过对误差做差分。

        【2026-09-29 第七步 7(b) 新增】依据见文件头 7(b) 段：
          实测原 D 输入 (error-error_prev)/dt 与真实机身角速度的 R^2 只有
          0.046（即 95% 不是角速度），|diff| 最大 122.7 deg/s 而真实转速
          只有 12.0 deg/s，导致 D 项饱和 11/82 帧。
          传入实测角速度后饱和降到 0 帧。

        【符号约定】调用方传入的 rate 取【与 (error-error_prev)/dt 同号】的
          值，即 rate = -omega（因为误差的导数 ≈ -角速度）。这样 p/i/d 的
          符号逻辑与原来完全一致，不需要改下面的公式。
        """
        if self.first:
            self.first = False
            self.error_prev = error
        elif self.second:
            self.second = False
            self.diff = (error - self.error_prev) / max(1e-9, dt)

        self.integral += error * dt * self.ki
        self.integral = _clamp(self.integral, self.integral_limit,
                               -self.integral_limit)
        if rate is not None:
            # 【真实角速度路径】不再对误差差分（那会引入病态噪声）。
            #   仍走同一套 sd 平滑，便于将来需要时低通。
            self.diff = _lerp(self.diff, float(rate), 1 - self.sd)
        else:
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
    """写 CSV 到 log-B1040-9-gfold/，列固定，便于离线分析。

    【2026-10-08 重写】删除恒空/恒定列（旧版 149 列里约 22 列无信息），
    并新增三类诊断列：
      · 矢量误差分量 dp_n/dp_e/dve_n/dve_e
        【为什么必须加】旧版只有 tf_perr/tf_verr（模长）与 tf_vdir_cos
        （夹角余弦），无法做矢量分解。所有离线闭环验证因此只能做
        "沿误差方向"的一维归约，而真实 3D 行为（尤其方位角）无法验证。
      · 内环诊断 ah_dem/ah_cmd/ah_sat/j_max/j_used
        用于核对"限加加速度"是否生效、需求是否被能力夹住。
      · 竖直串扰追踪 av_in_up/av_out_up/conic_clamp 的输入输出竖直分量
        【为什么必须加】旧版无法定位"竖直幅值从哪来"，实测
        conic_clamp 输入与输出的竖直分量差值中位 17.27 m/s^2。
    """

    COLS = ['t', 'wall', 'alt', 'dist_hz', 'vz', 'vh', 'vmag', 'mass',
            'thr_cmd', 'thrust_n', 'thr_real',
            'a_cmd_up', 'a_cmd_h', 'a_cmd_mag',
            'tilt_cmd', 'tilt_dir_cmd', 'tilt_act', 'att_err',
            'gnc_phase', 'replan_n', 'solve_ms', 'gfold_status',
            'tf', 'tgo', 'h_min_plan', 'x_err', 'v_err',
            # ---- ① 真实姿态 / 姿态目标（目标系分量）----
            'dir_vx', 'dir_vy', 'dir_vz',
            'tgt_vx', 'tgt_vy', 'tgt_vz',
            # ---- ② 参考轨迹采样点 + 跟踪误差 ----
            'n_i', 'plan_alt', 'plan_vz', 'plan_vh',
            'trk_pos', 'trk_pos_up', 'trk_pos_h', 'trk_vel',
            # ---- ③ 循环时标与姿态权限 ----
            'loop_dt', 'rpc_ms', 'ctrl_ms',
            'tau_p', 'tau_r', 'tau_y', 'I_p', 'I_r', 'I_y',
            'alpha_max', 'avel_mag', 'tilt_rate',
            #   【2026-10-11 加日志】指令/实际的姿态速率，单位 deg/s。
            #   与 tilt_rate 的区别：这里是一阶差分且明确用主循环 dt。
            #   【判读】两者之间的滞后 = 姿态相位滞后
            #          （决定是该给姿态环加阻尼，还是该降低指令速率）
            'tilt_cmd_rate', 'tilt_act_rate',
            # ---- ④ PID 内部量 ----
            'pid_err_x', 'pid_err_z',
            #   【2026-10-11 加日志】俯仰 P/D/I 与【偏航 P/D/I】。
            #   此前只有俯仰(_x)三项；而 satZ 达 20~33%、satX 恒 0
            #   ==> 饱和全在偏航，却看不到它的 P/D 构成。
            #   【判读】pid_pz 大而 pid_dz 小 -> D 项没在阻尼（查角速度馈入）
            #          pid_pz 与 pid_err_z 成比例 -> 纯 P 主导，误差本身大
            'pid_px', 'pid_dx', 'pid_ix',
            'pid_pz', 'pid_dz', 'pid_iz',
            'pid_out_x', 'pid_out_z', 'pid_out_r',
            'avel_p', 'avel_r', 'avel_y',
            'sched', 'rate_x', 'rate_y', 'ff_x', 'ff_y',
            'ref_slew', 'ref_clip',
            # ---- ⑤ 参考索引 ----
            'idx_raw', 'idx_frozen',
            # ---- ⑥ 水平误差【方向】（幅值之外，回答"往哪边偏"）----
            'he_n', 'he_e', 'hd_n', 'hd_e', 'h_misalign',
            # ---- ⑦ 【新增】矢量误差分量（消掉一维归约的盲区）----
            #   dp_*  = (x_ref - error) 水平分量，北/东分量，单位 m
            #   dve_* = (v_ref - vel) 水平分量，北/东分量，单位 m/s
            #   【判读】与 tf_perr/tf_verr 应满足 tf_perr == hypot(dp_n,dp_e)
            #   【维护须知】本列表的解析用非贪婪正则取列，因此列表内
            #     不得出现右方括号字符，否则列表会被提前截断。
            'dp_n', 'dp_e', 'dve_n', 'dve_e',
            # ---- ⑧ 姿态失守诊断 ----
            'atq_thr', 'atq_floor', 't_scale',
            'pid_sat_x', 'pid_sat_z', 'pid_sat_run',
            'nose_n', 'nose_e', 'nose_az', 'tgt_az', 'az_err',
            # ---- ⑨ 规划质量 ----
            'plan_tilt_j', 'plan_tf_used', 'plan_nj', 'vdesc',
            # ---- ⑩ 水平指令分项（与速度方向的夹角余弦）----
            'u_h', 'pdv_h', 'pdp_h', 'u_dot', 'pdv_dot', 'pdp_dot',
            'cmd_dot',
            # ---- ⑪ 倾角第一性律 ----
            'tf_plan', 'tf_corr', 'tf_want', 'tf_cmd',
            'tf_perr', 'tf_verr', 'tf_az_n', 'tf_az_e',
            'tf_taper', 'tf_arrived',
            #   【注意】旧版的 tf_tgo/tf_tcorr_tgo 已删：它们由 _tgo 计算块
            #   产出，而该块只喂诊断、已在重写中移除。剩余时间用 run() 里
            #   已有的 tgo 列（= tf*(1 - n_i/(N-1))）即可，两列重复无益。
            'tf_cap', 'tf_slew_allow', 'tf_lead_n',
            'tf_ahdir_cos', 'ca_T', 'ca_kv', 'ca_kp', 'ca_pc',
            # ---- ⑫ 【新增】内环（a_h 加加速度限幅）诊断 ----
            #   ah_dem  = 外环需求，限幅前的水平加速度幅值，单位 m/s^2
            #   ah_cmd  = 内环输出，限加加速度后的水平加速度幅值
            #   ah_sat  = 1 表示需求超过能力 av_up*tan(cap)，被夹住
            #   j_max   = 本帧允许的加加速度上限，由 alpha 在线导出
            #   j_used  = 本帧实际使用的加加速度
            #   【判读】ah_sat=1 占比高 -> 水平需求超能力，应查 cap/油门
            #          j_used 长期贴 j_max -> 内环在限速，跟踪必然滞后
            'ah_dem', 'ah_cmd', 'ah_sat', 'j_max', 'j_used', 'ah_cap_bind',
            #   【2026-10-10 P1 新增】
            #   j_phys = 物理可达上限（原判据），单位 m/s^3
            #   j_lim  = 物理上限 * JERK_SLEW_FRAC（新判据）
            #   【判读】j_max 现在 = min(j_phys, j_lim)；
            #         若 j_max 等于 j_lim 说明新判据在起作用。
            #   【维护须知】本列表内不得出现右方括号字符，否则解析会被截断。
            'j_phys', 'j_lim',
            # ---- ⑬ 竖直通道 ----
            'av_plan', 'av_need', 'av_up', 'av_net',
            # ---- ⑭ 【新增】竖直串扰追踪：conic_clamp 的输入/输出 ----
            #   av_in_up  = target_a 的竖直分量，进入 conic_clamp 之前
            #   av_out_up = 夹限后的竖直分量
            #   【判读】av_up 与 av_out_up 的差异即"幅值串扰"的量化
            'av_in_up', 'av_out_up', 'av_floor', 'min_mag_eff',
            # ---- ⑮ 绘图 ----
            'draw_on',
            # ---- ⑯ 【2026-10-09 P3】FIN 触地保持段 ----
            #   fin_on = 1 表示本帧处于 FIN 触地保持段（姿态瞄准天顶 +
            #            油门 PID 压 vz 到 2 m/s + 主动消横速）
            #   fin_a_up  = FIN 律算出的竖直推力指令，单位 m/s^2
            #   fin_need  = 该高度下"刹到目标垂速"所需的上界，单位 m/s^2
            #   fin_ah    = FIN 律算出的水平推力幅值，单位 m/s^2
            #   【判读】fin_on=1 期间 fin_a_up 应贴着 fin_need（满刹车），
            #          且 vh 应单调下降（这是旧实现缺的那一环）
            #   【维护须知】本列表的解析用非贪婪正则取列，列表内
            #     不得出现右方括号字符（上面已按此规则改写）。
            'fin_on', 'fin_a_up', 'fin_need', 'fin_ah',
            #   【2026-10-11 加日志】FIN 段的【实时】水平误差。
            #   此前 FIN 段 dp_*/dve_* 等 68 列全部冻结（不走 track()），
            #   而实测 FIN 既能纠正 11.6 m 也能恶化 13.1 m，查不出原因。
            #   fin_dp_*  = 到瞄准点的水平位置误差
            #   fin_dve_* = 到「水平速度归零」的误差（= -vel 水平分量）
            #   【判读】fin_dve 应单调收敛到 0；若 fin_dp 反向增大，
            #          说明 FIN 的消横速律把载具推过了头。
            'fin_dp_n', 'fin_dp_e', 'fin_dve_n', 'fin_dve_e',
            #   【2026-10-11】落地后是否成功把姿态交给 SAS（1=成功）
            'post_sas']

    def __init__(self, enabled=True, name=None):
        self.enabled = enabled
        self.rows = []
        self.notes = []
        os.makedirs(LOG_DIR, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(LOG_DIR, name or f'gfold_log_{stamp}.csv')

    def note(self, msg, echo=True):
        """记录一条注释行。

        默认【既写日志又打控制台】；批量/周期性注释传 echo=False 只落盘，
        避免把终端刷满、淹没真正的告警。
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

        # 【未绑定是明确状态】bind() 之前可能被访问（日志初始化/异常路径），
        #   全部显式置 None，避免空引用崩溃。
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
        self.dbg_dir = None          # 机头真实方向（surface 系，单位向量）
        self.dbg_tgt = None          # 目标方向（surface 系，单位向量）
        self.dbg_aim_err = float('nan')

        # ---- 水平误差方向（目标系 [1]=北, [2]=东）----
        self.dbg_he_n = float('nan')
        self.dbg_he_e = float('nan')
        self.dbg_hd_n = float('nan')
        self.dbg_hd_e = float('nan')
        self.dbg_h_misalign = float('nan')
        # ---- 【重写新增】矢量误差分量：消掉"只有模长"的诊断盲区 ----
        self.dbg_dp_n = float('nan')
        self.dbg_dp_e = float('nan')
        self.dbg_dve_n = float('nan')
        self.dbg_dve_e = float('nan')
        # ---- 真实执行器 ----
        self.dbg_thrust_n = float('nan')
        self.dbg_thr_real = float('nan')
        # ---- 姿态失守诊断 ----
        self.dbg_atq_thr = float('nan')
        self.dbg_atq_floor = float('nan')
        self.dbg_pid_sat_x = float('nan')
        self.dbg_pid_sat_z = float('nan')
        self.dbg_pid_sat_run = 0.0
        self.dbg_nose_n = float('nan')
        self.dbg_nose_e = float('nan')
        self.dbg_nose_az = float('nan')
        self.dbg_tgt_az = float('nan')
        self.dbg_az_err = float('nan')
        self._sat_run = 0
        # gfold 段本帧的油门下限（比例）
        self.thr_floor_last = float('nan')
        # 推力自校准系数（见 _thrust_scale）
        self._tscale = 1.0
        self._tscale_buf = []          # 最近的有效比值样本（取中位数）
        self._tscale_prev_cmd = float('nan')
        # 参考轨迹采样点与跟踪误差
        self.dbg_n_i = float('nan')
        self.dbg_plan_alt = float('nan')
        self.dbg_plan_vz = float('nan')
        self.dbg_plan_vh = float('nan')
        # 规划质量诊断
        self.dbg_plan_pk_tilt = float('nan')
        self.dbg_plan_tilt_j = float('nan')
        self.dbg_plan_x0_up = float('nan')
        self.dbg_plan_nj = float('nan')
        self.dbg_plan_tf_used = float('nan')
        self.dbg_vdesc = float('nan')
        # ---- 倾角第一性律的状态与诊断 ----
        #   tf_tilt_prev 已【删除】：重写后状态量是 a_h（见 tf_ah_prev），
        #   倾角由 a_h 反解，不再是独立状态。
        self.tf_az_dir = None        # 方位角状态（病态时保持）
        #   【2026-10-10 P1 新增诊断】
        #   j_phys : 物理可达上限（原判据），单位 m/s^3
        #   j_lim  : 物理上限 * JERK_SLEW_FRAC，单位 m/s^3
        self.dbg_j_phys = float('nan')
        self.dbg_j_lim = float('nan')
        self.dbg_tf_tilt_plan = float('nan')
        self.dbg_tf_tilt_corr = float('nan')
        self.dbg_tf_tilt_want = float('nan')
        self.dbg_tf_tilt_cmd = float('nan')
        self.dbg_tf_perr = float('nan')
        self.dbg_tf_verr = float('nan')
        self.dbg_tf_az_n = float('nan')
        self.dbg_tf_az_e = float('nan')
        self.dbg_tf_tgo = float('nan')
        self.dbg_tf_arrived = float('nan')
        self.dbg_tf_taper = float('nan')
        self.dbg_tf_cap = float('nan')
        self.tf_ah_dir = None
        self.dbg_tf_ahdir_cos = float('nan')
        self.dbg_tf_slew_allow = float('nan')
        self.dbg_tf_lead_n = float('nan')
        self.dbg_ca_T = float('nan')
        self.dbg_ca_kv = float('nan')
        self.dbg_ca_kp = float('nan')
        self.dbg_ca_posclip = float('nan')
        self.dbg_draw_on = float('nan')
        # ---- 【2026-10-09 P3】FIN 触地保持段诊断 ----
        #   fin_on   : 本帧是否处于 FIN 段（由 nav_mode=='fin' 导出）
        #   fin_a_up : FIN 律给出的竖直推力指令，单位 m/s^2
        #   fin_need : 该高度下"刹到目标垂速"的上界，单位 m/s^2
        #   fin_ah   : FIN 律给出的水平推力幅值，单位 m/s^2
        self.dbg_fin_a_up = float('nan')
        self.dbg_fin_need = float('nan')
        self.dbg_fin_ah = float('nan')
        # ---- 【重写新增】内环（加加速度限幅）诊断 ----
        #   ah_prev : 内环状态 = 水平加速度指令幅值 [m/s^2]
        #   ah_dem  : 外环需求（限幅前）[m/s^2]
        #   ah_sat  : 1=需求超过 av_up*tan(cap) 被能力夹住
        #   j_max   : 本帧允许的加加速度上限 [m/s^3]（由 alpha 在线导出）
        #   j_used  : 本帧实际使用的加加速度 [m/s^3]
        self.tf_ah_prev = 0.0
        self.dbg_ah_dem = float('nan')
        self.dbg_ah_cmd = float('nan')
        self.dbg_ah_sat = float('nan')
        self.dbg_ah_cap_bind = float('nan')
        self.dbg_j_max = float('nan')
        self.dbg_j_used = float('nan')
        # ---- 竖直通道诊断 ----
        #   av_plan : 规划自己的竖直推力分量 [m/s^2]
        #   av_need : 物理需求上界 g0 + (vz^2-V^2)/(2h) [m/s^2]
        #   av_up   : 最终竖直指令 [m/s^2]
        #   av_net  : av_up - g0（>0 表示会上升）
        #   av_in_up / av_out_up : conic_clamp 的【输入/输出】竖直分量
        #     【为什么必须记】此前无法定位"竖直幅值从哪来"，
        #     实测 av_up != conic_clamp 输入竖直分量，差值中位 17.27。
        self.dbg_av_plan = float('nan')
        self.dbg_av_need = float('nan')
        self.dbg_av_up = float('nan')
        self.dbg_av_net = float('nan')
        self.dbg_av_in_up = float('nan')
        self.dbg_av_out_up = float('nan')
        self.dbg_av_floor = float('nan')
        self.dbg_min_mag_eff = float('nan')
        # ---- 水平指令分项（与速度方向的夹角余弦）----
        self.dbg_u_h = float('nan')
        self.dbg_pdv_h = float('nan')
        self.dbg_pdp_h = float('nan')
        self.dbg_u_dot = float('nan')
        self.dbg_pdv_dot = float('nan')
        self.dbg_pdp_dot = float('nan')
        self.dbg_cmd_dot = float('nan')
        self.dbg_trk_pos = float('nan')
        self.dbg_trk_pos_up = float('nan')
        self.dbg_trk_pos_h = float('nan')
        self.dbg_trk_vel = float('nan')
        # ---- 姿态权限与循环时标 ----
        self.dbg_loop_dt = float('nan')
        self.dbg_rpc_ms = float('nan')
        #   【注意】run() 是【原样沿用】的旧代码，它写的属性名是
        #   dbg_seg_ctrl（不是 dbg_ctrl_ms）。日志列 ctrl_ms 由 _dbg_cols
        #   从 dbg_seg_ctrl 读取 —— 曾因两处命名不一致导致该列【恒空】。
        self.dbg_seg_ctrl = float('nan')
        self.dbg_tau_p = float('nan')
        self.dbg_tau_r = float('nan')
        self.dbg_tau_y = float('nan')
        self.dbg_I_p = float('nan')
        self.dbg_I_r = float('nan')
        self.dbg_I_y = float('nan')
        self.dbg_alpha_max = float('nan')
        self.dbg_avel_mag = float('nan')
        self.dbg_tilt_rate = float('nan')
        # ---- PID 内部量 ----
        self.dbg_pid_err_x = float('nan')
        self.dbg_pid_err_z = float('nan')
        self.dbg_pid_px = float('nan')
        self.dbg_pid_dx = float('nan')
        #   【2026-10-11 加日志】偏航三项 + 姿态速率 + FIN 实时误差 + SAS 标志
        self.dbg_pid_pz = float('nan')
        self.dbg_pid_dz = float('nan')
        self.dbg_pid_iz = float('nan')
        self.dbg_tilt_cmd_rate = float('nan')
        self.dbg_tilt_act_rate = float('nan')
        self._prev_tilt_cmd_log = None
        self._prev_tilt_act_log = None
        self.dbg_fin_dp_n = float('nan')
        self.dbg_fin_dp_e = float('nan')
        self.dbg_fin_dve_n = float('nan')
        self.dbg_fin_dve_e = float('nan')
        self.dbg_post_sas = 0.0
        self.dbg_pid_out_x = float('nan')
        self.dbg_pid_out_z = float('nan')
        self.dbg_pid_out_r = float('nan')
        self.dbg_avel_local = None
        # 增益调度 + D 项真实角速度 + 前馈
        self.dbg_sched = float('nan')
        self.dbg_rate_x = float('nan')
        self.dbg_rate_y = float('nan')
        self._ff_dir_prev = None
        self.dbg_ff_x = float('nan')
        self.dbg_ff_y = float('nan')
        # 参考斜率限制的状态
        self._ref_slew_prev = None
        self.dbg_ref_slew = float('nan')
        self.dbg_ref_clamped = 0.0
        self.dbg_pid_ix = float('nan')
        self._prev_tilt_act = float('nan')
        self._att_warned = False
        self.last_pkt = 0.02      # 上一帧游戏时间（供 track 的索引推进用）
        self.idx_frozen_n = 0
        self.est_time = 0.5
        # ---- debug-lines 开关（两个开关各自独立生效，后者优先）----
        if getattr(args, 'debug_lines', False):
            self.debug_lines = True
        elif getattr(args, 'no_debug_lines', False):
            self.debug_lines = False
        else:
            self.debug_lines = bool(DEBUG_LINES)
        self.drawn = []
        self._lines_N = None
        self._lines_fr = None
        self._draw_warned = False
        self.prev_vel = None
        self._solve_thread = None
        self._pending_plan = None
        self._pending_msg = None
        self.gfold_n_i = -100     # 参考仓库的路径索引 n_i（-100 = 未定位）
        self.gfold_traj_ready = False
        self.target_direction = None
        self.tilt_cmd = 0.0
        self.tilt_dir_cmd = 0.0
        # 状态机：none / hold / gfold（本项目已删掉参考仓库的 final 段）
        self.nav_mode = 'none'

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
        # 【2026-09-28】用 available_thrust（含推力限幅 + 当前大气）而非 max_thrust。
        #   理由同 a_cap_real()：本载具一级限幅 150%，
        #   max_thrust 会把 a_net 低估 50%，
        #   而候选筛选的阈值（15~80）与【取最接近 50】是按真实值标定的。
        _th_exc = None
        try:
            th = float(v.available_thrust)
        except Exception:                            # noqa: BLE001
            th = float(v.max_thrust)
            _th_exc = True
        _ = _th_exc
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
        #
        # 【2026-09-28 修正：时间基准必须用【游戏时间】，不能用墙钟】
        #   旧实现用 time.time()（墙钟）作 dtp，而 dp 是【游戏内位移】。
        #   两者不同基准：KSP 掉帧/求解阻塞时墙钟远快于游戏时间，
        #   于是 dtp 偏大 ⇒ vh_meas 被【低估】⇒ (vh - vh_meas) 虚高。
        #   实测 log ..._154409 在门限段（alt~35 km、v~770 m/s）误报：
        #       vh=240.4  vh_meas=69.8  v_rot=174.9
        #   而同一趟飞行在 gfold 段的 vh 只有 18~19 m/s（远低于自转
        #   174.9），vmag 与 sqrt(vz^2+vh^2) 逐帧吻合、dist_hz 平滑单调
        #   下降 —— 说明【速度口径本来就是干净的】，是自检本身误报。
        #   改用 self.sc.ut（游戏时间）后，dp 与 dtp 同基准。
        now = float(self.sc.ut)
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
        a_cap = self.a_cap_real(mass)
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
        a_net = self.a_cap_real(mass) - 9.80665
        # 【第十一步 11(a)】记录求解器【实际收到的】初状态高度（= h，已减
        #   TARGET_ALT）。它与 track() 里第一帧的 h_min_plan 对比，即可判定
        #   "规划起点与载具是否对齐" —— 这是查清 plan_alt 偏移之谜的钥匙。
        #   同时记录本次求解用的 tf（求解器内部探测后的值）。
        try:
            self.dbg_plan_x0_up = float(x0_state[0])
        except Exception:                            # noqa: BLE001
            self.dbg_plan_x0_up = float('nan')
        alt = float(x0_state[0]) + TARGET_ALT
        dist = math.hypot(float(x0_state[1]), float(x0_state[2]))
        vz = float(x0_state[3])
        vh = math.hypot(float(x0_state[4]), float(x0_state[5]))
        if a_net < 1.0:
            print(f'[G-FOLD] 预检失败：a_net = {a_net:.2f} m/s²（推重比过低）')
            print(f'          mass={mass/1000:.1f} t  available_thrust='
                  f'{self.v.available_thrust/1e6:.2f} MN '
                  f'(max_thrust={self.v.max_thrust/1e6:.2f} MN)')
            print('          这艘船物理上无法着陆 —— 多半是【选错了 vessel】')
            print(f'          当前 vessel={self.v.name!r}；'
                  '用 --vessel-name 指定回收一级')
            self.gfold_status = 'precheck: a_net too low'
            return None
        t0 = time.time()
        # 【求解参数全部来自文件顶部常量区】逐项对应参考仓库 params.txt。
        #   throttle=(0.1, 0.8) 是【求解器】的节流界（参考同值）；
        #   控制器侧用的是 [0.05, 1.0]，两套并存是原仓库设计。
        # 【第二步】t_max 必须用【真实可用推力】（同 a_cap_real 的口径）：
        #   kRPC 在本构建下把 available_thrust 低估 1.414 倍（见 a_cap_real
        #   的长注释）。求解器若按 8.99 MN 规划，会得到一条"需要 86.7 m/s²
        #   才能跟"的轨迹，而载具只有 49 —— 表现为跟不上、掉高度。
        #   下降率包络的 cap 里含 a_brk = t_max/m - g0，口径错则整条包络错。
        _t_avail = max(1.0, float(self.v.available_thrust)) * self._thrust_scale()
        # ================================================================
        # 【2026-10-08 改动 2】末段抬高 TERM_CLEARANCE：给"最后一段减速"留余量
        # ----------------------------------------------------------------
        # 【问题】规划的"速度归零点"在 x[0]=0 <=> alt=TARGET_ALT，而停机判据
        #   是 alt <= TARGET_ALT+1 —— 两者重合，载具在规划刹停前就触地，
        #   落地垂速 -1.6 ~ -10.8 m/s（实测六趟）。
        # 【做法】把求解器的高度基准抬高 TERM_CLEARANCE：
        #   传入前 h -= CLEAR；取回后 plan['x'][0,:] += CLEAR。
        #   这样规划末节点落在 alt = TARGET_ALT + CLEAR，
        #   而【跟踪用的坐标系不变】（track() 仍相对 TARGET_ALT 算误差）。
        # 【配套】最后 CLEAR 米由末段下降律接管（见 track() 的终端分支），
        #   否则规划要求 v=0，载具会在抬高点悬停。
        # ================================================================
        _x0_solver = np.array(x0_state, dtype=float)
        if TERM_CLEARANCE > 0.0:
            _x0_solver[0] = float(_x0_solver[0]) - float(TERM_CLEARANCE)
        r = solve_p3p4_state(
            _x0_solver, mass,
            isp=ISP_DEFAULT,
            t_max=_t_avail,
            throttle=(0.1, 0.8),
            tf_guess=TF_GUESS,
            straight_fac=STRAIGHT_FAC,
            gs_deg=GS_DEG,
            pcs_deg=PCS_END_DEG,
            pcs_start_deg=PCS_START_DEG,
            v_max=V_MAX_SOLVER,
            N3=N3, N4=N4, target_alt=TARGET_ALT,
            # 【第二步】下降率包络（见文件头 V_DESCENT_MAX_ENABLE 的长注释）
            v_descent_max=V_DESCENT_MAX_ENABLE,
            term_win=V_DESCENT_TERM_WIN,
            term_vz=V_DESCENT_TERM_VZ)
        self.solve_ms = (time.time() - t0) * 1000.0
        self.gfold_status = r.get('status', 'err')
        # 【改动 2】把规划的高度基准平移回来（+TERM_CLEARANCE）。
        #   求解器是在"抬高 CLEAR"的基准上解的，其 x[0,:] 全部少了 CLEAR；
        #   加回来之后，plan['x'][0,:] 仍表示【相对 TARGET_ALT 的高度】，
        #   于是 track() / 误差计算 / conic_clamp 全部无需改动。
        #   末节点因此停在 alt = TARGET_ALT + CLEAR，而不是 TARGET_ALT。
        if TERM_CLEARANCE > 0.0 and r.get('x') is not None:
            try:
                r['x'] = np.asarray(r['x'], float).copy()
                r['x'][0, :] = r['x'][0, :] + float(TERM_CLEARANCE)
            except Exception:                        # noqa: BLE001
                pass
        # 【第十一步 11(a)】记录求解器【返回的】tf（探测后的实际值）。
        #   与日志 tf 列、以及 x0_up 一起，可完整重建"求解器看到了什么、
        #   选了哪条规划"。这是判定退化规划是否被采纳的直接证据。
        try:
            self.dbg_plan_tf_used = float(r.get('tf', float('nan')))
        except Exception:                            # noqa: BLE001
            self.dbg_plan_tf_used = float('nan')
        # 【第二步】记录本次规划用的下降率包络上限（供 vdesc 日志列）
        self.dbg_vdesc = float(V_DESCENT_MAX_ENABLE) if V_DESCENT_MAX_ENABLE \
            else float('nan')
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
        # ============================================================
        # 【2026-09-28 修改①：度量里加回速度项】
        #   上游 demo3_gfold.py:262 本来就有这一项，但被作者自己注释掉了：
        #       mag = npl.norm(x[0:3, i] - r) # + npl.norm(x[3:6, i] - v) * 0.2
        #   纯位置度量下，只要载具越过轨迹的最近点，argmin 就永久粘住
        #   （见文件头 IDX_VEL_WEIGHT 处的完整证据）。
        #   加回速度项后，"位置近但速度方向已明显不符"的节点不再被选中，
        #   argmin 会继续沿轨迹前进。
        # ============================================================
        _w = float(IDX_VEL_WEIGHT)
        nearest_mag = (float(np.linalg.norm(x[0:3, 0] - r))
                       + float(np.linalg.norm(x[3:6, 0] - v)) * _w)
        nearest_i = 0
        for i in range(x.shape[1]):
            mag = (float(np.linalg.norm(x[0:3, i] - r))
                   + float(np.linalg.norm(x[3:6, i] - v)) * _w)
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
        # ============================================================
        # 【第十五步 15(b)】修"到达哨兵导致强制悬停"
        # ------------------------------------------------------------
        # 【原行为（逐行照抄参考仓库）】index >= N-1 时返回
        #     x_i = 0, v_i = 0, u_i = [g0, 0, 0]
        #   即"已到目标，只留重力补偿"。参考仓库用它表示【任务结束】。
        #
        # 【实飞事故】gfold_log_20260930_211419：
        #   n_i 在 267/463 帧（58%）打满 N-1=79，随后
        #     h_min_plan = 0.0000   （x_i 归零，证实走了本分支）
        #     aver_plan  = 9.807    （u_i = [g0,0,0]）
        #     a_up - g0  = -0.087   （净竖直加速度 ≈ 0）
        #   载具在 alt=21.8 m（瞄准点 17 m）【悬停 27.1 秒】，vh 从未消掉。
        #   ⇒ 本分支的语义是"任务完成"，但载具【还没落地】就被判成完成了。
        #
        # 【为什么这次会触发而以前不会】本趟竖直跟踪极好（|dalt| 从不 >20 m），
        #   略微高于规划末节点就把最近点投影推到 N-1 之外。
        #   讽刺的是【跟踪变好反而触发】了这个哨兵。
        #
        # 【修法】把"到达"的判据从【索引打满】改为【索引打满 且 确实已到达】：
        #   只有载具【接近目标高度】且【速度已很小】时才允许进入哨兵；
        #   否则继续使用规划【末端节点】的参考值（让 PD 把剩下的高度走完）。
        #   这样既不破坏参考仓库的到达语义，也不会在半空中被判"已到达"。
        # ============================================================
        if index >= N - 1:
            _i_last = int(max(0, N - 2))
            _x_last = np.asarray(x[:, _i_last], float)[0:3].copy()
            _v_last = np.asarray(x[:, _i_last], float)[3:6].copy()
            _u_last = np.asarray(u[:, _i_last], float).copy()
            # 判据由调用方通过 self 上的标志传入（track 每帧设置）。
            #   _arr_ok = True  -> 真的到了，按参考仓库返回哨兵
            #   _arr_ok = False -> 还没到，返回末端节点值
            _arr_ok = bool(getattr(self, '_sample_arrived_ok', False))
            if _arr_ok:
                return (np.zeros(3), np.zeros(3), np.array([9.807, 0.0, 0.0]))
            return (_x_last, _v_last, _u_last)
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

    def _lead_index(self, n_i, tf, N):
        """【16(h)】把采样索引沿规划向前推 t_lead 秒。

        物理含义：机身滞后指令约 1.5 s，故控制器应取【规划前方 1.5 s】
        处的参考，让指令提前发出。返回夹在合法范围内的索引。

        【限幅理由】
          · 下限 0        —— 不能取到规划起点之前
          · 上限 N-2      —— 与 gfold_n_i 的上限口径一致，避开末端退化节点
          · t_lead <= TILT_LEAD_MAX_FRAC * tf —— 防止 tf 异常时跨度过大
        【失败回退】任何异常都返回原 n_i，保证不比旧行为更差。
        """
        if not TILT_LEAD_ENABLE:
            return n_i
        try:
            _tf = float(tf)
            if not (_tf > 1e-06):
                return n_i
            _lead = min(float(TILT_LEAD_SEC),
                        TILT_LEAD_MAX_FRAC * _tf)
            _dn = _lead * float(N - 1) / _tf        # 秒 -> 节点数
            return max(0.0, min(float(N) - 2.0, float(n_i) + _dn))
        except Exception:                            # noqa: BLE001
            return n_i

    def track(self, r, tp, tv, n_i):
        """G-FOLD 跟随（重写版：单一 tilt-first 律）。

        ====================================================================
        【与旧版的区别（旧版见 gfold_land_old.py 的 track）】
        旧版有两条律并存：旧律（照抄参考仓库的 target_a = u + KV*dv + KP*dp，
        经 conic_clamp）与新律（倾角第一性）。本版【只保留 tilt-first】，
        并把旧律中真正必要的部分【整合】进两个标量通道：

            水平 -> a_h（幅值+方位角来自同一个矢量）
            竖直 -> a_up（真正的竖直 PD，不再是旧律向量的投影）

        【为什么要整合竖直】旧版的竖直幅值是
            _amag = |target_a|                     <- 旧律 PD 向量（含水平修正）
            a_up  = _amag * cos(tilt_cmd)          <- 投影到新律倾角
        于是【水平 PD 修正的幅值被当成竖直推力发出去】。实测证据：
            a_cmd_h == a_cmd_mag*sin(tilt_cmd)     偏差 < 0.003（恒等式）
            去掉 conic_clamp 托底后，该竖直值在 58.5% 的帧为负（中位 -8.10）
            => 所谓"竖直通道"其实是一个下限在承重的空壳。
        本版竖直改为【真正的 PD】，并保留原有的三层约束。

        ====================================================================
        【措施2：内环限加加速度】旧版限的是【倾角速率】
              tilt = tilt_prev + clip(tilt_want - tilt_prev, +-slew*dt)
        它经 a_h = a_up*tan(tilt) 映射后，等效加加速度为
              j = a_up * sec^2(tilt) * slew
        而 sec^2 在 1.04~1.49 漂、a_up 在 5.7~22.6 漂 => 等效 j 失控。
        实测：现行限速给出的 j 只有 1.40 m/s^3，而制导需求 2.20
        （59.3% 的帧需求超限）；物理上限 7.10，尚有 3.2 倍余量。
        本版把内环状态量改为【a_h 本身】，直接限 |da_h/dt|。

        【措施3：口径闭环】旧版前向用 a_up_est = max(1,|u[0]|)、
        后向用 av_up，两者不等 => 往返失配（实测 p10=0.518/p90=1.721）。
        本版 tilt 由【同一个 a_up】反解：tilt = atan2(a_h, a_up)。
        => 措施2 与措施3 在同一处改动里自然合一。
        （注：单独做措施3 会让 p90 失真恶化 46%，因为失配因子与限速因子
         负相关；故必须同时做。见 dev/reports/THREE_ITEMS_FEASIBILITY_MATLAB.md）

        ====================================================================
        【保留的飞行安全约束（每条都对应一次记录在案的事故）】
          · 索引几何投影 + 单调 + 夹到 N-2   （防索引冲到末端后悬停）
          · 前赡点 LEAD_SEC                   （抵消 1.2~2.2 s 机身滞后）
          · a_up <= a_ver_need               （防"刹得比需要更狠"导致爬升）
          · a_up >= 正下限（油门下限对应值） （防 36/70 帧零油门自由落体）
          · a_up < 0.99*g0                   （防净向上=>永不落地）
          · 倾角上限 cap=clamp(alt/3,5,35)    （防机头翻过 90 deg）
          · 终端收倾角但保留水平权限下限      （防低空 vh 消不掉）
          · 方位角病态时保持上一有效值        （防 atan2 近零矢量抖动）
          · conic_clamp 作最终安全限幅        （竖直保底 + 锥限幅）
        """
        x = r['x']
        u = r['u']
        N = x.shape[1]
        tf = r['tf']
        error = np.array(tp)
        vel = np.array(tv)

        # ---------------- ① 索引推进（几何投影 + 单调 + 夹 N-2）----------------
        dt_game = getattr(self, 'last_pkt', 0.02)
        _raw_near = self.find_nearest_index(x, error, vel, tf, N)
        self.gfold_traj_ready = _raw_near > -0.05
        _n_from_decay = n_i - dt_game * 0.2 * N / tf
        self.gfold_n_i = max(_n_from_decay, _raw_near)
        #   索引是"沿轨迹的弧长参数"，物理上时间不倒流；实测推进速率只有
        #   标称的 67%（载具一直落在轨迹后方），允许回退会加剧"追不上"。
        if IDX_MONOTONIC:
            self.gfold_n_i = max(self.gfold_n_i, float(n_i))
        #   【绝不能用时间增量推进索引】曾实现"速率地板"，实飞 3/3 趟全败：
        #   dt_game 不可信（实测 max 20.6 s 而真实帧间隔 ~0.16 s），
        #   索引一帧就冲到末端，制导以为"已到达"。
        self.gfold_n_i = max(0.0, self.gfold_n_i)
        #   夹到 N-2：最后一个节点（N-1）是求解器的退化占位
        #   （h=0, vz=0, u=0），既不可跟踪也不提供推力信息。
        self.gfold_n_i = min(self.gfold_n_i, float(N) - 2.0)
        self.dbg_idx_raw = float(_raw_near)
        if abs(self.gfold_n_i - float(n_i)) < 1e-9:
            self.idx_frozen_n = int(getattr(self, 'idx_frozen_n', 0)) + 1
        else:
            self.idx_frozen_n = 0
        self.dbg_idx_frozen = float(self.idx_frozen_n)
        #   "真的已到达"判据（供 sample_index 的哨兵分支）：
        #   必须高度接近目标【且】两轴速度都已很小，三者缺一不可。
        _arr_alt = float(error[0]) + TARGET_ALT
        _arr_vz = abs(float(np.asarray(vel, float)[0]))
        _arr_vh = float(np.linalg.norm(np.asarray(vel, float)[1:3]))
        _arr_ok = (_arr_alt < (TARGET_ALT + 2.0) and _arr_vz < 1.5
                   and _arr_vh < 1.5)
        self._sample_arrived_ok = bool(_arr_ok)
        self.dbg_tf_arrived = 1.0 if _arr_ok else 0.0

        x_i, v_i, u_i = self.sample_index(x, u, self.gfold_n_i, tf, N)

        # ---------------- ② 前赡点采样 ----------------
        #   实测机身相对倾角指令滞后 1.23~2.21 s（互相关峰值，corr 0.61~0.84）。
        #   取规划【前方】t_lead 秒的参考，让指令提前发出，滞后正好抵消。
        _n_look = self._lead_index(self.gfold_n_i, tf, N)
        if abs(_n_look - self.gfold_n_i) > 1e-09:
            x_r, v_r, u_r = self.sample_index(x, u, _n_look, tf, N)
        else:
            x_r, v_r, u_r = x_i, v_i, u_i
        self.dbg_tf_lead_n = float(_n_look - self.gfold_n_i)

        # ---------------- ③ 水平律：eguidance ----------------
        #   误差定义 dp = x_ref - error, dve = v_ref - vel。
        #   把 u_h 视作前馈，则误差回路的闭环特征为
        #       wn = sqrt(kp),  zeta = kv/(2*sqrt(kp))
        #   取 kv = 4/T, kp = 6/T^2 => zeta = 4/(2*sqrt(6)) = 0.816【与 T 无关】。
        #   T 用【水平自己的时间尺度】T = 2|dp|/|dve|（还差多久到达）。
        _dp = (np.asarray(x_r, float) - np.asarray(error, float))[1:3]
        _dve = (np.asarray(v_r, float) - np.asarray(vel, float))[1:3]
        _perr = float(np.linalg.norm(_dp))
        _verr = float(np.linalg.norm(_dve))
        self.dbg_tf_perr = _perr
        self.dbg_tf_verr = _verr
        #   【新增】矢量误差分量：消掉"只有模长"的诊断盲区。
        self.dbg_dp_n = float(_dp[0])
        self.dbg_dp_e = float(_dp[1])
        self.dbg_dve_n = float(_dve[0])
        self.dbg_dve_e = float(_dve[1])

        _ca_T = 2.0 * _perr / max(0.05, _verr)
        _ca_T = max(T_GAIN_MIN, min(T_GAIN_MAX, _ca_T))
        _ca_kv = 4.0 / _ca_T
        _ca_kp = 6.0 / (_ca_T * _ca_T)
        if TILT_GAIN_CAP_ENABLE:
            _ca_kv = min(TILT_GAIN_MAX, _ca_kv)
        _ah_vec = (np.asarray(u_r, float)[1:3].copy()
                   + _ca_kv * _dve
                   + _ca_kp * _dp)
        _ah_dem = float(np.linalg.norm(_ah_vec))
        self.dbg_ca_T = float(_ca_T)
        self.dbg_ca_kv = float(_ca_kv)
        self.dbg_ca_kp = float(_ca_kp)
        self.dbg_ah_dem = _ah_dem

        #   规划自己的竖直分量，作为倾角需求的分母（也是唯一"规划竖直"出处）。
        #   用【当前节点】的 u_i[0]，与 ⑥ 的竖直律同源，保证口径一致。
        _a_up_est = max(1.0, abs(float(u_i[0])))
        _tcorr = math.degrees(math.atan2(_ah_dem, _a_up_est))
        self.dbg_tf_tilt_corr = float(_tcorr)
        _tilt_plan = math.degrees(math.atan2(
            float(np.linalg.norm(np.asarray(u_i, float)[1:3])),
            max(1e-09, float(u_i[0]))))
        self.dbg_tf_tilt_plan = float(_tilt_plan)

        # ---------------- ④ 倾角上限（随高度）----------------
        #   cap(alt) = clamp(alt/TILT_CAP_SLOPE, TILT_CAP_LO, TILT_CAP_HI)
        #   物理依据：倾角同时决定"竖直制动能力 a_up*cos"与
        #   "水平修正能力 a_up*tan"。高空时间充裕、允许大倾角换水平权限；
        #   低空必须竖直落地，倾角收紧（触地姿态 + 防横漂）。
        _alt_now = float(error[0]) + TARGET_ALT
        _tcap = max(TILT_CAP_LO,
                    min(TILT_CAP_HI, _alt_now / max(1e-06, TILT_CAP_SLOPE)))
        self.dbg_tf_cap = float(_tcap)

        #   终端收倾角：低空收缩【允许的】倾角。
        #   原实现一路收到 0，实测 211419 在 alt=22 m 处 tilt 只剩 5.6 deg、
        #   a_h 仅 0.96 m/s^2，而 vh 还有 13~26 m/s => vh 永远消不掉。
        #   故保留水平权限下限：只要横速还没消掉，就不许把允许倾角压到
        #   TILT_CAP_LO 以下。
        #
        #   【关键】这里收缩的是【能力上限 _tilt_allow】，不是需求 _tcorr。
        #     初版写成 _tilt_allow = min(_tcap, _tilt_want)（_tilt_want 即需求），
        #     于是上限恒等于需求 => ah_sat 每帧都是 1，能力夹限形同虚设，
        #     而且 tilt_cmd 被钉在需求上、内环限速完全失效。
        _vhoriz = _verr
        if _alt_now < TILT_TERM_ALT:
            _taper = max(0.0, _alt_now / TILT_TERM_ALT)
            self.dbg_tf_taper = float(_taper)
            if _vhoriz <= 1.0:
                _tilt_allow = _tcap * _taper
            else:
                _tilt_allow = max(_tcap * _taper, min(_tcap, TILT_CAP_LO))
        else:
            self.dbg_tf_taper = 1.0
            _tilt_allow = _tcap
        #   允许倾角不得低于能力的硬下限（5 deg），否则 a_h 恒为 0、
        #   横速再也修不回来。
        _tilt_allow = max(min(TILT_CAP_LO, _tcap),
                          min(_tcap, _tilt_allow))
        self.dbg_tf_tilt_want = float(_tcorr)   # 需求（诊断用）

        # ---------------- ⑤ 方位角：与幅值同源 ----------------
        #   必须取【合矢量的方向】，否则又回到"大小来自一个向量、
        #   方向来自另一个向量"的老问题（实测那种做法有 49% 的帧方向是错的）。
        #   病态保护：|误差| 很小时方向无意义（实测倾角 0~10 deg 时方位角
        #   速率高达 137 deg/s），此时保持上一有效方位角。
        _az = self.tf_az_dir
        if _ah_dem > 1e-09 and _perr > TILT_AZ_MIN_ERR:
            _az = (_ah_vec / _ah_dem).copy()
            self.tf_az_dir = _az
        #   ============================================================
        #   【2026-10-10 P2 评估结论：不实施方位角限速】
        #   ------------------------------------------------------------
        #   曾打算对【标量方位角速率】限速（意图：az p90 达 103~116 deg/s
        #   而可达 slew 约 40 deg/s）。实施后用真日志复算，发现该设计
        #   【有害】，故回退。证据如下：
        #
        #   ① 方位角的三维代价是 sin(tilt)*Δazimuth，不是 Δazimuth。
        #      实测所有 60~175 deg 的方位角突变【都发生在 tilt 1~9 deg】，
        #      对应的三维姿态参考 tgt_v* 只动 1.8~8.9 deg
        #      ==> 那些突变在姿态上【本来无害】（小倾角时是绕竖直轴转）。
        #   ② 三维需求分解（p90）：倾角项 11~26 deg/s，
        #      方位项(sin 加权后) 只有 1.5~9.6 deg/s，占 9~28%。
        #      ==> 方位角不是姿态需求的主体。
        #   ③ 若仍按标量限速，复算显示会限住 33~52% 的帧，
        #      而这些帧的"限速后方位角 vs 请求方位角"偏差中位达
        #      144~176 deg ==> 水平推力方向被指错，
        #      横向损失 a_h*(1-cos) 达 0.5~6.0 m/s^2（与总需求同量级）。
        #      ==> 它会【加剧】水平振荡，与"方位角冻结"(TILT_AZ_MIN_ERR)
        #         是同一个病。
        #
        #   ④ 正确做法：要限就限【三维姿态参考的角速率】，而该机制
        #      【已经存在】= _limit_ref_slew(target_direction)（见 ⑫），
        #      它按 sqrt(2*alpha*headroom) 限制球面转动速率、物理判据正确。
        #      故方位角无需另加限速；只需保留既有机制并观察 ref_clip 列。
        #
        #   【若将来确需限制】必须用 sin(tilt) 加权：
        #       Δ(三维) ≈ sin(tilt) * Δazimuth <= slew_allow * dt
        #     即  Δazimuth_max = slew_allow*dt / max(sin(tilt), eps)
        #     而不能直接限 Δazimuth。本文件【不实施】以免再引入有害行为。
        #   ============================================================
        if _az is None:
            #   尚无有效方位角 -> 退回"指向瞄准点"方向（-error 的水平分量）。
            _az = np.array([0.0, 0.0])
            _eh = np.asarray(error, float)[1:3]
            _ehn = float(np.linalg.norm(_eh))
            if _ehn > 1e-09:
                _az = -_eh / _ehn
            self.tf_az_dir = _az
        _azn = float(np.linalg.norm(_az))
        if _azn > 1e-09:
            _az = _az / _azn
        else:
            _az = np.array([0.0, 0.0])
            self.tf_az_dir = _az
        self.dbg_tf_az_n = float(_az[0])
        self.dbg_tf_az_e = float(_az[1])
        self.tf_ah_dir = _ah_vec / _ah_dem if _ah_dem > 1e-09 else None
        #   合矢量方向 与 位置误差方向 的夹角余弦（应恒 >0）
        try:
            if self.tf_ah_dir is not None and _perr > 1e-06:
                self.dbg_tf_ahdir_cos = float(
                    np.dot(np.asarray(self.tf_ah_dir, float), _dp) / _perr)
            else:
                self.dbg_tf_ahdir_cos = float('nan')
        except Exception:                            # noqa: BLE001
            self.dbg_tf_ahdir_cos = float('nan')

        # ---------------- ⑥ 竖直律：真正的竖直 PD ----------------
        #   【重写要点】旧版这里是 a_up = |target_a|*cos(tilt)，幅值来自旧律
        #   PD 向量；本版改为【规划竖直 + 竖直 PD】，与水平通道完全独立。
        #
        #   【必须用当前节点，不能用前赡点】前赡点的用途是补偿【姿态】滞后
        #   （机身转动有 1.2~2.2 s 惯性），而【油门没有这种滞后】——
        #   推力几乎瞬时跟随。若竖直 PD 也拿前赡点作参考，就等于让推力
        #   去追一个"还没到"的未来状态，产生系统性过刹/欠刹。
        #   实测（本机 3DoF 闭环）：用前赡点时落地垂速 -24.0 m/s，
        #   而规划自身在末节点的垂速只有 -2.6 m/s。
        _h_now = max(0.0, float(error[0]))          # 本机相对目标的高度 [m]
        _vz_now = float(np.asarray(vel, float)[0])
        _p0 = float(u_i[0])
        # ================================================================
        # 【2026-10-08 改动 2 配套】末段下降律
        # ----------------------------------------------------------------
        # 【为什么必须有】改动 2 把规划的归零点抬到 alt=TARGET_ALT+CLEAR，
        #   规划在那一点的速度是 0。若继续纯跟踪规划，载具会在
        #   【抬高点悬停】而不是落地（历史同类故障：211419 在 alt=21.8 m
        #   悬停 27.1 s、vh 从未消掉）。
        # 【做法】h < CLEAR 时不再跟规划的竖直剖面，改为一个【速度环】，
        #   目标是 TERM_DESCENT_VZ（与 AVER_TGT_VZ 同口径，正数=向下）。
        #   a_ver_need 仍在其后作为上界，保证"刹得住"。
        # 【水平】不动 —— 规划要求 vh=0，继续由水平律收敛（这正是想要的）。
        # ================================================================
        _term_phase = (TERM_CLEARANCE > 0.0 and _h_now < TERM_CLEARANCE)
        if _term_phase:
            #   【必须用【满刹车】而不是一个弱 P 环】
            #     初版写 _a_up_raw = G0 + K_VEL*(vz_tgt - vz_now)，实测进入
            #     末段时 vz 已到 -10.4，该式给出 16.5，而物理需要 20.3
            #     => 落地 vz 仍达 -12.2（比不用末段律还差）。
            #     末段没有"留余量"的必要 —— 目标就是尽快把 vz 收到
            #     TERM_DESCENT_VZ。故直接命令 a_ver_need（在剩余高度内
            #     刚好把 vz 刹到目标值所需的值），即【满刹车】。
            #     置成 +inf 再由下面的 min(_a_up_raw, _a_ver_need) 夹住，
            #     语义最清晰：末段 = 以物理上限减速。
            _a_up_raw = float('inf')
        else:
            _a_up_raw = (_p0
                         + K_VEL * (float(v_i[0]) - _vz_now)
                         + K_POS * (float(x_i[0]) - float(error[0])))
        #   上界 a_ver_need = "要在剩余高度 h 内把 vz 刹到目标垂速"所需的值。
        #   施加【超过】它就等于刹得比需要更狠 => 在离地还有 h 时把速度刹到 0
        #   （停住/往上飞）。故 a_up <= a_ver_need 是"保证会落地"的充要条件。
        #   该公式已被规划自身验证：node 76 g0+10.98^2/(2*3.84)=25.49 vs 规划 25.49。
        _h_req = max(AVER_H_FLOOR, _h_now)
        if _vz_now > 0.0:
            #   上升保护：需求公式在 vz>0 时给出 >g0 的上界，min() 会保留向上的
            #   指令 => 数值发散（实测从 h=3/vz=+5 发散到 h=624 km）。
            #   故直接要求"先停止上升"，并加比例项加速反向。
            _a_ver_need = G0 - AVER_CLIMB_K * _vz_now - AVER_CLIMB_MARGIN
        else:
            _a_ver_need = (G0 + (_vz_now * _vz_now - AVER_TGT_VZ * AVER_TGT_VZ)
                           / (2.0 * _h_req))
        _a_up = min(_a_up_raw, _a_ver_need)
        #   正向下限：油门下限对应的加速度，保证姿态权限不会塌到 0
        #   （实测 36/70 帧零油门自由落体，7.2 s 掉 292 m）。
        _min_mag = self._gfold_floor_effective(
            self.a_cap_real(), self.v.mass, _p0,
            float(error[0]) - float(x_i[0]),
            float(np.linalg.norm(np.asarray(u_i, float))))
        self.min_mag_eff = float(_min_mag)
        self.dbg_min_mag_eff = float(_min_mag)
        #   下限与上界可能冲突（need 很小时）：此时以【不违反 a_ver_need】
        #   为准，因为"保证会落地"比"托高姿态权限"更不能牺牲。
        #   【注意 G0*0.99 只用于给【保底值】封顶，绝不是给 a_up 封顶】
        #     若把 a_up 整体限到 < g0，净竖直恒为负、永远无法减速
        #     （实测该错误会让落地垂速达到 -116 m/s）。
        #     a_up 的真正上界是 _a_ver_need，它已保证"能在剩余高度内停住"。
        if AVER_FLOOR_ENABLE:
            _fe = max(0.0, min(float(AVER_FLOOR), _p0, G0 * 0.99))
            self.dbg_av_floor = float(_fe)
            _a_up = max(_a_up, min(_min_mag, _a_ver_need),
                        min(_fe, _a_ver_need))
        else:
            _a_up = max(_a_up, min(_min_mag, _a_ver_need))
            self.dbg_av_floor = 0.0
        _a_up = max(0.0, _a_up)

        # ---------------- ⑦ 水平能力夹限 ----------------
        #   水平加速度的物理上限 = a_up*tan(允许倾角)。
        #   允许倾角 _tilt_allow 已在 ④ 算出（含高度上限与终端收敛），
        #   这里只消费它，不再重算 —— 否则两处口径会漂移。
        self.dbg_tf_cap = float(_tilt_allow)
        _a_h_max = _a_up * math.tan(math.radians(_tilt_allow))
        if _ah_dem > _a_h_max and _a_h_max > 1e-09:
            _ah_want = _a_h_max
            self.dbg_ah_sat = 1.0
        else:
            _ah_want = _ah_dem
            self.dbg_ah_sat = 0.0

        # ---------------- ⑧ 内环：限 a_h 的加加速度（措施2）----------------
        #   由倾角速率限制换算：a_h = a_up*tan(theta) =>
        #       d(a_h)/dt = a_up * sec^2(theta) * dtheta/dt
        #   取物理判据 slew = sqrt(2*alpha*headroom)（"加速到 slew 再同等减速
        #   停住，总角行程 = headroom"），则
        #       j_max = a_up * sec^2(theta) * sqrt(2*alpha*headroom)
        #   随 alpha（正比油门、实测 8 倍变化）【在线导出】，不是固定常数。
        _alpha_d = float(getattr(self, 'dbg_alpha_max', 0.0) or 0.0)
        _sec2 = 1.0 + (_ah_want / max(1e-09, _a_up)) ** 2
        if _alpha_d > 1e-09:
            _j_phys = (_a_up * _sec2 * math.sqrt(
                2.0 * math.radians(_alpha_d)
                * math.radians(REF_SLEW_HEADROOM_DEG)))
        else:
            #   alpha 采样失败时的保守兜底（不冻结，给基本跟踪能力）
            _j_phys = JERK_GUARD_MIN
        #   【2026-10-10 P1】把物理上限按 JERK_SLEW_FRAC 收紧。
        #     原上限的等效倾角速率约 39~44 deg/s，而实际指令 p50 只有
        #     3.7~10.6 deg/s ==> 限速器从不生效（实测 j_used/j_max p50=0.20、
        #     ah_cap_bind=0%）。乘 0.50 后等效上限约 20 deg/s。
        #     约束力（离线逐帧判据比较，非闭环仿真）：被限帧占比
        #     由 FRAC=1.0 时的 1~6% 升到 7~16% ==> 只咬最激进的少数帧，
        #     不改写正常跟踪。
        #     【为什么是"乘比例"】物理上限正比 sqrt(alpha)，而 alpha 随油门
        #     变化 8 倍；比例收紧可自动跟随权限，低油门端不会过紧。
        _j_lim = _j_phys * max(0.05, min(1.0, float(JERK_SLEW_FRAC)))
        _j_max = max(JERK_GUARD_MIN,
                     min(JERK_GUARD_MAX, _j_phys, _j_lim))
        self.dbg_j_phys = float(_j_phys)
        self.dbg_j_lim = float(_j_lim)
        _dt_j = float(getattr(self, 'last_pkt', 0.02))
        if not (_dt_j > 1e-09):
            _dt_j = 0.02
        #   ------------------------------------------------------------
        #   【内环状态必须是"真正发出去的 a_h"】
        #     初版把能力夹限放在限速【之后】：
        #         ah_cmd = clip_rate(ah_prev, ah_want)
        #         ah_cmd = min(ah_cmd, a_h_max)      <- 又改了一次
        #     于是状态与实际输出脱节 —— 当 a_h_max 在一帧内收缩时，
        #     输出被拉低的幅度不受 j_max 约束，实测 j_used 冲到 39.0
        #     （j_max 只有 4.97，超 7.8 倍），"限加加速度"形同虚设。
        #   【修法】先算"目标"，再做限速，且【把限速结果作为状态】：
        #         target = min(ah_want, a_h_max)     <- 能力可行的目标
        #         ah_cmd = clip_rate(ah_prev, target)
        #         tf_ah_prev = ah_cmd                <- 状态 = 实际输出
        #     这样 |ah_cmd - ah_prev| <= j_max*dt 恒成立（限速器定义）。
        #     注意 a_h_max 收缩时，目标变小、限速器自然把输出按 j_max
        #     的斜率降下来，而不是硬切。
        #   ------------------------------------------------------------
        _ah_target = max(0.0, min(_ah_want, _a_h_max))
        _ah_prev = float(getattr(self, 'tf_ah_prev', 0.0))
        _step_j = _j_max * _dt_j
        _ah_lim = _ah_prev + max(-_step_j, min(_step_j, _ah_target - _ah_prev))
        _ah_lim = max(0.0, _ah_lim)
        #   能力上限是【硬安全限】，可以覆盖加加速度限制：
        #   a_h_max = a_up*tan(tilt_allow) 依赖本帧的 a_up 与倾角上限，
        #   二者都可能在一帧内下降（例如终端收倾角、油门下限放开）。
        #   此时"少给水平加速度"是物理事实，不能为了满足 jerk 曲线而超发。
        #   故输出取 min，并把【真正发出去的】值作为下一帧的状态，
        #   否则状态与实际输出脱节，jerk 限制会形同虚设。
        _ah_cmd = min(_ah_lim, _a_h_max)
        _cap_bind = 1.0 if (_a_h_max < _ah_lim - 1e-09) else 0.0
        self.tf_ah_prev = _ah_cmd
        self.dbg_ah_cmd = float(_ah_cmd)
        self.dbg_j_max = float(_j_max)
        self.dbg_j_used = (_ah_cmd - _ah_prev) / _dt_j
        self.dbg_ah_cap_bind = _cap_bind
        self.dbg_tf_slew_allow = float(_j_max)

        # ---------------- ⑨ 由 a_h 反解倾角（措施3：口径闭环）----------------
        #   【关键】tilt 与 a_h 用【同一个 a_up】，不再有前向 a_up_est /
        #   后向 av_up 的往返失配。旧版该失配 p10=0.518 / p90=1.721。
        _tilt_cmd = math.degrees(math.atan2(_ah_cmd, max(1e-09, _a_up)))
        #   上限用 _tilt_allow（含终端收敛），与 ⑦ 的能力夹限同口径。
        _tilt_cmd = max(0.0, min(_tilt_allow, _tilt_cmd))
        self.dbg_tf_tilt_cmd = float(_tilt_cmd)

        # ---------------- ⑩ 构造指令（一次成型）----------------
        _tr = math.radians(_tilt_cmd)
        _a_h_new = _a_up * math.tan(_tr)
        _a_cmd = np.array([_a_up,
                           _a_h_new * float(_az[0]),
                           _a_h_new * float(_az[1])])
        self.dbg_av_in_up = float(_a_cmd[0])
        self.dbg_av_plan = float(_p0)
        self.dbg_av_need = (float(_a_ver_need)
                            if _a_ver_need == _a_ver_need else float('nan'))

        # ---------------- ⑪ 最终安全限幅（conic_clamp）----------------
        #   保留它作为【最后一道】安全网：竖直保底 + 锥限幅 + 幅值上限。
        #   注意：本版给它的输入是由 (a_up, a_h, az) 直接构造的向量，
        #   不再含旧律 PD 的幅值，故不存在"幅值串扰"。
        _a_cap_now = self.a_cap_real()
        _tilt_now = self._solver_cone_deg(self.gfold_n_i, N)
        _tilt_now = min(max(_tilt_now, PCS_END_DEG), 89.0)
        self.tilt_cap = _tilt_now
        target_a = self.conic_clamp(_a_cmd, _min_mag, 1.00 * _a_cap_now,
                                    _tilt_now,
                                    min_mag_override=_min_mag,
                                    plan_a_up=_p0)
        self.dbg_av_out_up = float(target_a[0])
        self.dbg_floor_lift = float(target_a[0]) - float(_a_cmd[0])
        self.dbg_a_ver_out = float(target_a[0])

        #   【一致性】限幅后重新取 a_up/a_h/tilt，保证
        #   "a_h == a_up*tan(tilt)" 与 "az 来自同一矢量" 两条恒等式成立。
        _a_up_f = max(0.0, float(target_a[0]))
        _ah_v = np.asarray(target_a, float)[1:3]
        _a_h_f = float(np.linalg.norm(_ah_v))
        if _a_h_f > 1e-09:
            _az = (_ah_v / _a_h_f).copy()
            self.tf_az_dir = _az
        _tilt_f = math.degrees(math.atan2(_a_h_f, max(1e-09, _a_up_f)))
        _tilt_f = max(0.0, min(_tilt_now, _tilt_f))
        self.tilt_cmd = float(_tilt_f)
        self.dbg_av_up = float(_a_up_f)
        self.dbg_av_net = float(_a_up_f - G0)

        # ---------------- ⑫ 姿态参考方向（由同一个 tilt/az 构造）----------------
        _trf = math.radians(_tilt_f)
        _dir_new = np.array([math.cos(_trf),
                             math.sin(_trf) * float(_az[0]),
                             math.sin(_trf) * float(_az[1])])
        _dn = float(np.linalg.norm(_dir_new))
        if _dn > 1e-09:
            self.target_direction = _dir_new / _dn
        #   参考方向速率限制（物理判据，防参考本身超出能力）
        self.target_direction = self._limit_ref_slew(self.target_direction)

        # ---------------- ⑬ 跟踪误差与分项诊断 ----------------
        _dpos = x_i - error
        self.dbg_n_i = float(self.gfold_n_i)
        self.dbg_plan_alt = float(x_i[0]) + TARGET_ALT
        self.dbg_plan_vz = float(v_i[0])
        self.dbg_plan_vh = float(np.linalg.norm(np.asarray(v_i, float)[1:3]))
        self.dbg_trk_pos = float(np.linalg.norm(_dpos))
        self.dbg_trk_pos_up = float(_dpos[0])
        self.dbg_trk_pos_h = float(np.linalg.norm(np.asarray(_dpos, float)[1:3]))
        self.dbg_trk_vel = float(np.linalg.norm(np.asarray(v_i, float)
                                                - np.asarray(vel, float)))
        self.dbg_plan_nj = float(self.gfold_n_i)
        self.dbg_plan_tilt_j = float(_tilt_plan)
        #   conic_clamp 位置项限幅标志（新律不单独夹位置项，恒 0）
        self.dbg_ca_posclip = 0.0

        #   水平指令分项：把指令拆成三项，各自与速度方向求夹角余弦。
        #   口径：取【限幅前】的 eguidance 矢量（那才是律真正算出的东西）。
        #   数值保护：|速度水平| < 1.0 时方向无意义，dot 置 NaN。
        try:
            _up3 = np.asarray(target_a, float)
            _uh_vec = np.asarray(u_r, float)[1:3]
            _dv_vec = (np.asarray(v_r, float) - np.asarray(vel, float))[1:3]
            _dp_v = (np.asarray(x_r, float) - np.asarray(error, float))[1:3]
            self.dbg_u_h = float(np.linalg.norm(_uh_vec))
            self.dbg_pdv_h = float(np.linalg.norm(_dv_vec) * _ca_kv)
            self.dbg_pdp_h = float(np.linalg.norm(_dp_v) * _ca_kp)
            _velh = np.asarray(vel, float)[1:3]
            _vh_mag = float(np.linalg.norm(_velh))
            if _vh_mag > 1.0:
                _vd = _velh / _vh_mag

                def _dot_with_vel(v):
                    n = float(np.linalg.norm(v))
                    return (float(np.dot(v, _vd)) / n
                            if n > 1e-09 else float('nan'))

                self.dbg_u_dot = _dot_with_vel(_uh_vec)
                self.dbg_pdv_dot = _dot_with_vel(_dv_vec)
                self.dbg_pdp_dot = _dot_with_vel(_dp_v)
                _cmd_h = np.asarray(_up3, float)[1:3]
                _cn = float(np.linalg.norm(_cmd_h))
                self.dbg_cmd_dot = (float(np.dot(_cmd_h, _vd)) / _cn
                                    if _cn > 1e-09 else float('nan'))
            else:
                self.dbg_u_dot = float('nan')
                self.dbg_pdv_dot = float('nan')
                self.dbg_pdp_dot = float('nan')
                self.dbg_cmd_dot = float('nan')
        except Exception:                            # noqa: BLE001
            self.dbg_u_dot = float('nan')
            self.dbg_pdv_dot = float('nan')
            self.dbg_pdp_dot = float('nan')
            self.dbg_cmd_dot = float('nan')

        #   水平误差方向（幅值之外，回答"往哪边偏"）
        _he = np.array([float(error[1]), float(error[2])])
        _hd = np.array([float(target_a[1]), float(target_a[2])])
        _he_norm = float(np.linalg.norm(_he))
        _hd_norm = float(np.linalg.norm(_hd))
        self.dbg_he_n = float(_he[0])
        self.dbg_he_e = float(_he[1])
        if _hd_norm > 1e-09:
            self.dbg_hd_n = float(_hd[0] / _hd_norm)
            self.dbg_hd_e = float(_hd[1] / _hd_norm)
        else:
            self.dbg_hd_n = 0.0
            self.dbg_hd_e = 0.0
        if _he_norm > 1e-06 and _hd_norm > 1e-09:
            #   期望方向 = -he/|he|（把载具推回瞄准点）
            _want = -_he / _he_norm
            _cosm = float(np.dot(_hd / _hd_norm, _want))
            self.dbg_h_misalign = math.degrees(
                math.acos(max(-1.0, min(1.0, _cosm))))
        else:
            self.dbg_h_misalign = float('nan')

        self.a_cmd_vec = target_a
        return target_a, self.gfold_n_i

    def _thrust_scale(self):
        """【自校准】kRPC 报的推力 与 实际推力 的比值。

        【为什么需要】gfold_log_20260928_235713.csv 实测（数据极稳，t>=4.1）：
            kRPC available_thrust 报 = 8.990 MN
            由 thrust_n / thr_cmd 反推 = 12.71 MN
            ratio = 1.414
          最佳候选：9 x 936.508 kN(海平面) x 1.5(限幅) = 12.643 MN（差 0.5%）
          ⇒ 本 kRPC 构建的 available_thrust 【既不含限幅、又取了真空值】。

        【为什么不在线测】可以：真实可用推力 = Vessel.thrust / throttle。
          但 throttle 很小或推力为 0 时该比值不可靠（实测有 0.000 的帧）。
          故做法是：每次采样都把有效样本累积进指数平均，仅在
          (throttle 足够大 且 推力>0) 时更新；未标定前先用 1.0（保守，
          不会故意抬高低限）。

        【为什么这样更稳】它自动适配：用户改限幅、TweakScale、不同大气/
          不同载具，都会反映到 Vessel.thrust 上，无需改代码。

        【返回值】>=1.0 的比值。恒 >=1 是因为 kRPC 只会低估（若实测
          比 reported 小，说明我们的 throttle 记录不准，此时不采信）。
        """
        return float(getattr(self, '_tscale', 1.0))

    def update_thrust_scale(self, thr_cmd):
        """用一帧的 (throttle, Vessel.thrust) 更新自校准系数。

        【调用点】apply() 里每帧调用一次。
        【取中位数，不用指数平均】见下方"为什么"。

        ============================================================
        【2026-09-29 重写：第一版有三个缺陷，实飞 gfold_log_20260929_001024
          把它们全部暴露出来】
        ------------------------------------------------------------
        【缺陷 1：推力【滞后】于油门指令，逐帧相除会得到垃圾比值】
          发动机有惯性。实测同一帧对照：
              t=0.48  thr_cmd=0.222  但 thrust=4.301 MN -> ratio 2.25（加速中）
              t=0.69  thr_cmd=0.714  但 thrust=2.821 MN -> ratio 0.46（减速中）
          这两个读数并非同一时刻的真实对应关系，只是相位差造成的假象。
        【缺陷 2：采样门槛造成【自锁】】
          旧门槛是 thr_cmd >= 0.12。而 t_scale 被污染偏高后，
          下限 = 0.96*g0*m/(avail*ts) 被算得【偏低】-> thr_cmd 掉到 0.113
          -> 0.113 < 0.12 -> 不再采样 -> t_scale 冻结在 1.839 长达 8.8 秒
          （真实值约 1.47，即偏高 25%）。这个自锁让它无法自愈。
        【缺陷 3：EMA 对离群值不稳健】
          单个 2.25/2.50 的样本就能把 EMA 拉动 +0.15~+0.20。

        【修法】
          ① 只采信【油门稳定】的帧：与上一帧的 thr_cmd 变化 < 0.02。
             这样发动机已进入稳态，thrust/cmd 才是真实对应关系。
          ② 门槛降到 0.08（打破自锁）。
          ③ 采样窗口收到 [1.0, 2.0]：物理上 kRPC 只会【低估】推力，
             且实测真值在 1.41~1.53，故 2.0 以上必为相位假象。
          ④ 用最近 N 个有效样本的【中位数】，而不是 EMA ——
             中位数对离群值免疫，且允许后续样本纠正早期污染。
        ============================================================
        """
        try:
            _thr = float(self.v.thrust)
            _cmd = float(thr_cmd)
            # 【① 只在油门稳定时采样】与上一帧比较
            _prev_cmd = float(getattr(self, '_tscale_prev_cmd', float('nan')))
            self._tscale_prev_cmd = _cmd
            if _prev_cmd == _prev_cmd and abs(_cmd - _prev_cmd) > 0.02:
                return
            # 【② 门槛 0.08】低到不会与下限自锁
            if _cmd < 0.08 or _thr < 1.0e5:
                return
            _rep = float(self.v.available_thrust)
            if _rep < 1e5:
                return
            _ratio = (_thr / max(1e-6, _cmd)) / _rep
            # 【③ 物理窗口】只可能低估
            if not (1.0 <= _ratio <= 2.0):
                return
            # 【④ 中位数】保留最近 20 个样本
            _buf = getattr(self, '_tscale_buf', None)
            if _buf is None:
                _buf = []
                self._tscale_buf = _buf
            _buf.append(_ratio)
            if len(_buf) > 20:
                del _buf[0]
            _s = sorted(_buf)
            self._tscale = float(_s[len(_s) // 2])
        except Exception:                            # noqa: BLE001
            pass

    def _limit_ref_slew(self, want_dir):
        """【2026-09-29 第七步 7(b)-③】把【姿态参考方向】的转动速率限到
        本帧权限允许的范围内，输出连续、不产生拐点。

        ====================================================================
        【与 6(b) 硬限幅的关系】
          6(b) 的 limit_tilt_dir 是【幅度限幅】：把倾角夹到 <=30 deg。
             它的副作用：min(raw,30) 在穿越处【斜率突变】，实测
             |d(tilt_dir_cmd)/dt| 最大 78.2 deg/s 全部来自这些边界。
          本函数是【速率限幅】：在保持方向连续的前提下，把每秒转过的角度
             限制到载具真能跟上的水平。两者【串联使用】：先夹幅度、再限速率。

        【限速上限怎么算】
            slew_max = sqrt(2 * alpha * headroom)      [rad/s]
          alpha    = dbg_alpha_max（本帧实测可用角加速度，随油门变化）
          headroom = REF_SLEW_HEADROOM_DEG（允许的角行程，度）
          物理意义：从静止加速到 slew_max 再以同样减速度停住，总角行程正好
          = headroom。因此按此限速，参考【永远停在能力之内】。

        【实现】把 want_dir 的方向用球面插值往 current 方向回拉：
          不足则原样返回（不干预）；超出则沿大圆方向只走 slew_max*dt。
          这样输出【始终是单位向量】，且斜率有界 => 无拐点 => D 项干净。
        ====================================================================
        """
        if not REF_SLEW_ENABLE:
            return want_dir
        # ============================================================
        # 【2026-10-07 第十八步 18(b)：ref_clip 是【粘滞】的 —— 每帧必须先归零】
        # ------------------------------------------------------------
        # 【故障】dbg_ref_clamped 全文只有两处赋值：
        #     __init__ 里 = 0.0，以及本函数限速分支里 = 1.0
        #   【没有任何地方把它复位】⇒ 一旦某帧被限，该列【永远】保持 1。
        #
        # 【实测证据（gfold_log_20261007_152048.csv，89 帧）】
        #     首次置 1 在第 3 帧（idx=2）
        #     1 -> 0 的翻转次数 = 0
        #     读数 ref_clip=1 占 87/89 = 97.8%
        #   而同一趟 ref_slew 的 p50 只有 8.50 deg/s（上限可达 45），
        #   即【绝大多数帧根本没被限】，但该列报 97.8% —— 纯属误读。
        #   这正是它此前"看起来限速器在满负荷工作"的原因，会误导排障。
        #
        # 【改法】在函数入口【每帧复位为 0】，只在本帧真的被限时置 1。
        #   放在 REF_SLEW_ENABLE 判断【之后】：限速器关闭时该列应保持
        #   初始 0（"没有限速"），由 __init__ 的 0.0 承担。
        #   放在 try【之前】：保证异常兜底路径也不会残留上一帧的 1。
        #   口径与 dbg_pid_sat_x / dbg_ref_slew 等"本帧瞬时量"一致。
        # ============================================================
        self.dbg_ref_clamped = 0.0
        try:
            _w = np.asarray(want_dir, dtype=float)
            _n = float(np.linalg.norm(_w))
            if not (_n > 1e-9):
                return want_dir
            _w = _w / _n
            _prev = getattr(self, '_ref_slew_prev', None)
            if _prev is None:
                self._ref_slew_prev = _w.copy()
                return _w
            _dt = float(getattr(self, 'last_pkt', 0.0))
            if not (_dt > 1e-9):
                self._ref_slew_prev = _w.copy()
                return _w
            # 本帧权限 -> 允许的最大角速率 [rad/s]
            #   allowed = sqrt(2*alpha*headroom)   [headroom 用角度]
            _a = math.radians(float(getattr(self, 'dbg_alpha_max', 0.0) or 0.0))
            _head = math.radians(REF_SLEW_HEADROOM_DEG)
            if _a > 1e-9:
                _slew_deg = math.degrees(math.sqrt(2.0 * _a * _head))
            else:
                _slew_deg = REF_SLEW_MIN
            # 上下限：低油门端不至于冻结、高油门端不至于过快
            _slew_deg = max(REF_SLEW_MIN, min(REF_SLEW_MAX, _slew_deg))
            _slew = math.radians(_slew_deg)
            _ang = math.acos(max(-1.0, min(1.0, float(np.dot(_prev, _w)))))
            _step = _slew * _dt
            if _ang <= _step or _ang < 1e-9:
                self._ref_slew_prev = _w.copy()
                self.dbg_ref_slew = math.degrees(_ang) / _dt
                return _w
            # 沿大圆插值到只走 _step 的位置
            _t = _step / _ang
            _out = _prev + _t * (_w - _prev)
            _on = float(np.linalg.norm(_out))
            _out = _out / _on if _on > 1e-9 else _prev.copy()
            self._ref_slew_prev = _out.copy()
            self.dbg_ref_slew = math.degrees(_step) / _dt
            self.dbg_ref_clamped = 1.0
            return _out
        except Exception:                            # noqa: BLE001
            return want_dir

    def _att_sched_factor(self):
        """【2026-09-29 第七步 7(b)】增益调度系数（用户要求"PID 随油门变化"）。

        ====================================================================
        【为什么按 alpha 调度而不是按油门】
        用户观察：载具惯量大、姿态能力与油门强相关。实测证实——
            alpha_max（=tau/I，含油门效应）与油门的 pearson r = 0.818，
            变化范围 4.44 ~ 35.37 deg/s^2，即 8.0 倍。
        二阶姿态模型：e'' = -alpha*(kp*e + kd*e')
            wn   = sqrt(alpha*kp)
            zeta = (kd/2)*sqrt(alpha/kp)
        kp 固定 2.0 时：zeta 在 0.246(低油门) ~ 0.694(高油门) 之间摆 2.8 倍。
        低油门端 zeta<0.3 严重欠阻尼 => 这正是"低油门时过冲"的根源。

        【调度律】令 kp 正比于 alpha：
            kp = kp0 * (alpha / alpha_ref)
        则 alpha/kp = alpha_ref/kp0 = 常数
            => zeta = (kd/2)*sqrt(alpha_ref/kp0) 【与 alpha 无关】
        即 zeta 成为常数，这正是"防止过冲随油门变化"想要的。

        【为什么用 sqrt(alpha/alpha_ref) 而不是线性】用【实测】决定，见文件头：
          线性调度会让 kp 在低油门端掉到很小、高油门端涨到 16（远超饱和阈值
          1/|e|max=3.0）。折中用平方根：变化范围从 8 倍压到 sqrt(8)=2.8 倍，
          两头都不极端。且 zeta ∝ sqrt(alpha/kp) 下用平方根调度得到
          zeta ∝ (alpha)^(1/4)，仍单调改善低油门端的阻尼。

        【上下限】TAU_SCHED_MIN/MAX 防止 alpha 异常（首帧、采样失败）时
          kp 变成 0 或爆掉。默认 1.0 即"不调度"。
        ====================================================================
        """
        if not TAU_SCHED_ENABLE:
            return 1.0
        try:
            a_now = float(getattr(self, 'dbg_alpha_max', float('nan')))
            if not (a_now == a_now) or a_now <= 1e-9:
                return 1.0
            # 参考权限：用本载具在本段油门的典型值。取 20 deg/s^2 为基准
            #   （实测中位 ~18.2，见 7(b) 段的统计），使系数在低油门端 <1、
            #   高油门端 >1，范围被 TAU_SCHED_MIN/MAX 夹住。
            _fac = math.sqrt(a_now / 20.0)
            return float(max(TAU_SCHED_MIN, min(TAU_SCHED_MAX, _fac)))
        except Exception:                            # noqa: BLE001
            return 1.0

    def a_cap_real(self, mass=None):
        """【真实可用推力加速度】= available_thrust / mass [m/s^2]。

        【为什么必须用 available_thrust 而不是 max_thrust】
          2026-09-28 实飞事故（用户发现）：本载具一级 9 台 SSME 在 craft
          文件里被设为 thrustPercentage = 150。
          kRPC 官方文档（本地 krpc/services/spacecenter.py）明确：
            · Engine.max_thrust
                thrust ... with its throttle AND throttle limiter set to 100%
                ⇒ 【不含】150% 限幅
            · Engine.available_thrust
                takes the engine current thrust_limit and atmospheric
                conditions into account
                ⇒ 【含】150% 限幅
            · Vessel.max_thrust / available_thrust = 各引擎对应量之和
          【实测结果（2026-09-29 修正此前的错误推断）】
            Vessel.max_thrust       = 8.990 MN
            Vessel.available_thrust = 8.990 MN   <- 与 max_thrust 相同！
            而由 thrust_n / thr_cmd 反推的真实可用推力 = 12.71 MN
              （最佳候选 9 x 936.508kN 海平面 x 1.5 限幅 = 12.643，差 0.5%）
          即本 kRPC 构建下 available_thrust 【既没含限幅、又取了真空值】，
          低估 1.414 倍（见 _thrust_scale 的完整证据与自校准做法）。

        【原来错在哪】全文件把 v.max_thrust 当 a_cap，于是：
          · 控制器：throttle = |target_a| / a_cap 被高估 1.5 倍
            ⇒ 我按 0.16 设的 gfold 油门下限，相对【真实】悬停点
              (g0*m/T_real = 0.111) 其实是 1.44 倍 ⇒ 【下限自己在把
              火箭往上推】。这正是此前一直查不出的【被顶上去】的原因。
          · 求解器：t_max 传的是 8.99e6，等于让求解器【按一台弱 50% 的
            火箭规划】⇒ 规划出更长的 tf、更大的油门 ⇒ 载具执行时冲过头。

        【为什么仍用 available_thrust】按官方定义它应含当前限幅与大气条件，
          比 max_thrust 更接近真值。但【实测本构建不满足该定义】，故必须再乘
          _thrust_scale() 在线自校准 —— 这样无论 kRPC 或载具怎么变都安全。

        【调用方式】不传参时用实时 body mass。传 mass 可复用已读到的值，
          避免多一次 RPC。
        """
        _m = max(1.0, float(self.v.mass if mass is None else mass))
        try:
            _t = float(self.v.available_thrust)
        except Exception:                            # noqa: BLE001
            # 【兜底】旧版 kRPC 或异常时退回 max_thrust，并只告警一次。
            if not getattr(self, '_avail_warned', False):
                self._avail_warned = True
                print('[WARN] available_thrust 不可用，退回 max_thrust')
                print('[WARN] 若载具用了推力限幅，油门会偏大 1/limiter 倍')
            _t = float(self.v.max_thrust)
        # 【2026-09-29 关键修正：乘自校准系数】
        #   实测（gfold_log_20260928_235713.csv，t>=4.1 数据极稳）：
        #     kRPC 报 available_thrust = 8.990 MN
        #     而 thrust_n/thr_cmd 反推的真实可用推力 = 12.71 MN
        #     ratio = 1.414
        #   候选比对：9 x 936.508kN(海平面) x 1.5(限幅) = 12.643 MN（差 0.5%）
        #   ⇒ kRPC 的 available_thrust 在本构建下【既没算限幅、又用了真空值】，
        #     低估了 1.414 倍。
        #   后果（此前所有"被顶上去"的根因）：
        #     · throttle = |a|/a_cap 被高估 1.414 倍
        #     · 悬停点被算高 1.414 倍 ⇒ 下限 0.16 变成真实悬停的 1.36 倍
        #       ⇒ 下限自己把火箭往上推（实测 a_up = +3.57 m/s^2 持续爬升）
        #   这里乘上在线测得的系数，使两者同口径。
        _t = _t * self._thrust_scale()
        return _t / _m

    def _gfold_floor(self, a_cap, mass):
        """gfold 段【执行油门下限】对应的加速度幅值下限 [m/s^2]。

        【为什么需要这个函数（审计 MEDIUM-2，CONFIRMED）】
          原先写死 GFOLD_THROTTLE_MIN=0.16。那个值是按交班质量 156.9 t
          取的，但 gfold 段质量会一路掉到 136~149 t，而悬停油门
          g0*m/T 随质量下降而下降：
              m=156.9 t -> 0.1712    m=149.0 t -> 0.1626
              m=143.4 t -> 0.1563    m=136.8 t -> 0.1492
          ⇒ 当 m < 149 t 时 0.16 > 悬停，【下限本身就在把火箭往上推】，
            与当初误取 0.35 是同一类失控（只是更温和）。

        【修法】下限 = 该质量下悬停油门的 GFOLD_THROTTLE_MARGIN 倍，
          并再取一个绝对值上限 GFOLD_THROTTLE_MIN：
              floor = min(GFOLD_THROTTLE_MIN,
                          GFOLD_THROTTLE_MARGIN * g0*m/T)
          这样：
            · 质量大时（交班）  0.96*0.1712 = 0.164 > 0.16 ⇒ 取 0.16
              （与修改前一致，不牺牲已有权限）
            · 质量小时（m=136.8t）0.96*0.1492 = 0.143 < 0.16 ⇒ 取 0.143
              （恒留 4% 的下降余量，不会反冲）

        【返回值】下限对应的加速度幅值 = floor * a_cap。
          调用方用的是加速度域（min_mag），故这里直接换算好。
        """
        _m = max(1.0, float(mass))
        # 【2026-09-29】悬停点必须用【真实可用推力】。
        #   注意 available_thrust 本身还低估 1.414 倍（实测），
        #   故必须乘自校准系数 _thrust_scale()，否则下限会高于真实悬停
        #   ⇒ 下限自己把火箭往上推（实测 a_up=+3.57 m/s^2，一路爬到 409 m
        #   后 vz 转正、永不落地，导致 PID 根本无法被检验）。
        _t_true = max(1.0, float(self.v.available_thrust)) * self._thrust_scale()
        _hover = G0 * _m / _t_true
        floor = min(GFOLD_THROTTLE_MIN, GFOLD_THROTTLE_MARGIN * _hover)
        return floor * a_cap

    def _gfold_floor_effective(self, a_cap, mass, plan_a_up, err_up,
                               plan_mag=None):
        """【2026-09-29 第三步改动 3(a)】按"是否要下降"放开油门下限。

        ====================================================================
        【为什么必须改：实飞日志 gfold_log_20260929_164243 的铁证】
        --------------------------------------------------------------------
        该趟用的是新的 [80 m/s 包络] 规划（vdesc=80 全程生效）。离线用
        【同一交班状态】复现那条规划，逐节点对比"规划要的推力"与
        "控制器的油门下限"（m=157.1 t, a_cap=81.71 m/s²）：

            node   plan |u|   floor_mag(9.41)   规划低于下限?
             0       5.72        9.41             YES
            12       5.75        9.41             YES
            24       5.76        9.41             YES
            36       7.81        9.41             YES
            48      28.12        9.41              -
            78      48.13        9.41              -
            => 38/79 个节点（48%）规划要的推力【低于下限】

        【机制】conic_clamp 先把竖直分量夹到 >= min_mag：
            a_ver = max(a_ver_min, min(a_ver_max, a_ver))
        当 |u| < floor_mag 时，"跟轨迹"这个意图会被下限直接推翻 ——
        载具被迫提供【比规划更多】的推力，于是：
          · 竖直方向被额外顶上去（下降比规划慢）
          · 水平方向拿不到应有的份额
          · 实测 |a_cmd| 恒被钉在 9.41（= floor_mag）长达 2.5 s
            （日志帧 7..31，规划只要 5.7）

        【旧注释为什么曾把下限当必需】它是为"姿态权限"加的：gimbal 力矩
          正比于油门，油门太低则姿态转不动。但那个理由只在【需要大角度
          转姿】时成立；巡航段规划本来就是接近竖直的小推力，此时下限
          反而把姿态"锁"在竖直、阻止它去追水平修正。

        【判据（本改动的核心）】只在【确实要往下走】时放开下限：
            err_up = error[0] - x_i[0]
                   = 本机相对【规划点】的高度偏差（正 = 本机高于规划点）
            plan_a_up - G0
                   = 规划本帧的净竖直加速度（推力竖直分量 - 重力）
          若载具【已经落后于规划】(err_up > 0) 或【规划本身要求净向下加速】
          (plan_a_up - G0 < 0)，则【降低】下限，让 a_ver 由规划与 PD 决定。
          否则（巡航/需要姿态权限）保留原下限。

        【口径陷阱（第一版写错过，实测暴露）】err_up 必须是【相对规划点】，
          不能传 error[0]（相对【瞄准点】的高度）—— 下降全程它都是大正数
          （交班点 650 m），拿它当判据会【全程放开下限】，等于把姿态权限
          彻底丢掉，与改动目的相反。

        【为什么不直接删掉下限】它会退回"油门下限 0.05 < 悬停 => 近乎
          自由落体、无姿态权限"那个老故障（见 GFOLD_THROTTLE_MIN 的长注释）。
          保留"该有时有、该无时无"是两边都不牺牲的做法。

        ================================================================
        【2026-09-29 第四步 4(a)：放开时【放一个非零小值】，不能放到 0】
        ----------------------------------------------------------------
        【事故】第一版放开到 0，直接制造了一个【零值陷阱】。实飞
          gfold_log_20260929_173800.csv：36/70 帧 thr_cmd == 0.000，
          持续 7.21 s（帧 7..42），期间 a_cmd_up = a_cmd_h = 0.000，
          载具从 597.5 m 纯自由落体到 305.7 m、vz 从 -48 掉到 -69。
        【根因】conic_clamp 用 a_ver = max(a_ver_min, ...) 给竖直分量托底，
          而 a_ver_min 由 min_mag 算出。min_mag = 0 => a_ver_min = 0 =>
          【负的竖直指令不再被托底】，直接返回 [0,0,0]。
          用真实的 conic_clamp 实测（a_cap=81.71）：
              a_ver=-0.73 a_hor=5.0  min_mag=0.0   -> [0.00,0.00] 推力 0.0000
              a_ver=-0.73 a_hor=5.0  min_mag=5.70  -> [5.70,0.10] 推力 0.0698
        【这是个自锁】载具下降比规划慢 => PD 速度项把竖直指令压负
          => 下限已放开 => 全零 => 下降更快 => 下一帧更负 => 越陷越深。
          用该趟 70 帧复算：raw_a_ver 有 31 帧为负。
        【修法（第四步 4(a)】先改成返回 GFOLD_THROTTLE_RELEASE*a_cap（=0.05），
          它修好了零推力陷阱（实飞 181402 全零帧 0 个），
          但实测发现它只是把天花板降低了（见下方 5(a)）。
        ================================================================
        【第五步 5(a)：放开值必须【不超过规划自己的推力】】
        ----------------------------------------------------------------
        实飞 gfold_log_20260929_181402.csv（4(a) 把放开值从 0 改成 0.05）：
            zero-throttle frames : 0        <- 零推力陷阱确实修好了
            但 frames 4..45 的 thr_cmd 恒等于 0.050（42 帧 / 7.5 s）
        用该趟日志与它自己的规划逐帧对照：
            node   plan |u|   占 a_cap
             0      5.73      0.070
            20      5.76      0.070
            30      8.36      0.102
            45     12.19      0.149
        而 a_cap≈81.7 时 0.05·a_cap = 4.09 m/s²
            => 62/67 帧的【输出幅值等于下限】
        ⇒ 下限【仍然是绑定约束】，只不过从 9.41 降到 4.09 m/s²。

        【正确判据】放开时取【两者较小】：
            floor_released = min(GFOLD_THROTTLE_RELEASE * a_cap, 1.02 * |u_plan|)
        · 上界 1.02·|u_plan| —— 保证下限【永不超过规划要的推力】，
          即"跟轨迹"这个意图再也不会被下限推翻（这才是第三步的本意）。
          取 1.02 而不是 1.00：给离散化/浮点留 2% 余量。
        · 自动 > 0：规划 |u| 是推力幅值、恒 > 0，故 1.02·|u_plan| > 0
          ⇒ 【自动满足"不能放到 0"】，比 4(a) 的固定小值更可靠。
        · 仍保留 GFOLD_THROTTLE_RELEASE 作【兜底上界】。
        ================================================================

        【参数】
          a_cap     : 本帧可用加速度（= a_cap_real）
          mass      : 本帧质量
          plan_a_up : 规划在当前节点的竖直推力分量 u_i[0]
          err_up    : 本机相对【规划点】的高度偏差（error[0] - x_i[0]）
          plan_mag  : 规划在当前节点的推力幅值 |u_i|
                      （None 时退回 4(a) 的固定比例行为）
        【返回】下限对应的加速度幅值 [m/s²]（恒 > 0）
        """
        base = self._gfold_floor(a_cap, mass)
        # 规划本帧的净竖直加速度（推力竖直分量 - 重力）
        net_up = float(plan_a_up) - G0
        # 载具落后于规划、或规划要求净向下加速 => 降低下限
        if float(err_up) > 0.0 or net_up < 0.0:
            cap_by_release = GFOLD_THROTTLE_RELEASE * float(a_cap)
            if plan_mag is None or not (float(plan_mag) > 0.0):
                # 拿不到规划幅值（旧调用点 / 诊断脚本）=> 退回 4(a) 行为
                return cap_by_release
            # 5(a)：下限不得超过规划自己的推力（×1.02 数值余量）
            return min(cap_by_release, 1.02 * float(plan_mag))
        return base

    def _solver_cone_deg(self, index, N):
        """求解器那条【推力指向锥】在第 index 个节点处的半角 [deg]。

        【为什么要与求解器完全一致（第一步改动 1c）】
          求解器（gfold_p3p4._solve_one / gfold_codegen.build）施加的约束是
              u[0,n] >= cos(ang(n)) * |u[:,n]|,
              ang(n) = PCS_START_DEG + (PCS_END_DEG - PCS_START_DEG) * n/(N-2)
          这是【规划可行域的边界】。控制器若用另一条曲线做限幅，两者就
          自相矛盾：限幅比它紧 -> 参考轨迹自己被剪掉（不跟踪）；
          限幅比它松 -> 允许规划从未验证过的姿态（姿态失守）。
          所以这里的公式必须与求解器【逐字对应】。

        【与旧行为的区别】旧代码用 max(tilt_traj + 8, 25)：
          · 25 是地板 -> 末端永远 >= 25°，与 pcs_end=5 矛盾
          · +8 裕量   -> 末端停在 pcs_end+8，仍不满足"< 5°"
        现在直接返回解析锥角，末端 = PCS_END_DEG = 5°。

        【参数】
          index : 当前沿轨迹的索引（gfold_n_i，可含小数）
          N     : 参考轨迹的节点数（= x.shape[1]，P4 为 N4=80）
        """
        _N = max(2, int(N))
        frac = float(index) / max(1.0, float(_N - 2))
        frac = max(0.0, min(1.0, frac))
        return float(PCS_START_DEG + (PCS_END_DEG - PCS_START_DEG) * frac)

    def conic_clamp(self, target_a, min_mag, max_mag, tilt_deg=None,
                    min_mag_override=None, plan_a_up=None):
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
        a_cap = self.a_cap_real()
        # ================================================================
        # 【2026-09-29 第三步 3(a) 修复：这里【必须尊重传入的 min_mag】】
        # ----------------------------------------------------------------
        # 【原来的 bug（本改动第一版暴露）】本函数签名收了 min_mag 参数，
        #   却在函数体内【无视它】、用自己的 _gfold_floor() 重算一遍：
        #       min_mag = self._gfold_floor(a_cap, self.v.mass)
        #   于是 track() 里算好的【条件下限】（该放开时=0）被这里的
        #   【无条件下限】二次改写 —— 3(a) 的改动完全失效。
        #   这条注释的原话是"两处必须一致，否则 track() 的限幅会被这里
        #   的下限二次改写"，说明作者当年已经意识到这个机制，
        #   但当 3(a) 让两处【本来就该不同】时，这行就成了 bug。
        #
        # 【修法】优先用调用方显式给的 min_mag_override（track() 会传它的
        #   条件下限）；没给时沿用旧的"自己算"的行为，保证 conic_clamp
        #   的其它调用点（诊断脚本等）语义不变。
        # ================================================================
        if min_mag_override is not None:
            min_mag = float(min_mag_override)
        else:
            min_mag = self._gfold_floor(a_cap, self.v.mass)
        max_mag = 1.0 * a_cap
        max_tilt = math.radians(CONIC_TILT_DEG if tilt_deg is None else tilt_deg)
        a_hor = float(np.linalg.norm(target_a[1:3]))
        a_ver = float(target_a[0])
        # ============================================================
        # 【第十步 10(a)】竖直保底：与锥角解耦，并用规划自身封顶
        #   原式 a_ver_min = cos(cone)*min_mag 在 cone=85 deg 时只剩 0.087 倍，
        #   等于没有保护（实测 a_up 塌到 0.46）。这里加一个【与锥角无关】的
        #   保底值，且【绝不超过规划自己的 a_ver】—— 理由见常量处：
        #     · 不能 >= g0        ：否则净竖直 >= 0，载具永不落地（反推约束）
        #     · 不能 > plan_a_ver ：否则推翻规划
        #   plan_a_up 为 None 时（旧调用点/诊断脚本）跳过保底，语义不变。
        # ============================================================
        _floor_extra = 0.0
        if (AVER_FLOOR_ENABLE and plan_a_up is not None
                and float(plan_a_up) == float(plan_a_up)):
            _cap = min(float(AVER_FLOOR), float(plan_a_up))
            _cap = min(_cap, G0 * 0.99)        # 硬约束：必须 < g0
            _floor_extra = max(0.0, _cap)
        if a_hor < 1e-09:
            # 纯竖直指令：直接把竖直分量夹到 [min_mag, max_mag]
            return np.array([max(min(a_ver, max_mag), min_mag, _floor_extra),
                             0.0, 0.0])
        hor_dir = np.array([0.0, float(target_a[1]), float(target_a[2])]) / a_hor
        if a_hor < min_mag * math.sin(max_tilt):
            a_ver_min = math.sqrt(max(0.0, min_mag ** 2 - a_hor ** 2))
        else:
            a_ver_min = math.cos(max_tilt) * min_mag
        a_ver_min = max(a_ver_min, _floor_extra)
        if a_hor < max_mag * math.sin(max_tilt):
            a_ver_max = math.sqrt(max(0.0, max_mag ** 2 - a_hor ** 2))
        else:
            a_ver_max = math.cos(max_tilt) * max_mag
        # 注意 clamp 的参数顺序：这里是 max(min(...)) 形式，等价于 _clamp
        a_ver = max(a_ver_min, min(a_ver_max, a_ver))
        a_hor = min(a_hor, a_ver * math.tan(max_tilt))
        return hor_dir * a_hor + np.array([a_ver, 0.0, 0.0])

    def fin_control(self, alt, dist, vz, vh, vmag, mass, error, vel,
                    game_dt=0.02):
        """【2026-10-09 P3】FIN 触地保持段的控制律（最简单的 PID）。

        【它取代了什么】原实现在 alt <= TARGET_ALT+1 时 break + 油门清零 +
        退出程序（kRPC 连接随 main() 的 finally 关闭）。视频证实后果是：
        引擎熄灭 -> 静止约 1 s -> 缓慢单向倾覆（属自由物理，直到操作员
        用 VesselMover 接管）。

        【本律做什么（三轴各一条最简单的 PID）】
          竖直： a_up = g0 + FIN_KP*(vz_tgt - vz) + FIN_KD*d(vz)/dt
                       其中 vz_tgt = -FIN_DESCENT_VZ（向下 2 m/s）
                 再夹到 [FIN_UP_MIN*悬停, a_ver_need] ——
                 上界 a_ver_need 保证"在剩余高度内刹得住"（会落地），
                 下界保证油门不太小（有力矩做姿态保持）。
          水平： a_h = -FIN_VH_KP*vh_vec - FIN_VH_KD*d(vh_vec)/dt
                 方向为"水平速度的反方向"（主动把残余横速消掉 ——
                 这正是既有 _term_phase 缺失的一环）。
          姿态： 瞄准【天顶】（竖直向上），即 tilt -> 0，
                 用 FIN_TILT_MAX_DEG 限幅，避免本段做出大倾角。

        【为什么姿态要瞄准天顶而不是"速度反方向"】
          本段的目标是【竖直触地且不再平移】。瞄准速度反方向会在
          vh 已很小时方向抖动（vh->0 时方向不定），且不利于把箭体立直。
          立直 + 消横速，才是不倾倒的条件。

        【返回】(target_a, target_direction, reason) —— reason 供日志/退出判定。
        """
        # ---- 竖直：目标垂速闭环（增益随高度调度的比例律 + D 阻尼）----
        #   ⚠️【为什么不能用一个固定增益的 P 环（本律第一版实测失败）】
        #     第一版写成 a_up = G0 + FIN_KP*(vz_tgt - vz)，再对 a_need 取 min。
        #     离线积分（用真实末段初值 alt=22/vz=-10.4）实测落地垂速
        #     -7.25 / -6.87 / -6.18 m/s —— 【根本没收到 -2】。
        #     原因（结构性，不是调增益能解决的）：
        #       要在剩余高度 h 内把 vz 从 -10.4 收到 -2，所需的竖直推力为
        #           a_up = g0 + (vz^2 - vz_tgt^2) / (2h)
        #       它相对 vz 的【等效增益】= vz/h。
        #       h=5 m 时约 2.1，h=0.5 m 时约 20 —— 随 h->0【无界增长】。
        #       任何【固定】增益都必然在接近地面时偏小 => 一路欠刹车
        #       => 越落越快 => 硬着陆。
        #     这与本项目此前在 _term_phase 上踩过的坑【完全同型】
        #     （当时的记录：弱 P 环给 16.5 而物理需要 20.3 => 落地 -12.2）。
        #   ⇒ 正确结构：直接用 a_need 作为指令 —— 它本身就是一条
        #     【增益 = 1/(2h) 的比例律】，比例作用已由高度调度承担。
        #     再叠一个固定增益 P 项只会重复计入、放大到过冲，故不再叠加。
        #   ⇒ D 项保留：d(vz)/dt < 0（还在加速下坠）时增加刹车；
        #     > 0（已在减速）时略减刹车，抑制垂速振荡。
        #     符号：a_up += -FIN_KD * d_vz，d_vz<0 => 增加 a_up。
        _vz_tgt = -abs(float(FIN_DESCENT_VZ))
        _dvz = float(getattr(self, '_fin_vz_prev', vz))
        _d_vz = (vz - _dvz) / max(1e-3, float(game_dt))
        self._fin_vz_prev = float(vz)
        #   a_need = "在剩余高度 h 内恰好把 vz 收到目标垂速"所需的推力。
        #   （与 track() 的 a_ver_need 同构；保证不会悬停在空中）
        _h_req = max(AVER_H_FLOOR, max(0.0, float(error[0])))
        if vz > 0.0:
            a_need = G0 - AVER_CLIMB_K * float(vz) - AVER_CLIMB_MARGIN
        else:
            a_need = (G0 + (float(vz) ** 2 - _vz_tgt ** 2) / (2.0 * _h_req))
        #   指令 = a_need + D 阻尼。上界仍夹到物理推力上限（由 apply 处理）。
        #   ⚠️ 不允许把 a_up 整体夹到 g0 以下：a_up < g0 时净竖直恒为负，
        #      永远无法减速（本项目实测该错误会让落地垂速达 -116 m/s）。
        a_up = a_need - FIN_KD * _d_vz
        # 下界：悬停的一部分（保证有力矩）。绝不超过 a_need（否则反冲）
        _hover = G0
        a_up = max(a_up, min(FIN_UP_MIN * _hover, a_need))
        # ⚠️【绝对不要在这里给 a_up 加 "<= G0*0.99" 之类的整体封顶】
        #   那是本项目已经踩过的坑：a_up < g0 时净竖直恒为负 =>
        #   永远无法减速 => 实测落地垂速达 -116 m/s（坠毁）。
        #   a_up 的真正上界是 a_need，它已保证"能在剩余高度内刹到目标垂速"。
        #   G0*0.99 那种系数只允许用在【给下界封顶】上（见上一行），
        #   绝不能用在 a_up 本身。

        # ---- 水平：P-D 消横速（方向 = 横速反方向）----
        _vhn = float(vel[1])
        _vhe = float(vel[2])
        _pn = float(getattr(self, '_fin_vh_prev_n', _vhn))
        _pe = float(getattr(self, '_fin_vh_prev_e', _vhe))
        _d_vhn = (_vhn - _pn) / max(1e-3, float(game_dt))
        _d_vhe = (_vhe - _pe) / max(1e-3, float(game_dt))
        self._fin_vh_prev_n = _vhn
        self._fin_vh_prev_e = _vhe
        a_hn = -FIN_VH_KP * _vhn - FIN_VH_KD * _d_vhn
        a_he = -FIN_VH_KP * _vhe - FIN_VH_KD * _d_vhe
        self.dbg_fin_ah = float(math.hypot(a_hn, a_he))

        # ---- 合成推力指令（x=上，y/z 水平）----
        target_a = np.array([a_up, a_hn, a_he], dtype=float)

        # ---- 姿态：瞄准天顶（tilt -> 0），并把倾角限在 FIN_TILT_MAX_DEG ----
        if FIN_TILT_HOLD:
            _up = np.array([1.0, 0.0, 0.0])
            #   【倾角取多少？】推力方向应与推力指令 target_a 一致，
            #   即 tan(tilt) = a_h / a_up。直接照此取，再夹到
            #   FIN_TILT_MAX_DEG 上限。
            #   ⚠️【为什么不能固定取满 FIN_TILT_MAX_DEG（本律第一版）】
            #     第一版写成"只要 a_h > 0 就用 8 deg"，于是即便
            #     a_h 只有 0.1（几乎不需要横向推力）也命令 8 deg。
            #     后果：姿态被无谓地支偏，落地时残余倾角反而变大
            #     （与"立直触地"的目标相反），且给姿态环制造无谓负担。
            #     正确做法是让倾角【与所需横向推力成比例】。
            _tilt_deg = 0.0
            if a_up > 1e-6 and self.dbg_fin_ah > 1e-9:
                _tilt_deg = math.degrees(math.atan2(self.dbg_fin_ah, a_up))
                _tilt_deg = min(_tilt_deg, float(FIN_TILT_MAX_DEG))
            if _tilt_deg > 1e-9:
                _h = np.array([0.0, a_hn, a_he]) / self.dbg_fin_ah
                _t = math.radians(_tilt_deg)
                target_direction = (math.cos(_t) * _up
                                    + math.sin(_t) * _h)
            else:
                target_direction = _up
        else:
            target_direction = None

        # ---- 记录（供日志列与退出判定）----
        self.tilt_cmd = math.degrees(math.acos(
            _clamp(float(target_direction[0]), 1.0, -1.0)))
        self.dbg_fin_a_up = float(a_up)
        self.dbg_fin_need = float(a_need)
        return target_a, target_direction

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
        a_cap = self.a_cap_real()
        throttle = mag / a_cap if a_cap > 1e-9 else 0.0
        throttle = _clamp(throttle, 1.0, 0.0)
        self.v.control.throttle = throttle
        self.thr_cmd = throttle
        # 【2026-09-29】用本帧的 (throttle, Vessel.thrust) 更新推力自校准系数。
        #   必须在 a_cap 被下一次使用前更新；这里紧跟写入之后最合适。
        self.update_thrust_scale(throttle)
        # 【2026-09-28】记住本帧的 gfold 段油门下限（= 悬停的比例），
        #   供诊断列 atq_floor 判断"油门是否被下限钉住"。
        # 【2026-09-29 第三步 3(a)】改用【实际生效】的下限（track() 里
        #   按"是否要下降"算出的那个），否则诊断列会在下限被放开时
        #   仍显示旧值，无法反映真实情况。
        try:
            self.thr_floor_last = float(
                getattr(self, 'min_mag_eff', 0.0)) / max(1e-9, a_cap)
        except Exception:                            # noqa: BLE001
            self.thr_floor_last = float('nan')

        # ---- 姿态（原版 line 461-477）----
        if target_direction is None:
            # 【2026-09-29 第六步 6(b)】兜底路径也要限幅，否则一旦调用方
            #   忘记传方向，限幅就被绕过。依据见 MAX_TILT_CMD_DEG。
            target_direction = limit_tilt_dir(target_a / max(1e-9, mag),
                                              MAX_TILT_CMD_DEG)
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
            # ============================================================
            # 【2026-09-29 第七步 7(b)：姿态三项改动（依据见文件头 7(b) 段）】
            # ------------------------------------------------------------
            # ① 增益随权限调度：kp 正比于 alpha_max，使 zeta 与油门无关。
            #    实测 alpha_max 随油门变化 8 倍，kp 固定时 zeta 摆 2.8 倍。
            # ② D 项改用【实测机身角速度】。实测原 D 输入与角速度的 R^2 仅
            #    0.046（即 95% 是噪声/病态量），换成角速度后饱和帧 11 -> 0。
            # ③ 参考倾角做斜率限制（在 track() 里实现，这里只消费其结果）。
            #    u = kp*e + kd*d(e)/dt 在 e=ref-theta 时【已含前馈】
            #    (+kd*dref/dt) 与阻尼 (-kd*omega)，故无需另加前馈项。
            # ============================================================
            _a_sched = self._att_sched_factor()
            _kp_x = CTRL_X_ROT_KP * _a_sched
            _kp_z = CTRL_Z_ROT_KP * _a_sched
            self.ctrl_x_rot.kp = _kp_x
            self.ctrl_z_rot.kp = _kp_z
            self.dbg_sched = float(_a_sched)

            # 【D 项改用真实角速度】avel_local[0]=俯仰, [2]=偏航 [rad/s]。
            # ============================================================
            # 【2026-09-29 第七步 7(b)-② 符号修正 —— 我第一版写反了】
            # ------------------------------------------------------------
            # 【事故】第一版写的是 rate = -omega。实飞
            #   gfold_log_20260929_212056.csv 证明它把 D 项变成了【正反馈】：
            #     avel_p 从 12 deg/s 冲到 205 deg/s，avel_y 冲到 175 deg/s，
            #     tilt_act 148.3 deg，27 帧 >90 deg，落地 vz=-92.2 vh=67.4，
            #     落点偏差 428.6 m。比改动前严重得多。
            #
            # 【为什么写反了】我以为 d(err)/dt = d(ref)/dt - omega，所以
            #   "与 d(err)/dt 同号"应取 -omega。但【实测】稳定那趟
            #   （gfold_log_20260929_195847.csv）的回归是：
            #       ch x : d(err_x)/dt = +0.782 * avel_p   (corr +0.215)
            #       ch z : d(err_z)/dt = +0.724 * avel_y   (corr +0.447)
            #   即【实测符号是 +omega，不是 -omega】。
            #   该符号关系在 212056 那趟更是精确到 -1.000（因为那一趟就是
            #   用 -avel 直接喂进去的，diff = -avel_p，回归自然得到 -1）。
            #
            # 【被控对象符号（212056 实测）】out_x 长时间饱和时的角加速度：
            #     out_x = +1.00 持续 12 帧 -> d(avel_p)/dt = -0.309 rad/s^2
            #     out_x = -1.00 持续 47 帧 -> d(avel_p)/dt = +0.888 rad/s^2
            #   => d(avel_p)/dt = -A*u  (A>0)，即正指令使角速度【变负】。
            #
            # 【因此正确的阻尼项】applied = err*kp + diff*kd。
            #   要阻尼就必须让 diff 对 avel 的系数为正：
            #       avel>0 -> 需要 d(avel)/dt<0 -> 需要 u>0 -> 需要 +avel 项
            #       avel<0 -> 需要 d(avel)/dt>0 -> 需要 u<0 -> 同样 +avel 项
            #   => diff = +avel。第一版取 -avel 恰好是【反阻尼】。
            #
            # 【与旧律的一致性核对】旧律 diff=d(err)/dt 的实测等效系数是
            #   +0.782（ch x），故旧律等效阻尼增益 = kd*0.782 = 1.955，
            #   而本改动给出 kd*1.0 = 2.500 —— 【同号、略强】。
            #   这解释了为什么旧律是稳定的、而我的 -avel 立刻发散。
            # ============================================================
            if RATE_D_ENABLE:
                _rate_x = float(avel_local[0]) * RATE_D_GAIN
                _rate_y = float(avel_local[2]) * RATE_D_GAIN
            else:
                # 回退到旧律：对误差做差分（PID.update 内部自行计算）
                _rate_x = None
                _rate_y = None

            # ============================================================
            # 【8(a) 参考方向速率前馈】
            #   把"参考自己在这一帧动了多少"折算成误差的变化率，
            #   再【加】到 D 输入上 —— 这正是旧律 d(err)/dt 里被删掉的那部分。
            #   符号不靠推理：用同一个 _angle_around_axis 对参考做扰动得到。
            # ============================================================
            _ff_x = 0.0
            _ff_y = 0.0
            if FF_ENABLE and _rate_x is not None and game_dt > 1e-9:
                try:
                    _wprev = getattr(self, '_ff_dir_prev', None)
                    _wnow = np.array(target_direction, dtype=float)
                    if _wprev is not None:
                        # 参考方向在本帧的位移（地面/目标系）
                        _dw = (_wnow - _wprev) / float(game_dt)
                        # 转到机体系（与 err 同坐标系）
                        _dw_local = _transform(_dw, rot_s2l)
                        # 让"参考动一点、机身不动"，用同一个函数算新误差
                        _pert = _normalize(tgt_local + _dw_local * float(game_dt))
                        _ex2 = _angle_around_axis(
                            _pert, np.array([0.0, 1.0, 0.0]), np.array([1.0, 0.0, 0.0]))
                        _ey2 = _angle_around_axis(
                            _pert, np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0]))
                        # 参考对误差的贡献率 [rad/s]
                        _ff_x = (_ex2 - err_x) / float(game_dt) * FF_GAIN
                        _ff_y = (_ey2 - err_z) / float(game_dt) * FF_GAIN
                        # 限幅：前馈只做"提前量"，不夺走全部控制权限
                        _cl = FF_CLAMP_RAD
                        _ff_x = max(-_cl, min(_cl, _ff_x))
                        _ff_y = max(-_cl, min(_cl, _ff_y))
                    self._ff_dir_prev = _wnow.copy()
                except Exception:                    # noqa: BLE001
                    _ff_x = 0.0
                    _ff_y = 0.0
            self.dbg_ff_x = _ff_x
            self.dbg_ff_y = _ff_y
            # 【类型说明（修 Pylance reportOptionalOperand）】
            #   _rate_x/_rate_y 的类型是 float|None（RATE_D_ENABLE=False 时为
            #   None，表示回退到旧律、由 PID.update 自己差分）。
            #   这里两个都要判非 None 才能相加：pyright 只会对
            #   【被判断过的那个变量】做类型收窄，所以必须两个都写进去，
            #   否则 _rate_y 仍被当成 Optional 而报错（实测正是这一条）。
            if _rate_x is not None and _rate_y is not None:
                _rate_x = _rate_x + _ff_x
                _rate_y = _rate_y + _ff_y
            self.dbg_rate_x = _rate_x
            self.dbg_rate_y = _rate_y

            out_x = self.ctrl_x_rot.update(err_x, game_dt, rate=_rate_x)
            out_z = self.ctrl_z_rot.update(err_z, game_dt, rate=_rate_y)

            # 记录 p/i/d 分解（PID.update 里 result = p+i+d，已带符号）
            self.dbg_pid_px = float(-err_x * _kp_x)
            self.dbg_pid_dx = float(-self.ctrl_x_rot.diff * CTRL_X_ROT_KD)
            self.dbg_pid_ix = float(-self.ctrl_x_rot.integral)
            #   【2026-10-11 加日志】偏航三项。
            #   此前只有俯仰(_x)被记录，偏航(_z)三项【从未定义】。
            #   而实飞 satZ 达 20~33%、satX 恒为 0 ==> 饱和全在偏航，
            #   偏航的 P/D/误差 构成却完全看不见，无法对症下药。
            #   【口径】与俯仰完全一致（都是 -项，符号已含在 PID 约定里），
            #   kp 用本帧实际生效的 _kp_z（经 _att_sched_factor 调度）。
            self.dbg_pid_pz = float(-err_z * _kp_z)
            self.dbg_pid_dz = float(-self.ctrl_z_rot.diff * CTRL_Z_ROT_KD)
            self.dbg_pid_iz = float(-self.ctrl_z_rot.integral)
            cp_ = -_clamp(out_x, 1, -1)
            cy_ = -_clamp(out_z, 1, -1)
            # ============================================================
            # 【2026-09-28 新增：俯仰/偏航的【机身角速度阻尼】】
            # ------------------------------------------------------------
            # 【为什么必须加】上面 PID 的 D 项微分的是【误差信号】，不是
            #   【机身角速度】。当误差已经很大（实测 |pid_err_z| 中位
            #   29.2 deg）、输出饱和时，误差变化率不再反映转速，
            #   D 项看不见"载具正在以 85 deg/s 旋转"。
            #   而角速度不清零，姿态就必然过冲、进入极限环。
            #
            # 【为什么滚转早就有】参考仓库的 control_roll 直接读
            #   avel_local[1]（见 demo3_gfold.py:473）。俯仰/偏航没有，
            #   这是不对称的 —— 本次补上。
            #
            # 【轴向索引的依据（已核实）】
            #   · 滚转通道用 avel_local[1]，与 roll 轴对应；
            #   · 参考仓库注释掉的 469/470 行把 pitch 对应 avel_local[0]、
            #     yaw 对应 avel_local[2]（那两行把角速度误当 dt 传参，
            #     本身是坏实验，但【索引对应关系】可作旁证）；
            #   · 实测日志交叉验证：pid_out_x 饱和时 avel_p([0]) 变化，
            #     pid_out_z 饱和时 avel_y([2]) 变化，avel_r([1]) 恒 0
            #     （滚转无指令）。
            #   ⇒ [0]=俯仰角速度, [1]=滚转角速度, [2]=偏航角速度。
            #
            # 【符号】与 control_pitch/yaw 的约定一致（都取负），
            #   即"角速度为正时施加反向力矩"——这就是阻尼。
            #   实际符号正确性由实飞日志的 pid_damp_* 列与姿态收敛性判定。
            # ============================================================
            # ============================================================
            # 【2026-09-28 撤回：俯仰/偏航的机身角速度阻尼【已被实飞证伪】】
            # ------------------------------------------------------------
            # 曾在此加入 _damp_x/_damp_z = avel * CTRL_X/Z_AVEL_KD 并从
            # cp_/cy_ 中减去。实飞 gfold_log_20260928_214050.csv 证明【有害】：
            #
            # 【为什么会错】滚转通道写 rate->0 是对的，因为【滚转没有指令
            #   转动】；但俯仰/偏航【有指令转动】（tilt_cmd 会在 25 -> 1.8
            #   之间跳变，载具必须跟着转）。照抄滚转公式，等于让阻尼项
            #   【同时对抗合法跟踪所需的转动】。
            #
            # 【实飞数据】在 |pid_err_z| > 25 deg 的帧里：
            #     阻尼【削减】了权限的：45 帧
            #     阻尼未削减权限的  ：18 帧
            #   例：t=9.18 err=34.1 out=0.58 damp=0.442
            #         -> P+D 本应给 1.02，被削到 0.58
            #       t=12.84 err=-32.0 out=-1.00 damp=-0.039
            #         -> 阻尼反向，把输出推得更负
            #
            # 【后果】姿态跟不上 -> 误差更大 -> 又被阻尼削弱 -> 越追越慢：
            #     飞行时长 20.5 -> 24.3 s
            #     avel 最大 85.8 -> 158.0 deg/s
            #     att_err 最大 73.2 -> 149.8 deg
            #     落地 tilt 94.7 -> 131.6 deg
            #   匹配高度重比后发现低空（alt<300）明显恶化，而那正是最要命的段。
            #
            # 【注意】同一轮把 KP 5.0 -> 2.0 是【有效】的：
            #     pid_sat_z 饱和 73 -> 31 帧，早期(t<8) avel 28.4 -> 10.4。
            #   但降 KP 的收益被这个阻尼污染了，无法单独评估，故本轮【只撤
            #   阻尼、保留 KP=2.0】，一次只改一个变量。
            #
            # 【正确做法（未实现）】若要阻尼，只能阻尼【超调部分】，即
            #   (avel - 指令转动速率)，这需要引入指令速率前馈。先不做。
            # ============================================================
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
                # 【2026-09-28 新增：姿态失守诊断（候选 1/2/3）】
                #   详见 Logger.COLS 第 (10) 段的完整说明。这里只做采集。
                # ============================================================
                # --- 候选 1：把姿态权限与【当帧真实油门】对齐 ---
                #   alpha_max 随油门变化（实测 5.4~81.9，差 15 倍），
                #   而 PID 增益是常数。过去两列分开记录，无法判断
                #   "权限低"到底是不是"油门低"造成的。
                self.dbg_atq_thr = float(throttle)
                # 【2026-09-29 第四步 4(b)：判据改为"下限是否真的抬高了竖直分量"】
                #   【旧判据的两个毛病】（实测 173800）
                #     ① 下限放开到 0 时，若指令本身也是 0，则
                #        throttle == floor == 0 => 误报"被下限钉住"。
                #        该趟 36 个指令全零帧的 atq_floor 全是 1.0。
                #     ② 它比的是【油门】，而下限作用在【加速度】上
                #        （a_ver = max(a_ver_min, ...)），两者之间还隔着
                #        a_cap 与方向分摊，本来就不是同一个量纲。
                #   新判据直接量"夹限把竖直分量抬高了多少"：
                #        floor_lift = a_ver_after - a_ver_before  (>0 即被抬高)
                #   注意用【很小的容差】判 >0：a_ver_min 是 sqrt(...) 算出来的，
                #   与浮点比较要用 1e-9 而不是 == 0。
                #
                # 【必须再要求输出非零 —— 否则判据仍会误报】
                #   只用 lift>0 是不够的：当下限=0 且 raw_a_ver<0 时，
                #   a_ver 被从 -1.25 "抬"到 0.0，lift=1.25>0 —— 但它其实
                #   【什么也没做】（输出仍是零推力）。实测这个反例：
                #       release=0.00（旧 bug）: lift 判据报 35 帧"生效"，
                #         而这 35 帧里 30 帧的输出是 0（纯自由落体）
                #       release=0.05（修复后）: 报 42 帧"生效"，输出全部 >0
                #   ⇒ 加上 a_ver_out > 0 之后，前者归 0、后者保持 42，
                #     判据才真正反映"下限在托着载具"。
                _lift = float(getattr(self, 'dbg_floor_lift', 0.0) or 0.0)
                _outv = float(getattr(self, 'dbg_a_ver_out', 0.0) or 0.0)
                self.dbg_atq_floor = 1.0 if (_lift > 1e-9 and _outv > 1e-9) else 0.0
                # --- 候选 2：两轴饱和（大倾角下俯仰/偏航可能不再解耦）---
                _sx = 1.0 if abs(float(cp_)) > 0.999 else 0.0
                _sz = 1.0 if abs(float(cy_)) > 0.999 else 0.0
                self.dbg_pid_sat_x = _sx
                self.dbg_pid_sat_z = _sz
                if _sx > 0.5 and _sz > 0.5:
                    self._sat_run = int(getattr(self, '_sat_run', 0)) + 1
                else:
                    self._sat_run = 0
                self.dbg_pid_sat_run = float(self._sat_run)
                # --- 候选 3：机头方位角 vs 指令方位角 ---
                #   目标系 [1]=北, [2]=东（见 state() 注释）。
                #   atan2(east, north) 给出方位角：0=正北, 90=正东。
                #   只需水平分量，故先归一化水平投影。
                def _az(vec_n, vec_e):
                    if abs(vec_n) < 1e-9 and abs(vec_e) < 1e-9:
                        return float('nan')
                    return math.degrees(math.atan2(vec_e, vec_n))
                _nn, _ne = float(nose_srf[1]), float(nose_srf[2])
                _tn, _te = float(target_direction[1]), float(target_direction[2])
                self.dbg_nose_n = _nn
                self.dbg_nose_e = _ne
                self.dbg_nose_az = _az(_nn, _ne)
                self.dbg_tgt_az = _az(_tn, _te)
                if (self.dbg_nose_az == self.dbg_nose_az
                        and self.dbg_tgt_az == self.dbg_tgt_az):
                    _d = self.dbg_nose_az - self.dbg_tgt_az
                    # 折到 [-180,180]
                    _d = ((_d + 180.0) % 360.0) - 180.0
                    self.dbg_az_err = _d
                else:
                    self.dbg_az_err = float('nan')
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

    def _read_actuator(self):
        """读【真实执行器状态】：推力与等效油门。

        【为什么不用 control.throttle】那是 kRPC 的写入回读缓存，
          实测 log ..._154409 有 9/117 帧读到 0.0000 的假值，
          会被误判成【执行器没跟上】。而 Vessel.thrust 是
          SpaceCenter.Engine.thrust 的实测求和，可信。

        【实现】只读 thrust 与 available_thrust 两次轻量 RPC；
          异常时保留上一帧值（写 nan 会让日志出现空洞）。
        """
        try:
            _th = float(self.v.thrust)
            _tmax = float(self.v.available_thrust)
            self.dbg_thrust_n = _th
            if _tmax > 1e-6:
                # 【2026-09-28】thr_real = 实测推力 / 【真实可用推力】
                #   分母用 available_thrust，与控制器的 a_cap 同口径。
                #   这样 thr_real 应该≈ thr_cmd（而不是之前的 1.5 倍）。
                self.dbg_thr_real = _th / _tmax
        except Exception:                            # noqa: BLE001
            pass

    def _dbg_cols(self):
        """产出诊断列（键名与 Logger.COLS 一一对应）。

        两个日志点（hold / gfold 段）都要写这些列，抽成一个方法避免两处漂移
        （列名与顺序由 Logger.COLS 强制检查）。NaN 写空串，便于离线脚本
        用 float() 失败即跳过。
        """
        def _f(v, nd=3):
            """float 安全格式化：NaN -> ''（CSV 留空，不写 nan）。"""
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
            """取【机体系角速度】的第 idx 个分量。"""
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
            idx_raw=_f(getattr(self, 'dbg_idx_raw', None), 2),
            idx_frozen=_f(getattr(self, 'dbg_idx_frozen', None), 0),
            he_n=_f(getattr(self, 'dbg_he_n', None), 2),
            he_e=_f(getattr(self, 'dbg_he_e', None), 2),
            hd_n=_f(getattr(self, 'dbg_hd_n', None), 3),
            hd_e=_f(getattr(self, 'dbg_hd_e', None), 3),
            h_misalign=_f(getattr(self, 'dbg_h_misalign', None), 1),
            # ---- 【新增】矢量误差分量 ----
            dp_n=_f(getattr(self, 'dbg_dp_n', None), 2),
            dp_e=_f(getattr(self, 'dbg_dp_e', None), 2),
            dve_n=_f(getattr(self, 'dbg_dve_n', None), 3),
            dve_e=_f(getattr(self, 'dbg_dve_e', None), 3),
            thrust_n=_f(getattr(self, 'dbg_thrust_n', None), 0),
            thr_real=_f(getattr(self, 'dbg_thr_real', None), 4),
            atq_thr=_f(getattr(self, 'dbg_atq_thr', None), 4),
            atq_floor=_f(getattr(self, 'dbg_atq_floor', None), 0),
            t_scale=_f(getattr(self, '_tscale', None), 4),
            pid_sat_x=_f(getattr(self, 'dbg_pid_sat_x', None), 0),
            pid_sat_z=_f(getattr(self, 'dbg_pid_sat_z', None), 0),
            pid_sat_run=_f(getattr(self, 'dbg_pid_sat_run', None), 0),
            nose_n=_f(getattr(self, 'dbg_nose_n', None), 3),
            nose_e=_f(getattr(self, 'dbg_nose_e', None), 3),
            nose_az=_f(getattr(self, 'dbg_nose_az', None), 1),
            tgt_az=_f(getattr(self, 'dbg_tgt_az', None), 1),
            az_err=_f(getattr(self, 'dbg_az_err', None), 1),
            plan_alt=_f(self.dbg_plan_alt, 2),
            plan_vz=_f(self.dbg_plan_vz, 3),
            plan_vh=_f(self.dbg_plan_vh, 3),
            plan_tilt_j=_f(getattr(self, 'dbg_plan_tilt_j', None), 2),
            plan_tf_used=_f(getattr(self, 'dbg_plan_tf_used', None), 2),
            plan_nj=_f(getattr(self, 'dbg_plan_nj', None), 2),
            vdesc=_f(getattr(self, 'dbg_vdesc', None), 1),
            trk_pos=_f(self.dbg_trk_pos, 2),
            trk_pos_up=_f(self.dbg_trk_pos_up, 2),
            trk_pos_h=_f(self.dbg_trk_pos_h, 2),
            trk_vel=_f(self.dbg_trk_vel, 3),
            loop_dt=_f(getattr(self, 'dbg_loop_dt', None), 4),
            rpc_ms=_f(getattr(self, 'dbg_rpc_ms', None), 1),
            ctrl_ms=_f(getattr(self, 'dbg_seg_ctrl', None), 1),
            tau_p=_f(getattr(self, 'dbg_tau_p', None), 0),
            tau_r=_f(getattr(self, 'dbg_tau_r', None), 0),
            tau_y=_f(getattr(self, 'dbg_tau_y', None), 0),
            I_p=_f(getattr(self, 'dbg_I_p', None), 0),
            I_r=_f(getattr(self, 'dbg_I_r', None), 0),
            I_y=_f(getattr(self, 'dbg_I_y', None), 0),
            alpha_max=_f(getattr(self, 'dbg_alpha_max', None), 2),
            avel_mag=_f(getattr(self, 'dbg_avel_mag', None), 2),
            tilt_rate=_f(getattr(self, 'dbg_tilt_rate', None), 2),
            tilt_cmd_rate=_f(
                getattr(self, 'dbg_tilt_cmd_rate', None), 2),
            tilt_act_rate=_f(
                getattr(self, 'dbg_tilt_act_rate', None), 2),
            pid_err_x=_f(getattr(self, 'dbg_pid_err_x', None), 3),
            pid_err_z=_f(getattr(self, 'dbg_pid_err_z', None), 3),
            pid_px=_f(getattr(self, 'dbg_pid_px', None), 4),
            pid_dx=_f(getattr(self, 'dbg_pid_dx', None), 4),
            pid_ix=_f(getattr(self, 'dbg_pid_ix', None), 4),
            pid_pz=_f(getattr(self, 'dbg_pid_pz', None), 4),
            pid_dz=_f(getattr(self, 'dbg_pid_dz', None), 4),
            pid_iz=_f(getattr(self, 'dbg_pid_iz', None), 4),
            pid_out_x=_f(getattr(self, 'dbg_pid_out_x', None), 3),
            pid_out_z=_f(getattr(self, 'dbg_pid_out_z', None), 3),
            pid_out_r=_f(getattr(self, 'dbg_pid_out_r', None), 3),
            avel_p=_av_c(0),
            avel_r=_av_c(1),
            avel_y=_av_c(2),
            sched=_f(getattr(self, 'dbg_sched', None), 3),
            rate_x=_f(getattr(self, 'dbg_rate_x', None), 4),
            rate_y=_f(getattr(self, 'dbg_rate_y', None), 4),
            ref_slew=_f(getattr(self, 'dbg_ref_slew', None), 1),
            ref_clip=_f(getattr(self, 'dbg_ref_clamped', None), 1),
            ff_x=_f(getattr(self, 'dbg_ff_x', None), 4),
            ff_y=_f(getattr(self, 'dbg_ff_y', None), 4),
            u_h=_f(getattr(self, 'dbg_u_h', None), 3),
            pdv_h=_f(getattr(self, 'dbg_pdv_h', None), 3),
            pdp_h=_f(getattr(self, 'dbg_pdp_h', None), 3),
            u_dot=_f(getattr(self, 'dbg_u_dot', None), 3),
            pdv_dot=_f(getattr(self, 'dbg_pdv_dot', None), 3),
            pdp_dot=_f(getattr(self, 'dbg_pdp_dot', None), 3),
            cmd_dot=_f(getattr(self, 'dbg_cmd_dot', None), 3),
            tf_plan=_f(getattr(self, 'dbg_tf_tilt_plan', None), 2),
            tf_corr=_f(getattr(self, 'dbg_tf_tilt_corr', None), 2),
            tf_want=_f(getattr(self, 'dbg_tf_tilt_want', None), 2),
            tf_cmd=_f(getattr(self, 'dbg_tf_tilt_cmd', None), 2),
            tf_perr=_f(getattr(self, 'dbg_tf_perr', None), 2),
            tf_verr=_f(getattr(self, 'dbg_tf_verr', None), 3),
            tf_az_n=_f(getattr(self, 'dbg_tf_az_n', None), 4),
            tf_az_e=_f(getattr(self, 'dbg_tf_az_e', None), 4),
            tf_taper=_f(getattr(self, 'dbg_tf_taper', None), 3),
            tf_arrived=_f(getattr(self, 'dbg_tf_arrived', None), 0),
            tf_cap=_f(getattr(self, 'dbg_tf_cap', None), 2),
            tf_slew_allow=_f(getattr(self, 'dbg_tf_slew_allow', None), 2),
            tf_lead_n=_f(getattr(self, 'dbg_tf_lead_n', None), 2),
            tf_ahdir_cos=_f(getattr(self, 'dbg_tf_ahdir_cos', None), 3),
            ca_T=_f(getattr(self, 'dbg_ca_T', None), 3),
            ca_kv=_f(getattr(self, 'dbg_ca_kv', None), 4),
            ca_kp=_f(getattr(self, 'dbg_ca_kp', None), 4),
            ca_pc=_f(getattr(self, 'dbg_ca_posclip', None), 0),
            # ---- 【新增】内环（加加速度限幅）----
            ah_dem=_f(getattr(self, 'dbg_ah_dem', None), 3),
            ah_cmd=_f(getattr(self, 'dbg_ah_cmd', None), 3),
            ah_sat=_f(getattr(self, 'dbg_ah_sat', None), 0),
            ah_cap_bind=_f(getattr(self, 'dbg_ah_cap_bind', None), 0),
            j_max=_f(getattr(self, 'dbg_j_max', None), 1),
            j_phys=_f(getattr(self, 'dbg_j_phys', None), 1),
            j_lim=_f(getattr(self, 'dbg_j_lim', None), 1),

            j_used=_f(getattr(self, 'dbg_j_used', None), 1),
            av_plan=_f(getattr(self, 'dbg_av_plan', None), 3),
            av_need=_f(getattr(self, 'dbg_av_need', None), 3),
            av_up=_f(getattr(self, 'dbg_av_up', None), 3),
            av_net=_f(getattr(self, 'dbg_av_net', None), 3),
            # ---- 【新增】竖直串扰追踪 ----
            av_in_up=_f(getattr(self, 'dbg_av_in_up', None), 3),
            av_out_up=_f(getattr(self, 'dbg_av_out_up', None), 3),
            av_floor=_f(getattr(self, 'dbg_av_floor', None), 3),
            min_mag_eff=_f(getattr(self, 'dbg_min_mag_eff', None), 3),
            draw_on=_f(getattr(self, 'dbg_draw_on', None), 0),
            # ---- 【2026-10-09 P3】FIN 触地保持段 ----
            #   fin_on: 本帧是否处于 FIN 段（1/0）
            fin_on=_f(1.0 if getattr(self, 'nav_mode', '') == 'fin'
                      else 0.0, 0),
            fin_a_up=_f(getattr(self, 'dbg_fin_a_up', None), 3),
            fin_need=_f(getattr(self, 'dbg_fin_need', None), 3),
            fin_ah=_f(getattr(self, 'dbg_fin_ah', None), 3),
            fin_dp_n=_f(getattr(self, 'dbg_fin_dp_n', None), 2),
            fin_dp_e=_f(getattr(self, 'dbg_fin_dp_e', None), 2),
            fin_dve_n=_f(getattr(self, 'dbg_fin_dve_n', None), 2),
            fin_dve_e=_f(getattr(self, 'dbg_fin_dve_e', None), 2),
            post_sas=_f(getattr(self, 'dbg_post_sas', None), 0),
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
            self.idx_frozen_n = 0
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
                              f"avail_thrust={d['thrust']:.0f}N "
                              f"a_net={d['a_net']:.1f}")
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
                a_avail = self.a_cap_real(mass)
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
            # ⑪ 分段计时起点（2026-09-29 第六步 6(a)）
            _t_seg0 = time.time()

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

            _t_state = time.time()
            alt, dist, vz, vh, vmag, mass, tp, tv = self.state()
            _t_state_done = time.time()
            tt = time.time() - t0
            self.dbg_rpc_ms = (time.time() - _t_rpc) * 1000.0
            # ⑪ 分段：seg_rpc=上一帧结束→本节开始；seg_auth=权限采样；
            #    seg_state=self.state()。seg_ctrl/seg_draw 在 draw() 之后填。
            self.dbg_seg_rpc = (_t_rpc - _t_seg0) * 1000.0
            self.dbg_seg_auth = (_t_state - _t_rpc) * 1000.0
            self.dbg_seg_state = (_t_state_done - _t_state) * 1000.0
            # actuator sample: real thrust, not the throttl readback
            if not self.args.dry_run:
                self._read_actuator()

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

            # ================================================================
            # 【2026-09-28 重大修复：game_prev_time 从未推进】
            # ----------------------------------------------------------------
            # 【症状】实飞日志 gfold_log_20260928_145755.csv 的 loop_dt 列
            #   从 0.08 一路涨到 15.08，且【几乎等于 t】：
            #       t= 1.17  loop_dt= 1.06
            #       t= 5.04  loop_dt= 4.58
            #       t=15.73  loop_dt=15.08
            #   而真实帧间隔（相邻日志行的 t 差）只有 0.119~0.288 s（均值 0.190）。
            #   ⇒ loop_dt 不是【帧间隔】，而是【自循环开始累计的游戏时间】，
            #     平均偏大 40 倍，最大偏大 79 倍。
            #
            # 【根因】主循环开头（2042 行附近）
            #       game_prev_time = self.sc.ut          # 只在循环【之前】赋一次
            #       while True:
            #           ut = self.sc.ut
            #           game_dt = ut - game_prev_time    # 因此恒等于累计时间
            #   循环体内【没有】game_prev_time = ut 这一句。
            #
            # 【参考仓库有这一句】dev/gfold/demo3_gfold.py:485
            #       prev_vel = vel
            #       game_prev_time = ut          <-- 就在循环末尾
            #   即【移植时漏抄了一行】，不是设计如此。
            #
            # 【影响面（game_dt 的全部消费者）】
            #   ① PID.update(err, game_dt)（1553/1559 行）
            #        D 项 = kd * d(err)/dt，dt 偏大 40~79 倍
            #        ⇒ D 项被压小同样倍数 ⇒ 姿态回路【退化成纯 P】。
            #        实测 |pid_px|/|pid_dx| = 85:1，与这个推断一致。
            #        I 项 = error*dt*ki，但 ki=0.0，暂不受影响。
            #   ② tilt_rate = d(tilt_act)/game_dt（1599-1601 行）
            #        实测最大只有 2.58 deg/s，真实应有数十 deg/s。
            #   ③ acc = (vel - prev_vel)/game_dt（2168-2169 行）
            #        外推用的加速度被压小同样倍数。
            #   ④ self.last_pkt = game_dt（2050 行）→ track() 的 dt_game，
            #        影响索引推进的衰减项。
            #   ⇒ 这是【姿态始终不稳定的直接原因】：反馈回路里唯一的阻尼
            #     通道被时间标度错误压死，只剩 P 项，必然极限环振荡。
            #
            # 【修法】照抄参考仓库：在循环体【末尾】推进 game_prev_time。
            #   放在循环末尾（而不是开头）的原因：所有 continue 分支
            #   （2156 / 2328 行）都会跳过本句，于是 game_dt 正确地累积
            #   【自上次真正计算以来】经过的游戏时间 —— 这正是
            #   game_dt < 0.01 时 continue 想要的语义（还没过一个物理帧
            #   就不算）。参考仓库同样是 continue 在 339 行、赋值在 485 行。
            # ================================================================
            game_prev_time = ut

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
                    a_cap = self.a_cap_real(mass)
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
                    # ⑪ hold 段耗时起点（见下方 continue 前的两列赋值）
                    _t_hold = time.time()
                    hold_dir = hold / max(1e-9, float(np.linalg.norm(hold)))
                    # 【2026-09-29 第六步 6(b)】hold 段同样是姿态指令，
                    #   一样加 30 deg 硬限幅（依据见 MAX_TILT_CMD_DEG）。
                    #   注意这【不改变 hold 的推力幅值】——apply() 的节流
                    #   来自第一个参数（hold），方向来自第二个参数。
                    hold_dir = limit_tilt_dir(hold_dir, MAX_TILT_CMD_DEG)
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
                    # ⑪ hold 帧同样补齐 seg_ctrl/seg_draw，否则这两列会
                    #   残留【上一帧 gfold 段】的值，被误读成 hold 帧的耗时。
                    #   hold 段的控制工作在 apply() 里（上面已调用）。
                    self.dbg_seg_ctrl = (_t_hold - _t_state_done) * 1000.0
                    self.dbg_seg_draw = (time.time() - _t_hold) * 1000.0
                    continue

            # ============================================================
            # 【2026-09-29 第三步 3(b)：final 分支【整体删除】】
            # ------------------------------------------------------------
            # 【原来是什么】进入目标圆柱区（dist<FINAL_RADIUS=20 且
            #   h<FINAL_HEIGHT=200）后切到另一套控制律：
            #       · 自杀点火油门  thr = lerp(0.8, 1.0, frac)
            #       · 水平 P/D      ctrl_hor = -e*0.03 - v*0.06
            #       · 固定 25 度锥限幅
            #
            # 【为什么要删（三条实测证据）】
            #   ① frac 在 vz->0 时【奇异】：
            #        denom = est_h - est_h_low ∝ vz²
            #      vz 从 -40 到 -0.5 时 denom 缩了 6400 倍，frac 从 -27.5
            #      跑到 -2.3e5。用真实日志 003035 逐帧复算，油门在 5 帧
            #      （0.8 s）内从 0.12 扫到 1.00 —— 这就是【末端突然大推力】。
            #   ② 整个 gfold 段有【两套完全不同的律】在切换，交界处必然
            #      阶跃。实测日志 003035 帧 53->54：thr_cmd 0.440 -> 0.120，
            #      同时 gnc_phase 冻在 64.69 不再前进。
            #   ③ 本次（第二步）给 P4 加了下降率包络后，参考轨迹【本身】
            #      就是一条从巡航平滑收到触地的剖面，末端不再需要另一套
            #      律来兜底 —— 让 PD 一直跟到地面即可。
            #
            # 【删除后的行为】全程只有一条律（track() 的 PD 跟随 +
            #   条件下限 + 解析锥限幅），gfold 段天然连续。
            # 【保留的常量】FINAL_* / CONIC_TILT_DEG 已无引用，但它们
            #   在文件头仍有说明价值；见各自定义处的标注。
            # ============================================================
            # 原版 line 361-405（【唯一剩下的一条律】）
            assert self.plan is not None      # hold 分支已保证
            plan_x = self.plan['x']
            # ============================================================
            # 【2026-10-09 P3】FIN 触地保持段 —— 【在 track() 之前】决定
            # ------------------------------------------------------------
            # 【为什么必须放在这里（而不是 track() 之后）】
            #   FIN 段是一条【独立的律】，它取代 gfold 跟踪律。
            #   若放在 track()+apply() 之后，同一帧会先按 gfold 写一次
            #   执行器、再按 FIN 写一次（双写），且 track() 的
            #   _term_phase / 锥限幅会与 FIN 打架。
            #   ⇒ 在这里二选一：FIN 段走 fin_control()，其余走 track()。
            #   两者共用下面的【公共日志行】与退出判定，日志口径一致。
            #
            # 【旧行为（已废除）】原来是：
            #     if alt <= TARGET_ALT + 1.0 or situation == 'landed':
            #         break + throttle=0 + return（main() finally 关 kRPC 连接）
            #   即姿态与油门【全部撒手】。视频证据：引擎 ~0.1 s 内熄灭，
            #   静止约 1 s 后【缓慢单向倾覆】。
            #   日志证据：末帧仍有 vh=1.10~3.77 m/s、tilt=5.5~6.0 deg，
            #   而既有 _term_phase 明确"水平不动"→ 横速没被消掉。
            # ============================================================
            _in_fin = (FIN_ENABLE and self.nav_mode == 'fin') or (
                FIN_ENABLE and alt <= TARGET_ALT + float(FIN_ALT))
            if _in_fin:
                if self.nav_mode != 'fin':
                    self.nav_mode = 'fin'
                    self._fin_t0 = tt
                    self._fin_vz_prev = float(vz)
                    self._fin_vh_prev_n = float(vel[1])
                    self._fin_vh_prev_e = float(vel[2])
                    self.log.note('FIN enter alt=%.2f vz=%.2f vh=%.2f'
                                  % (alt, vz, vh))
                    print('[FIN] 进入触地保持段 alt=%.2f vz=%.2f vh=%.2f'
                          % (alt, vz, vh))
                target_a, target_direction_fin = self.fin_control(
                    alt, dist, vz, vh, vmag, mass, error, vel, game_dt)
                target_direction = target_direction_fin
                #   【2026-10-11 加日志】FIN 段的【实时】水平误差与速度误差。
                #   【为什么必须加】实测落地偏差有相当一部分是在 FIN 段形成的：
                #     FIN 既能纠正 11.6 m（183123），也能恶化 13.1 m（183756）。
                #   但 FIN 段不走 track()，于是 dp_*/dve_* 等 68 列【全部冻结】，
                #   无法回答"FIN 为什么变差"。
                #   【为什么不直接复用 dp_*】dp_* 的语义是"参考 − 当前"
                #   （相对规划轨迹的偏差）；FIN 段没有参考轨迹，复用会曲解语义。
                #   故【新增】4 列，语义明确为：
                #     fin_dp_*  = 到瞄准点的水平位置误差 = error[1:3]
                #     fin_dve_* = 到"水平速度归零"的误差   = -vel[1:3]
                #   （FIN 的目标就是水平速度→0、位置→瞄准点）
                self.dbg_fin_dp_n = float(error[1])
                self.dbg_fin_dp_e = float(error[2])
                self.dbg_fin_dve_n = -float(vel[1])
                self.dbg_fin_dve_e = -float(vel[2])
                idx = float(self.gfold_n_i)     # 日志沿用当前参考索引
                # 起落架在 FIN 段必须展开（触地前一定要放下）
                self.v.control.gear = True
            else:
                target_a, idx = self.track(self.plan, tp, tv, self.gfold_n_i)
                target_direction = self.target_direction
                # 放起落架（原版 line 407-408）
                if (plan_x.shape[1] - self.gfold_n_i) \
                        * self.plan['tf'] / plan_x.shape[1] < 10:
                    self.v.control.gear = True
            # 【2026-09-29 第三步 3(b)】final 段的自杀点火律【已整体删除】。
            #   原实现的完整证据与理由见本节开头的长注释。摘要：
            #     · frac ∝ 1/vz² 在触地前奇异 => 油门 5 帧内 0.12->1.00
            #     · 两套律切换 => gnc_phase 冻结点 + 油门阶跃
            #     · 第二步的下降率包络已让参考轨迹本身平滑收到触地
            #   ⇒ 全程只保留 track() 一条律（FIN 段是它的触地接班）。
            if self.args.dry_run:
                # dry-run 不写执行器（apply 里才更新 thr_cmd）
                self.thr_cmd = 0.0
            else:
                if not self.engaged:
                    break
                if self.nav_mode in ('gfold', 'fin'):
                    self.apply(target_a, target_direction, game_dt)
                    # apply() 内部会写 self.thr_cmd / self.tilt_act

            # 画参考轨迹（对齐参考仓库 line 326-327：解完就更新线，
            #   与 dry_run / nav_mode 无关）。用 --debug-lines 控制开关。
            # 【2026-09-26 修正】plan 为 None 时不能画（旧版会 TypeError 被
            #   except 吞掉，表现为"完全看不到线"且毫无提示）。这里显式判断。
            _t_ctrl = time.time()
            # 【第十五步 15(d)】把"本帧是否真的画了线"落盘。
            #   此前只有 seg_draw 耗时列，无法区分"没画"和"画了但很快"。
            #   draw_on=1 是 --debug-lines 生效的【直接证据】。
            if self.debug_lines and self.plan is not None:
                self.draw(self.plan)
                self.dbg_draw_on = 1.0
            else:
                self.dbg_draw_on = 0.0
            self.dbg_seg_ctrl = (_t_ctrl - _t_state_done) * 1000.0
            self.dbg_seg_draw = (time.time() - _t_ctrl) * 1000.0

            # 姿态实测
            # 【2026-09-26 修正】旧版这里写 `tilt_act = self.tilt_cmd`，
            #   于是日志的 tilt_act 列【只是命令的回声】，看不出姿态有没有跟上。
            #   现在 tilt_act 是 apply() 里从机头方向（flight().direction）
            #   实测的真实倾角。
            att_err = self.att_err
            tilt_act = getattr(self, 'tilt_act', float('nan'))
            #   【2026-10-11 加日志】指令/实际的姿态【速率】。
            #   【为什么需要】姿态相位滞后是唯一还没量准的量：它决定
            #   "该给姿态环加阻尼，还是该降低指令速率"。
            #   现有的 tilt_rate 用的是 game_dt（历史上出过时间标度错误，
            #   见文件中 game_prev_time 的长注释），本列用与主循环同源的
            #   game_dt 但【明确记录一阶差分】，两者可交叉核对。
            #   单位 deg/s，正 = 倾角增大。
            _dt_log = float(game_dt) if game_dt > 1e-6 else 0.0
            if _dt_log > 0.0:
                _pc = getattr(self, '_prev_tilt_cmd_log', None)
                _pa = getattr(self, '_prev_tilt_act_log', None)
                if _pc is not None and self.tilt_cmd == self.tilt_cmd:
                    self.dbg_tilt_cmd_rate = (float(self.tilt_cmd)
                                              - float(_pc)) / _dt_log
                if _pa is not None and tilt_act == tilt_act:
                    self.dbg_tilt_act_rate = (float(tilt_act)
                                              - float(_pa)) / _dt_log
            if self.tilt_cmd == self.tilt_cmd:
                self._prev_tilt_cmd_log = float(self.tilt_cmd)
            if tilt_act == tilt_act:
                self._prev_tilt_act_log = float(tilt_act)

            # ---- 公共日志 ----
            # 【pyright 提示的风险】这行读 self.plan['tf'] / self.plan['x']。
            #   hold 分支带 continue（不会走到这里），gfold/final 分支都要求
            #   plan 已就绪 —— 所以正常情况下安全。但为防将来改动引入
            #   "final 段在 plan 就绪前进入"这类路径，这里显式兜底，
            #   避免一个 TypeError 让整个 kRPC 进程退出（火箭失控）。
            _plan = self.plan
            _tf = float(_plan['tf']) if _plan is not None else 0.0
            # 【2026-09-28 修正 h_min_plan】
            #   旧实现取 plan['x'][0,:].min() —— 那是【整条规划的最低高度】，
            #   而规划本来就要落到地面，所以它恒为 ~0（实测 117/117 帧都是
            #   -0.0）。它是【规划的不变量】，逐帧记录毫无信息量。
            #   改为记录【规划在当前索引处的高度】= 此刻"应该在哪"，
            #   与 alt 并列即可一眼看出领先/滞后。
            _hmin = float('nan')
            if _plan is not None:
                try:
                    _N = _plan['x'].shape[1]
                    _j = int(max(0, min(_N - 1, round(self.gfold_n_i))))
                    _hmin = float(_plan['x'][0, _j])
                except Exception:                    # noqa: BLE001
                    _hmin = float('nan')
            self.log.row(
                t=round(tt, 3), wall=round(time.time(), 2),
                alt=round(alt, 2), dist_hz=round(dist, 2),
                vz=round(vz, 3), vh=round(vh, 3), vmag=round(vmag, 3),
                mass=round(mass, 1), thr_cmd=round(getattr(self, 'thr_cmd', 0), 4),
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

            # ============================================================
            # 【2026-10-09 P3】结束条件
            # ------------------------------------------------------------
            #   FIN 段：只有【真正触地】(situation == 'landed') 才算完成，
            #     另有超时与爬升两个安全出口（见下）。
            #   非 FIN 段（FIN_ENABLE=False 的回退路径）：沿用旧的
            #     alt <= TARGET_ALT+1 判定，避免漏掉触地。
            # ============================================================
            if self.nav_mode == 'fin':
                _fin_dt = tt - float(getattr(self, '_fin_t0', tt))
                _situ = self.v.situation.name
                if _situ == 'landed':
                    print(f'[done] 触地 alt={alt:.2f} vz={vz:.2f} '
                          f'vh={vh:.2f} dist={dist:.2f} tilt={tilt_act:.2f} '
                          f'(FIN {_fin_dt:.1f}s)')
                    self.log.note(f'LANDED alt={alt:.2f} vz={vz:.2f} '
                                  f'vh={vh:.2f} dist={dist:.2f} '
                                  f'tilt={tilt_act:.2f} fin={_fin_dt:.2f}')
                    break
                if _fin_dt > float(FIN_TIMEOUT):
                    print('[FIN] 超时 %.1fs 仍未触地，退出' % _fin_dt)
                    self.log.note('FIN timeout %.2f' % _fin_dt)
                    break
                if vz > float(FIN_TAKEOFF_GUARD):
                    print('[FIN] 检测到爬升 vz=%.2f，退出' % vz)
                    self.log.note('FIN climb abort vz=%.2f' % vz)
                    break
                time.sleep(self.args.dt)
                continue

            # 结束条件（非 FIN 路径）
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
            # ============================================================
            # 【2026-10-11 用户要求：落地后不关 RCS/SAS，姿态交给 SAS】
            # ------------------------------------------------------------
            # 用户实机观察："rcs 是一直开着的，只要落地后开启 sas
            # 瞄准轨道径向向外就可以了"。
            #
            # 【为什么必须显式打开 SAS】
            #   本进程结束后就不再写 control.pitch/yaw/roll。此时若 SAS
            #   是关闭的，姿态没有任何东西维持 —— 一级会在触地后慢慢歪倒。
            #   径向模式(SASMode.radial)让机头保持指向天顶，与 FIN 律的
            #   目标一致，正好实现"保持原来的指向"。
            #
            # 【为什么必须放在循环之外】
            #   SAS 一旦打开，KSP 会【忽略】control.pitch/yaw/roll 的写入。
            #   若在循环内打开，FIN 律（以及 gfold 段）的姿态控制会立刻失效。
            #   放在这里 = 主循环已退出、不再输出控制量的时刻。
            #
            # 【为什么只对 landed 生效】
            #   超时/爬升等异常出口不应强行给 SAS 模式（可能正在爬升，
            #   径向保持没有意义，且会改变异常场景的既有行为）。
            #
            # 【异常安全】kRPC 的枚举属性名在不同版本间有差异，且此刻
            #   飞船状态可能已不可读；任何异常都不能让收尾路径崩掉
            #   （崩了就不写日志、不给 SAS，反而更糟）。
            # ============================================================
            try:
                _landed = (str(self.v.situation.name) == 'landed')
            except Exception:                        # noqa: BLE001
                _landed = False
            if _landed:
                try:
                    from krpc.services.spacecenter import SASMode as _SAS
                    #   RCS 保持开启（用户确认它本来就是一直开着的，
                    #   这里显式写一次是为了让日志/行为不依赖外部状态）
                    self.v.control.rcs = True
                    self.v.control.sas = True
                    self.v.control.sas_mode = _SAS.radial
                    self.dbg_post_sas = 1.0
                    print('[done] 落地后姿态交给 SAS（径向向外），'
                          'RCS 保持开启')
                except Exception as _e:              # noqa: BLE001
                    self.dbg_post_sas = 0.0
                    print('[WARN] 落地后设置 SAS 失败: %s: %s'
                          % (type(_e).__name__, _e))
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
            try:
                th = float(v.available_thrust)
            except Exception:                        # noqa: BLE001
                th = float(v.max_thrust)
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
    # 【第十五步 15(d)】补齐【正向】开关。此前只有 --no-debug-lines，
    #   而 DEBUG_LINES 常量是 False，导致画线【永远打不开】（见 __init__
    #   里 self.debug_lines 处的完整说明）。
    tl = ap.add_mutually_exclusive_group()
    tl.add_argument('--debug-lines', action='store_true',
                    help='画参考轨迹线（覆盖常量；屏幕可见，但每帧约 +100 ms）')
    tl.add_argument('--no-debug-lines', action='store_true',
                    help='不画参考轨迹线（覆盖常量）')
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
