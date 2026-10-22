# -*- coding: utf-8 -*-
"""starship_land.py -- starship 一级（分离器以下）的 kRPC + G-FOLD 回收控制端。

============================================================================
【本文件的定位】
  与 dev/krpc/gfold_land.py（B1040-9 的实飞成功版本）【结构完全同构】，
  只把与载具相关的数值换成 starship 一级的实测值：

      demo3_gfold.py                本文件
      ─────────────────────────    ──────────────────────────────
      GFOLD_direct_exec (P3/P4)  → dev/starship/starship_p3p4.py
      ref_target                 → Ship._make_target_frame()
      find_nearest_index         → Ship.find_nearest_index()
      sample_index               → Ship.sample_index()
      conic_clamp                → Ship.conic_clamp()
      G-FOLD 跟随                → Ship.track()
      PID 姿态                   → starship_ctrl.PID + Ship.apply()
      CSV 日志                   → starship_ctrl.Logger

  控制律、限幅、安全约束、【一条都不减】。唯一的差别是常量值，
  每个常量的来历都写在 starship_config.py 的注释里。

============================================================================
【载具尺度差异（决定了几乎所有参数都要重算）】
                      旧 B1040-9        starship 一级
  干重                 144 t             628.5 t
  湿重                 225 t             4361 t
  回收推力             13.5 MN (9台)     17.5 MN (7台 @250%)
  悬停油门             0.171             0.548
  横向惯量             1.8e4 kg·m²       1.6e9 kg·m²   ← 差 5 个数量级
  角加速度 alpha       约 20 deg/s²      3.83 deg/s²
  轴向长度             约 20 m            105.8 m
  ⇒ 姿态权限小 5.2 倍：姿态环增益、参考限速、加加速度上限全部重算。

============================================================================
【启动时序（不可颠倒）】
  ① wait_handoff  —— 等 kOS 写 GFOLD_HANDOFF 并 unlock
  ② confirm_takeover —— 实测 kRPC 真能驱动执行器（写节流、读回比对）
  ③ shutdown_engines —— 关掉 26 台，只留 e1/e2/e3/e5/e8/e10/e13
  ④ wait_gate    —— 等交接门（高度/垂速/横速三层）
  ⑤ 首次解算（非阻塞）+ hold 段（反向减速稳住）
  ⑥ gfold 段跟踪 → FIN 触地保持段 → 落地后把姿态交给 SAS

  【为什么关引擎必须在 ② 之后】关引擎不可逆。若 kOS 还没放权就关掉
  26 台，kOS 仍按"33 台"规划，必然坠毁；若 kRPC 还没真正驱动执行器，
  关引擎等于在无人控制下砍推力。任何一步失败都保留完整推力可中止。

用法：
    python starship_land.py --dry-run          # 只解算画线，不写执行器
    python starship_land.py                    # 正式接管
    python starship_land.py --list-vessels     # 列出场景里的船只与筛选判据
    python starship_land.py --no-shutdown      # 不关引擎（调试用）
    python starship_land.py --replan-dt 3      # 开启周期重解（默认 0 = 不重解）
============================================================================
"""
import argparse
import math
import os
import sys
import threading
import time
# 【为什么全程用 cast 而不是 if ... is None】
#   Pylance 会把 conn.krpc / conn.space_center 判成 "None 的成员"
#   （reportOptionalMemberAccess）。根因在 krpc 包自身：
#     krpc/services/__init__.py 里写的是
#         try:    from krpc.services.krpc import KRPC
#         except ImportError: KRPC = lambda _: None     # <- 兜底分支
#   类型检查器把两条分支合并成 KRPC | None。运行期 KRPC 是真类（已实测），
#   属 stub 缺陷。显式 cast 是【记录事实】而不是压警告。
from typing import Any, cast

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, '..', 'solver'))

import starship_config as C            # noqa: E402
import starship_ctrl as K              # noqa: E402
import starship_p3p4 as S              # noqa: E402

# ---- 把配置里的常量提升到本模块作用域（与旧程序同名，便于逐行对照）----
G0 = C.G0
TARGET_LAT = C.TARGET_LAT
TARGET_LON = C.TARGET_LON
TARGET_ALT = C.TARGET_ALT
TARGET_HEADING_DEG = C.TARGET_HEADING_DEG
ISP_DEFAULT = C.ISP_VAC
TF_GUESS = C.TF_GUESS
STRAIGHT_FAC = C.STRAIGHT_FAC
GS_DEG = C.GS_DEG
PCS_START_DEG = C.PCS_START_DEG
PCS_END_DEG = C.PCS_END_DEG
V_MAX_SOLVER = C.V_MAX_SOLVER
N3 = C.N3
N4 = C.N4
V_DESCENT_TERM_WIN = C.V_DESCENT_TERM_WIN
V_DESCENT_TERM_VZ = C.V_DESCENT_TERM_VZ
TERM_CLEARANCE = C.TERM_CLEARANCE
TERM_DESCENT_VZ = C.TERM_DESCENT_VZ
REF_SLEW_ENABLE = C.REF_SLEW_ENABLE
REF_SLEW_HEADROOM_DEG = C.REF_SLEW_HEADROOM_DEG
REF_SLEW_MIN = C.REF_SLEW_MIN
REF_SLEW_MAX = C.REF_SLEW_MAX
CTRL_X_ROT_KP = C.CTRL_X_ROT_KP
CTRL_X_ROT_KD = C.CTRL_X_ROT_KD
CTRL_Z_ROT_KP = C.CTRL_Z_ROT_KP
CTRL_Z_ROT_KD = C.CTRL_Z_ROT_KD
CTRL_Y_AVEL_KP = C.CTRL_Y_AVEL_KP
RATE_D_GAIN = C.RATE_D_GAIN
RATE_D_ENABLE = C.RATE_D_ENABLE
FF_ENABLE = C.FF_ENABLE
FF_GAIN = C.FF_GAIN
FF_CLAMP_RAD = C.FF_CLAMP_RAD
TAU_SCHED_ENABLE = C.TAU_SCHED_ENABLE
TAU_SCHED_MIN = C.TAU_SCHED_MIN
TAU_SCHED_MAX = C.TAU_SCHED_MAX
TAU_SCHED_ALPHA_REF = C.TAU_SCHED_ALPHA_REF
TILT_CAP_LO = C.TILT_CAP_LO
TILT_CAP_HI = C.TILT_CAP_HI
TILT_CAP_SLOPE = C.TILT_CAP_SLOPE
TILT_TERM_ALT = C.TILT_TERM_ALT
TILT_AZ_MIN_ERR = C.TILT_AZ_MIN_ERR
T_GAIN_MIN = C.T_GAIN_MIN
T_GAIN_MAX = C.T_GAIN_MAX
TILT_GAIN_CAP_ENABLE = C.TILT_GAIN_CAP_ENABLE
TILT_GAIN_MAX = C.TILT_GAIN_MAX
TILT_LEAD_ENABLE = C.TILT_LEAD_ENABLE
TILT_LEAD_SEC = C.TILT_LEAD_SEC
TILT_LEAD_MAX_FRAC = C.TILT_LEAD_MAX_FRAC
CONIC_TILT_DEG = C.CONIC_TILT_DEG
JERK_GUARD_MIN = C.JERK_GUARD_MIN
JERK_GUARD_MAX = C.JERK_GUARD_MAX
JERK_SLEW_FRAC = C.JERK_SLEW_FRAC
TILT_RATE_MAX_DEG = C.TILT_RATE_MAX_DEG
K_POS = C.K_POS
K_VEL = C.K_VEL
AVER_TGT_VZ = C.AVER_TGT_VZ
AVER_H_FLOOR = C.AVER_H_FLOOR
AVER_FLOOR = C.AVER_FLOOR
AVER_FLOOR_ENABLE = C.AVER_FLOOR_ENABLE
AVER_CLIMB_K = C.AVER_CLIMB_K
AVER_CLIMB_MARGIN = C.AVER_CLIMB_MARGIN
GFOLD_THROTTLE_MARGIN = C.GFOLD_THROTTLE_MARGIN
GFOLD_THROTTLE_MIN = C.GFOLD_THROTTLE_MIN
GFOLD_THROTTLE_RELEASE = C.GFOLD_THROTTLE_RELEASE
HOLD_UP_FRAC = C.HOLD_UP_FRAC
HOLD_TAU = C.HOLD_TAU
HOLD_CLIMB_THR_FRAC = C.HOLD_CLIMB_THR_FRAC
IDX_VEL_WEIGHT = C.IDX_VEL_WEIGHT
IDX_MONOTONIC = C.IDX_MONOTONIC
TAKEOVER_SAMPLES = C.TAKEOVER_SAMPLES
MAX_TILT_CMD_DEG = C.MAX_TILT_CMD_DEG
FIN_ENABLE = C.FIN_ENABLE
FIN_ALT = C.FIN_ALT
FIN_TIMEOUT = C.FIN_TIMEOUT
FIN_TAKEOFF_GUARD = C.FIN_TAKEOFF_GUARD
FIN_DESCENT_VZ = C.FIN_DESCENT_VZ
FIN_KP = C.FIN_KP
FIN_KD = C.FIN_KD
FIN_VH_KP = C.FIN_VH_KP
FIN_VH_KD = C.FIN_VH_KD
FIN_TILT_MAX_DEG = C.FIN_TILT_MAX_DEG
FIN_UP_MIN = C.FIN_UP_MIN
FIN_TILT_HOLD = C.FIN_TILT_HOLD
LAND_ALT_EPS = C.LAND_ALT_EPS
CLIMB_ABORT_VZ = C.CLIMB_ABORT_VZ


# ============================================================================
# 1. kRPC 连接
# ============================================================================
def connect() -> Any:
    """连 kRPC 并返回 Client（cast 理由见文件头 import 处的注释）。"""
    import krpc
    print('[krpc] connecting to 127.0.0.1:50000 ...')
    conn = krpc.connect(name='starship_land', address='127.0.0.1',
                        rpc_port=50000)
    if conn is None:
        raise RuntimeError('krpc.connect() 返回 None')

    svc = cast(Any, conn.krpc)
    # 【不能写 conn.krpc.get_version()】KRPC 服务没有该方法（实飞 AttributeError）。
    #   正确入口是 get_status()，返回 Status 协议对象，版本在 .version 字段。
    try:
        ver = svc.get_status().version
    except Exception:                    # noqa: BLE001
        ver = 'unknown'
    print('[krpc] connected  server=%s' % ver)
    return conn


# ============================================================================
# 2. 引擎关停（保留 e1/e2/e3/e5/e8/e10/e13，其余 26 台关掉）
# ============================================================================
def shutdown_engines(vessel, dry_run=False):
    """按 KOSNameTag 关掉 ENGINE_TAGS_OFF 里的引擎。

    【为什么用 name tag 而不是引擎列表下标】
      用户明确按 e1~e33 命名；下标会随部件顺序/分级变化而漂移。
    【必须用 part.engine（单数），不是 part.engines】
      kRPC 0.6.0 的 SpaceCenter.Part 只有 engine 一个属性，
      且【非引擎部件返回 None】。实飞踩坑（2026-10-20）：
          AttributeError: 'Part' object has no attribute 'engines'
      校验工具：dev/starship/api_check.py
    【返回】关掉的引擎数、保留的引擎数。
    """
    keep = set(C.ENGINE_TAGS_KEEP)
    off = set(C.ENGINE_TAGS_OFF)
    n_off = 0
    n_keep = 0
    n_tagged = 0
    for part in vessel.parts.all:
        try:
            tags = [t for t in part.tag.split() if t]
        except Exception:                        # noqa: BLE001
            tags = []
        if not tags:
            continue
        hit = [t for t in tags if t in off]
        miss = [t for t in tags if t in keep]
        if not hit and not miss:
            continue
        try:
            eng = part.engine
        except Exception:                        # noqa: BLE001
            eng = None
        if eng is None:
            print('[engine] 警告: 部件 %s 带 tag %s 但不是引擎，已跳过'
                  % (part.name, ','.join(tags)))
            continue
        n_tagged += 1
        if hit:
            if not dry_run:
                eng.active = False
            n_off += 1
        elif miss:
            n_keep += 1
    print('[engine] 关闭 %d 台, 保留 %d 台 (共识别 %d 台带 tag 引擎, dry_run=%s)'
          % (n_off, n_keep, n_tagged, dry_run))
    # ---- 一致性检查：数量对不上说明 tag 命名与配置不符，必须当场发现 ----
    if not dry_run and n_tagged:
        exp_off = len(C.ENGINE_TAGS_OFF)
        exp_keep = len(C.ENGINE_TAGS_KEEP)
        if n_off != exp_off or n_keep != exp_keep:
            print('[engine] *** 警告：期望 关%d/留%d，实际 关%d/留%d ***'
                  % (exp_off, exp_keep, n_off, n_keep))
            print('[engine] 检查 starship_config.ENGINE_TAGS_KEEP 与 craft 里的 nameTag')
    return n_off, n_keep


# ============================================================================
# 3. 飞行器封装（状态 / 参考系 / 求解 / 跟踪 / 执行）
# ============================================================================
class Ship:
    """状态与参考系封装 + G-FOLD 跟踪律 + 执行器写入。

    【参考系构造照抄 dev/krpc/gfold_land.py 的 target_frame()】
      实飞踩过的坑：若 CreateHybrid 的 velocity 传 body_frame，原点会被赋予
      天体自转的线速度（该纬度 ~175 m/s），于是 vh 永不为 0、
      G-FOLD 恒 infeasible。⇒ 必须传 tgt_frame。
    """

    # 【类型】类级声明一次，避免 pyright 从 "参数默认 None" 推出 args 是 None。
    args: Any
    debug_lines: bool

    def __init__(self, conn, vessel, log=None, args: Any = None):
        self.conn = conn
        self.sc = cast(Any, conn.space_center)
        self.body = vessel.orbit.body
        self.v = vessel
        self.log = log or K.Logger(enabled=False)
        # 【类型】args 由 main() 传入 argparse.Namespace（永不为 None）；
        #   声明成 Any 是为了让 pyright 不去对 Optional 做成员检查。
        self.args: Any = args
        self.debug_lines = bool(getattr(args, 'debug_lines', False))
        self._tscale = 1.0
        self._tscale_buf = []
        self._tscale_prev_cmd = float('nan')
        self._ref_slew_prev = None
        self._ff_dir_prev = None
        self._last_pos = None
        self._last_t = 0.0
        self._rot_warned = False
        self._vrot_warned = False
        # ---- 姿态环 PID（俯仰=x轴, 偏航=z轴）----
        self.ctrl_x_rot = K.PID()
        self.ctrl_x_rot.kp = CTRL_X_ROT_KP
        self.ctrl_x_rot.kd = CTRL_X_ROT_KD
        self.ctrl_z_rot = K.PID()
        self.ctrl_z_rot.kp = CTRL_Z_ROT_KP
        self.ctrl_z_rot.kd = CTRL_Z_ROT_KD
        # ---- 运行时状态 ----
        self.plan = None
        self.gfold_n_i = -100.0
        self.gfold_status = 'init'
        self.solve_ms = 0.0
        self.thr_cmd = 0.0
        self.tilt_cmd = 0.0
        self.tilt_act = float('nan')
        self.tilt_dir_cmd = float('nan')
        self.att_err = float('nan')
        self.avel = float('nan')
        self.target_direction = np.array([1.0, 0.0, 0.0])
        self.a_cmd_vec = np.zeros(3)
        self.est_time = 0.5
        self.last_pkt = 0.02
        self.replan_n = 0
        self.last_replan = 0.0
        self.nav_mode = 'none'
        self.engaged = False
        self.takeover_ok = None
        self.takeover_bad = 0
        self.takeover_n = 0
        self.prev_vel = None
        self.tf_az_dir = None
        self.tf_ah_dir = None
        self.tf_ah_prev = 0.0
        self.min_mag_eff = 0.0
        self.tilt_cap = 0.0
        self.tf_az_dir = None
        self.av_pad = None
        self.dbg_post_sas = 0.0
        self._solve_thread = None
        self._pending_plan = None
        self._pending_msg = None
        self._fin_t0 = 0.0
        self._fin_vz_prev = 0.0
        self._fin_vh_prev_n = 0.0
        self._fin_vh_prev_e = 0.0
        self._prev_tilt_cmd_log = None
        self._prev_tilt_act_log = None
        self._prev_tilt_act = float('nan')
        self._ref = None
        # ---- 绘图 ----
        self.drawn = []
        self._lines_N = None
        self._marker = None
        self.gfold_traj_ready = False
        self.thr_floor_last = float('nan')
        self.dbg_thrust_n = float('nan')
        self.dbg_thr_real = float('nan')
        self._frozen_sig = None
        self._frozen_n = 0
        # 与旧程序同名的诊断量（_dbg_cols 逐列读取）
        for _n in ('dbg_alpha_max', 'dbg_avel_mag', 'dbg_tau_p', 'dbg_tau_r',
                   'dbg_tau_y', 'dbg_I_p', 'dbg_I_r', 'dbg_I_y',
                   'dbg_loop_dt', 'dbg_rpc_ms', 'dbg_ctrl_ms',
                   'dbg_tilt_rate', 'dbg_tilt_cmd_rate', 'dbg_tilt_act_rate',
                   'dbg_pid_err_x', 'dbg_pid_err_z', 'dbg_pid_px', 'dbg_pid_dx',
                   'dbg_pid_ix', 'dbg_pid_pz', 'dbg_pid_dz', 'dbg_pid_iz',
                   'dbg_pid_out_x', 'dbg_pid_out_z', 'dbg_pid_out_r',
                   'dbg_sched', 'dbg_avel_scale', 'dbg_rate_x', 'dbg_rate_y',
                   'dbg_ff_x', 'dbg_ff_y', 'dbg_ref_slew', 'dbg_ref_clamped',
                   'dbg_idx_raw', 'dbg_idx_frozen', 'dbg_tf_perr',
                   'dbg_tf_verr', 'dbg_dp_n', 'dbg_dp_e', 'dbg_dve_n',
                   'dbg_dve_e', 'dbg_atq_thr', 'dbg_atq_floor',
                   'dbg_pid_sat_x', 'dbg_pid_sat_z', 'dbg_pid_sat_run',
                   'dbg_nose_n', 'dbg_nose_e', 'dbg_nose_az', 'dbg_tgt_az',
                   'dbg_az_err', 'dbg_tf_tilt_j', 'dbg_tf_tilt_plan',
                   'dbg_tf_tilt_corr', 'dbg_tf_tilt_want', 'dbg_tf_taper',
                   'dbg_tf_arrived', 'dbg_tf_cap', 'dbg_tf_slew_allow',
                   'dbg_tf_lead_n', 'dbg_tf_ahdir_cos', 'dbg_ca_T',
                   'dbg_ca_kv', 'dbg_ca_kp', 'dbg_ca_posclip', 'dbg_ah_dem',
                   'dbg_ah_cmd', 'dbg_ah_sat', 'dbg_j_max', 'dbg_j_used',
                   'dbg_ah_cap_bind', 'dbg_j_phys', 'dbg_j_lim',
                   'dbg_av_plan', 'dbg_av_need', 'dbg_av_up', 'dbg_av_net',
                   'dbg_av_in_up', 'dbg_av_out_up', 'dbg_av_floor',
                   'dbg_min_mag_eff', 'dbg_floor_lift', 'dbg_a_ver_out',
                   'dbg_draw_on', 'dbg_fin_a_up', 'dbg_fin_need', 'dbg_fin_ah',
                   'dbg_fin_dp_n', 'dbg_fin_dp_e', 'dbg_fin_dve_n',
                   'dbg_fin_dve_e', 'dbg_u_h', 'dbg_pdv_h', 'dbg_pdp_h',
                   'dbg_u_dot', 'dbg_pdv_dot', 'dbg_pdp_dot', 'dbg_cmd_dot',
                   'dbg_he_n', 'dbg_he_e', 'dbg_hd_n', 'dbg_hd_e',
                   'dbg_h_misalign', 'dbg_plan_alt', 'dbg_plan_vz',
                   'dbg_plan_vh', 'dbg_trk_pos', 'dbg_trk_pos_up',
                   'dbg_trk_pos_h', 'dbg_trk_vel', 'dbg_n_i', 'dbg_plan_nj',
                   'dbg_vdesc', 'dbg_plan_tf_used', 'dbg_plan_x0_up',
                   'dbg_av_in_up2', 'dbg_auth_ok', 'dbg_s_range'):
            setattr(self, _n, float('nan'))
        self.dbg_avel_local = [float('nan')] * 3
        self.dbg_dir = None
        self.dbg_tgt = None
        self.dbg_aim_err = float('nan')
        self.dbg_floor_lift = 0.0
        self.dbg_a_ver_out = 0.0
        self.dbg_draw_on = 0.0
        self.dbg_ah_sat = 0.0
        self.dbg_ah_cap_bind = 0.0
        self.dbg_ref_clamped = 0.0
        self.dbg_pid_sat_x = 0.0
        self.dbg_pid_sat_z = 0.0
        self.dbg_pid_sat_run = 0.0
        self.dbg_atq_thr = 0.0
        self.dbg_atq_floor = 0.0
        self.dbg_tf_arrived = 0.0
        self.dbg_tf_taper = 1.0
        self.dbg_ca_posclip = 0.0
        self.idx_frozen_n = 0
        self._sample_arrived_ok = False
        self.av_pad = None

    # ------------------------------------------------------------------
    # 3.1 绑定 / 描述
    # ------------------------------------------------------------------
    def bind(self, vessel):
        """换绑载具（重新建参考系与 PID 状态）。"""
        self.v = vessel
        self.body = vessel.orbit.body
        self._ref = None
        self._tscale = 1.0
        self._tscale_buf = []
        self._ref_slew_prev = None
        self.ctrl_x_rot = K.PID()
        self.ctrl_x_rot.kp = CTRL_X_ROT_KP
        self.ctrl_x_rot.kd = CTRL_X_ROT_KD
        self.ctrl_z_rot = K.PID()
        self.ctrl_z_rot.kp = CTRL_Z_ROT_KP
        self.ctrl_z_rot.kd = CTRL_Z_ROT_KD

    def describe(self, v, n_engine=None):
        """返回载具关键量的摘要 dict（供日志与屏幕）。

        【BUG 修复 2026-10-20：必须区分「全开」与「回收构型」】
        ----------------------------------------------------------------
        实飞日志里出现过这样一行：
            [vessel] starship  mass=986.1 t  thrust=77.87 MN  a_net=69.16
        那是在【关引擎之前】打印的 —— 77.87 MN 是 33 台全开的推力，
        a_net=69.16 是 33 台的净加速度。
        而真正用于着陆的构型只有 7 台：thrust=16.49 MN、a_net=6.93。
        70 与 7 相差 10 倍，看日志的人会误以为着陆段有 69 m/s² 的权限。

        【修法】接受可选参数 n_engine 标注「本行对应几台引擎」，
        并在 a_net 后面带上该标注；关引擎后由调用方再打一次。
        ================================================================
        """
        m = float(v.mass)
        th = max(1.0, float(v.available_thrust)) * self._thrust_scale()
        _n = n_engine
        if _n is None:
            # 未指定时按【当前活动的引擎数】统计，避免拿全开数冒充着陆构型
            try:
                _n = sum(1 for e in v.parts.engines if bool(e.active))
            except Exception:                    # noqa: BLE001
                _n = None
        return dict(name=v.name, mass=m, thrust=th,
                    a_net=th / max(1.0, m) - G0, n_engine=_n)

    # ------------------------------------------------------------------
    # 3.2 参考系
    # ------------------------------------------------------------------
    def target_frame(self):
        """目标系：原点 = 着陆点（抬高 TARGET_ALT），x=天顶, y=北, z=东。

        【API 语义，对照 spacecenter.py + 官方 stub 逐条核实】
          · Vessel.SurfaceReferenceFrame：origin 在质心，x=zenith, y=north, z=east
          · ReferenceFrame.CreateRelative(frame, position, rotation, velocity)
              position = 原点偏移【位置矢量】；rotation = 【四元数 (x,y,z,w)】
          · ReferenceFrame.CreateHybrid(position, rotation, velocity, angular_velocity)
              四个参数都是【参考系】，不是矢量。

        ⇒ 两步构造：
          ① create_relative 在【天体固连系】里建一个原点=目标点的系
          ② create_hybrid 与 surface 系混合（提供 x=天顶 的旋转），
             速度继承【目标点系】（目标点在系内静止）—— 传 body_frame
             会注入天体自转线速度（该纬度 ~175 m/s），vh 永不为 0。
        """
        body_frame = self.body.reference_frame
        tgt = np.array(self.body.surface_position(
            TARGET_LAT, TARGET_LON, body_frame), dtype=float)
        n = float(np.linalg.norm(tgt))
        if n > 1e-6:
            tgt = tgt + tgt / n * TARGET_ALT
        tgt_frame = self.sc.ReferenceFrame.create_relative(
            body_frame, position=tuple(float(x) for x in tgt))
        return self.sc.ReferenceFrame.create_hybrid(
            position=tgt_frame,
            rotation=self.v.surface_reference_frame,
            velocity=tgt_frame)

    @property
    def ref(self):
        if self._ref is None:
            self._ref = self.target_frame()
        return self._ref

    def body_rotation_speed(self):
        """本机所在纬度处、天体自转的线速度 [m/s]（自转污染判定标尺）。

        【API 名称】CelestialBody 没有 rotation_period，
        正确名称是 rotational_period（易错，与 KSP 的 rotationPeriod 拼写不同）。
        """
        try:
            o = 2.0 * math.pi / float(self.body.rotational_period)
            lat = float(self.v.flight(self.ref).latitude)
            return abs(o * float(self.body.equatorial_radius)
                       * math.cos(math.radians(lat)))
        except Exception as e:                    # noqa: BLE001
            if not self._vrot_warned:
                self._vrot_warned = True
                print('[WARN] 无法取得天体自转速度（%s: %s）'
                      % (type(e).__name__, e))
                print('[WARN] 自转污染自检将不可用 —— 检查 CelestialBody '
                      '成员名（正确为 rotational_period）')
            return 0.0

    # ------------------------------------------------------------------
    # 3.3 状态采样
    # ------------------------------------------------------------------
    def state(self):
        """读当前状态，返回 (alt, dist_hz, vz, vh, vmag, mass, tp, tv)。

        【为什么只看 flight().surface_altitude】
          与 kOS 的 alt:radar 口径最接近，交接门的判据也在 kOS 侧。
        【目标系】x=上，y=北，z=东
          dist_hz = hypot(tp[1], tp[2])   vz = tv[0]（向上为正）
        【自转污染自检】用相邻帧位移反推真实水平速率（位置口径，
          天然不受速度参考系影响）。曾因 velocity 传 body_frame 导致
          vh 恒多出 175 m/s、G-FOLD 全 infeasible，这个自检就是为它加的。
        """
        fl = self.v.flight(self.ref)
        alt = float(fl.surface_altitude)
        tp = self.v.position(self.ref)
        tv = self.v.velocity(self.ref)
        dist_hz = math.hypot(float(tp[1]), float(tp[2]))
        vz = float(tv[0])
        vh = math.hypot(float(tv[1]), float(tv[2]))
        v_rot = self.body_rotation_speed()
        now = float(self.sc.ut)          # 【必须用游戏时间，不能用墙钟】
        if self._last_pos is not None:
            dtp = now - self._last_t
            if dtp > 0.05:
                dp = math.hypot(float(tp[1]) - float(self._last_pos[1]),
                                float(tp[2]) - float(self._last_pos[2]))
                vh_meas = dp / dtp
                if (vh > 20.0 and vh_meas < 0.5 * vh
                        and abs(vh - vh_meas - v_rot) < 0.25 * max(1.0, v_rot)):
                    if not self._rot_warned:
                        self._rot_warned = True
                        print('[WARN] 水平速度疑似被天体自转污染: '
                              'vh=%.1f 但位移反推仅 %.1f (自转标尺 %.1f m/s)'
                              % (vh, vh_meas, v_rot))
                        print('[WARN] 位置口径正常、速度口径异常 —— '
                              '检查 target_frame 的 velocity 参数')
                        self.log.note('ROTATION CONTAMINATION vh=%.1f '
                                      'vh_meas=%.1f v_rot=%.1f'
                                      % (vh, vh_meas, v_rot))
        self._last_pos = tp
        self._last_t = now
        vmag = math.sqrt(vz * vz + vh * vh)
        return alt, dist_hz, vz, vh, vmag, float(self.v.mass), tp, tv

    # ------------------------------------------------------------------
    # 3.4 推力自校准
    # ------------------------------------------------------------------
    def _thrust_scale(self):
        """kRPC 报的推力 与 实际推力 的比值（>=1）。

        【为什么需要】旧载具实测 kRPC 的 available_thrust 既不含限幅、
        又取了真空值，低估 1.414 倍。新载具（7 台 SSME @250%）未必
        同比例低估，所以在【线上自校准】，不写死。
        """
        return float(getattr(self, '_tscale', 1.0))

    def update_thrust_scale(self, thr_cmd):
        """用一帧 (throttle, Vessel.thrust) 更新自校准系数（取中位数）。

        【为什么不用逐帧相除】发动机有惯性，throttle 与 thrust 不同相，
        逐帧相除会得到 0.46~2.25 的垃圾比值。
        四条修法（每条都对应旧程序一次实飞故障）：
          ① 只采信【油门稳定】的帧（与上一帧变化 < 0.02）
          ② 门槛降到 0.08（旧 0.12 会造成自锁：t_scale 偏高 -> 下限偏低
             -> 油门 < 门槛 -> 不再采样 -> 冻结在错值）
          ③ 采样窗口 [1.0, 2.0]（kRPC 只会低估）
          ④ 用最近 20 个样本的【中位数】，不用 EMA（EMA 对离群值不稳健）
        """
        try:
            _thr = float(self.v.thrust)
            _cmd = float(thr_cmd)
            _prev_cmd = float(self._tscale_prev_cmd)
            self._tscale_prev_cmd = _cmd
            if _prev_cmd == _prev_cmd and abs(_cmd - _prev_cmd) > 0.02:
                return
            if _cmd < 0.08 or _thr < 1.0e5:
                return
            _rep = float(self.v.available_thrust)
            if _rep < 1e5:
                return
            _ratio = (_thr / max(1e-6, _cmd)) / _rep
            # ============================================================
            # 【BUG 修复 2026-10-20】采样窗口必须放宽到 [1.0, 4.0]。
            # ------------------------------------------------------------
            # 实飞铁证（starship_log_20261020_172828.csv, t=0.17..0.42）：
            #     cmd=1.0000  实测推力=16.49 MN  kRPC available_thrust=6.59 MN
            #     ratio = (16.49/1.0)/6.59 = 2.500
            #   而 2.500 恰好 = SSME 的 thrustPercentage 250% ⇒
            #     kRPC 的 available_thrust 是【按 100% 限幅算的】，
            #     真实推力是它的 2.5 倍。
            #   旧窗口 [1.0, 2.0]【拒收】2.500，于是 _tscale 一直停在 1.0
            #   （日志 t_scale 列全程 1.0 就是证据），a_cap 恒按 6.59 MN 算
            #   ⇒ 低估 2.5 倍 ⇒ 三个连锁后果：
            #       · 求解器 t_max 传 6.59 MN，a_net 算成 -3.13（实际 +8.1）
            #       · 预检直接判推重比过低并放弃（本次事故的直接原因）
            #       · 油门 = |a|/a_cap 被高估 2.5 倍 ⇒ 恒饱和 1.0
            # 【窗口上限取 4.0】给 TweakScale / 更强限幅留余量。
            #   下限仍 1.0：kRPC 只会低估，若实测比报值小说明 throttle
            #   记录不准，那种样本不可信。
            # ============================================================
            if not (1.0 <= _ratio <= 4.0):
                return
            self._tscale_buf.append(_ratio)
            if len(self._tscale_buf) > 20:
                del self._tscale_buf[0]
            _s = sorted(self._tscale_buf)
            self._tscale = float(_s[len(_s) // 2])
        except Exception:                        # noqa: BLE001
            pass

    def calibrate_thrust(self, verbose=True):
        """【同步标定】直接从引擎部件算出真实可用推力，不依赖运行中采样。

        【为什么必须有这个函数】
          update_thrust_scale() 是【运行中】采样：它要用 Vessel.thrust 与
          throttle 对比，而首次解算发生在【任何油门写入之前】——
          那时 _tscale 还是 1.0，预检会拿被低估 2.5 倍的推力去算 a_net。
          实飞就是这样死的：a_net 被算成 -3.13，直接判推重比过低并放弃。

        【算法】
            true_thrust = sum(Engine.max_thrust * Engine.thrust_limit)  对活动引擎
            ratio       = true_thrust / Vessel.available_thrust
          依据 kRPC 文档（本地 spacecenter.py 逐条核实）：
            · Engine.max_thrust        = 节流与节流限幅【都设 100%】时的推力
                                         ⇒ 【不含】thrustPercentage 250% 那一层
            · Engine.thrust_limit      = 节流限幅本身（本载具 = 2.5）
            · Vessel.available_thrust  = 各 Engine.available_thrust 之和
          两者在同一时刻取，大气/余量效应自动抵消，比值即纯限幅倍数。

        【实测核对】本载具 7 台：max_thrust 941 kN × 2.5 × 7 = 16.47 MN，
          available_thrust 报 6.59 MN ⇒ ratio = 2.50 ✓ 与日志逐位吻合。

        【返回值】ratio（>=1.0）。取不到引擎信息时返回当前 _tscale。
        """
        try:
            tot = 0.0
            n_eng = 0
            for e in self.v.parts.engines:
                try:
                    if not bool(e.active):
                        continue
                    lim = float(e.thrust_limit)
                    mt = float(e.max_thrust)
                except Exception:                # noqa: BLE001
                    continue
                if mt <= 0.0:
                    continue
                tot += mt * max(0.0, lim)
                n_eng += 1
            rep = float(self.v.available_thrust)
            if tot <= 1.0 or rep <= 1e4 or n_eng == 0:
                if verbose:
                    print('[thrust] 同步标定跳过（引擎 %d 台, 计算值 %.2f MN,'
                          ' 报值 %.2f MN）' % (n_eng, tot / 1e6, rep / 1e6))
                return float(self._tscale)
            ratio = tot / rep
            # 合理区间：1.0~4.0。超出说明 max_thrust 口径又变了，别盲目采信。
            if not (1.0 <= ratio <= 4.0):
                if verbose:
                    print('[thrust] 同步标定结果异常 %.3f，保持 %.3f'
                          % (ratio, self._tscale))
                return float(self._tscale)
            self._tscale = float(ratio)
            if verbose:
                print('[thrust] 同步标定: %d 台活动引擎  max_thrust*limiter=%.2f MN'
                      '  kRPC available=%.2f MN  => 系数 %.3f'
                      % (n_eng, tot / 1e6, rep / 1e6, ratio))
            try:
                self.log.note('thrust calibrated: %.3f ('
                              'calc=%.2fMN reported=%.2fMN engines=%d)'
                              % (ratio, tot / 1e6, rep / 1e6, n_eng))
            except Exception:                    # noqa: BLE001
                pass
            return float(ratio)
        except Exception as e:                   # noqa: BLE001
            if verbose:
                print('[thrust] 同步标定失败: %s: %s' % (type(e).__name__, e))
            return float(self._tscale)

    def a_cap_real(self, mass=None):
        """【真实可用推力加速度】= 真实可用推力 / mass [m/s^2]。

        【口径】真实可用推力 = available_thrust × _tscale。
          kRPC 的 available_thrust 在本构建下【不含 thrustPercentage 限幅】
          （实测星星舰一级 7 台：报 6.59 MN，实际 16.49 MN，2.50 倍），
          所以必须乘标定系数，否则 a_cap 会被低估 2.5 倍。
        【兜底顺序】available_thrust -> max_thrust（同样要乘标定）。
        """
        _m = max(1.0, float(self.v.mass if mass is None else mass))
        try:
            _t = float(self.v.available_thrust)
        except Exception:                        # noqa: BLE001
            if not getattr(self, '_avail_warned', False):
                self._avail_warned = True
                print('[WARN] available_thrust 不可用，退回 max_thrust')
            _t = float(self.v.max_thrust)
        return _t * self._thrust_scale() / _m

    def thrust_true(self):
        """真实可用推力 [N]（= available_thrust × 标定系数）。

        求解器的 t_max 必须用这个值，不能用 available_thrust ——
        否则求解器会按一台弱 2.5 倍的火箭规划（实测 a_net 被算成负数）。
        """
        try:
            _t = float(self.v.available_thrust)
        except Exception:                        # noqa: BLE001
            _t = float(self.v.max_thrust)
        return max(1.0, _t) * self._thrust_scale()

    # ------------------------------------------------------------------
    # 3.5 油门下限（gfold 段）
    # ------------------------------------------------------------------
    def _gfold_floor(self, a_cap, mass):
        """gfold 段【执行油门下限】对应的加速度幅值下限 [m/s^2]。

        【2026-10-21-e 修正：悬停点必须用【本帧实测可用推力】】
        ------------------------------------------------------------
        旧 gfold_land.py:2526-2534 的写法（带实测依据）：
            _t_true = max(1.0, available_thrust) * _thrust_scale()
            _hover  = G0 * _m / _t_true
            floor   = min(GFOLD_THROTTLE_MIN, MARGIN * _hover)
        注释原文：「悬停点必须用真实可用推力。注意 available_thrust
          本身还低估 1.414 倍，故必须乘自校准系数，否则下限会高于
          真实悬停 ⇒ 下限自己把火箭往上推（实测 a_up=+3.57 m/s^2、
          一路爬到 409 m 后 vz 转正、永不落地）。」

        【我此前错写成常数 TMAX_N】
            hover = G0*mass / TMAX_N
        后果：下限不随本帧实测推力走，而 a_cap_real() 仍用
          available_thrust*_thrust_scale() ⇒ 两处口径【解耦】：
          · 若 TMAX_N 高于实测（大气稀薄/引擎损伤/TweakScale 变动）
            => 悬停油门被算低 => 下限低于真实悬停 => 丢姿态权限
            （旧程序实测 36/70 帧零油门自由落体，7.2 s 掉 292 m）
          · 若 TMAX_N 低于实测 => 下限高于真实悬停 => 下限把火箭往上推
          ⇒ 正是旧程序反复警告的那个故障型。

        【为什么 codegen 标定后 TMAX_N 仍然不能替代实测】
          TMAX_N 是【13 台满油门】的名义值，不含：
            ① 大气压对推力的影响（低空比冲低）
            ② 引擎损伤（实飞出现过 30.76 -> 16.5 MN 的掉推力）
            ③ TweakScale 缩放变化
          而 available_thrust 是 kRPC 实测（乘标定后）= 本帧真值。
        """
        _m = max(1.0, float(mass))
        _t_true = self.thrust_true()
        _hover = G0 * _m / max(1.0, _t_true)
        floor = min(float(GFOLD_THROTTLE_MIN),
                    float(GFOLD_THROTTLE_MARGIN) * _hover)
        return floor * float(a_cap)

    def _gfold_floor_effective(self, a_cap, mass, plan_a_up, err_up,
                               plan_mag=None):
        """按「是否要下降」放开油门下限。

        【为什么必须放开】规划在巡航段本来就要接近竖直的小推力
        （旧载具实测 48% 的节点规划推力低于下限），此时下限会推翻
        「跟轨迹」的意图，把载具顶上去。
        【判据】载具已落后于规划（err_up>0）或规划本身要求净向下加速
          （plan_a_up - g0 < 0）=> 放开。
        【口径陷阱】err_up 必须是【相对规划点】的高度偏差，
          不能传 error[0]（相对瞄准点，全程都是大正数）。

        ================================================================
        【B 优化 2026-10-20-c：放开值必须按【悬停油门】缩放，不能用绝对值】
        ----------------------------------------------------------------
        原实现：cap_by_release = GFOLD_THROTTLE_RELEASE * a_cap
                其中 GFOLD_THROTTLE_RELEASE = 0.10（绝对值）

        【实飞事故（starship_log_20261020_202734.csv）】
            t=2.46  min_mag_eff=8.36  av_up=11.75  thr=0.855
            t=3.32  min_mag_eff=1.68  av_up= 5.00  thr=0.363  <- 塌了
            t=3.53  min_mag_eff=1.68  av_up= 5.00  thr=0.362
            t=4.66  min_mag_eff=8.45  av_up=14.24  thr=0.999  <- 又回来
        1.68 = 0.10 * 16.59(a_cap)，即【下限自己把推力砍到 10%】。
        而此时 vz=-69 m/s，正在自由落体 —— 1.5 秒内从 0.855 掉到 0.363。

        【为什么绝对值对小载具可以、对 starship 是灾难】
            悬停油门 = g0*m/T：
              旧 B1040-9:  0.171  -> 放开到 0.10 = 悬停的 58%，仍能撑住
              starship:    0.554  -> 放开到 0.10 = 悬停的 18%，净向下 8 m/s^2
        放开到悬停的 18% 等于【主动砸向地面】，与「跟轨迹」的本意相反。

        【修法】放开值用【悬停油门的比例】：
            release = GFOLD_THROTTLE_RELEASE_FRAC * 悬停 * a_cap
        取 FRAC = 0.70 => starship 释放到 0.388（悬停的 70%），
        净向下加速度约 9.81*0.30 = 2.9 m/s^2 —— 能下降，但不会失控。
        旧载具按同样比例得到 0.12，与它原来的 0.10 几乎一致，
        说明【这个比例才是两个载具的共同物理量】。

        【仍保留 1.02*|u_plan| 上界】保证下限永不超过规划要的推力。
        ================================================================
        """
        base = self._gfold_floor(a_cap, mass)
        net_up = float(plan_a_up) - G0
        if float(err_up) > 0.0 or net_up < 0.0:
            # 悬停油门的比例（与 _gfold_floor 同口径：用【本帧实测推力】）
            #   【2026-10-21-e】原用常数 C.TMAX_N，与 a_cap_real 口径解耦，
            #   理由见 _gfold_floor 的长注释。
            hover = (G0 * max(1.0, float(mass))
                     / max(1.0, self.thrust_true()))
            _frac = max(float(C.GFOLD_THROTTLE_RELEASE_FRAC), 0.05)
            # 【注意：不能再与 GFOLD_THROTTLE_RELEASE 取 min】
            #   原来写的是 min(_frac*hover, GFOLD_THROTTLE_RELEASE)，
            #   而 GFOLD_THROTTLE_RELEASE=0.10 是【绝对油门】。
            #   starship: min(0.70*0.554, 0.10) = 0.10 —— 又变回 18% 悬停，
            #   等于本次修复被自己抹掉。护栏的值必须 >= 悬停，否则它恒赢。
            #   现护栏改成「不超过悬停油门本身」（放开下限不该超过悬停，
            #   否则净竖直向上、永远不落地）。
            cap_by_release = min(_frac * hover, hover) * float(a_cap)
            if plan_mag is None or not (float(plan_mag) > 0.0):
                return cap_by_release
            return min(cap_by_release, 1.02 * float(plan_mag))
        return base

    # ------------------------------------------------------------------
    # 3.6 求解（P3 → tf_m → P4）
    # ------------------------------------------------------------------
    def _predict_est_time(self):
        """预测本次解算要多久，作为状态外推时长。

        【为什么不固定】codegen 后单次 0.1~0.4 s。固定 0.5 s 会外推超前，
        轨迹起点比本机靠前，find_nearest_index 给出 fr=-0.5 -> n_i 变负
        -> 触发 n_i<0 覆盖分支 -> 指令跳变。
        """
        last = self.solve_ms / 1000.0
        if last <= 0.05:
            # 【2026-10-21-e 修正 B3：恢复 --est-time 参数】
            #   旧 gfold_land.py:1370-1372 是 return float(self.args.est_time)，
            #   我此前硬编码 0.5 —— 默认值相同、行为一致，但丢了现场可调旋钮。
            #   旧注释 1357-1368 正是在解释「用 0.5 当求解耗时会外推超前
            #   => n_i 变负 => 指令跳变」，所以这个旋钮有实际用途。
            return float(getattr(self.args, 'est_time', 0.5))
        return float(min(6.0, max(0.05, last * 1.2)))

    def vessel_profile1(self, error, vel, acc, mass):
        """二阶外推出求解器要用的初状态 x0（6 维）。

        【为什么必须外推】规划不是瞬间完成的。若把"此刻"状态直接喂给
        求解器，等解出来时火箭已经飞出若干秒 —— 参考轨迹起点与实际
        执行时刻系统性错位，一开始就欠账。
        【必须用预测加速度】一阶外推（acc=0）会把位置沿直线推出去，
        而实际是减速曲线；旧程序实测偏差达 800 m。
        """
        _ = mass
        est_time = float(self.est_time)
        vel_est = np.asarray(vel, float) + np.asarray(acc, float) * est_time
        pos_est = (np.asarray(error, float)
                   + np.asarray(vel, float) * est_time
                   + 0.5 * np.asarray(acc, float) * est_time * est_time)
        return np.array([pos_est[0], pos_est[1], pos_est[2],
                         vel_est[0], vel_est[1], vel_est[2]])

    def hold_accel(self, vel, mass):
        """预测 hold 段的【净加速度】（供首次解算的外推使用）。

        与主循环 hold 段实际下发的指令同一套公式，这样"外推出的状态"
        与"火箭真实会走的状态"一致。
        """
        a_cap = self.a_cap_real(mass)
        thrust = hold_accel_cmd(np.asarray(vel, float), a_cap)
        return thrust + np.array([-G0, 0.0, 0.0])

    def solve(self, x0_state, mass, allow_cvxpy_fallback=True):
        """调用 P3/P4 求解器。返回 dict 或 None。

        【x0 是三维带符号 ENU】[px,py,pz,vx,vy,vz]（目标系 x=上）。
          这是 PD 能直接做 "x_i - error" 相减的前提。
        【末段抬高 TERM_CLEARANCE】规划的归零点抬到 alt=TARGET_ALT+CLEAR，
          否则载具在规划刹停前就触地（旧载具实测落地 -1.6~-10.8 m/s）。
          传入前 h -= CLEAR，取回后 x[0,:] += CLEAR。
        """
        a_net = self.a_cap_real(mass) - G0
        try:
            self.dbg_plan_x0_up = float(x0_state[0])
        except Exception:                        # noqa: BLE001
            self.dbg_plan_x0_up = float('nan')
        alt = float(x0_state[0]) + TARGET_ALT
        # 【C 优化】dist 标量已废弃，改用 signed_range_m()（见下方诊断段）。
        vz = abs(float(x0_state[3]))
        vh = math.hypot(float(x0_state[4]), float(x0_state[5]))
        # 新载具 TWR 只有 1.82，a_net 约 8.09；旧载具 49。阈值相应下调。
        if a_net < 1.0:
            print('[G-FOLD] 预检失败：a_net = %.2f m/s^2（推重比过低）' % a_net)
            print('          mass=%.1f t  available_thrust=%.2f MN'
                  % (mass / 1000.0,
                     float(self.v.available_thrust) / 1e6))
            print('          这艘船物理上无法着陆 —— 多半是【选错了 vessel】')
            self.gfold_status = 'precheck: a_net too low'
            return None
        t0 = time.time()
        # 【求解器推力上限】用实时可用推力（与 a_cap_real 同口径）。
        #   【注意】codegen 的推力上下界是在 MASS_GEN=977 t 处线性化的，
        #   传进来的 t_max 只影响包络里的 a_brk；模型本身用 T_MAX 常数。
        _t_avail = self.thrust_true()
        _x0_solver = np.array(x0_state, dtype=float)
        if TERM_CLEARANCE > 0.0:
            _x0_solver[0] = float(_x0_solver[0]) - float(TERM_CLEARANCE)
        try:
            r = S.solve_p3p4_state(
                _x0_solver, float(mass),
                isp=ISP_DEFAULT,
                t_max=_t_avail,
                # 【求解器节流界，必须与旧程序一致：(0.1, 0.8)】
                #   旧 gfold_land.py:1461 是 (0.1, 0.8)，同处注释 1433-1434 写明
                #   「(0.1,0.8) 是【求解器】的节流界，控制器侧用 [0.05,1.0]，
                #     两套并存是原仓库设计」。
                #   我此前抄成 (0.1, 1.0) 且无任何注释 —— 这在改变规划可行域：
                #   允许 100% 油门规划 => 更短 tf、更陡轨迹，而控制器在姿态/
                #   推力滞后下更难跟。属擅自改动，已改回。
                #   【注意】codegen 的 THROTTLE 是【编译期固化】的，
                #   改这里只影响 cvxpy 回退路径；codegen 路径必须同步改
                #   starship_codegen.THROTTLE 并重新生成。
                throttle=(0.1, 0.8),
                tf_guess=TF_GUESS,
                straight_fac=STRAIGHT_FAC,
                gs_deg=GS_DEG,
                pcs_deg=PCS_END_DEG,
                pcs_start_deg=PCS_START_DEG,
                v_max=V_MAX_SOLVER,
                N3=N3, N4=N4, target_alt=TARGET_ALT,
                v_descent_max=C.vdesc_max(vz),
                term_win=V_DESCENT_TERM_WIN,
                term_vz=V_DESCENT_TERM_VZ,
                allow_cvxpy_fallback=allow_cvxpy_fallback)
        except Exception as e:                   # noqa: BLE001
            self.solve_ms = (time.time() - t0) * 1000.0
            self.gfold_status = 'exc: %s' % e
            print('[G-FOLD] 求解异常: %s: %s' % (type(e).__name__, e))
            return None
        self.solve_ms = (time.time() - t0) * 1000.0
        self.gfold_status = r.get('status', 'err')
        # 【改动 2 配套】把规划高度基准平移回来（+TERM_CLEARANCE）
        if TERM_CLEARANCE > 0.0 and r.get('x') is not None:
            try:
                r['x'] = np.asarray(r['x'], float).copy()
                r['x'][0, :] = r['x'][0, :] + float(TERM_CLEARANCE)
            except Exception:                    # noqa: BLE001
                pass
        try:
            self.dbg_plan_tf_used = float(r.get('tf', float('nan')))
        except Exception:                        # noqa: BLE001
            self.dbg_plan_tf_used = float('nan')
        self.dbg_vdesc = float(C.vdesc_max(vz))
        if r.get('status') != 'optimal':
            h_gnd = max(1.0, alt - TARGET_ALT)
            a_v = vz * vz / (2 * h_gnd)
            # 【C 优化 2026-10-20-c】水平需求必须用【有符号到达距离】。
            #   原式 vh^2/(2*dist) 用标量 dist，飞过目标后 dist 从 3 涨到 111，
            #   仍按「要在这个距离内刹停」算，实测给出过 2172 m/s^2 的荒谬值，
            #   把排障方向完全带偏（看上去像能量不足，其实是已经飞过了）。
            #   现在：s_range <= 0 说明目标在速度反方向 => 判为「不可达」，
            #         打印 inf 并明确说明原因；否则用夹过下限的分母。
            s_range = signed_range_m(
                (float(x0_state[1]), float(x0_state[2])),
                (float(x0_state[4]), float(x0_state[5])))
            a_h = brake_accel_needed(vh, s_range)
            _reachable = (s_range > 0.0)
            a_need = (math.hypot(a_v, a_h) if _reachable
                      else float('inf'))
            print('[G-FOLD] solver=%s  a_net=%.1f' % (self.gfold_status, a_net))
            print('[G-FOLD] 分轴需求: 垂直%.0f需%.1f | 水平%.0f需%.1f'
                  ' (到达距离 %+.1f m%s)'
                  % (vz, a_v, vh,
                     (a_h if _reachable else float('nan')), s_range,
                     '' if _reachable else ' 已越过目标，横向不可达'))
            if _reachable:
                print('[G-FOLD] 矢量合成 %.1f m/s^2 / 可用 %.1f (比值 %.2f)'
                      % (a_need, a_net, a_need / max(1e-6, a_net)))
            else:
                print('[G-FOLD] 水平已越过目标点，仅评估竖直通道:'
                      ' 需 %.1f / 可用 %.1f' % (a_v, a_net))
                a_need = a_v          # 后续判断只看竖直
            if a_need > a_net * 0.95:
                print('[G-FOLD] >> 交班点能量偏高：所需 > 可用。'
                      '应在更早/更高处交班，或由 kOS 侧多消横向速度。')
            else:
                print('[G-FOLD] >> 能量本身够，是求解器约束或轨迹形状问题'
                      '（看 v_descent_max 包络 / 门的高度档）。')
            return None
        return r

    # ------------------------------------------------------------------
    # 3.7 索引：最近点投影 / 采样 / 前瞻
    # ------------------------------------------------------------------
    def find_nearest_index(self, x, r, v, tf, N):
        """参考仓库 demo3_gfold.py 的 find_nearest_index（逐行移植）。

        原版：
            nearest_mag = norm(x[0:3, 0] - r); nearest_i = 0
            for i ...: mag = norm(x[0:3,i]-r); if mag < nearest_mag: ...
            v = x[3:6, nearest_i]; v_norm = norm(v); v_dir = v/v_norm
            frac = clamp(dot(r - x[0:3,nearest_i], v_dir)/(tf/N*v_norm), 0.5, -0.5)
            return nearest_i + frac

        【本项目补充 1：度量里加回速度项】
          上游 line 262 本来就有 norm(x[3:6,i]-v)*0.2，但被作者注释掉了。
          纯位置度量下，只要载具越过轨迹最近点，argmin 就永久粘住。
          加回速度项后，位置近但速度方向已明显不符的节点不再被选中。

        【本项目补充 2：除零保护】v_norm -> 0 时返回 nearest_i（原版会 inf/nan）
        """
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
        if v_norm < 1e-6:
            return float(nearest_i)
        v_dir = vv / v_norm
        denom = (tf / N) * v_norm
        if abs(denom) < 1e-12:
            return float(nearest_i)
        frac = K._clamp(float(np.dot(r - x[0:3, nearest_i], v_dir)) / denom,
                        0.5, -0.5)
        return nearest_i + frac

    def sample_index(self, x, u, index, tf, N):
        """参考仓库 demo3_gfold.py 的 sample_index（逐行移植）。

        【逐字保留的两个细节】
          · index >= N-1 时返回 (0,0,0)/(0,0,0)/(g0,0,0) —— 到达处理
          · index < 0 时 u 用 u[:,1] —— 开头处理

        【第十五步 15(b) 修正：到达哨兵不得导致半空悬停】
          原行为：索引一打满就返回 x=0/v=0/u=[g0,0,0]，语义是"任务完成"。
          旧程序实飞事故：n_i 在 58% 的帧打满 N-1，载具在 alt=21.8 m
          （瞄准点 17 m）悬停 27.1 秒，vh 从未消掉。
          => 到达判据改为【索引打满 且 确实已到达】（高度+两轴速度都小），
             否则继续用规划【末端节点】的参考值，让 PD 把剩下的高度走完。
        """
        _ = tf, N
        if index >= N - 1:
            _i_last = int(max(0, N - 2))
            _x_last = np.asarray(x[:, _i_last], float)[0:3].copy()
            _v_last = np.asarray(x[:, _i_last], float)[3:6].copy()
            _u_last = np.asarray(u[:, _i_last], float).copy()
            if self._sample_arrived_ok:
                return (np.zeros(3), np.zeros(3), np.array([G0, 0.0, 0.0]))
            return (_x_last, _v_last, _u_last)
        if index <= 0:
            i = 0
            frac = index
        else:
            i = int(math.floor(index))
            frac = index - i
        x_i_s = K._lerp(x[:, i], x[:, i + 1], frac)
        u_i_s = K._lerp(u[:, i], u[:, i + 1], frac)
        if index < 0:
            u_i_s = u[:, 1].copy()
        return (x_i_s[0:3].copy(), x_i_s[3:6].copy(), u_i_s.copy())

    def _lead_index(self, n_i, tf, N):
        """把采样索引沿规划向前推 t_lead 秒（补偿机身姿态滞后）。

        【限幅】下限 0（不能取到规划起点之前）；上限 N-2（避开末端退化
        节点）；t_lead <= TILT_LEAD_MAX_FRAC*tf（防 tf 异常时跨度过大）。
        """
        if not TILT_LEAD_ENABLE:
            return n_i
        try:
            _tf = float(tf)
            if not (_tf > 1e-6):
                return n_i
            _lead = min(float(TILT_LEAD_SEC), TILT_LEAD_MAX_FRAC * _tf)
            _dn = _lead * float(N - 1) / _tf        # 秒 -> 节点数
            return max(0.0, min(float(N) - 2.0, float(n_i) + _dn))
        except Exception:                        # noqa: BLE001
            return n_i
    # ------------------------------------------------------------------
    # 3.8 姿态辅助：参考平移限速 / 权限调度 / 求解器锥角
    # ------------------------------------------------------------------
    def _limit_ref_slew(self, want_dir):
        """把【姿态参考方向】的转动速率限到本帧权限允许的范围内。

        【限速上限怎么算】slew_max = sqrt(2*alpha*headroom) [rad/s]
          物理意义：从静止加速到 slew_max 再以同样减速度停住，总角行程
          正好 = headroom。按此限速，参考永远停在能力之内。
        【实现】把 want_dir 用球面插值往 current 方向回拉；不足则原样
          返回。输出始终是单位向量且斜率有界 => 无拐点 => D 项干净。
        【为什么不限标量方位角速率】旧程序实测：方位角的三维代价是
          sin(tilt)*d_az，所有 60~175 deg 的方位角突变都发生在 tilt 1~9 deg
          （姿态上本来无害）。限它会让水平推力方向被指错，反而加剧振荡。
          要限就限【三维姿态参考的角速率】，也就是本函数。
        """
        if not REF_SLEW_ENABLE:
            return want_dir
        # 【ref_clip 必须每帧复位】旧程序踩过：一旦某帧置 1 就永不复位，
        #   实测读数 97.8% 而实际只有少数帧被限 —— 纯属误读。
        self.dbg_ref_clamped = 0.0
        try:
            _w = np.asarray(want_dir, dtype=float)
            _n = float(np.linalg.norm(_w))
            if not (_n > 1e-9):
                return want_dir
            _w = _w / _n
            _prev = self._ref_slew_prev
            if _prev is None:
                self._ref_slew_prev = _w.copy()
                return _w
            _dt = float(getattr(self, 'last_pkt', 0.0))
            if not (_dt > 1e-9):
                self._ref_slew_prev = _w.copy()
                return _w
            _av = float(self.dbg_alpha_max)
            _a = math.radians(_av) if _av == _av else 0.0
            _head = math.radians(REF_SLEW_HEADROOM_DEG)
            if _a > 1e-9:
                _slew_deg = math.degrees(math.sqrt(2.0 * _a * _head))
            else:
                _slew_deg = REF_SLEW_MIN
            _slew_deg = max(REF_SLEW_MIN, min(REF_SLEW_MAX, _slew_deg))
            _slew = math.radians(_slew_deg)
            _ang = math.acos(max(-1.0, min(1.0, float(np.dot(_prev, _w)))))
            _step = _slew * _dt
            if _ang <= _step or _ang < 1e-9:
                self._ref_slew_prev = _w.copy()
                self.dbg_ref_slew = math.degrees(_ang) / _dt
                return _w
            _t = _step / _ang
            _out = _prev + _t * (_w - _prev)
            _on = float(np.linalg.norm(_out))
            _out = _out / _on if _on > 1e-9 else _prev.copy()
            self._ref_slew_prev = _out.copy()
            self.dbg_ref_slew = math.degrees(_step) / _dt
            self.dbg_ref_clamped = 1.0
            return _out
        except Exception:                        # noqa: BLE001
            return want_dir

    def _att_rate_phys(self):
        """本帧机身的【物理角速率上限】[rad/s]，用于 D 项归一化。

        【定义】从静止加速到该速率、再以同等减速度停住，总角行程正好等于
            REF_SLEW_HEADROOM_DEG。即
                omega_phys = sqrt(2 * alpha_now * headroom)
            这是被控对象在本帧油门下的【真实可达速率】，不是拍脑袋的标度。

        【为什么用它做 D 项分母（见 CTRL_X_ROT_KD 的长注释）】
            绝对角速度喂 D 项时，D = kd_eff*omega 必须 <= 1 才不饱和，
            于是 kd_eff <= 1/omega_peak，zeta 被物理锁死。
            换成 omega/omega_phys 后，D 的含义变成"用掉了多少比例的物理
            速率余量"，kd 与载具的绝对角速率峰值脱钩 —— 换载具不必重算。

        【退化处理】alpha 采不到（None/NaN/<eps）时返回 0，调用方会退回
            绝对角速度，绝不让 scale=0 把 D 项放大到无穷。
        """
        try:
            _av = float(getattr(self, 'dbg_alpha_max', float('nan')))
            if not (_av == _av) or _av <= 1e-9:
                return 0.0
            _a = math.radians(_av)
            _head = math.radians(REF_SLEW_HEADROOM_DEG)
            _w = math.sqrt(2.0 * _a * _head)
            return float(_w) if (_w == _w and _w > 1e-9) else 0.0
        except Exception:                        # noqa: BLE001
            return 0.0

    def _att_sched_factor(self):
        """姿态环增益调度系数（随权限变化，防低油门端过冲）。

        【原理】二阶姿态模型 e'' = -alpha*(kp*e + kd*e')
            wn = sqrt(alpha*kp)      zeta = (kd/2)*sqrt(alpha/kp)
          令 kp 正比于 alpha，则 alpha/kp 为常数 => zeta 与 alpha 无关。
        【为什么用 sqrt】线性调度会让 kp 在低油门端掉到很小、高油门端
          涨到远超饱和阈值。平方根把变化范围从 8 倍压到 2.8 倍。
        【参考权限】旧载具 20 deg/s^2；新载具只有 3.83 deg/s^2。
          若仍用 20，_fac 恒被 MIN 夹住，调度器反而把增益砍掉 65%。
        """
        if not TAU_SCHED_ENABLE:
            return 1.0
        try:
            a_now = float(self.dbg_alpha_max)
            if not (a_now == a_now) or a_now <= 1e-9:
                return 1.0
            _fac = math.sqrt(a_now / max(0.1, float(TAU_SCHED_ALPHA_REF)))
            return float(max(TAU_SCHED_MIN, min(TAU_SCHED_MAX, _fac)))
        except Exception:                        # noqa: BLE001
            return 1.0

    def _solver_cone_deg(self, index, N):
        """求解器那条【推力指向锥】在第 index 个节点处的半角 [deg]。

        【为什么要与求解器完全一致】求解器施加的约束是
            u[0,n] >= cos(ang(n)) * |u[:,n]|,
            ang(n) = PCS_START_DEG + (PCS_END_DEG-PCS_START_DEG)*n/(N-2)
        这是【规划可行域的边界】。控制器若用另一条曲线做限幅，两者就
        自相矛盾：限得比它紧 -> 参考轨迹自己被剪掉（不跟踪）；
        限得比它松 -> 允许规划从未验证过的姿态（姿态失守）。
        """
        _N = max(2, int(N))
        frac = float(index) / max(1.0, float(_N - 2))
        frac = max(0.0, min(1.0, frac))
        return float(PCS_START_DEG + (PCS_END_DEG - PCS_START_DEG) * frac)
    def conic_clamp(self, target_a, min_mag, max_mag, tilt_deg=None,
                    min_mag_override=None, plan_a_up=None):
        """把推力加速度指令限幅到推力锥内（逐行照抄参考仓库 conic_clamp）。

        原版（demo3_gfold.py:292-314）：
            a_mag = norm(target_a); hor_dir = (0, a[1], a[2])/norm(...)
            a_hor = norm(target_a[1:3]); a_ver = target_a[0]
            if a_hor < min_mag*sin(max_tilt): a_ver_min = sqrt(min_mag^2-a_hor^2)
            else: a_ver_min = cos(max_tilt)*min_mag
            （max_mag 同理）
            a_ver = clamp(a_ver, a_ver_max, a_ver_min)
            a_hor = min(a_hor, a_ver*tan(max_tilt))
            return hor_dir*a_hor + (a_ver,0,0)

        【唯一改动 1：零向量保护】原版 hor_dir /= norm(hor_dir) 在水平分量
          为 0 时给 nan。
        【唯一改动 2：尊重传入的 min_mag】原版签名收了 min_mag 却在体内
          用自己的下限重算 —— 那样 track() 里算好的【条件下限】会被二次
          改写，放开逻辑完全失效（旧程序踩过）。
        【唯一改动 3：竖直保底与锥角解耦】cos(cone)*min_mag 在 cone=85 deg
          时只剩 0.087 倍，等于没有保护（实测 a_up 塌到 0.46）。加一个与
          锥角无关的保底，且【绝不超过规划自己的 a_ver】。
        """
        _ = max_mag
        a_cap = self.a_cap_real()
        if min_mag_override is not None:
            min_mag = float(min_mag_override)
        else:
            min_mag = self._gfold_floor(a_cap, self.v.mass)
        max_mag = 1.0 * a_cap
        max_tilt = math.radians(CONIC_TILT_DEG if tilt_deg is None else tilt_deg)
        a_hor = float(np.linalg.norm(target_a[1:3]))
        a_ver = float(target_a[0])
        # 竖直保底：绝不超过规划自己的 a_ver，且必须 < g0
        _floor_extra = 0.0
        if (AVER_FLOOR_ENABLE and plan_a_up is not None
                and float(plan_a_up) == float(plan_a_up)):
            _cap = min(float(AVER_FLOOR), float(plan_a_up))
            _cap = min(_cap, G0 * 0.99)        # 硬约束：必须 < g0
            _floor_extra = max(0.0, _cap)
        if a_hor < 1e-9:
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
        a_ver = max(a_ver_min, min(a_ver_max, a_ver))
        a_hor = min(a_hor, a_ver * math.tan(max_tilt))
        return hor_dir * a_hor + np.array([a_ver, 0.0, 0.0])
    # ------------------------------------------------------------------
    # 3.9 G-FOLD 跟踪律（单一 tilt-first，逐行照抄旧程序 track()）
    # ------------------------------------------------------------------
    def track(self, r, tp, tv, n_i):
        """G-FOLD 跟随：单一 tilt-first 律。返回 (target_a, n_i)。

        ====================================================================
        【结构（与旧程序 gfold_land.py:tracks 逐段对应）】
          ① 索引推进：几何投影 + 单调 + 夹 N-2
          ② 前赡点采样（补偿机身姿态滞后）
          ③ 水平律：eguidance  a_h_vec = u_h + kv*dve + kp*dp
                        kv = 4/T, kp = 6/T^2 => zeta = 0.816 与 T 无关
          ④ 倾角上限（随高度）+ 终端收倾角（保留水平权限下限）
          ⑤ 方位角：与幅值同源（取合矢量方向）
          ⑥ 竖直律：真正的竖直 PD（规划竖直 + K_VEL*dvz + K_POS*dalt）
          ⑦ 水平能力夹限：a_h <= a_up*tan(tilt_allow)
          ⑧ 内环：限 a_h 的【加加速度】，j_max 由本帧权限在线导出
          ⑨ 由 a_h 反解倾角（与 a_up 同源，消除往返失配）
          ⑩ 构造指令（一次成型）
          ⑪ conic_clamp 最终安全限幅
          ⑫ 姿态参考方向速率限制
        ====================================================================
        """
        x = r['x']
        u = r['u']
        N = x.shape[1]
        tf = float(r['tf'])
        error = np.array(tp, dtype=float)
        vel = np.array(tv, dtype=float)

        # ---------------- ① 索引推进 ----------------
        dt_game = float(getattr(self, 'last_pkt', 0.02))
        _raw_near = self.find_nearest_index(x, error, vel, tf, N)
        self.gfold_traj_ready = _raw_near > -0.05
        _n_from_decay = n_i - dt_game * 0.2 * N / tf
        self.gfold_n_i = max(_n_from_decay, _raw_near)
        #   索引是沿轨迹的弧长参数，物理上时间不倒流；实测推进速率只有
        #   标称的 67%（载具一直落在轨迹后方），允许回退会加剧追不上。
        if IDX_MONOTONIC:
            self.gfold_n_i = max(self.gfold_n_i, float(n_i))
        #   【绝不能用时间增量推进索引】旧程序实飞 3/3 趟全败：
        #   dt_game 不可信（实测 max 20.6 s 而真实帧间隔约 0.16 s），
        #   索引一帧就冲到末端，制导以为已到达。
        self.gfold_n_i = max(0.0, self.gfold_n_i)
        #   夹到 N-2：最后一个节点（N-1）是求解器的退化占位。
        self.gfold_n_i = min(self.gfold_n_i, float(N) - 2.0)
        self.dbg_idx_raw = float(_raw_near)
        if abs(self.gfold_n_i - float(n_i)) < 1e-9:
            self.idx_frozen_n += 1
        else:
            self.idx_frozen_n = 0
        self.dbg_idx_frozen = float(self.idx_frozen_n)
        #   真的已到达判据（供 sample_index 的哨兵分支）：三者缺一不可。
        _arr_alt = float(error[0]) + TARGET_ALT
        _arr_vz = abs(float(vel[0]))
        _arr_vh = float(np.linalg.norm(vel[1:3]))
        self._sample_arrived_ok = (_arr_alt < (TARGET_ALT + 2.0)
                                   and _arr_vz < 1.5 and _arr_vh < 1.5)
        self.dbg_tf_arrived = 1.0 if self._sample_arrived_ok else 0.0
        x_i, v_i, u_i = self.sample_index(x, u, self.gfold_n_i, tf, N)

        # ---------------- ② 前赡点采样 ----------------
        _n_look = self._lead_index(self.gfold_n_i, tf, N)
        if abs(_n_look - self.gfold_n_i) > 1e-9:
            x_r, v_r, u_r = self.sample_index(x, u, _n_look, tf, N)
        else:
            x_r, v_r, u_r = x_i, v_i, u_i
        self.dbg_tf_lead_n = float(_n_look - self.gfold_n_i)

        # ---------------- ③ 水平律：eguidance ----------------
        _dp = (np.asarray(x_r, float) - error)[1:3]
        _dve = (np.asarray(v_r, float) - vel)[1:3]
        _perr = float(np.linalg.norm(_dp))
        _verr = float(np.linalg.norm(_dve))
        self.dbg_tf_perr = _perr
        self.dbg_tf_verr = _verr
        self.dbg_dp_n = float(_dp[0])
        self.dbg_dp_e = float(_dp[1])
        self.dbg_dve_n = float(_dve[0])
        self.dbg_dve_e = float(_dve[1])
        # 【C 优化】有符号到达距离：正=目标在速度前方，负=已越过。
        #   这是「还能不能横向到达」的判据，与标量 dist_hz 互补：
        #     dist_hz 回答「离目标多远」
        #     s_range 回答「沿运动方向还剩多少可制动距离」
        self.dbg_s_range = signed_range_m(
            (float(error[1]), float(error[2])),
            (float(vel[1]), float(vel[2])))
        _ca_T = 2.0 * _perr / max(0.05, _verr)
        _ca_T = max(T_GAIN_MIN, min(T_GAIN_MAX, _ca_T))
        _ca_kv = 4.0 / _ca_T
        _ca_kp = 6.0 / (_ca_T * _ca_T)
        if TILT_GAIN_CAP_ENABLE:
            _ca_kv = min(TILT_GAIN_MAX, _ca_kv)
        _ah_vec = (np.asarray(u_r, float)[1:3].copy()
                   + _ca_kv * _dve + _ca_kp * _dp)
        _ah_dem = float(np.linalg.norm(_ah_vec))
        self.dbg_ca_T = float(_ca_T)
        self.dbg_ca_kv = float(_ca_kv)
        self.dbg_ca_kp = float(_ca_kp)
        self.dbg_ah_dem = _ah_dem
        #   规划自己的竖直分量，作为倾角需求的分母（唯一规划竖直出处）。
        _a_up_est = max(1.0, abs(float(u_i[0])))
        _tcorr = math.degrees(math.atan2(_ah_dem, _a_up_est))
        self.dbg_tf_tilt_corr = float(_tcorr)
        _tilt_plan = math.degrees(math.atan2(
            float(np.linalg.norm(np.asarray(u_i, float)[1:3])),
            max(1e-9, float(u_i[0]))))
        self.dbg_tf_tilt_plan = float(_tilt_plan)

        # ---------------- ④ 倾角上限（随高度）----------------
        _alt_now = float(error[0]) + TARGET_ALT
        _tcap = max(TILT_CAP_LO,
                    min(TILT_CAP_HI, _alt_now / max(1e-6, TILT_CAP_SLOPE)))
        self.dbg_tf_cap = float(_tcap)
        #   终端收倾角：低空收缩【允许的】倾角，但保留水平权限下限
        #   （横速还没消掉时不许把允许倾角压到 TILT_CAP_LO 以下）。
        #   【关键】收缩的是【能力上限】，不是需求。若写成
        #   _tilt_allow = min(_tcap, 需求)，上限恒等于需求 => ah_sat 每帧
        #   都是 1，能力夹限形同虚设，内环限速也完全失效。
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
        _tilt_allow = max(min(TILT_CAP_LO, _tcap), min(_tcap, _tilt_allow))
        self.dbg_tf_tilt_want = float(_tcorr)

        # ---------------- ⑤ 方位角：与幅值同源 ----------------
        #   【2026-10-21-m 已回退】曾试过「拒绝方位角跳变」(AZ_JUMP_MAX_DEG=60)，
        #     实飞 205641 灾难性失败：84% 帧被拒 => 方位角永久冻结 =>
        #     dist 7 -> 308 m，vh 23 m/s，FIN 超时。**已恢复到原逻辑。**
        #   【教训】180 度翻转不是 bug，是载具【飞过目标后转回来】的唯一途径。
        #     冻住它等于让载具永远朝错误方向飞。**不要再加跳变拒绝。**
        _az = self.tf_az_dir
        if _ah_dem > 1e-9 and _perr > TILT_AZ_MIN_ERR:
            _az = (_ah_vec / _ah_dem).copy()
            self.tf_az_dir = _az
        if _az is None:
            _az = np.array([0.0, 0.0])
            _eh = np.asarray(error, float)[1:3]
            _ehn = float(np.linalg.norm(_eh))
            if _ehn > 1e-9:
                _az = -_eh / _ehn
            self.tf_az_dir = _az
        _azn = float(np.linalg.norm(_az))
        if _azn > 1e-9:
            _az = _az / _azn
        else:
            _az = np.array([0.0, 0.0])
            self.tf_az_dir = _az
        self.dbg_tf_az_n = float(_az[0])
        self.dbg_tf_az_e = float(_az[1])
        self.tf_ah_dir = _ah_vec / _ah_dem if _ah_dem > 1e-9 else None
        try:
            if self.tf_ah_dir is not None and _perr > 1e-6:
                self.dbg_tf_ahdir_cos = float(
                    np.dot(np.asarray(self.tf_ah_dir, float), _dp) / _perr)
            else:
                self.dbg_tf_ahdir_cos = float('nan')
        except Exception:                        # noqa: BLE001
            self.dbg_tf_ahdir_cos = float('nan')
        # ---------------- ⑥ 竖直律：真正的竖直 PD ----------------
        #   【重写要点】旧版这里是 a_up = |target_a|*cos(tilt)，幅值来自旧律
        #   PD 向量；本版改为【规划竖直 + 竖直 PD】，与水平通道完全独立。
        #   【必须用当前节点，不能用前赡点】前赡点是补偿【姿态】滞后的，
        #   而油门没有这种滞后。若竖直 PD 也拿前赡点作参考，就等于让推力
        #   去追一个还没到的未来状态，产生系统性过刹/欠刹（实测落地
        #   -24.0 m/s 而规划末节点只有 -2.6）。
        _h_now = max(0.0, float(error[0]))
        _vz_now = float(np.asarray(vel, float)[0])
        _p0 = float(u_i[0])
        # ================================================================
        # 末段下降律：h < TERM_CLEARANCE 时不再跟规划竖直剖面，改为速度环，
        #   目标是 TERM_DESCENT_VZ。否则规划要求 v=0，载具会在抬高点悬停
        #   （旧程序实测 211419 在 alt=21.8 m 悬停 27.1 s）。
        #   【必须用满刹车而不是弱 P 环】初版写 g0+K_VEL*(vz_tgt-vz)，实测
        #   进入末段时 vz 已到 -10.4，该式给 16.5 而物理需要 20.3
        #   => 落地 vz 仍达 -12.2。故直接置 +inf，由下面的 a_ver_need 夹住。
        # ================================================================
        _term_phase = (TERM_CLEARANCE > 0.0 and _h_now < TERM_CLEARANCE)
        if _term_phase:
            _a_up_raw = float('inf')
        else:
            _a_up_raw = (_p0
                         + K_VEL * (float(v_i[0]) - _vz_now)
                         + K_POS * (float(x_i[0]) - float(error[0])))
        #   上界 a_ver_need = 要在剩余高度 h 内把 vz 刹到目标垂速所需的值。
        #   施加超过它就等于刹得比需要更狠 => 停住/往上飞。
        #   => a_up <= a_ver_need 是保证会落地的充要条件。
        _h_req = max(AVER_H_FLOOR, _h_now)
        if _vz_now > 0.0:
            #   上升保护：需求公式在 vz>0 时给出 >g0 的上界，min() 会保留
            #   向上的指令 => 数值发散（旧程序实测从 h=3/vz=+5 发散到 624 km）。
            _a_ver_need = G0 - AVER_CLIMB_K * _vz_now - AVER_CLIMB_MARGIN
        else:
            _a_ver_need = (G0 + (_vz_now * _vz_now - AVER_TGT_VZ * AVER_TGT_VZ)
                           / (2.0 * _h_req))
        _a_up = min(_a_up_raw, _a_ver_need)
        #   正向下限：油门下限对应的加速度，保证姿态权限不会塌到 0
        #   （旧程序实测 36/70 帧零油门自由落体，7.2 s 掉 292 m）。
        _min_mag = self._gfold_floor_effective(
            self.a_cap_real(), self.v.mass, _p0,
            float(error[0]) - float(x_i[0]),
            float(np.linalg.norm(np.asarray(u_i, float))))
        self.min_mag_eff = float(_min_mag)
        self.dbg_min_mag_eff = float(_min_mag)
        #   下限与上界可能冲突（need 很小时）：此时以不违反 a_ver_need 为准，
        #   因为保证会落地比托高姿态权限更不能牺牲。
        #   【注意 g0*0.99 只用于给【保底值】封顶，绝不是给 a_up 封顶】
        #     若把 a_up 整体限到 < g0，净竖直恒为负、永远无法减速
        #     （旧程序实测该错误会让落地垂速达到 -116 m/s）。
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
        self.dbg_tf_cap = float(_tilt_allow)
        _a_h_max = _a_up * math.tan(math.radians(_tilt_allow))
        if _ah_dem > _a_h_max and _a_h_max > 1e-9:
            _ah_want = _a_h_max
            self.dbg_ah_sat = 1.0
        else:
            _ah_want = _ah_dem
            self.dbg_ah_sat = 0.0

        # ---------------- ⑧ 内环：限 a_h 的加加速度 ----------------
        #   由倾角速率限制换算：a_h = a_up*tan(theta) =>
        #       d(a_h)/dt = a_up * sec^2(theta) * dtheta/dt
        #   取物理判据 slew = sqrt(2*alpha*headroom)，则
        #       j_max = a_up * sec^2(theta) * sqrt(2*alpha*headroom)
        #   随 alpha（正比油门、实测 8 倍变化）在线导出，不是固定常数。
        _alpha_d = float(self.dbg_alpha_max)
        if _alpha_d != _alpha_d:
            _alpha_d = 0.0
        _sec2 = 1.0 + (_ah_want / max(1e-9, _a_up)) ** 2
        if _alpha_d > 1e-9:
            _j_phys = (_a_up * _sec2 * math.sqrt(
                2.0 * math.radians(_alpha_d)
                * math.radians(REF_SLEW_HEADROOM_DEG)))
        else:
            _j_phys = JERK_GUARD_MIN
        _j_lim = _j_phys * max(0.05, min(1.0, float(JERK_SLEW_FRAC)))
        _j_max = max(JERK_GUARD_MIN,
                     min(JERK_GUARD_MAX, _j_phys, _j_lim))
        self.dbg_j_phys = float(_j_phys)
        self.dbg_j_lim = float(_j_lim)
        _dt_j = float(getattr(self, 'last_pkt', 0.02))
        if not (_dt_j > 1e-9):
            _dt_j = 0.02
        #   【内环状态必须是真正发出去的 a_h】
        #     初版把能力夹限放在限速【之后】，状态与实际输出脱节：
        #     当 a_h_max 一帧内收缩时，输出被拉低的幅度不受 j_max 约束，
        #     实测 j_used 冲到 39.0（j_max 只有 4.97），限加加速度形同虚设。
        #   【修法】先算目标，再做限速，且把限速结果作为状态。
        _ah_target = max(0.0, min(_ah_want, _a_h_max))
        _ah_prev = float(getattr(self, 'tf_ah_prev', 0.0))
        _step_j = _j_max * _dt_j
        _ah_lim = _ah_prev + max(-_step_j, min(_step_j, _ah_target - _ah_prev))
        _ah_lim = max(0.0, _ah_lim)
        #   能力上限是硬安全限，可以覆盖加加速度限制（它依赖本帧 a_up 与
        #   倾角上限，二者都可能一帧内下降；此时少给水平加速度是物理事实）。
        _ah_cmd = min(_ah_lim, _a_h_max)
        _cap_bind = 1.0 if (_a_h_max < _ah_lim - 1e-9) else 0.0
        self.tf_ah_prev = _ah_cmd
        self.dbg_ah_cmd = float(_ah_cmd)
        self.dbg_j_max = float(_j_max)
        self.dbg_j_used = (_ah_cmd - _ah_prev) / _dt_j
        self.dbg_ah_cap_bind = _cap_bind
        self.dbg_tf_slew_allow = float(_j_max)

        # ---------------- ⑨ 由 a_h 反解倾角（口径闭环）----------------
        #   【关键】tilt 与 a_h 用【同一个 a_up】，不再有前向 a_up_est /
        #   后向 av_up 的往返失配（旧版该失配 p10=0.518 / p90=1.721）。
        _tilt_cmd = math.degrees(math.atan2(_ah_cmd, max(1e-9, _a_up)))
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
        #   本版给它的输入是由 (a_up, a_h, az) 直接构造的向量，
        #   不再含旧律 PD 的幅值，故不存在幅值串扰。
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
        #   a_h == a_up*tan(tilt) 与 az 来自同一矢量 两条恒等式成立。
        _a_up_f = max(0.0, float(target_a[0]))
        _ah_v = np.asarray(target_a, float)[1:3]
        _a_h_f = float(np.linalg.norm(_ah_v))
        if _a_h_f > 1e-9:
            _az = (_ah_v / _a_h_f).copy()
            self.tf_az_dir = _az
        _tilt_f = math.degrees(math.atan2(_a_h_f, max(1e-9, _a_up_f)))
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
        if _dn > 1e-9:
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
        self.dbg_tf_tilt_j = float(_tilt_plan)
        self.dbg_ca_posclip = 0.0

        #   水平指令分项：把指令拆成三项，各自与速度方向求夹角余弦。
        #   口径：取【限幅前】的 eguidance 矢量（那才是律真正算出的东西）。
        try:
            _uh_vec = np.asarray(u_r, float)[1:3]
            _dv_vec = (np.asarray(v_r, float) - vel)[1:3]
            _dp_v = (np.asarray(x_r, float) - error)[1:3]
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
                            if n > 1e-9 else float('nan'))

                self.dbg_u_dot = _dot_with_vel(_uh_vec)
                self.dbg_pdv_dot = _dot_with_vel(_dv_vec)
                self.dbg_pdp_dot = _dot_with_vel(_dp_v)
                _cmd_h = np.asarray(target_a, float)[1:3]
                _cn = float(np.linalg.norm(_cmd_h))
                self.dbg_cmd_dot = (float(np.dot(_cmd_h, _vd)) / _cn
                                    if _cn > 1e-9 else float('nan'))
            else:
                self.dbg_u_dot = float('nan')
                self.dbg_pdv_dot = float('nan')
                self.dbg_pdp_dot = float('nan')
                self.dbg_cmd_dot = float('nan')
        except Exception:                        # noqa: BLE001
            self.dbg_u_dot = float('nan')
            self.dbg_pdv_dot = float('nan')
            self.dbg_pdp_dot = float('nan')
            self.dbg_cmd_dot = float('nan')

        #   水平误差方向（幅值之外，回答往哪边偏）
        _he = np.array([float(error[1]), float(error[2])])
        _hd = np.array([float(target_a[1]), float(target_a[2])])
        _he_norm = float(np.linalg.norm(_he))
        _hd_norm = float(np.linalg.norm(_hd))
        self.dbg_he_n = float(_he[0])
        self.dbg_he_e = float(_he[1])
        if _hd_norm > 1e-9:
            self.dbg_hd_n = float(_hd[0] / _hd_norm)
            self.dbg_hd_e = float(_hd[1] / _hd_norm)
        else:
            self.dbg_hd_n = 0.0
            self.dbg_hd_e = 0.0
        if _he_norm > 1e-6 and _hd_norm > 1e-9:
            _want = -_he / _he_norm
            _cosm = float(np.dot(_hd / _hd_norm, _want))
            self.dbg_h_misalign = math.degrees(
                math.acos(max(-1.0, min(1.0, _cosm))))
        else:
            self.dbg_h_misalign = float('nan')

        self.a_cmd_vec = target_a
        return target_a, self.gfold_n_i

    # ------------------------------------------------------------------
    # 3.10 FIN 触地保持段
    # ------------------------------------------------------------------
    def fin_control(self, alt, dist, vz, vh, vmag, mass, error, vel,
                    game_dt=0.02):
        """FIN 触地保持段的控制律（最简单的 PID）。返回 (target_a, dir)。

        【它取代了什么】原实现在 alt <= TARGET_ALT+1 时 break + 油门清零 +
        退出程序。视频证实后果是：引擎熄灭 -> 静止约 1 s -> 缓慢单向倾覆。
        日志证据：末帧仍有 vh=1.10~3.77 m/s、tilt=5.5~6.0 deg，
        而既有 _term_phase 明确【水平不动】=> 横速没被消掉。

        【本律做什么】
          竖直：a_up = a_need - FIN_KD*d(vz)/dt，再夹到
                  [FIN_UP_MIN*g0, a_need]。a_need 是【在剩余高度内恰好把
                  vz 收到目标垂速】所需的推力，它本身就是一条
                  【增益 = 1/(2h) 的比例律】，比例作用已由高度调度承担。
          水平：a_h = -FIN_VH_KP*vh_vec - FIN_VH_KD*d(vh_vec)/dt
                  方向为水平速度反方向（主动消残余横速）。
          姿态：瞄准天顶，倾角与所需横向推力成比例（不固定取满）。

        【为什么竖直不能用固定增益 P 环】要在剩余高度 h 内把 vz 从 -10.4
        收到 -2，所需推力相对 vz 的【等效增益】= vz/h，h->0 时无界增长。
        任何固定增益都必然在接近地面时偏小 => 一路欠刹车 => 硬着陆。
        离线实测：固定 P 环落地 -7.25/-6.87/-6.18（目标 -2）。
        【为什么 FIN_KD=0】a_need 解析上已在 h=0 把 vz 精确收到目标；
        减速过程中 d(vz)/dt>0，叠加 -KD*d(vz) 会【减少刹车】。
        离线 5 个工况：KD=0 误差和 9.54；KD=0.45 误差和 17.79。
        """
        _ = alt, dist, vmag, mass
        _vz_tgt = -abs(float(FIN_DESCENT_VZ))
        _dvz_prev = float(self._fin_vz_prev)
        _d_vz = (float(vz) - _dvz_prev) / max(1e-3, float(game_dt))
        self._fin_vz_prev = float(vz)
        _h_req = max(AVER_H_FLOOR, max(0.0, float(error[0])))
        if float(vz) > 0.0:
            a_need = G0 - AVER_CLIMB_K * float(vz) - AVER_CLIMB_MARGIN
        else:
            a_need = (G0 + (float(vz) ** 2 - _vz_tgt ** 2) / (2.0 * _h_req))
        a_up = a_need - float(FIN_KD) * _d_vz
        # 下界：悬停的一部分（保证有力矩）。绝不超过 a_need（否则反冲）
        a_up = max(a_up, min(float(FIN_UP_MIN) * G0, a_need))
        # 【绝对不要给 a_up 加 <= g0*0.99 的封顶】a_up<g0 时净竖直恒为负，
        #   永远无法减速（旧程序实测该错误会让落地垂速达 -116 m/s）。
        #   g0*0.99 只允许用在给【下界】封顶。

        # ---- 水平：P-D 消横速（方向 = 横速反方向）----
        _vhn = float(vel[1])
        _vhe = float(vel[2])
        _d_vhn = (_vhn - float(self._fin_vh_prev_n)) / max(1e-3, float(game_dt))
        _d_vhe = (_vhe - float(self._fin_vh_prev_e)) / max(1e-3, float(game_dt))
        self._fin_vh_prev_n = _vhn
        self._fin_vh_prev_e = _vhe
        a_hn = -float(FIN_VH_KP) * _vhn - float(FIN_VH_KD) * _d_vhn
        a_he = -float(FIN_VH_KP) * _vhe - float(FIN_VH_KD) * _d_vhe
        self.dbg_fin_ah = float(math.hypot(a_hn, a_he))

        target_a = np.array([a_up, a_hn, a_he], dtype=float)

        # ---- 姿态：瞄准天顶，倾角与横向推力成比例 ----
        #   【为什么不能固定取满 FIN_TILT_MAX_DEG】初版只要 a_h>0 就用 8 deg，
        #   即便 a_h 只有 0.1 也命令 8 deg -> 姿态被无谓支偏，落地残余倾角
        #   反而变大（与立直触地的目标相反）。
        if FIN_TILT_HOLD:
            _up = np.array([1.0, 0.0, 0.0])
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

        # 【类型】FIN_TILT_HOLD=False 时 target_direction 为 None（回退路径），
        #   此时不更新 tilt_cmd（保持上一帧值），避免越界下标。
        if target_direction is not None:
            self.tilt_cmd = math.degrees(math.acos(
                K._clamp(float(target_direction[0]), 1.0, -1.0)))
        self.dbg_fin_a_up = float(a_up)
        self.dbg_fin_need = float(a_need)
        return target_a, target_direction
    # ------------------------------------------------------------------
    # 3.11 执行器写入（节流 + 姿态 PID）
    # ------------------------------------------------------------------
    def apply(self, target_a, target_direction=None, game_dt=0.02):
        """逐行照抄参考仓库 demo3_gfold.py 的执行部分（line 397-405, 460-477）。

        原版：
            target_direction = target_a_ / norm(target_a_)
            target_throttle  = norm(target_a) / (max_thrust / mass)
            vessel.control.throttle = target_throttle
            target_direction_local = transform(target_direction, rot_s2l)
            avel_local = transform(avel, rot_s2l)
            control_pitch = -clamp(ctrl_x_rot.update(angle_around_axis(
                target_direction_local, v3(0,1,0), v3(1,0,0)), dt), 1, -1)
            control_yaw   = -clamp(ctrl_z_rot.update(angle_around_axis(
                target_direction_local, v3(0,1,0), v3(0,0,1)), dt), 1, -1)
            control_roll  = clamp(avel_local[1] * ctrl_y_avel_kp, 1, -1)
            vessel.control.pitch = control_pitch
            vessel.control.yaw   = control_yaw
            vessel.control.roll  = control_roll

        【为什么不用 auto_pilot】参考仓库自己写 PID 直接写 control.pitch/
        yaw/roll。旧程序用 auto_pilot 踩过三个坑：never-engaged、错误的
        engage() 名称（kRPC 0.6.0 根本没有 engage，只有 engaged 属性）、
        以及与 kOS 抢 control 通道。照抄参考仓库即可全部避开。

        【节流与姿态用不同的向量】节流用 target_a（当前点），姿态用
        target_a_（前瞻点）—— 这是原版的设计（line 397-398），保留。
        """
        # ---- 节流（原版 line 398, 405）----
        mag = float(np.linalg.norm(target_a))
        a_cap = self.a_cap_real()
        throttle = mag / a_cap if a_cap > 1e-9 else 0.0
        throttle = K._clamp(throttle, 1.0, 0.0)
        self.v.control.throttle = throttle
        self.thr_cmd = throttle
        #   用本帧的 (throttle, Vessel.thrust) 更新推力自校准系数。
        #   必须在 a_cap 被下一次使用前更新；这里紧跟写入之后最合适。
        self.update_thrust_scale(throttle)
        #   记住本帧的 gfold 段油门下限（= 悬停的比例），供 atq_floor 判读。
        try:
            self.dbg_atq_thr = float(throttle)
            self.thr_floor_last = float(
                getattr(self, 'min_mag_eff', 0.0)) / max(1e-9, a_cap)
        except Exception:                        # noqa: BLE001
            self.thr_floor_last = float('nan')

        # ---- 姿态（原版 line 461-477）----
        if target_direction is None:
            # 兜底路径也要限幅，否则调用方忘记传方向时限幅被绕过
            target_direction = K.limit_tilt_dir(
                np.asarray(target_a, float) / max(1e-9, mag),
                MAX_TILT_CMD_DEG)
        try:
            # 机体系 <- 地面系 的旋转（原版 line 348-349）
            q = self.v.rotation(self.v.surface_reference_frame)
            rot_l2s = K.rotation_mat(tuple(q))
            rot_s2l = np.linalg.inv(rot_l2s)
            avel = np.array(self.v.angular_velocity(
                self.v.surface_reference_frame), dtype=float)
            tgt_local = K.transform(target_direction, rot_s2l)
            avel_local = K.transform(avel, rot_s2l)
            err_x = K.angle_around_axis(
                tgt_local, np.array([0.0, 1.0, 0.0]), np.array([1.0, 0.0, 0.0]))
            err_z = K.angle_around_axis(
                tgt_local, np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0]))
            # ① 增益随权限调度：kp 正比于 alpha，使 zeta 与油门无关
            _a_sched = self._att_sched_factor()
            _kp_x = CTRL_X_ROT_KP * _a_sched
            _kp_z = CTRL_Z_ROT_KP * _a_sched
            self.ctrl_x_rot.kp = _kp_x
            self.ctrl_z_rot.kp = _kp_z
            self.dbg_sched = float(_a_sched)
            # ② D 项改用【实测机身角速度】。符号必须是 +omega：
            #    旧程序实测 d(err_x)/dt = +0.782*avel_p（corr +0.215）
            #    第一版写成 -omega，实飞把 D 项变成正反馈：avel_p 冲到
            #    205 deg/s、tilt_act 148.3 deg、落地 vz=-92.2，比不改更惨。
            #    被控对象符号：ctrl.out=+1 -> d(avel_p)/dt=-0.309 rad/s^2
            #    => 要阻尼必须让 diff 对 avel 的系数为正 => diff = +avel。
            # ③ D 项【归一化】：rate = RATE_D_GAIN * omega / omega_phys
            #    【2026-10-21-d 定论】见 starship_config.CTRL_X_ROT_KD 的注释。
            #    原来的 rate = RATE_D_GAIN*omega 是【绝对】角速度，D = kd_eff*omega
            #    必须 <= 1 才不饱和 => kd_eff <= 1/omega_peak。
            #    旧载具 omega_peak=0.21 能吃 kd_eff=4.68；本载具 0.96 只能吃 1.0，
            #    于是 zeta 被物理地锁死在 0.21 —— 实飞 att_err 涨到 77.6 deg。
            #    归一化后 |D| = kd*RATE_D_GAIN*(omega/omega_phys)，饱和门槛
            #        omega = omega_phys/(kd*RATE_D_GAIN) = 0.26*omega_phys
            #    正常跟踪 omega 0.05~0.15 rad/s => D 0.25~0.76 不饱和；
            #    超过门槛的是过冲瞬态，D 顶满 = 满权限反向制动，是正确行为。
            #    关键是【kd 与载具角速率峰值脱钩】：kd 不再受 omega_peak 限制。
            if RATE_D_ENABLE:
                _avel_scale = self._att_rate_phys()
                self.dbg_avel_scale = _avel_scale
                if _avel_scale > 1e-9:
                    _rate_x = (float(avel_local[0]) / _avel_scale
                               * RATE_D_GAIN)
                    _rate_y = (float(avel_local[2]) / _avel_scale
                               * RATE_D_GAIN)
                else:
                    # 权限采样失败时退化为原来的绝对角速度（宁可不归一，
                    # 也不能让 scale=0 把 D 项放大到无穷）。
                    _rate_x = float(avel_local[0]) * RATE_D_GAIN
                    _rate_y = float(avel_local[2]) * RATE_D_GAIN
            else:
                _rate_x = None
                _rate_y = None
            # ④ 参考方向速率前馈：把参考在这一帧动了多少折算成误差的变化率
            _ff_x = 0.0
            _ff_y = 0.0
            if FF_ENABLE and _rate_x is not None and game_dt > 1e-9:
                try:
                    _wnow = np.array(target_direction, dtype=float)
                    _wprev = self._ff_dir_prev
                    if _wprev is not None:
                        _dw = (_wnow - _wprev) / float(game_dt)
                        _dw_local = K.transform(_dw, rot_s2l)
                        _pert = K._normalize(
                            tgt_local + _dw_local * float(game_dt))
                        _ex2 = K.angle_around_axis(
                            _pert, np.array([0.0, 1.0, 0.0]),
                            np.array([1.0, 0.0, 0.0]))
                        _ey2 = K.angle_around_axis(
                            _pert, np.array([0.0, 1.0, 0.0]),
                            np.array([0.0, 0.0, 1.0]))
                        _ff_x = (_ex2 - err_x) / float(game_dt) * FF_GAIN
                        _ff_y = (_ey2 - err_z) / float(game_dt) * FF_GAIN
                        _cl = FF_CLAMP_RAD
                        _ff_x = max(-_cl, min(_cl, _ff_x))
                        _ff_y = max(-_cl, min(_cl, _ff_y))
                    self._ff_dir_prev = _wnow.copy()
                except Exception:                # noqa: BLE001
                    _ff_x = 0.0
                    _ff_y = 0.0
            # ④ 前馈【必须与 D 项同量纲】。
            #    _ff_* 是按 (误差变化率)[rad/s] 算出来的；D 项的 rate 已归一化
            #    成 omega/omega_phys（无量纲），若直接把 rad/s 的 ff 加上去，
            #    单位不一致会把前馈放大 1/omega_phys ≈ 3 倍。
            #    => 归一化生效时，前馈也除以同一个 omega_phys。
            if _rate_x is not None and _rate_y is not None:
                _ffs = getattr(self, 'dbg_avel_scale', 0.0)
                if _ffs > 1e-9:
                    _ff_x = _ff_x / _ffs
                    _ff_y = _ff_y / _ffs
            self.dbg_ff_x = _ff_x
            self.dbg_ff_y = _ff_y
            if _rate_x is not None and _rate_y is not None:
                _rate_x = _rate_x + _ff_x
                _rate_y = _rate_y + _ff_y
            self.dbg_rate_x = _rate_x
            self.dbg_rate_y = _rate_y
            out_x = self.ctrl_x_rot.update(err_x, game_dt, rate=_rate_x)
            out_z = self.ctrl_z_rot.update(err_z, game_dt, rate=_rate_y)
            # 记录 p/i/d 分解（PID.update 的 result = p+i+d，已带符号）
            self.dbg_pid_px = float(-err_x * _kp_x)
            self.dbg_pid_dx = float(-self.ctrl_x_rot.diff * CTRL_X_ROT_KD)
            self.dbg_pid_ix = float(-self.ctrl_x_rot.integral)
            self.dbg_pid_pz = float(-err_z * _kp_z)
            self.dbg_pid_dz = float(-self.ctrl_z_rot.diff * CTRL_Z_ROT_KD)
            self.dbg_pid_iz = float(-self.ctrl_z_rot.integral)
            cp_ = -K._clamp(out_x, 1, -1)
            cy_ = -K._clamp(out_z, 1, -1)
            #   滚转通道直接读角速度（参考仓库 line 473 的做法）。
            #   【不要把同样的公式照抄给俯仰/偏航】旧程序实飞证伪：
            #   滚转没有指令转动，所以 rate->0 是对的；但俯仰/偏航【有】
            #   指令转动，照抄等于让阻尼同时对抗合法跟踪所需的转动，
            #   实测飞行时长 20.5->24.3 s、att_err 73.2->149.8 deg。
            cr_ = K._clamp(float(avel_local[1]) * CTRL_Y_AVEL_KP, 1, -1)
            self.dbg_pid_err_x = math.degrees(err_x)
            self.dbg_pid_err_z = math.degrees(err_z)
            self.dbg_pid_out_x = float(cp_)
            self.dbg_pid_out_z = float(cy_)
            self.dbg_pid_out_r = float(cr_)
            self.dbg_avel_local = [float(x) for x in avel_local]
            self.v.control.pitch = float(cp_)
            self.v.control.yaw = float(cy_)
            self.v.control.roll = float(cr_)
            self.att_err = math.degrees(math.acos(max(-1.0, min(1.0, float(
                np.dot(K._normalize(tgt_local), np.array([0.0, 1.0, 0.0])))))))
            # ---- 记录本机机头相对天顶的真实倾角 ----
            #   【不要把 tilt_act 写成 tilt_cmd】旧程序那样做过，于是该列
            #   只是命令的回声，完全看不出姿态跟没跟上，排查时被误导。
            try:
                nose_srf = np.array(self.v.flight(
                    self.v.surface_reference_frame).direction, dtype=float)
                zen = np.array([1.0, 0.0, 0.0])
                c = float(np.dot(K._normalize(nose_srf), zen))
                self.tilt_act = math.degrees(math.acos(max(-1.0, min(1.0, c))))
                if (self._prev_tilt_act == self._prev_tilt_act
                        and game_dt > 1e-6):
                    self.dbg_tilt_rate = (
                        (self.tilt_act - self._prev_tilt_act) / game_dt)
                self._prev_tilt_act = self.tilt_act
                # --- 候选 1：姿态权限与当帧真实油门对齐 ---
                self.dbg_atq_thr = float(throttle)
                # --- 候选 2：两轴饱和（大倾角下俯仰/偏航可能不再解耦）---
                _lift = float(self.dbg_floor_lift or 0.0)
                _outv = float(self.dbg_a_ver_out or 0.0)
                self.dbg_atq_floor = (1.0 if (_lift > 1e-9 and _outv > 1e-9)
                                      else 0.0)
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
                def _az_of(vn, ve):
                    if abs(vn) < 1e-9 and abs(ve) < 1e-9:
                        return float('nan')
                    return math.degrees(math.atan2(ve, vn))
                _nn, _ne = float(nose_srf[1]), float(nose_srf[2])
                _tn, _te = float(target_direction[1]), float(target_direction[2])
                self.dbg_nose_n = _nn
                self.dbg_nose_e = _ne
                self.dbg_nose_az = _az_of(_nn, _ne)
                self.dbg_tgt_az = _az_of(_tn, _te)
                if (self.dbg_nose_az == self.dbg_nose_az
                        and self.dbg_tgt_az == self.dbg_tgt_az):
                    _d = self.dbg_nose_az - self.dbg_tgt_az
                    _d = ((_d + 180.0) % 360.0) - 180.0
                    self.dbg_az_err = _d
                else:
                    self.dbg_az_err = float('nan')
                # ---- 真实姿态向量 vs 姿态目标向量 ----
                #   【必须记向量而不是只记角度】倾角 24 度既可能朝东也可能
                #   朝西，而本载具要给横向制动，方向反了就是加速而不是减速。
                tgt_srf = np.array(target_direction, dtype=float)
                tgt_srf = tgt_srf / max(1e-9, float(np.linalg.norm(tgt_srf)))
                #   tilt_dir_cmd 必须在这里更新：apply() 是两个分支的共同
                #   出口，target_direction 在此已定稿。旧程序曾只在 track()
                #   里更新它，于是 FIN 段整段打印的是进入前的陈旧值。
                try:
                    self.tilt_dir_cmd = math.degrees(math.acos(
                        max(-1.0, min(1.0, float(tgt_srf[0])))))
                except Exception:                # noqa: BLE001
                    self.tilt_dir_cmd = float('nan')
                self.dbg_dir = K._normalize(nose_srf)
                self.dbg_tgt = tgt_srf
                self.dbg_aim_err = math.degrees(math.acos(max(-1.0, min(1.0,
                    float(np.dot(self.dbg_dir, self.dbg_tgt))))))
            except Exception:                    # noqa: BLE001
                self.tilt_act = float('nan')
        except Exception as e:                   # noqa: BLE001
            if not getattr(self, '_att_warned', False):
                self._att_warned = True
                print('[WARN] 姿态控制异常: %s: %s' % (type(e).__name__, e))
        return throttle
    # ------------------------------------------------------------------
    # 3.12 执行器实测 / 诊断列
    # ------------------------------------------------------------------
    def _read_actuator(self):
        """读【真实执行器状态】：推力与等效油门。

        【为什么不用 control.throttle】那是 kRPC 的写入回读缓存，
          旧程序实测有 9/117 帧读到 0.0000 的假值，会被误判成
          执行器没跟上。而 Vessel.thrust 是 Engine.thrust 的实测求和。

        ================================================================
        【BUG 修复 2026-10-20：thr_real 的分母必须用【标定后】的真实推力】
        ----------------------------------------------------------------
        原实现： thr_real = Vessel.thrust / Vessel.available_thrust
        而 available_thrust 在本构建下【不含 thrustPercentage 限幅】，
        比真实可用推力小 2.500 倍（见 calibrate_thrust 的说明）。
        ⇒ thr_real 恒为真实油门的 2.5 倍，日志里写成 1.87~2.62，
          看上去像「油门超限 190%」，而实际油门只有 0.75。

        实飞证据（starship_log_20261020_180405.csv）：
            t=2.70  thr_cmd=0.754  thrust_n=12.47MN  thr_real=1.895
            真实油门 = 12.47/(6.60*2.5) = 0.756  ← 与 thr_cmd 几乎逐位吻合
        即【执行器本来就是好的】，纯属诊断口径写错，会严重误导排查。

        【修法】分母改用 thrust_true()（= available_thrust * _tscale）。
        这样 thr_real ≈ thr_cmd 才成立；若两者持续不等，才是真的没跟上。
        ================================================================
        """
        try:
            _th = float(self.v.thrust)
            _ttrue = self.thrust_true()
            self.dbg_thrust_n = _th
            if _ttrue > 1e-6:
                self.dbg_thr_real = _th / _ttrue
        except Exception:                        # noqa: BLE001
            pass

    def _dbg_cols(self):
        """产出诊断列（键名与 Logger.COLS 一一对应）。

        两个日志点（hold / gfold 段）都要写这些列，抽成一个方法避免两处漂移。
        NaN 写空串，便于离线脚本用 float() 失败即跳过。
        """
        def _f(v, nd=3):
            try:
                x = float(v)
            except (TypeError, ValueError):
                return ''
            return round(x, nd) if x == x else ''

        def _v(vec, idx):
            if vec is None:
                return ''
            try:
                return _f(vec[idx])
            except (TypeError, IndexError):
                return ''

        def _av_c(idx):
            _a = getattr(self, 'dbg_avel_local', None)
            if not _a:
                return ''
            try:
                return _f(_a[idx])
            except (TypeError, IndexError):
                return ''

        return dict(
            thrust_n=_f(getattr(self, 'dbg_thrust_n', float('nan')), 0),
            thr_real=_f(getattr(self, 'dbg_thr_real', float('nan')), 4),
            dir_vx=_v(getattr(self, 'dbg_dir', None), 0),
            dir_vy=_v(getattr(self, 'dbg_dir', None), 1),
            dir_vz=_v(getattr(self, 'dbg_dir', None), 2),
            tgt_vx=_v(getattr(self, 'dbg_tgt', None), 0),
            tgt_vy=_v(getattr(self, 'dbg_tgt', None), 1),
            tgt_vz=_v(getattr(self, 'dbg_tgt', None), 2),
            n_i=_f(getattr(self, 'dbg_n_i', float('nan'))),
            plan_alt=_f(getattr(self, 'dbg_plan_alt', float('nan'))),
            plan_vz=_f(getattr(self, 'dbg_plan_vz', float('nan'))),
            plan_vh=_f(getattr(self, 'dbg_plan_vh', float('nan'))),
            trk_pos=_f(getattr(self, 'dbg_trk_pos', float('nan'))),
            trk_pos_up=_f(getattr(self, 'dbg_trk_pos_up', float('nan'))),
            trk_pos_h=_f(getattr(self, 'dbg_trk_pos_h', float('nan'))),
            trk_vel=_f(getattr(self, 'dbg_trk_vel', float('nan'))),
            loop_dt=_f(getattr(self, 'dbg_loop_dt', float('nan')), 4),
            rpc_ms=_f(getattr(self, 'dbg_rpc_ms', float('nan')), 1),
            ctrl_ms=_f(getattr(self, 'dbg_ctrl_ms', float('nan')), 1),
            tau_p=_f(getattr(self, 'dbg_tau_p', float('nan')), 0),
            tau_r=_f(getattr(self, 'dbg_tau_r', float('nan')), 0),
            tau_y=_f(getattr(self, 'dbg_tau_y', float('nan')), 0),
            I_p=_f(getattr(self, 'dbg_I_p', float('nan')), 0),
            I_r=_f(getattr(self, 'dbg_I_r', float('nan')), 0),
            I_y=_f(getattr(self, 'dbg_I_y', float('nan')), 0),
            alpha_max=_f(getattr(self, 'dbg_alpha_max', float('nan')), 2),
            avel_mag=_f(getattr(self, 'dbg_avel_mag', float('nan')), 2),
            tilt_rate=_f(getattr(self, 'dbg_tilt_rate', float('nan')), 2),
            tilt_cmd_rate=_f(getattr(self, 'dbg_tilt_cmd_rate', float('nan')), 2),
            tilt_act_rate=_f(getattr(self, 'dbg_tilt_act_rate', float('nan')), 2),
            pid_err_x=_f(getattr(self, 'dbg_pid_err_x', float('nan')), 2),
            pid_err_z=_f(getattr(self, 'dbg_pid_err_z', float('nan')), 2),
            pid_px=_f(getattr(self, 'dbg_pid_px', float('nan'))),
            pid_dx=_f(getattr(self, 'dbg_pid_dx', float('nan'))),
            pid_ix=_f(getattr(self, 'dbg_pid_ix', float('nan'))),
            pid_pz=_f(getattr(self, 'dbg_pid_pz', float('nan'))),
            pid_dz=_f(getattr(self, 'dbg_pid_dz', float('nan'))),
            pid_iz=_f(getattr(self, 'dbg_pid_iz', float('nan'))),
            pid_out_x=_f(getattr(self, 'dbg_pid_out_x', float('nan'))),
            pid_out_z=_f(getattr(self, 'dbg_pid_out_z', float('nan'))),
            pid_out_r=_f(getattr(self, 'dbg_pid_out_r', float('nan'))),
            avel_p=_av_c(0),
            avel_r=_av_c(1),
            avel_y=_av_c(2),
            sched=_f(getattr(self, 'dbg_sched', float('nan')), 3),
            #   D 项归一化分母（本帧物理角速率上限）[rad/s]。
            #   rate_x 与 avel_p 之比应当恒等于 RATE_D_GAIN/avel_scale；
            #   若 avel_scale 为 0 说明当帧权限采样失败、已退回绝对值。
            avel_scale=_f(getattr(self, 'dbg_avel_scale', float('nan')), 4),
            rate_x=_f(getattr(self, 'dbg_rate_x', float('nan')), 3),
            rate_y=_f(getattr(self, 'dbg_rate_y', float('nan')), 3),
            ff_x=_f(getattr(self, 'dbg_ff_x', float('nan')), 4),
            ff_y=_f(getattr(self, 'dbg_ff_y', float('nan')), 4),
            ref_slew=_f(getattr(self, 'dbg_ref_slew', float('nan')), 2),
            ref_clip=_f(getattr(self, 'dbg_ref_clamped', 0.0), 0),
            idx_raw=_f(getattr(self, 'dbg_idx_raw', float('nan')), 2),
            idx_frozen=_f(getattr(self, 'dbg_idx_frozen', float('nan')), 0),
            he_n=_f(getattr(self, 'dbg_he_n', float('nan'))),
            he_e=_f(getattr(self, 'dbg_he_e', float('nan'))),
            hd_n=_f(getattr(self, 'dbg_hd_n', float('nan')), 4),
            hd_e=_f(getattr(self, 'dbg_hd_e', float('nan')), 4),
            h_misalign=_f(getattr(self, 'dbg_h_misalign', float('nan')), 2),
            dp_n=_f(getattr(self, 'dbg_dp_n', float('nan'))),
            dp_e=_f(getattr(self, 'dbg_dp_e', float('nan'))),
            dve_n=_f(getattr(self, 'dbg_dve_n', float('nan'))),
            dve_e=_f(getattr(self, 'dbg_dve_e', float('nan'))),
            s_range=_f(getattr(self, 'dbg_s_range', float('nan'))),
            atq_thr=_f(getattr(self, 'dbg_atq_thr', 0.0), 4),
            atq_floor=_f(getattr(self, 'dbg_atq_floor', 0.0), 0),
            t_scale=_f(self._thrust_scale(), 4),
            pid_sat_x=_f(getattr(self, 'dbg_pid_sat_x', 0.0), 0),
            pid_sat_z=_f(getattr(self, 'dbg_pid_sat_z', 0.0), 0),
            pid_sat_run=_f(getattr(self, 'dbg_pid_sat_run', 0.0), 0),
            nose_n=_f(getattr(self, 'dbg_nose_n', float('nan')), 4),
            nose_e=_f(getattr(self, 'dbg_nose_e', float('nan')), 4),
            nose_az=_f(getattr(self, 'dbg_nose_az', float('nan')), 2),
            tgt_az=_f(getattr(self, 'dbg_tgt_az', float('nan')), 2),
            az_err=_f(getattr(self, 'dbg_az_err', float('nan')), 2),
            plan_tilt_j=_f(getattr(self, 'dbg_tf_tilt_j', float('nan')), 2),
            plan_tf_used=_f(getattr(self, 'dbg_plan_tf_used', float('nan')), 2),
            plan_nj=_f(getattr(self, 'dbg_plan_nj', float('nan')), 2),
            vdesc=_f(getattr(self, 'dbg_vdesc', float('nan')), 1),
            u_h=_f(getattr(self, 'dbg_u_h', float('nan')), 3),
            pdv_h=_f(getattr(self, 'dbg_pdv_h', float('nan')), 3),
            pdp_h=_f(getattr(self, 'dbg_pdp_h', float('nan')), 3),
            u_dot=_f(getattr(self, 'dbg_u_dot', float('nan')), 3),
            pdv_dot=_f(getattr(self, 'dbg_pdv_dot', float('nan')), 3),
            pdp_dot=_f(getattr(self, 'dbg_pdp_dot', float('nan')), 3),
            cmd_dot=_f(getattr(self, 'dbg_cmd_dot', float('nan')), 3),
            tf_plan=_f(getattr(self, 'dbg_tf_tilt_plan', float('nan')), 2),
            tf_corr=_f(getattr(self, 'dbg_tf_tilt_corr', float('nan')), 2),
            tf_want=_f(getattr(self, 'dbg_tf_tilt_want', float('nan')), 2),
            tf_cmd=_f(getattr(self, 'dbg_tf_tilt_cmd', float('nan')), 2),
            tf_perr=_f(getattr(self, 'dbg_tf_perr', float('nan'))),
            tf_verr=_f(getattr(self, 'dbg_tf_verr', float('nan'))),
            tf_az_n=_f(getattr(self, 'dbg_tf_az_n', float('nan')), 4),
            tf_az_e=_f(getattr(self, 'dbg_tf_az_e', float('nan')), 4),
            tf_taper=_f(getattr(self, 'dbg_tf_taper', float('nan')), 3),
            tf_arrived=_f(getattr(self, 'dbg_tf_arrived', 0.0), 0),
            tf_cap=_f(getattr(self, 'dbg_tf_cap', float('nan')), 2),
            tf_slew_allow=_f(getattr(self, 'dbg_tf_slew_allow', float('nan')), 2),
            tf_lead_n=_f(getattr(self, 'dbg_tf_lead_n', float('nan')), 2),
            tf_ahdir_cos=_f(getattr(self, 'dbg_tf_ahdir_cos', float('nan')), 3),
            ca_T=_f(getattr(self, 'dbg_ca_T', float('nan')), 2),
            ca_kv=_f(getattr(self, 'dbg_ca_kv', float('nan')), 3),
            ca_kp=_f(getattr(self, 'dbg_ca_kp', float('nan')), 4),
            ca_pc=_f(getattr(self, 'dbg_ca_posclip', 0.0), 0),
            ah_dem=_f(getattr(self, 'dbg_ah_dem', float('nan'))),
            ah_cmd=_f(getattr(self, 'dbg_ah_cmd', float('nan'))),
            ah_sat=_f(getattr(self, 'dbg_ah_sat', 0.0), 0),
            j_max=_f(getattr(self, 'dbg_j_max', float('nan')), 2),
            j_used=_f(getattr(self, 'dbg_j_used', float('nan')), 2),
            ah_cap_bind=_f(getattr(self, 'dbg_ah_cap_bind', 0.0), 0),
            j_phys=_f(getattr(self, 'dbg_j_phys', float('nan')), 2),
            j_lim=_f(getattr(self, 'dbg_j_lim', float('nan')), 2),
            av_plan=_f(getattr(self, 'dbg_av_plan', float('nan'))),
            av_need=_f(getattr(self, 'dbg_av_need', float('nan'))),
            av_up=_f(getattr(self, 'dbg_av_up', float('nan'))),
            av_net=_f(getattr(self, 'dbg_av_net', float('nan'))),
            av_in_up=_f(getattr(self, 'dbg_av_in_up', float('nan'))),
            av_out_up=_f(getattr(self, 'dbg_av_out_up', float('nan'))),
            av_floor=_f(getattr(self, 'dbg_av_floor', 0.0)),
            min_mag_eff=_f(getattr(self, 'dbg_min_mag_eff', float('nan'))),
            draw_on=_f(getattr(self, 'dbg_draw_on', 0.0), 0),
            fin_on=_f(getattr(self, 'dbg_fin_on', 0.0), 0),
            fin_a_up=_f(getattr(self, 'dbg_fin_a_up', float('nan'))),
            fin_need=_f(getattr(self, 'dbg_fin_need', float('nan'))),
            fin_ah=_f(getattr(self, 'dbg_fin_ah', float('nan'))),
            fin_dp_n=_f(getattr(self, 'dbg_fin_dp_n', float('nan'))),
            fin_dp_e=_f(getattr(self, 'dbg_fin_dp_e', float('nan'))),
            fin_dve_n=_f(getattr(self, 'dbg_fin_dve_n', float('nan'))),
            fin_dve_e=_f(getattr(self, 'dbg_fin_dve_e', float('nan'))),
            post_sas=_f(getattr(self, 'dbg_post_sas', 0.0), 0),

        )
    # ------------------------------------------------------------------
    # 3.13 接管校验 / 后台重解
    # ------------------------------------------------------------------
    def verify_takeover_step(self):
        """接管校验（非阻塞）：每帧采一个样本，够数后判定。

        【为什么必须验证】旧程序实飞出现过最隐蔽的一种失败：kRPC 每帧
        都成功写入 control.throttle，日志里 thr_cmd 记的也是自己写的值，
        但实际执行的是 kOS 的值 —— 因为 kOS 侧 lock throttle 仍然存在，
        kOS 的 lock 每物理帧重算、优先级高于 kRPC 的写入。

        【已知缺陷（照实记录，不掩盖）】
          读回的 act = control.throttle 读的就是上一帧我们自己写的同一个
          字段，KSP 只是回显。所以本校验只能发现【有别人覆盖了该字段】，
          不能证明执行器真有权限。项目的关引擎动作不可逆，因此另有一次
          【阻塞式】confirm_takeover() 在关引擎前执行（见 main）。

        【游戏暂停时不得判负】暂停时物理不前进，control.throttle 停在旧值，
          而 cmd 一直在变 => 连续不符 => 误判为未被接管并放弃控制。
          旧程序实飞就是这样丢了一趟。故冻结时不计入、也不判负。
        """
        if self.takeover_ok is not None:
            return
        # 【注意】不要写 self.args is not None 这种判空：pyright 会在
        #   该 if 之后的代码里把 self.args 收窄成 None（因为它不再被赋值），
        #   于是后面每一处 self.args.xxx 都报 reportOptionalMemberAccess。
        #   args 由 main() 传入 argparse.Namespace，永不为 None。
        if getattr(self.args, 'dry_run', False):
            self.takeover_ok = True
            return
        cmd = getattr(self, 'thr_cmd', 0.0)
        try:
            act = float(self.v.control.throttle)
        except Exception:                        # noqa: BLE001
            return
        if getattr(self, '_frozen_n', 0) >= 3:
            return
        if self.takeover_n < 12:
            self.takeover_n += 1
            return
        if abs(cmd - act) > 0.15:
            self.takeover_bad += 1
        self.takeover_n += 1
        if self.takeover_n >= TAKEOVER_SAMPLES:
            self.takeover_ok = (self.takeover_bad < TAKEOVER_SAMPLES * 0.6)
            if self.takeover_ok:
                print('[kRPC] 接管校验通过：%d 帧中不符 %d 帧'
                      % (self.takeover_n, self.takeover_bad))
                self.log.note('takeover OK: mismatch %d/%d'
                              % (self.takeover_bad, self.takeover_n))
            else:
                print('[kRPC] >> 节流未被真正接管！%d 帧中 %d 帧不符'
                      % (self.takeover_n, self.takeover_bad))
                print('[kRPC] >> 多半是 kOS 侧 lock throttle 仍存在'
                      '（kOS 的 lock 优先级高于 kRPC 写入）。')
                self.log.note('takeover FAILED: %d/%d'
                              % (self.takeover_bad, self.takeover_n))

    def _solve_thread_alive(self):
        return (self._solve_thread is not None
                and self._solve_thread.is_alive())

    def _start_resolve(self, x0_state, mass):
        """在【后台线程】里重解（照抄参考仓库的 Thread(target=solve_gfold)）。

        【为什么用线程】P3/P4 在本载具上 0.06~0.36 s，虽比旧的 3.2 s 快，
        但同步执行仍会停摆主循环、期间不写执行器。参考仓库的做法是起线程，
        主循环继续按【旧轨迹】飞，解好后原子替换。
        【线程安全】只让线程写 self._pending_plan（单一赋值的原子引用），
        主循环在下一帧检查并接管，不做加锁 —— 与参考仓库一致。
        """
        def _work(x0, m, first):
            try:
                res = self.solve(x0, m, allow_cvxpy_fallback=first)
                if res is not None and res.get('status') == 'optimal':
                    self._pending_plan = res
                    self._pending_msg = 'replan OK tf_m=%.1f' % (
                        float(res.get('tf_m', float('nan'))))
                else:
                    self._pending_msg = 'replan fail: %s' % self.gfold_status
            except Exception as e:                # noqa: BLE001
                self._pending_msg = 'replan EXC: %s: %s' % (
                    type(e).__name__, e)

        # 【D 优化】只有【首次解算】（plan 还是 None）才允许 cvxpy 回退。
        #   重解时主循环正按旧轨迹飞，绝不能阻塞 2.7 s。
        _first = (self.plan is None)
        self._solve_thread = threading.Thread(
            target=_work,
            args=(np.array(x0_state, float), mass, _first), daemon=True)
        self._pending_plan = None
        self._pending_msg = None
        self._solve_thread.start()

    def _collect_resolve(self):
        """主循环每帧调用：若后台解好了就把 self.plan 设为新轨迹。

        【2026-10-21-d 恢复成旧程序的简洁形式】
        ------------------------------------------------------------
        旧程序 gfold_land.py:3629 就是这么写的（无条件 n_i = -100）：
            if self._pending_plan is not None:
                self.plan = self._pending_plan
                self.replan_n += 1
                self.gfold_n_i = -100.0
                self.gfold_traj_ready = False

        我此前为「优化重解」加了两版补丁（保留索引 / 重投影），
        但那两版都有问题，且【REPLAN_DT 已改为 0、永不重解】，
        所以本函数实际只会执行【首次解算】那一次：
            plan 从 None 变成有值，n_i 从 -100 定位到真实位置。
        保留复杂分支只会让代码难读、且掩盖「重解已关闭」这个事实。

        【为什么这里必须 n_i = -100（不能直接 0）】
          find_nearest_index 对 n_i 无依赖（它扫全轨迹找最近点），
          但 track() 里有 `_n_from_decay = n_i - dt*0.2*N/tf`，
          若 n_i=0 会得负数，与 IDX_MONOTONIC 的 max() 组合后
          第一帧会锁死在 0。用 -100 让 gfold_traj_ready=False，
          强制 track() 第一帧走「重新定位」路径。
        """
        if self._pending_plan is not None:
            self.plan = self._pending_plan
            self._pending_plan = None
            self.replan_n += 1
            self.gfold_n_i = -100.0
            self.gfold_traj_ready = False
            self.idx_frozen_n = 0
            msg = self._pending_msg or ''
            print('[plan] 采纳轨迹 #%d  %s' % (self.replan_n, msg))
            self.log.note('plan adopted #%d %s' % (self.replan_n, msg))
            return
        if (self._pending_msg and self.plan is None
                and not getattr(self, '_first_fail_reported', False)):
            self._first_fail_reported = True
            print('[G-FOLD] 首次解算失败（%s），保持 hold 并等待重解'
                  % self._pending_msg)
            self.log.note('first solve failed: %s' % self._pending_msg)

    # ------------------------------------------------------------------
    # 3.14 绘图
    # ------------------------------------------------------------------
    def _ensure_lines(self, N):
        """【启动时一次性】创建线对象（照抄参考仓库 line 233-234）。

        【为什么必须建一次、之后只改端点】每次先 remove 再 add_line 会让
        对象只活一帧，KSP 里根本来不及显示，表现为完全看不到线。
        """
        if self._lines_N == N and self.drawn:
            return
        for ln in self.drawn:
            try:
                ln.remove()
            except Exception:                    # noqa: BLE001
                pass
        self.drawn = []
        fr = self.ref
        try:
            for _ in range(max(1, N - 1)):
                ln = self.conn.drawing.add_line((0, 0, 0), (0, 0, 0), fr)
                ln.color = (255, 60, 60)
                ln.width = 2.0
                self.drawn.append(ln)
            self._lines_N = N
            print('[draw] 已创建 %d 条轨迹线' % len(self.drawn))
        except Exception as e:                   # noqa: BLE001
            if not getattr(self, '_draw_warned', False):
                self._draw_warned = True
                print('[WARN] 创建轨迹线失败: %s: %s' % (type(e).__name__, e))
                print('[WARN] 检查 kRPC 的 drawing 服务是否可用')

    def draw_target_marker(self):
        """在目标点画一个竖直短线（hold 期间的可见标记）。"""
        try:
            fr = self.ref
            if self._marker is None:
                ln = self.conn.drawing.add_line((0, 0, 0), (0, 0, 0), fr)
                ln.color = (60, 200, 60)
                ln.width = 3.0
                self._marker = ln
                print('[draw] 已创建目标点标记线')
            self._marker.start = (0.0, 0.0, 0.0)
            self._marker.end = (30.0, 0.0, 0.0)
        except Exception as e:                   # noqa: BLE001
            if not getattr(self, '_marker_warned', False):
                self._marker_warned = True
                print('[WARN] 目标标记线创建失败: %s: %s'
                      % (type(e).__name__, e))

    def draw(self, r):
        """更新参考轨迹线（建一次、只改端点，照抄参考仓库 update_lines）。"""
        try:
            if r is None or r.get('x') is None:
                return
            x = r['x']
            N = x.shape[1]
            self._ensure_lines(N)
            if not self.drawn:
                return
            step = max(1, N // 40)
            idxs = list(range(0, N, step))
            if idxs[-1] != N - 1:
                idxs.append(N - 1)
            for k in range(len(idxs) - 1):
                if k >= len(self.drawn):
                    break
                i0, i1 = idxs[k], idxs[k + 1]
                self.drawn[k].start = (float(x[0, i0]), float(x[1, i0]),
                                       float(x[2, i0]))
                self.drawn[k].end = (float(x[0, i1]), float(x[1, i1]),
                                     float(x[2, i1]))
            for k in range(len(idxs) - 1, len(self.drawn)):
                self.drawn[k].start = (0.0, 0.0, 0.0)
                self.drawn[k].end = (0.0, 0.0, 0.0)
        except Exception as e:                   # noqa: BLE001
            if not getattr(self, '_draw_warned', False):
                self._draw_warned = True
                print('[WARN] draw() 异常: %s: %s' % (type(e).__name__, e))

    # ------------------------------------------------------------------
    # 3.15 主循环
    # ------------------------------------------------------------------
    def run(self):
        """主循环：等门 → 首解（非阻塞）→ gfold 跟踪 → FIN → 落地。"""
        self.log.note('GATE alt<%.0f vmax<%.0f vzmax<%.0f vhmax<%.0f cone<%.0fdeg'
                      % (C.GATE_ALT, C.GATE_VMAX, C.GATE_VZ_MAX,
                         C.GATE_VH_MAX, C.GATE_CONE_DEG))
        self.log.note('vessel=%s mass=%.1f t engines_kept=%s'
                      % (self.v.name, float(self.v.mass) / 1000.0,
                         ','.join(C.ENGINE_TAGS_KEEP)))
        self.log.note('ctrl kp=%.1f kd=%.1f slew=[%.1f,%.1f] alpha_ref=%.2f'
                      % (CTRL_X_ROT_KP, CTRL_X_ROT_KD, REF_SLEW_MIN,
                         REF_SLEW_MAX, TAU_SCHED_ALPHA_REF))

        # 起落架（进入 gfold 段之前就放下，避免低空才放）
        if not getattr(self.args, 'dry_run', False):
            try:
                self.v.control.gear = True
            except Exception:                    # noqa: BLE001
                pass

        self.engaged = True
        self.replan_n = 0
        self.nav_mode = 'hold'

        # ---- 首次解算（状态外推 + 后台线程，非阻塞）----
        _alt, _dist, _vz, _vh, _vmag, mass0, tp0, tv0 = self.state()
        acc_pred = self.hold_accel(np.array(tv0, float), mass0)
        self.est_time = self._predict_est_time()
        x0_state = self.vessel_profile1(np.array(tp0, float),
                                        np.array(tv0, float),
                                        acc_pred, mass0)
        print('[G-FOLD] 状态外推 est_time=%.2fs  起点 alt=%.0f vz=%.1f'
              % (self.est_time, x0_state[0] + TARGET_ALT, x0_state[3]))
        self.log.note('handoff solve at gate: alt=%.1f vz=%.1f vh=%.1f'
                      ' mass=%.1ft est_time=%.2f'
                      % (_alt, -_vz, _vh, mass0 / 1000.0, self.est_time))
        self._start_resolve(x0_state, mass0)
        self.takeover_ok = None
        self.takeover_bad = 0
        self.takeover_n = 0

        t0 = time.time()
        game_prev_time = float(self.sc.ut)
        tilt_act = float('nan')
        idx = 0.0
        target_a = np.zeros(3)
        while True:
            time.sleep(self.args.dt)
            # 参考仓库 line 337-340：游戏时间没前进就跳过（每物理帧一次）
            ut = float(self.sc.ut)
            game_dt = ut - game_prev_time
            if game_dt < 0.01:
                continue
            self.last_pkt = game_dt
            _t_seg0 = time.time()

            # ---- 姿态权限与循环时标测量 ----
            _t_rpc = time.time()
            self.dbg_loop_dt = game_dt
            try:
                _tau_pair = self.v.available_torque
                _tau_l = [float(x) for x in _tau_pair[0]]
                _I_l = [float(x) for x in self.v.moment_of_inertia]
                _av = np.array(self.v.angular_velocity(
                    self.v.surface_reference_frame), dtype=float)
                self.dbg_tau_p = _tau_l[0]
                self.dbg_tau_r = _tau_l[1]
                self.dbg_tau_y = _tau_l[2]
                self.dbg_I_p = _I_l[0]
                self.dbg_I_r = _I_l[1]
                self.dbg_I_y = _I_l[2]
                # 取三轴里最弱的那一轴的角加速度（姿态受最弱轴限制）
                _alphas = []
                for _tq, _in in zip(_tau_l, _I_l):
                    if _in > 1e-6:
                        _alphas.append(abs(_tq) / _in)
                self.dbg_alpha_max = (math.degrees(min(_alphas))
                                      if _alphas else float('nan'))
                self.dbg_avel_mag = math.degrees(float(np.linalg.norm(_av)))
            except Exception as _e:              # noqa: BLE001
                if not getattr(self, '_auth_warned', False):
                    self._auth_warned = True
                    print('[WARN] 姿态权限采样失败: %s: %s'
                          % (type(_e).__name__, _e))
                    self.log.note('auth sample failed: %s: %s'
                                  % (type(_e).__name__, _e))

            _t_state = time.time()
            alt, dist, vz, vh, vmag, mass, tp, tv = self.state()
            _t_state_done = time.time()
            tt = time.time() - t0
            self.dbg_rpc_ms = (_t_rpc - _t_seg0) * 1000.0
            if not self.args.dry_run:
                self._read_actuator()

            # ---- 游戏暂停检测（高度与质量连续多帧完全不变）----
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
                          '（高度与质量都不变）—— 多半是 KSP 被暂停。')
                    print('[kRPC] >> 请取消暂停；本程序不再继续写执行器。')
                    self.log.note('GAME FROZEN: alt/mass unchanged 10 frames')
                    self.log.flush()
                time.sleep(0.2)
                continue
            error = np.array(tp, dtype=float)
            vel = np.array(tv, dtype=float)
            self._collect_resolve()
            self.verify_takeover_step()
            if self.takeover_ok is False:
                print('[kRPC] 接管失败，放开执行器（避免与 kOS 抢控制）')
                self.log.flush()
                break
            if self.prev_vel is not None and game_dt > 1e-6:
                acc = (vel - self.prev_vel) / game_dt
            else:
                acc = np.zeros(3)
            self.prev_vel = vel.copy()
            # 【必须推进 game_prev_time】旧程序漏抄了这一行，导致 loop_dt
            #   从 0.08 一路涨到 15.08（= 累计时间），PID 的 D 项被压小
            #   40~79 倍，姿态回路退化成纯 P，必然极限环振荡。
            game_prev_time = ut

            # ---- 周期性重解 ----
            if (self.args.replan_dt > 0.0
                    and self.nav_mode == 'gfold'
                    and not self._solve_thread_alive()
                    and tt - self.last_replan > self.args.replan_dt
                    and float(error[0]) < (C.GATE_ALT - TARGET_ALT)):
                self.est_time = self._predict_est_time()
                x0_rs = self.vessel_profile1(error, vel, acc, mass)
                self._start_resolve(x0_rs, mass)
                self.last_replan = tt
            # ================= 三态导航 =================
            #   gfold : 跟随参考轨迹
            #   hold  : 首次解算进行中（只稳住，不机动）
            #   fin   : 触地保持段
            if self.nav_mode == 'hold':
                if self.plan is not None:
                    self.nav_mode = 'gfold'
                    self.gfold_n_i = -100.0
                    print('[G-FOLD] 首次解算完成 tf=%.1fs 落地=%.1ft 耗时=%.0fms'
                          ' -> nav_mode=gfold'
                          % (float(self.plan['tf']),
                             float(self.plan['mass_land']) / 1000.0,
                             self.solve_ms))
                    self.log.note('first solve OK tf=%.1f solve_ms=%.0f'
                                  % (float(self.plan['tf']), self.solve_ms))
                else:
                    # ---- 等待首解期间：瞄准速度反方向继续减速（防反冲）----
                    #   参考仓库 simple 模式：target_a = -vel*k_v*3 - error*k_x*0
                    #                             + (g0,0,0)，再限最大倾角。
                    #   本实现沿用其结构，另加两道反冲保护：
                    #     ① 竖直分量上限 = g0 + HOLD_UP_FRAC*(a_cap - g0)
                    #     ② 与 |vz| 挂钩：净减速 <= |vz|/HOLD_TAU
                    #   两者取小 => vz 单调趋近 0 但绝不穿越。
                    a_cap = self.a_cap_real(mass)
                    hold = hold_accel_cmd(vel, a_cap)
                    hold_dir = hold / max(1e-9, float(np.linalg.norm(hold)))
                    hold_dir = K.limit_tilt_dir(hold_dir, MAX_TILT_CMD_DEG)
                    if not self.args.dry_run:
                        self.apply(hold, hold_dir, game_dt)
                    self.tilt_cmd = math.degrees(math.atan2(
                        float(np.linalg.norm(hold[1:3])),
                        max(1e-9, float(hold[0]))))
                    if not self.args.dry_run:
                        _fv = self.v.flight(self.v.surface_reference_frame)
                        _c = float(np.dot(K._normalize(np.array(_fv.direction)),
                                          np.array([1.0, 0.0, 0.0])))
                        self.tilt_act = math.degrees(math.acos(
                            max(-1.0, min(1.0, _c))))
                    self.dbg_fin_on = 0.0
                    self.log.row(
                        t=round(tt, 3), wall=round(time.time(), 2),
                        alt=round(alt, 2), dist_hz=round(dist, 2),
                        vz=round(vz, 3), vh=round(vh, 3), vmag=round(vmag, 3),
                        mass=round(mass, 1), thr_cmd=round(self.thr_cmd, 4),
                        a_cmd_up=round(float(hold[0]), 3),
                        a_cmd_h=round(float(np.linalg.norm(hold[1:3])), 3),
                        a_cmd_mag=round(float(np.linalg.norm(hold)), 3),
                        tilt_cmd=round(self.tilt_cmd, 2),
                        tilt_dir_cmd=(round(self.tilt_dir_cmd, 2)
                                      if self.tilt_dir_cmd == self.tilt_dir_cmd
                                      else ''),
                        tilt_act=(round(self.tilt_act, 2)
                                  if self.tilt_act == self.tilt_act else ''),
                        att_err='',
                        gnc_phase=0, replan_n=0,
                        solve_ms=round(self.solve_ms, 1),
                        gfold_status='hold(decel, waiting 1st solve)',
                        tf=0.0, tgo=0.0, h_min_plan=0.0,
                        x_err=round(float(np.linalg.norm(error)), 2),
                        v_err=round(vmag, 3),
                        **self._dbg_cols())
                    if alt <= TARGET_ALT + LAND_ALT_EPS:
                        print('[done] 等待首次解算期间已触地')
                        break
                    if self.debug_lines:
                        self.draw_target_marker()
                    self.dbg_ctrl_ms = (_t_state_done - _t_state) * 1000.0
                    continue

            # ================= gfold / FIN =================
            assert self.plan is not None
            plan_x = self.plan['x']
            # ---- FIN 触地保持段：在 track() 之前二选一 ----
            #   【为什么必须二选一】若放在 track()+apply() 之后，同一帧会
            #   先按 gfold 写一次执行器、再按 FIN 写一次（双写），且 track()
            #   的末段律与锥限幅会与 FIN 打架。
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
                self.dbg_fin_on = 1.0
                target_a, target_direction = self.fin_control(
                    alt, dist, vz, vh, vmag, mass, error, vel, game_dt)
                self.dbg_fin_dp_n = float(error[1])
                self.dbg_fin_dp_e = float(error[2])
                self.dbg_fin_dve_n = -float(vel[1])
                self.dbg_fin_dve_e = -float(vel[2])
                idx = float(self.gfold_n_i)
                if not self.args.dry_run:
                    self.v.control.gear = True
            else:
                self.dbg_fin_on = 0.0
                target_a, idx = self.track(self.plan, tp, tv, self.gfold_n_i)
                target_direction = self.target_direction
                # 放起落架（原版 line 407-408）：剩余时间 < 10 s
                if (plan_x.shape[1] - self.gfold_n_i) \
                        * float(self.plan['tf']) / plan_x.shape[1] < 10:
                    if not self.args.dry_run:
                        self.v.control.gear = True

            if self.args.dry_run:
                self.thr_cmd = 0.0
            else:
                if not self.engaged:
                    break
                if self.nav_mode in ('gfold', 'fin'):
                    self.apply(target_a, target_direction, game_dt)

            # ---- 画参考轨迹（与 dry_run / nav_mode 无关）----
            _t_ctrl = time.time()
            if self.debug_lines and self.plan is not None:
                self.draw(self.plan)
                self.dbg_draw_on = 1.0
            else:
                self.dbg_draw_on = 0.0
            self.dbg_ctrl_ms = (_t_ctrl - _t_state_done) * 1000.0

            # ---- 姿态实测与速率 ----
            att_err = self.att_err
            tilt_act = self.tilt_act
            _dt_log = float(game_dt) if game_dt > 1e-6 else 0.0
            if _dt_log > 0.0:
                if (self._prev_tilt_cmd_log is not None
                        and self.tilt_cmd == self.tilt_cmd):
                    self.dbg_tilt_cmd_rate = (float(self.tilt_cmd)
                                              - self._prev_tilt_cmd_log) / _dt_log
                if (self._prev_tilt_act_log is not None
                        and tilt_act == tilt_act):
                    self.dbg_tilt_act_rate = (float(tilt_act)
                                              - self._prev_tilt_act_log) / _dt_log
            if self.tilt_cmd == self.tilt_cmd:
                self._prev_tilt_cmd_log = float(self.tilt_cmd)
            if tilt_act == tilt_act:
                self._prev_tilt_act_log = float(tilt_act)
            # ---- 公共日志 ----
            _plan = self.plan
            _tf = float(_plan['tf']) if _plan is not None else 0.0
            #   h_min_plan 记【规划在当前索引处的高度】= 此刻应该在哪，
            #   不能用 x[0,:].min()（那是整条规划的最低点，恒为 ~0，无信息量）。
            _hmin = float('nan')
            if _plan is not None:
                try:
                    _N = _plan['x'].shape[1]
                    _j = int(max(0, min(_N - 1, round(self.gfold_n_i))))
                    _hmin = float(_plan['x'][0, _j])
                except Exception:                # noqa: BLE001
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
                tilt_dir_cmd=(round(self.tilt_dir_cmd, 2)
                              if self.tilt_dir_cmd == self.tilt_dir_cmd else ''),
                tilt_act=round(tilt_act, 2) if tilt_act == tilt_act else '',
                att_err=round(att_err, 2) if att_err == att_err else '',
                gnc_phase=idx, replan_n=self.replan_n,
                solve_ms=round(self.solve_ms, 1), gfold_status=self.gfold_status,
                tf=round(_tf, 2),
                tgo=round(max(0.0, _tf * (1 - max(0.0, idx)
                                      / max(1, (_plan['x'].shape[1] - 1)
                                            if _plan is not None else 1))), 2),
                h_min_plan=round(_hmin, 2),
                x_err=round(float(np.linalg.norm(error)), 2),
                v_err=round(vmag, 3),
                **self._dbg_cols())

            if int(tt * 2) % 4 == 0:
                print('  t=%5.1f alt=%7.1f dist=%6.1f vz=%7.2f vh=%6.2f'
                      ' thr=%.3f tilt=%5.1f'
                      % (tt, alt, dist, vz, vh,
                         getattr(self, 'thr_cmd', 0.0), tilt_act))

            # ---- 结束条件 ----
            if self.nav_mode == 'fin':
                _fin_dt = tt - float(self._fin_t0)
                try:
                    _situ = self.v.situation.name
                except Exception:                # noqa: BLE001
                    _situ = ''
                if _situ == 'landed':
                    print('[done] 触地 alt=%.2f vz=%.2f vh=%.2f dist=%.2f'
                          ' tilt=%.2f (FIN %.1fs)'
                          % (alt, vz, vh, dist, tilt_act, _fin_dt))
                    self.log.note('LANDED alt=%.2f vz=%.2f vh=%.2f dist=%.2f'
                                  ' tilt=%.2f fin=%.2f'
                                  % (alt, vz, vh, dist, tilt_act, _fin_dt))
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
            if alt <= TARGET_ALT + LAND_ALT_EPS:
                print('[done] alt=%.2f vz=%.2f vh=%.2f dist=%.2f tilt=%.2f'
                      % (alt, vz, vh, dist, tilt_act))
                self.log.note('LANDED alt=%.2f vz=%.2f vh=%.2f dist=%.2f'
                              ' tilt=%.2f' % (alt, vz, vh, dist, tilt_act))
                break
            if vz > CLIMB_ABORT_VZ:
                print('[abort] 检测到爬升，退出')
                self.log.note('climb abort vz=%.2f' % vz)
                break
            time.sleep(self.args.dt)

        # ================= 收尾 =================
        if not self.args.dry_run:
            self.v.control.throttle = 0.0
            # ---- 落地后把姿态交给 SAS（径向向外），RCS 保持开启 ----
            #   【为什么必须显式打开 SAS】本进程结束后不再写 control.pitch/
            #   yaw/roll。若 SAS 是关闭的，姿态没有任何东西维持，一级会在
            #   触地后慢慢歪倒。径向模式让机头保持指向天顶，与 FIN 律目标一致。
            #   【为什么必须放在循环之外】SAS 一旦打开，KSP 会忽略
            #   control.pitch/yaw/roll 的写入。放在循环内会让 FIN 律失效。
            try:
                _landed = (str(self.v.situation.name) == 'landed')
            except Exception:                    # noqa: BLE001
                _landed = False
            if _landed:
                try:
                    from krpc.services.spacecenter import SASMode as _SAS
                    self.v.control.rcs = True
                    self.v.control.sas = True
                    self.v.control.sas_mode = _SAS.radial
                    self.dbg_post_sas = 1.0
                    print('[done] 落地后姿态交给 SAS（径向向外），RCS 保持开启')
                except Exception as _e:          # noqa: BLE001
                    self.dbg_post_sas = 0.0
                    print('[WARN] 落地后设置 SAS 失败: %s: %s'
                          % (type(_e).__name__, _e))
        self.log.flush()
        return 0


# ============================================================================
# 3.x hold 段的反向减速律（模块级共享，hold 分支与 hold_accel 共用一份）
# ============================================================================
def hold_accel_cmd(vel, a_cap):
    """hold 段（等首解期间）的推力加速度指令 [m/s^2]（目标系 x=上）。

    【来源】参考仓库 demo3_gfold.py 的 simple 模式：
        target_a = -vel*k_v*3 + (g0,0,0)，再限最大倾角
    本实现沿用其结构，另加两道反冲保护。

    ================================================================
    【BUG 修复 2026-10-20：vz>0（载具已在爬升）时推力会指向下方】
    ----------------------------------------------------------------
    旧实现（逐字照抄的原文）是：
        rev   = -vel / |vel|
        a_mag = min(a_cap, up_cap / max(1e-6, rev[0]))
    问题在 max(1e-6, rev[0])：载具向上运动时 rev[0] 是负数，
    被换成 1e-6（一个极小的正数），于是 up_cap/1e-6 = 巨大值，
    a_mag 取满 a_cap，hold = rev*a_cap —— 推力方向就是速度反方向，
    也就是【朝下推】。

    实飞铁证（starship_log_20261020_172828.csv）：
        384 帧里 300 帧 a_cmd_up < 0，tilt_cmd 恒 90.0 deg
        t= 8.66 vz=+4.32  a_cmd_up=-3.69  tilt_cmd=90.0  thr=0.9998
        t= 8.84 vz=+6.15  a_cmd_up=-4.42  tilt_cmd=90.0  thr=0.9998
    载具本来只是被一次过冲顶上去 9 m/s，被这么一推就是加速冲顶，
    alt 从 22 m 一路涨到 1172 m、vz 到 +88 —— 与旧程序当年
    「把火箭推着往上爬」（alt=3006、vz=+5.3）是同一个故障型。

    【修法】rev[0] <= 0 时不反推，改为纯竖直稳住：
        只抵消重力（hold = (g0,0,0)），让重力自己把上升速度收掉。
        绝不朝下推 —— 那会加速上升。
    """
    v = np.asarray(vel, float)
    vmag = float(np.linalg.norm(v))
    # ---- 兜底：速度近零 => 保持抵消重力 ----
    if not (vmag > 1e-6):
        return np.array([G0 * HOLD_CLIMB_THR_FRAC, 0.0, 0.0])
    rev = -v / vmag
    # ---- 关键判据：速度的竖直分量向上时不能反推 ----
    if float(rev[0]) <= 1e-6:
        # 载具正在向上运动（或纯水平运动），反推会把它推向地面/加速上升。
        # => 只抵消重力，让重力自己把上升速度收掉。
        # 【必须精确等于 g0，不能多一点点】throttle = G0/a_cap，若写成
        #   G0*(1+eps) 就是净向上加速，永远止不住上升。而 kRPC 的
        #   available_thrust 与真实推力有标定偏差时，G0/a_cap 也会偏。
        #   这里返回【恰好抵消重力】的值，并把最终 throttle 交给 apply()
        #   按 a_cap 归一 —— 实测 a_cap=16.66 时 throttle=0.589，
        #   比悬停油门 0.548 高 7.5%，会缓慢上升。
        # => 额外乘一个【略小于 1】的系数，保证净竖直向下。
        return np.array([G0 * HOLD_CLIMB_THR_FRAC, 0.0, 0.0])
    up_cap = G0 + HOLD_UP_FRAC * max(0.0, a_cap - G0)
    a_mag = min(a_cap, up_cap / float(rev[0]))
    vz_net_cap = abs(float(v[0])) / max(0.1, HOLD_TAU)
    a_net_now = a_mag * float(rev[0]) - G0
    if a_net_now > vz_net_cap:
        need_up = vz_net_cap + G0
        a_mag = min(a_mag, need_up / float(rev[0]))
    a_mag = max(0.0, min(a_mag, a_cap))
    return rev * a_mag


# ============================================================================
# 4. 选船 / 交班行 / 交接门
# ============================================================================
def find_handoff_line(max_age_s=300.0):
    """从 kOS Archive 卷（项目根）读 gfold_state.txt 的最后一条 GFOLD_HANDOFF。

    【为什么读文件而不是看 kRPC 状态】kOS 的 lock 是非 kRPC 对象，
    kRPC 看不到 kOS 是否还在写执行器。kOS 交班时会 unlock 并写这行，
    所以这个文件是【唯一可靠】的交班时刻证据（旧程序同样做法）。

    【格式】KEY=value，空格分隔：
      GFOLD_HANDOFF lat=.. lon=.. alt=.. t=.. dist=.. v=.. vz=.. vh=..
                    mass=.. vdesc_k=.. type=.. name=..
    """
    import glob
    import re
    pat = re.compile(r'GFOLD_HANDOFF\s+lat=(?P<lat>[-0-9.]+)\s+lon=(?P<lon>[-0-9.]+)')
    kv = re.compile(r'(\w+)=([-0-9A-Za-z_.]+)')
    dirs = [os.path.dirname(os.path.dirname(_HERE)),
            os.path.join(os.path.dirname(os.path.dirname(_HERE)), 'boot')]
    best = None
    best_t = -1.0
    for d in dirs:
        for p in glob.glob(os.path.join(d, 'gfold_state*')):
            try:
                if time.time() - os.path.getmtime(p) > max_age_s:
                    continue
                with open(p, 'r', encoding='utf-8', errors='replace') as f:
                    for line in f:
                        if not pat.search(line):
                            continue
                        d2 = dict(kv.findall(line))
                        _t = float(d2.get('t', '0') or 0)
                        if _t >= best_t:
                            best_t = _t
                            best = d2
            except OSError:
                continue
    return best


def signed_range_m(pos_h, vel_h):
    """【C 优化 2026-10-20-c】有符号的水平「到达距离」[m]。

    【要解决的问题】原代码到处用标量 `dist = hypot(tp[1], tp[2])`，
      它只能回答「离目标多远」，不能回答「在接近还是在远离」。
      于是所有基于 dist 的制动需求全部算错：
          a_h = vh^2 / (2*dist)
      飞过目标点之后 dist 从 3 m 涨到 111 m，而 vh 还有 18 m/s，
      该式给出的需求是【错的量级】。

    【实飞铁证（starship_log_20261020_202734.csv）】
        dist 时间线: 21.9 -> 3.88(最接近) -> 18.9 -> 50.6 -> 111.4
        t=7.65  能量预检「垂直66需2172.4」—— 荒谬值，因为 dist 已过零
        实际那次 dist=50.6，但更早一帧 dist≈0.8 时算出 2172

    【定义】把「当前位置到目标点」的矢量投影到「当前水平速度方向」上：
        r    = -pos_h                       (指向目标)
        vdir = vel_h / |vel_h|
        s    = dot(r, vdir)
      s > 0 : 目标在【速度前方】，正在接近（可以制动）
      s < 0 : 目标在【速度后方】，已经越过（再怎么推也回不去）
      s ≈ 0 : 横掠而过 / 悬停

    【为什么要投影而不是直接用 dist】
      制动的物理量是「沿速度方向还剩多少距离」。
      若目标在正侧方（r ⊥ v），沿速度方向的距离是 0，
      此时不可能靠沿速度方向的推力到达，必须转向 —— 投影正好给出 0。

    【参数】pos_h, vel_h : 目标系下的水平分量 (y=北, z=东)，长度 2
    【返回】s [m]；输入退化时返回 +|pos_h|（保守：当成还在接近）
    """
    try:
        px = float(pos_h[0])
        pz = float(pos_h[1])
        vx = float(vel_h[0])
        vz = float(vel_h[1])
    except (TypeError, IndexError, ValueError):
        return 0.0
    rn = math.hypot(px, pz)
    vn = math.hypot(vx, vz)
    if not (vn > 1e-6):
        # 没有水平速度：谈不上「接近」，返回模长（保守，仍允许制动）
        return rn
    # r = -pos （从当前位置指向目标）
    return (-px * vx - pz * vz) / vn


def brake_accel_needed(speed, s_range):
    """制动到零所需的水平加速度 [m/s^2]，带符号距离保护。

    【与原来的区别】原式 `vh^2/(2*dist)` 在 dist -> 0 时发散。
      本函数用【有符号到达距离】并加下限保护：
        · s_range <= 0（已越过）=> 返回 inf，调用方据此判定「不可达」
          而不是算出一个荒谬但有限的数（原来给出过 2172 m/s^2）
        · s_range 很小但 > 0      => 分母夹到 C.BRAKE_RANGE_FLOOR
    """
    s = float(s_range)
    v = max(0.0, float(speed))
    if not (s > 0.0):
        return float('inf')
    h = max(float(C.BRAKE_RANGE_FLOOR), s)
    return v * v / (2.0 * h)


def _calib_ratio(vessel):
    """【独立标定】不依赖 Ship 实例，供选船阶段使用。

    与 Ship.calibrate_thrust() 同一算法：
        ratio = sum(Engine.max_thrust * Engine.thrust_limit) / available_thrust
    但这里【不去改任何状态】，只返回比值，用于选船打分。

    【为什么需要独立一份】选船发生在 main() 的 ② 步，那时还没创建 Ship
    （Ship 需要 vessel 才能构造）。而选船打分必须用真实推力口径，
    否则 1008 t 时 a_net 只有 2.39，压在下界上，容易全被筛掉。

    【返回】ratio（>=1.0）；取不到时返回 1.0（退化为旧行为）
    """
    try:
        tot = 0.0
        for e in vessel.parts.engines:
            try:
                if not bool(e.active):
                    continue
                tot += float(e.max_thrust) * max(0.0, float(e.thrust_limit))
            except Exception:                    # noqa: BLE001
                continue
        rep = float(vessel.available_thrust)
        if tot <= 1.0 or rep <= 1e4:
            return 1.0
        r = tot / rep
        return float(r) if (1.0 <= r <= 4.0) else 1.0
    except Exception:                        # noqa: BLE001
        return 1.0

def _vessel_score(vessel, tscale=None):
    """按物理特征给候选船打分（越大越像要回收的一级）。

    【2026-10-21-e 修正 A3：必须用【标定后的真实推力】】
    ------------------------------------------------------------
    原实现 th = float(vessel.available_thrust) —— 【未乘标定系数】。
    而新载具的 available_thrust 比真实值低估 2.500 倍（13 台实测：
    报 12.30 MN vs 真实 30.76 MN）。于是：
        1008 t 时 a_net = 12.30e6/1008e3 - 9.81 = 2.39 m/s^2
    正好压在旧下界 2.0 上；质量稍大 a_net 变负 => 全被筛掉
    => pick_vessel 返回 None => main 直接 return 2，程序根本跑不起来。
    若调了 TweakScale 或引擎损伤，真实推力再变，这个筛选完全失真。

    【修法】乘标定系数，得到与 a_cap_real() 同口径的 a_net，
    并按【真实 TWR】重标区间：
      13 台满油 TWR = 30.76e6/(1008e3*9.81) = 3.11 => a_net = 20.7
      关引擎前 33 台 a_net ≈ 69
      正常着陆段 13 台、质量 1008->890 t => a_net 20.7 -> 25.4
    => 取 [3.0, 120.0]：能覆盖 13 台与 33 台两种构型，
       且能排除「几乎无推力」的残骸（a_net < 3）与「异常巨大」的物体。
    """
    try:
        fl = vessel.flight()
        alt = float(fl.surface_altitude)
        vz = float(fl.vertical_speed)
        m = float(vessel.mass)
        th_rep = float(vessel.available_thrust)
    except Exception:                          # noqa: BLE001
        return None
    if m < 1e4:
        return None
    # 标定系数：调用方给了就用，否则【现场独立标定】
    #   【不能默认 1.0】那会让 a_net 低估 2.5 倍，1008 t 时压在 2.39 上。
    _ts = (max(1.0, float(tscale)) if tscale is not None
           else _calib_ratio(vessel))
    a_net = (th_rep * _ts) / max(1.0, m) - G0
    if vz >= 0.0:
        return None
    if not (100.0 <= alt <= 25000.0):
        return None
    if not (3.0 <= a_net <= 120.0):
        return None
    # 打分：旧程序是 abs(a_net-50) 最小，因为旧载具 a_net≈49。
    #   本载具 13 台 a_net≈20.7，33 台≈69。着陆段只可能是 13 台构型，
    #   故以 20.7 为基准；同时保留「高度接近门」作为次要权重。
    score = 1000.0 - abs(a_net - 20.7) * 10.0 - abs(alt - C.GATE_ALT) * 0.1
    return dict(score=score, alt=alt, vz=vz, mass=m, a_net=a_net,
                name=vessel.name)


def pick_vessel(conn, handoff=None, tscale=None):
    """挑出要回收的一级。

    【优先用 kOS 交班行匹配】kOS 交班行里带 lat/lon/alt，按【经纬度+高度】
    匹配比按名字可靠（绕开名字重复 / vessel id 继承的所有坑）。
    【为什么不用 ship:id】kOS 的 Vessel 结构没有 ID 后缀，用了会
    当场终止脚本（旧程序记录在案的事故）。
    """
    sc = cast(Any, conn.space_center)
    vessels = list(sc.vessels)
    if handoff is not None:
        try:
            hlat = float(handoff.get('lat', 'nan'))
            hlon = float(handoff.get('lon', 'nan'))
            halt = float(handoff.get('alt', 'nan'))
            best = None
            best_d = 1e18
            for v in vessels:
                try:
                    fl = v.flight()
                    dlat = float(fl.latitude) - hlat
                    dlon = float(fl.longitude) - hlon
                    dalt = float(fl.surface_altitude) - halt
                except Exception:              # noqa: BLE001
                    continue
                # 经纬度差换算成米（Kerbin 半径 600 km）
                dn = math.radians(dlat) * 600000.0
                de = (math.radians(dlon) * math.cos(math.radians(hlat))
                      * 600000.0)
                # 【2026-10-21-e 修正 B6：与旧程序一致用 hypot】
                #   旧 gfold_land.py:1124-1131 是 tot = hypot(dist, dh)，
                #   剔除阈 tot >= 3000。我此前写成 hypot(dn,de)+abs(dalt)，
                #   量纲更严（等价于 L1 而非 L2 范数）且无注释。
                d = math.hypot(math.hypot(dn, de), abs(dalt))
                if d < best_d:
                    best_d = d
                    best = v
            # 交班行是几秒前写的，允许 3000 m 半径（与旧程序同）
            if best is not None and best_d < 3000.0:
                return best
        except Exception:                      # noqa: BLE001
            pass
    best = None
    best_s = -1e18
    for v in vessels:
        s = _vessel_score(v, tscale)
        if s is None:
            continue
        if s['score'] > best_s:
            best_s = s['score']
            best = v
    return best


def list_vessels(conn):
    """列出场景内所有 vessel，并标出哪个会被选中（调试用）。"""
    sc = cast(Any, conn.space_center)
    print('%-4s %-28s %8s %9s %8s %7s %8s %s'
          % ('#', 'name', 'mass(t)', 'alt(m)', 'vz(m/s)', 'a_net', '选中', 'type'))
    best = None
    best_s = -1e18
    for i, v in enumerate(list(sc.vessels)):
        s = _vessel_score(v)
        if s is not None and s['score'] > best_s:
            best_s = s['score']
            best = i
        try:
            fl = v.flight()
            print('%-4d %-28s %8.1f %9.1f %8.1f %7.2f %8s %s'
                  % (i, v.name[:28], float(v.mass) / 1000.0,
                     float(fl.surface_altitude), float(fl.vertical_speed),
                     (float(v.available_thrust) * _calib_ratio(v))
                     / max(1.0, float(v.mass)) - G0,
                     'YES' if i == best else '',
                     str(getattr(v, 'type', ''))))
        except Exception as e:                 # noqa: BLE001
            print('%-4d %-28s <读取失败: %s>' % (i, v.name[:28], e))
    hs = find_handoff_line()
    if hs:
        print('\n最近的 kOS 交班行: lat=%s lon=%s alt=%s v=%s mass=%s name=%s'
              % (hs.get('lat'), hs.get('lon'), hs.get('alt'), hs.get('v'),
                 hs.get('mass'), hs.get('name')))
    else:
        print('\n没有找到 kOS 交班行（gfold_state.txt）')


def confirm_takeover(vessel, dry_run=False, hold_thr=0.0, samples=40,
                     skip=12):
    """【阻塞式】实测确认 kRPC 真的能驱动执行器（关引擎前的最后一道门）。

    【为什么必须做，不能只看 handoff 文件】
      旧程序实测过最隐蔽的失败：kRPC 每帧都成功写 control.throttle，
      日志里 thr_cmd 记的也是自己写的值，但 kOS 的 lock 仍存在且优先级
      更高，真正执行的是 kOS 的值 —— kRPC【以为】自己接管了。
      本项目的关引擎动作【不可逆】（关了就回不来），所以必须先用一个
      可逆量验证写权限。

    【已知局限（照实记录，不掩盖）】比较的是 control.throttle 的【回显】，
      只能发现【有别人覆盖该字段】，不能证明执行器真有权限。真正证明权限
      的只有第一次 apply() 之后 Vessel.thrust 是否跟着变（_read_actuator）
      与主循环的 verify_takeover_step()。两条一起看。
    """
    if dry_run:
        print('[takeover] dry-run 不写执行器，跳过校验')
        return True
    # ================================================================
    # 【性能修复 2026-10-20：把阻塞时间从 1.04 s 压到 0.16 s】
    # ----------------------------------------------------------------
    # 旧参数 samples=40, skip=12, sleep=0.02 => 52 × 0.02 = 1.04 s
    # 【全部阻塞】在 kOS 放权之后、任何执行器写入之前。
    # 加上后面的 sleep(0.5) 与关引擎，共 2.20 s 真空 —— 实测掉 170 m。
    #
    # 【为什么可以缩短而安全性不降】
    #   本校验读的是 control.throttle 的【回显】，它的语义是
    #   「有没有别的东西在覆盖这个字段」。kOS 的 lock 一旦撤除，
    #   第一帧就已经不再覆盖；而若 lock 仍在，第一帧就会不符。
    #   所以【8 帧足以判定】，40 帧只是把同一个结论重复 5 倍。
    #   真正的执行器权限由主循环的 verify_takeover_step()（非阻塞、
    #   散在 40 帧里）与 Vessel.thrust 实测共同保证。
    #
    # 【失败仍不关引擎】本函数返回 False 时 main() 直接 return 5，
    #   保留全部 33 台推力可中止，所以缩短采样不会降低可逆性。
    # ================================================================
    samples = min(int(samples), 8)
    skip = min(int(skip), 2)
    bad = 0
    n = 0
    try:
        prev_thr = float(vessel.control.throttle)
    except Exception as e:                     # noqa: BLE001
        print('[takeover] 读执行器失败: %s' % e)
        return False
    for i in range(samples + skip):
        try:
            cmd = hold_thr + 0.05 * (i % 2)
            vessel.control.throttle = cmd
            time.sleep(0.02)
            act = float(vessel.control.throttle)
        except Exception as e:                 # noqa: BLE001
            print('[takeover] 读写执行器异常: %s' % e)
            return False
        if i < skip:
            continue
        n += 1
        if abs(cmd - act) > 0.15:
            bad += 1
    try:
        vessel.control.throttle = prev_thr
    except Exception:                          # noqa: BLE001
        pass
    ok = (n > 0 and bad < n * 0.6)
    print('[takeover] %d/%d 帧不符 -> %s' % (bad, n, '通过' if ok else '失败'))
    if not ok:
        print('[takeover] 多半是 kOS 的 lock throttle 仍在'
              '（kOS lock 优先级高于 kRPC）')
        print('[takeover] 请确认 superheavy.ks 交班时已 unlock throttle/steering')
    return ok


def wait_handoff(timeout):
    """等 kOS 交班（写 GFOLD_HANDOFF 行 + 我方还能读状态）。

    判据：两件事都要成立
      ① gfold_state.txt 里出现最近的 handoff 行（kOS 已 unlock 并退出）
      ② kRPC 能正常读状态（连接仍在）
    """
    t0 = time.time()
    announced = False
    while time.time() - t0 < timeout:
        hs = find_handoff_line()
        if hs:
            print('[handoff] 检测到 kOS 交班行')
            print('          lat=%s lon=%s alt=%s v=%s vz=%s vh=%s mass=%s'
                  % (hs.get('lat'), hs.get('lon'), hs.get('alt'),
                     hs.get('v'), hs.get('vz'), hs.get('vh'), hs.get('mass')))
            return hs
        if not announced:
            announced = True
            print('[handoff] 等待 kOS 交班（gfold_state.txt 里还没有'
                  ' GFOLD_HANDOFF）...')
        time.sleep(0.2)
    print('[handoff] 超时未检测到交班')
    return None


def wait_gate(ship, timeout, args, handoff=None):
    """等 kOS 交班后的【接手门】。

    ================================================================
    【门判据必须与 kOS 完全一致，且不能在 kOS 放行后挡路】
    ----------------------------------------------------------------
    kOS 侧（boot/superheavy.ks:1389）的交班条件是：
        h_aim < gfold_h and v_now:mag < gfold_vmax
        and v_up < gfold_vz_max and gfold_cone_ok and h_gnd > p66_h
    其中 h_aim = alt:radar - (target_alt + aim_alt)，gfold_h = 150
    ⇒ 交班高度 = target_alt + aim_alt + gfold_h = 667 m

    一旦 kOS 写了 GFOLD_HANDOFF 并 unlock，【控制权已经在我们手上】。
    此时再做任何“等门”都是纯粹浪费 —— 那段时间无人点火，
    载具纯自由落体。实测代价（starship_log_20261020_180405）：
        交班 666.2 m/66.73 m/s  ->  过门 496.0 m/85.6 m/s
        掉 170 m，需要 7.65 m/s² 而可用只有 6.93  => P3 infeasible
    ⇒ 正确设计：
        · kOS 放行 = 门已通过，立即 return True（主循环第一帧就开始写执行器）
        · 物理判据只做【能量预检告警】，不拦路
          （判断依据：vz²/(2h) 是否超过 a_net；超了就打印警告，
            让调用方知道这趟大概率失败，但仍然尽力去控）
        · 只有【游戏冻结】和【已低于 GATE_ALT_MIN】才是硬性不接
          （前者读数不可信，后者再等也没意义）

    【为什么旧代码在 496 m 才“满足”】
      旧实现完全没用 GATE_ALT：它的判据是
          GATE_ALT_MIN(400) < alt < GATE_ALT_MAX(900)
      于是 496 m 是“合法”的；叠加冻结帧 |v|=0（暂停时速度插值归零）
      让速度判据也全绿，门就在错误的高度“成功”了。
    ================================================================

    【参数】
      handoff : kOS 交班行（dict）或 None。非 None 表示 kOS 已放行，
                此时【立即返回 True】，不再等任何物理条件。
    """
    _ = args
    # ---- kOS 已放行 => 门已通过，立刻接手（不等任何东西）----
    if handoff is not None:
        # ============================================================
        # 【BUG 修复 2026-10-20-b：handoff 分支缺冻结检测】
        # ------------------------------------------------------------
        # 实飞铁证（starship_log_20261020_190025.csv）：
        #     [gate] kOS 已放行 => 立即接手  alt=632 |v|=0.0 (vz=-0.0 vh=0.0)
        #     [gate] 能量预检: 需要 0.00 / 可用 6.91 [充足]
        # 下一行立刻变成 alt=630.3 vz=70.4 vh=15.8。
        # 也就是说【能量预检被冻结态的零速度污染成 0.00】，报“充足”，
        # 完全误导；更糟的是首解的 x0 也可能拿到 vz=0。
        #
        # 【为什么不能用同一帧的重试】冻结态 kRPC 会持续返回同一快照，
        # 重读同一个值永远不变。必须【等到 alt/mass 发生变化】——
        # 那才是物理真的前进了的证明。
        #
        # 【代价】等待期间不写执行器。但此时 kOS 刚放权、载具在自由落体，
        # 反正我们还没解出轨迹，hold 律也只是一个粗略反向减速；
        # 拿到【真实】速度比早 100 ms 开始推更有价值 ——
        # 错误的 x0 会让整条规划作废（本次就是 P3 infeasible 三连）。
        # ============================================================
        _fg_sig = None
        _fg_frozen = 0
        _fg_t0 = time.time()
        while True:
            try:
                fl = ship.v.flight()
                alt = float(fl.surface_altitude)
                vz = -float(fl.vertical_speed)
                vh = float(fl.horizontal_speed)
                _fm = round(float(ship.v.mass), 1)
            except Exception as e:               # noqa: BLE001
                print('[gate] 读状态失败: %s: %s' % (type(e).__name__, e))
                return False
            _fsig = (round(alt, 3), _fm)
            if _fsig == _fg_sig:
                _fg_frozen += 1
            else:
                _fg_frozen = 0
            _fg_sig = _fsig
            if _fg_frozen >= 1:
                if _fg_frozen in (1, 40) or _fg_frozen % 100 == 0:
                    print('[gate] 状态冻结中（alt=%.1f mass=%.1f 不变）——'
                          ' 请取消暂停；不采信本次读数' % (alt, _fm))
                if time.time() - _fg_t0 > max(30.0, float(timeout)):
                    print('[gate] 等待非冻结读数超时 %.0f s' % timeout)
                    return False
                time.sleep(0.02)
                continue
            break
        vmag = math.hypot(vz, vh)
        # ============================================================
        # 【2026-10-21-e 修正 B2：交班行可能是【陈旧】的】
        # ------------------------------------------------------------
        # 原实现只检查文件 mtime 在 600 s 内 —— 一条 10 分钟前的交班行
        # 也算「新鲜」，于是在完全错的高度/速度下立刻接管。
        # 旧程序 gfold_land.py:3673 用的是 max_age_s=300 且【找不到交班行
        # 仍按物理特征选船、等门 600 s】。
        # 【修法】① 窗口收到 300 s（本函数默认已改）
        #        ② 接手前比对【当前 alt】与【交班行里的 alt】：
        #           差 > ALT_STALE_M 说明这条行不是本趟的（或已过去很久），
        #           拒绝接手并提示。
        # ============================================================
        try:
            _h_alt = float(handoff.get('alt', 'nan'))
        except (TypeError, ValueError):
            _h_alt = float('nan')
        if _h_alt == _h_alt:            # 交班行里有 alt
            _dalt = abs(alt - _h_alt)
            if _dalt > C.GATE_STALE_ALT_M:
                print('[gate] 拒绝接手：交班行 alt=%.0f 与当前 alt=%.0f'
                      ' 相差 %.0f m > %.0f m'
                      % (_h_alt, alt, _dalt, C.GATE_STALE_ALT_M))
                print('[gate] 说明：这条 GFOLD_HANDOFF 可能是【上一趟】留下的，'
                      '或 kOS 放行后已过去很久。')
                print('[gate] 处理：确认 gfold_state.txt 是本趟写的；'
                      '或删除该文件让程序走兜底物理门。')
                return False
        if alt < C.GATE_ALT_MIN:
            print('[gate] 失败：kOS 放行时 alt=%.0f 已低于下限 %.0f，'
                  '剩余高度不足以制动' % (alt, C.GATE_ALT_MIN))
            return False
        # ---- 能量预检（只告警，不拦路）----
        a_net = ship.a_cap_real() - G0
        h_gnd = max(1.0, alt - C.TARGET_ALT)
        a_need = vz * vz / (2.0 * h_gnd)
        print('[gate] kOS 已放行 => 立即接手  alt=%.0f |v|=%.1f'
              ' (vz=%.1f vh=%.1f)' % (alt, vmag, vz, vh))
        print('[gate] 能量预检: 需要 %.2f / 可用 %.2f m/s^2'
              ' (比值 %.2f)%s'
              % (a_need, a_net, a_need / max(1e-6, a_net),
                 '  [不足，本趟大概率失败]'
                 if a_need > a_net else '  [充足]'))
        try:
            ship.log.note('gate(handoff-driven): alt=%.1f vz=%.1f vh=%.1f'
                          ' a_need=%.2f a_net=%.2f'
                          % (alt, vz, vh, a_need, a_net))
        except Exception:                # noqa: BLE001
            pass
        return True

    # ---- 兜底路径：没有交班行时，按旧逻辑等物理门 ----
    t0 = time.time()
    last_print = -1e18
    _gate_sig = None
    _gate_frozen = 0
    while True:
        now = time.time()
        fl = ship.v.flight()
        alt = float(fl.surface_altitude)
        vz = -float(fl.vertical_speed)
        vh = float(fl.horizontal_speed)
        vmag = math.hypot(vz, vh)
        # ---- 冻结检测：暂停时速度插值归零，会让所有判据假性通过 ----
        try:
            _m = round(float(ship.v.mass), 1)
        except Exception:                    # noqa: BLE001
            _m = -1.0
        _sig = (round(alt, 3), _m)
        if _sig == _gate_sig:
            _gate_frozen += 1
        else:
            _gate_frozen = 0
        _gate_sig = _sig
        # 【关键】哪怕只冻了 1 帧也【不采信】—— 旧实现要连续 5 帧才跳过，
        #   而事故那一帧正是第 1 帧冻结值，于是被当成真门用了。
        if _gate_frozen >= 1:
            if now - last_print > 1.0:
                last_print = now
                print('[gate] 状态冻结中（alt/mass 不变）—— 请取消暂停；'
                      '不采信本次读数')
            if now - t0 > timeout:
                print('[gate] 超时 %.0f s（期间处于冻结）' % timeout)
                return False
            time.sleep(0.02)
            continue
        cap = (min(float(C.GATE_VZ_MAX), C.vz_gate_max(alt, vh))
               - float(C.GATE_VZ_MARGIN))
        # 【2026-10-21-e 修正 B1：补回旧程序的锥角门 cone_ok】
        #   旧 gfold_land.py:3718/3727-3728：
        #       cone_ok = dist <= (alt-TARGET_ALT)/tan(radians(GATE_CONE_DEG))
        #       if (alt<GATE_ALT and vmag<GATE_VMAX and cone_ok and alt>GATE_ALT_MIN)
        #   我此前漏了这一项，于是兜底路径会接受「横向偏离瞄准点过远」的
        #   初始条件 —— 那种状态 P3 必然 infeasible（dist 太大、水平制不动）。
        #   虽然 main() 在没有交班行时直接 return 2（该路径实际只 --dry-run 走到），
        #   但一旦恢复「无交班行也能跑」就是洞。现在补回。
        _cone_ok = (vh <= 0.0
                    or (alt - C.TARGET_ALT) * math.tan(
                        math.radians(C.GATE_CONE_DEG)) >= 0.0)
        #   注：兜底路径拿不到真实 dist（要建参考系才算），
        #   这里用「垂速与横速的比值」作近似判据：
        #   tan(cone) = dist/(alt-TA)，而 vh/vz 反映同一几何关系。
        #   保守起：只拒绝 vh 明显大于「锥内允许值」的情形。
        if alt > C.TARGET_ALT:
            _cone_ok = (vh <= max(1.0, vz) * math.tan(math.radians(C.GATE_CONE_DEG)))
        ok = (C.GATE_ALT < alt < C.GATE_ALT_MAX
              and vz < cap and vh < C.GATE_VH_MAX
              and vmag < C.GATE_VMAX and _cone_ok)
        if ok:
            print('[gate] 满足 alt=%.0f |v|=%.1f (vz=%.1f/%.0f vh=%.1f)'
                  % (alt, vmag, vz, cap, vh))
            return True
        if now - last_print > 1.0:
            last_print = now
            print('[gate] alt=%.0f vz=%.1f/%.0f vh=%.1f |v|=%.1f  (窗 %.0f~%.0f m)'
                  % (alt, vz, cap, vh, vmag,
                     C.GATE_ALT, C.GATE_ALT_MAX))
        if now - t0 > timeout:
            print('[gate] 超时 %.0f s' % timeout)
            return False
        time.sleep(0.02)


# ============================================================================
# 5. 入口
# ============================================================================
def main():
    """入口：参数解析 → 连接 → 选船 → 关引擎（门控）→ 交接门 → 主循环。"""
    ap = argparse.ArgumentParser()
    ap.add_argument('--dry-run', action='store_true', help='只解算画线，不写执行器')
    ap.add_argument('--no-shutdown', action='store_true', help='不关引擎（调试用）')
    ap.add_argument('--gate-timeout', type=float, default=C.GATE_TIMEOUT)
    ap.add_argument('--handoff-timeout', type=float, default=C.GATE_TIMEOUT,
                    help='等待 kOS 交班的最长时间')
    ap.add_argument('--dt', type=float, default=0.02, help='主循环最小间隔 [s]')
    ap.add_argument('--est-time', type=float, default=C.EST_TIME,
                    help='状态外推时长 [s]，用于补偿解算耗时；'
                         '仅当上次求解耗时 <=0.05 s 时生效（默认 0.5）')
    ap.add_argument('--replan-dt', type=float, default=C.REPLAN_DT,
                    help='周期重解间隔 [s]，0 = 不重解')
    ap.add_argument('--debug-lines', action='store_true', help='画参考轨迹')
    ap.add_argument('--list-vessels', action='store_true',
                    help='列出场景里的船只与筛选判据后退出')
    args = ap.parse_args()

    conn = connect()
    # 同 connect()：space_center 被 stub 判成 Optional，cast 记录事实。
    sc = cast(Any, conn.space_center)
    if sc is None:
        print('[fatal] space_center 不可用，检查 kRPC 服务器与游戏场景')
        return 4
    if args.list_vessels:
        list_vessels(conn)
        return 0

    log = K.Logger(enabled=True)
    log.note('starship_land start  dry_run=%s no_shutdown=%s'
             % (args.dry_run, args.no_shutdown))

    # ================= ① 等 kOS 交班 =================
    hs = wait_handoff(args.handoff_timeout)
    if hs is None and not args.dry_run:
        log.note('handoff timeout')
        log.flush()
        return 2
    handoff = hs

    # ================= ② 选船 =================
    vessel = pick_vessel(conn, handoff)
    if vessel is None:
        print('[fatal] 没有找到合适的回收一级')
        print('        用 --list-vessels 查看当前场景与判据')
        log.note('no candidate vessel')
        log.flush()
        return 2
    ship = Ship(conn, vessel, log=log, args=args)
    ship.debug_lines = bool(args.debug_lines)
    # ================================================================
    # 【BUG 修复 2026-10-20】先做【同步】推力标定，再 any 预检/求解。
    #   实飞事故：available_thrust 报 6.59 MN 而真实是 16.49 MN（2.5 倍），
    #   程序拿 6.59 算 a_net = -3.13，直接判推重比过低并放弃。
    #   标定必须在【首解之前】，否则外推、预检、求解器 t_max 全部按错的
    #   推力算。运行中采样（update_thrust_scale）只能作为补充。
    # ================================================================
    ship.calibrate_thrust(verbose=True)
    d = ship.describe(vessel)
    print('[vessel] %s  mass=%.1f t  %s 台引擎  thrust=%.2f MN  a_net=%+.2f m/s^2'
          '  (启动时构型，尚未关引擎)'
          % (d['name'], d['mass'] / 1000.0, d['n_engine'],
             d['thrust'] / 1e6, d['a_net']))
    log.note('vessel=%s mass=%.0fkg engines=%s thrust=%.0fN a_net=%+.2f t_scale=%.3f'
             % (d['name'], d['mass'], d['n_engine'], d['thrust'], d['a_net'],
                ship._thrust_scale()))
    log.note('log path = %s' % log.path)

    # ================= ③ 关引擎（门控，不可逆动作）=================
    # 【时序不可颠倒】用户要求回收程序启动时关闭 26 台，但直接在任何
    #   时刻关都不安全：
    #     · 若 kOS 还没交班（仍 hold 着 lock throttle），关掉 26 台会让
    #       一级在 kOS 段1 就掉到 TWR<1 —— kOS 还在按 33 台规划，必然坠毁。
    #     · 若 kOS 已交班但 kRPC 还没真正写进执行器，此时关引擎等于在
    #       无人控制的情况下砍推力。
    #   => 顺序：① handoff ② confirm_takeover ③ shutdown_engines
    #   任何一步失败都【保留完整推力】可中止。
    if not args.no_shutdown:
        thr0 = max(1.0, float(vessel.available_thrust))
        print('[engine] 关引擎前: 可用推力 %.2f MN, a_max %.2f m/s^2'
              % (thr0 / 1e6, thr0 / max(1.0, float(vessel.mass))))
        if not confirm_takeover(vessel, dry_run=args.dry_run, hold_thr=0.0):
            print('[engine] 接管未确认，【不关引擎】以保留中止能力')
            log.note('takeover not confirmed -> engines kept')
            log.flush()
            return 5
        print('[engine] 接管已确认 -> 现在关闭 26 台引擎')
        _t_shut = time.time()
        shutdown_engines(vessel, dry_run=args.dry_run)
        # ============================================================
        # 【性能修复 2026-10-20】去掉 time.sleep(0.5)，改成短轮询。
        #   原来那 0.5 s 是【硬等待】，目的只是让 KSP 把引擎状态刷过来，
        #   但它发生在 kOS 放权之后、主循环第一帧之前 —— 纯自由落体。
        #   实测 0.5 s 掉约 37 m、涨 5 m/s。
        #   改成：最多等 ENGINE_SETTLE_S，一旦【报值推力真的降下来】
        #   就立刻往下走。正常情况 0.1~0.2 s 就完成。
        # ============================================================
        _deadline = _t_shut + float(C.ENGINE_SETTLE_S)
        while time.time() < _deadline:
            try:
                if float(vessel.available_thrust) < 0.5 * thr0:
                    break
            except Exception:                    # noqa: BLE001
                break
            time.sleep(0.02)
        # 引擎数变了（33 -> 7），标定系数必须【重算】。
        #   同步标定是按活动引擎的 max_thrust*limiter 求和再比 available_thrust，
        #   引擎集合一变比值也会变，沿用旧值会把 a_cap 算错。
        ship._tscale_buf = []
        ship._tscale = 1.0
        ship.calibrate_thrust(verbose=True)
        thr = max(1.0, float(vessel.available_thrust)) * ship._thrust_scale()
        print('[engine] 关引擎后: 可用推力 %.2f MN (报值 %.2f MN), a_max %.2f m/s^2'
              % (thr / 1e6, float(vessel.available_thrust) / 1e6,
                 thr / max(1.0, float(vessel.mass))))
        log.note('engines: avail %.2f -> %.2f MN (true %.2f MN)'
                 % (thr0 / 1e6, float(vessel.available_thrust) / 1e6, thr / 1e6))
        # 【判据要按真实推力比】thr 已含标定，thr0 未含，不能直接比。
        _rep = float(vessel.available_thrust)
        if _rep > 0.5 * thr0:
            print('[engine] *** 警告：报值推力没有明显下降，关引擎可能没生效 ***')
            log.note('WARN thrust did not drop as expected')
        # ============================================================
        # 【修复 2026-10-20】关引擎后【必须重打一次载具摘要】。
        #   上面那次 describe() 是【33 台全开】的值（a_net≈69），
        #   而着陆真正可用的只有 7 台（a_net≈6.9）。
        #   两行并排打印，判读时一眼就能看出「权限从 69 掉到 6.9」，
        #   不会再拿全开数当着陆构型的性能。
        # ============================================================
        d2 = ship.describe(vessel, n_engine=len(C.ENGINE_TAGS_KEEP))
        print('[vessel] %s  mass=%.1f t  %s 台引擎  thrust=%.2f MN  a_net=%+.2f m/s^2'
              '  (回收构型，用于着陆)'
              % (d2['name'], d2['mass'] / 1000.0, d2['n_engine'],
                 d2['thrust'] / 1e6, d2['a_net']))
        log.note('vessel(recovery)=%s mass=%.0fkg engines=%s thrust=%.0fN'
                 ' a_net=%+.2f'
                 % (d2['name'], d2['mass'], d2['n_engine'], d2['thrust'],
                    d2['a_net']))

    # ================= ④ 接手门 =================
    # 【关键时序修复 2026-10-20】kOS 交班行（handoff）非 None 就代表
    #   **kOS 已经放权**，此时门判据【立即通过】，主循环第一帧就开始
    #   写执行器。物理判据只做能量预检告警，不拦路。
    #   旧实现在这里还要再等一次 height/vz/vh 判据，而那段等待本身
    #   就是白掉的 170 m（详见 wait_gate 的文档字符串）。
    if not wait_gate(ship, args.gate_timeout, args, handoff=handoff):
        log.note('gate rejected / unrecoverable')
        log.flush()
        return 2

    # ================= ⑤ 主循环 =================
    try:
        return ship.run()
    except KeyboardInterrupt:
        print('\n[abort] 用户中断')
        if not args.dry_run:
            try:
                vessel.control.throttle = 0.0
            except Exception:                  # noqa: BLE001
                pass
        log.note('KeyboardInterrupt')
        log.flush()
        return 1
    except Exception as e:                     # noqa: BLE001
        import traceback
        print('[fatal] %s: %s' % (type(e).__name__, e))
        traceback.print_exc()
        log.note('fatal %s: %s' % (type(e).__name__, e))
        log.flush()
        return 1
    finally:
        try:
            conn.close()
        except Exception:                      # noqa: BLE001
            pass


if __name__ == '__main__':
    sys.exit(main())
