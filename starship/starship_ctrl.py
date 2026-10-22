# -*- coding: utf-8 -*-
"""starship_ctrl.py -- starship 回收程序的控制学公共件（纯数学 + 日志）。

【为什么单独成文件】
  这些函数/类与载具无关，只与"参考仓库的控制结构"有关：
      clamp / lerp / sgn / normalize / angle_around_axis / rotation_mat
      transform / limit_tilt_dir / PID / Logger
  把它们与飞行逻辑分开，改载具参数时不必碰这里。

【全部逐行照抄 dev/krpc/gfold_land.py】
  参考来源是该文件 line 371~705（唯一实飞验证过的 G-FOLD 实现。
  上游 demo3_gfold.py 的原版公式）。凡是它有的都照抄，
  只把"日志目录"和"列名注释"改成与 starship 相关。
"""
import csv
import math
import os
from datetime import datetime

import numpy as np

import starship_config as C


# ============================================================================
# 1. 基础数学（原版 clamp / lerp / sgn / normalize）
# ============================================================================
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


def angle_around_axis(v1, v2, axis):
    """原版 angle_around_axis()（demo3_gfold.py line 79-84）。"""
    axis = _normalize(axis)
    v1 = _normalize(np.cross(v1, axis))
    v2 = _normalize(np.cross(v2, axis))
    direction = _sgn(np.dot(np.cross(v1, v2), axis))
    return direction * math.acos(max(-1.0, min(1.0, float(np.dot(v1, v2)))))


def rotation_mat(q):
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


def transform(vec, mat):
    """原版 transform()（demo3_gfold.py line 75-77）。"""
    res = np.asarray(vec) @ mat
    return np.array([res[0], res[1], res[2]])


# ============================================================================
# 2. PID（逐行照抄参考仓库 demo3_gfold.py 的 PID 类，line 100-137）
# ============================================================================
class PID:
    """原版行为：
        update(error, dt):
            首帧只记录 error_prev；第二帧算 diff
            integral += error*dt*ki, 并用 integral_limit 限幅
            diff = lerp(diff, (error-error_prev)/dt, 1-sd)
            p = -error*kp ; i = -integral ; d = -diff*kd
            result = p*(ep) + i*(ei) + d*(ed)
    注意原版返回的 result 【已经带了负号】，调用方又取负
    （control_pitch = -clamp(...)），所以净效果是"正误差 -> 正控制量"。
    移植时【保持原样】，不做简化。
    """

    def __init__(self):
        self.ep = True
        self.ei = True
        self.ed = True
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

        【符号约定】调用方传入的 rate 必须取【与 (error-error_prev)/dt 同号】
          的值。旧程序实飞证明该符号是 +omega（不是 -omega），
          详见 gfold_land.py apply() 里 7(b)-② 的完整回归数据。
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


# ============================================================================
# 3. 日志（CSV，列固定，便于离线分析）
# ============================================================================
class Logger:
    """写 CSV 到 C.LOG_DIR（= 项目根的 log-B1040-9-gfold/），列固定。

    【与旧 B1040-9 程序的日志完全同格式、同目录】
      用户要求日志放在同一个 log-B1040-9-gfold 目录。
      文件名前缀用 starship_log 区分载具，不会覆盖旧程序的 gfold_log_*。

    【列的含义分组】（与旧程序逐列一致，便于两套日志用同一套离线脚本）
      ① 真实姿态 / 姿态目标（目标系分量）dir_*/tgt_*
      ② 参考轨迹采样点 + 跟踪误差 n_i/plan_*/trk_*
      ③ 循环时标与姿态权限 loop_dt/rpc_ms/tau_*/I_*/alpha_max
      ④ PID 内部量 pid_*
      ⑤ 参考索引 idx_*
      ⑥ 水平误差方向 he_*/hd_*
      ⑦ 矢量误差分量 dp_*/dve_*
      ⑧ 姿态失守诊断 atq_*/sat/az
      ⑨ 规划质量 plan_tilt_j/plan_tf_used/vdesc
      ⑩ 水平指令分项 u_h/pdv_h/pdp_h 及其与速度方向的夹角余弦
      ⑪ 倾角第一性律 tf_*
      ⑫ 内环（a_h 加加速度限幅）ah_*/j_*
      ⑬ 竖直通道 av_*
      ⑭ conic_clamp 的输入/输出竖直分量 av_in_up/av_out_up
      ⑮ 绘图 draw_on
      ⑯ FIN 触地保持段 fin_*
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
            'tilt_cmd_rate', 'tilt_act_rate',
            # ---- ④ PID 内部量 ----
            'pid_err_x', 'pid_err_z',
            'pid_px', 'pid_dx', 'pid_ix',
            'pid_pz', 'pid_dz', 'pid_iz',
            'pid_out_x', 'pid_out_z', 'pid_out_r',
            'avel_p', 'avel_r', 'avel_y',
            'sched', 'avel_scale', 'rate_x', 'rate_y', 'ff_x', 'ff_y',
            'ref_slew', 'ref_clip',
            # ---- ⑤ 参考索引 ----
            'idx_raw', 'idx_frozen',
            # ---- ⑥ 水平误差【方向】----
            'he_n', 'he_e', 'hd_n', 'hd_e', 'h_misalign',
            # ---- ⑦ 矢量误差分量 ----
            'dp_n', 'dp_e', 'dve_n', 'dve_e',
            #   【C 优化 2026-10-20-c】有符号到达距离 [m]：
            #   正 = 目标在速度前方（还有距离可制动）
            #   负 = 已越过目标（横向不可达，再怎么推也回不去）
            #   与标量 dist_hz 互补：dist_hz 只说「多远」，不说「能否到达」。
            's_range',
            # ---- ⑧ 姿态失守诊断 ----
            'atq_thr', 'atq_floor', 't_scale',
            'pid_sat_x', 'pid_sat_z', 'pid_sat_run',
            'nose_n', 'nose_e', 'nose_az', 'tgt_az', 'az_err',
            # ---- ⑨ 规划质量 ----
            'plan_tilt_j', 'plan_tf_used', 'plan_nj', 'vdesc',
            # ---- ⑩ 水平指令分项 ----
            'u_h', 'pdv_h', 'pdp_h', 'u_dot', 'pdv_dot', 'pdp_dot',
            'cmd_dot',
            # ---- ⑪ 倾角第一性律 ----
            'tf_plan', 'tf_corr', 'tf_want', 'tf_cmd',
            'tf_perr', 'tf_verr', 'tf_az_n', 'tf_az_e',
            'tf_taper', 'tf_arrived',
            'tf_cap', 'tf_slew_allow', 'tf_lead_n',
            'tf_ahdir_cos', 'ca_T', 'ca_kv', 'ca_kp', 'ca_pc',
            # ---- ⑫ 内环（a_h 加加速度限幅）----
            'ah_dem', 'ah_cmd', 'ah_sat', 'j_max', 'j_used', 'ah_cap_bind',
            'j_phys', 'j_lim',
            # ---- ⑬ 竖直通道 ----
            'av_plan', 'av_need', 'av_up', 'av_net',
            # ---- ⑭ conic_clamp 的输入/输出竖直分量 ----
            'av_in_up', 'av_out_up', 'av_floor', 'min_mag_eff',
            # ---- ⑮ 绘图 ----
            'draw_on',
            # ---- ⑯ FIN 触地保持段 ----
            'fin_on', 'fin_a_up', 'fin_need', 'fin_ah',
            'fin_dp_n', 'fin_dp_e', 'fin_dve_n', 'fin_dve_e',
            'post_sas']

    def __init__(self, enabled=True, name=None):
        self.enabled = enabled
        self.rows = []
        self.notes = []
        try:
            os.makedirs(C.LOG_DIR, exist_ok=True)
        except OSError as e:
            print('[log] 无法创建日志目录 %s: %s' % (C.LOG_DIR, e))
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(
            C.LOG_DIR, name or ('%s_%s.csv' % (C.LOG_PREFIX, stamp)))

    def note(self, msg, echo=True):
        """记录一条注释行。默认既写日志又打控制台。"""
        self.notes.append('# %s' % msg)
        if echo:
            print('[note] %s' % msg)

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
            print('[log] %d 行 -> %s' % (len(self.rows), self.path))
        except OSError as e:
            print('[log] 写入失败: %s' % e)


# ============================================================================
# 4. 地理换算（与 kOS 侧口径一致）
# ============================================================================
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
