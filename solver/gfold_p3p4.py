# gfold_p3p4.py — 逐行移植参考仓库的 P3/P4 两级求解器
#
# ============================================================================
# 【来源】dev/gfold/GFOLD_direct_exec.py （原始仓库 xdedss/GFOLD_KSP）
#         以及 dev/gfold/GFOLD_run.py 的 solver 类
#
# 【为什么要有这个文件】
#   2026-09-26 我【自行发明】了"tf 二分搜索 + 单次求解"的结构，理由是
#   "tf 给长了会贴地抖动"。但参考仓库本来就有正确做法：
#       P3（Minimum Landing Error）: tf 固定为一个【预估上限】，只要求落在
#           目标点（x[0,N-1]==0，不要求水平也归零），目标函数是
#           Σ‖x[0:3,i]‖·(i/N) —— 即"尽快靠近目标"。
#           解完后【从轨迹里读出真正落地的那一时刻】作为 tf_m：
#               for i in range(x.shape[1]):
#                   if (norm(x[0:3,i]) + norm(x[3:6,i])) < 0.1:
#                       tf_m = i / x.shape[1] * self.tf_
#                       break
#       P4（Minimum Fuel Use）: 用 tf = tf_m + 0.1*straight_fac 固定，
#           要求 x[0:3,N-1]==0（位置速度全归零），最大化落地质量。
#
#   ⇒ 两级结构天然解决"tf 该给多少"：P3 只管找时间，P4 在该时间上求燃料最优。
#     不需要二分搜索（我那版一次求解要 7.4 s，太重）。
#
# 【本文件与原版的差异（仅此两处，都是环境适配）】
#   ① 原版用 `from cvxpy import *`（老式 0.4.x API）+ ECOS。
#      本项目环境是 cvxpy 1.9.3 + Clarabel（已验证可用），故改用 import cvxpy as cp
#      与 solver=CLARABEL。数学式【逐条不变】。
#   ② 原版依赖 cvxpy_codegen 生成的 gfold_solver_p3/p4 C 扩展
#      （GFOLD_run.py 里 import gfold_solver_p3）。本环境没编译该扩展，
#      故直接用 GFOLD_direct 的 cvxpy 路径（原版也支持，即 params.txt 里
#      direct=True 的模式）。
#
# 【原版的两条约束，本项目按需调整并在下方标注】
#   · con += [u[:,0] == s[0,0]*[1,0,0]]  —— 原版要求【推力起点竖直】。
#     原版是"从悬停附近开始规划"，起点竖直合理；但本项目是【带横速入场】
#     （vh 可达 350 m/s），起点必须允许倾斜，否则无解。
#     这与项目里已修过的"推力指向锥不能全程固定"是同一个问题（坑①）。
#     故保留该约束的【语义】但放到 pcs 锥里统一处理（见 PCS 说明）。
import math
import os
import numpy as np
import cvxpy as cp

G0 = 9.80665
SOLVER_NAME = 'CLARABEL'

# ============================================================================
# 【2026-09-29 第五步 5(b)：P4 的 tf 可行性探测参数】
# ----------------------------------------------------------------------------
#   tf4 = tf_m + 0.1*straight_fac 在【开启下降率包络】后不够用：
#   包络拉长剖面，P4 的最小可行 tf 比 tf_m 增得更快。
#   实测（AIM_ALT=2000 时）：P3 tf_m=29.8 s，而 P4 需要 tf>=34.0 s。
#   这里用【步长递增探测】而不是严格二分：cgen 单次仅 ~0.06 s，
#   10 次探测约 0.6 s，而二分至少 6~7 次且逻辑更繁。
#   步长取 0.5 s：与旧的 +0.5 s 余量同量级，避免跳过最小可行点太远而多烧油。
# ============================================================================
_TF4_PROBE_STEP = 0.5    # 探测步长 [s]
_TF4_PROBE_N = 16        # 最多探测次数（ 0.5*16 = 8 s 余量）

# ============================================================================
# 【2026-09-30 第十二步 12(a)：规划质量闸门（峰值倾角上限）】
# ----------------------------------------------------------------------------
# 【问题（已确证）】上面的探测取【第一个可行 tf】即 break，而最小时间解在
#   边界上是【退化】的 —— 求解器用极端姿态去满足边界条件。
#
#   实测（cgen，mass 157.8 t，x0 = h 1111.03 / vz -57.36 / vh 5.81，这是
#   实飞 gfold_log_20260930_010341 的规划起点的复原）：
#       tf=18.05 -> 规划峰值倾角 82.95 deg     <- 探测会选中这一档
#       tf=18.15 -> 83.88
#       tf=18.25 -> 83.97
#       tf=18.30 -> 57.47
#       tf=18.35 -> 16.29
#       tf=18.40 ->  7.14                     <- 加 0.35 s 就完全正常
#   ⇒ 0.15~0.35 s 跨过一道悬崖，而这个"最省时间"的选择把控制器逼到
#     84 deg —— 远超载具能力（alpha 低油门端只有 4.6 deg/s^2）。
#
# 【怎么确证"飞行追的就是这条退化规划"】用日志的 plan_alt/plan_vz 两列还原
#   规划相图，与各 tf 下重新解出的规划曲线比对 RMS：
#       tf=18.05 RMS=0.43  (峰值倾角 82.95)
#       tf=18.25 RMS=1.48  (83.97)
#       tf=18.50 RMS=2.63  ( 6.87)
#       tf=22.00 RMS=12.55 ( 4.25)
#   RMS 随 tf 单调上升，最小值就在退化档；且实测 tilt_cmd 峰值 80.31 deg
#   与该档的 82.95 deg 吻合。对 x0 做 +-20 m / +-3 m/s 扰动后，最佳匹配的
#   规划倾角仍恒为 82.9~84.0 deg ⇒ 结论稳健。
#
# 【修法】探测到可行 tf 后【不要立刻采信】，先算这条规划的峰值倾角；
#   若超过 _PLAN_TILT_MAX_DEG 则继续向上探测，直到找到既可行又"姿态正常"
#   的规划。仍找不到就退回【第一个可行解】（宁可飞退化规划，也不能不给规划）。
#
# 【阈值取 20 deg 的依据（用户指定 20）】
#   · 非退化的正常规划峰值倾角实测 4.2~16.3 deg（上面的表 + 多种交接状态）
#   · 退化解实测 57~84 deg
#   · 20 deg 落在两者之间的空档里，两边的余量都很大（正常侧 +3.7、退化侧 -37）
#   · 与控制器侧 MAX_TILT_CMD_DEG=30 一致：规划要 20 以内，控制器才有余量
#     在 30 的锥内做修正，而不是一开始就贴着锥壁。
#
# 【代价】实测每次只需多探 1~4 档（步长 0.5 s），单次 0.06 s ⇒ 最多 +0.25 s
#   解算耗时；燃料代价 ~0.7 t（tf 18.25->18.5 时 146.3->145.6 t）。
#   相比"控制器追 84 deg 姿态导致失控"，这个代价可忽略。
# ============================================================================
_PLAN_TILT_MAX_DEG = 20.0   # 规划允许的峰值倾角 [deg]；超过则继续探测
_PLAN_TILT_GATE = True      # 闸门开关（False = 旧行为：取第一个可行 tf）


def _plan_peak_tilt_deg(u):
    """一条规划的【峰值倾角】[deg] = max atan2(|u[1:3]|, u[0])。

    【为什么单独写一个函数】控制器侧（gfold_land.py 的 plan_pk_tilt 列）
    用的是同一个口径，两处必须一致，否则"日志显示正常但求解器判退化"。
    【u 的形状】solver 内部一律是 (3, N)（见 gfold_p3p4.py 的 reshape）。
    """
    try:
        U = np.asarray(u, float)
        if U.ndim != 2 or U.shape[0] < 3 or U.shape[1] < 1:
            return float('nan')
        tw = np.degrees(np.arctan2(
            np.linalg.norm(U[1:3, :], axis=0),
            np.maximum(1e-09, np.abs(U[0, :]))))
        return float(np.max(tw))
    except Exception:                                # noqa: BLE001
        return float('nan')


# ============================================================================
# 【2026-09-29 第二步：下降率包络（v_descent_max）】
# ----------------------------------------------------------------------------
# 【为什么需要（症状）】纯燃料最优的解是 bang-bang：
#   先几乎不推地加速下坠，最后 1~2 s 猛刹。
#   实测（交班点 h=650 m）默认（无包络）的 peak|vz| = 119.6 m/s：
#       t=0.0 h=650 vz=-70 | t=2.4 h=469 vz=-86  | t=4.7 h=243 vz=-109
#       t=5.5 h=155 vz=-117| t=6.3 h= 67 vz=-98  | t=7.1 h= 12 vz=-43
#   这在数学上最优，但【物理上不可跟踪】——需要完美的时序，
#   任何偏差都会变成撞地。上游离线闭环实测：落点 0.21 m、
#   倾角 0.12° 都很漂亮，但【触地 vz = -96.9 m/s】（坠毁）。
#
# 【怎么做】给每个节点一个下降率上限 cap[n]，约束 x[3,n] >= -cap[n]。
#   cap 作为【参数向量】传入（而非编译期固定），因为：
#     · 对参数与变量都是【仿射】的 => DCP/DPP 安全
#       （对比：gfold_terminal.py 把 cap 写成编译期常量，故
#        它每次改包络都必须重建模型）
#     · cap 依赖当前质量 / 初速 / 剩余高度，每次解算都不同
#
# 【形状很重要（上游踩过的两个坑，见 gfold_terminal.py:154-204）】
#   坑① 写成"从 |vz0| 线性收到 0" => 全部 infeasible。
#     它把【总下降量】限死在 ~1100 m（积分 cap 得 72·T/2），
#     而实际要下降 650~5000 m。
#   坑② 第二代写成 cap = sqrt(2·a_brk·h_rem) + 5 后再与初速取 min，
#     在【带初速冲进来】的实飞交班点自相矛盾：
#         |vz0| = 314 而 cap = 160 => 节点 0 同时要 vz == -314 与 vz >= -160
#     修正：包络必须【从初速出发】并与初值自洽（cap0）。
#   另：a_brk 必须用【扣重力后的真实可用减速】，不是 0.35 倍满推
#     （后者把包络压得比初速还低，全程每个节点都违反）。
#
# 【对比：为什么做成参数而不是像 gfold_terminal 那样写死】
#   gfold_terminal.py 把 cap 算在模型里（用 h、mass、vz 作常量），
#   于是每个不同初态都要重建模型。而 codegen 路径的模型是
#   【编译期生成、运行时复用】的，按初态重建就意味着每次都要编译（数十秒）。
#   故必须把 cap 变成 Parameter。
# ============================================================================
# 【参数名】生成器与求解器共用同一个名字，避免两处漂移。
VDESC_PARAM = 'vdesc_cap'
# 【默认包络值】None = 不加包络（与 HEAD 行为一致，便于回退与 A/B）。
#   【为什么不直接设成 80】默认关闭可以保证：
#     · 未显式传参的调用方（诊断脚本等）行为与 HEAD 完全一致
#     · 真要启用时是【显式的一次改动】，便于 A/B 与快速回退
#   启用方式见 gfold_land.py 的 V_DESCENT_MAX_ENABLE。
V_DESCENT_MAX = None
# 【生成期自检用的包络】只用于 gfold_codegen.generate_one() 的 cvxpy 自查 ——
#   它保证 vdesc 这条新约束【真的被求解器验证过】，而不是"默认关闭、
#   从未执行"。取实测通过的工作点值 80 m/s。
#   【为什么是 80】见 gfold_codegen.py 里 vdesc 约束处的实测表：
#     70 会让巡航段 4/7 工况 infeasible；80 是【全部 7 个工况通过】的最小值。
SELFCHECK_VDESC = 80.0
# 【生成期自检用的状态】必须与 gfold_land.py 的 GATE_* 一致（真实工作点），
#   理由见 gfold_codegen.generate_one() 里的长注释：
#   旧状态 h=2963/vz=-180 需要 141 m/s 的平均下降率，与"限速 80"物理上
#   不相容，会得到假 infeasible。
SELFCHECK_H = 650.0        # 到目标点高度 = GATE_ALT(686.8) - TARGET_ALT(36.8)
SELFCHECK_Y = -300.0       # 水平偏差（锥内）
SELFCHECK_VZ = -70.0
SELFCHECK_VH = 60.0
SELFCHECK_MASS = 156.9e3
# 【末端窗口】最后 term_win 比例时间内，把 cap 压向 term_vz。
#   上游实测：无论 v_descent_max 取 160 还是 80，"末次 |vz|>50" 总出现在
#   alt≈56 m、剩 ~3 s —— 即总是"贴着地猛刹"。加窗口后末段平滑收到 term_vz。
TERM_WIN = 0.40
TERM_VZ = 4.0


def vdesc_cap_vector(N, h, vz, mass, t_max, tf, isp=315.0,
                     v_descent_max=None, term_win=TERM_WIN, term_vz=TERM_VZ,
                     verbose=False):
    """构造逐节点的下降率上限向量 cap[n]  [m/s]，供 x[3,n] >= -cap[n]。

    【两种用法】
      · v_descent_max is None -> 返回 None（调用方不加该约束）
        —— 这是 HEAD 行为，用于 A/B 与快速回退。
      · 数值 -> 返回长度 N 的 ndarray。

    【参数含义】
      h     : 当前到目标点的高度 [m]
      vz    : 当前垂速 [m/s]（下降为负）
      mass  : 当前质量 [kg]
      t_max : 可用推力 [N]（已含限幅与自校准）
      tf    : 本次规划的总时间 [s]（dt = tf/N）

    ================================================================
    【为什么必须传 tf（第一版的缺陷）】
    ----------------------------------------------------------------
      第一版没有 tf，写成：
          cap = min(vdm, env)
          cap = max(cap, min(cap0, env + 5.0))
      实测（h=650, vz=-70, m=156.9t, a_brk=47.5）得到的向量是
      【前 48 个节点恒为 70】（= cap0），即【它根本没有限制任何东西】：
          因为 env+5 在前段远大于 cap0，所以 min(cap0, env+5) = cap0
          —— 那一项把整条曲线抬到了初速上。
      而且它无法判断【这条包络到底能不能下到地面】：
          下降距离 = sum(cap)·dt，而 dt = tf/N 需要 tf。
      实测：vdm=40 时 sum(cap)·dt = 536.8 m < h=650 m
          => P4 直接 infeasible。而这个失败在旧版里
             「看不出原因」（只看到 infeasible）。

    【当前形状（四道限制取交）】
      ① 初速：cap 不低于 |vz0|（否则节点 0 自相矛盾）
      ② 可用减速：|vz| 只能按 a_brk 逐步下降 =>
          cap <= |vz0| + a_brk·t   对于加速段，但我们要限制的是
          【下降率上限】，故取 cap <= |vz0| - a_brk·t 与下一条取大
      ③ 自杀点火包络：cap <= sqrt(2·a_brk·h_rem) + margin
      ④ 巡航速度：cap <= v_descent_max（用户期望的匀速下滑速度）
      末端窗口：最后 term_win 比例内把 cap 压向 term_vz

    【可行性保护（最关键）】
      算完形状后检查 D = sum(cap)·dt。若 D < h，说明【tf 太短，
      用这么低的速度完成不了这段下降】。此时给全程叠加一个
      常数偏移 c = (h - D)/tf，使 sum(cap')·dt == h 恰好。
      这保证：
        · 约束在【距离意义上】恒可满足（不会因包络而 infeasible）
        · tf 充足时 c=0，包络完全生效
        · tf 不足时自动退化为“尽量平缓但仍能到达”
    """
    if v_descent_max is None:
        return None
    N = max(2, int(N))
    tf = max(1e-3, float(tf))
    dt = tf / N
    a_brk = max(1.0, float(t_max) / max(1.0, float(mass)) - G0)
    cap0 = abs(float(vz))
    vdm = float(v_descent_max)
    H = max(0.0, float(h))
    caps = np.zeros(N)
    for n in range(N):
        t = n * dt
        h_rem = max(0.0, H * (1.0 - n / (N - 1)))
        # ③ 自杀点火包络（天然满足“还来得及刹住”）
        env = float(np.sqrt(2.0 * a_brk * h_rem) + 5.0)
        # ① 初速：在能按 a_brk 减速的前提下，允许从 |vz0| 开始下降
        #      ramp = |vz0| - a_brk·t（下限）；下降率上限不能低于它
        ramp = cap0 - a_brk * t
        cap = min(vdm, env)
        cap = max(cap, ramp)
        if n == 0:
            cap = max(cap, cap0)
        # 末端窗口：平滑收到 term_vz
        frac_end = n / max(1, N - 1)
        if frac_end > (1.0 - term_win):
            w = (frac_end - (1.0 - term_win)) / max(1e-6, term_win)
            cap = min(cap, term_vz + (cap - term_vz) * max(0.0, 1.0 - w))
        caps[n] = max(cap, 1.0)
    # ---- 可行性保护：保证 sum(cap)·dt >= h ----
    D = float(np.sum(caps)) * dt
    if D < H:
        c = (H - D) / tf
        caps = caps + c
        if verbose:
            print('[vdesc] tf=%.1fs 太短（包络只能下降 %.0f m < %.0f m）'
                  ' -> 全程叠加 +%.1f m/s' % (tf, D, H, c))
    return caps


# ============================================================================
# 【C 代码生成求解器（cvxpygen）—— 加速路径】
# ============================================================================
# 【为什么需要】cvxpy 直接解在【实飞】要 3.6 s（KSP 抢占 CPU），
#   而 kOS 交班后火箭仍以 ~300 m/s 下坠 —— 这几秒会掉约 1100 m，
#   导致"轨迹起点"与"实际位置"严重错位。
#   原仓库本来就靠 C 代码生成解决（readme: 直接 cvxpy 可能慢 10 倍以上），
#   但它的 cvxpy_codegen 需要 cvxpy==0.4.11，在 Python 3.13 上装不上。
#   ⇒ 用现代后继 cvxpygen 生成（见 dev/gfold/solver/gfold_codegen.py）。
#
# 【实测】同一工况（alt=4432 / vz=−301 / vh=217 / 182.8 t）：
#     cvxpy 0.347 s   →   cpg 0.0084 s     加速 41.5 倍
#     末端位置误差两者都是 0.000000 m，轨迹最大逐元素差 0.09（求解容差内）
#
# 【设计】优先用 codegen；任何环节失败都【静默回退】到 cvxpy，
#   保证程序不会因为"扩展没编译/路径不对"而整个挂掉。
# ============================================================================
_CGEN_CACHE = {}
# 【为什么用变量而不是直接写】生成的 C 模型把 T_max 固化成 8.99e6
#   （见 gfold_codegen.T_MAX）。这里保持一致，避免两处漂移。
P_TMAX_DEFAULT = 8.99e6


class _CGenMod:
    """cvxpygen 生成扩展的轻量包装（模块 + 参数名）。

    【为什么包一层】不想往 ModuleType 实例上动态挂属性（_cp / _cpg_param_names），
    那会让静态检查报 reportAttributeAccessIssue。包一层后类型清晰。
    """

    def __init__(self, mod, cpg_params_cls, param_names):
        self.mod = mod
        self.cpg_params_cls = cpg_params_cls
        self.param_names = param_names

    def cpg_params(self):
        return self.cpg_params_cls()

    def cpg_updated(self):
        return self.mod.cpg_updated()

    def solve(self, upd, par):
        return self.mod.solve(upd, par)

    def set_solver_default_settings(self):
        self.mod.set_solver_default_settings()


def _cache_put(key, val):
    _CGEN_CACHE[key] = val
    return val


def _load_cgen(program, N):
    """尝试加载 cpg_p3 / cpg_p4 扩展。返回模块或 None。

    只在【生成时固化的 N 与实际 N 一致】时可用（模型维度是编译期固化的）。
    """
    key = (program, N)
    if key in _CGEN_CACHE:
        return _CGEN_CACHE[key]
    mod = None
    try:
        import importlib.util
        d = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         'cpg_p%d' % program)
        if os.path.isdir(d):
            pyd = None
            for f in os.listdir(d):
                if f.endswith(('.pyd', '.so')):
                    pyd = os.path.join(d, f)
                    break
            if pyd:
                # 【模块名唯一化】.pyd 的初始化符号在编译期生成（PyInit_<name>），
                #   两个扩展若都叫 cpg_module，CPython 会按名字缓存 ——
                #   先加载 P3 后加载 P4 会拿到 P3（实测报
                #   "cannot reshape array of size 960 into (6,80)"，960=6×160）。
                #   importlib 层的各种绕法都无效，故在生成时就把模块名改成
                #   cpg_module_p3 / cpg_module_p4（见 gfold_codegen.rename_module）。
                modname = 'cpg_module_p%d' % program
                spec = importlib.util.spec_from_file_location(modname, pyd)
                if spec is None or spec.loader is None:
                    return _cache_put(key, None)
                m = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(m)
                # 【类名保持原名 + py::module_local()】
                #   pybind11 的类注册以 C++ typeid 为键，是【全局】的；
                #   两个扩展里的 CPG_*_cpp_t 同名会互相冲突。官方解法是
                #   给每个 py::class_ 加 py::module_local()（见
                #   gfold_codegen.rename_module），这样类名不必改。
                #   模块名仍需唯一（PyInit 符号）。
                # 用一个轻量包装持有模块与参数名，避免往 ModuleType 上挂属性
                # （那样 pyright 会报 reportAttributeAccessIssue）。
                cpg_params = m.cpg_params
                names = [n for n in dir(cpg_params) if not n.startswith('_')]
                if 'x0' in names:
                    mod = _CGenMod(m, cpg_params, names)
    except Exception as e:                       # noqa: BLE001
        # 【不要静默】加载失败时打印一次原因，否则"为何没走 codegen"无从判断
        if not getattr(_load_cgen, '_warned', False):
            _load_cgen._warned = True
            print('[cgen] 加载扩展失败: %s: %s' % (type(e).__name__, e))
        mod = None
    # ================================================================
    # 【2026-09-28 新增：启动可见性】
    #   事故：清理时删掉了 cpg_p3/ 与 cpg_p4/，程序【静默】回退 cvxpy，
    #   实飞 solve_ms 从 ~0.08 s 恶化到 3441 ms（48 倍），载具在解算期间
    #   白掉 118 m、整段失控。而当时的提示只有一行淹没在刷屏里的
    #   「[cgen] 加载扩展失败」—— 这种失败必须【不可能被忽略】。
    #   现在每个 program/N 组合都打印一行明确的 已加载/未加载。
    # ================================================================
    _tag = 'P%d(N=%d)' % (program, N)
    if mod is not None:
        print('[cgen] %s C 求解器已加载（快速路径）' % _tag)
    else:
        _dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'cpg_p%d' % program)
        print('[cgen] *** %s 未加载 -> 回退 cvxpy（慢约 48 倍，实飞会掉高度）***'
              % _tag)
        if not os.path.isdir(_dir):
            print('[cgen]     目录不存在: %s' % _dir)
            print('[cgen]     修复: pwsh -File tools/gen_codegen.ps1')
        else:
            _files = [f for f in os.listdir(_dir)
                      if f.endswith(('.pyd', '.so'))]
            if not _files:
                print('[cgen]     目录存在但没有 .pyd/.so: %s' % _dir)
                print('[cgen]     修复: pwsh -File tools/gen_codegen.ps1')
            else:
                print('[cgen]     找到 %s 但加载失败（见上方原因）' % _files[0])
    _CGEN_CACHE[key] = mod
    return mod


def _solve_one_cgen(program, x0, mass, tf, N, pcs_deg=None, pcs_start_deg=None,
                    vdesc=None):
    """用 C 代码生成求解器求解。成功返回与 _solve_one 同构的 dict。

    【tf 已参数化】生成的模型把 tf 作为 Parameter（验证过 DCP+DPP 均通过），
    所以【一份模型覆盖任意 tf】—— 不再需要按 tf 网格生成多份。
    只有 N（节点数）是编译期固化的，必须与生成时一致。

    ========================================================================
    【2026-09-29 第一步改动 1d：pcs_deg / pcs_start_deg 在 codegen 路径上
      【不起作用】—— 现在会明确告警，不再静默忽略】
    ------------------------------------------------------------------------
    【事故】生成的 C 模型把锥角曲线【编译期固化】进约束矩阵，运行时无法
      修改。而调用方 gfold_land.solve() 一直在传 pcs_deg=PCS_END_DEG，
      看起来"改了参数就生效"，实际【完全无效】：
          实测 pcs_deg=30 与 pcs_deg=5 走 codegen 得到【逐位相同】的轨迹
          （末端倾角都是 9.52°）。
      这让人误判"末端倾角已经按 pcs_deg 控制好了"，而真相是它一直是
      生成时烧进去的 gfold_codegen.PCS_END_DEG。
    【修法】把两个参数收进来，与 GC 的同名常量比对；不一致就打印一行
      【醒目】告警（只打一次），并说明该改哪个文件。仍然走 codegen
      （它快 48 倍），只是不再骗人。
    【正确的改法】改锥角要改 dev/solver/gfold_codegen.py 的
      PCS_START_DEG / PCS_END_DEG，然后重跑 tools/gen_codegen.ps1。
    ========================================================================
    """
    m = _load_cgen(program, N)
    if m is None:
        return None
    try:
        import gfold_codegen as GC
        want_n = GC.N_P3 if program == 3 else GC.N_P4
        if N != want_n:
            return None                          # 维度固化，必须一致
        # ================================================================
        # 【1d】锥角是编译期固化的：调用方传的值【不生效】。
        #   不一致时给一行醒目告警（只打一次），避免"改了参数以为生效"。
        #   判据用【有效值】：未传(None)表示"沿用生成时的值"，视为一致。
        # ================================================================
        _ps = GC.PCS_START_DEG if pcs_start_deg is None else float(pcs_start_deg)
        _pe = GC.PCS_END_DEG if pcs_deg is None else float(pcs_deg)
        if (abs(_ps - GC.PCS_START_DEG) > 1e-9
                or abs(_pe - GC.PCS_END_DEG) > 1e-9):
            if not getattr(_solve_one_cgen, '_cone_warned', False):
                _solve_one_cgen._cone_warned = True
                print('[cgen] *** 锥角参数被忽略：codegen 模型已把锥角固化 ***')
                print('[cgen]     调用方想要 pcs_start=%.1f pcs_end=%.1f'
                      % (_ps, _pe))
                print('[cgen]     模型实际是 pcs_start=%.1f pcs_end=%.1f'
                      % (GC.PCS_START_DEG, GC.PCS_END_DEG))
                print('[cgen]     改锥角请改 dev/solver/gfold_codegen.py 的'
                      ' PCS_START_DEG / PCS_END_DEG 并重跑 gen_codegen.ps1')
        par = m.cpg_params()
        upd = m.cpg_updated()
        names = m.param_names
        # 【第二步】下降率包络也是运行时参数（模型里已声明为 Parameter）。
        #   vdesc is None -> 用一个大值，等价于"不加包络"（与 HEAD 一致）。
        #   注意 P3 模型【声明了该参数但不施加约束】，arg 仍要赋值，
        #   否则 param_names 里它有名字却没值 -> 下面直接 return None。
        if vdesc is None:
            _vd_list = [1.0e4] * int(N)
        else:
            _va = np.asarray(vdesc, float).reshape(-1)
            _vd_list = [float(max(x, 1.0e-3)) for x in _va]
            if len(_vd_list) != int(N):
                return None
        valmap = {
            'x0': [float(v) for v in np.asarray(x0).flatten()],
            'm_wet_log': float(np.log(mass)),
            's_hi0': float(P_TMAX_DEFAULT * 0.8 / mass),
            's_lo0': float(P_TMAX_DEFAULT * 0.1 / mass),
            'tf': float(tf),
            'vdesc_cap': _vd_list,
        }
        for nm in names:
            if nm not in valmap:
                return None
            v = valmap[nm]
            setattr(par, nm, v if isinstance(v, list) else float(v))
            setattr(upd, nm, 1)
        m.set_solver_default_settings()
        res = m.solve(upd, par)
        # 【字段名】cpg_result_pN 的【类名】带了后缀，但它的【成员字段】
        #   仍叫 cpg_prim / cpg_info（生成器只改了类名，没改字段名）。
        #   实测报错：'cpg_result_p3' object has no attribute 'cpg_prim_p3'。
        prim = res.cpg_prim
        info = res.cpg_info
        if int(info.status) != 0:
            return None
        xs = np.array(prim.x).reshape((6, N), order='F')
        us = np.array(prim.u).reshape((3, N), order='F')
        zs = np.array(prim.z).reshape((1, N), order='F')
    except Exception as e:                       # noqa: BLE001
        # 【排障用】回退是静默设计，但第一次失败时打印一次原因，
        #   否则"为何没走 codegen"完全看不出来。
        if not getattr(_solve_one_cgen, '_warned', False):
            _solve_one_cgen._warned = True
            print('[cgen] 回退 cvxpy 原因: %s: %s' % (type(e).__name__, e))
        return None

    mv = np.exp(zs)
    a_mag = np.linalg.norm(us, axis=0)
    a_lim = P_TMAX_DEFAULT / mv[0, :]
    return {
        'status': 'optimal', 'tf': tf,
        'x': xs, 'u': us, 'm': mv, 's': None, 'z': zs,
        'mass_land': float(mv[0, -1]), 'fuel': float(mass - mv[0, -1]),
        'end_pos_err': float(np.linalg.norm(xs[0:3, -1])),
        'end_vel_err': float(np.linalg.norm(xs[3:6, -1])),
        'thrust_over_ratio': float(np.max(a_mag / np.maximum(a_lim, 1e-9))),
        'end_tilt_deg': float('nan'), 'peak_tilt_deg': float('nan'),
        'tf_m': -1.0, 'solve_time': 0.0, 'stage': '', 'engine': 'cgen',
    }


def _pack(x0, mass, isp, t_max, throttle, tf, gs_deg, pcs_deg, v_max, N,
          straight_fac, alpha):
    """对应原版 GFOLD_run.solver.pack_data()。

    原版:
        dt = self.tf_ / N
        alpha_dt = self.alpha * dt
        t = np.linspace(0, (N-1) * dt, N)
        z0_term = self.m_wet - self.alpha * self.r2 * t
        z0_term_inv = (1 / z0_term)
        z0_term_log = np.log(z0_term)
    注意原版 dt = tf_/N（不是 tf/(N-1)），这里【照抄】。
    """
    dt = tf / N
    alpha_dt = alpha * dt
    t = np.linspace(0, (N - 1) * dt, N)
    r2 = t_max * throttle[1]
    z0_term = mass - alpha * r2 * t
    # 数值保护：z0_term 必须为正（否则 log/倒数发散）。原版假设 tf 不会长到烧干。
    z0_term = np.maximum(z0_term, 0.05 * mass)
    return dict(dt=dt, alpha_dt=alpha_dt,
                z0_term_inv=(1.0 / z0_term).reshape(1, N),
                z0_term_log=np.log(z0_term).reshape(1, N),
                r1=t_max * throttle[0], r2=r2, tf=tf,
                straight_fac=straight_fac, gs_cot=1.0 / np.tan(np.radians(gs_deg)),
                pcs_cos=np.cos(np.radians(pcs_deg)), v_max=v_max, N=N)


def _fail(status, tf=-1.0, **extra):
    """失败返回：与成功路径【同构】的 dict（所有键都在）。

    【为什么统一形状（pyright 排查后改）】
      原先失败路径写 `return {'status': ...}`，成功路径返回十几键的 dict。
      pyright 据此把它们并成 `dict[str, str | float | ndarray]`，
      于是调用方 `r['tf'] / 1000`、`r['x'].shape` 全部报"str 没有 shape"
      之类的 15 条错误 —— 把真正的问题淹没了。
      统一形状后类型清晰，调用方也不必到处判 key 是否存在。
    """
    d = {
        'status': status, 'tf': float(tf),
        'x': None, 'u': None, 'm': None, 's': None, 'z': None,
        'mass_land': 0.0, 'fuel': 0.0,
        'end_pos_err': float('nan'), 'end_vel_err': float('nan'),
        'thrust_over_ratio': float('nan'),
        'end_tilt_deg': float('nan'), 'peak_tilt_deg': float('nan'),
        'tf_m': -1.0, 'solve_time': 0.0, 'stage': '',
    }
    d.update(extra)
    return d


def _solve_one(program, x0, mass, isp, t_max, throttle, tf, gs_deg, pcs_deg,
               v_max, N, straight_fac, target_alt, pcs_start_deg=None,
               verbose=False, v_descent_max=None,
               term_win=TERM_WIN, term_vz=TERM_VZ):
    """对应原版 GFOLD_direct_exec.GFOLD_direct(N, pmark, packed_data)。

    【第二步新增】v_descent_max / term_win / term_vz 控制【下降率包络】：
      非 None 时逐节点加 x[3,n] >= -cap[n]，cap 由 vdesc_cap_vector() 给出。
      详见 vdesc_cap_vector 的文档字符串（含上游两个坑的完整记录）。
      None（默认）= 不加该约束，与 HEAD 行为一致（便于 A/B 与回退）。
    """
    alpha = 1.0 / G0 / isp
    alpha_dt_par = _pack(x0, mass, isp, t_max, throttle, tf, gs_deg, pcs_deg,
                         v_max, N, straight_fac, alpha)
    dt = alpha_dt_par['dt']
    alpha_dt = alpha_dt_par['alpha_dt']
    z0_term_inv = alpha_dt_par['z0_term_inv']
    z0_term_log = alpha_dt_par['z0_term_log']
    r1, r2 = alpha_dt_par['r1'], alpha_dt_par['r2']
    y_gs_cot = alpha_dt_par['gs_cot']
    p_cs_cos = alpha_dt_par['pcs_cos']
    m_wet_log = np.log(mass)
    g = np.array([-G0, 0.0, 0.0])
    # ---- 第二步：算好逐节点下降率上限（None 表示不加该约束）----
    _vdesc = vdesc_cap_vector(N, float(x0[0]), float(x0[3]), mass, t_max, tf,
                              isp=isp, v_descent_max=v_descent_max,
                              term_win=term_win, term_vz=term_vz,
                              verbose=verbose)

    # 高度基准：本项目 x0[0] 已含 target_alt（见 solve_p3p4 的换算）
    x = cp.Variable((6, N), name='var_x')
    u = cp.Variable((3, N), name='var_u')
    z = cp.Variable((1, N), name='var_z')
    s = cp.Variable((1, N), name='var_s')

    con = []
    con += [x[0:3, 0] == x0[0:3]]           # 初位置
    con += [x[3:6, 0] == x0[3:6]]           # 初速度
    con += [x[3:6, N - 1] == np.zeros(3)]   # 末速度必须为零
    con += [s[0, N - 1] == 0]               # 末端推力为零
    con += [z[0, 0] == m_wet_log]           # 质量初值

    # ================================================================
    # 【2026-09-27 关键修正：恢复原版的【起点推力竖直】约束】
    # ================================================================
    # 【原版 GFOLD_direct_exec.py:59-60】
    #     con += [u[:,0]   == s[0,0]  *np.array([1,0,0])]   # thrust START straight
    #     con += [u[:,N-1] == s[0,N-1]*np.array([1,0,0])]   # thrust END straight
    #   两端都强制竖直。
    #
    # 【我此前删掉了起点那条】理由写在旧注释里："带横速入场时起点不能强制竖直"。
    #   **这个理由是错的**，实测证据（log ..._164450，交接 alt=3167 vz=-82.98 vh=47.73）：
    #     · 删掉后，求解器把【节点 0 的倾角解成 85°】：
    #           u[:,0] = [3.785, 0, 43.264]   tilt = 85.0°
    #     · 而节点 0 正是【交接后第一帧立刻执行】的指令。
    #     · kOS 交接时载具姿态只有 tilt ≈ 24.5°（实测 tilt_act=24.52）。
    #       ⇒ 要求它在约 0.2 s 内转 60.5° —— 物理上不可能
    #         （实测姿态权限 α≈13.5°/s²，转 60° 需 ~4.2 s）。
    #       ⇒ 推力方向错 ⇒ 前 3 帧 0.86~0.95 的油门几乎全用在【竖直减速】
    #       ⇒ vz 从 -82.98 掉到 -69.65（比计划快 20 m/s 地刹住）
    #       ⇒ vterm[0] 变负 ⇒ conic_clamp 把水平指令清零 ⇒ 完全不跟踪。
    #
    # 【恢复后实测】同一个初值：
    #     · 节点 0 = [43.429, 0, 0]  tilt = 0.0°（纯竖直，与交接姿态 24.5° 只差 24.5°）
    #     · 且【仍然求解成功】：tf=24.50、落地 145.5 t
    #       （对比删掉时 tf=24.25、落地 146.3 t —— 只少 0.8 t）
    #   ⇒ 当初"起点不能竖直"的担心【不成立】；代价仅 0.8 t，
    #     换来的是【首帧姿态可执行】。
    #
    # 【为什么这条约束如此关键】它让计划的两端都是竖直的：
    #   起点竖直 ←→ 与"载具刚从段1的准竖直姿态交接过来"天然吻合
    #   末端竖直 ←→ 保证触地姿态
    #   中间的倾斜段由 pcs 时变锥放开，仍能完成大横速制动。
    # ================================================================
    con += [u[:, 0] == s[0, 0] * np.array([1, 0, 0])]      # 原版 line 59
    con += [u[:, N - 1] == s[0, N - 1] * np.array([1, 0, 0])]  # 原版 line 60

    if program == 3:
        con += [x[0, N - 1] == 0]                       # P3: 只要高度归零
    elif program == 4:
        con += [x[0:3, N - 1] == np.zeros(3)]           # P4: 位置全部归零

    # ================================================================
    # 【2026-09-27 已核查：末端节点 N-1 无约束 —— 不是 bug】
    # ================================================================
    # 【曾怀疑】本循环原为 `for n in range(0, N - 1)`（照抄原版 line 74），
    #   末端节点 N-1 不进循环 ⇒ 不受推力上限/锥角/滑翔角/速度上限约束。
    #   我据此改了循环并加了末端推力上界。
    #
    # 【核查结论：这个怀疑是错的，已回退】用【随质量变化的正确上限】
    #   r2/m(z) 逐个节点核对生产轨迹（log ..._164450 的交接状态）：
    #       nodes violating the true bound: 0
    #       node 77 |u|=48.659  bound=48.83
    #       node 78 |u|=48.891  bound=49.06
    #       node 79 |u|= 0.000  bound=49.18
    #   —— 全部合规。
    #   【我之前"18 个节点超限"是【我自己的错误】：拿了
    #     r2/m_wet（用【初始】质量算的常数）去比，而真实上限随燃油消耗
    #     从 43.3 涨到 49.2。】
    #   另外末端有 s[0,N-1]==0 与 u[:,N-1]==s[0,N-1]*[1,0,0]（见上方
    #   line 299/305），末端推力被强制为 0 —— 这是原版设计，故意为之。
    #
    # 【教训】"约束缺失"这类判断必须用【该约束自己的量纲与取值时点】去核，
    #   否则会把自己的算术错误当成求解器的 bug，并据此改坏正确的模型。
    #   实测：把循环扩到 N 并在 N-1 加上 mu_1（最小油门下界）会让问题
    #   立刻 infeasible（tf=18..40 全挂），因为它与"末端推力=0"矛盾。
    # ================================================================
    for n in range(0, N - 1):
        con += [x[3:6, n + 1] == x[3:6, n]
                + (dt * 0.5) * ((u[:, n] + g) + (u[:, n + 1] + g))]
        con += [x[0:3, n + 1] == x[0:3, n]
                + (dt * 0.5) * (x[3:6, n + 1] + x[3:6, n])]
        # 滑翔角锥（原版 line 75）
        con += [cp.norm(x[1:3, n]) - y_gs_cot * x[0, n] <= 0]
        # 速度上限（原版 line 77）
        con += [cp.norm(x[3:6, n]) <= v_max]
        # 质量递减（原版 line 79）
        con += [z[0, n + 1] == z[0, n]
                - (alpha_dt * 0.5) * (s[0, n] + s[0, n + 1])]
        # 推力幅值与指向锥（原版 line 80, 83）
        con += [cp.norm(u[:, n]) <= s[0, n]]
        if pcs_start_deg is None:
            con += [u[0, n] >= p_cs_cos * s[0, n]]
        else:
            # 项目修正（坑①）：锥角按时序从宽收到窄，而不是全程固定。
            #   原版固定 p_cs；本项目入场横速大，全程固定会 infeasible。
            frac = n / max(1, N - 2)
            ang = np.radians(pcs_start_deg + (pcs_deg - pcs_start_deg) * frac)
            con += [u[0, n] >= np.cos(ang) * s[0, n]]
        # 地面以下禁止（原版注释掉了，本项目保留：防止"先落地再飞起"）
        con += [x[0, n] >= 0]
        # ================================================================
        # 【2026-09-29 第一步改动 4：竖直铁律 —— 全程不许爬升】
        #   与上面的 x[0,n] >= 0 配对。只有"不穿地"时，求解器仍可给出
        #   "俯冲-回弹"畸形解（diag10 实测节点 28 高度 0、垂速 -1.3，
        #   随后爬到 157 m 再落回）。
        #   真实回收剖面单调下降，这条不损失最优性，只封掉畸形解。
        #
        #   【只对 P4 施加 —— P3 必须排除】实测（gate 686.8/d300/-70/60/
        #   156.9 t，N3=160，tf_guess=40）：
        #        no_climb OFF : P3 在节点 36 落到 h=0.0 并停住
        #                       -> tf_m 检测(位置+速度 < 0.1)命中节点 36
        #                       -> tf_m = 9.0 s
        #        no_climb ON  : 轨迹形状几乎不变，但"停住"的那段变成
        #                       h=0.2（而不是 0.0）—— 因为要满足 vz<=0，
        #                       离散解把驻留高度抬了 0.2 m
        #                       -> 0.2 > 0.1 的检测阈值 -> 检测直到节点 158
        #                       才命中 -> tf_m = 39.5 s（虚高 4.4 倍）
        #   ⇒ 这条约束本身【在物理上无害】（两版轨迹逐节点几乎相同），
        #     但它暴露了 P3 的 tf_m 检测阈值（绝对 0.1 m）过于脆弱。
        #     tf_m 是 tf4 的输入，tf4 直接决定 P4 的轨迹长度 —— 虚高 4.4 倍
        #     会让 P4 规划一条 40 s 的松散轨迹，实际飞行完全跟不上。
        #   【为什么不在本次修阈值】阈值一改就同时改变了 P3 的既有行为，
        #     属于"顺手重构"，违背"一次只改一件事、改完就能验证"的原则。
        #     本次取最小改法：P3 保持原样（它的轨迹本来就【被丢弃】，
        #     唯一产物 tf_m 用原阈值即可），只把竖直铁律施加在真正要飞的
        #     P4 上。阈值脆弱性已记在此处，留作后续单独处理。
        #   【与 codegen 的关系】codegen 对 P3/P4 【各生成一份独立模型】
        #     （gc.generate_one(3,…) 与 (4,…)），所以 gfold_codegen.py 里
        #     同样必须写成 program == 4 才加 —— 两边语义要一致。
        # ================================================================
        if program == 4:
            con += [x[3, n] <= 0]
        # ---- 第二步：下降率包络（参数向量，仿射约束，DCP/DPP 安全）----
        if _vdesc is not None:
            con += [x[3, n] >= -float(_vdesc[n])]

        if n > 0:
            z0 = z0_term_log[0, n]
            mu_1 = r1 * z0_term_inv[0, n]
            mu_2 = r2 * z0_term_inv[0, n]
            # 原版 line 99-100（含有二阶项的推力下界 / 一阶上界）
            con += [s[0, n] >= mu_1 * (1 - (z[0, n] - z0)
                                       + (z[0, n] - z0) ** 2 * 0.5)]
            con += [s[0, n] <= mu_2 * (1 - (z[0, n] - z0))]
        else:
            # ---------------------------------------------------------------
            # 【本项目修正：node 0 的推力上界（原版遗漏）】
            #   原版把推力上下界写在 `if n > 0` 里，于是 **node 0 完全没有
            #   推力上界**。而 n=0 恰好是【当前时刻、马上要执行的那一帧】。
            #   实测（实飞交班点 alt=5031.8 / dist=1285 / vz=-314 / vh=361.5）:
            #       原版移植版解出 |u[:,0]| = 812.5 m/s²
            #       而该节点真实上限 T/m = 49.18  ->  超 16.5 倍
            #   其余所有节点都正常（ratio <= 0.80）。
            #   ⇒ 这说明原版的遗漏在它的用例里无害（throttle=(0.2,0.8)、
            #     tf=80 s 很宽松，轨迹平缓），但在本项目这种"带大横速入场、
            #     时间紧"的工况下会直接给出一个物理上飞不出来的首帧指令。
            #
            # 【修法】node 0 直接用【真实推力上限】约束（此时 z[0,0]=log(m_wet)
            #   已知，无需线性化），与其余节点保持一致语义。
            con += [s[0, 0] <= r2 / mass]          # = T_max*throttle[1] / m_wet
            if throttle[0] > 0:
                con += [s[0, 0] >= r1 / mass]

    if program == 3:
        # 原版：Minimize( Σ ‖x[0:3,i]‖·(i/N) )  —— 尽快靠近目标
        expr = 0
        for i in range(N):
            expr += cp.norm(x[0:3, i]) * (i / N)
        prob = cp.Problem(cp.Minimize(expr), con)
    else:
        # ================================================================
        # 【本项目修正 ①：水平速度的权重由 (i/N) 改为【常数 1】】
        # ================================================================
        # 原版（GFOLD_direct_exec.py line 126-129）：
        #     for i in range(N):
        #         expression += norm(x[4:6,i]) * (i/N)
        #     expression *= straight_fac
        #     expression += -z[0,N-1]*N
        # 权重 (i/N) 在【早期趋近 0、晚期趋近 1】—— 即只惩罚"末段的水平速度"，
        # 早期水平速度几乎不花钱。参考 params.txt 的注释也印证了这个意图：
        #     straight_fac = 5  # 值越大，末段越直
        #
        # 【为什么本项目必须改（2026-09-26 实飞跟踪不稳的根因）】
        #   本项目交班点的水平能量远大于参考用例：
        #       参考 x0=[1400, 450, -330, -20, 40, 40]  水平 450 m / 横速 40 m/s
        #       本项目   水平 1314 m / 横速 221 m/s     （大 5.5 倍）
        #   垂直下降被下降率约束拖到 ~21 s，而水平只需 ~12 s 就能消完，
        #   多出来的 ~9 s 让"燃料最优"解选择【先朝目标加速冲过去，再反向制动】。
        #   实测（alt=4994 / vz=−287 / vh=221）P4 的水平推力 u[1]：
        #       节点 0..5: +38.9 … +39.2   ← 与 vy 同向 => 加速
        #       节点 6..:  −39.3 …         ← 反向制动
        #   于是节点 5→6 的【推力方向瞬间翻转 156°】—— 而 KSP 一级姿态转动很慢，
        #   跟踪器根本跟不上，表现为：
        #       · tilt_cmd 在 0° 与 80° 之间反复跳
        #       · tilt_act 与 tilt_cmd 正负交替差 ±30~66°
        #       · 横向先冲到 dist=62 m 又飞走到 1240 m
        #
        # 【改法】把权重由 (i/N) 改为常数 1 —— 全程同等惩罚水平速度，
        #   不再"只压末段"。这样水平机动没有"早期免费"的空间，
        #   求解器自然改为【一开始就制动】。
        # 【实测效果】
        #   alt 1500/vh 90 : 最大 |Δu_hor| 65.4 → 26.1
        #   alt 4994/vh 221: 最大 |Δu_hor| 71.1 → 25.8，推力方向跳变 156° → 31°
        #   u_hor 从"前 7 节点 +39 加速"变为"一开始 −34.9 制动"
        #   （代价：轻微多耗燃料，属可接受范围）
        # 【仍照抄原仓库的部分】目标函数结构（水平速度项 − 落地质量项）、
        #   straight_fac 的含义、P3 分支均未改动。
        expr = 0
        for i in range(N):
            expr += cp.norm(x[4:6, i])           # 常权重（原为 * (i / N)）
        expr = expr * straight_fac - z[0, N - 1] * N
        prob = cp.Problem(cp.Minimize(expr), con)

    try:
        prob.solve(solver=SOLVER_NAME, verbose=verbose)
    except Exception as e:                      # noqa: BLE001
        return _fail(f'EXC: {e}', tf)
    if z.value is None or x.value is None or u.value is None:
        return _fail(prob.status, tf)
    m = np.exp(z.value)
    a_mag = np.linalg.norm(u.value, axis=0)
    a_lim = t_max / m[0, :]
    return {
        'status': prob.status, 'tf': tf,
        'x': x.value, 'u': u.value, 'm': m, 's': s.value, 'z': z.value,
        'mass_land': float(m[0, -1]), 'fuel': float(mass - m[0, -1]),
        'end_pos_err': float(np.linalg.norm(x.value[0:3, -1])),
        'end_vel_err': float(np.linalg.norm(x.value[3:6, -1])),
        'thrust_over_ratio': float(np.max(a_mag / np.maximum(a_lim, 1e-9))),
        'end_tilt_deg': float('nan'), 'peak_tilt_deg': float('nan'),
        'tf_m': -1.0, 'solve_time': 0.0, 'stage': '',
    }


def solve_p3p4(alt, dist, vz, vh, mass, isp=315.0, t_max=8.99e6,
               throttle=(0.1, 0.8), tf_guess=40.0, straight_fac=5.0,
               gs_deg=30.0, pcs_deg=30.0, pcs_start_deg=85.0, v_max=1200.0,
               N3=160, N4=80, target_alt=36.8, verbose=False):
    """参考仓库的完整求解流程（GFOLD_run.solver.solve）。

    流程（逐条对应 GFOLD_run.py）：
        1. P3 用 tf_guess 求解，从轨迹里读出真正落地时刻 tf_m
        2. tf = tf_m + 0.1 * straight_fac
        3. P4 在 tf 上求解燃料最优

    【tf_guess 取值（本项目标定，非原版默认）】
      原版 params.txt 的 tf = 20 s，那是给"轻载、低横速、start_altitude=1100 m"
      的用例定的。本项目是【带大横速入场】的实飞交班点
      （alt 5031 / dist 1285 / vz −314 / vh 361），实测扫描：
          tf=20 -> P3 infeasible（推力锥咬死：需 ~61° 倾角，
                   而锥在 tf=20 的时程内已收紧到 ~58°）
          tf>=25 -> optimal，tf_m≈21 s，落地 136.5 t，末端倾角 0.00°
      故取 40 s。注意 P3 的 tf 只是【上限】，真正的落地时刻由轨迹读出
      （tf_m），所以给大一点是安全的、不会浪费。

    【水平初值：一维 vs 三维（2026-09-26 修正）】
      本函数【向下兼容两种调用】。
      · 传 (dist, vh) 标量：按"一维朝目标轴"构造 x0=[h, dist, 0, vz, -vh, 0]。
        这【不是】参考仓库的用法，只在本文件自测时用。
      · 传 (y, z, vy, vz_h) 三维：见 solve_p3p4_state()。
      参考仓库 demo3_gfold.py line 185 直接传【三维带符号】状态，
      因为它的 ref_target 与求解器坐标系同构、且 PD 就是 error 直接相减。
      """
    h = alt - target_alt
    return _solve_p3p4_impl(np.array([h, dist, 0.0, vz, -vh, 0.0]), mass,
                            isp=isp, t_max=t_max, throttle=throttle,
                            tf_guess=tf_guess, straight_fac=straight_fac,
                            gs_deg=gs_deg, pcs_deg=pcs_deg,
                            pcs_start_deg=pcs_start_deg, v_max=v_max,
                            N3=N3, N4=N4, target_alt=target_alt,
                            verbose=verbose)


def solve_p3p4_state(x0, mass, **kw):
    """**参考仓库的原生入口**：直接吃【三维带符号】初状态。

    对应 GFOLD_run.solver.set_params(v_data) 里的
        self.x0 = v_data['x0']
    以及 demo3_gfold.py vessel_profile1() line 185：
        'x0' : np.array([pos_est[0], pos_est[1], pos_est[2],
                         vel_est[0], vel_est[1], vel_est[2]])

    【为什么必须提供这个入口（2026-09-26 闭环发散事故的根因）】
      参考仓库的 x0 是【三维带符号 ENU】：
          GFOLD_run.py 测试用例： [1400, 450, -330, -20, 40, 40]
              —— z=-330 非零、vy/vz 都是正的
          另一用例：             [230.6, -259.6, 107.3, -1424.0, -153.4, 25.9]
              —— y=-259.6 是【负的】
      而 PD 跟随就是 `target_a = u_i + (v_i-vel)*k_v + (x_i-error)*k_x`，
      error = vessel.position(ref_target)，【直接相减、不做投影】。

      我此前把求解器写成了"一维朝目标轴"（x0=[h, dist, 0, vz, -vh, 0]，
      其中 dist 恒正、vh 恒负），于是：
        · 求解器把水平偏差全塞在 +y 轴
        · 而 kRPC 的 error 可能给出 y 负值（火箭在目标的另一侧）
        · 两者相减 ⇒ pos_err 凭空多出 2·dist ⇒ 水平指令爆炸
          （离线闭环实测：dist 从 1278 涨到 4334，vz 从 −314 涨到 +4494）
      修法不是"加投影"，而是【让求解器和参考仓库一样吃三维带符号状态】。
    """
    kw.pop('alt', None)
    kw.pop('dist', None)
    kw.pop('vz', None)
    kw.pop('vh', None)
    return _solve_p3p4_impl(np.asarray(x0, dtype=float), mass, **kw)


def _solve_p3p4_impl(x0, mass, isp=315.0, t_max=8.99e6,
                     throttle=(0.1, 0.8), tf_guess=40.0, straight_fac=5.0,
                     gs_deg=30.0, pcs_deg=30.0, pcs_start_deg=85.0,
                     v_max=1200.0, N3=160, N4=80, target_alt=36.8,
                     verbose=False, v_descent_max=None,
                     term_win=TERM_WIN, term_vz=TERM_VZ):
    """P3 → tf_m → P4 的公共实现（x0 已就绪，三维带符号）。

    【第二步新增】v_descent_max / term_win / term_vz：
      下降率包络，【只施加在 P4 上】（真正要飞的那条轨迹）。
      理由与 no_climb、末端锥角相同：P3 的轨迹被丢弃，只用来估 tf_m，
      而 tf_m 是 tf4 的输入 —— 改变 P3 的轨迹形状就会改变 tf_m，
      进而改变 P4 的规划长度。本次保持 P3 与 HEAD 完全一致。
      详见 vdesc_cap_vector 的文档字符串。
    """
    import time as _t
    t0 = _t.time()
    # ---- P3：同样优先走 C 代码生成 ----
    #   P3 的 N3=160（比 P4 大一倍），cvxpy 解要 1.5~2.8 s，是现在的主要瓶颈。
    #   实测 tf_m 对 N3 敏感（N3=40 会退化到 38 s），故【不降 N3】，
    #   改为生成 N3=160 的 C 求解器。
    # 【第二步】P3 也要带包络 —— 它决定 tf_m，进而决定 tf4。
    #   详见 gfold_codegen.py 里 vdesc 约束处的实测表。
    _vdesc3 = vdesc_cap_vector(N3, float(x0[0]), float(x0[3]), mass, t_max,
                               tf_guess, isp=isp, v_descent_max=v_descent_max,
                               term_win=term_win, term_vz=term_vz,
                               verbose=verbose)
    r3 = _solve_one_cgen(3, np.asarray(x0, float), mass, tf_guess, N3,
                         pcs_deg=pcs_deg, pcs_start_deg=pcs_start_deg,
                         vdesc=_vdesc3)
    if r3 is None:
        r3 = _solve_one(3, np.asarray(x0, float), mass, isp, t_max, throttle,
                        tf_guess, gs_deg, pcs_deg, v_max, N3, straight_fac,
                        target_alt, pcs_start_deg=pcs_start_deg,
                        verbose=verbose, v_descent_max=v_descent_max,
                        term_win=term_win, term_vz=term_vz)
        r3['engine'] = 'cvxpy'
    if r3['status'] != 'optimal':
        return _fail(f'p3 {r3["status"]}', tf_guess, stage='p3')
    xs = r3['x']
    assert xs is not None                     # optimal 时必有轨迹
    tf_m = tf_guess
    for i in range(xs.shape[1]):
        if (np.linalg.norm(xs[0:3, i]) + np.linalg.norm(xs[3:6, i])) < 0.1:
            tf_m = i / xs.shape[1] * tf_guess
            break
    # ================================================================
    # 【2026-09-29 第五步 5(b)：tf4 的余量必须取【最小可行 tf】与 tf_m 的较大者】
    # ----------------------------------------------------------------
    # 旧写法：tf4 = tf_m + 0.1*straight_fac（straight_fac=5 ⇒ +0.5 s）。
    # 它在【无包络】时够用，但加了 80 m/s 包络之后不够：
    # 包络把剖面拉长，P4 的最小可行 tf 增加得比 P3 的 tf_m 更快。
    # 实测（cgen，mass 157 t，入口 vz=-66.4 vh=19.9）：
    #     AIM_ALT  entry_h  P3 tf_m  P4 min_tf  tf4=tf_m+0.5   结果
    #        500     650      10.2     10.5        10.8        OK
    #        700     850      13.5     13.5        14.0        OK
    #        900    1050      15.0     17.0        15.5        INFEASIBLE (差 1.5 s)
    #       1100    1250      17.5     20.5        18.0        INFEASIBLE (差 2.5 s)
    #       1400    1550      21.2     24.5        21.8        INFEASIBLE (差 2.8 s)
    #       2000    2150      29.8     34.0        30.2        INFEASIBLE (差 3.8 s)
    #   ⇒ 抬高睒准点（AIM_ALT 500 -> 2000）在旧写法下【必然 infeasible】，
    #     而不是因为能量不够（P4 在 tf=34 s 时 optimal，落地 138 t）。
    #
    # 【修法】把余量从固定 +0.5 s 改为：
    #     tf4 = max(tf_m + tid, tf_min_feasible * 1.02)
    #   其中 tf_min_feasible 用【一次快速二分】在 P4 模型上估出。
    #
    # 【为什么不能只把 0.5 改大】回退到下一节说明：
    #   tf 越长，燃料越贵（实测 10.8 s->148.4 t，
    #   18 s->145.1 t，34 s->138.0 t），固定加大余量会白白烧油；
    #   而且不同 AIM_ALT 需要的余量不同（0.3~3.9 s）。
    # ================================================================
    _tf_lo = float(tf_m) + 0.1 * float(straight_fac)
    _tf_hi = max(_tf_lo, 60.0)
    tf4 = _tf_lo
    # 先试当前值；若不可行则向上扩到一个可行值（步长递增，
    # 避免真正的二分：每次试算都要调用一次求解器，而 cgen 单次仅 ~0.06 s）
    _tf_probe = tf4
    # ================================================================
    # 【第十二步 12(a)】探测时【同时做规划质量检查】
    #   旧逻辑：第一个可行解就 break（= 最小时间 = 边界 = 常常退化）
    #   新逻辑：可行【且】峰值倾角 <= _PLAN_TILT_MAX_DEG 才采信；
    #          退化解记下来当【兜底】，继续向上找。
    #   兜底的必要性：宁可飞一条退化规划，也不能因为闸门而【没有规划】——
    #          后者会让 nav_mode 停在 hold、载具直接掉下去。
    # ================================================================
    _tf_first_feas = None          # 第一个可行 tf（兜底）
    _res_first_feas = None         # 对应的解
    _tf_gate_ok = None             # 通过倾角闸门的 tf
    for _ in range(_TF4_PROBE_N):
        _v = vdesc_cap_vector(N4, float(x0[0]), float(x0[3]), mass, t_max,
                              _tf_probe, isp=isp, v_descent_max=v_descent_max,
                              term_win=term_win, term_vz=term_vz)
        _r = _solve_one_cgen(4, np.asarray(x0, float), mass, _tf_probe, N4,
                             pcs_deg=pcs_deg, pcs_start_deg=pcs_start_deg,
                             vdesc=_v)
        if _r is not None:
            if _tf_first_feas is None:
                _tf_first_feas = _tf_probe
                _res_first_feas = _r
            _pk = _plan_peak_tilt_deg(_r.get('u'))
            # 闸门关闭 / 倾角无法计算 -> 直接采信第一个可行解（旧行为）
            if (not _PLAN_TILT_GATE) or not (_pk == _pk):
                _tf_gate_ok = _tf_probe
                break
            if _pk <= _PLAN_TILT_MAX_DEG:
                _tf_gate_ok = _tf_probe
                break
            if verbose:
                print('[gate] tf=%.2f 峰值倾角 %.1f deg > %.1f，继续探测'
                      % (_tf_probe, _pk, _PLAN_TILT_MAX_DEG))
        _tf_probe = min(_tf_hi, _tf_probe + _TF4_PROBE_STEP)
    if _tf_gate_ok is not None:
        tf4 = _tf_gate_ok
    elif _tf_first_feas is not None:
        # 闸门在探测范围内没找到合格解 -> 退回第一个可行解（保底）
        tf4 = _tf_first_feas
        _pk_fb = _plan_peak_tilt_deg(_res_first_feas.get('u')) \
            if _res_first_feas is not None else float('nan')
        print('[gate] 探测 %d 次未找到峰值倾角 <= %.1f deg 的可行规划，'
              '退回第一个可行 tf=%.2f（其峰值倾角 %.1f deg）'
              % (_TF4_PROBE_N, _PLAN_TILT_MAX_DEG, tf4, _pk_fb))
        if verbose:
            print('[gate] 兜底 tf=%.2f 峰值倾角 %.1f deg' % (tf4, _pk_fb))

    # ---- P4：优先走 C 代码生成（快 ~40 倍），失败回退 cvxpy ----
    #   【为什么只对 P4 走这条】P4 输出最终轨迹，是耗时大头；
    #   而生成版把 tf 固化在模型里（TF_GEN=21.0），只有本次 tf4 接近时可用。
    #   P3 只用来估 tf_m，成本较低且 tf 可变，保持 cvxpy。
    # 【第二步】把下降率包络向量算好，两条路径共用同一份（保证一致）
    _vdesc4 = vdesc_cap_vector(N4, float(x0[0]), float(x0[3]), mass, t_max, tf4,
                               isp=isp, v_descent_max=v_descent_max,
                               term_win=term_win, term_vz=term_vz,
                               verbose=verbose)
    r4 = _solve_one_cgen(4, np.asarray(x0, float), mass, tf4, N4,
                         pcs_deg=pcs_deg, pcs_start_deg=pcs_start_deg,
                         vdesc=_vdesc4)
    if r4 is None:
        r4 = _solve_one(4, np.asarray(x0, float), mass, isp, t_max, throttle,
                        tf4, gs_deg, pcs_deg, v_max, N4, straight_fac, target_alt,
                        pcs_start_deg=pcs_start_deg, verbose=verbose,
                        v_descent_max=v_descent_max, term_win=term_win,
                        term_vz=term_vz)
        r4['engine'] = 'cvxpy'
    if r4['status'] != 'optimal':
        return _fail(f'p4 {r4["status"]}', tf4, stage='p4', tf_m=tf_m)

    def _tilt(i):
        uh = float(np.linalg.norm(r4['u'][1:3, i]))
        uv = float(abs(r4['u'][0, i]))
        return float(np.degrees(np.arctan2(uh, uv))) if uv > 1e-9 else float('nan')

    n4 = r4['u'].shape[1]
    r4.update({'tf_m': tf_m, 'stage': 'p4', 'solve_time': _t.time() - t0,
               'end_tilt_deg': _tilt(n4 - 2),
               'peak_tilt_deg': float(max(
                   (_tilt(i) for i in range(n4 - 2) if _tilt(i) == _tilt(i)),
                   default=float('nan')))})
    return r4


if __name__ == '__main__':
    print('P3/P4 两级求解器自测（参考仓库结构）')
    # 【自测用例改为实际工作点（2026-09-26）】
    #   原自测用 vz=-314 / vh=361.5，那是【横速 361】的极端状态：
    #   水平需求 a = 361.5²/(2×1349) = 48.4，可用仅 48.8 -> 比值 0.99，
    #   本身就在能量边界上，末段倾角压不到 0（实测 17.6°）。
    #   那不是我们的工作点，却会让人误以为"求解器坏了"。
    #   实飞真实交班横速是 221（yaml 日志 kOS handoff v=366.9，含垂直分量）。
    cases = [
        ('实飞交班点', 4994.0, -287.0, 221.0),
        ('低空备选  ', 1500.0, -95.0, 90.0),
    ]
    for name, alt0, vz0, vh0 in cases:
        dist0 = (alt0 - 36.8) * 0.27          # 锥内水平偏差
        r = solve_p3p4_state([alt0 - 36.8, -dist0, 0.0, vz0, vh0, 0.0],
                             mass=184200.0, t_max=8.99e6, N3=160, N4=80)
        print('\n[%s] alt=%.0f vz=%.0f vh=%.0f' % (name, alt0, vz0, vh0))
        print('  status = %s   stage = %s' % (r['status'], r.get('stage')))
        if r['status'] != 'optimal':
            continue
        u = r['u']
        N = u.shape[1]
        j = float(np.max(np.abs(np.diff(u[1, :]))))
        angs = []
        for i in range(1, N):
            a, b = u[:, i - 1], u[:, i]
            na, nb = float(np.linalg.norm(a)), float(np.linalg.norm(b))
            if na > 1e-6 and nb > 1e-6:
                angs.append(math.degrees(math.acos(
                    max(-1.0, min(1.0, float(np.dot(a, b)) / (na * nb))))))
        print('  tf_m=%.2f s  tf=%.2f s  耗时 %.2f s'
              % (r['tf_m'], r['tf'], r['solve_time']))
        print('  落地 %.1f t  末端倾角 %.2f°  推力越界 %.4f'
              % (r['mass_land'] / 1000, r['end_tilt_deg'],
                 r['thrust_over_ratio']))
        print('  轨迹平滑度: 最大|du_hor|=%.2f  最大相邻推力方向夹角=%.1f°'
              % (j, max(angs) if angs else float('nan')))
        print('  u_hor 前 8 个节点: %s' % np.round(u[1, :8], 1).tolist())
