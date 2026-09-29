# gfold_codegen.py — 用 cvxpygen 为 P3/P4 生成 C 求解器
#
# ============================================================================
# 【为什么要这个文件】
#   P3/P4 用 cvxpy 直接解，实测【实飞 3.6 s】（离线 2.17 s，KSP 抢占 CPU 后更慢）。
#   而 kOS 交班后火箭仍在以 ~300 m/s 下坠 —— 这 3.6 s 里必须靠 hold 段硬撑，
#   期间还会下降约 1100 m，导致解出来的轨迹起点与实际位置错位。
#
#   原仓库用 cvxpy_codegen 生成 C 代码解决这个问题（readme: "直接 cvxpy 求解
#   耗时可能达到 c 的 10 倍以上"）。但 cvxpy_codegen 需要 cvxpy==0.4.11，
#   在 Python 3.13 上装不上（use_2to3 已被 setuptools 移除）。
#
#   ⇒ 用现代后继 cvxpygen（Stanford Boyd 组）替代，支持 cvxpy 1.9 / Python 3.13。
#
# 【工作方式】
#   本文件把 gfold_p3p4.py 的 P3/P4 模型【参数化】地重建一遍
#   （x0 与 m_wet 作为 cvxpy Parameter），然后调用 cvxpygen 生成 C 代码。
#   生成的求解器把参数当输入，可以在飞控循环里反复快速调用。
#
# 【运行】
#   pwsh -File tools/gen_codegen.ps1      （会设置 PYTHONPATH 指向 pdaqp stub）
#   或直接：python dev/gfold/solver/gfold_codegen.py
#
# 【产物】
#   dev/gfold/solver/cpg_p3/    P3 求解器（N=40）
#   dev/gfold/solver/cpg_p4/    P4 求解器（N=40）
# ============================================================================
import math
import os
import shutil
import sys

import cvxpy as cp
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
# 本文件的上一级（dev/gfold/solver）就是 gfold_p3p4.py 所在处
sys.path.insert(0, _HERE)

from gfold_p3p4 import G0                      # noqa: E402 复用同一常量

# ---- 生成时固化的模型参数（与 gfold_p3p4 的默认保持一致）----
# 【N 必须与求解侧一致】生成的 C 模型把节点数【编译期固化】：
#     P4 输出最终轨迹 -> 用 gfold_p3p4 的 N4 = 80
#     P3 只估 tf_m      -> 用 gfold_p3p4 的 N3 = 160
#   【为什么 P3 不靠降 N 提速】实测 tf_m 对 N3 敏感：
#       N3=160 -> 14.75 s    N3=120 -> 14.33 s    N3=60 -> 15.33 s
#       N3=80  -> 18.00 s    N3=40  -> 38.00 s（退化！）
#   ⇒ 保持 N3=160，靠 codegen 提速。
N_P3 = 160
N_P4 = 80
N_GEN = N_P4        # 兼容旧引用
TF_GEN = 21.0       # 标称 tf（只用于算 z0i/z0l 的展开点；tf 本身已参数化）
# 【标称质量】z0i/z0l 必须用 numpy 常数（DCP 要求，见 build() 说明），
#   所以这里把质量固定在交班典型值。实飞日志实测交班质量 182.8~184.2 t。
#   落地质量约 140~168 t —— 偏差只影响"推力上下界线性化展开点"，
#   不影响约束形式。若实测发现偏差过大，可按质量区间生成多份。
MASS_GEN = 183000.0
ISP = 315.0
T_MAX = 8.99e6
THROTTLE = (0.1, 0.8)
GS_DEG = 30.0
# ============================================================================
# 【2026-09-29 第一步改动 1a：末端锥角 30 -> 5（仅 P4）】
# ----------------------------------------------------------------------------
# 【为什么必须改】codegen 路径把锥角【编译期固化】进模型，运行时传的
#   pcs_deg 被静默忽略（旧的 _solve_one_cgen 根本没有这个参数）。
#   实测（交班点 alt=686.8 dist=300 vz=-70 vh=60 m=156.9 t）：
#       codegen 路径 pcs_deg 传 30 -> 末端倾角(N-2) = 9.52 deg
#       codegen 路径 pcs_deg 传  5 -> 末端倾角(N-2) = 9.52 deg  ← 参数被丢
#       强制切 cvxpy，pcs_deg=10 -> 9.50 deg
#       强制切 cvxpy，pcs_deg= 5 -> 5.00 deg
#   ⇒ 锥角是【天花板】，自然最优解停在 9.52 deg；必须把锥压到 9.5 以下
#     才能逼出竖直姿态。本项目指标要求触地倾角 < 5 deg。
#
# 【代价】实测 tf 与落地质量【完全不变】（9.5 s / 148.2 t，两者逐位相同）。
#
# ============================================================================
# 【关键：锥角必须【按程序分开】—— P3 不能收紧】
# ----------------------------------------------------------------------------
#   用同一份 HEAD 的 .pyd 做 A/B（实测）：
#                  HEAD pyd(锥 30)   NEW pyd(锥 5 全收)
#       d=  0 vh= 0     9.00  ok          8.00  infeasible
#       d=  1 vh= 0     9.00  ok          8.00  infeasible
#       d=  5 vh= 0     9.00  ok          8.00  infeasible
#       d= 30 vh=15     9.00  ok          8.25  ok
#       d=150 vh=40     9.00  ok          P3 infeasible
#       d=300 vh=60     9.00  ok          P3 infeasible
#   ⇒ 把 P3 的末端锥也收到 5° 会让 P3 自身变得不可行（大横速入场），
#     或者把 tf_m 从 9.00 压到 8.00。而 tf_m 是 tf4 的输入
#     （tf4 = tf_m + 0.5），tf4=8.5 会让 P4 直接 infeasible。
#   【为什么这是安全的】P3 的轨迹【本来就被丢弃】，它只有一个用途：
#     估出着陆时刻 tf_m。“触地姿态矩直”是【即将执行的那条轨迹】
#     的性质，只能由 P4 保证。因此：
#         P3 保持 pcs_end = 30（与 HEAD 一致，保证 tf_m 不变）
#         P4 收到 pcs_end =  5（逼出竖直触地）
#   【当初为什么会想到全收】因为把 P3 当成了“一条也要飞的轨迹”。
#     它不是。这个误解与 gfold_p3p4.py 里 no_climb 只能加在
#     P4 上是同一个道理（那里是 tf_m 检测阈值被跨过）。
# ============================================================================
# P3：保持 HEAD 值（只用来估 tf_m，轨迹丢弃）
PCS_END_DEG_P3 = 30.0
# P4：真正要飞的那条 —— 收到 5° 以逼出竖直触地
PCS_END_DEG_P4 = 5.0
# 【兼容旧名】保留 PCS_END_DEG = P4 的值，使旧引用与日志不致失真。
PCS_END_DEG = PCS_END_DEG_P4
PCS_START_DEG = 85.0
V_MAX = 1200.0
STRAIGHT_FAC = 5.0
TARGET_ALT = 36.8


def build(program, N=None, tf=TF_GEN):
    """构建参数化的 P3 或 P4。

    N 默认按 program 取（P3->160, P4->80），与 gfold_p3p4 的默认一致。
    """
    if N is None:
        N = N_P3 if program == 3 else N_P4
    """构建参数化的 P3 或 P4（x0 与 m_wet 为 cvxpy Parameter）。

    约束【逐条对齐】gfold_p3p4._solve_one()，含本项目两处修正：
      · P4 目标函数用【常权重】（不用 i/N）—— 见 gfold_p3p4 的说明
      · node 0 有【真实推力上界】（原仓库漏了）
    """
    alpha = 1.0 / G0 / ISP
    r1 = T_MAX * THROTTLE[0]
    r2 = T_MAX * THROTTLE[1]
    y_gs_cot = 1.0 / math.tan(math.radians(GS_DEG))
    # 【归一化时间】t = tf * frac。这样 tf 变化只缩放时间轴，
    #   不需要重新生成模型（见下面的 tf Parameter）。
    frac = np.linspace(0.0, (N - 1) / N, N)

    x0 = cp.Parameter(6, name='x0')
    # 【DPP 关键 ①】参考仓库用【预先算好的 log(m)】作为参数：
    #   GFOLD_codegen.py line 50: sparse_params 里含 m_wet_log
    #   line 71: con += [z[0,0] == m_wet_log]
    # 若写成 `z[0,0] == cp.log(m_wet)`（Parameter 的 log）会破坏 DPP
    #   —— 实测这是本模型唯一的 DPP 违规点（见 tools/_dpp.py）。
    m_wet_log = cp.Parameter(name='m_wet_log')
    # 【DPP 关键 ②】node 0 的推力上下界（本项目修正：原仓库漏了 node 0）。
    #   参数不能出现在分母（`r2/m_wet` 破坏 DPP），故预乘成两个标量 Parameter。
    s_hi0 = cp.Parameter(nonneg=True, name='s_hi0')
    s_lo0 = cp.Parameter(nonneg=True, name='s_lo0')
    # ========================================================================
    # 【关键改进：tf 也是 Parameter ⇒ 一份生成模型覆盖任意 tf】
    # ========================================================================
    #   若不参数化 tf，dt=tf/N 会被固化进所有动力学约束 —— 而 tf 是
    #   P3 每次现估的（实测 15~22 s 视工况而定），于是必须按 tf 网格
    #   生成多份模型，且 tf 落点仍会引入偏差。
    #   【实测】把 tf 声明成 Parameter 后，DCP 与 DPP【都保持通过】
    #   （验证脚本 tools/_tfparam.py）—— 因为 dt 只以
    #       dt * 0.5 * (变量 + 变量)
    #   的形式出现，是"参数 × 变量"的线性组合，正是 DPP 允许的。
    #   ⇒ 生成一次即可，运行时把真实 tf 传进来。
    tf_p = cp.Parameter(nonneg=True, name='tf')
    dt = tf_p / N                       # 参数表达式，替代原来的常数 dt
    alpha_dt = alpha * dt               # 同样是参数表达式
    # ========================================================================
    # 【DCP 关键：z0i / z0l 必须是 numpy 常数，不能用 Parameter】
    # ========================================================================
    #   参考仓库 GFOLD_codegen.py 把 z0_term_inv / z0_term_log 声明成 Parameter
    #   （line 46-47），在 cvxpy 0.4.11 下能过 DCP。但在 cvxpy 1.9.3 下
    #   【一律 non-DCP】。我实测了 5 种等价写法（见 tools/_dcp2.py）：
    #       A numpy 常数            -> DCP      ← 唯一可行
    #       B 全 Parameter          -> NOT DCP
    #       C mu1 预乘成 Parameter  -> NOT DCP
    #       D 改用 cp.square()      -> NOT DCP
    #       E 二次项系数全Parameter -> NOT DCP
    #   原因：约束 s >= mu*(1 - d + d^2*0.5)（d=z-z0）右端是【凸函数】，
    #   含【数值常数】时 cvxpy 能把它归入可接受形式；一旦系数变成
    #   Parameter，DCP 判定就失败。
    #
    #   ⇒ 结论：mass 不能在生成期参数化到这条约束里。
    #     本文件把 mass 固定在 MASS_GEN（标称交班质量），
    #     z0i/z0l 用该质量在 numpy 里算成常数写进模型。
    #     实际质量与标称值有偏差时，只影响"推力上下界的线性化展开点"，
    #     不影响物理约束的形式（下一节有偏差评估）。
    # ========================================================================
    # 【注意】z0_term 依赖 tf（t=tf*frac），但 z0i/z0l 必须是 numpy 常数
    #   （DCP 要求，见下面的长注释）。故这里用 TF_GEN 作为【标称 tf】算
    #   z0i/z0l —— 它们只是"推力上下界的线性化展开点"，标称值与实际 tf
    #   的偏差只让展开点略微偏离，不改变约束形式。
    z0_term = MASS_GEN - alpha * r2 * (TF_GEN * frac)
    z0_term = np.maximum(z0_term, 0.05 * MASS_GEN)
    z0i = (1.0 / z0_term).reshape(1, N)          # numpy 常数
    z0l = np.log(z0_term).reshape(1, N)          # numpy 常数

    x = cp.Variable((6, N), name='x')
    u = cp.Variable((3, N), name='u')
    z = cp.Variable((1, N), name='z')
    s = cp.Variable((1, N), name='s')

    con = [
        x[0:3, 0] == x0[0:3],
        x[3:6, 0] == x0[3:6],
        x[3:6, N - 1] == np.zeros(3),
        s[0, N - 1] == 0,
        z[0, 0] == m_wet_log,
        # 【2026-09-27 修正：补回原版的【起点推力竖直】约束】
        #   原版 GFOLD_direct_exec.py:59-60 两端都强制竖直：
        #       con += [u[:,0]   == s[0,0]  *np.array([1,0,0])]
        #       con += [u[:,N-1] == s[0,N-1]*np.array([1,0,0])]
        #   本文件此前只有末端那条，起点缺失 ⇒ 求解器把节点 0 解成 85° 倾角，
        #   而节点 0 是交接后第一帧就执行、且载具当时只有 24.5° 姿态 ⇒
        #   要求 60.5° 瞬时转姿，物理上不可能 ⇒ 姿态跟不上 ⇒ 不跟踪。
        #   详见 dev/reports/SOLVER_DIVERGENCE_FROM_ORIGINAL.md
        #   【注意】这里必须与 gfold_p3p4.py 的同名约束保持一致，
        #   否则 cvxpy 路径与 codegen 路径会解出不同形状的轨迹。
        u[:, 0] == s[0, 0] * np.array([1, 0, 0]),
        u[:, N - 1] == s[0, N - 1] * np.array([1, 0, 0]),
    ]
    if program == 3:
        con += [x[0, N - 1] == 0]
    else:
        con += [x[0:3, N - 1] == np.zeros(3)]

    g = np.array([-G0, 0.0, 0.0])
    for n in range(N - 1):
        con += [x[3:6, n + 1] == x[3:6, n]
                + (dt * 0.5) * ((u[:, n] + g) + (u[:, n + 1] + g))]
        con += [x[0:3, n + 1] == x[0:3, n]
                + (dt * 0.5) * (x[3:6, n + 1] + x[3:6, n])]
        con += [cp.norm(x[1:3, n]) - y_gs_cot * x[0, n] <= 0]
        con += [cp.norm(x[3:6, n]) <= V_MAX]
        con += [z[0, n + 1] == z[0, n]
                - (alpha_dt * 0.5) * (s[0, n] + s[0, n + 1])]
        con += [cp.norm(u[:, n]) <= s[0, n]]
        frac = n / max(1, N - 2)
        # 【按程序取末端锥角】P3 用 30（保 tf_m），P4 用 5（保触地姿态）。
        #   理由见文件顶部 PCS_END_DEG_P3/P4 的 A/B 实测表。
        _pcs_end = PCS_END_DEG_P3 if program == 3 else PCS_END_DEG_P4
        ang = np.radians(PCS_START_DEG + (_pcs_end - PCS_START_DEG) * frac)
        con += [u[0, n] >= np.cos(ang) * s[0, n]]
        con += [x[0, n] >= 0]
        # ================================================================
        # 【2026-09-29 第一步改动 4：竖直铁律 —— 全程不许爬升】
        # ----------------------------------------------------------------
        #   与 x[0,n] >= 0（不穿地）配对。原实现只有"不穿地"，求解器
        #   仍可给出"俯冲-回弹"的畸形解（diag10 实测节点 28 高度 0、
        #   垂速 -1.3，随后爬到 157 m 再落回）。真实剖面单调下降，
        #   这条不损失最优性，只封掉畸形解。
        #
        #   【只对 P4 施加 —— 必须与 gfold_p3p4.py 的同名约束一致】
        #     实测（gate 686.8/d300/-70/60/156.9t，N3=160，tf_guess=40）：
        #       加在 P3 上会让 tf_m 的检测（位置+速度 < 0.1，绝对量）
        #       从节点 36 推迟到节点 158 —— 因为 no_climb 强制 vz<=0 后，
        #       地面驻留高度由 0.0 抬到 0.2 m，越过 0.1 阈值。
        #       结果 tf_m 由 9.0 s 虚高到 39.5 s（4.4 倍），P4 随即被规划
        #       成一条 40 s 的松散轨迹，实际飞行完全跟不上。
        #       而 P3 的轨迹【本来就被丢弃】，只有 tf_m 有用。
        #     ⇒ 本次取最小改法：铁律只加在真正要飞的 P4 上。
        #     ⇒ 生成器的 P3/P4 是【两份独立模型】，此处的 program == 4
        #       与 gfold_p3p4.py 的写法语义一致。
        # ================================================================
        if program == 4:
            con += [x[3, n] <= 0]
        # ================================================================
        # 推力上下限（对数凸化）——【必须保留 if n > 0 的守卫】
        #   【2026-09-29 事故记录】本次改动曾在整理注释时把 `if n > 0:`
        #     这一行【误删】，使推力上下限被并进了上面 `if program == 4:`
        #     的分支 —— 于是 P3 变成【完全没有推力上下限】：
        #       P3 codegen tf_m 由 9.00 掉到 8.00（不可行区间的伪解）
        #       d=150/d=300 的大横速工况 tf_m 掉到 8.25
        #     tf_m 是 tf4 的输入（tf4 = tf_m + 0.5），tf4=8.5 直接让 P4
        #     infeasible。修复后 tf_m 回到 9.00。
        #   ⇒ 教训：改注释块时【必须逐个分支核对缩进归属】，
        #     不能只看它读起来通顺。
        # ================================================================
        if n > 0:
            zz = z0l[0, n]
            con += [s[0, n] >= r1 * z0i[0, n]
                    * (1 - (z[0, n] - zz) + (z[0, n] - zz) ** 2 * 0.5)]
            con += [s[0, n] <= r2 * z0i[0, n] * (1 - (z[0, n] - zz))]
        else:
            # node 0 的真实推力上下界（本项目修正；原仓库 `if n>0` 导致 node 0
            # 完全没有上界，实测解出 |u[:,0]|=812 而真实上限只有 49）。
            # 用预乘好的 Parameter，避免"参数在分母"破坏 DPP。
            con += [s[0, 0] <= s_hi0]
            con += [s[0, 0] >= s_lo0]

    if program == 3:
        expr = 0
        for i in range(N):
            expr += cp.norm(x[0:3, i]) * (i / N)     # P3 保持原仓库权重
    else:
        expr = 0
        for i in range(N):
            expr += cp.norm(x[4:6, i])               # P4 常权重（本项目修正）
        expr = expr * STRAIGHT_FAC - z[0, N - 1] * N

    # 【pyright】con 里既有 Equality 又有 Inequality，cvxpy 的
    #   Problem.constraints 注解是 list[Constraint]；pyright 推不出公共父类，
    #   报 reportArgumentType。这里显式转型（运行期无影响）。
    prob = cp.Problem(cp.Minimize(expr),
                      list(con))                       # type: ignore[arg-type]
    return prob, x0, m_wet_log, s_hi0, s_lo0, tf_p


def set_params(prob_params, x0_val, mass_val, tf_val=TF_GEN):
    """设好运行时参数：x0 / log(m) / node0 推力上下界 / tf。

    tf 现在也是参数，所以【同一份生成模型可服务任意 tf】。
    """
    x0p, mwl, s_hi, s_lo, tfp = prob_params
    x0p.value = np.asarray(x0_val, float)
    mwl.value = float(np.log(mass_val))            # 预先算 log，保证 DPP
    s_hi.value = T_MAX * THROTTLE[1] / float(mass_val)
    s_lo.value = T_MAX * THROTTLE[0] / float(mass_val)
    tfp.value = float(tf_val)


def fix_generated_import(code_dir, pkg_name):
    """修正 cvxpygen 生成文件里的【坏 import 行】。

    【cvxpygen 0.7.0 的 bug】生成的 cpg_solver.py 顶部会写：
        from D:.path.to.dir import cpg_module      # 非法语法
    它把绝对路径的 '\\' 换成 '.'，却保留了盘符冒号。
    实测：与空格无关，与盘符有关（D:\\x -> D:.x）。

    【修法】把 `from <任意> import cpg_module` 改成直接 `import cpg_module`
    （同目录下），因为飞控里我们会把该目录加到 sys.path。
    """
    f = os.path.join(code_dir, 'cpg_solver.py')
    if not os.path.exists(f):
        return False
    src = open(f, encoding='utf-8').read()
    out = []
    changed = False
    for line in src.split('\n'):
        st = line.strip()
        if st.startswith('from ') and st.endswith(' import cpg_module'):
            indent = line[:len(line) - len(line.lstrip())]
            out.append(indent + 'import cpg_module')
            changed = True
        else:
            out.append(line)
    if changed:
        open(f, 'w', encoding='utf-8').write('\n'.join(out))
    return changed


def rename_module(code_dir, program):
    """把生成扩展的模块名从 cpg_module 改成 cpg_module_p3 / cpg_module_p4。

    【为什么必须改（实测踩坑）】
      两个 .pyd 默认同名 `cpg_module`，且初始化符号都是 `PyInit_cpg_module`。
      CPython 对扩展模块有【按名字的内部缓存】—— 先加载 P3 之后再加载 P4，
      会直接拿到 P3 的扩展（实测报
          ValueError: cannot reshape array of size 960 into shape (6,80)
      960 = 6×160，正是 P3 的 x 尺寸）。
      importlib 层（ExtensionFileLoader、pop sys.modules）【都绕不过】这个缓存。

    【修法】把模块名统一改成 cpg_module_p<N>，涉及三处：
      · setup.py：Extension('cpg_module') / setup(name='cpg_module')
      · cpp/src/cpg_module.cpp：PYBIND11_MODULE(cpg_module, m) ← 定义 PyInit 符号
      · cpg_solver.py：import cpg_module
    漏掉 cpp 那处会链接失败：
      LINK : error LNK2001: 无法解析的外部符号 PyInit_cpg_module_p3
    """
    new = 'cpg_module_p%d' % program
    changed = []

    sp = os.path.join(code_dir, 'setup.py')
    if os.path.exists(sp):
        s = open(sp, encoding='utf-8').read()
        s2 = s.replace("Extension('cpg_module'", "Extension('%s'" % new)
        s2 = s2.replace("setup(name='cpg_module'", "setup(name='%s'" % new)
        if s2 != s:
            open(sp, 'w', encoding='utf-8').write(s2)
            changed.append('setup.py')

    # 【关键】PYBIND11_MODULE 决定 pybind11 生成的 PyInit 符号名
    cpp = os.path.join(code_dir, 'cpp', 'src', 'cpg_module.cpp')
    if os.path.exists(cpp):
        s = open(cpp, encoding='utf-8').read()
        s2 = s.replace('PYBIND11_MODULE(cpg_module,',
                       'PYBIND11_MODULE(%s,' % new)
        # ================================================================
        # 【pybind11 的类注册是【全局】的（按 C++ 类型 typeid 为键）】
        #   两个扩展里的 C++ 结构体都叫 CPG_Params_cpp_t 等，typeid 相同，
        #   于是加载第二个时抛：
        #       ImportError: generic_type: type "cpg_params_p4"
        #                    is already registered!
        #   注意：报错里虽然是 _p4，但根因是【C++ 类型重名】，
        #   所以只改 Python 类名【没用】。
        #
        # 【官方正解】pybind11 文档 docs/advanced/classes.rst 的 module_local：
        #   "py::module_local() ... the class will only be registered in the
        #    module where it is defined"（同名类型可在各模块中各自注册）
        #   做法：在每个 py::class_<T>(m, "name") 后加 .def(py::module_local())
        #   或直接在 class_ 构造里传 py::module_local()。
        #   这里用后者：py::class_<T>(m, "name", py::module_local())
        # ================================================================
        import re as _re
        s2 = _re.sub(r'py::class_<([A-Za-z_0-9]+)>\(m,\s*"([A-Za-z_0-9]+)"\)',
                     r'py::class_<\1>(m, "\2", py::module_local())', s2)
        # 【pybind11 的类名是全局注册的】两个扩展若都用 "cpg_params"
        #   等名字，第二个加载时会抛
        #       ImportError: generic_type: type "cpg_params" is already registered!
        #   ⇒ 把导出的 Python 类名也加上程序后缀。
        for cls in ('cpg_params', 'cpg_updated', 'cpg_prim',
                    'cpg_dual', 'cpg_info', 'cpg_result'):
            s2 = s2.replace('(m, "%s")' % cls,
                            '(m, "%s_p%d")' % (cls, program))
        if s2 != s:
            open(cpp, 'w', encoding='utf-8').write(s2)
            changed.append('cpg_module.cpp')

    cs = os.path.join(code_dir, 'cpg_solver.py')
    if os.path.exists(cs):
        s = open(cs, encoding='utf-8').read()
        s2 = s.replace('import cpg_module', 'import %s as cpg_module' % new)
        if s2 != s:
            open(cs, 'w', encoding='utf-8').write(s2)
            changed.append('cpg_solver.py')

    return changed


def generate_one(program, out_dir):
    # 【API 位置（0.7.0 核实）】generate_code 在 cvxpygen.cpg 子模块里，
    #   不在 cvxpygen 顶层（顶层 __init__ 是空的）。
    from cvxpygen import cpg
    print('=' * 70)
    print('生成 P%d  ->  %s' % (program, out_dir))
    print('=' * 70)
    prob, x0, m_wet_log, s_hi0, s_lo0, tf_p = build(program)
    print('  变量 %d / 参数 %d / 约束 %d'
          % (sum(v.size for v in prob.variables()),
             sum(p.size for p in prob.parameters()),
             len(prob.constraints)))

    # 先用 cvxpy 自查一次（确认模型可行），再生成
    set_params((x0, m_wet_log, s_hi0, s_lo0, tf_p),
               np.array([3000 - TARGET_ALT, -800.0, 0.0, -180.0, 120.0, 0.0]),
               MASS_GEN)
    prob.solve(solver=cp.CLARABEL)
    print('  cvxpy 自查: %s' % prob.status)
    if prob.status != 'optimal':
        print('  !! 模型不可行，先查模型再生成')
        return False

    if os.path.exists(out_dir):
        shutil.rmtree(out_dir)
    # ========================================================================
    # 【cvxpygen 0.7.0 的 bug 及绕法】
    # ========================================================================
    # cpg.generate_code() 末尾（cpg.py:92-96）写的是：
    #     if wrapper:
    #         compile_python_module(code_dir)
    #         module = importlib.import_module(f'{code_dir}.cpg_solver')   # ← BUG
    #         cpg_solve = getattr(module, 'cpg_solve')
    #         problem.register_solve('CPG', cpg_solve)
    # 它把【绝对路径】当模块名传给 import_module，任何平台都必然
    # ModuleNotFoundError。而且 import 之前调用了 chdir(code_dir)，
    # 所以正确的模块名其实只是 'cpg_solver'。
    #
    # 该分支的用途是 `problem.register_solve('CPG', cpg_solve)` —— 一个
    # 便利注册，让用户写 prob.solve(method='CPG')。我们不用它（自己封装调用）。
    #
    # ⇒ 绕法（方案 D）：wrapper=False 绕过整段（含编译），
    #    然后【自己调用公开函数】cpg.compile_python_module(code_dir) 完成编译。
    #    这样既避开 bug，又拿到编译好的 .pyd。
    # ========================================================================
    cpg.generate_code(prob, code_dir=out_dir, solver=cp.ECOS, wrapper=False)
    ok = fix_generated_import(out_dir, os.path.basename(out_dir))
    # 【必须在编译前改名】否则两个扩展都叫 cpg_module，运行时会互相顶掉
    ren = rename_module(out_dir, program)
    print('  坏 import 修正: %s  模块改名: %s'
          % ('已修正' if ok else '未发现', ren or '无'))
    cpg.compile_python_module(out_dir)
    pyds = [f for f in os.listdir(out_dir) if f.endswith(('.pyd', '.so'))]
    print('  产物: %s' % pyds)
    return True


def main():
    print('gfold_codegen — 为 P3/P4 生成 C 求解器（cvxpygen）')
    print('N_P3=%d  N_P4=%d  TF_GEN=%.1f (tf 已参数化)'
          % (N_P3, N_P4, TF_GEN))
    base = _HERE
    ok3 = generate_one(3, os.path.join(base, 'cpg_p3'))
    ok4 = generate_one(4, os.path.join(base, 'cpg_p4'))
    print('\n结果: P3=%s  P4=%s' % (ok3, ok4))
    return 0 if (ok3 and ok4) else 1


if __name__ == '__main__':
    sys.exit(main())
