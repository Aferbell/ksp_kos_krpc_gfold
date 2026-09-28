# 火箭定点回收凸优化代码库精读与 kOS 可移植性报告

精读对象：
- **LCVX**（3DoF 无损凸化，MATLAB/CVX）：`...\LosslessConvexification_MatlabCVX_master\Lossless-convexification-master\main.m`（全库仅此一个建模文件）
- **SCvx-6DoF**（C++/Eigen/CppAD/ECOS，free final time）：`...\RocketLanding_SCvx_6DoF_Mars_free_final_time_Cpp\RocketLanding-main\` 下的 `src\solver.cpp`、`src\vehicle.cpp`、`src\discretization.cpp`、`include\types.hpp`、`demo\config.yaml`

---

## A. 3DoF 无损凸化问题（LCVX, main.m）

### A.1 状态 / 控制 / 参数

- **状态** y = [r(3); v(3); z]，其中 **z = ln(m)**（main.m:25 `y0=[position_0 velocity_0 log(m_wet)]'`），7 维。
- **控制** η 每节点 4 维：u = [u_c(3); σ]，u_c = T/m（比推力/加速度量纲），σ 为推力松弛变量（‖u_c‖ ≤ σ）。
- 参数（main.m:9-17）：g=[-3.7114,0,0]（x 轴向上），m_dry=1505, m_wet=1905, Isp=225, 推力界 T1=930/T2=2480（单台），n=6 台，安装角 φ=27°，α=5.09e-4（=1/(Isp·g0) 的燃料消耗率系数）。
- 固定末时刻 t_f=68 s，N=55 段（main.m:19-21）。

### A.2 动力学与离散化

连续模型（main.m:30-32）：
- A_c：ṙ=v；B_c：v̇ 由 u_c 驱动，ż = -α·σ。
- `c2d` 精确离散化得到 A(7×7)、B(7×4)。
- main.m:35-57 预计算状态转移幂 φ=A^k、λ=B+AB+…、ψ（块下三角卷积阵）、υ（选择阵）。x_k = φ_k·y0 + λ_k·[g;0] + ψ_k·η，即全部状态写成决策变量 η 的**仿射函数**（main.m:66-69 的 ε_k 是零输入响应）。这是把整条轨迹"拍平"成单次 QP/SOCP 的经典做法——矩阵巨大，无法搬 kOS。

### A.3 全部约束的精确形式

1. **终端约束**（main.m:96-100）：末位置三分量=0、末速度三分量=0（对 z_f 自由）。
2. **推力锥（凸化核心）**（main.m:103-105）：`‖E_u·u_k‖₂ ≤ σ_k`，即松弛后的二阶锥。
3. **推力幅值上下界**（main.m:60-64, 108-113）：
   - 定义 z₀(k)=ln(m_wet − α·n·T2·cosφ·Δt·k)（最大消耗下的质量下界轨迹，main.m:60）；
   - μ₁(k)=n·T1·cosφ·e^(−z₀(k))，μ₂(k)=n·T2·cosφ·e^(−z₀(k))（main.m:62-63）；
   - 下界（**二阶泰勒展开的凹函数下界**）：
     `μ₁(k)·[1 − (z_k − z₀(k)) + ½(z_k − z₀(k))²] ≤ σ_k`（main.m:108, 111）；
   - 上界（线性化）：
     `σ_k ≤ μ₂(k)·[1 − (z_k − z₀(k))]`（main.m:109, 112）。
   即把非凸约束 ρ₁e^(−z) ≤ ‖u‖ ≤ ρ₂e^(−z) 中的 e^(−z) 在 z₀ 处展开：下界用二阶泰勒（凸），上界用一阶泰勒（仿射）。
4. **质量界**（main.m:116-119）：ln(m_wet − α·n·T2·cosφ·Δt·k) ≤ z_k ≤ ln(m_wet − α·n·T1·cosφ·Δt·k)，即质量被夹在"最大/最小消耗"两条解析界之间，并隐含 z_k ≥ ln(m_dry)。
5. **目标**（main.m:75-79, 92）：minimize Δt·Σσ_k，等价于最大化末质量（最小燃料）。
6. **滑翔角锥**在代码里被**注释掉了**（main.m:121-122），但保留的形式是标准形式：`‖[r_y; r_z]‖₂ − tan(γ_gs)... 写作 norm(...) + c·x ≤ 0`，即水平位移范数 ≤ (1/tanγ)·高度，是线性锥（SOC），本来就可直接进 CVX。

### A.4 为什么"无损"

- 松弛把非凸约束 ‖u_c‖ ∈ [ρ₁e^(−z), ρ₂e^(−z)] 换成凸锥 ‖u_c‖ ≤ σ 加 σ 的上下界。理论（Açıkmeşe–Blackmore 无损凸化定理，目录下附有原文 PDF）保证：在终端位置速度固定、目标为燃料最优（min Σσ）且 t_f 不小于最优值时，最优解必满足 ‖u_c‖=σ（锥面取等），即松弛**严格无损**——最优推力始终 bang-bang 式位于边界上。e^(−z) 的展开之所以可行，是因为下界用了 ≥ 真实值的凹上界函数、上界用了 ≤ 真实值的线性下界（对 z≥z₀ 时泰勒余项符号固定），二者都不缩小真实可行域。

---

## B. 6DoF SCvx（RocketLanding-main）

### B.1 状态/控制

- 状态 14 维（types.hpp:12, 18-22）：x = [m, r_i(3), v_i(3), q_b_i(4), ω_b(3)]。控制 u(3) = 机体系推力向量（types.hpp:13）。
- 注意：这里 m 是**线性质量**，不是 ln(m)。

### B.2 姿态动力学（vehicle.cpp:70-100）

- 质量：ṁ = −α·‖u‖（‖·‖ 加 1e-10 正则化避免雅可比奇异，vehicle.cpp:90）。
- ṙ = v；v̇ = (1/m)·DCM(q)ᵀ·u + g_i（vehicle.cpp:95；q_b_i 是 B←I 姿态，转置即 B→I）。
- 四元数运动学：q̇ = ½·Ω(ω_b)·q（skew4，vehicle.cpp:97）。
- 角速度动力学：ω̇ = J⁻¹·(r_t_b × u − ω×(Jω))（vehicle.cpp:99）——推力作用在离质心 r_t_b 的万向点上产生控制力矩。
- A、B 雅可比由 CppAD 自动微分并代码生成（vehicle.cpp:33-67）——kOS 上不可复现。

### B.3 约束的精确形式（solver.cpp `set_up_socp`）

- **离散动力学+虚拟控制**（solver.cpp:268-275）：x_{k+1} = Āx_k + B̄u_k + C̄u_{k+1} + Σ̄σ + z̄_k + ν_k。
- **滑翔角锥**（solver.cpp:299）：`‖[r_y;r_z]‖ ≤ (1/tan γ_gs)·r_x`（SOC，凸）。
- **倾角约束（四元数二次型）**（solver.cpp:301）：`‖[q_2; q_3]‖ ≤ sqrt((1−cos θ_max)/2)`。推导：竖直轴与机体 x 轴夹角 θ 满足 cosθ = 1 − 2(q₂²+q₃²)（q 为单位四元数时），故 cosθ ≥ cosθ_max ⟺ q₂²+q₃² ≤ (1−cosθ_max)/2。形式上是**凸二次约束**（恒小于凸函数——严格说该写法利用单位范数约束把它变成凸的）。终端倾角同理用 θ_f_max（solver.cpp:289）。
- **四元数单位范数**：上界 ‖q‖≤1 是 SOC（solver.cpp:307）；下界 ‖q‖≥1 非凸，线性化为围绕参考 q_ref 的半空间 `q_refᵀ·q ≥ 1 − ε_q`（solver.cpp:308-312，ε_q=0.1，config.yaml:29）。
- **角速度界**：‖ω_b‖ ≤ ω_max（solver.cpp:303，SOC）。
- **万向节角约束**（solver.cpp:322）：`‖u‖ ≤ (1/cos δ_max)·u_x`，即推力横向分量 ≤ tanδ_max·u_x 的等价 SOC 写法。
- **推力界**：下界非凸，线性化为沿上一迭代推力方向的投影 `û_ref·u ≥ T_min`（solver.cpp:315-318；û_ref 在 update_dynamic_params 中更新，solver.cpp:354-357）；上界 ‖u‖≤T_max（solver.cpp:320，SOC）。
- 终端：位置/速度固定、末角速度=0、末推力纯轴向（solver.cpp:285-293）。

### B.4 free-final-time：时间膨胀因子 σ

- 归一化时间 τ∈[0,1]，dt=1/(K−1)（solver.cpp:215）；真实时间 t=σ·τ，动力学整体乘 σ（discretization.cpp:44-46, 53：`A*=σ; B*=σ; dx/dτ=σ·f`）。
- σ 本身是**优化变量**（solver.cpp:258），进入线性化动力学项 Σ̄·σ（discretization.cpp:64-65，Σ̄=Φ⁻¹f）。
- 代价含 w_sigma·σ=100·σ（最小化飞行时间，config.yaml:25），并有信赖域 |σ−σ_ref|≤Δσ（solver.cpp:330-331）。

### B.5 信赖域与虚拟控制：权重与更新

- 信赖域：‖(x_k,u_k)−(x̄_k,ū_k)‖ ≤ Δ̄_k（逐节点变量，solver.cpp:325-328）；|σ−σ̄|≤Δσ（solver.cpp:330-331）。Δ̄、Δσ 是**优化变量**并进入代价（w_delta_bar·‖Δ̄‖，w_delta_sigma·Δσ，solver.cpp:344-346）——软信赖域。
- 权重（config.yaml:22-25）：w_nu=1e5（虚拟控制，L1 通过 Nu_bound 盒约束实现，solver.cpp:335-336, 342），w_delta_bar=0.1，w_delta_sigma=0.1，w_sigma=100。
- **更新规则**：w_delta_bar 前 5 次迭代乘 1.0，之后乘 100（solver.cpp:358 `solver_iter < 5 ? 1.0 : 100.0`）——先松后紧，让轨迹先大步骤收敛再精细定型。信赖域半径不做经典的"按实际/预测改善比扩缩"，靠代价惩罚自然收缩。

### B.6 SCP 收敛判据与初值猜测（兼答 C）

- **初值猜测是解析直线插值**（solver.cpp:174-206）：位置/速度/质量/角速度在初值与终值间线性插值（α₁x₀+α₂x_f），姿态用 **slerp** 从初始四元数插到竖直 (1,0,0,0)，控制初值取 **重力抵消** u_k = −m·DCM·g_i（solver.cpp:199）。σ 初值 = t_f_guess=5。
- **收敛判据**（solver.cpp:425）：`‖Δ̄‖ < delta_bar_tol(1e-3)` 且 `Σ|ν| < nu_tol(1e-5)`，即轨迹不再变动且虚拟控制（动力学缺陷）归零；最大 15 次迭代（config.yaml:26-28）。每轮解完对四元数重新归一化（solver.cpp:407-410）。
- 离散化用 RKF7(8) 自适应积分状态转移张量 V=[x, Φ_A, B̃, C̃, Σ̃, z̃]（discretization.cpp:21-69）——这是计算量大头，kOS 完全不可行。

---

## C. 初值猜测与收敛判据（汇总）

见 B.6：LCVX 无初值猜测（单次凸问题不需要）；SCvx 用"线性插值 + slerp + 重力抵消推力 + t_f 猜测"，收敛判据为信赖域量与虚拟控制 L1 双阈值。

---

## D. 可搬进 kOS 的【思想】（无求解器、10–20 Hz、纯标量/向量）

| # | 思想 | kOS 对应 |
|---|------|---------|
| 1 | 倾角约束的**点积/二次型形式**（B.3） | 不用四元数：倾角约束 ⟺ `ship:up:vector * facing:vector ≥ cos θ_max`（两单位向量点积），每帧可算，0 成本 |
| 2 | 滑翔角锥（A.3/B.3） | 每帧检查 `水平距离 ≤ 高度/tan(γ_gs)`；若违反则把目标瞄准点沿水平方向外推，或提高推力俯仰分量 |
| 3 | 万向节约束的锥形式（B.3） | 指令推力向量投影限幅：`若 |T_横向| > tanδ_max·T_轴向 则压缩横向分量`——即每帧 steering 限幅 |
| 4 | 信赖域（B.5） | 对应**每帧指令限幅**：|Δ油门| ≤ Δmax、|Δ瞄准方向角| ≤ Δψmax/帧；远处松、近地紧（模仿 w_delta_bar 前松后紧，solver.cpp:358） |
| 5 | 虚拟控制 ν（B.5） | 对应**降级模式**：当解析制导无可行解（如所需加速度 > T_max/m）时，不崩溃，改为"可行域边界投影 + 记录缺陷量"，缺陷持续超阈值则触发中止/改开环自杀燃烧 |
| 6 | free-final-time 的 σ（B.4） | 每帧在线重估 T_go 并当"软变量"平滑更新：`T_go ← 0.9·T_go + 0.1·T_go_new`，模仿 σ 信赖域 |σ−σ̄|≤Δσ |
| 7 | 推力下界的方向投影线性化（solver.cpp:317） | KSP 引擎不能低节流到底：保持推力沿上一帧方向的分量 ≥ T_min，等价于"最小油门约束下优先转方向后调油门" |
| 8 | 重力抵消初值（solver.cpp:199） | kOS 制导律的偏置项：推力指令 = m·(g 抵消 + 制导加速度)，即 a_cmd = a_guide − g |

### kOS 伪代码（E-G 制导 + SCvx 思想外壳）

```
// 每帧（~10 Hz）
set g to body:mu / (body:radius + altitude)^2 * up:vector.  // 重力向量
// 1. T_go 估计（思想6：free-final-time 的帧级类比，带"信赖域"平滑）
set tgo_raw to estimate_tgo().   // 例如 -2*alt/vel 或燃料/动力学闭式估计
set tgo to 0.85*tgo + 0.15*tgo_raw.   // 低通 = |σ-σ̄|≤Δσ

// 2. E-G 制导律（Apollo 类二次制导，充当"每帧一次的无求解器 SCP 步"）
set r to targetPos - shipPos.
set v to targetVel - shipVel.
set a_guide to 6*r/tgo^2 - 2*v/tgo.   // 使终端位置速度同时归零的闭式解
set a_cmd to a_guide - g.             // 思想8：重力抵消偏置

// 3. 虚拟控制/降级模式（思想5）
set a_avail to maxthrust/ship:mass.
if a_cmd:mag > a_avail {
    set deficit to a_cmd:mag - a_avail.
    set a_cmd to a_cmd:normalized * a_avail.   // 投影到可行边界
    if deficit > DEFICIT_ABORT { abortSequence(). }  // 持续缺陷 → 中止
}

// 4. 倾角约束（思想1：点积形式）
set thrustDir to a_cmd:normalized.
set tiltCos to vdot(thrustDir, up:vector).
if tiltCos < cos(TILT_MAX) {
    // 把推力方向旋回锥面：混合 up 方向直到贴锥
    set thrustDir to coneProject(thrustDir, up:vector, TILT_MAX).
    set a_cmd to thrustDir * a_cmd:mag.
}

// 5. 万向节/横向限幅（思想3）
set axial to vdot(a_cmd, facing:forevector).
set lateral to a_cmd - axial*facing:forevector.
if lateral:mag > tan(GIMBAL_MAX)*max(axial,0.01) {
    set lateral to lateral:normalized * tan(GIMBAL_MAX)*max(axial,0.01).
    set a_cmd to axial*facing:forevector + lateral.
}

// 6. 油门与信赖域限幅（思想4、7）
set thr to a_cmd:mag * ship:mass / maxthrust.
set thr to max(MIN_THROTTLE, min(1, thr)).
set thr to clamp(thr, thr_prev - DTHR, thr_prev + DTHR).  // 信赖域
set thr_prev to thr.
lock throttle to thr.
lock steering to thrustDir.
```

---

## E. 绝对不能搬进 kOS 的部分

1. **任何矩阵分解/批量预计算**：main.m:35-57 的 φ/λ/ψ 卷积阵、types.hpp:58 的 V 张量积分——内存与算力都不可行。
2. **自动微分雅可比**（vehicle.cpp:33-67，CppAD 代码生成 + 动态库编译）。
3. **RKF7(8) 自适应积分状态转移矩阵**（discretization.cpp 全文、solver.cpp:228）——每段积分 14×(14+6+2+2) 维 ODE。
4. **ECOS/CVX/任何 SOCP 求解器调用**（solver.cpp:349, 381；main.m:85-87）。
5. **z=ln(m) 变量替换 + 泰勒展开推力界的整套无损凸化机器**（main.m:60-64, 108-113）——它只为求解器服务；kOS 直接用线性质量模型 ṁ=−α‖u‖ 即可（B.2 的写法本来就更适合实时）。
6. **四元数单位范数的线性化下界 + slerp 初值**（solver.cpp:307-312, 190）——kOS 姿态由 steering 管理器闭环，不需要在制导层保证 ‖q‖=1。
7. **迭代收敛循环本身**（solver.cpp:378-429）——10-20 Hz 下只能做"每帧一步校正"的连续重规划，而不是批量 SCP。
8. **异常处理/配置校验模式**（solver.cpp:18-119 的 throw 机制）——kOS 无 try/catch，需改为状态标志轮询。

---

## 结论

两个代码库的价值不在代码而在**约束的解析形式**：倾角↔四元数二次型↔点积、滑翔角↔线性锥、万向节↔推力锥比、free-final-time↔σ 低通、虚拟控制↔降级、信赖域↔指令限幅。这些全部是标量/向量运算，可在 kOS 每帧执行；而求解器、离散化、自动微分属于地面离线工具链，不可上船。
