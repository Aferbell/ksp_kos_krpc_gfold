# G-FOLD (C++) 与 PythonRobotics 火箭动力着陆代码 可移植性报告
## ——面向 KSP + kOS 实时制导的约束移植分析

阅读对象：
- 代码库1（下称 **GF**）：`gfold_Cpp_G-FOLD_implementation/gfold-master/`
  - `src/gfold.cpp`（核心求解逻辑，202 行）
  - `src/gfold.h`（参数结构体与类接口，250 行）
  - `src/main.cpp`（主例程，火星着陆数值例，83 行）
- 代码库2（下称 **PR**）：`PythonRobotics_rocket_powered_landing.py`（674 行，Szmuk & Açıkmeşe 的 6-DoF Successive Convexification 实现）

kOS 约束背景：无矩阵库、无优化求解器、无 try/catch、三角函数为角度制、主循环 10–20 Hz、仅有标量 + 三维向量（vdot/vxcl/vecangle 等）。

---

## A. G-FOLD 完整问题陈述（GF）

### A.1 决策变量（`gfold.cpp` 第 53–64 行）
对每个时间片 i = 0 … N−1（N = steps）：
- `x`：6×N，状态 x = [r(3); ṙ(3)]（第 54–56 行，`r` = x 前三行，`r_dot` = 后三行）
- `u`：3×N，**推力/质量** 向量（u = T/m，见 `gfold.h` 第 223–231 行 "thrust divided by mass"）
- `σ`：1×N，推力幅值松弛变量（||u|| 的上界代理，第 62 行）
- `z`：1×N，对数质量 z = ln(m)（第 64 行，`gfold.h` 第 235–244 行）
- `obj`：标量，P3 的落点误差上界（第 53 行）

坐标系：e₁ = (1,0,0) 为"向上/当地垂线"轴（`main.cpp` 第 33 行 g = (−3.71, 0, 0) 沿 −e₁），E = [0 1 0; 0 0 1] 为横向（水平面）投影矩阵（`gfold.cpp` 第 46–47 行）。

### A.2 目标：P3 → P4 两级结构（`gfold.cpp` 第 66–68 行、148–156 行）
1. **P3（最小落点误差）**：min `obj`，s.t. ‖E·r(t_f) − q‖ ≤ obj（第 67–68 行）。q 为目标点水平坐标。
   P3 解出后取 target = ‖E·r(i_f) − q‖（第 148–149 行）。
2. **P4（最小燃料）**：在同一问题对象上追加 min Σᵢ σᵢ（即总推力积分 = 燃料，第 152 行），并追加落点约束 ‖E·r(t_f) − q‖ ≤ target（Eq. 20，第 154–155 行），重新求解。
即"先找到能落多准，再在保持该精度的前提下最省燃料"。

### A.3 动力学与逐条约束（`gfold.cpp`）

| 约束 | 位置（行号） | 精确数学形式 |
|---|---|---|
| 连续动力学（含行星自转） | 34–44 | A = [[0, I], [−S(ω)², −2S(ω)]]，B = [0; I]；S(ω) 为 ω 的反对称矩阵（34–37 行）。即 r̈ = g + u − 2ω×ṙ − ω×(ω×r)（科氏+离心项） |
| 离散动力学 Eq.(17) | 105–108 | x_{i+1} = x_i + (A x_i + B(g + u_i))·Δt —— **显式欧拉** |
| 初始对数质量 Eq.(7) | 74 | z₀ = ln(m₀) |
| 终端燃料约束 Eq.(7) | 76 | 0 ≤ z(i_f) − ln(m₀ − m_f)（即 z(t_f) ≥ ln(m_dry)，质量不低于干重） |
| 初始位置/速度 Eq.(8) | 79–81 | r(0) = r₀，ṙ(0) = ṙ₀（**逐分量等式**） |
| 终端高度 Eq.(9) | 84 | e₁ᵀ r(t_f) = 0（落到地面，e₁ 方向为零） |
| 终端速度 Eq.(9) | 86 | ṙ(t_f) = 0（三维全零，软着陆） |
| 速度上界 Eq.(5) | 89–90 | ‖ṙ_i‖ ≤ V_max，∀i（二阶锥） |
| 滑翔角锥 Eq.(5) | 92–97（i ≠ i_f） | ‖E(r_i − r_f)‖ ≤ (e₁/tan γ_gs)ᵀ(r_i − r_f)，即水平位移 ≤ 高度差 / tan(γ_gs)，γ_gs 为最小滑翔角（从地面量起） |
| 推力幅值松弛 Eq.(34) | 119–120 | ‖u_i‖ ≤ σ_i（二阶锥，lossless 松弛） |
| 推力指向锥 Eq.(34) | 122–123 | n̂ᵀ u_i ≥ cos(θ)·σ_i，推力轴与 n̂ 的夹角 ≤ θ |
| 质量演化 Eq.(33) | 110–111 | z_{i+1} = z_i − α·σ_i（注意：**此实现漏乘 dt**，见下注） |
| 推力上界 Eq.(36) | 125–136 | σ_i ≤ ρ₂ e^{−z₀ᵢ} (1 − (z_i − z₀ᵢ))，其中 z₀ᵢ = ln(m₀ − α ρ₂ iΔt)（最大推力随质量下降的线性化） |
| ~~推力下界 Eq.(35)~~ | 129–132 | 代码中被注释掉的 box(ρ₁e^{−z₀}(1−z_diff+…), σ, …) —— **本实现未启用 ρ₁ 下界约束**（虽然 lander_data 里有 rho_1） |

> 注：`main.cpp` 用 dt = 1 s（第 11 行），因此第 110 行缺 dt 不影响该例；换 dt ≠ 1 是 bug。这对移植者是一个警示：原作者的示例参数掩盖了离散化错误。

### A.4 求解流程（`gfold.cpp` compute()，168–182 行）
对 t_f 从 min_duration 到 max_duration 按 duration_search_interval **线性扫描**，对每个 t_f 解 P3+P4，第一个可行即停（173–179 行）。求解器：ECOS（epigraph 包装，139 行）。

---

## B. 初始条件、离散化与 t_f 的选择

1. **初始条件**（`main.cpp` 23–35 行 + `gfold.cpp` 79–81 行）：
   - r₀、ṙ₀ 为**任意三维向量**，逐分量硬等式钉死。例程用 r₀ = (2400, 3400, 0) m、ṙ₀ = (−40, 45, 0) m/s——注意 ṙ₀ 的 e₁ 分量为 **−40（下降）**，横向 +45。**对初始速度方向没有结构性假设**（不要求垂直向下、不要求对准目标）；滑翔角锥约束（92–97 行）只要求轨迹其余时刻留在锥内，初始点只要满足锥约束即可（代码中锥约束从 i=0 就施加，除 i=i_f 外）。
   - 因此 kOS 移植时可直接把传感器实测 r、v 代入，无需特殊前置机动。
2. **离散化**：N = ⌊t_f/dt⌋（`gfold.cpp` 28 行），dt 由调用者给定（`main.cpp` 11 行取 1 s），显式欧拉（106–108 行）。例程 t_f ∈ [45, 60] s、步长 1 s → N = 45–60。
3. **t_f**：不是优化变量，而是**外层暴力扫描**（172–174 行：45 s → 60 s，1 s 步进），第一个使 P3+P4 可行的 t_f 即被采用。这是 G-FOLD 论文标准做法（free-final-time 用一维搜索代替非凸性）。

---

## C. PythonRobotics（PR）与 G-FOLD（GF）的异同

| 维度 | GF（gfold-master） | PR（PythonRobotics） |
|---|---|---|
| 论文来源 | Açıkmeşe & Ploen G-FOLD（凸化后**一次 SOCP 精确求解**） | Szmuk & Açıkmeşe **Successive Convexification**（迭代求解非凸问题的凸近似序列） |
| 自由度 | 3-DoF 质点 + 质量（状态 7 维：r, v, ln m） | **6-DoF**（14 维状态：m, r, v, 四元数 q, 角速度 ω；`PythonRobotics` 第 52–53 行） |
| 动力学离散化 | 显式欧拉（gfold.cpp 106–108） | **精确零阶/一阶保持离散化**：对状态转移矩阵 Φ_A、B̄、C̄、S̄、z̄ 做 ODE 积分（PR 319–416 行，odeint） |
| 离散点数 | N = t_f/dt（45–60） | K = 50 固定（PR 23 行），归一化时间 dt = 1/(K−1)（352 行），真实时长 = σ（自由终时） |
| 终时 t_f | 外层线性扫描 t_f∈[45,60]（gfold.cpp 172–179） | σ 是优化变量，通过时间伸缩 free-final-time（PR 433 行），初猜 t_f_guess = 10 s（60 行） |
| 凸化手法 | 变量替换 u=T/m, σ=‖T‖/m, z=ln m + lossless 松弛 ‖u‖≤σ | 每次迭代在当前参考轨迹处线性化（A_func/B_func 雅可比，PR 150–203 行），加**虚拟控制 ν**（434 行，权重 1e5，32 行）与**信赖域** ‖Δx‖+‖Δu‖（478–485 行） |
| 推力下界 | 未启用（被注释，gfold.cpp 129–132） | 线性化下界 T_min ≤ (Û/‖Û‖)ᵀU（PR 309–314 行） |
| 推力指向 | 锥约束 n̂ᵀu ≥ cosθ·σ（gfold.cpp 122–123） | 摆角约束 ‖U_{1:3}‖ ≤ tan δ_max·U₀（PR 304–305 行）+ 姿态角约束 ‖q_{1:2}‖ ≤ √((1−cosθ_max)/2)（298–299 行） |
| 滑翔角锥 | 有，γ_gs（gfold.cpp 92–97） | 有，‖r_{2:4}‖ ≤ r₁/tan γ_gs（PR 296–297 行） |
| 速度上界 | 有，V_max（gfold.cpp 90） | 无显式 V_max；有角速度上界 ‖ω‖ ≤ 60°/s（PR 300–301 行） |
| 行星自转 | 含科氏/离心项（gfold.cpp 34–41） | 无（均匀重力场 g = (−1,0,0)，PR 87 行） |
| 求解器 | ECOS via epigraph | ECOS via cvxpy（PR 35 行） |
| 收敛判据 | SOCP 一次求解即最优 | δ_norm<1e−3 且 σ_norm<1e−3 且 ‖ν‖∞<1e−7（PR 656 行），最多 30 次迭代（26 行），信赖域权重 ×1.5/次（659 行） |
| 初始轨迹 | 无需（直接凸问题） | 初末状态线性插值，U = −m·g 悬停猜测（PR 249–268 行） |

**共同点**：滑翔角锥、推力幅值上下界、推力指向锥、质量消耗 ∝ ‖推力‖ 这四类**约束的物理形式完全一致**——这正是可移植到 kOS 的核心子集。

---

## D. 可移植到 kOS 的约束物理含义（每帧状态检查 + 指令投影）

以下约束都是**瞬时不等式**，不含跨时刻耦合，可用标量 + 三维向量在每帧 O(dt) 计算。kOS 三角函数用角度制（sin/cos/arctan 直接收/出角度），向量运算用 VDOT、VXCL、VECANGLE、:MAG、:NORMALIZED。

设（每帧可获得）：
- `r` = 落点相对向量：目标点位置 − 本船位置，在"up-east-north"式局部系；分解为 高度 h = VDOT(r, up) 与 水平向量 rh = VXCL(up, r)。
- `v` = 地面系速度；`m` = 船质量（SHIP:MASS）；`Tmax`/`Tmin` = 引擎可用推力上下限；`adir` = 期望加速度指令（制导律输出）；`up` = 当地天顶单位向量；`n̂` = 推力参考方向（通常取 up）。

### D.1 滑翔角锥（GF 92–97 行；PR 296–297 行）
物理含义：飞行器必须保持在以落点为顶点、半顶角 (90°−γ_gs) 的锥内——防止轨迹过于贴地/撞山。
- 精确形式：‖r_h‖ ≤ h / tan(γ_gs) ⟺ arctan(‖r_h‖/h) ≤ 90°−γ_gs（从水平面量起的高度角 ≥ γ_gs）。
- kOS 检查（角度制直接可用）：
```kerboscript
// 是否违反锥约束
LOCAL elev IS ARCTAN2(h, rh:MAG).        // 高度角（度）
IF elev < gamma_gs { /* 违反：需要抬高轨迹 */ }
```
- 指令投影：限制指令加速度的"向下"分量，使预测下一帧位置仍在锥内：
```kerboscript
// 简单的预测-钳制
LOCAL r_next IS r + v*dt.                 // 无控预测
LOCAL h_n IS VDOT(r_next, up).
LOCAL rh_n IS VXCL(up, r_next).
IF ARCTAN2(h_n, rh_n:MAG) < gamma_gs {
    // 需要的最小上升加速度修正（沿 up）
    LOCAL dh_need IS rh_n:MAG / TAN(gamma_gs) - h_n.
    SET adir TO adir + up:NORMALIZED * (dh_need / (dt*dt)).
}
```

### D.2 推力指向锥（GF 122–123 行；PR 298–305 行）
物理含义：推力方向不得偏离参考轴 n̂（近似竖直）超过 θ_max——保证姿态可控、避免大过载翻转。
- 精确形式：n̂ᵀT ≥ cos(θ_max)·‖T‖ ⟺ VECANGLE(T, n̂) ≤ θ_max。
- 指令投影（把指令向量投影到锥面上，纯向量运算）：
```kerboscript
FUNCTION project_cone {  // a_cmd: 指令加速度, n: 参考轴(单位), amax: 半锥角(度)
  PARAMETER a_cmd, n, amax.
  LOCAL along IS VDOT(a_cmd, n).
  LOCAL perp  IS a_cmd - n*along.          // 或 VXCL(n, a_cmd)
  LOCAL perp_max IS along * TAN(amax).     // 横向允许上限
  IF perp:MAG > perp_max AND perp_max > 0 {
    SET perp TO perp:NORMALIZED * perp_max.
  }
  RETURN n*along + perp.
}
```
（这就是 GF 锥约束的对偶操作：‖T⊥‖ ≤ tanθ·T∥，与 PR 304–305 行摆角约束形式相同。）

### D.3 推力幅值上下界（GF 119–136 行；PR 306–314 行）
物理含义：引擎节流阀有死区，T ∈ [ρ₁, ρ₂]（不能任意小）；上界随质量下降 e^{−z} 项在 kOS 中直接用实时质量代替。
- 投影：
```kerboscript
LOCAL a_req  IS adir:MAG.                  // 期望加速度幅值
LOCAL T_req  IS m * a_req.                 // 期望推力 (N)
LOCAL T_cmd  IS MAX(rho1, MIN(rho2, T_req)).
// 幅值被钳制后，保持方向不变
IF a_req > 0 { SET adir TO adir:NORMALIZED * (T_cmd / m). }
SET throt TO T_cmd / availthrust_max.
```
- 若要严格复现 GF 的"推力随质量上界"：ρ₂ 用当前 m 标定即可（TWR 不变），无需 e^{−z₀} 线性化——那是求解器需要的凸近似，不是物理。

### D.4 速度上界（GF 89–90 行）
物理含义：结构/热/可控性限速。
```kerboscript
IF v:MAG > V_max {
    // 要求指令加速度含有逆速度分量
    LOCAL v_dir IS v:NORMALIZED.
    LOCAL a_along IS VDOT(adir, v_dir).
    IF a_along > 0 { SET adir TO adir - v_dir * a_along. } // 去掉加速分量
}
```

### D.5 质量演化（GF 74、76、110–111 行；PR 90、131 行）
物理含义：ṁ = −α‖T‖，燃料耗尽即 m ≥ m_dry。可作为**可行性监控**：
```kerboscript
// 燃料可行性检查（无需求解器）
LOCAL mdot IS alpha * throt * availthrust_max.   // kg/s
LOCAL t_fuel IS (m - m_dry) / MAX(mdot, 0.001).
IF t_fuel < tgo_estimate * 1.1 { /* 触发放弃/紧急程序 */ }
```
z = ln(m) 变量替换本身（GF 64 行）**不需要移植**——它只为把动力学变凸，kOS 直接读 SHIP:MASS。

### D.6 终端约束（GF 84–86 行；PR 63–66、289–290 行）
物理含义：r(t_f) 的高度=0、v(t_f)=0（GF）或 v(t_f)=(−0.1,0,0) 触地速度（PR 64 行）。可移植为**触地前的终端走廊检查**：
```kerboscript
IF h < h_touchdown_band {
    // 投影指令，使 |v| ≤ v_touch_max 且垂直速度 ≤ 限值
    LOCAL v_vert IS VDOT(v, up).
    IF v_vert < -v_touch_max {
        SET adir TO adir + up:NORMALIZED * ((-v_touch_max - v_vert)/dt).
    }
}
```

> 这些投影可以**级联**：先限速，再指向锥，再推力幅值钳制，再终端走廊——每帧总开销约 20 次向量运算，10–20 Hz 下对 kOS 无压力。kOS 无 try/catch 不影响：所有操作需防除零（用 MAX(x, ε) 保护，上面伪代码已体现）。

---

## E. 纯求解器依赖、绝对不能搬的部分

| 项 | 位置 | 为什么不能搬 |
|---|---|---|
| SOCP 求解本身（ECOS/epigraph/cvxpy） | GF 50、139；PR 35、500、539 | kOS 无求解器，且单次求解在 GF 中也要扫描多个 t_f × 数十秒轨迹——10 Hz 实时不可行 |
| t_f 外层线性扫描 | GF 168–182 | 每个候选 t_f 都要重解一次 SOCP；kOS 只能用解析 tgo 估计（如 Δv/TWR 估计）替代 |
| 变量替换 z=ln m、u=T/m、σ 松弛 | GF 58–64 | 纯凸化技巧，无物理执行意义 |
| 推力下界的逐点线性化（Eq. 35 注释块 / PR 309–314） | GF 129–132；PR 309–314 | 依赖上一次迭代解 Û 做一阶泰勒展开——是 SCvx 迭代机制的一部分 |
| SCvx 全部机制：状态转移矩阵积分（ĀB̄C̄S̄z̄）、虚拟控制 ν、信赖域、迭代收敛判据、权重递增 | PR 319–416（ODE 积分）、434、478–485、656–659 | 需要 14×14 矩阵求逆 + ODE 积分器 + LP/SOCP 求解器；kOS 无矩阵库，且 30 次迭代 × 50 节点的离线性质与 10–20 Hz 在线制导不兼容 |
| 6-DoF 姿态动力学（四元数传播、陀螺力矩） | PR 125–148、141–144 | kOS 中姿态由飞船 SAS/PID 舵控系统负责，制导层只输出指向；姿态可行性用 D.2 指向锥间接保证即可 |
| P3/P4 两级目标的"最小落点误差" | GF 66–68、148–156 | 需要全局优化；kOS 中改为"误差反馈 + 指令投影"（把落点预测误差转成加速度修正） |
| 显式欧拉离散的等式动力学 x_{i+1}=… | GF 106–108 | 优化变量间的等式约束，无法也无须移植；kOS 由物理引擎推进状态 |
| 科氏/离心项矩阵 A | GF 34–41 | KSP 的 KSP/惯性系转换已处理自转，制导用地面系速度即可，不必显式建模 |

### 结论一句话
两个代码库中**可移植的是约束几何（滑翔角锥、指向锥、推力上下界、限速、终端走廊）**，它们都可写成逐帧的标量/向量不等式 + 解析投影；**不可移植的是一切"跨越整条轨迹"的东西**（离散化动力学等式、变量替换、线性化、信赖域、求解器调用、t_f 搜索）。kOS 侧的正确架构是：离线用 G-FOLD/SCvx 生成标定参数（γ_gs、θ_max、ρ₁/ρ₂、V_max），在线只做 D 节的"状态检查 + 指令投影"安全过滤器，包在任意制导律（如现有 exec_lvd_control.ks）之外。
