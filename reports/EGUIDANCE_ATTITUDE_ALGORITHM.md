# 回收段"位置+速度+姿态(倾角)"三约束制导算法调研报告
## —— 面向 kOS 一级火箭定点软着陆的制导律重构

> 目标指标：落点 < 1 m、触地 vz ≈ −2 m/s、倾角 < 5°。
> 约束环境：kOS（Kerbal OS）只有标量/向量运算，**无数值优化求解器**，**每帧实时**（O(1)）。
> 调研对象：Apollo11 AGC 源码、PythonRobotics rocket_powered_landing、G-FOLD(C++)、lcvx(dmalyuta)、SCvx(C++ 6DoF)、LosslessConvexification(MATLAB)、文献资料汇总.md 及 25 篇论文笔记。

---

## 摘要（结论速览）

1. **Apollo E-guidance（二次多项式制导律）天然只闭合"终端位置+终端速度"6 个标量约束，姿态是它的盲区**（证据：`文献资料汇总.md:37`、`2025_分岔法…md:47`）。它的 `ACG = 12(RDG−RG)/T² + 6(VDG+VG)/T + ADG` 一项都不含姿态。
2. **Apollo 把"姿态竖直"拆到分阶段架构里解决，而不是塞进二次律**：P63/P64 用二次律管位置+速度 → P65 用一阶律消速度 → P66 用 ROD（下降率）+ 姿态保持管末段；P66 里用**硬限幅"推力方向 ≤ 20° from vertical"**保竖直。这正是 kOS 该抄的结构。
3. **glide slope 是位置约束，thrust pointing 是（准）姿态约束**——四个凸优化库一致。3DoF 库（G-FOLD/lcvx/Lossless）无姿态状态，"推力指向"只是推力矢量方向；只有 6DoF SCvx 用四元数 `‖[q2,q3]‖ ≤ √((1−cosθ)/2)` 实现了真正的姿态倾角锥。**这些都需要求解器，kOS 抄不了其求解，只能抄其约束形式**。
4. **存在"闭式律 + 姿态边界"的统一做法**：把多项式从二次升到**四次（quartic）**，即可把"终端加速度方向 = 终端推力方向 = 终端姿态"作为边界条件嵌入（`2025_重复使用火箭垂直着陆…md:103-106, 610-628`）。这是在 kOS 里"用一条闭式律同时管位置+速度+末端姿态"的最干净途径。
5. **推荐 kOS 结构**：主段四次多项式 E-guidance（位置+速度+末端姿态边界）→ 低空 gate（≈30 m / TTF≈5–8 s）切换竖直段 → 竖直段一阶速度归零 + 推力方向渐近 up + 姿态 P 律。全程 O(1) 标量/向量。

---

## 问题 1：Apollo E-guidance / 二次制导律

### 1.1 二次制导律如何同时约束末速度和末位置

**公式（源码注释，`LUNAR_LANDING_GUIDANCE_EQUATIONS.agc:533-544`）：**

```
#	AS PUBLISHED --
#		ACG = ADG + 6(VDG+VG)/TTF + 12(RDG-RG)/TTF²      (:534-537)
#	AS HERE PROGRAMMED --
#		ACG = (3/4)[ (1/4)(RDG-RG)/(TTF/8) + VDG + VG ]/(TTF/8) + ADG   (:539-543)
```

- 发布形式 `ACG = ADG + 6(VDG+VG)/TTF + 12(RDG−RG)/TTF²`，即 Klumpp 论文里大家熟知的
  **a = ADG + 6(vf+v)/T + 12(rf−r)/T²**（与笔记 `Klumpp1971…md:673-679` 的 `ACG = 12(RTG−RG)/T² + 6(VTG+VG)/T + ATG` 完全一致，是隐式制导取 Kr=12, Kv=−6 的特例）。
- **实现代码 `QUADGUID` 在 `:546-627`**：第 581–599 行依次把 `VGU×系数 + VDG×系数 + (RDG−RGU)/(TTF/8)×系数` 累加，最后 `VAD ADG`（加目标加速度）。

**为什么它能同时约束末位置+末速度**（每轴 5 自由度、四次多项式是最低阶，见 `Klumpp1971…md:607-612`）：
这条律本质是"从当前 (r, v) 出发、用一条多项式弹道在 TTF 后精确命中 (RDG, VDG, ADG)"。TTF（time-to-go）不是拍的，而是由**三次方程求根**解出（`TTF/8CL, :489-522`），其系数含 JDG(jerk)、ADG、VDG、RDG 与当前 RGU/VGU——即**显式解一个"满足终端位置/速度/加速度约束所需时间"的方程**（牛顿法 ROOTPSRS，`:1272+`）。每个制导周期重算 TTF 与 ACG，形成闭环重瞄准。所以**位置与速度约束是通过"TTF 求根 + 二次型加速度指令"同时闭合的**。

### 1.2 它是否/如何约束末端姿态（倾角）

**不约束。** 制导方程里没有任何姿态量：
- 状态只有位置 RGU、速度 VGU（`RGVGCALC, :410-480`：`RGU = CG(R−LAND)`、`VGU = CG(V − WM×R)`），坐标变换矩阵 CG 的第一行是 `UNIT(LAND)`（着陆点当地竖直方向，`CGCALC :661-663`）——即**制导系 X 轴始终对准当地竖直**，但这只是把加速度指令分解到"竖直/水平通道"的坐标选择，**不是对箭体姿态的约束**。
- 文献笔记一致确认：`文献资料汇总.md:37`"E制导（多项式制导）本身只能闭合'终端位置+终端速度'共 6 个标量约束，**倾角/姿态约束属于它的盲区**"；`2025_分岔法…md:47`"In E-guidance law only initial and final positions and velocity constraints can be satisfied"。

**Apollo 怎么间接得到末端竖直（关键）**：姿态不是由二次律保证的，而是：
1. **坐标系构造**：CG 第一行 = 当地竖直，使推力指令天然被分解，末端时推力主分量在竖直方向（`:600-608 AFCCALC1` 经 `VXM CG` 转回惯性系）。
2. **P64 末段指令向量渐近竖直**：`EXNORM (:748-773)` 先取 `UNIT(LAND−R)`（指向着陆点上方≈当地竖直）作为指令向量初值，再用 `PROJ = UNIT(LAND−R) 与窗口方向的投影` 同 `PROJMAX=SIN(25°)/8、PROJMIN=SIN(15°)/8 (:1455-1456)` 比较，**在斜距方向与竖直方向之间线性混合**——接近竖直时把指令向量渐变到 `UNIT(LAND−R)`。这就是"P64 末段推力方向渐近当地竖直"的具体代码。
3. **P66 推力方向硬限幅**（最直接的末端姿态手段，笔记 `Klumpp1971…md:1092-1095`）："The direction of the thrust-acceleration command is **limited to 20° from vertical** … to maintain a nearly erect LM attitude"。

### 1.3 P63/P64 的目标条件与切换门

**目标位置**：着陆点向量 `LAND`（月固系，由 `RLS` 经 `RP-TO-R` 算出，`THE_LUNAR_LANDING.agc:77-88 STCALL LAND`）；`/LAND/` = 着陆点月半径。每拍按月球自转修正（`TTFINCR :288-322`：`LAND = /LAND/·UNIT(LAND − LAND·ΔT×WM)`）。**实际变量名是 `LAND`/`RLS`，不是注释里可能出现的 LRLAND/RLAND**（见 §5 盲信注释警示）。

**目标速度/加速度**：`VGU→VDG`、`ADG`，按段取不同目标组：
- `RDG=RBRFG, VDG=VBRFG, ADG=ABRFG`（制动段高门）/接近段 `RAPFG/VAPFG/AAPFG`，变量等价定义在 `:1435-1440`；
- 用 `TARGTDEX` 索引选组（`:97-99`：BRAKQUAD 偏移 0、APPRQUAD 偏移 034 八进制）。

**切换门（P63→P64→P65），集中在 `EXTLOGIC :692-709`：**
```
EXTLOGIC  INDEX WCHPHASE     # 0=BRAKQUAD(P63), 1=APPRQUAD(P64)
          CA    TENDBRAK     # 表格: WCHPHASE=0 取 TENDBRAK, =1 取 TENDAPPR
          AD    TTF/8
          BZMF  WHATEXIT     # TENDBRAK + TTF/8 ≤ 0 → 不切换
          ...                # 否则 WCHPHASE+1（CA WCHPHOLD / AD ONE / TS WCHPHASE）
```
- **判据量是 TTF（time-to-go）≤ −TENDBRAK / −TENDAPPR**（TENDBRAK/TENDAPPR 为负值 pad-load，"LANDING PHASE SWITCHING CRITERION"，`ERASABLE_ASSIGNMENTS.agc:2096-2099`）。
- **注意：切换不直接用高度或目标角，而是用 TTF 时间门。** 高度相关量（LOWCRIT/HIGHCRIT）用于别的环节（如 LR 数据合理性），目标角 LOOKANGL 仅供 P64 显示（`:473-480`）。WCHPHASE 含义：`:40-44`：−1=IGNALG, 0=P63, 1=P64, 2=VERTICAL(P65/66/67)。
- P64 启动 `STARTP64 :260-268`：`CA DELTTFAP / ADS TTF/8TMP`（切换瞬间给 TTF 加增量）并使能 RUPT10（redesignator 中断）。P65 启动 `P65START :252-258`。

### 1.4 P65 与 P66 的分工

分派在 `VERTGUID :897-899`：WCHVERT>0→P67，=0→P66，<0→P65。

- **P65（速度归零，velocity nulling）**：制导式 `:901-913`：
  ```
  #	ACG = (V2FG - VGU) / TAUVERT
  P65VERT  VLOAD VSU  V2FG, VGU   →  V/SC TAUVERT  →  GOTO AFCCALC1
  ```
  一阶指数收敛到目标速度 V2FG（"DESIRED VELOCITY FOR P65 / TIME CONSTANT FOR P65 VEL. NULLING"，`ERASABLE:1370-1371`）。**只消速度，不管位置。**

- **P66（rate-of-descent + 姿态保持）**：`P66VERT→P66VERTA (:919-920, 941-946)` 启动 1 秒周期 `RODTASK/RODCOMP (:934-946)`：
  - 期望下降率：`RODCOUNT×RODSCAL1` 累加进 `VDGVERT`（`:948-953`）；进入 P66 时 `VDGVERT` 初始化为当前 `HDOTDISP`（`GUILDEN :147-148`）。
  - 加速度指令：`(VDGVERT − HDOTDISP)/TAUROD`（`:1031-1034`）+ 重力补偿 + 测量加速度前馈 DELVROD + 滞后补偿 + MINFORCE/MAXFORCE 限幅（`:1076-1094`）→ `/AFC/` 送 THROTTLE。
  - **姿态保持**：P66 不调 FINDCDUW 改姿态。`RATESTOP (:811-818)` 检测 CHAN31 BIT13（ATTITUDE-HOLD 开关）：处于姿态保持则直接退出，否则 `STOPRATE`（停角速度）。`GUILDEN (:130-216)` 监听手动油门/姿态保持离散量自动在 P66/P67 间切换（"ON EVERY APPEARANCE OF THE ATTITUDE-HOLD DISCRETE TO SELECT P66"）。
  - **分工总结**：P65 把速度一阶压到门限（水平速度归零→产生 vertical approach，笔记 `:377-384`），P66 接管后用"恒下降率控制 + 姿态保持 + 推力方向 ≤20° from vertical"把箭体竖直地、以受控下降率放到月面。笔记 `Klumpp1971…md:376-380`：P66 典型在 **30 m 高度、11 m 地面距离**自动开始。

---

## 问题 2：姿态约束怎么处理（凸优化库对照）

**四个库一致结论：glide slope = 位置约束；thrust pointing 分两种形态；只有 6DoF SCvx 有真姿态锥。**

### 2.1 Glide slope（滑翔角锥）＝ 位置约束

只含位置 `r`，不含姿态：
- **G-FOLD C++**（`gfold.cpp`）：状态仅 6 维（位置+速度，`L38 A=Zero(6,6)`、`L54 addVariable("x",6,…)`），**无姿态状态**。glide slope：`L46-48 E=[0 1 0;0 0 1]`、`c=e1/tan(γ_gs)`；`L94-95 ‖E(r−r_f)‖ ≤ (1/tanγ)·e1ᵀ(r−r_f)`——纯位置锥。
- **lcvx（dmalyuta）**（`landing.py`）：`L154-157 [0,1]·Ex·x[k] ≥ ‖Ex·x[k]‖·cos(gs)`（余弦形式位置锥）。
- **SCvx 6DoF**（`solver.cpp`）：`L298-299 ‖X(ry:rz)‖ ≤ (1/tanγ_gs)·X(rx)`，不含 q。
- **LosslessConvexification**（`main.m`）：glide slope 那段锥约束**被注释掉未启用**（`L121-122`），全文件无姿态约束。

文献侧佐证：glide-slope 锥保证"轨迹在锥内、不过浅、不穿地"（`2010_专利US20100228409A1…md:475-481, 736-748`）；`2024_航空学报…md:607-624` 式(12) `rz ≥ ‖r‖cosυ` 是"轨迹倾斜角约束"。**金字塔形接近角约束**（`2026_金字塔…md:53, 193-196`）是把 glide-slope 圆锥改成"以落点为顶点、由若干三角面围成的倒金字塔凸多面锥"，逼近角由金字塔面倾角设定——**仍约束位置，不是姿态**。

### 2.2 Thrust pointing / tilt —— 3DoF 是控制量约束，6DoF 才是真姿态约束

- **3DoF 质点库**（G-FOLD、lcvx landing、Lossless）：模型无姿态状态，约束的是**推力矢量 u 与竖直方向夹角**（属控制量约束，不是姿态约束）：
  - G-FOLD `L121-123 n̂ᵀu ≥ cos(θ)·‖u‖`（Eq.19）；
  - lcvx `landing.py L152 C[i]·u ≤ 0`，锥构造 `lib/tools.py L281-284` 以竖直为轴、推力与竖直夹角 ≤ α/2。
- **6DoF SCvx**（`solver.cpp`，状态 14 维含四元数+角速度，`types.hpp L12 N_X=14, L19-22 r=1:3,v=4:6,q=7:10,w=11:13`）：**真正的姿态倾角锥**
  - 全程：`L300-301 ‖[q2,q3]‖ ≤ √((1−cosθ_max)/2)`（限制体 x 轴偏离竖直的倾角）；
  - 末端：`L288-289 ‖[q2,q3](K-1)‖ ≤ √((1−cosθ_f_max)/2)`（末端更严）+ `L290-291 末端角速度 w_b(K-1)=0` + `L292-293 末端推力仅体系竖直分量 U(K-1)(2:3)=0`；
  - Gimbal 角约束：`L321-322 ‖U‖ ≤ U_x/cos(δ_max)`（推力在体系内方向锥）。
  - **注意**：其末端姿态是**倾角锥约束（θ_f_max）+ 零角速度**，而非严格 `q_final=[1,0,0,0]` 等式（竖直仅出现在初值猜测 `L179 q_b_i_f={1,0,0,0}`）。

### 2.3 PythonRobotics rocket_powered_landing 的末端姿态

`PythonRobotics_rocket_powered_landing.py` 是 6DoF SCvx（Szmuk & Açıkmeşe 论文的 Python 版，见文件 docstring `:9-12`）：
- 末端边界（`get_constraints :289-290`）：`X_v[1:,-1] == x_final[1:]`（位置+速度+姿态+角速度全固定，其中 `q_B_I_final = euler_to_quat((0,0,0))` 竖直，`w_B_final=0`，见 `:63-66`）+ `U_v[1:3,-1]==0`（末端推力横向分量为 0）。
- 全程姿态锥（`:298-299`）：`‖X_v[9:11,:]‖ ≤ √((1−cosθ_max)/2)`（四元数 q2,q3 的倾角锥，与 SCvx C++ 同型）；
- 滑翔角位置锥（`:296-297`）：`‖X_v[2:4,:]‖ ≤ X_v[1,:]/tan(γ_gs)`；
- gimbal 角（`:304-305`）：`‖U_v[1:3,:]‖ ≤ tan(δ_max)·U_v[0,:]`。
- **但**：它每一步都要求解一个 SOCP（ECOS solver，`:35 solver='ECOS'`、`:539 prob.solve()`），迭代 ≤30 次（`:26`），每次迭代内部还有 ODE 积分求离散化矩阵——**这正是 kOS 无法承担的"求解器"**。

### 2.4 kOS（无求解器）里姿态约束的成熟做法

**结论：kOS 里应把"末端姿态竖直"作为终端段的独立目标，而非全程约束。** 理由：
1. 全程推力指向锥（thrust pointing cone）在数学上是二阶锥，需要求解器才能最优满足；kOS 没有。
2. 但**工程上等价的做法**：主段用闭式律管位置+速度（这是位置/速度通道，每帧 O(1) 向量代数），到**低空 gate 后进入竖直段**，此时水平速度已很小，把目标简化为"竖直下落"，推力方向自然趋于 up，姿态用一个简单律回正。
3. 这正是 Apollo 的实际架构（P63/P64 → P65/P66）与文献建议（嫦娥3号在主减速后设**专门过渡段**完成"姿态从水平到接近垂直的过渡"，`2015_深空探测学报…md:225-236`；凸优化文献用**末段节点线性收紧**推力角/轨迹倾斜角 `2024_航空学报…md:651-675`）。

**例外（推荐增强）**：若想在**同一条闭式律里**就带上末端姿态，把二次升到**四次多项式**，把"终端加速度方向 = up"作为边界条件嵌入（见问题 3）。这是 kOS 可行（仍 O(1)）且比"两段硬切换"更平滑的方案。

---

## 问题 3：三约束的统一律

### 3.1 是否存在"二次型 + 末端姿态目标"的闭式制导律？

**纯二次型不行**（姿态是其盲区，§1.2）。**但把"推力方向 = up"作为终端加速度边界条件嵌入、并把多项式升到四次，就可以用一条闭式律同时闭合位置+速度+末端姿态。**

- **四次（quartic）多项式制导**（`2025_重复使用火箭垂直着陆…md`）：
  - `:103-106` "考虑终端姿态约束…四次多项式制导律"；
  - `:610-620` "四次多项式制导…在求解制导律的过程中**引入了姿态约束**，与凸优化的**终端推力约束**相吻合"；
  - 式(18) `:621-628`：**`ua = af + 12(rf−r0)/tg² − 6(vf+v0)/tg − g`**（注意与二次律同构，但多了终端加速度 `af` 项，且 tg 求解不同）；
  - 式(19) `:642-656`：由 `H(tg)=0` 求 tg 的四次方程。
  - **机理**：二次律闭合"位置+速度"（加速度由前两者唯一确定，姿态不可控）；四次律多引入"终端加速度 `af`"这一边界，而**推力沿体轴**（该文假设3，`:186-188`），所以 `af` 的方向 = 终端推力方向 = 终端姿态。把 `af` 设为竖直（up 方向），即把"末端姿态竖直"嵌进了制导律。该文 `:317-321` 明言"火箭在着陆时要求姿态保持严格垂直"，由终端推力约束 `T(tf)=[0,Ty,0]ᵀ`（`:341-347` 式6）实现。
- **五次多项式**进一步把"初始姿态角（初始加速度 a0）"也作边界（`2024_航空学报…md:427-432`），避免初始姿态剧烈变化——对我们的"从倾倒/翻转状态回收"有用，但通常四次已够。
- **解析最优路线**：`2026_金字塔…md` 用 PMP 推**分段解析闭式解**（`:173-175` 无约束弧 `u*=−6r/tgo²−4V/tgo−g`，约束激活时"抵消约束面法向重力分量"`u*n=−gn`，`:606-611`），是"闭式律同时管位置+速度+路径接近角"的最干净做法——但它约束的是**轨迹接近角**，不是箭体姿态倾角。

**关键换算（kOS 里必须想清楚的）**：单矢量推力火箭，推力沿体轴，**"推力方向"就是"姿态"**。所以：
- "末端姿态竖直(倾角<5°)" ⟺ "末端推力方向偏离 up < 5°" ⟺ "末端加速度矢量的水平分量 ≈ 0"。
- 四次律通过 `af` 边界直接控制末端加速度方向 ⇒ 控制末端姿态。这是**把姿态约束翻译成加速度边界条件**，是唯一不需要姿态动力学/求解器的闭式途径。

### 3.2 Apollo 的做法是不是"主段 E-guidance 到 gate，之后 P65 消速度、P66 保姿态"？

**是。** 源码与笔记都确认这个三段结构（§1.3、§1.4）：
- 主段 P63/P64：二次 E-guidance 管**位置+速度**到 gate（TTF 时间门切换，不直接用高度）；
- P65：一阶律 `ACG=(V2FG−VGU)/TAUVERT` 消速度（把水平速度归零→vertical approach）；
- P66：恒下降率(ROD)控制 + 姿态保持 + 推力方向 ≤20° from vertical，把箭体竖直、受控地放到月面。
- 切换门：P64→P65 由 TTF 门（`EXTLOGIC :692-709`），P65→P66 典型在 30 m / 11 m 地面距离（笔记 `Klumpp1971…md:376-380`），也可手动。

**这与"四次多项式统一律"是两条等价路线**：Apollo 用"分段 + 各段专门律"；四次多项式用"一条高阶闭式律嵌入姿态边界"。kOS 里两者都可行（都 O(1)），见问题 4 推荐。

### 3.3 PEG 是否约束姿态？

**不约束。** PEG（Powered Explicit Guidance）目标集是"轨道能量/圆化/拱线/r,v,γ/轨道面"（`2025_航天飞机PEG…AAS25-844.md:241-243`），全部为目标位置、速度、航迹角、轨道面；**无姿态**。PEG 的**输出就是推力方向本身**（`Jaggers1976_PEG…md:294-298`"determine real time values of the unit thrust vector i+…steer to desired target conditions"；`:488-499` linear-tangent steering `i = λ + λ̇(t−K)`）——**姿态（推力指向）是 PEG 的输出量，不是约束对象**。对长弧/低推重比的发散用 safeguard 限幅（`2018_PEG改进与增强_SLS…md:264-302`），是工程限幅，不是终端姿态边界条件。**所以 PEG 不适合直接拿来做定点软着陆的末端姿态控制**（它是上升段/轨道制导工具）。

---

## 问题 4：kOS 可实现的推荐结构

### 4.1 总体架构（三段，全部每帧 O(1) 标量/向量）

```
┌─────────────────────────────────────────────────────────────────┐
│ 段1 主减速/制导段（高空 → gate）                                   │
│   律：四次多项式 E-guidance（位置+速度+末端姿态边界，一条闭式律）      │
│   负责约束：位置(r)、速度(v)、末端姿态(经 af=up 边界)               │
│   输出：指令加速度 a_cmd → 油门=|a_cmd|·m/Tmax，姿态=沿 a_cmd      │
├─────────────────────────────────────────────────────────────────┤
│ 段2 竖直段（gate 后，≈30 m / TTF≈5–8 s）                          │
│   律：一阶速度归零（P65 型）a_cmd=(v_target−v)/τ + g_up           │
│        + 推力方向渐近 up（把 a_cmd 的水平分量渐限到 0）             │
│   负责约束：速度归零、姿态回正（倾角→<5°）                          │
├─────────────────────────────────────────────────────────────────┤
│ 段3 触地段（≈数 m）                                               │
│   律：恒下降率控制（P66 ROD 型）a_cmd=(vz_target−vz)/τ_rod + g    │
│        水平速度 P 律归零 + 姿态 P 律保持竖直                        │
│   负责约束：触地 vz≈−2 m/s、落点精修、姿态保持竖直                  │
└─────────────────────────────────────────────────────────────────┘
```

### 4.2 各段伪代码（kOS 风格，O(1)）

**段1：四次多项式 E-guidance（主推）**

```
// 每帧
r_err  = r_target − r            // 位置误差（向量）
v_err  = v_target − v            // 速度误差（向量）
tgo    = solve_quartic_H(r_err, v_err, a_f, g)   // 四次方程求 tg（牛顿 3-5 次迭代，O(1)）
a_cmd  = a_f + 12*r_err/tgo² − 6*(v_target + v)/tgo − g
       // a_f 设为 (g, 0, 0) 方向 = up，即"末端推力竖直"= 末端姿态竖直
throttle = clamp(|a_cmd|*mass / max_thrust, 0, 1)
set steering to a_cmd:direction   // 箭体对准指令加速度方向
```
> 位置约束 ← `12·r_err/tgo²`；速度约束 ← `−6(v_target+v)/tgo`；**末端姿态约束 ← `a_f=up` 边界**。tgo 求根用牛顿法（kOS 只有标量/向量，3–5 次迭代即可，参考 Apollo 用 ROOTPSRS 牛顿法解三次方程 `LUNAR_LANDING_GUIDANCE_EQUATIONS.agc:1272+`）。
> 若嫌四次方程求根麻烦，退回**二次律**（Apollo 原版 `a = 12(rf−r)/T²+6(vf+v)/T+af`），姿态交给段2 回正——这是 Apollo 的实测可行方案。

**段2：竖直段（速度归零 + 姿态回正）**

```
// gate 条件：altitude < GATE_ALT(≈30m) 或 tgo < GATE_TGO(≈5–8s)
a_cmd_vert = (v_target_vert − v) / TAU_VERT          // 一阶消速度(P65 型)
a_cmd_vert = a_cmd_vert − (a_cmd_vert ⋅ horiz) * horiz  // 砍掉水平分量→推力渐近 up
a_cmd = a_cmd_vert + g_up                             // 重力补偿
// 姿态：steering 仍跟 a_cmd:direction；因 a_cmd 渐近 up，姿态自动回正
```
> 速度归零 ← 一阶律；**姿态回正 ← 推力方向渐近 up**（等价 Apollo P66 "thrust ≤ 20° from vertical"，笔记 `Klumpp1971…md:1092-1095`；也等价 SCvx/PythonRobotics 的"末端推力横向分量=0"）。

**段3：触地段（恒下降率 + 落点精修 + 姿态保持）**

```
a_cmd  = (vz_target − vz)/TAU_ROD * up              // 恒下降率(P66 ROD 型), vz_target=−2
       + KP_POS * r_err_horiz + KP_VEL * v_horiz     // 水平 P/PD 修落点
       + g_up
// 姿态：steering = up（独立姿态 P 律，小倾角下限幅推力方向 ≤5°）
```

### 4.3 末端姿态（倾角<5°）何时开始收敛、用什么律

**时机（gate）：**
- Apollo 证据：P66 典型在 **30 m 高度 / 11 m 地面距离**（笔记 `Klumpp1971…md:376-380`）；P64 redesignation 在触点前 **10 s** 截止、切入 P66（`:866-867`）；P64→P65 用 TTF 时间门（源码 `EXTLOGIC :692-709`）。
- 建议 kOS 取 **高度 gate ≈ 30–60 m 或 tgo gate ≈ 5–8 s**（取先到者），与 Apollo 的 30 m 一致并留裕度。

**用什么律收敛姿态：**
1. **首选（平滑）**：段1 四次律的 `a_f=up` 边界，使推力方向在接近 gate 时已渐近竖直，段2 只需"砍水平分量"即可把倾角压到 <5°。
2. **工程硬限幅（Apollo P66 实测）**：末段把指令推力方向限到 `≤5° from vertical`：
   ```
   thrust_dir = unit(a_cmd)
   tilt = arccos(thrust_dir ⋅ up)
   if tilt > 5° { thrust_dir = slerp(up, thrust_dir, 5°/tilt) }  // 限幅到 5°
   set steering to thrust_dir
   ```
   这是 Apollo P66 "limited to 20° from vertical"（笔记 `:1092`）的直接移植，把 20° 改成 5°。
3. **姿态 P 律（段3 辅助）**：触地段姿态与推力解耦，用独立姿态 P 律 `ω_cmd = −K_ATT·tilt_error_vector` 保持竖直（倾角角速度双环）。
> **推荐组合**：段1 四次律（a_f=up）让姿态"自然收敛"，段2 砍水平分量，段3 用"推力方向 ≤5° 限幅 + 姿态 P 律"做最后精修。三层都把"倾角<5°"翻译成"推力方向偏离 up <5°"，全部 O(1) 向量运算。

---

## 问题 5：不要盲信注释（源码/论文为准）

1. **变量名不符**：注释/文献里可能出现的 LRLAND/RLAND 并不存在；Apollo 着陆点向量实际名是 **`LAND`**（月固系）和 **`RLS`**（landing site，`THE_LUNAR_LANDING.agc:77-88`）。P63/P64 显示是 **V06N63/V06N64**（`:850, :865`），不是 V16N68（V16N68 的 DELTAH 初始化在 IGNALG 内 `THE_LUNAR_LANDING.agc:103-104`）。未发现名为 GASH 的符号。
2. **切换判据**：不能凭"接近段→竖直段应该按高度切换"的直觉；**源码 `EXTLOGIC :692-709` 用的是 TTF 时间门**（`TENDBRAK+TTF/8≤0`），不直接用高度或目标角。目标角 LOOKANGL 仅供显示（`:473-480`）。
3. **二次律"含加速度项"≠"约束姿态"**：QUADGUID 的 `+ADG` 项约束的是**终端加速度大小/方向的轨迹边界**（为 TTF 求根和弹道塑形服务），**不是对箭体姿态角的约束**；姿态要靠坐标系构造(CG)+EXNORM 混合+P66 限幅间接保证（§1.2）。文献 `文献资料汇总.md:37`、`2025_分岔法…md:47` 明确"二次律只闭合位置+速度"。
4. **G-FOLD 的 `cos(phii)`（LosslessConvexification `main.m:16 phii=27°`）只是发动机安装角的推力幅值缩放**，不构成推力方向锥；且该实例的 glide-slope 锥**被注释掉未启用**（`main.m:121-122`）。不能据此说"LosslessConvexification 实现了 glide slope"。
5. **SCvx 6DoF 的"末端姿态"是倾角锥 + 零角速度**（`solver.cpp:288-293`），**不是严格 q_final=[1,0,0,0] 等式**；竖直只出现在初值猜测（`:179`）。
6. **公式形式差异**：Klumpp 发布形式 `ACG=ADG+6(VDG+VG)/TTF+12(RDG−RG)/TTF²`（源码注释 `:534-537`）与源码实际编程形式 `(3/4)[(1/4)(RDG−RG)/(TTF/8)+VDG+VG]/(TTF/8)+ADG`（`:539-543`）代数等价但写法不同（用 TTF/8 缩放），引用时应注明是同一律。
7. **笔记优先级**：本报告所有 AGC 结论以 `Luminary099` 源码行为准；论文/笔记（Klumpp1971、各中文笔记）用于交叉印证机理，凡冲突处以源码行号为准。

---

## 关键证据索引（文件:行号）

**Apollo11 AGC（`...\Apollo11_AGC_source_master\Apollo-11-master\Luminary099\`）**
- 二次制导律公式注释：`LUNAR_LANDING_GUIDANCE_EQUATIONS.agc:533-544`；实现 QUADGUID `:546-627`（TTF/8 系数 `:581-599`）
- TTF 求根：`LUNAR_LANDING_GUIDANCE_EQUATIONS.agc:489-522`（TTF/8CL）
- 状态坐标变换 RGU/VGU：`LUNAR_LANDING_GUIDANCE_EQUATIONS.agc:410-480`（RGVGCALC）；CG 第一行=竖直 `:661-663`
- 阶段表 WCHPHASE：`LUNAR_LANDING_GUIDANCE_EQUATIONS.agc:40-44, 50-87`
- 切换门 EXTLOGIC：`LUNAR_LANDING_GUIDANCE_EQUATIONS.agc:692-709`
- P64 末段指令向量渐竖直 EXNORM：`LUNAR_LANDING_GUIDANCE_EQUATIONS.agc:748-773`（PROJMAX/MIN `:1455-1456`）
- P65 速度归零：`LUNAR_LANDING_GUIDANCE_EQUATIONS.agc:901-913`；V2FG/TAUVERT 定义 `ERASABLE_ASSIGNMENTS.agc:1370-1371`
- P66 ROD：`LUNAR_LANDING_GUIDANCE_EQUATIONS.agc:934-1098`；姿态保持 RATESTOP `:811-818`；GUILDEN `:130-216`
- FINDCDUW 推力方向→姿态：`FINDCDUW--GUIDAP_INTERFACE.agc:53-132`（UNITX `:108-111`，85° 限 `:723`）
- 着陆点 LAND：`THE_LUNAR_LANDING.agc:77-88`

**凸优化库（`...\90_代码与源码\`）**
- G-FOLD 3DoF 无姿态 / glide slope 位置锥 / thrust pointing 推力方向：`gfold_Cpp_G-FOLD_implementation\gfold-master\src\gfold.cpp:38,46-48,54,94-95,121-123`
- lcvx 无 6DoF / glide slope 位置锥 / 推力方向锥：`lcvx_dmalyuta_master\lcvx-master\ifac_wc_2020\landing.py:55,144,152-157`；`lib\tools.py:251-298`
- SCvx 6DoF 姿态倾角锥（全程+末端）：`RocketLanding_SCvx_6DoF_Mars_free_final_time_Cpp\RocketLanding-main\src\solver.cpp:288-293,298-301,321-322`；状态布局 `include\types.hpp:12,19-22`
- LosslessConvexification 无姿态 / glide slope 被注释：`LosslessConvexification_MatlabCVX_master\Lossless-convexification-master\main.m:25,96-105,121-122`
- PythonRobotics 6DoF SCvx 末端姿态全固定 + 全程倾角锥 + 需 ECOS 求解器：`90_代码与源码\PythonRobotics_rocket_powered_landing.py:26,35,63-66,289-290,296-305,539`

**文献笔记（`...\notes\` 与 `文献资料汇总.md`）**
- 二次律只闭合位置+速度、姿态是盲区：`文献资料汇总.md:37,52`；`2025_分岔法动力下降制导…md:47`
- Apollo 二次律形式与五次多项式：`Klumpp1971_Apollo登月舱…R-695原典.md:607-612,673-679`
- P66 30m/11m 起始、推力 ≤20° from vertical：`Klumpp1971…md:376-384,1092-1095`；P64 16° 接近角 `:1639-1644`
- 四次多项式嵌入末端姿态（a_f 边界）：`2025_重复使用火箭垂直着陆…md:103-106,610-656,317-321,341-347`
- 五次多项式嵌入初始姿态：`2024_航空学报_火箭着陆段…md:427-432,607-624,642-675`
- 金字塔形接近角约束（位置）：`2026_金字塔形接近角约束…md:53,193-196,291-293,368-381,606-611`
- PEG 不约束姿态、推力方向是输出：`2025_航天飞机PEG…AAS25-844.md:241-243`；`Jaggers1976_PEG推力积分推….md:294-298,488-499`
- 嫦娥3号专门过渡段做姿态回正：`2015_深空探测学报_阿波罗…md:225-236`
