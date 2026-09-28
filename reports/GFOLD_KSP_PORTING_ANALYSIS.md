# GFOLD_KSP 移植分析（针对 B1040-9）

日期：2026-09-24。来源：[xdedss/GFOLD_KSP](https://github.com/xdedss/GFOLD_KSP)（全部源码已逐行读过）、
[kRPC 官方文档](https://krpc.github.io/krpc/)、本机载具数据见 [VEHICLE_B1040-9_ANALYSIS.md](VEHICLE_B1040-9_ANALYSIS.md)。

## 1. GFOLD_KSP 是什么

把 G-FOLD（燃料最优动力下降凸优化，Açıkmeşe）跑在**本机 Python** 里，
通过 kRPC 实时控制 KSP 里的火箭：求解器给出整条最优轨迹 (x,u)，
控制循环沿轨迹做"最近点跟踪 + PD 修正"，轨迹随飞随重解（MPC 式）。

### 源码结构（5 个文件的分工）
| 文件 | 作用 |
|---|---|
| `GFOLD_direct_exec.py` | cvxpy 建模的 G-FOLD：P3（最小落点误差+估 tf）→ P4（燃料最优）。N3=160 / N4=80 节点 |
| `GFOLD_codegen.py` + `postprocess.py` + `build.bat` | 可选：用 cvxpy_codegen 把求解器编成 C 包（gfold_solver_p3/p4），提速 ~10× |
| `GFOLD_run.py` | solver 封装类（pack_data → 调 codegen 版或直接 cvxpy 版） |
| `demo3_gfold.py` | 主程序：krpc 连接 + 控制循环（轨迹跟踪 PD、conic_clamp、末段 PID、姿态 PID） |
| `params.txt` | 全部可调参数 |

### 求解的数学问题（GFOLD_direct_exec.py 实际建模）
- 状态 x=(r,v)，控制 u=推力加速度，z=ln(m)，s=推力幅值松弛变量；
- 约束：初值/末值（末速度=0、末位置=原点（P4））、分段线性动力学、**滑翔角锥**（横向 ≤ alt·cot γ）、
  速度上限 V_max、质量消耗、推力上下限（对数凸化）、**推力指向锥**（u_up ≥ cos(p_cs)·|u|）、
  末端推力方向竖直（u= s·[1,0,0]，即姿态约束——正是我们要的"触地倾角≈0"）；
- P3 目标：最小化 Σ norm(x)·i/N（早到目标附近）→ 顺便估出最优 tf；
- P4 目标：最大化落地质量（−z(N)·N）+ straight_fac 加权横向位置（末段轨迹拉直）。
- 求解器：cvxpy + ECOS。

### 控制循环（demo3_gfold.py）
- 坐标系：`ref_target = create_hybrid(position=目标点@天体坐标, rotation=surface系, velocity=目标点自转速度)`
  —— 目标点静止、x 轴朝天，与 G-FOLD 的 g=[−g,0,0] 约定一致；
- 触发：alt < start_altitude 后**每个物理帧都重解**（MPC 重规划）；
- 跟踪：`find_nearest_index` 找轨迹最近点 → 前视采样 → `a_cmd = u_i + k_v·(v_i−v) + k_x·(x_i−r)`；
- `conic_clamp`：把指令加速度钳进"幅值 [thr_min,thr_max]·max_thrust/m + 倾角 ≤ max_tilt"的锥；
- 末段（final）：距目标 <final_radius 且高 <final_height 时切纯 PID（自杀燃烧节流插值 + 弱横向 PD）；
- 姿态：把目标推力方向变换到机体系，`angle_around_axis` 求俯仰/偏航角误差 → PID → `control.pitch/yaw/roll`（滚转只消角速度）；
- 终止：位置/速度/角速度全小，或已过目标开始上升。

## 2. 需要的工具链

| 组件 | 版本要求 | 备注 |
|---|---|---|
| kRPC | 最新即可 | 你已装。Python 端 `pip install krpc` |
| Python | 3.6–3.8（原作者建议 3.6） | **因为 cvxpy 0.4.11 太老** |
| cvxpy | **0.4.11**（配合 cvxpy_codegen） | 老版本，现代 Python 装不动 |
| ECOS | 随 cvxpy 0.4 | P3/P4 的 SOCP 求解器 |
| scipy 1.2.1 / CVXcanon 0.1.0 / setuptools ≤57.5.0 | 钉死 | 老依赖 |
| cvxpy_codegen | moehle 版 | 已弃坑，需删 setup.py 里一行才能装 |

**版本建议（重要）**：不要复刻老工具链。cvxpy 0.4.11 的建模代码与 1.x **几乎兼容**
（`Variable(6,N)`、`norm`、`==` 约束、`Minimize`、`problem.solve()` 都没变），
把 `solver=ECOS` 换成 `cp.CLARABEL` 或 `cp.SCS`（现代 cvxpy 已移除/弃用 ECOS），
即可在**现代 Python（3.10+）+ cvxpy 1.5+** 上跑 `direct` 模式。
先跑通 direct（慢 10× 但免编译）；确有必要再考虑 C 化（cvxpy_codegen 已死，
现代替代是自写 P3/P4 的专用求解器或用 [cvxpygen](https://github.com/cvxgrp/cvxpygen)）。

## 3. 需要的 kRPC API（对照 demo3 逐条核实）

| 用途 | API |
|---|---|
| 连接 | `krpc.connect(name=...)`、`conn.space_center` |
| 目标系 | `sc.ReferenceFrame.create_relative(...)` + `create_hybrid(position, rotation, velocity)` |
| 状态 | `vessel.position(ref)`、`vessel.velocity(ref)`、`vessel.rotation(ref_srf)`、`vessel.angular_velocity(ref_srf)`、`vessel.mass`、`vessel.moment_of_inertia` |
| 发动机 | `vessel.max_thrust`、`vessel.specific_impulse`、`vessel.available_thrust` |
| 天体 | `vessel.orbit.body`、`body.surface_height(lat,lon)`、`body.equatorial_radius` |
| 控制 | `vessel.control.throttle / pitch / yaw / roll / gear` |
| 调试画线 | `conn.drawing.add_line`（可选） |
| 物理帧同步 | 比较 `space_center.ut` 增量 ≥ 0.01（对应 KSP 物理帧） |

文档：[Vessel API](https://krpc.github.io/krpc/python/api/space-center/vessel.html)、
[Reference frames](https://krpc.github.io/krpc/python/api/space-center/reference-frames.html)、
[Control](https://krpc.github.io/krpc/python/api/space-center/control.html)。

## 4. 针对 B1040-9 需要的调整（params.txt 逐项）

| 参数 | 原值（小着陆器） | B1040-9 应改为 | 依据 |
|---|---|---|---|
| `start_altitude` | 1100 m | **≥13000**（我们的点火高度 ~12 km） | 日志 47822691：alt 12357 点火 |
| `tf` | 20 s | **45–55**（先试 50；P3 会自己修正最优 tf） | 12 km / ~300 m/s 均值 ≈ 40 s；tf 太小会 infeasible |
| `V_max` | 150 m/s | **1000** | 点火 |v|≈975 m/s |
| `G_max` | 10 g | 10 保持（a_net 峰值 ~90 m/s²≈9 g） | 日志 lat_net |
| `y_gs` | 30° | 20–30° 试 | 滑翔角锥，我们的下降剖面较陡 |
| `max_tilt` | 25° | 25–45° | 主段需要 ~50° 倾斜刹车；先 45 |
| `throttle_limit` | [0.1, 0.8] | **[0.0, 1.0]**（规划用全范围；Vector 可到 0） | 150% 限幅下的真实节流范围 |
| `throttle_limit_ctrl` | [0.05, 1.0] | [0.0, 1.0] | 同上 |
| `target_lat/lon/height` | 原目标 | **−0.0972060952 / −74.5576822740 / 36.8+地形高** | 我们的着陆点；注意 height = target_alt + body.surface_height |
| `final_height/radius` | 200 / 20 | 150 / 30（大火箭末段早一点） | 配合 §3 末段 |
| 姿态 PID | kp=5, kd=2.5 | **必须用 kRPC 的 `vessel.available_torque` 在线标定增益**（demo 里被注释掉的方案），不能照搬 | 我们 I≈1.8e4 kg·m²、主控是 gimbal（3.2 MN·m），增益差数量级 |
| `k_x / k_v` | 0.5 / 0.8 | 先保持，实测调 | 轨迹跟踪 PD |

### 结构性调整（比参数更重要）
1. **气动阻力是最大模型误差**。G-FOLD 无阻力项；我们在 12 km 以 ~Mach 2.5 下降，
   FAR 下阻力巨大（这也是实飞能 26 s 刹下来的原因）。处理方式：
   **早点开始解（start_altitude 给足）+ MPC 式逐帧重解**（demo 已这么做），
   把阻力当成"额外减速能力"，规划偏保守（把 T_max 打 8 折再喂给求解器，即 `T_max *= 0.8`），
   让阻力成为正向裕量而不是缺失项。
2. **末段换成我们验证过的终端律**。demo 的 final 段横向 PD 增益太弱（0.03/0.06），
   大火箭会漂。建议 final 段直接移植我们 kOS 段3（恒下降率 + 落点 PD + 推力方向 ≤5°）。
3. **姿态控制**：demo 的自定义 PID 是为小火箭调的。B1040-9 惯量小、gimbal 力矩大（3.2 MN·m），
   直接用 kRPC 自带 `vessel.auto_pilot`（`target_direction` + `sas=False`）可能更稳，
   或用 demo 的 PID 但 kp/kd 按 `available_torque/I` 在线标定。RCS（0.55 MN·m）只做辅助。
4. **节流语义**：krpc 的 `vessel.max_thrust` 已含 thrustLimiter=150%（1.5 倍），
   **不需要** kOS 版那套 Engine:THRUST 反推修正——但要实测验证一次（打印 max_thrust 应 ≈13.5 MN）。
5. **起落架/事件**：`vessel.control.gear = True`（demo 已有），无 stage 需求（一级回收时二级已分离）。

## 5. 实施路线建议

1. 装现代 Python 环境：`pip install krpc cvxpy clarabel numpy`；
2. 克隆 GFOLD_KSP，`GFOLD_direct_exec.py` 里 `solver=ECOS` → `solver=cp.CLARABEL`，
   跑 `GFOLD_run.py direct` 的自测用例（能解出不 infeasible 即求解器 OK）；
3. 按第 4 节改 params.txt（先只改数值参数），跑 `GFOLD_run.py direct` 用**我们的点火帧**
   （x0 = 相对目标的 [alt≈12300, 水平位置, v]，Isp=315, m=225t→144t, T_max=12.8 MN×0.8）；
4. 先在 KSP 里做**开环验证**：只画线（debug_lines=True）不接管控制，看轨迹是否合理；
5. 再闭环飞；末段（final）换成我们的段3 律。

风险：逐帧重解的耗时（direct 模式 N4=80 在本机可能 0.5–2 s/次，KSP 里火箭 30 m/s² 下
1 s 差很多）；若太慢，第一优先级是把 N4 降到 40，或上 cvxpygen 编 C。
