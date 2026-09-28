# 新会话启动提示词 —— G-FOLD 一级回收问题

把下面整段（从分隔线内开始）作为新会话的第一条消息。

---

## 【任务】

分析并修复 KSP 火箭一级的 G-FOLD 自主着陆问题。**当前故障：一级不能跟踪规划轨迹。**

工作目录：`D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script`

---

## 【硬性要求】

1. **一切结论必须来自代码行号或实飞日志中的数字。** 不允许推测、不允许"我觉得可能是"。
   说不清楚就写 UNDETERMINED，并给出能判定的实验。
2. **优先搜索官方文档**（kRPC / kOS / KSP API），不要凭印象编造 API 名称与语义。
3. **改动前先验证**：能用离线仿真验证的，先在离线跑通再改代码。
4. **不要重写架构**。参考仓库 `dev/gfold/` 是本项目唯一验证过的 G-FOLD 实飞实现，
   凡是它有的，**逐行照抄**；只修改与载具实际数值相关的参数。
5. 改完跑 `pwsh -File tools/check_py.ps1`（6 项静态检查），全过再交付。
6. 控制台是 **GBK**：Python 打印只用 ASCII，不要用 → ⇒ ≈ ² 等符号。
7. **不要开子代理。**

---

## 【系统架构】

两级火箭。一级为可回收助推器（B1040-9），分两段控制，**两段用不同工具**：

| 段 | 工具 | 职责 |
|---|---|---|
| 段1 | **kOS**（`boot/B1040-9.ks`，~1570 行） | 高空 E-guidance，飞到**瞄准点**（目标点上方 3000 m），把位置与速度都收到接近 0 |
| 段2 | **kRPC**（`dev/krpc/gfold_land.py`，~2530 行） | 从瞄准点起，用 G-FOLD 凸优化（P3/P4 SOCP）落地 |

**交接机制**：kOS 每帧广播自己的状态；满足交班门后 kOS `unlock throttle`/`unlock steering`
并 `break` 退出；kRPC 侧检测到门后接管，**在交接点用真实状态重新解算**，然后跟踪。

**关键坐标约定**（目标参考系）：`x = 天顶（向上）`，`y = 北`，`z = 东`。
所有位置、速度、推力加速度向量都在这一个系里。

---

## 【关键常量】（`dev/krpc/gfold_land.py`）

```
TARGET_ALT      = 36.8      # 着陆腿触点高度（着陆点在地形上方的高度）
AIM_ALT         = 3000.0    # 瞄准点 = 目标点上方
GATE_AIM_H      = 150.0     # 交班门 = 瞄准点上方
GATE_ALT        = 36.8+3000+150 = 3186.8
GATE_VMAX       = 120.0     # 交班门速度上限
GATE_CONE_DEG   = 48.0      # 交班门锥角
K_POS = 0.5   K_VEL = 0.8   # 跟踪增益（来自参考 params.txt 的 k_x / k_v）
CONIC_TILT_DEG  = 25.0      # 末端姿态锥
FINAL_THROTTLE=0.8  FINAL_RADIUS=20  FINAL_HEIGHT=200
CTRL_X_ROT_KP=5  CTRL_X_ROT_KD=2.5（yaw 同）  CTRL_Y_AVEL_KP=2（roll）
```

**载具**（从 craft + 部件 cfg 实测）：
- 一级 9× `SSME`，`maxThrust=1000 kN` 每个 ⇒ 合计 **8.99 MN**，`minThrust = 0`（**可以关到 0**），
  `gimbalRange = 10.5°`，`Isp` 真空 315 / 海平面 295
- 交接质量约 **165 t** ⇒ `a_cap = T/m ≈ 54 m/s²`
- 姿态权限（用陀螺力矩估算）：`α ≈ 13.5 °/s²`，转 60° 约需 4.2 s

---

## 【跟踪算法】（`track()`，`gfold_land.py:1153-1413`）

求解器给出 `x(6,N)` 计划状态、`u(3,N)` 计划**推力加速度**（不是油门）、`tf`、`N=80`。

每帧：

```python
# 1. 索引推进
n_i = max(n_i - game_dt*0.2*N/tf, find_nearest_index(x, error, vel))
n_i = max(0.0, n_i)

# 2. 采样（当前点 + 前瞻点）
x_i,  v_i,  u_i  = sample_index(x, u, n_i, tf, N)
_x_i_, v_i_, u_i_ = sample_index(x, u, n_i + step, tf, N)
step = min(1.5*N/tf, |vel|/50*N/tf)

# 3. 控制律（两行的位置项都用 x_i，这是原仓库设计）
target_a  = u_i  + (v_i  - vel)*K_VEL + (x_i - error)*K_POS   # 驱动【油门】
target_a_ = u_i_ + (v_i_ - vel)*K_VEL + (x_i - error)*K_POS   # 驱动【姿态】

# 4. 锥限幅
target_a  = conic_clamp(target_a,  min_mag, max_mag, tilt_now)
target_a_ = conic_clamp(target_a_, min_mag, max_mag, tilt_now)

# 5. 分派：油门用当前点，姿态用前瞻点
throttle         = |target_a| / a_cap
target_direction = target_a_ / |target_a_|

# 6. 姿态 PID，直接写 vessel.control.pitch/yaw/roll

# 7. 写执行器
```

`conic_clamp` 的语义（照抄原仓库 `demo3_gfold.py:292-314`）：

```python
a_ver_min = sqrt(min_mag² - a_hor²) 或 cos(tilt)*min_mag
a_ver_max = sqrt(max_mag² - a_hor²) 或 cos(tilt)*max_mag
a_ver = clamp(a_ver, a_ver_max, a_ver_min)   # 竖直被【抬到】正的下限
a_hor = min(a_hor, a_ver*tan(tilt))
```

其中 `min_mag = 0.05*a_cap`、`max_mag = 1.00*a_cap`（对应参考 `throttle_limit_ctrl=[0.05,1.0]`），
`tilt_now = min(max(计划当前节点倾角 + 8°, 25°), 89°)` —— **锥角跟随计划自己的倾角**。

**控制律的性格**：前馈 `u_i` 决定油门主量，反馈项只做修正；
**姿态由 `target_a_` 决定 ⇒ 姿态跟不上，推力就指向错的方向。**

---

## 【求解器】

`dev/gfold/solver/gfold_p3p4.py`（cvxpy 路径）与 `dev/gfold/solver/gfold_codegen.py`
（codegen 路径，**两份问题定义必须保持同步**）实现 G-FOLD 的 P3/P4 SOCP。

- P3 = 最小落点误差（估 `tf`）；P4 = 最小燃料（出最终轨迹）
- 动力学为**梯形积分**：`v[n+1]=v[n]+dt/2*((u_n+g)+(u_{n+1}+g))`，`x[n+1]=x[n]+dt/2*(v[n]+v[n+1])`
- 求解器节流界 `throttle=(0.1, 0.8)`；控制器节流界 `[0.05, 1.0]`（**两套并存是原仓库设计**）
- codegen 走 C 编译：`pwsh -File tools/gen_codegen.ps1` 重新生成
  `cpg_p3/*.pyd`、`cpg_p4/*.pyd`。**约束在生成时固化**，改了 `gfold_codegen.py` 必须重新生成，
  否则 `.pyd` 里还是旧约束。实测 codegen 0.1 s vs cvxpy 2~3.6 s
- 参考仓库的原始问题定义在 `dev/gfold/GFOLD_direct_exec.py`（**权威对照物**）

---

## 【当前故障与最新数据】

**最新日志**：`log-B1040-9-gfold/gfold_log_20260927_181030.csv`（149 帧，41 列）

交接条件（日志注释）：
```
handoff solve at gate: alt=3180.5 vz=-86.7 vh=47.4 mass=165.4t
first solve OK tf=24.5 solve_ms=130        # codegen，交接点直接重解
LANDED alt=36.28 vz=-53.28 vh=29.16 dist=374.59 tilt=66.10
```

**现象**：

| 观察项 | 数值 |
|---|---|
| `thr_cmd` | **116/149 帧恰好等于 0.05**（= `min_mag/a_cap`，即指令幅值下限） |
| `a_cmd_h` | 147/149 帧 > 0（**水平指令是有输出的**） |
| `a_cmd_mag` | 这些帧恰好等于 `min_mag ≈ 2.65`，且 `hypot(a_up,a_h)` 恒等于它 |
| `att_err` | 5.6° ~ **79.1°**，全程不收敛 |
| `trk_pos_up`（竖直跟踪误差） | 最大 **27.7 m**（很小） |
| `trk_pos_h`（水平跟踪误差） | 最大 **400.7 m**（很大） |
| `vh` 实际 vs 计划 | 计划 20 帧内降到 9.5，实际只降到 31.8；计划后期 `plan_vh=0`，实际仍 15~18 |

**已确定的因果链**：

1. 计划要求大倾角制动（`tilt_cmd` 达 89°），而载具实际姿态只有 18~20°
   ⇒ `att_err` 长期 57~79°
2. 姿态滞后 ⇒ 推力方向几乎竖直 ⇒ **水平制动失效**（`trk_pos_h` 涨到 400 m）
3. 但竖直减速过度 ⇒ 载具比计划**下降慢** ⇒ `vterm = (plan_vz - vz)*0.8` 长期 **−10~−15**
4. `vterm` 把 `target_a` 的竖直分量压成负数 ⇒ `a_ver < 0`
5. `conic_clamp` 把竖直抬到 `a_ver_min`，水平也随之被削 ⇒ 幅值不足 `min_mag`
6. 触发**幅值下限**，等比放大到 `min_mag` ⇒ `throttle = 0.05`

**⇒ 油门小是【结果】不是【原因】；根因在【姿态跟不上计划】。**

**注意**：帧 0–1 是 `a_cmd_up=15.24, a_cmd_h=0`（纯竖直），说明求解器的
「起点推力竖直」约束工作正常；**从帧 2 起计划就切到 73~89°**。

---

## 【待判定问题 —— 这是你要重点做的】

**为什么姿态跟不上？** 三种可能，需要用数据区分：

| 可能 | 支持证据 | 反证 |
|---|---|---|
| **A. 计划倾角要求变化太快** | 交接姿态 ~20°，计划帧 2 起要 73~89° | 节点 0 已是竖直，且载具确实在转 |
| **B. 姿态权限不足** | 实测倾角变化率仅 20~30°/s | `tilt_act` 曾从 18° 涨到 93°，说明有权限，只是慢 |
| **C. 控制律/轴映射问题** | `att_err` 不收敛 | 代码与 `params.txt` 逐行一致 |

**建议的第一步（零风险，纯读日志）**：
用 `tilt_act` 时间序列反算载具的**实际最大角加速度**，与理论值 13.5 °/s² 对照，
判断 B 是否成立。日志里 `tilt_act`、`att_err`、`tilt_dir_cmd`、`dir_vx/vy/vz`、
`tgt_vx/vy/vz` 都是实测/命令向量，可直接用。

---

## 【可用的离线工具】

| 用途 | 命令 |
|---|---|
| 全量静态检查（6 项） | `pwsh -File tools/check_py.ps1` |
| 重新生成 codegen | `pwsh -File tools/gen_codegen.ps1` |
| 解一个状态、看计划形状 | `dev/krpc/diag_plan_shape.py` |
| 对比 `conic_clamp` 与原仓库 | `dev/krpc/diag_clamp_reference_vs_ours.py` |
| 验证 clamp 修复 | `dev/krpc/verify_conic_clamp_fix.py` |
| 约束集三方对比 | `dev/krpc/diag_constraint_diff.py` |
| 日志油门分析 | `dev/krpc/diag_throttle_small.py` |
| 姿态响应分析 | `dev/krpc/diag_attitude_response.py` |
| 跟踪误差分析 | `dev/krpc/diag_track_error.py` |

调用 Python 需要环境变量：
```powershell
$env:PYTHONPATH="tools\stubs"; $env:PYTHONJULIAPKG_OFFLINE="yes"
```
（`tools/stubs/pdaqp.py` 是空壳，用于绕开 cvxpygen 导入时联网下载 Julia 的挂起问题。）

---

## 【参考仓库文件对照】（`dev/gfold/`）

| 文件 | 用途 |
|---|---|
| `demo3_gfold.py` | **实飞控制器**，本项目 `gfold_land.py` 的移植来源（485 行） |
| `GFOLD_direct_exec.py` | **原始 SOCP 问题定义**，逐行对照的权威（154 行） |
| `params.txt` | 全部参数（增益、锥角、节流界、final 段参数） |
| `GFOLD_run.py` / `GFOLD_codegen.py` | 求解器封装 |
| `solver/gfold_p3p4.py` | 本项目的求解器（已含项目改动） |

---

## 【日志格式】（41 列）

```
t, wall, alt, dist_hz, vz, vh, vmag, mass,
thr_cmd, thr_act, a_cmd_up, a_cmd_h, a_cmd_mag,
tilt_cmd, tilt_dir_cmd, tilt_act, att_err, gnc_phase, replan_n,
solve_ms, gfold_status, tf, tgo, h_min_plan, x_err, v_err,
dir_vx, dir_vy, dir_vz,          # 载具机头真实方向（surface 系三分量）
tgt_vx, tgt_vy, tgt_vz,          # 姿态目标方向（surface 系三分量）
n_i, plan_alt, plan_vz, plan_vh, # 参考轨迹采样点的计划状态
trk_pos, trk_pos_up, trk_pos_h, trk_vel,  # 真实跟踪误差
note
```

- 前 26 列中：`tilt_cmd` 是**油门向量**的倾角；`tilt_dir_cmd` 是**姿态目标**的倾角
  （两者是不同向量，别混）；`x_err`/`v_err` 是**载具自身位置/速度幅值**，**不是跟踪误差**
- 后 14 列是真正的跟踪数据，`trk_pos_up < 0` 表示载具低于计划
- 日志前若干行以 `#` 开头，是注释/元信息；表头是第一条非 `#` 行

---

## 【已完成且验证的修复】（供你了解起点，不必重做）

1. **交接点直接重解**，删除了预热机制（预热轨迹起点与交接点差 112 m / 24 m/s，导致完全不跟踪）
2. **求解器补回原仓库的「起点推力竖直」约束**（`GFOLD_direct_exec.py:59`），
   `gfold_p3p4.py` 与 `gfold_codegen.py` 两处同步，`.pyd` 已重新生成。
   效果：节点 0 倾角 85° → 0°，代价仅 0.7 t
3. **`conic_clamp` 恢复原仓库语义**（负竖直「抬到下限」而非「归零」），
   修复前有 76% 帧水平指令被清零
4. **`final` 段补回锥限幅**（原仓库 `demo3_gfold.py:427`）

**以上修复已在最新日志中确认生效**：`a_cmd_h` 由 0 帧变为 147/149 帧非零。

---

## 【已知但未解决的问题】

- `thr_cmd` 与 `a_cmd_mag` 存在 ~4% 系统性偏差（`a_cmd_mag/(thr_cmd·T/m) ≈ 0.94`，
  反推 `T ≈ 8.6e6` 而配置为 `8.99e6`）。已排除 clamp 顺序与采样错位。
  **建议**：在日志加一列同帧的 `self.v.max_thrust`，下次实飞即可判定。
- 原仓库 `GFOLD_direct_exec.py:78` 有一条**推力变化率约束**（原版自己注释掉了），
  本项目未实现。它是唯一能阻止「bang-bang 计划」的约束，是否启用待定。

---

## 【交付要求】

- 每个结论附**行号或日志数字**。
- 改动前说明**为什么这样改**，并给出**可验证的预期**。
- 完成后报告：改了什么文件、验证结果、下次实飞该看哪几列。
