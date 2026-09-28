# 交接机制与操作说明（kOS ⇄ kRPC/G-FOLD）

> 更新：2026-09-25
> 适用：`boot/B1040-9.ks`（段1，kOS） + `dev/krpc/gfold_land.py`（段2，kRPC + G-FOLD）

---

## 1. 交接机制

### 1.1 为什么要交接

**单一所有权**：任一时刻只有一个程序写同一执行器通道。
kOS 的 `lock throttle` / `lock steering` 是"每物理帧重算"的最高优先级触发器，
如果 kOS 继续跑主循环，会每帧覆盖 kRPC 写的 `control.throttle` / `auto_pilot`，
两边互相打架。**所以交班后 kOS 必须停止写执行器。**

### 1.2 分工

| 段 | 高度 | 工具 | 职责 |
|---|---|---|---|
| 段1 | 12.4 km → ~5 km | **kOS** | 两级律把火箭带进 G-FOLD 可行域 |
| 段2 | ~5 km → 地面 | **kRPC + G-FOLD** | 燃料最优 + 精确落点 + 末端姿态 |

可行域由真实日志离线扫定：**alt 358 ~ 5950 m**（详见
`dev/reports/EGUIDANCE_GFOLD_HYBRID_REPORT.md` §4）。

### 1.3 交班门（四条件同时满足）

在 `boot/B1040-9.ks` 参数区（`gfold_*`）：

| 条件 | 参数 | 值 | 理由 |
|---|---|---|---|
| ① 高度够低 | `gfold_h` | 5000 m | 可行域上沿 5950，留余量 |
| ② 速度够小 | `gfold_vmax` | 480 m/s | 超出则先在段1 继续减速 |
| ③ 在锥内 | `gfold_cone` | 48° | G-FOLD 滑翔角锥要求 `dist ≤ h/tan γ` |
| ④ 高于地面 | `p66_h` | 30 m | 不能贴地才交班 |

MATLAB 闭环仿真实测：**该门在 t=36.7 s 满足，alt=5036 / dist=142 / |v|=75 m/s**。
该状态（真实质量 152 t）喂 G-FOLD 已验证：
`optimal`，**末端倾角 0.00°，末端位置/速度误差 0.0000**。

### 1.4 交班时的动作（kOS 侧，2026-09-26 重构）

**交班是一次性事件：发信号 → 放权 → 结束脚本。**

```ks
set ctrl_main to false.        // ① 让 throttle_cmd() 不再走制导分支
set throttle_demand to 0.
set thr_cmd to 0.
set steer_mode to 0.
unlock throttle.               // ② 【必须】撤掉 lock，否则每帧覆盖 kRPC
unlock steering.
set ship:control:pilotmainthrottle to 0.
print "GFOLD_RELEASED kOS 已放权，控制权移交 kRPC，脚本结束".
LOG "GFOLD_RELEASED t=..." TO "0:/gfold_state.txt".
set phase to 3.
break.                         // ③ 跳出主循环，脚本到此为止
```

**为什么旧版是错的**：旧版只清零 `throttle_demand`，但 `ctrl_main` 仍为
`true`，于是 `throttle_cmd()` 返回的是 **kOS 自己算的节流**
（`t_vec:mag / avail_thrust()`），清零值从未生效。而 kOS 的 `lock` 每物理帧
重算、优先级**高于** kRPC 对 `control.throttle` 的写入 ⇒ kOS 每帧覆盖 kRPC。

**实飞日志铁证**（`gfold_log_20260926_105737.csv`）：

| 量 | 值 |
|---|---|
| `thr_cmd`（kRPC 写的） | 恒 **1.0**（285/306 行） |
| `thr_act`（实际执行） | **0 ~ 0.5** |

⇒ **kRPC 从未真正接管节流**。仅清零内部变量没有用，**必须 `unlock`**。

### 1.5 已删除的"超时回落"（及其原因）

旧版有 `gfold_wait = 8.0`：8 秒内 kRPC 没接管就解除武装、回落 kOS 自主控制。
**已删除**，原因：

1. 交班后 kOS 已 `unlock` 并结束脚本，**不存在"回落"这个状态**（死代码）；
2. 那套超时在实飞里**从未真正生效过**（`throttle_cmd` 走的是制导分支，
   不是被清零的 `throttle_demand`），保留只会误导；
3. 保留 `ctrl_main=true` 等残留状态，**正是导致 kOS 覆盖 kRPC 的原因**。

> **现在明确接受的代价**：若 kRPC 没接管（没开服务器 / 程序没跑），
> 火箭会自由落体。宁可**明确失败**，也不要两边抢执行权导致行为不可解释。
> **操作上必须先启动 kRPC 程序，再执行 kOS 脚本**（见 §2）。

### 1.6 kRPC 侧的识别与接管校验

`gfold_land.py` 启动后自己轮询状态，满足与 kOS **相同**的门限就接管。
两侧门限必须一致（改一边要改另一边）：

| 常量 | kOS 侧 | kRPC 侧 |
|---|---|---|
| 高度门 | `gfold_h = 5000` | `GATE_ALT = 5000.0` |
| 速度门 | `gfold_vmax = 480` | `GATE_VMAX = 480.0` |
| 锥角 | `gfold_cone = 48` | `GATE_CONE_DEG = 48.0` |
| 可行域下沿 | — | `GATE_ALT_MIN = 358.0` |

**接管校验 `verify_takeover()`**（新增）：接管后连续 10 帧比对
**自己写入的 `thr_cmd`** 与 **`vessel.control.throttle` 实际读数**：

- 不符 < 6 帧 → 判定接管成功，打印 `auto_pilot.engaged`
- 不符 ≥ 6 帧 → 判定**未接管**（多为 kOS 的 `lock` 仍未撤掉），
  **放弃控制并退出**，不再与 kOS 抢执行器

> kRPC 侧**不解析** kOS 的打印行，而是独立判断状态 —— 这样即使 kOS 的日志
> 缓冲没刷出来也能工作。`GFOLD_HANDOFF` / `GFOLD_RELEASED` 是给人看的记录。

---

## 2. 操作步骤

### 2.1 一次性准备（已完成，核对即可）

| 项 | 状态 |
|---|---|
| kRPC mod | `GameData\kRPC`（v0.6.0，匹配 KSP 1.12.5） |
| Python 客户端 | `krpc 0.6.0` → `D:\Applications\Python\Lib\site-packages\krpc` |
| protobuf | 7.36.2（满足 krpc 的 >=7.35.1） |
| 求解器依赖 | `cvxpy 1.9.3` + `clarabel 0.10.0` |
| kOS 脚本 | `boot/B1040-9.ks`（kos_lint **0 诊断**） |

### 2.2 你需要手动改的 kRPC 设置

文件：`GameData\kRPC\PluginData\settings.cfg`，**KSP 关闭时改**：

```cfg
autoStartServers = True       # 原 False：免去每次手动点 Start Server
autoAcceptConnections = True  # 原 False：免去 Python 连接时的弹窗确认
```

`rpc_port` / `stream_port` 未显式写出，走默认 **50000 / 50001**，**不用改**。

### 2.3 每次飞行的步骤

**1 · 启动 KSP 并载入存档**
改了 `autoStartServers=True` 后服务器自动启动；否则点游戏内 kRPC 图标 → Start Server，
确认显示 `Server running on 127.0.0.1:50000`。

**2 · 先跑 dry-run（强烈建议首飞这样做）**
```powershell
cd "D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script\dev\krpc"
python gfold_land.py --dry-run
```
它会连上 kRPC → 等待交班门（打印 `[wait]`）→ 门满足后解算 G-FOLD 并**画线**
（不接管控制）。确认轨迹合理后 Ctrl+C。

**3 · 起飞前启动正式接管**（⚠️ **必须在执行 kOS 脚本之前**）

> **顺序很重要**：交班后 kOS 会立即 `unlock` 并结束脚本，不再等待 kRPC。
> 若 kRPC 没在跑，火箭会从交班点自由落体。**先启动本程序，再切到游戏执行脚本。**

```powershell
python gfold_land.py
```

**4 · 在 KSP 里正常执行发射脚本**
按你的流程跑 `boot/B1040-9.ks`。kOS 飞段1，到交班门打印 `GFOLD_HANDOFF ...`
后停止写执行器；kRPC 侧接管飞完末段。

**5 · 看日志**
- 段1：`log-B1040-9/land_log_*.csv`（kOS）
- 段2：`log-B1040-9-gfold/gfold_log_*.csv`（本程序，列定义见该目录 README.md）

---

## 3. 参数速查

### 3.1 段1（`boot/B1040-9.ks`）

**段1 已改为 B1040-7 的成功结构（2026-09-26）**——这是本项目唯一实飞成功的一级
回收脚本同源结构，与我之前所有尝试有一个**根本差别：不分通道**。

| 项 | 我之前 | B1040-7（现用） |
|---|---|---|
| 制动段律 | 拆成 upv 竖直 + 横向，各用常数/sqrt 剖面 | **整个 3D 矢量一次算出** |
| 公式 | 多种尝试，均失败 | `a_cmd = r_tgt·(6/T²) − v_now·(4/T)` |
| 时标 | `solve_eg_T`（自杀剖面） | `T = 2·h_aim / max(sqrt(h_aim), \|vz\|)` |
| 超限处理 | 整体归一化（**会改方向，是败因**） | **推力分配竖直优先、水平吃余额** |

**自洽性（我独立复核过）**：`T = 2h/|vz|` 代入 ⇒ `a = 6h/T² − 4|vz|/T = vz²/(2h)`，
而 `vz²/(2h)` 正是"在 h 内把 vz 削到 0"所需减速度 ⇒ **指令 ≡ 需要，比值恒为 1**
（MATLAB 复核 h=10000/vz=−500：a=12.50，需求=12.50，比值 **1.000**）。
闭环下 `v²/h` 守恒 = 精确等减速轨迹，速度恰好在瞄准点归零。

**超限的验证**（点火帧实测 `|a_cmd|=117` vs `a_net=50`）：
竖直优先分配后，竖直需求 17.4 被**精确满足**，水平拿到 53.6（需 29.3）。

| 参数 | 值 | 说明 |
|---|---|---|
| `aim_alt` | **3000 m** | 瞄准点抬到目标点上方 3000 m（原 15 m） |
| `term_v` | 2.0 | 终端目标下降率 [m/s] |
| `term_h` / `gate_h` / `p66_h` | 180 / 30 | 段门（到**地面**高度） |
| `gamma_gs` | **48°** | 滑翔角锥（**从水平量起**，越大越紧） |
| `gfold_h`/`gfold_vmax`/`gfold_cone` | 5000 / 480 / 48 | 交班门 |
| `gfold_wait` | 8.0 | kRPC 接管超时 [s] |

> **`gamma_gs` 语义**：G-FOLD 约束是 `‖x_h‖ ≤ (1/tan γ)·x_v`，γ **从水平量起**，
> 所以 **γ 越大锥越陡越紧**。旧值 60° 只允许 `dist/alt ≤ 0.577`，而真实轨迹是
> 0.61~0.91 ⇒ 比真实还紧，会全程触发回锥偏置。已改 48°。

### 3.2 段2（`dev/krpc/gfold_land.py`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `--dt` | 0.02 | 循环周期 [s] |
| `--replan-dt` | 1.0 | 重解周期 [s]（**不能逐帧重解**，见下） |
| `--dry-run` | 关 | 只解算+画线，不接管 |
| `--no-autopilot` | 关 | 不控姿态 |
| `--gate-timeout` | 600 | 等交班门超时 [s] |
| `SOLVER['N']` | 40 | 节点数（耗时 ~0.29 s） |
| `SOLVER['pcs_end_deg']` | 3.0 | 末端推力指向锥（<5° 指标留余量） |
| `SOLVER['v_descent_max']` | 160 | 下降率上限 [m/s]（**必须**，见 §4） |
| `SOLVER['term_win']` | 0.40 | 末端收紧窗口（占 tf 比例） |
| `SOLVER['term_vz']` | 4.0 | 末端目标下降率 [m/s] |

**为什么不能逐物理帧重解**：实测单次求解 0.15~0.44 s，而 KSP 物理帧 0.02 s
（见 `dev/gfold/analysis/timing_check.py`）。所以采用三段时间管理：
① 交班时解一次（tf 自动搜索，~5 s）→ ② 每 `replan_dt` 秒用剩余时间重解
（单次 ~0.29 s）→ ③ 帧间沿参考轨迹 PD 跟踪。

---

## 4. 离线闭环验证结果（`verify_control_loop.py`）

### 4.1 理想交班点（早期仿真值）

输入 alt=5036 / dist=142 / vz=−72 / vh=20.5 / mass=152 t：

| 判据 | 结果 |
|---|---|
| 落地垂速 \|vz\| < 8 m/s | **6.42** ✓ |
| 落地横速 vh < 5 m/s | **0.00** ✓ |
| 落点偏差 < 10 m | **0.00** ✓ |
| 触地倾角 < 5° | **0.00** ✓ |
| 重解耗时中位 < 500 ms | **290 ms** ✓ |

### 4.2 实飞真实交班点（`--real`，2026-09-26 实飞日志）

**这才是复现实飞失败的工况**：横向速度远未消掉。

输入 alt=5031.8 / dist=1285 / \|v\|=478.8（**vz=−314, vh=361.5**）/
mass=182.8 t / T_MAX=8.99 MN：

| 判据 | 结果 |
|---|---|
| 首次解算 | **optimal**，tf=22.2 s，落地 141.5 t ✓ |
| 参考轨迹推力越界比 | **1.0000**（物理可实现）✓ |
| 落地垂速 \|vz\| < 8 m/s | **1.87** ✓ |
| 落地横速 vh < 5 m/s | **0.00** ✓ |
| 落点偏差 < 10 m | **0.00** ✓ |
| 触地倾角 < 5° | **0.00** ✓ |
| 重解耗时中位 < 1000 ms | **489 ms** ✓ |

**结论：全部通过。** 修复（坑⑯~⑲）后，实飞真实交班点从
`infeasible` 变为可解、可跟踪、并落到 −1.87 m/s（指标要求 ≈ −2）。

> **注意**：4.1 的横速只有 20.5，4.2 是 361.5 —— 差 17 倍。
> 只跑 4.1 会得出"一切正常"的错误信心。**每次改求解器都要跑 `--real`。**

### 4.3 参考系口径回归（`verify_frames.py`，2026-09-26 实飞事故后新增）

用实飞日志做**独立口径交叉验证**：拿**位置位移**（不受速度参考系影响）
反推真实速率，去校验日志里的速度分量。

| 分量 | 位移反推 | 日志 | 差 |
|---|---|---|---|
| 水平 vh | 3.46 | 176.32 | **172.86** |
| 竖直 vz | −0.50 | 0.72 | 1.22 |

**差 172.9 ≈ 自转标尺 174.9 m/s（差 1%）** ⇒ 水平速度被天体自转污染的
特征签名（坑⑳）。竖直则吻合 ⇒ 证明**位置口径正确，只有速度多了自转分量**。

该测试在**修复前的日志上会 FAIL**（这正是它存在的意义），修复后应 PASS。
**判据**：② 水平速度与位移反推之差**不得接近**自转速度。

---

## 5. 故障排查

| 现象 | 原因 | 处理 |
|---|---|---|
| `ModuleNotFoundError: krpc` | 客户端没装 | `python -m pip install krpc` |
| 连接超时/拒绝 | 服务器没开 | 游戏内点 Start Server |
| 连接后卡住无输出 | `autoAcceptConnections=False` | 改配置或在游戏内点同意 |
| **`mass/thrust` 明显不对**（如 761 t / 8.5 MN） | **选错了 vessel**（一级分离后 `active_vessel` 变成二级） | 已修：按 `--vessel-name` 找；见下 |
| `首次解算失败: infeasible` + `a_net too low` | 同上（选错船，推重比≈0） | 已加预检诊断，会打印 vessel 列表 |
| `首次解算失败: infeasible`（`a_net` 正常） | 初值不在可行域 | 检查交班门是否与 kOS 一致 |
| kOS 打印 `GFOLD_ABORT` | 8 s 内 kRPC 没接管 | 确认 `gfold_land.py` 在运行且已连上 |
| 求解慢（>2 s） | N 太大 | 降 `SOLVER['N']`（40→25） |

### 5.1 vessel 选择（2026-09-25/26 三次实飞踩坑，已按【备用方案】解决）

**三次失败模式**：

| 趟次 | kRPC 拿到 | kOS 实际 | 后果 |
|---|---|---|---|
| 23:11 | 761.6 t / 8459 kN | 224.6 t | `a_net=1.3` ⇒ infeasible |
| 23:44 | 759.2 t / 8459 kN | 224.5 t | 同上 |
| 00:04 | 726.0 t / 8481 kN | 224.4 t | 同上 |

**根因（按发现顺序）**：
1. `active_vessel` 在一级分离后变成二级；
2. 按名字也不行 —— 场景里 `B1040-9`、`B1040-9 中继卫星`、`B1040-9的残骸`
   名字互相包含，且 **KSP 让二级/载荷继承原 vessel id**，真正的回收一级是
   **新出现的 vessel**，程序从锁定那一刻起再没看过它；
3. kRPC 常在火箭**还没分离**时启动，那一刻选到的只是"整箭"（726 t）。

**已实现的【主路径】：kOS 广播位置，kRPC 按位置+质量锁定**

kOS 侧（`boot/B1040-9.ks`）在交班时打印并落盘一行固定格式：

```
GFOLD_HANDOFF lat=.. lon=.. alt=.. t=.. dist=.. v=.. vz=.. vh=.. mass=.. type=.. name=..
```

```ks
set gfold_msg to "GFOLD_HANDOFF" + " lat=" + ... + " name=" + ship:name.
print gfold_msg.
LOG gfold_msg TO "0:/gfold_state.txt".     // 落盘，kRPC 直接读文件
```

kRPC 侧（`dev/krpc/gfold_land.py`）：
- `find_handoff_line()` 读 `<KSP>/Ships/Script/gfold_state.txt`（kOS 的 `0:` 卷）；
- `match_by_handoff()` 用**两个物理量**打分选船：
  - **位置**：三维距离（经纬度大圆距离 + 高度差）< 3 km
  - **质量**：候选质量应在 `[广播质量−5%, 广播质量+35%]`（下落中持续烧油）
- 两个条件都满足才选中，按 `距离 + 0.1×|质量差|` 打分取最小。

**不要用 `type` 判据（两次踩坑的最终结论）**

| 尝试 | 结果 |
|---|---|
| 只排除 `debris` | 选中了在轨 `relay` 卫星（761 t / 250 t） |
| 加"排除 `relay`" | **把真正的一级也排除了** —— 实飞证据：kOS 交班行 `type=Relay name=B1040-9 mass=192.7 t`，kRPC 候选 `type=relay mass=185.7 t`，质量差 7 t、高度差 685 m，正是下落中烧油 |

**玩家在 VAB 里能把一级的 vessel type 设成 Relay**，所以 `type` 完全不可信。
判据只用物理量：质量 > 20 t、高度 100~22000 m、正在下降、`a_net ≤ 80`。

**诊断**：`python gfold_land.py --list-vessels` 列出全部船并标出 ★ 会被选中的那个，
未选中的会打印**排除原因**。



---

## 6. 开发中踩过的坑（避免重复）

| # | 坑 | 表现 | 修法 |
|---|---|---|---|
| ① | **推力指向锥全程固定** | `p_cs=20~25°` 全程施加 ⇒ 横向制动被禁 ⇒ 全 infeasible | 按时序从宽收到窄 |
| ② | **`z0` 质量参考用满推力** | tf 末算到 48 t（真实 ~140 t）⇒ **tf 越长越 infeasible**（反物理） | 用参考减速率估质量 |
| ③ | **`v_max=400` < 真实 \|v\|** | node 0 同时要等于初值又要 ≤v_max，自相矛盾 | 放宽到 1200 |
| ④ | **缺"不许穿地/爬升"** | 解出"先落地→飞起 157 m→再落地" | 加 `x[0,n]≥0` + `x[3,n]≤0` |
| ⑤ | **`tf` 是硬输入** | 给长了会在触地后贴地抖动 10 s | tf 自动搜索（二分最小可行值） |
| ⑥ | **纯燃料最优 = bang-bang** | 自由落体 24 s + 最后 5 s 以 71 m/s² 猛刹 ⇒ **不可跟踪**，触地 vz=−96.9 | 加 `v_descent_max` 包络 + 末端窗口 |
| ⑦ | **`u` 语义搞错** | G-FOLD 的 `u` 是【推力加速度】，又加了一次 g | 净加速度 = `u + g`，不要重复补 |
| ⑧ | **最近点索引跟踪** | 参考点滑到本机前方 ⇒ P 项命令【向下】追 ⇒ vz 涨到 −260 | 改**时间索引** `k=t/dt_plan` |
| ⑨ | **重解用 0.8×首次 tf** | 加了包络后最小可行 tf 变长 ⇒ 重解全失败（replan=0） | 用剩余时间 + 裕度 |
| ⑩ | **用 `active_vessel`** | 一级分离后指向二级 ⇒ 质量 726~761 t、`a_net≈2` ⇒ infeasible | 按交班点匹配（§5.1） |
| ⑪ | **`replan_dt` 太短** | 单次求解 0.4~0.7 s，`replan_dt=1.0` 时占 42~72% 周期 ⇒ 跟踪帧被饿死 | 默认改 2.0 s |
| ⑫ | **kOS `ship:id` 不存在** | `GET Suffix 'ID' not found on object VESSEL(...)` ⇒ **当场终止整个脚本**（kOS 无 try/catch），交班瞬间脚本死掉 | 已在发射台自检里预拼一次交班行 |
| ⑬ | **用 `type` 选船** | 先漏掉 relay 选中在轨卫星；加"排除 relay"又把**真正的一级**（玩家把它设成 Relay 型）排除了 | 判据只用物理量：质量/高度/下降，不用 type |
| ⑭ | **终端段 `tp` 符号** | kRPC 的 `tp = 本机−目标`，而 B1040-7 的 `r_tgt = 目标−本机`；漏负号 ⇒ 位置项**把火箭推离目标**，d 从 14.9 涨到 36.8 | 水平位置项用 `-tp` |
| ⑮ | **擅自改 B1040-7 的 `term_h`** | 从 15 改成 80/400/25/150，**越改越差** | 照抄 15。终端段只处理"最后几米残余"，横向收拢由上游负责 |
| ⑯ | **`v_descent_max` 包络与初速自相矛盾** | 实飞交班 \|vz\|=314，而包络上限 min(160, …)=160 ⇒ node 0 同时要 `vz=-314`（初值）和 `vz≥-160`（约束）⇒ **infeasible**。与 tf/锥角/爬升约束**全无关**（逐一放松都无效，去掉本约束立刻 optimal） | 包络必须**从初速出发**：`cap0=\|vz0\|`，node 0 恒 `cap≥cap0`；`a_brk` 改用**扣重力后的真实减速**（原 0.35·T/m 只取到可用值的 44%） |
| ⑰ | **推力上界对数凸化方向搞反** | `s ≤ (T_max/exp(z0))·(1−(z−z0))` 是 `exp(−z)` 的**切线=全局下界**，拿来当上界是反向的；叠加 `m_ref` 模型偏低 12.33% ⇒ 解出 \|u\| 超物理上限 **5.4%~76%**，而求解器仍返回 `optimal` | 用切线方向修正并逐节点按真实质量校验；新增 `thrust_over_ratio` 后验体检（>1.0 即拒绝该轨迹） |
| ⑱ | **没把本载具真实推力传给求解器** | `gfold_terminal` 默认 `t_max=12.8e6`（**B1040-7 的推力**），而 B1040-9 实际 8.99 MN ⇒ 参考轨迹需要 86.7 m/s² 才能跟，载具只有 49.2 ⇒ 闭环表现为"跟不上、掉高度" | `gfold_land.py` 显式传 `t_max=self.v.max_thrust` |
| ⑲ | **"需要多少才停得住"算错口径** | 旧版 `need = hypot(vz,vh)²/(2h)` 把**总速度**当**单一方向**制动 ⇒ 报 22.9 看似轻松；分轴核算实为 51.8 > 可用 49.2 | 分轴算：垂直 `vz²/(2h)`、水平 `vh²/(2d)`，再矢量合成；并区分"能量不够"与"求解器约束" |
| ⑳ | **速度参考系传错 ⇒ vh 被天体自转污染** | `create_hybrid(velocity=body_frame)` 把**系自身线速度**当成原点速度 ⇒ `v.velocity()` 恒多出 **175.0 m/s**（= Kerbin 该纬度自转 **174.9**，差 0.06%）。**位置 dist 正确、竖直 vz 正确，只有水平 vh 错**——所以极难发现。后果：求解器以为"要在 0 米内消 176 m/s"⇒ **全程 infeasible**；跟踪律 `vel_err` 带入假速度 ⇒ `a_cmd` 冲到 **817 m/s²**（可用 47.6）⇒ throttle 饱和 1.0 ⇒ **火箭被满油门推着往上爬**（日志末尾 alt=3006、vz=+5.3） | `velocity` 改传 `tgt_frame`；并在 `state()` 加**位移反推自检**（位置口径独立于速度口径），接近自转标尺即告警 |
| ㉑ | **kOS 交班后未 `unlock`，每帧覆盖 kRPC** | 只清零 `throttle_demand` 但 `ctrl_main` 仍 `true` ⇒ `throttle_cmd()` 返回**制导值**而非 0；kOS 的 `lock` 每物理帧重算、优先级**高于** kRPC 写入 ⇒ 日志 `thr_cmd` 恒 **1.0**（285/306 行）而 `thr_act` 只有 **0~0.5**。**kRPC 从未真正接管节流**，却"以为自己接管了" | 交班时 `ctrl_main=false` + **`unlock throttle/steering`** + `break` 结束脚本；kRPC 侧新增 `verify_takeover()` 用 cmd vs act 实测比对 |
| ㉒ | **kRPC 从未 engage，姿态根本没接管** | 只设了 `ap.reference_frame/target_direction/up_reference` —— AutoPilot 在 engage 前只是"存了一组目标值"，不控制任何东西。日志 `tilt_cmd` 命令 27~41° 而 `tilt_act` 漂到 **63~89°**，完全不相关 ⇒ 姿态始终由 kOS 的 `lock steering` 掌控 | `apply()` 里 `ap.engaged = True` 并打印确认 |
| ㉓ | **`auto_pilot.engage()` 这个 API 不存在（靠猜写错）** | 实飞报 `AttributeError: 'AutoPilot' object has no attribute 'engage'. Did you mean: 'engaged'?` —— 我凭印象写了 `engage()`，触发**未捕获异常直接崩溃**（kRPC 侧退出，火箭失控） | 官方 XML 文档 `P:...AutoPilot.Engaged`：*"Setting to **true** engages the auto-pilot"*；预生成 stub 里 `AutoPilot` **没有 `engage()` 方法**，只有可读写属性 `engaged`（底层 RPC `AutoPilot_set_Engaged`）⇒ 正确写法 **`ap.engaged = True`** |
| ㉔ | **`track()` 坐标系符号不一致** | 求解器约定水平量是**一维朝目标轴**（位置恒 `+dist`、速度恒 `-vh`），而 kRPC `tp/tv` 是**带符号的相对偏移**。直接相减 ⇒ 火箭在"负侧"时 `pos_err` 凭空多出 `2·dist`、`vel_err` 多出 `2·vh` ⇒ `a_cmd` 被 PD 顶到 **720 m/s²**（可用 38.7） | 把实测位置/速度**投影到"本机→目标"单位向量**，与求解器同一约定，再把水平指令转回 y/z 分量 |
| ㉕ | **漏掉参考仓库的 `conic_clamp` ⇒ 火箭横飞** | PD 之后**没有把指令限幅回推力锥**。实飞首帧 `a_cmd_mag=1290.7`（可用 **49.28**，超 **26 倍**）⇒ throttle 饱和 1.0；竖直被削平后水平占比失控 ⇒ `vh` 从 **186 涨到 873 m/s**，`alt` 几乎不降（1310~2500 m 徘徊）——**满油门横飞** | 逐行移植参考仓库 `demo3_gfold.py` 的 `conic_clamp()`，PD 后立刻调用 |
| ㉖ | **自行发明结构而非照抄参考仓库** | 我自创了 `tf` 二分搜索、时间索引跟踪、投影对齐、`auto_pilot` 接管；参考仓库用的是 **P3→tf→P4 两级求解**、`find_nearest_index` 最近点跟踪、直接写 `control.pitch/yaw/roll`。自创结构连错三次（㉑㉒㉕） | **先读 `dev/gfold/demo3_gfold.py` 再动手**，能用参考实现就用参考实现，只改载具相关参数 |
| ㉗ | **首次解算同步阻塞 ⇒ 交班后 4.5 s 无人控制** | 实测 `solve_ms = 3898.8 ms`。kOS 交班后已 unlock 并结束脚本，这 3.9 s 内**没人写执行器** ⇒ 自由落体 **1396 m**（日志铁证：kOS 交班 `alt=5034.4` → kRPC 首帧 `alt=3638.7`）。等解完时状态已恶化到**分轴能量比 1.29**（需 63.7 / 可用 49.3）⇒ **一开头就超能量，后面救不回来** | 照抄参考仓库的**非阻塞**模式（它 `nav_mode='none'` 时仍每帧写执行器）：首次解算改后台线程，等待期间进 `nav_mode='hold'` 稳住 |
| ㉘ | **只在 `--dry-run` 里画线 ⇒ 实飞看不到线** | `self.draw()` 被放在 `if self.args.dry_run:` 分支内，正常飞行（`dry_run=False`）**一个点都不画**。参考仓库是 `debug_lines=True` 时**解完就更新线**（line 326-327），与是否接管控制无关 | 画线移出 dry-run 分支，由 `--no-debug-lines` 控制（默认画）；失败不再静默 `pass`，改为告警 |
| ㉙ | **`est_time` 用成"求解耗时"⇒ 外推过头** | 我把外推量写成 `max(0.5, solve_ms/1000)=3.3 s`，结果 `alt` 被推低 **1034 m**、`dist` 从 1278 变成 85 ⇒ 冲出滑翔锥，**21 次重解全部 infeasible** | `est_time` 补偿的是**执行延迟**（0.5 s），**不是求解耗时** —— 求解跑在后台线程，主循环期间一直在飞，耗时不入外推 |
| ㉚ | **离线仿真器用欧拉法，求解器用梯形法** | 求解器动力学是 `v[n+1]=v[n]+dt/2*((u_n+g)+(u_{n+1}+g))`（梯形），我的仿真用 `v+=acc*dt`（欧拉）。开环复现：直接喂参考轨迹**自己的** `u`，落点偏 **107 m**（应为 0）⇒ "轨迹不可跟踪"是**假象** | 仿真积分改成同一套梯形法；改成后竖直方向立刻正常（`alt` 5025→36.8 落地） |
| ㉛ | **删掉旧 import 后没补 `G0` ⇒ NameError 崩溃** | 从 `from gfold_terminal import ...` 改成 `from gfold_p3p4 import ...` 时，`G0` 这个常量**没跟着补**。文件里 4 处用 `G0`，Python 只在**执行到那一行**才报错 ⇒ 实飞跑到 `hold_up`（line 1429）当场 `NameError` 崩溃，kRPC 进程退出、火箭失控 | 补上 `G0 = 9.80665`（与求解器同值）；新增 `tools/check_names.py` 做**顺序敏感**的未定义名检查（已验证：删掉 G0 定义会报出全部 4 处） |
| ㉜ | **跟踪增益接反 ⇒ 姿态乱动** | 参考是 `(v_i-vel)*k_v + (x_i-error)*k_x`（速度 0.8 / 位置 0.5），我写成 `(v_i-vel)*KP_X + (x_i-error)*KV_V` —— **两个增益互换**：阻尼削弱 38%、刚度放大 60%，阻尼比 ζ 从 **1.13 掉到 0.56**（欠阻尼）。实飞表现为 `tilt_cmd` 在 0° 与 60~80° 之间反复跳、`att_err` 一路涨到 **119°** | 改成 `* K_VEL` / `* K_POS`；**并把常量改名**（`K_POS`/`K_VEL`，名字里写明乘在谁身上），杜绝再接反。修好后离线闭环实测落地 `dist=0.0 / vh=0.0 / vz=−0.7` |
| ㉝ | **画线两次都看不到** | ① `self.draw()` 写在 `if dry_run:` 里 ⇒ 实飞从不调用；② 修①后仍看不到：每次 `remove()` 旧线再 `add_line()` 新建，对象只活一帧。参考仓库是**启动时建一次、之后只改 `.start/.end`**（line 233-234 / 248-255） | 新增 `_ensure_lines()`（建一次）+ `draw()` 只更新端点；并把画线失败从静默 `pass` 改为**显式告警** |
| ㉞ | **日志 `tilt_act` 只是命令的回声** | 代码里 `tilt_act = self.tilt_cmd`，于是日志的 tilt_act 列**永远等于命令值**，完全看不出姿态有没有跟上 —— 我据此排查时被误导 | 改为从 `flight().direction`（机头在地面系的方向）与天顶求夹角，记录**真实倾角** |
| ㉟ | **改 import 后 `self.v = None` 淹没了真问题** | pyright 把 `self.v = None` 推成"永远 None"，于是 42 处 `self.v.xxx` 全报 `reportOptionalMemberAccess`。**真问题被噪音埋掉**：PID 的 `kp:int` 被赋 float（4 处）、`self.plan` 可能为 None 却被下标（7 处） | 类级加 `if TYPE_CHECKING:` 声明 kRPC 类型 + `__init__` 里 `# type: ignore[assignment]`。噪音 42→0，真问题全部暴露并修好 |
| ㊱ | **`_solve_one` 成功/失败返回的 dict 形状不一致** | 失败路径写 `return {'status': ...}`，成功路径返回十几键 —— pyright 并成 `dict[str, str|float]`，导致调用方 15 处"str 没有 shape"之类误报，**又把真问题埋掉** | 新增 `_fail()` 统一失败返回形状（所有键都在）。`gfold_p3p4.py` 现为 **pyright 0 错误** |

### 6.1 参考仓库是唯一权威（坑㉕ 的教训）

**原始仓库 `dev/gfold/`（GFOLD_KSP）里的 `demo3_gfold.py` 是唯一验证过的
G-FOLD 实飞程序。** 我自行设计控制律时漏掉了它的关键步骤，导致实飞"横飞"。

**参考仓库的完整控制结构**（`demo3_gfold.py`）：

| 环节 | 参考仓库做法 | 我原来的做法 | 结果 |
|---|---|---|---|
| 目标系 | `create_hybrid(pos=temp, rot=surface, **velocity=temp**)` (L225) | 传 `body_frame` | **vh 多出 174.9 m/s 自转**（坑⑳） |
| 跟踪律 | `target_a = u_i + (v_i-vel)*k_v + (x_i-error)*k_x` (L378) | 同（但加了自作聪明的投影） | — |
| **推力锥限幅** | **`conic_clamp(target_a, min, max, max_tilt)` (L389-390)** | **无** | **a_cmd 冲到 1290 m/s²（可用 49）⇒ 横飞** |
| 姿态 | `vessel.control.pitch/yaw/roll` + 自写 PID (L471-477) | `auto_pilot` | 两套结构不一致 |
| 索引 | `n_i = max(n_i - dt*0.2*N/tf, find_nearest_index(...))` (L363) | 时间索引 | — |

**核心教训**：`conic_clamp` 是把"PD 修正后的期望加速度"重新压回
**物理可执行锥内**的一步。没有它，PD 增益 × 任意大的位置/速度误差
会直接顶到几百 m/s²，然后被 throttle 饱和掩盖掉 ——
表现就是**满油门但方向失控**。

**已修正**：`track()` 里 PD 之后立刻调用 `conic_clamp()`（逐行移植参考仓库，
仅加零向量除零保护）。参数对齐参考 `params.txt`：

| 量 | 参考 params.txt | 本项目 |
|---|---|---|
| `k_x` / `k_v` | 0.5 / 0.8 | `KP_X=0.5` / `KV_V=0.8`（本来就一致） |
| `max_tilt` | 25° | `CONIC_TILT_DEG=25.0` |
| `throttle_limit_ctrl` | [0.05, 1.0] | min_mag=0.05·a_cap, max_mag=1.0·a_cap |

**验证**（`dev/krpc/verify_conic_clamp.py`，用实飞首帧真实数字）：

```
原始指令 |a| = 1290.74 m/s²  (超 26.2 倍, throttle 饱和 1.0)
限幅后   |a| =   49.28 m/s²  (throttle 1.000, 推力轴倾角 25.00°)
1000 组随机输入: 最大 |a|/(T/m)=1.0000, 最大倾角=25.00°  全部通过
```

### 6.2 kRPC API 名称纪律（坑㉓ 的教训）

**不要凭印象写 kRPC 的 API 名称。** 2026-09-26 一次实飞因为这个崩溃：

```
AttributeError: 'AutoPilot' object has no attribute 'engage'.
    Did you mean: 'engaged'?
```

我写了 `ap.engage()`，触发**未捕获异常** ⇒ **kRPC 进程当场退出** ⇒ 火箭失控。
同一天还发现 `body.rotation_period` 也是错的（正确：**`rotational_period`**，
注意与 KSP 自身的 `body.rotationPeriod` 拼写不同）。

**三个权威来源，按优先级用**：

| # | 来源 | 路径 / 链接 | 用途 |
|---|---|---|---|
| ① | **本机 mod 的 XML 文档** | `GameData/kRPC/KRPC.SpaceCenter.xml` | 最权威：就是这版 mod 的行为 |
| ② | **官方预生成 stub** | `D:\Applications\Python\Lib\site-packages\krpc\services\spacecenter.py` | 可直接 `hasattr` 内省；给出准确签名 |
| ③ | 官方文档站 | [krpc.github.io/krpc](https://krpc.github.io/krpc/) | 概念与教程（注意版本可能与 mod 不一致） |

`AutoPilot` 的接管语义（来源 ①，原文）：

> `P:KRPC.SpaceCenter.Services.AutoPilot.Engaged`
> *"Whether the auto-pilot is engaged. **Setting to `true` engages the
> auto-pilot**; setting to `false` disengages it."*

⇒ 正确写法是 **`ap.engaged = True`**，本构建**没有** `engage()` 方法。

**已加自动化防线**：`tools/check_krpc_api.py`
用官方 stub 做**真实类内省**（不是正则抓文本），逐个核对代码里出现的
kRPC 成员，把"猜出来的 API"在**飞行前**揪出来。

```powershell
python tools/check_krpc_api.py     # PASS 才允许上飞
```

> 已验证该脚本能抓住上面两个真实 bug（注入后立刻报出对应行号）。

### 6.3 kOS 后缀纪律（坑⑫ 的教训）

**kOS 没有 try/catch —— 任何后缀写错都会当场终止整个脚本。**
实飞中交班那一行用了 `ship:id`，结果在 `alt<5000` 的交班瞬间脚本死掉，
整趟白飞，kRPC 也永远等不到交班行。

**已做的防御**：发射台自检阶段就**预拼一次完整的交班行**（见
`boot/B1040-9.ks` 的"交班字段自检"），在起飞前就能看到：

```
交班字段自检：GFOLD_HANDOFF lat=-0.0972 lon=-74.5576 alt=4.2 t=... mass=... type=Ship name=B1040-9
```

**拼错的名字会在这里就暴露**，而不是等到最关键的那一刻。

**Vessel 的合法身份后缀**（[官方文档](https://ksp-kos.github.io/KOS/structures/vessels/vessel.html)核实）：
`SHIPNAME` / `NAME`（同义）、`TYPE`、`STATUS`。**没有 `ID`。**

---

## 7. 相关文件

| 文件 | 作用 |
|---|---|
| `boot/B1040-9.ks` | 段1 飞行脚本（kOS） |
| `dev/krpc/gfold_land.py` | 段2 控制端（kRPC + G-FOLD） |
| `dev/krpc/verify_control_loop.py` | 离线闭环验证（不连 KSP）；`--real` 用实飞交班点 |
| `dev/krpc/verify_terminal.py` | B1040-7 终端律验证（5/5） |
| `dev/krpc/verify_real_handoff.py` | 实飞交班点可行性 + 推力可实现性验证 |
| `dev/krpc/verify_frames.py` | **参考系口径回归**（抓自转污染，坑⑳） |
| `dev/krpc/diag_frame.py` | 连 KSP 实机诊断各参考系口径（排障用） |
| `dev/krpc/diag_track.py` | 连 KSP 实机诊断 track() 坐标/符号一致性 |
| `dev/krpc/diag_ap.py` | 连 KSP 实机列出 AutoPilot 真实 API（防猜错，坑㉓） |
| `tools/check_krpc_api.py` | **飞行前**核对全部 kRPC 成员名（用官方 stub 内省） |
| `tools/check_cols.py` | 核对 kOS 日志表头列数 = 数据列数 |
| `tools/kos_lint.ps1` | kOS 语法权威检查 |
| `dev/krpc/HANDOFF.md` | 本文 |
| `dev/gfold/solver/gfold_terminal.py` | G-FOLD 求解器（修正版） |
| `log-B1040-9-gfold/` | 段2 飞行日志 + README |
| `dev/reports/EGUIDANCE_GFOLD_HYBRID_REPORT.md` | 混合架构验证报告 |
| `dev/reports/PHASE1_GUIDANCE_ATTEMPTS.md` | 段1 五次尝试与失败原因 |
