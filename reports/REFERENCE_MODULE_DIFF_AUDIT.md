# 与原仓库逐模块对比：发现的 Bug 清单

**日期**: 2026-09-27
**方法**: 逐行对照 `dev/gfold/` 原仓库，不自行推演。
**对照文件**:
- `demo3_gfold.py`（实飞控制器，485 行）
- `GFOLD_direct_exec.py`（作者的原始 SOCP 问题定义，154 行）
- `params.txt`（全部参数）
- `GFOLD_run.py` / `GFOLD_codegen.py` / `postprocess.py`

---

## 一、发现的 Bug（按严重度排序）

### BUG 1（致命，已修）：求解器删掉了原版的【起点推力竖直】约束

**原版 `GFOLD_direct_exec.py:59-60`**：
```python
con += [u[:,0]   == s[0,0]  *np.array([1,0,0])]   # thrust START straight
con += [u[:,N-1] == s[0,N-1]*np.array([1,0,0])]   # thrust END straight
```
**两端都强制竖直。**

**我们只保留了末端那条**（`gfold_p3p4.py:305` 附近），起点删掉了。
旧注释给的理由是"带横速入场时起点不能强制竖直" —— **这个理由是错的**。

**实测证据**（交接 `alt=3167.2 vz=-82.98 vh=47.73`）：

| | 节点 0 的 `u` | 节点 0 倾角 | tf | 落地 |
|---|---|---|---|---|
| 删除后（旧） | `[3.785, 0, 43.264]` | **85.0°** | 24.25 | 146.3 t |
| 恢复后（新） | `[43.429, 0, 0]` | **0.0°** | 24.25 | 145.6 t |

**为什么这是致命的**：节点 0 正是**交接后第一帧立刻执行**的指令。
- 旧：要求 85° ⇒ 载具当时姿态只有 **24.5°**（实测 `tilt_act=24.52`）
  ⇒ 需在 ~0.2 s 内转 **60.5°**。姿态权限实测约 13.5°/s²，转 60° 需 ~4.2 s ⇒ **不可能**
  ⇒ 推力方向错 ⇒ 前 3 帧 0.86~0.95 的油门几乎全用在竖直减速
  ⇒ `vz` 从 −82.98 掉到 −69.65（比计划快 20 m/s 地刹住）
  ⇒ `vterm[0]` 变负 ⇒ `conic_clamp` 清零水平 ⇒ **完全不跟踪**
- 新：要求 0°（纯竖直）⇒ 载具已在竖直方向 24.5° 内，
  `cos(24.5°)=0.91` 的竖直推力**立刻可用**，无需先转姿。

**代价**：落地质量 146.3 → 145.6 t（**−0.7 t，0.5%**）。
**当初的担心不成立** —— 恢复约束后求解器**照样解得出**。

**已修**：`gfold_p3p4.py` 与 `gfold_codegen.py` **两处都补上**，并**重新生成 codegen `.pyd`**
（约束是编译时固化的，不改生成器就等于没改）。
两条路径现在数值一致：cgen `tf=24.25/145.6t`、cvxpy `tf=24.50/145.5t`，节点 0 倾角均为 0°。

---

### BUG 2（严重，已修）：`conic_clamp` 把负竖直"归零"而非"抬高"

**原版 `demo3_gfold.py:300-312`**：
```python
if a_hor < min_mag*sin(t):  a_ver_min = sqrt(min_mag**2 - a_hor**2)
else:                       a_ver_min = cos(t)*min_mag
if a_hor < max_mag*sin(t):  a_ver_max = sqrt(max_mag**2 - a_hor**2)
else:                       a_ver_max = cos(t)*max_mag
a_ver = clamp(a_ver, a_ver_max, a_ver_min)   # <== 抬到正的下限
a_hor = min(a_hor, a_ver*tan(t))
```
**竖直被抬到 `a_ver_min > 0`** ⇒ `cap_hor` 恒为正 ⇒ 水平指令**永不被清零**。

**我们旧实现**：
```python
if a_ver < 0: a_ver = 0            # 归零（原版是抬高）
cap_hor = a_ver*tan(t)             # => 0
if a_hor > cap_hor: a_hor = cap_hor # 水平被清零
... 幅值不足则等比放大 ...
=> 返回 [min_mag, 0, 0]
```

**实飞数值对比**（`dev/krpc/diag_clamp_reference_vs_ours.py`）：

| 输入 | 倾角 | 原版 | 旧实现 | 丢失水平 |
|---|---|---|---|---|
| `a_ver=-2.44, a_hor=45` | 25° | `[2.484, 1.158]` | `[2.741, 0.000]` | 1.158 |
| 同上 | 85° | `[0.239, 2.731]` | `[2.741, 0.000]` | **2.731** |
| 同上 | 89° | `[0.048, 2.741]` | `[2.741, 0.000]` | **2.741** |
| `a_ver=+0.50` | 25° | `[2.484, 1.158]` | `[2.484, 1.158]` | 0（一致） |

**实飞命中率**：112/148 帧（76%）`a_cmd_h = 0.00` 且 `a_cmd_up = 2.64 = min_mag` ——
正是该签名的产物。

**已修**；`dev/krpc/verify_conic_clamp_fix.py` 在 9×4×4=144 组输入网格上与逐行移植的
原版**逐点一致**。

---

### BUG 3（中等，已修）：`final` 段漏掉锥限幅

**原版 `demo3_gfold.py:427`**：
```python
target_direction = conic_clamp(target_direction, 1, 1, max_tilt)
```
（参数 `(1, 1, max_tilt)` = 只限方向、不改幅值。）

**我们漏了这一行** ⇒ `final` 段可命令超出 `max_tilt` 的姿态
⇒ 触地前姿态可能越界（本项目指标要求触地倾角 < 5°）。

**已修**：对单位向量做锥限幅（压水平到 `a_ver*tan(max_tilt)` 后再归一化）。

---

### BUG 4（轻微，**未修，待你决定**）：原版的推力变化率约束我们没实现

**原版 `GFOLD_direct_exec.py:78`（原版自己注释掉了）**：
```python
#con += [norm(u[:,n+1]-u[:,n]) <= dt*T_max/m_dry * 3]
```

这是**唯一**能阻止"bang-bang 计划"的约束。原版注释掉是因为它的用例平缓
（起始 1.1 km、`tf=80 s`）；本项目是**大推力 + 短 tf**，计划因此呈
"重刹 4 节点 → 低推力滑行 19 s → 末端猛刹"的形状。

**影响**：计划的推力剖面剧烈变化，跟踪器的 `u_i` 也随之剧烈变化。
但它**不是**本次"不跟踪"的原因（根因是 BUG 1）。是否启用取决于你是否
希望计划更平滑。

**若启用需注意**：会改变最优性（燃料增加），且可能使某些工况 infeasible。

---

## 二、核对后确认**不是** Bug 的项（避免后人重查）

| 项 | 结论 |
|---|---|
| `throttle_limit = [0.1, 0.8]`（求解器）vs `[0.05, 1.0]`（控制器） | **两套并存是原版设计**。我们 `conic_clamp` 用 `0.05/1.00`、求解器用 `0.1/0.8` —— **正确** |
| 传动增益 `k_x=0.5, k_v=0.8` | 与原版 `params.txt` 一致 |
| `CONIC_TILT_DEG = 25`、`FINAL_*` 全套 | 与 `params.txt` 一致 |
| `find_nearest_index` | 逐行一致（仅加了除零保护） |
| `sample_index` | 逐行一致，含两个怪癖：`index>=N-1` 返回全零、`index<0` 用 `u[:,1]` |
| `track()` 跟踪律（含两处都用 `x_i`） | 逐行一致 |
| `n_i` 推进式 `max(n_i - dt*0.2*N/tf, nearest)` | 与原版 `line 363` 一致 |
| `vessel_profile1` 外推公式 | 与原版 `line 168-189` 一致 |
| PID 类与姿态轴映射 | 与原版 `line 461-477` 一致（连 `[-0.5,0.5]` 的 clamp 都一致） |
| `p_cs = max_tilt*0.85` | 原版是固定锥；**我们改成时变锥是【有意的且必要】**——否则大横速入场 infeasible。**保留** |
| `est_h_center` 在原版算了但没用 | 我们也不算 —— 一致 |
| `final` 段下界 `0.05` | 原版 `throttle_limit_ctrl[0]=0.05` —— 我们硬编码 0.05 **是对的** |
| 地面约束 `x[0,n] >= 0` | 原版 `line 103` 注释掉了；我们启用。**保留**（防止"先落地再飞起"） |

---

## 三、本次改动清单

| 文件 | 改动 |
|---|---|
| `dev/gfold/solver/gfold_p3p4.py` | 补回起点竖直约束（BUG 1） |
| `dev/gfold/solver/gfold_codegen.py` | 同上（必须与上者同步，否则两条路径解出不同轨迹） |
| `dev/gfold/solver/cpg_p3/*.pyd` | **重新生成**（约束编译时固化） |
| `dev/gfold/solver/cpg_p4/*.pyd` | **重新生成** |
| `dev/krpc/gfold_land.py` | `conic_clamp` 恢复原版语义（BUG 2）；`final` 段补锥限幅（BUG 3） |
| `dev/krpc/diag_constraint_diff.py` | 新增：约束集三方对比工具 |
| `dev/krpc/diag_clamp_reference_vs_ours.py` | 新增：clamp 与原版对比 |
| `dev/krpc/verify_conic_clamp_fix.py` | 新增：修复后的回归验证 |

**验证**：6 项预检全部 PASS；两条求解路径数值一致；节点 0 倾角 85° → 0°。

---

## 四、遗留与下一步

1. **BUG 4 未修**（推力变化率）—— 需要你决定是否启用。
2. **实飞验证**：节点 0 现在是纯竖直，期望：
   - 第 0 帧 `att_err` 远小于 48°
   - `tilt_act` 能跟上 `tilt_dir_cmd`
   - `a_cmd_h` **全程非零**
   - `vz` 与 `plan_vz` 不发散
3. **仍未解释**：`thr_cmd` 与 `a_cmd_mag` 的 ~4% 系统性偏差
   （`a_cmd_mag/(thr_cmd·T/m) ≈ 0.94`）。已排除 clamp 顺序与采样错位。
   子代理判断为"内部自洽、非 bug"，但 `implied T ≈ 8.6e6` vs 配置 `8.99e6` 仍无解释。
   **建议**：在日志里加一列 `kRPC 的 self.v.max_thrust` 与 `self.v.mass` 同帧记录，
   下次实飞即可判定。
