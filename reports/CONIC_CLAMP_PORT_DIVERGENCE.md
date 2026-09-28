# 按原仓库方法定位：conic_clamp 移植走样

**日期**: 2026-09-27
**日志**: `log-B1040-9-gfold/gfold_log_20260927_164450.csv`
**方法**: 按用户要求 —— **逐行对照原仓库的轨迹跟踪方法**，不自行推演。

---

## 一、原仓库的跟踪方法（逐行）

`dev/gfold/demo3_gfold.py:360-405` 每帧流程：

```python
# 362-363  索引推进
n_i = max(n_i - game_delta_time*0.2*N/tf, find_nearest_index(x, error, vel))
# 368-369  采样当前点 + 前瞻点
(x_i, v_i, u_i)    = sample_index(n_i)
(x_i_, v_i_, u_i_) = sample_index(n_i + min(1.5*N/tf, norm(vel)/50*N/tf))
# 378-379  跟踪律（位置项都用 x_i）
target_a  = u_i  + (v_i  - vel)*k_v + (x_i - error)*k_x
target_a_ = u_i_ + (v_i_ - vel)*k_v + (x_i - error)*k_x
# 387-390  锥限幅
target_a  = conic_clamp(target_a,  min_throttle_ctrl, max_throttle_ctrl, max_tilt)
target_a_ = conic_clamp(target_a_, min_throttle_ctrl, max_throttle_ctrl, max_tilt)
# 391-392  索引未定位时的降级
if n_i < 0: target_a = [g0,0,0] + u_i
# 397-398  姿态用前瞻、节流用当前
target_direction = target_a_ / norm(target_a_)
target_throttle  = norm(target_a) / (max_thrust/mass)
```

**我们的 `target_a`/`target_a_` 公式、增益、索引推进、采样、降级分支：与原仓库一致。**

---

## 二、关键差异：`conic_clamp`

### 2.1 原仓库（`demo3_gfold.py:292-314`）

```python
def conic_clamp(target_a, min_mag, max_mag, max_tilt):
    a_hor = norm(target_a[1:3]);  a_ver = target_a[0]

    if a_hor < min_mag*sin(max_tilt):  a_ver_min = sqrt(min_mag**2 - a_hor**2)
    else:                              a_ver_min = cos(max_tilt)*min_mag
    if a_hor < max_mag*sin(max_tilt):  a_ver_max = sqrt(max_mag**2 - a_hor**2)
    else:                              a_ver_max = cos(max_tilt)*max_mag

    a_ver = clamp(a_ver, a_ver_max, a_ver_min)      # <== 竖直被抬到 a_ver_min
    a_hor = min(a_hor, a_ver*tan(max_tilt))
    return hor_dir*a_hor + v3(a_ver,0,0)
```

**要点**：`a_ver` 被 `clamp` 到 **`a_ver_min > 0`**（因为 `min_mag > 0`）。
**竖直分量永远不会变成 0 或负数**，所以 `a_hor = min(a_hor, a_ver*tan)` 里的 `cap` 永远为正，
**水平指令永远不会被清零**。

### 2.2 我们的（`gfold_land.py:1472-1504`）

```python
if a_hor < 1e-9: return [clamp(a_ver,min_mag,max_mag), 0, 0]
if a_ver < 0.0:  a_ver = 0.0            # <== 归零（原仓库是"抬高到 a_ver_min"）
cap_hor = a_ver * tan(max_tilt)         # <== a_ver=0 ⇒ cap_hor=0
if a_hor > cap_hor: a_hor = cap_hor     # <== 水平被清零
mag2 = hypot(a_ver, a_hor)
if mag2 < min_mag: ... 等比放大 ...      # <== 把纯竖直放大到 min_mag
return hor_dir*a_hor + [a_ver,0,0]       # = [min_mag, 0, 0]
```

**走样点**：原仓库把负的 `a_ver` **抬到正常数下限**；我们把它**归零**。
归零导致 `cap_hor = 0`，于是 `a_hor → 0`，最后被放大成 **`[min_mag, 0, 0]`** ——
**正是日志里 `a_cmd_up=2.64, a_cmd_h=0.00, thr_cmd=0.05` 的签名。**

### 2.3 用实飞数值直接对比（`diag_clamp_reference_vs_ours.py`）

命令取日志反推的真实原始指令，`a_cap=54.82, min_mag=2.74`：

| 输入 `a_ver` | 倾斜角 | 原仓库输出 `[a_ver, a_hor]` | 我们的输出 | 丢失的水平 |
|---|---|---|---|---|
| −2.44 | 25° | `[2.484, 1.158]` | `[2.741, 0.000]` | **1.158 m/s²** |
| −15.67 | 25° | `[2.484, 1.158]` | `[2.741, 0.000]` | **1.158 m/s²** |
| −2.44 | 60° | `[1.371, 2.374]` | `[2.741, 0.000]` | **2.374 m/s²** |
| −2.44 | 85° | `[0.239, 2.731]` | `[2.741, 0.000]` | **2.731 m/s²** |
| −2.44 | 89° | `[0.048, 2.741]` | `[2.741, 0.000]` | **2.741 m/s²** |
| **+0.50** | 25° | `[2.484, 1.158]` | `[2.484, 1.158]` | 0（一致） |

**结论**：只要 `a_ver < 0`，我们就把水平指令全部丢掉；原仓库则保留它。
实飞中 **112/148 帧**（76%）命中这个分支。

---

## 三、为什么 `a_ver` 会变负 —— 这不是 bug，是正常的 PD 响应

用真实日志状态逐帧重放（`diag_step_direction.py`），第 3 帧起：

```
t=1.22  u_i_up=4.88  vterm_up=-14.40  pterm_up=+7.08  ->  raw a_ver = -2.44
t=2.06  u_i_up=0.82  vterm_up=-15.20  pterm_up=-1.29  ->  raw a_ver = -15.67
t=5.79  u_i_up=1.45  vterm_up=-18.39  pterm_up=+1.78  ->  raw a_ver = -15.15
```

`vterm_up` 长期在 −14…−18：因为**计划的下降速度比载具实际快**（计划 node40 是 vz=−158，
载具当时只有 −120 左右）。PD 于是给出"往下压"的指令 —— **这是正确的控制行为**
（载具比计划慢，就该少减速/加速下坠）。

原仓库遇到同样情况会输出 `[0.048, 2.741]`（几乎纯水平、保留 2.7 m/s² 水平权限）；
我们输出 `[2.741, 0]`（纯竖直、**水平权限为 0**）。
⇒ 横向再也无法修正 ⇒ 完全不跟踪。

---

## 四、必须记录的：我这次排查中走过的弯路

| # | 错误判断 | 如何被推翻 |
|---|---|---|
| 1 | 预热轨迹起点错位是根因 | 上游修复后**仍不跟踪**，且现象改变（前 3 帧正常） |
| 2 | 求解器末端节点缺约束（"18 节点超限"） | 我用了**冻结初始质量**算的常数上限；改用随质量变化的 `r2/m(t)` 后**零违规** |
| 3 | 最小油门 0.1 导致计划飞不出来 | SSME 配置实为 **`minThrust = 0`**；且 `thr_act=0.05` 确实被执行 |
| 4 | 前瞻 `step` 跨过推力悬崖导致 `target_a_` 塌陷 | 子代理用 `step=0 vs 4.948` 得到**逐位相同**的油门输出 → 看似推翻 |
| 5 | 同上（重新检验） | **#4 的检验不完整**：它只比了 `target_a`（节流），没比 `target_a_`（姿态）。补测后 `step` 对姿态目标影响达 **86.4°** → 前瞻**确实**有影响 |

**第 5 条是本次的关键教训**：验证一个量时必须确认它**参与**被检验的输出。
`target_a` 与 `target_a_` 是两个不同的向量，只比前者无法证伪关于后者的假设。

**但即使前瞻有影响，它也不是最终根因** —— 因为 `a_h` 归零是 `conic_clamp` 造成的，
而 `conic_clamp` 对**任何** `a_ver<0` 的输入都会归零，与前瞻无关。
前瞻只是让 `a_ver` 更容易变负的**诱因之一**。

---

## 五、修复方案

按"照抄原仓库"的原则，**把 `conic_clamp` 改回原仓库语义**：

```python
def conic_clamp(self, target_a, min_mag, max_mag, tilt_deg=None):
    max_tilt = math.radians(CONIC_TILT_DEG if tilt_deg is None else tilt_deg)
    a_hor = float(np.linalg.norm(target_a[1:3]))
    a_ver = float(target_a[0])
    if a_hor < 1e-9:                       # 原仓库此处会除零，保留保护
        return np.array([max(min(a_ver, max_mag), min_mag), 0.0, 0.0])
    hor_dir = np.array([0.0, target_a[1], target_a[2]]) / a_hor

    # ---- 原仓库 demo3_gfold.py:300-308 ----
    if a_hor < min_mag * math.sin(max_tilt):
        a_ver_min = math.sqrt(max(0.0, min_mag**2 - a_hor**2))
    else:
        a_ver_min = math.cos(max_tilt) * min_mag
    if a_hor < max_mag * math.sin(max_tilt):
        a_ver_max = math.sqrt(max(0.0, max_mag**2 - a_hor**2))
    else:
        a_ver_max = math.cos(max_tilt) * max_mag

    # ---- 原仓库 line 310-312：竖直被【抬到】a_ver_min，不是归零 ----
    a_ver = max(a_ver_min, min(a_ver_max, a_ver))
    a_hor = min(a_hor, a_ver * math.tan(max_tilt))
    return hor_dir * a_hor + np.array([a_ver, 0.0, 0.0])
```

**预期效果**：`a_ver<0` 的 112 帧不再丢水平权限，`a_cmd_h` 保持非零 ⇒ 横向可修正。

### 需要保留我们自己的两点（不照抄的部分）
1. **零向量保护**：原仓库 `hor_dir /= norm(hor_dir)` 在 `a_hor=0` 时会除零。
2. **时变锥角**：原仓库用固定 `max_tilt=25°`；我们用 `tilt_now = max(tilt_traj+8, 25)`
   是因为求解器用了 `pcs_start=85°→pcs_end=30°` 的时变锥（原仓库的轨迹由同一个 25° 锥解出，
   天然自洽）。**这条保留**，否则轨迹倾角 85° 会被固定 25° 锥砍掉 58% 水平。

---

## 六、下一步

1. 应用上述 `conic_clamp` 修正。
2. 跑 6 项预检。
3. 用 `diag_clamp_reference_vs_ours.py` 与本文件 §2.3 的表格核对（应全部一致）。
4. 实飞验证：`a_cmd_h` 应全程非零；`tilt_dir_cmd` 不应长期为 0。

> **注意**：本次修正**只改控制器，不改求解器**。计划本身的形状（前 4 节点重刹、
> 中间 19 s 低推力滑行、末端大推力）是求解器在 `minThrust` 与最优性下的正常输出，
> 不是 bug。若实飞后仍显得"油门很小"，那是**计划如此**，不是跟踪失败。
