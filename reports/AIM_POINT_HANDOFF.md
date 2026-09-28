# 交接对齐「瞄准点」（2026-09-27，按用户要求）

> 用户要求："瞄准点是在目标点上3000米，我是要你在一级到达瞄准点附近时开始交接，
> 对齐这个要求，搞清楚流程"
>
> 结论：**旧交班门理解错了** —— 它用绝对高度 5000 m，而一级此时
> **离瞄准点还有 2000 m**。这正是"交接后不跟踪"的上游根源。

---

## 1. 流程澄清（这是核心）

### 1.1 瞄准点在哪

`boot/B1040-9.ks:127`
```kos
local aim_alt is 3000.   // 瞄准点抬高量 [m]（目标点上方 3000 m）
local target_alt is 36.8. // 最终触地高度 [m]
```

⇒ **瞄准点绝对高度 = target_alt + aim_alt = 3036.8 m**

### 1.2 段1（E-guidance）是奔着瞄准点去的

`boot/B1040-9.ks:1188`（`r_tgt` 指向抬高后的瞄准点）：
```kos
set r_tgt to vxcl(upv, tgt_site:altitudeposition(
             tgt_site:terrainheight + target_alt + aim_alt))
             - upv * (alt:radar - target_alt - aim_alt).
```

`h_aim` = `alt:radar - target_alt - aim_alt`（到瞄准点的垂距）

制导律（B1040-7 结构，`a = r_tgt·(6/T²) − v·(4/T)`）的意图是
**在瞄准点把位置与速度同时归零**。

### 1.3 ⇒「到达瞄准点附近」≡ `h_aim ≈ 0`

### 1.4 旧交班判据错在哪

```kos
-- 旧（错）
if h_gnd < gfold_h and ...          gfold_h = 5000
```

用 **`h_gnd < 5000`（绝对高度）** 作门 ⇒ 交班时：

```
alt = 5044,  h_aim = 5044 - 36.8 - 3000 = 2007 m
```

**一级离瞄准点还有 2000 m 就被交出去了**，段1 根本没机会在瞄准点收速。

### 1.5 实飞数据印证（`log-B1040-9/land_log_47883027.csv`）

| t | alt | h_aim | dist_hz | vz | vh | thr | gfold_cone_ok |
|---|---|---|---|---|---|---|---|
| 47882999.8 | 19622 | 16586 | 16551 | −561 | 706 | 0.15 | 0 |
| 47883014.8 | 10549 | 7525 | 6356 | −623 | 656 | 0.91 | 1 |
| 47883025.8 | 5405 | 2368 | 1612 | −315 | 250 | 0.55 | 1 |
| **47883027.0** | **5044** | **2013** | **1336** | **−288** | **221** | 0.54 | **1** ← 交班 |

交班瞬间 `h_aim=2013`、`vz=−288`、`dist_hz=1336` ——
**带着 363 m/s 动能和 1.3 km 水平偏差交给 kRPC**。

⇒ 这就是"交接后不跟踪"的**上游根源**：
PD 的位置项 `(x_i − error)` 量级与轨迹指令 `u_i` 相当，
一旦打架就把 `target_a` 竖直分量压负 ⇒ `conic_clamp` 返回纯竖直
`[min_mag, 0, 0]` ⇒ **`a_cmd_h=0`、`thr=0.05`**。

---

## 2. 可行性核算：段1 能否在瞄准点收完速

用交班点数据（`alt=5044.2, h_aim=2007.4, vz=−288.1, vh=220.9`）：

```
段1 时标  T = 2·h_aim / max(√h_aim, |vz|) = 2×2007/288.1 = 13.9 s
|v|       = 363.0 m/s
需减速度  = 363.0 / 13.9 = 26.0 m/s²
可用 a_net = 28.9 m/s²   （日志）
```

**26.0 < 28.9 ⇒ 能收完** ✓
收完后在瞄准点：`vz→0`、`vh→0`、`dist→0`。

---

## 3. 改动

### 3.1 kOS 交班判据：绝对高度 → 瞄准点

```kos
-- 旧
if h_gnd < gfold_h and v_now:mag < gfold_vmax and gfold_cone_ok and h_gnd > p66_h

-- 新（行 1251）
if h_aim < gfold_h and v_now:mag < gfold_vmax and gfold_cone_ok and h_gnd > p66_h
```

**`gfold_h` 的语义随之改变**：从"绝对高度门"变成"**瞄准点上方余量**"。

| 参数 | 旧值 | **新值** | 含义 |
|---|---|---|---|
| `gfold_h` | 5000（绝对高度） | **150** | 瞄准点上方余量 [m] |
| `gfold_vmax` | 480 | **120** | 段1 已收速，门槛可收紧 |
| `gfold_cone` | 48 | 48 | 不变 |

交班高度 = `36.8 + 3000 + 150 = 3186.8 m`

### 3.2 kRPC 侧同步

```python
AIM_ALT = 3000.0
GATE_AIM_H = 150.0
GATE_ALT = TARGET_ALT + AIM_ALT + GATE_AIM_H     # = 3186.8 m
GATE_VMAX = 120.0
GATE_CONE_DEG = 48.0
START_ALTITUDE = GATE_ALT
```

### 3.3 预热配套（随新门重新标定）

新交班点段1 已收速，下降率很小（`vz` 约 −20~−40），
同样的高度差对应**更长的时间**：

| 项 | 值 |
|---|---|
| `PREWARM_ALT_FACTOR` | **1.10** → 预热门 3505 m |
| 到门高度差 | **318.7 m** |
| `PREWARM_MAX_ALT_GAP` | **400 m**（> 318.7，不会误判过期）✓ |

> 说明：低速段的速度外推比高空准得多（段1 已收速、加速度小），
> 所以允许更大的起点差，才能保住"零等待"。

---

## 4. 验证

```
pyright 0 错误 / pyflakes / 未定义名 / kRPC 成员 / kOS 语法   全部 PASS
```

### 判据一致性

```
[kOS]  gfold_h=150(余量) gfold_vmax=120 gfold_cone=48  → 交班 alt=3186.8
[kRPC] GATE_ALT=3186.8   GATE_VMAX=120  GATE_CONE_DEG=48
一致: True
```

### 新交班点求解可行性（瞄准点附近状态）

| 到瞄准点 \|v\| | 状态 | tf | 落地 |
|---|---|---|---|
| 20 (vz=−8 vh=18) | optimal | 7.0 s | **179.2 t** |
| 60 (vz=−24 vh=54) | optimal | 6.0 s | 177.3 t |
| 110 (vz=−44 vh=99) | optimal | 8.2 s | 172.5 t |

**落地质量 172~179 t**（对比旧门 150.9 t）—— 少烧约 25 t，
因为交接时速度已接近 0，G-FOLD 只需管最后 3 km 的垂直下降。

### 回归

```
verify_conic_clamp  全部通过
verify_terminal     5/5
diag_log            离线闭环 dist=0.0 vz=-1.4 vh=0.0
verify_prewarm_fix  重解后 a_cmd=[8.42,-38.15] tilt=77.55 thr=0.800（正常）
```

---

## 5. 改动汇总

| 文件 | 改动 |
|---|---|
| `boot/B1040-9.ks` | 交班判据 `h_gnd` → **`h_aim`**；`gfold_h` 5000→**150**（语义变为"瞄准点上方余量"）；`gfold_vmax` 480→**120** |
| `dev/krpc/gfold_land.py` | 新增 `AIM_ALT`/`GATE_AIM_H`；`GATE_ALT` = **3186.8**；`GATE_VMAX` = **120**；`START_ALTITUDE` = `GATE_ALT` |
| `dev/krpc/gfold_land.py` | `PREWARM_ALT_FACTOR` → **1.10**、`PREWARM_MAX_ALT_GAP` → **400** |
| `dev/krpc/verify_aimgate.py` | 新增验证脚本 |

---

## 6. 下趟应看到

| 观察 | 旧（5000 门） | 期望（瞄准点门） |
|---|---|---|
| 交班 alt | ~5044 m | **~3187 m** |
| 交班时 `h_aim` | **2013 m** | **~150 m** |
| 交班 `\|v\|` | ~363 m/s | **< 120 m/s** |
| 交班 `dist_hz` | ~1336 m | **接近 0** |
| `a_cmd_h` | 第 2 帧起 0.00 | **持续非零** |
| `thr_act` | 0.05 | **0.8~1.0** |
| 落地质量 | — | 应优于 150 t（实测可达 172~179 t） |

> kOS 侧日志的 `h_aim` 列可直接验证交班时刻：
> 应在 `h_aim < 150` 时才出现 `gfold_cone_ok=1` 并交班。
