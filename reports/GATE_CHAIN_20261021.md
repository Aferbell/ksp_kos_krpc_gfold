# 门槛链重构：TARGET_ALT 18 / TERM_CLEARANCE 20 / FIN_ALT 10 / AVER_TGT_VZ 1（2026-10-21-h）

> 用户要求：修正 `TARGET_ALT`（实测雷达高度 18 m），`FIN_ALT` 改为 10 m，
> `TERM_CLEARANCE` 改为 20，`AVER_TGT_VZ` 改为 1。
> 验证脚本：`dev/analysis/check_new_gates.py`
> 前一份报告：`dev/reports/LANDING_172305_ANALYSIS_20261021.md`

---

## 0. 结论摘要

| 项 | 改前 | 改后 |
|---|---|---|
| `TARGET_ALT` | 17.0 | **18.0** |
| `TERM_CLEARANCE` | 5.0 | **20.0** |
| `FIN_ALT` | 1.0 | **10.0** |
| `AVER_TGT_VZ` | 2.0 | **1.0** |
| `FIN_DESCENT_VZ` | 1.0 | 1.0（上一轮已改） |

**门槛链从「4 m + 1 m」变成「10 m + 10 m」：**

```
改前                          改后
──────────────────────────    ──────────────────────────
alt=22.0  规划零速点/末段律    alt=38.0  规划零速点/末段律
alt=18.0  FIN 接管             alt=28.0  FIN 接管
alt=18.0  非FIN触地判据 ⚠️重合  alt=19.0  非FIN触地判据
alt=17.0  规划归零点            alt=18.0  规划归零点

末段律窗口  4.0 m             末段律窗口 10.0 m
FIN 窗口    1.0 m             FIN 窗口   10.0 m
```

**验证结果：门槛链严格递减、互不重合；求解器在新 `TERM_CLEARANCE` 下全部 optimal。**

---

## 1. `TARGET_ALT` 17 → 18

用户实测：**落地时雷达高度 = 18 m**，而代码原先认为触点在 17 m。

这 1 m 的偏差有两个后果：

```
① 规划以为 17 m 就到位，实际 18 m 已经触地 => 提前 1 m 认为到达
② FIN 窗口 = (TARGET_ALT + FIN_ALT) - 18
   改前 TARGET=17, FIN_ALT=1  =>  18 - 18 = 0 m   ← FIN 拿到 0 米行程
```

**必须同步 kOS 侧**（否则 `h_aim`/`h_gnd` 与 kRPC 的 `error[0]` 错位 1 m）：

```
boot/superheavy.ks:209   local target_alt is 17.  ->  18.
```

`GATE_ALT` 由 `TARGET_ALT + AIM_ALT + GATE_AIM_H` 推导，自动跟着从 967 → **968 m**，无需手改。

---

## 2. `TERM_CLEARANCE` 5 → 20

### 改前为什么不够

`TERM_CLEARANCE` 决定两件事：

```python
① 求解器把规划零速点抬高 TERM_CLEARANCE：
   _x0_solver[0] -= TERM_CLEARANCE       // 少给求解器这么多高度
   r['x'][0,:]   += TERM_CLEARANCE       // 解完再平移回来
   => 规划末节点落在 alt = TARGET_ALT + TERM_CLEARANCE

② 末段下降律（track() 内）的触发门：
   _term_phase = (h_now < TERM_CLEARANCE)
```

**所以它同时是「规划零速点」和「末段律接管点」，两者必须同高，否则载具会在中间悬停。**

原值 5.0 让末段律窗口只有 `TERM_CLEARANCE - FIN_ALT = 5 - 1 = 4 m`。实测：

| log | 末段律触发 alt | FIN 接管 alt | 中间帧数 |
|---|---|---|---|
| 154351 | 18.28 | 12.30 | **1** |
| 144058 | 21.67 | 9.03 | **1** |
| 135036 | 13.34 | 13.34 | **0** |
| 172305 | 从未触发（最低 22.63 m）| — | — |

**4 m 窗口在 0.2 s/帧、`vz=-10~-90 m/s` 下只有 1~2 帧，末段律来不及做任何事。**

### 改后

`TERM_CLEARANCE = 20` ⇒ 规划零速点 38 m，末段律窗口 10 m。

**已验证求解器在 x0 高度少 15 m 时仍全部 feasible：**

| 工况 | `tc=5` | `tc=20` | `tf` 差 |
|---|---|---|---|
| 交班 968 m / vz −70 / vh 3 | optimal | **optimal** | −1.8 s |
| 中段 700 m / vz −60 / vh 5 | optimal | **optimal** | −0.2 s |
| 低空 400 m / vz −50 / vh 3 | optimal | **optimal** | −0.2 s |

`tf` 略减（轨迹更短），无副作用。

---

## 3. `FIN_ALT` 1 → 10

### 硬限制：FIN 能兜住的入口速度取决于行程

```
最大可兜 vz = sqrt(2 * a_net * FIN_ALT + FIN_DESCENT_VZ²)
a_net = a_cap - g = 30.42 - 9.81 = 20.61 m/s²

FIN_ALT = 1.0  =>  sqrt(2*20.61*1 + 1) =  6.5 m/s
FIN_ALT = 10.0 =>  sqrt(2*20.61*10 + 1) = 20.3 m/s
```

### 实测 FIN 入口速度（10 趟日志）

| log | FIN_alt | FIN_vz | 刹车需高度 | 结果 |
|---|---|---|---|---|
| 20261020_1900 | 16.50 | −26.96 | 17.6 m | LANDED |
| 20261020_2027 | 15.59 | −19.38 | 9.1 m | LANDED |
| 20261021_1318 | 17.10 | −40.77 | 40.3 m | abort |
| 20261021_1350 | 13.34 | −93.66 | 212.8 m | LANDED |
| 20261021_1440 | 9.03 | −58.24 | 82.3 m | LANDED |
| 20261021_1543 | 12.30 | −6.67 | 1.1 m | LANDED |

**6/6 趟的「刹车需高度」都远超「可用高度」。** `FIN_ALT=1.0` 只能兜 6.5 m/s，而实测入口是 6.7~93.7 m/s。

### 新行程的覆盖范围

| 入口 vz | 刹车需高度 | 10 m 够不够 |
|---|---|---|
| −5 m/s | 0.6 m | OK |
| −10 m/s | 2.4 m | OK |
| −15 m/s | 5.4 m | OK |
| **−20 m/s** | **9.7 m** | **OK（刚好）** |
| −25 m/s | 15.1 m | 不够 |
| −30 m/s | 21.8 m | 不够 |
| −40 m/s | 38.8 m | 不够 |

**⇒ 只要 gfold 段能把 `vz` 收到 −20 m/s 以内，FIN 10 m 就够。**
这正好把责任明确切成两段：**gfold 负责把速度降到 20 以内，FIN 负责最后 20 → 1**。

---

## 4. `AVER_TGT_VZ` 2 → 1

**这是三个「目标垂速」里唯一真正生效的那个：**

```python
av_need = g0 + (vz² - AVER_TGT_VZ²) / (2h)
```

这个公式被 `track()` 的末段下降律和 `fin_control()` 同时使用（同一形式），所以：

| 相位 | 改前目标 | 改后目标 |
|---|---|---|
| gfold 末段律 | −2.0 m/s | **−1.0 m/s** |
| FIN 段 | −1.0 m/s | −1.0 m/s |

**口径统一了：全程 1 m/s。**

### 关于 `TERM_DESCENT_VZ`

它是**死常量**，全项目零引用：

```
starship_config.py:241    TERM_DESCENT_VZ = 2.0        <- 定义
starship_land.py:100      TERM_DESCENT_VZ = C.TERM...   <- 导入
starship_land.py:1441     # 目标是 TERM_DESCENT_VZ       <- 只有这一句注释提到
starship_land.py:1463     ... - AVER_TGT_VZ * AVER_TGT_VZ  <- 实际用的是这个
```

**注释与代码不一致，已在 config 里加警告。** 本次不动它（避免"改一个常量变成改三处口径"），
但它是文档债，应该在下次重构时接入或删除。

---

## 5. 改动清单

| 文件 | 常量/变量 | 改前 | 改后 |
|---|---|---|---|
| `dev/starship/starship_config.py` | `TARGET_ALT` | 17.0 | **18.0** |
| 同上 | `TERM_CLEARANCE` | 5.0 | **20.0** |
| 同上 | `FIN_ALT` | 1.0 | **10.0** |
| 同上 | `AVER_TGT_VZ` | 2.0 | **1.0** |
| `boot/superheavy.ks` | `local target_alt is` | 17. | **18.** |

**未改动**：`FIN_DESCENT_VZ`（1.0，上一轮已改）、`TERM_DESCENT_VZ`（死常量）、
`LAND_ALT_EPS`、`CLIMB_ABORT_VZ`、`FIN_KD`、`FIN_UP_MIN`、求解器、codegen。

**`GATE_ALT` 自动跟随**：967 → 968 m（由 `TARGET_ALT + AIM_ALT + GATE_AIM_H` 推导）。

---

## 6. 验证

```
pyflakes（6 个生产文件 + 4 个分析脚本）   = 0
常量断言（5 个新值 + 门槛链严格递减）      = 通过
_selftest.py（977.5 / 628.5 / 800 t）    = 3/3 OK, pos_err=0.00
门槛自洽检查                              = 通过
求解器 tc=5 vs tc=20 对比                  = 3/3 均 optimal
kOS 后缀检查（check_kos_suffix.py）       = ALL PASS
codegen 需重生成？                        = 否（改动项与编译期集合交集为空）
```

---

## 7. 新的阶段结构（这是本次改动的核心收益）

```
alt = 968 m   ← kOS 交班（GATE_ALT，自动跟随 TARGET_ALT）
alt =  38 m   ← 规划零速点 / 末段下降律接管
      │        gfold 段负责：把 vz 从 -70 降到 -20 以内
      │        窗口 10 m，末段律有 10 m 行程可用
alt =  28 m   ← FIN 接管
      │        FIN 段负责：把 vz 从 -20 收到 -1
      │        窗口 10 m，能兜住 20.3 m/s 的入口
alt =  19 m   ← 非 FIN 路径判「已落地」（不再与 FIN 门重合）
alt =  18 m   ← 规划归零点 = 着陆腿触点
```

**分界清楚了：gfold 管到 20 m/s，FIN 管最后 20 → 1 m/s。**

---


## 8. 仍未解决的事（诚实标注）

1. **`TERM_DESCENT_VZ` 是死常量**，注释声称它用于末段律，实际代码用 `AVER_TGT_VZ`。
   本次只在 config 加警告，未重构。

2. **`av_need` 在 `vz≈0` 附近的「谷」没修**。
   `172305` 那趟的爬升就是这个谷造成的（`av_need` 一帧从 17.53 掉到 9.45，推力撤不及时）。
   本次把 `TERM_CLEARANCE` 从 5 提到 20 会**显著缓解**它（末段律有 10 m 行程，
   不会像原来那样在 1~2 帧内被迫做巨大调整），但**没有从根本上消除**。

3. **`172305` 那趟 `dvz/dt = +38.9 m/s²` 超过满推上限 23.4 的未定项仍在**。
   下一趟必须加 `situation` 列才能定性。

4. **`FIN_ALT=10` 能否在实飞中保住 20 m/s 的入口速度，取决于 gfold 段**。
   如果 `av_need` 的谷又让载具在 30~40 m 处刹过头爬升，FIN 就没机会工作。
   **这条要下一趟实飞验证，不能只看静态计算。**
