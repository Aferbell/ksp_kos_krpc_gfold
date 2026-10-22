# 高度基准（datum）审计：kOS alt:radar vs kRPC surface_altitude（2026-10-21-j）

> 触发：用户指出「18 是 kOS 的 `alt:radar`」，问旧程序怎么判断的、
> 以及 kRPC 是不是不用 kOS 的 `alt:radar`、两者读数是否不同。
> 复跑脚本：`dev/analysis/probe_184650.py`（纯日志分析，不依赖 KSP 在线）
> **本次未改动任何飞行代码**

---

## 0. 三句话结论

1. **kOS 的 alt:radar 和 kRPC 的 surface_altitude 是【同一个物理量】**，
   实测交班点两者只差 1.5~2.9 m —— 正好等于交班延迟。
   **不存在「两个 API 读数不一样」的问题。**

2. **真正的问题是我们对 TARGET_ALT 的物理含义理解错了。**
   这两个 API 都是 **Measured from the center of mass（从质心量）**，
   而 TARGET_ALT 一直被当成「着陆腿离地高度」在用。
   **质心高度 != 腿触点高度**，差一个载具几何常数。

3. **旧程序能落地是因为方向和量级碰巧都对**
   （TARGET_ALT=17 > 真触地 14.6，留了 2.4 m 余量）；
   **starship 用 TARGET_ALT=18 < 真触地 24.2，等于要求「质心穿过地面 6.2 m」**，
   物理上永远到不了，所以撞地。

---

## 1. 旧程序是怎么判断的

### 1.1 旧程序的高度来源

```python
# dev/krpc/gfold_land.py:1147-1155
def state(self):
    fl = self.v.flight(self.ref)
    # 用 flight 的海拔（与 kOS 的 alt:radar 口径最接近）
    alt = fl.surface_altitude
```

**注释里写的「与 kOS 的 alt:radar 口径最接近」—— 这是正确的。**
旧程序从头到尾用的也是 surface_altitude，不是别的。

### 1.2 旧程序的两代落地判据

| 时代 | 判据 | 高度门 |
|---|---|---|
| FIN 引入前 | alt <= TARGET_ALT + 1.0 **或** situation == landed | 18.0 m |
| FIN 引入后 | FIN 内：只有 situation == landed 才算完成 | — |

代码原文（gfold_land.py:4406-4416）：

```python
if self.nav_mode == "fin":
    _situ = self.v.situation.name
    if _situ == "landed":
        break            # 只有真触地才退出
    if _fin_dt > FIN_TIMEOUT: break
    if vz > FIN_TAKEOFF_GUARD: break
```

### 1.3 旧程序 26 趟 FIN 时代的真实触地高度

统计 log-B1040-9-gfold/gfold_log_*.csv 里所有带 fin= 字段的 LANDED 行：

| 量 | 值 |
|---|---|
| 样本数 | 26 趟 |
| min | 13.11 m |
| p10 | 14.09 m |
| **p50** | **14.63 m** |
| p90 | 15.67 m |
| max | 20.70 m |

**而旧程序 TARGET_ALT = 17。**

```
TARGET_ALT(17.0) - 真触地(p50 14.63) = +2.37 m
```

**⇒ 旧程序的 TARGET_ALT 比真实触地读数【高 2.4 m】。**
这意味着规划零速点停在触地上方 2.4 m，载具在到达规划零点前就已经触地。
**这是「保守」，不是「精确」—— 但它方向对、量级对，所以能落地。**

---

## 2. kOS alt:radar vs kRPC surface_altitude：同一物理量

### 2.1 交班点直接对比（10 趟）

kOS 写进 gfold_state.txt 的 alt=round(alt:radar,1)，
对比 kRPC 首帧 flight().surface_altitude：

| log | kOS alt | kRPC alt | 差 | vz |
|---|---|---|---|---|
| 20261020_190025 | 630.3 | 631.7 | -1.4 | 70.4 |
| 20261020_202734 | 616.5 | 619.4 | -2.9 | 72.3 |
| 20261021_112739 | 630.0 | 632.8 | -2.8 | 70.9 |
| 20261021_123854 | 920.5 | 923.4 | -2.9 | 75.5 |
| 20261021_131806 | 932.3 | 932.3 | 0.0 | 72.1 |
| 20261021_135036 | 931.9 | 933.4 | -1.5 | 71.9 |
| 20261021_144058 | 917.7 | 917.7 | 0.0 | 75.3 |
| 20261021_154351 | 938.1 | 938.1 | 0.0 | 74.1 |
| 20261021_172305 | 921.9 | 921.9 | 0.0 | 75.7 |
| 20261021_184650 | 929.3 | 930.8 | -1.5 | 73.6 |

```
|kOS - kRPC| : p50 = 1.5 m   max = 2.9 m
换算成时间   : p50 = 0.020 s   max = 0.040 s
```

**差值全部落在「交班延迟」的量级**（kOS 写完文件 -> kRPC 读到 -> 建立连接，几十毫秒）。
差 0.0 的那几趟是采样恰好同帧。

**⇒ 两个 API 是同一物理量，不存在「读数不一样」。**

### 2.2 kRPC 文档原文（三个高度属性全是「从质心」）

```
Flight.surface_altitude:
  "The altitude above the surface of the body or sea level, whichever is
   closer, in meters. Measured from the center of mass of the vessel."

Flight.bedrock_altitude:
  "The altitude above the surface of the body, in meters. When over water,
   this is the altitude above the sea floor.
   Measured from the center of mass of the vessel."

Flight.mean_altitude:
  "The altitude above sea level, in meters.
   Measured from the center of mass of the vessel."
```

**三个属性全部是 Measured from the center of mass —— 没有任何一个给「到着陆腿」。**

---

## 3. 那为什么两台车「触地读数」差 10 m

### 3.1 数据

| | 旧 B1040-9 | starship |
|---|---|---|
| 载具长度 | 约 30 m | 约 70 m |
| 真触地时 surface_altitude | **14.1 ~ 15.7 m**（p50 14.63）| **24.20 m**（184650）|
| 程序里 TARGET_ALT | 17.0 | 18.0 |
| 差（TARGET_ALT - 真触地） | **+2.37 m** | **-6.2 m** |

### 3.2 解释

surface_altitude 是**从质心量**的。触地那一刻，它读到的就是「质心到地面的距离」。

```
质心离地距离 = 载具半长（质量集中在两端时略偏）
  旧 B1040-9: 30 m 长 => 质心约 15 m => 读数 14~16 m  吻合
  starship  : 70 m 长 => 质心约 24 m => 读数 24.2 m     吻合
```

**⇒ 两台车的「触地读数」差 10 m 是【载具几何】决定的，不是 API 有问题。**

### 3.3 TARGET_ALT 的物理含义（这是关键认知）

```
TARGET_ALT 不是「腿离地多高」，而是「质心离地多高」
  —— 因为它直接与 surface_altitude 比较

旧程序 TARGET_ALT=17 的隐含意思：
  「让质心降到离地 17 m」
  而该载具腿触地时质心在 14.6 m
  => 规划要求降到 17 m，物理上到不了 => 提前触地，余量 2.4 m  安全

starship TARGET_ALT=18 的隐含意思：
  「让质心降到离地 18 m」
  而该载具腿触地时质心在 24.2 m
  => 规划要求降到 18 m，比触地还低 6.2 m => 要求穿地 => 必然撞地
```

---

## 4. 这条错误怎么导致 184650 那趟的连锁故障

### 4.1 直接后果：规划要求不可能达到的状态

184650 在 alt=28.09 时：

```
plan_vz = -2.34      （规划要求几乎刹停）
实际 vz = -16.76     （还在猛掉）
差      = -14.42 m/s
```

因为规划认为「再往下 11 m 就到 18 m 的零速点了」，所以全力刹车；
而载具实际在 24.2 m 就撞上了，**规划还在要求继续下降**。

### 4.2 间接后果：FIN 窗口被吃光

```
FIN 门   = TARGET_ALT + FIN_ALT = 18 + 10 = 28.0 m
真实触地 = 24.20 m
窗口     = 28.0 - 24.2 = 3.8 m

而末段每帧位移 4~5 m（loop_dt p50=0.240，vz=-12~-17）
=> 3.8 m 窗口【不足一帧】=> FIN 永远拿不到控制帧
```

实测正是 fin=0.00s —— 进入即退出。

### 4.3 姿态问题与之独立

姿态问题是另一条链（av_plan=3.15 -> 油门 0.143 -> alpha 7.5 -> slew 26 < 需求 34.5），
详见 dev/reports/LANDING_184650_ANALYSIS_20261021.md 第 2 节。
**它和高度基准错误是两件事，不要混在一起改。**

---

## 5. TARGET_ALT 应该取多少

### 5.1 按旧程序口径反推

```
旧程序余量 = TARGET_ALT(17.0) - 真触地_p50(14.63) = +2.37 m
starship 真触地 = 24.20 m
=> TARGET_ALT = 24.20 + 2.37 = 26.57，约 26.5 m
```

### 5.2 三个候选

| 取法 | 值 | 说明 |
|---|---|---|
| 实测触地值 | 24.2 | 精确对齐，无余量，风险在地形起伏 +/-1 m |
| 实测 + 1 m | 25.2 | 小幅余量 |
| **实测 + 2.4 m（旧程序口径）** | **26.5** | 与旧程序同策略 |

### 5.3 改了之后各门怎么变

| TARGET_ALT | FIN_ALT | FIN 门 | 真实可用行程 |
|---|---|---|---|
| 18.0 | 10.0 | 28.0 | **3.8 m**（不足一帧）|
| 24.2 | 10.0 | 34.2 | 10.0 m |
| **26.5** | **10.0** | **36.5** | **12.3 m** |
| 26.5 | 15.0 | 41.5 | 17.3 m |

**TARGET_ALT=26.5 + FIN_ALT=10 给出 12.3 m 真实行程**，
按 sqrt(2*20.61*12.3 + 1) = 22.5 m/s，能兜住 22.5 m/s 的入口速度。

### 5.4 同时必须改的

```
boot/superheavy.ks:209            local target_alt is 18.  ->  26.5
dev/starship/starship_config.py   TARGET_ALT = 26.5
```

**因为 kOS 的 alt:radar 与 kRPC 的 surface_altitude 同口径（第 2 节已证），
两边必须同值，否则 h_aim/h_gnd 与 error[0] 错位。**

GATE_ALT 会自动从 TARGET_ALT + AIM_ALT + GATE_AIM_H 变成 976.5 m。

---

## 6. 教训（要写进 HANDOFF）

**TARGET_ALT 不是「腿离地多高」，是「质心离地多高」。**

换载具时 TARGET_ALT 必须重新实测，**方法**：

1. 让载具以任意速度落到地面（哪怕砸坏）；
2. 读 flight().surface_altitude 的触地读数；
3. TARGET_ALT = 触地读数 + 余量（旧程序用 +2.4 m）。

**不能照抄旧载具的 17，也不能凭「腿有多高」估。**
旧 B1040-9 的 17 和 starship 的 24.2 差 7 m —— 全部是几何差异。

---

## 7. 待确认

1. **TARGET_ALT 取 24.2 / 25.2 / 26.5 哪一个？**
   我建议 **26.5**（沿用旧程序的 +2.4 m 余量策略，已验证有效）。

2. **FIN_ALT 保持 10 还是加到 15？**
   TARGET_ALT=26.5 + FIN_ALT=10 已给 12.3 m 真实行程，能兜 22.5 m/s，
   **建议保持 10**（先验证，不够再加）。

3. **kOS 侧 target_alt 同步改 26.5** —— 这个必须做，否则口径错位。

4. **situation==landed 判据要不要加 vz 保护？**
   加了能拦住「还在掉就退出」，但也可能让 FIN 在真触地后继续推；
   **建议先不改**，等 TARGET_ALT 修正后再看。

---

## 8. 本次落地的改动（2026-10-21-j）

| 文件 | 常量 | 原值 | 新值 |
|---|---|---|---|
| `dev/starship/starship_config.py` | `TARGET_ALT` | 18.0 | **26.5** |
| `boot/superheavy.ks` | `local target_alt is` | 18. | **26.5** |

**其它一律未动**（按用户要求「其他先不变」）：
`TERM_CLEARANCE=20`、`FIN_ALT=10`、`AVER_TGT_VZ=1`、`FIN_DESCENT_VZ=1`、
`LAND_ALT_EPS=1`、`CTRL_X_ROT_KP=2.0`、`REF_SLEW_MAX=20`、`TILT_LEAD_SEC=1.5`。

### 8.1 门槛链的变化

| | 改前（`TARGET_ALT=18`）| 改后（`26.5`）|
|---|---|---|
| 规划零点 / 末段律 | 38.0 m | **46.5 m** |
| FIN 门 | 28.0 m | **36.5 m** |
| 非 FIN 触地判据 | 19.0 m | **27.5 m** |
| 规划归零点 | 18.0 m | **26.5 m** |
| `GATE_ALT` | 968.0 m | **976.5 m** |
| **FIN 真实可用行程** | **3.8 m** | **12.3 m** |
| **能兜住入口 vz** | 9.4 m/s | **22.5 m/s** |

真实行程 = `FIN 门 − 真触地读数` = 36.5 − 24.2 = **12.3 m**
（`TARGET_ALT` 比真触地读数高 2.3 m，所以行程比名义 `FIN_ALT` 大）。

### 8.2 验证

```
pyflakes（6 生产 + 7 分析脚本）         = 0
常量断言                              = 全部 OK
门槛链严格递减 46.5 > 36.5 > 27.5 > 26.5 = OK
_selftest.py                          = 3/3 OK, pos_err=0.00
求解器 TA=18 vs TA=26.5                = 3/3 均 optimal
kOS 与 kRPC 交班门同步                  = 976.5 vs 976.5，差 0.0 m
kOS 后缀检查                          = ALL PASS
codegen 需重生成？                     = 否（TARGET_ALT 不在编译期集合）
```

---

## 9. 下次实飞的观察重点

| 列 / 现象 | 期望 | 若不达标的含义 |
|---|---|---|
| `FIN enter alt` | **≈ 36.5 m** | < 30 ⇒ `TARGET_ALT` 还需上调 |
| `fin=...` 时长 | **> 0.5 s** | = 0.00 ⇒ FIN 窗口仍不足一帧 |
| `LANDED alt` | **≈ 24.2 m** | 明显偏离 ⇒ 真触地读数不是 24.2 |
| `LANDED vz` | **接近 −1.0** | 远大于 1 ⇒ FIN 没来得及收敛 |
| `plan_vz` @ `alt=36` | 应 ≈ −1 ~ −3 | 仍是 −2 而实际 −16 ⇒ 竖直跟踪崩了 |

**判据纪律**：先看 `FIN enter alt` 和 `fin` 时长。
这两项对了，说明高度基准修好了；再看 `LANDED vz` 才轮到讨论减速能力。
