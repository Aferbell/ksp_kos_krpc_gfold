# 方案B：吸收 RocketRecoveryTutorial 的直给回收算法

> 来源：`C:\Users\xzq\Desktop\DSharness Works\RocketRecoveryTutorial-main`
> （Mr_GEFORCE 可回收火箭教程，B站 BV1kz4y1G7x8）——**本项目 B1040-7 的源头仓库**。
> 两个实现：`starShipRecovery.py`（kRPC 版）、`KOS/bst-pidloop.ks`（kOS 版）。
> 仿真证据：`dev/analysis/sim_recovery_plans.py`（L2 / L2n 行）。

---

## 一、教程算法是什么

### kOS 版（`KOS/bst-pidloop.ks`，37 行，最贴近我们要写的东西）

```kos
set target_alt to 38.
lock steering to R(srfRetrograde:pitch, srfRetrograde:yaw+10, srfRetrograde:roll).  // ① 纯反速度
set throttle_pid to pidLoop(0.015, 0.001, 0.05, 0, 1).                            // ② 定增益 PID
set throttle_pid:setpoint to target_alt.
wait until alt:radar < 26000.  lock throttle to 0.33.     // ③ 高空减速段
wait until ship:velocity:surface:mag < 1300.  lock throttle to 0.
wait until alt:radar < 1300.                              // ④ 固定高度门
until FALSE {
    if alt:radar < 250 { gear on. }
    if ship:groundspeed < 30 { lock steering to srfRetrograde. }  // ⑤ 横速小了去掉偏置
    lock throttle to throttle_pid:update(time:seconds, alt:radar). // ⑥ PID 看着高度调油门
    if alt:radar < 38 { lock throttle to 0. break. }
}
```

### kRPC 版（`starShipRecovery.py:139-179`）—— 终端律更清楚

```python
final_throttle.init(kp=0.01, ki=0, kd=0.03, upper=0.8, lower=0)
...
while vessel.flight().surface_altitude > 14:
    throttle = final_throttle.update(13 - surface_altitude, -vertical_speed)  # ⑦ PD(高度误差, 下降率)
    if |velocity[1:]| > 50: steering_ap(-10)   # 横速大 → 反速度 + 10° 偏置
    else:                   steering_ap(0)     # 横速小 → 纯反速度
```
（`PIDUsingV.update` 返回 `err*kp + integral + v*kd`，即这是个
 **"高度误差 P + 下降率 D"** 的 PD，不带 I。）

### 三件套归纳

| # | 结构 | 作用 |
|---|---|---|
| ① | **纯反速度指向** `srfRetrograde` | 推力与速度矢量反向 ⇒ **天然把竖直+水平合成一路**，不需要任何通道仲裁 |
| ② | **固定高度门**（1300 m） | "高空不开车、到点再开" —— 正是你上一轮想要的**自杀点火** |
| ③ | **PD 看着高度调油门** | 用下降率闭环，简单 |

**为什么 ① 很值钱**：反速度指向下，倾角**恰好等于飞行路径角**
`tilt = atan2(vh, vz)`，随速度方向自动旋转。
它不需要 `arctan2 → cos/sin`（上一轮那个单位坑的整条链），
kOS 里一行 `lock steering to srfRetrograde.` 就完事。
同时也**不存在"竖直通道 vs 水平通道抢推力"**的问题 —— 推力全部用于消速度矢量。

---

## 二、★ 为什么不能照搬：定增益 PD 在 Mun 上必然失效（解析证据）

教程的油门律（kRPC 版 ⑦）是：

```
thr = 0.01*(13 - h) + 0.03*vz        (clamp 0 .. 0.8)
```

**点火条件 `thr > 0`**：

```
0.03*vz > 0.01*(h - 13)   =>   vz > (h - 13)/3
```

在 **Mun（无大气）** 自由落体：`vz = sqrt(2·g·h)`，`g = 1.63 m/s²`。
两式联立：

```
sqrt(2*1.63*h) = (h-13)/3    =>   h ≈ 52 m
```

> **⇒ 在 Mun 上要掉到约 52 m 才点火 —— 太晚，必摔。**

**教程在 Kerbin 能用**，是因为**大气终端速度**远大于 `(h-13)/3`：
低空 `vz` 已经有 100~250 m/s，所以约 250 m 就能触发。
Mun 没有大气，"速度换高度"的裕度小得多。

**仿真复核**（`sim_recovery_plans.py` 的 **L2n** 行，照抄教程）：

| 律 | 落地 vz | 落地 vh | 耗燃料 | 结论 |
|---|---|---|---|---|
| L2n 教程 PD 原样 | **302.28** | **389.29** | 0.067 t | **摔** |

⇒ **结论：教程的「结构」可以吸收，「增益/触发」必须重算。**
这也是本项目 `MUNLANDING_PLAN_20261014.md` §2.1 已经记录过的坑
（"参考仓库用的正是固定增益 PID，在 Mun 这种低 g 环境末段会不够力"），
本节把它从"经验判断"升级为**可复算的解析结论**。

---

## 三、吸收什么、替换什么

| 教程元素 | 处置 | 理由 |
|---|---|---|
| ① 纯反速度指向 | **原样吸收** | 结构最优，且天然避开 arctan2 单位坑 |
| ② 固定高度门 1300 m | **替换**为与载具能力挂钩的门 | 固定高度随入口速度失配 |
| ③ 定增益 PD 油门 | **替换**为"等减速度需求 ÷ cos(倾角)" | 见 §二，PD 在 Mun 上点火太晚 |
| ④ 250 m 放起落架 | 保留（`munlanding.ks` 已有 `gear_alt=250`） | — |
| ⑤ 横速<30 去掉偏置 | **对应**现状 `phase 2`（vh<2 切末段） | 语义一致 |
| ⑥ 高空 0.33 油门减速段 | **可选保留** | Mun 上不是必需，可省燃料 |

**替换后的点火门**（与载具真实能力挂钩，逐帧重算）：

```
h_gate = v² / (2·(a_max − g)) · margin          margin ≈ 1.1 ~ 1.3
```

**替换后的油门律**（保留"用下降率闭环"的思想）：

```
a_up_need = g + (vz² − vz_tgt²) / (2·h_eff)     // 竖直：等减速需求
cos_t     = vz / |v|                            // 反速度指向的几何关系
A         = a_up_need / cos_t                   // 所需【总】加速度
thr       = min(a_max, A) / a_max
```

---

## 四、kOS 代码骨架

```kos
// ============ 方案B：纯反速度指向 + 自杀点火门 ============
// 1) 姿态：一行搞定，零角度计算（kOS 内建反速度）
lock steering to srfRetrograde.

// 2) 自杀点火门：与载具能力挂钩，替代教程的固定 1300 m
set g_now to ship:body:mu / (r_mag() * r_mag()).
set a_max to avail_kn / ship:mass.
set v_mag to ship:velocity:surface:mag.
if not ignited and alt:radar < v_mag*v_mag/(2*max(0.5, a_max - g_now)) * ign_margin {
    set ignited to true.
    set local_neng to ignite_engines().
}
if not ignited { set thr to 0. }     // 滑行段：不开车
else {
    // 3) 油门 = 竖直等减速度需求 ÷ cos(倾角)
    set vz    to -ship:verticalspeed.
    set v_mag to max(0.01, ship:velocity:surface:mag).
    set cos_t to max(0.05, vz / v_mag).   // 反速度指向下 cos(倾角)=vz/|v|
    set a_up_need to g_now + (vz*vz - vz_tgt*vz_tgt)/(2*max(alt:radar - h_touch, 1)).
    set a_max to avail_kn / ship:mass.
    set A to min(a_max, a_up_need / cos_t).
    set thr to A / a_max.
}
```

**注意（不要照抄教程的地方）**：
- 教程的 `yaw+10` 偏置是**为了气动/栅格舵**。Mun 真空里没有气动，
  这个偏置只会平白引入横向分量 ⇒ **去掉**。
- 教程的 `target_alt = 38` + `break` 是"悬停到 38 m 然后关车"，
  对 Mun 着陆器是**硬砸**（从 38 m 自由落体）。本方案改用
  `vz_tgt` 控制触地率（等减速度律），不做悬停。

---

## 五、仿真结果（同一入口 vz=203.5 / vh=408.6 / h=19940 / m=8.282 t）

| 律 | 落地 vz | 落地 vh | 耗燃料 | dv | 动力段 | 结论 |
|---|---|---|---|---|---|---|
| L0 现状 | 2.05 | 0.00 | 2.408 t | 1028 m/s | 364 s | 安全 |
| **L2 方案B** | **2.30** | **0.04** | **1.538 t** | **614 m/s** | 121 s | **安全** |
| L2n 教程原样 | 302.28 | 389.29 | 0.067 t | 24 m/s | 77 s | **摔** |

- **省燃料 0.870 t = 36.1%**（相对现状），dv 从 1028 降到 614 m/s
- 滑行 27 s（纯弹道自由落体），然后动力 121 s
- 最大倾角 60.8°

**★ 一个必须说清的数字**：点火高度 ≈ **14007 m**，**不是"临地"**。
因为入口速度 474 m/s 在 TWR6.3 下需要约 13 km 才刹得住。
"高空不开车"成立，但"接近地面才开"在物理上不成立 —— 门只能开在约 14 km。

---

## 六、混合方案（自杀门 + 去定点 E制导）—— 顺便比较

把方案B 的点火门与方案A 的矢量律拼起来（仿真的 **L3** 行）：

| 律 | 耗燃料 | dv | 动力段 |
|---|---|---|---|
| L1 方案A | 2.277 t | 962 | 347 s |
| L3 混合 | 2.114 t | 882 | 259 s |
| **L2 方案B** | **1.538 t** | **614** | **121 s** |

⇒ 混合方案夹在中间，**省燃料仍远不如方案B**。
原因：E制导的等减速剖面一旦启动就会把减速摊开，滑行省下的重力损失被它吃掉。
**真正带来 36% 节省的是"反速度 + 到点猛烧"这个结构本身，不是点火门。**

---

## 七、收益与风险

**收益**
1. **省燃料 36%** —— 三个方案里最好。
2. **代码最短**：姿态一行 `lock steering to srfRetrograde.`，
   **完全没有 arctan2 / 倾角计算**，上一轮的单位坑结构性不存在。
3. **没有通道仲裁**：推力全部用于消速度矢量，不需要锥限幅那套。
4. 正好实现你要的"高空滑行 + 到点自杀点火"。

**风险**
1. **倾角越界（已扫，7/42 个入口）**。反速度倾角 = 飞行路径角 `atan2(vh, vz)`。
   扫描 `vh0 ∈ {100..800} × vz0 ∈ {100..600}`，有 7 个入口越过 `tilt_max=70°`：

   | 入口 | 倾角 |
   |---|---|
   | vh=300, vz=100 | 71.6° |
   | vh=400, vz=100 | 76.0° |
   | vh=500, vz=100 | 78.7° |
   | vh=600, vz=100 | 80.5° |
   | vh=600, vz=200 | 71.6° |
   | vh=800, vz=100 | 82.9° |
   | vh=800, vz=200 | 76.0° |

   ⇒ **方案B 必须保留一套锥限幅兜底**（限幅后不再是严格反速度，横速消得慢）。
   特征很清楚：**"平而快"的入口**（横速大、垂速小）才会越界。

2. **可刹边界（已扫）**。除倾角外，还有一批入口**真的刹不住**
   （落地 vz 134~372 m/s）。原因是"航程"不够，判据为：

   ```
   |v| · vz  ≤  2 · h_avail · ( a_max − g · vz/|v| )
   ```
   （左 = 速度×垂速；右 = 可用航程×有效减速度。航程 = 高度差 ÷ sin(γ)，
   横速越大航程越长 —— 这就是为什么"平着进"反而好刹。）

   实测边界（h_avail≈19.9 km）：

   | 入口 | 总速度 | 结果 |
   |---|---|---|
   | vh=300, vz=600 | 670 | 摔（vz=135） |
   | vh=400, vz=600 | 721 | 摔（vz=210） |
   | vh=600, vz=500 | 781 | 摔（vz=309） |
   | vh=800, vz=400 | 894 | **安全**（vz=2.61）← 平着进，航程长 |
   | vh=500, vz=500 | 707 | 安全（vz=2.88，临界） |

   ⇒ **越"竖着掉"越刹不住；`vz0 ≥ 600` 一律失败。**
   另有一批 `vz` 落在 2.9~3.2 的"落地偏快"，不是摔，是刚好压线。

3. **姿态权限**：60~83° 倾角对 RCS/反作用轮要求高。本项目
   `mun_probe_4` 那轮**没测到姿态权限**（见 MUNLANDING_PLAN §一），
   这是方案B 唯一的未验证前提。**建议先补测再实飞。**
4. 反速度指向在 `|v| → 0` 时方向退化，需要兜底（`v_mag` 下限 + 末段切竖直）。

---

## 八、验证步骤

1. 静态：`kos_lint.ps1 munlanding.ks`（本轮沙箱跑不了，需在 VS Code 确认）
2. 闭环：`python dev/analysis/sim_recovery_plans.py` 看 L2 行
3. **入口扫边界**：对 `vh0 ∈ {100..800}`、`vz0 ∈ {100..600}` 扫，
   记录最大倾角是否越过 `tilt_max`（决定要不要保留锥限幅）
4. **先补测姿态权限**：`ship:availabletorque` / `momentofinertia`
   （MUNLANDING_PLAN §八 列为"仍缺的一项数据"）
5. 实机分步：3 km 低速 → 20 km vz≈150/vh≈300 → 完整剖面
