# 方案A：把 E-guidance「去定点化」用于非定点回收

> 来源：`boot/B1040-9.ks` §12（段1 E-guidance + 段2 终端），本项目已实飞的
> 定点回收结构（B1040-7 落偏 11.8 m 的同源律）。
> 目标：改成**只消横速、不追落点**的非定点回收，用于 `munlanding.ks`。
> 仿真证据：`dev/analysis/sim_recovery_plans.py`（L1 行）。

---

## 一、源算法是什么

`boot/B1040-9.ks:1468-1520` 段1 的核心只有两行：

```kos
set t_go to max(eg_tmin, 2*h_aim / max(sqrt(max(1, h_aim)), -ship:verticalspeed)).
set a_cmd3 to r_tgt * (6 / (t_go * t_go)) - v_now * (4 / t_go).
```

- `r_tgt` = **从飞船指向瞄准点**的矢量（含水平偏移 + 竖直 `-h_aim`）
- `v_now` = 当前速度矢量
- `t_go` = 时间标尺，由"剩余垂距 / 参考下降率"给出

**它的自洽性**（`B1040-9.ks:1482-1486` 已证明，我独立复核）：
把 `T = 2h/|vz|` 代入，竖直分量恰好化成 `vz²/(2h)` —— 就是"在 h 内把 vz 削到 0"
所需的等减速度。**指令 ≡ 需求**，所以它是精确等减速轨迹。

**它比双通道好的地方**（`B1040-9.ks:1473-1480` 的原话）：
旧的"竖直/水平各自一套律"会**互相抢推力**，谁被配额卡住谁失控；
E-guidance 是**整个 3D 矢量一次算出**，位置与速度同一时刻归零。

**段2 终端**（`B1040-9.ks:395-402` + 终端逻辑）：
```kos
a_up = term_k * (vz - term_v)          // 恒定下降率
a_h  = term_kp * r_tgt_h - term_kd * v_h   // 位置环 + 速度阻尼
tilt_cap 随 h_gnd/term_span 收缩到 0   // ⇒ 末段推力结构性地变竖直
```

---

## 二、"定点"体现在哪一项，怎么去掉

**定点 = `r_tgt` 的水平分量**。它把飞船往一个特定经纬度拽。
非定点 = **不关心落在哪**，所以：

```kos
// 去定点化：目标点永远在【正下方】，水平位置误差 ≡ 0
r_tgt_eff = upv * (-h_aim)          // 只保留竖直分量
```

代入原律，得到一个**不需要任何目标点/经纬度**的律：

| 通道 | 去定点后的净指令 | 含义 |
|---|---|---|
| 竖直 | `a_up_net = -6·h_aim/T² + 4·vz/T` = **`vz²/(2·h_aim)`** | 与现状竖直律**同构**（等减速度） |
| 水平 | `a_h_net = v_h · (4/T)` | 指数消速，**时间常数 τ_h = T/4** |

**关键收获**：`τ_h = T/4` 是**随高度自动收缩**的（见下表），
而现状 `munlanding.ks` 的 `T_h` 是**固定 8 s**。

| h_gnd | T | τ_h = T/4 |
|---|---|---|
| 19940 | 196.9 | 49.2 |
| 10000 | 97.5 | 24.4 |
| 2000 | 17.5 | 4.4 |
| 500 | 4.0（=eg_tmin 下限） | 1.0 |

⇒ 高空消横速温和（省燃料），**低空自动变急**（落地前把横速收干净）。
这正好是固定 `T_h` 做不到的：固定值只能全段取一个折中。

---

## 三、kOS 代码骨架（可直接替换 `guidance()`）

```kos
// ============ 方案A：E制导「去定点化」 ============
// 单位：全程【度】（kOS 三角函数语义）；不要出现 57.29578
function eg_guidance {
    set g_now to ship:body:mu / (r_mag() * r_mag()).
    set vz    to -ship:verticalspeed.          // 下降率为正
    set vh    to ship:groundspeed.
    set h_gnd to alt:radar.

    if h_gnd > aim_alt {
        // ---- 段1：E制导（去定点）----
        set h_aim to max(1, h_gnd - aim_alt).   // 只留【竖直】位置误差
        set T     to max(eg_tmin, 2*h_aim / max(sqrt(max(1,h_aim)), vz)).
        set a_up_n to -6*h_aim/(T*T) + 4*vz/T.  // 净竖直（= vz^2/(2h_aim)）
        set a_h_n  to vh * (4/T).               // 净水平（时间常数 T/4）
    } else {
        // ---- 段2：终端（B1040-9 段2 去掉位置项）----
        set a_up_n to term_k * (vz - term_v).
        set a_h_n  to term_kd * vh.
        set tcap   to term_tilt * min(1, h_gnd / term_span).
        set a_h_n  to min(a_h_n, tan(tcap) * max(0.5, a_up_n + g_now)).
    }
    if a_up_n < 0 { set a_up_n to 0. }          // 绝不主动爬升

    // 推力口径 = 净指令 + 重力补偿
    set a_up to a_up_n + g_now.

    // ★ 竖直优先分配（B1040-9 同款；avail_kn 单位 kN，mass 单位 t
    //   ⇒ a[m/s2] * m[t] = kN，量纲自洽）
    set t_up   to min(a_up * ship:mass, avail_kn).
    set t_hmax to sqrt(max(0, avail_kn*avail_kn - t_up*t_up)).
    set t_h    to min(a_h_n * ship:mass, t_hmax).
    set t_vec  to upv * t_up + hdir * t_h.

    set thr to t_vec:mag / avail_kn.
    set tdir to t_vec:normalized.   // ★ 沿推力矢量指 —— 不再算 arctan2
}
```

**★ 结构性收益**：`tdir` 由**推力矢量归一化**得到，全程不需要
`arctan2` → `cos/sin` 这条链。上一轮那个"度当弧度"的整段失效
**在结构上不可能再发生**。

---

## 四、参数表（新增）

| 参数 | 建议值 | 来源 |
|---|---|---|
| `eg_tmin` | 4.0 | 照抄 B1040-9（T 下限，防 1/T² 末端炸掉） |
| `aim_alt` | 250. | 段1→段2 的**纯高度门**（去定点后不需要位置门） |
| `term_k` | 2.0 | 照抄（垂直 P，时间常数 0.5 s） |
| `term_v` | 2.0 | 目标触地下降率 |
| `term_kd` | 1.8 | 照抄（水平速度阻尼；**去掉** `term_kp` 位置环） |
| `term_tilt` | 12.0 | 段2 允许的最大推力轴倾角 [度] |
| `term_span` | 25.0 | 倾角收缩到 0 所跨高度 [m] |

> 与 B1040-9 的区别：**`term_kp`（水平位置环）整项删除** —— 那是定点用的。

---

## 五、仿真结果（同一入口 vz=203.5 / vh=408.6 / h=19940 / m=8.282 t）

| 律 | 落地 vz | 落地 vh | 耗燃料 | dv | 动力段 | 结论 |
|---|---|---|---|---|---|---|
| L0 现状 | 2.05 | 0.00 | 2.408 t | 1028 m/s | 364 s | 安全 |
| **L1 方案A** | **2.00** | **0.00** | **2.277 t** | **962 m/s** | 347 s | **安全** |

- 比现状**省 0.131 t（5.4%）**、dv 省 66 m/s
- 油门形状与现状同类（"全程低油门长烧"），**没有**质变

---

## 六、收益与风险

**收益**
1. **单位坑结构性消失**：不再有 arctan2→cos/sin 链。
2. **不需要目标点**：去定点后连经纬度都不需要，只依赖 `alt:radar`。
3. **水平时间常数随高度自适应**（τ_h = T/4），低空自动收急。
4. **同一套代码可切换**：将来要做定点回收，只需把 `r_tgt` 的水平项**加回来**
   —— 非定点/定点共用一个框架。这是方案A 最独特的价值。
5. 复用本项目**已实飞**的结构（B1040-7/B1040-9 段1），不是新造律。

**风险**
1. **高空水平时间常数偏大**（19940 m 时 τ_h=49 s），靠锥限幅兜底 ⇒
   与现状性能接近，**省燃料有限**。
2. `T` 的分母 `max(sqrt(h_aim), vz)` 在"悬停"（vz≈0）时退化，
   必须保留 `eg_tmin` 下限。
3. 段2 的 `tilt_cap` 收缩依赖 `term_span`，取值不当会在低空过度限制横速。
4. `avail_kn` 必须是**当前可用推力**（考虑点火/节流），否则量纲虽对但数值错。

---

## 七、验证步骤

1. 静态：`kos_lint.ps1 munlanding.ks`（本轮沙箱跑不了，需在 VS Code 确认）
2. 结构：`python dev/analysis/verify_munlanding.py`
3. 闭环：`python dev/analysis/sim_recovery_plans.py` 看 L1 行
4. 实机：按 MUNLANDING_PLAN 的分步试飞表（先 3 km 低空 → 20 km 慢速 → 完整剖面）
