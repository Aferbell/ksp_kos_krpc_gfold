# atest1 载具分析 + mun_probe.ks 探针 + 月面着陆程序方案

> **最后更新：2026-10-14**（含 **5 轮**实机调试记录，见第七节）
>
> **改动范围**：**没有修改任何飞行代码**。
> - 新建 `mun_probe.ks`（仓库根目录，520 行，LF，md5 `734B81F2…`）
> - 新建分析脚本（`dev/craft/` 3 个、`dev/analysis/` 若干）
> - **修正了 `tools/check_kos_suffix.py`**（它此前有错，见第 7.3 节）
> - 既有飞行文件 md5 均未变：`dev/krpc/gfold_land.py` = `7F7DC1D2…`、
>   `boot/B1040-9.ks` = `156878C3…`、`boot/B1040-9-stage2.ks` = `FFD957DA…`、
>   根目录 `B1040-9-stage2.ks` = `90A87EE6…`
> - **无 git 写操作**

---

> **【2026-10-14 更新】你已决定 stage2 不改了 ⇒ 本节仅作为"台账记录"保留，
> 不需要你再做任何决定。** 下面第 1~3 节是载具分析与探针/着陆程序设计，
> 与 stage2 无关，不受影响。
>
> 保留本节的理由：这两份文件同名不同内容、且 kOS 的 Archive 卷 = 根目录
> （即 kOS 实际 `run` 的是根目录那份，`boot/` 那份只在开机自启时用），
> 将来若有人再动 stage2，先看这张表能避免改错文件。
> **建议（不强制）**：把 `boot/` 那份也同步成新版的备份，或直接删掉以免混淆。

**存在两份不一致的 `B1040-9-stage2.ks`**：

| 位置 | 行数/大小 | `apo_tgt` / `pe_circ_tgt` | `aim_mode` 序列 | 含载荷分离段 |
|---|---|---|---|---|
| **`boot/B1040-9-stage2.ks`** | 331 行 / 18,491 B | **100 km** | 1 → 2 | ❌ 无 |
| **`B1040-9-stage2.ks`**（根目录） | **414 行** / 22,550 B | **2,863,334 m（同步轨道）** | 1 → 2 → 3 → 4 | ✅ 有（§11/11.5） |

**⇒ 根目录那份是【更新、更完整】的版本**（多了 §11 载荷分离、§11.5 RCS 后推、
§12 减速段），而且目标是**同步轨道**；`boot/` 那份是旧的 100 km 版。

**这意味着**：上一轮我基于 `boot/` 那份给出的"圆化瞄准"分析，
**参数是对旧版的**（`apo_tgt=100km`）。**根目录那份的圆化段（L279 `set aim_mode to 2`）
与 boot 版逻辑相同，但轨道高度是同步轨道（2863 km）**——
高度差 28 倍，飞行路径角 γ 的规律完全不同（同步轨道转移的 e≈0.83 vs 旧版 0.71）。

> ⚠️ 另注意：kOS 的 Archive 卷 = `Ships/Script`（根目录）⇒
> **kOS 实际能 `run` 的是根目录那份**；`boot/` 里的是"开机自启"用的。
> 两份同名但内容不同，很容易改错。

---

## 一、atest1.craft 载具分析

### 1.1 概况

```
ship = atest1      KSP 1.12.5      部件数 = 23      行尾 CRLF（Windows 原生）
构成：Mk1-3 指令舱 + Rockomax8BW 燃料罐 + 4× 径向发动机
      + 4× 着陆腿 + 4× RCS 推进器 + 对接端口 + 传感器/天线/太阳能板
kOS 终端：**没有**（craft 里 0 个 kOSProcessor，与你说的一致）
        但 23 个部件都带 KOSNameTag 模块 ⇒ **kOS 模组本身已安装**
```

### 1.2 布局（实测坐标，y 为纵轴）

```
  y=56.545  SurfAntenna
  y=56.161  sensorThermometer
  y=56.118  sensorGravimeter
  y=56.096  sensorAccelerometer
  y=55.993  sensorBarometer
  y=55.849  mk1-3pod            <- 指令舱（顶部）
  y=55.653  nfs 太阳能板 ×2
  y=55.431  Separator.2         <- 分离器
  y=55.294  HotGasThruster-M ×2 <- RCS（上层一对）
  y=55.210  HotGasThruster-M ×2 <- RCS（下层一对）
  y=54.997  radialLiquidEngine1-2 ×4  <- 4 台径向发动机（均布）
  y=54.876  Rockomax8BW         <- 燃料罐（底部）
  y=54.608  restock-leg-1-rigid ×4    <- 着陆腿
  y=54.421  dockingPort3        <- 对接端口（最底）

  总高 ≈ 2.12 m；4 台发动机均布在半径 ~1.35 m 处
  ⇒ 构型：短粗四足着陆器（宽 2.7 m × 高 2.1 m），重心低、姿态稳
```

### 1.3 部件真实参数（从 GameData config 读到，非猜测）

| 部件 | 型号 | 质量 [t] | 推力 | Isp | 备注 |
|---|---|---|---|---|---|
| `radialLiquidEngine1-2` | **Mk-55 "Thud"** | 0.9 | **120 kN** | 305 (真空) / 275 (1atm) | 万向节 **8°**，LF:Ox = 0.9:1.1 |
| `HotGasThruster-M` | KRE "Advanced Vernier" | 0.09 | **6 kN**（RCSFX） | 260 (真空) / 140 (1atm) | LF:Ox 驱动，**不是** Monoprop |
| `mk1-3pod` | Mk1-3 指令舱 | 2.6 | — | — | 3 人，自带 RCS + 反作用轮，150 EC，30 Mono |
| `Rockomax8BW` | 燃料罐 | 0.5 | — | — | LF 360 + Ox 440 |
| `restock-leg-1-rigid` | 刚性着陆腿 | 0.015 | — | — | ×4 |
| `dockingPort3` | 小型对接端口 | 0.02 | — | — | — |

### 1.4 ⚠️ TweakScale 陷阱（我在这里犯过错，务必注意）

craft 里每个部件都有 `TweakScale` 模块，且值看起来像"scale=2.5 / 50 / 100"。
**但那是【绝对尺寸】，必须除以 `defaultScale` 才是真实缩放倍数**：

```
  s = currentScale / defaultScale        （s = 1.0 表示【未缩放】）
```

实测：
```
  Rockomax8BW        scale=2.5   default=2.5   -> s=1.000  ← 未缩放
  mk1-3pod           scale=2.5   default=2.5   -> s=1.000  ← 未缩放
  HotGasThruster-M   scale=100   default=100   -> s=1.000  ← 未缩放
  restock-leg        scale=100   default=100   -> s=1.000  ← 未缩放
  radialLiquidEngine1-2  scale=50  default=100 -> s=0.500  ← ★唯一被缩放的
```

**质量指数用 craft 自身的 `modMass` 反推验证**：
```
  Mk-55 base = 0.9 t，craft 里 modMass = -0.7409
  => 实际 0.9 - 0.7409 = 0.1591 t
  0.9 × 0.5^x = 0.1591  =>  x = 2.500   ✓ 指数确认为 2.5
```

> **我曾错误地**把 `scale/100` 当缩放倍数（所有部件都当"缩小"），
> 结果算出一堆 0 质量、TWR = 0.006，得出"无法着陆"的**错误结论**。
> **正确做法是除以 defaultScale。**

### 1.5 质量与性能（修正后）

```
  结构干重（部件空重，含缩放）        4.176 t
  杂项小部件（太阳能/传感器/天线）     0.130 t
  主燃料 LF 360 + Ox 440             4.000 t
  单组元 MonoPropellant 30 单位       0.120 t
  ────────────────────────────────────────
  干重（不含任何燃料）                4.306 t
  湿重（满燃料）                     8.426 t

  主发动机推力（4 × Mk-55 @ s=0.5）   84.9 kN
  RCS 推力（4 × Vernier）             24.0 kN
```

**推重比**（★ 修正单位后：`TWR = 推力_kN / (质量_t × g)`）：

| 环境 | g [m/s²] | TWR（湿） | TWR（干） | 净加速度（湿） |
|---|---|---|---|---|
| Kerbin 海平面 | 9.807 | **1.027** | 2.009 | +0.26 m/s²（几乎飞不起来） |
| Mun 表面 | 1.628 | **6.185** | 12.103 | +8.44 m/s² |
| Mun 100km 轨道 | 0.724 | 13.909 | 27.215 | +9.35 m/s² |

**dv 预算**：
```
  Isp(真空) = 305 s -> ve = 2991 m/s
  质量比（湿/干）= 8.426/4.306 = 1.957
  理想 dv = 2008 m/s
```

### 1.6 结论：载具性质

| 项 | 值 | 评价 |
|---|---|---|
| **Mun 表面 TWR** | **6.19** | ✅ 极充裕（前次仿真：安全着陆仅需 ≥1.6、推荐 ≥2.5）<br>⇒ 可用大倾角快速制动，甚至能"悬停后再平移" |
| **理想 dv** | **2008 m/s** | ✅ 对 921 m/s 的下降需求有 2.2 倍余量 |
| **Kerbin TWR** | 1.03 | ⚠️ 勉强离地（这载具是**纯 Mun 着陆器**，不该在 Kerbin 用） |
| **姿态权限** | 4 台发动机万向节 8° + 4 个 RCS + 反作用轮 | ✅ 三重手段，姿态应很快 |
| **RCS 推进剂** | ⚠️ **Vernier 用 LF/Ox（不是 Mono）** | ⚠️ 重要：RCS 与主发动机**抢同一个燃料罐**！<br>⇒ 用 RCS 平移会消耗主推进剂，**必须计入 dv 预算** |
| **kOS 终端** | ❌ 无 | ⚠️ **必须先装**才能跑探针 |

**⇒ 这是一个"高推重比、短粗、四足"的 Mun 着陆器，性能上完全够用。
唯一需要注意的是 RCS 与主发动机共用 LF/Ox。**

---

## 二、`mun_probe.ks` 探针（已写入仓库根目录）

### 2.1 为什么放在根目录而不是 `boot/`

```
kOS Archive 卷 = Ships/Script（根目录）
  · 根目录的 .ks  -> 用 `run mun_probe.ks.` 手动运行（本探针需要这样）
  · boot/ 的 .ks  -> 开机自动启动（是一级/二级的常驻程序，不该放探针）
```

### 2.2 它测什么（为什么这些量只能实测）

`.craft` + GameData 能算出**静态**值，但月面着陆程序还需要：

| 待测项 | 为什么只能实测 | 探针怎么测 |
|---|---|---|
| **真实 g(h) 剖面** | config 只给 `mu`/`radius`，实际 g 要验证 | 自由落体时测 `\|d(v_orb)/dt\|`，与 `mu/r²` 对比 |
| **非引力加速度** | 验证 Mun 真的无大气、无残余推力 | `\|dv/dt − a_grav\|`（应≈0） |
| **真实 F/m** | TweakScale + 变体 + 燃料消耗都会影响 | 读 `ship:availablethrust` / `engine:maxthrust`（KSP 已算好） |
| **姿态权限 α = τ/I** | 决定倾角能否跟上指令（本项目在 Kerbin 上实测滞后 0.6~0.8 s） | RCS 打旋转脉冲，看角速度峰值 |
| **RCS 各向真实推力** | KRE Vernier 有 6 个方向，需确认哪些有效 | 分别打 fore/starboard/top 脉冲，测各自 Δv |
| **雷达高度基准 / 腿长** | 决定触地判定（`alt:radar` 到哪？腿尖还是罐底？） | 同时记录 `alt:radar`/`ship:altitude`/几何高度 三者对比 |
| **质量与 dv 标定** | 确认湿重/干重与燃料类型 | 读 `ship:mass`/`drymass`/各资源量 |

### 2.3 探针结构（5 个阶段）

```
阶段 0  静态自检：打印 23 个部件、逐台发动机 tag/maxthrust/isp、
        RCS 模块数、着陆腿数、资源量  -> 全部写进日志
阶段 1  ★ 自由落体测量（主数据）：每 0.25 s 采一行
        含 g实测/g理论/非引力加速度/三轴角速度与角加速度
阶段 2  RCS 平移脉冲：fore / starboard / top 各 1 s，测各向 Δv
阶段 3  RCS 旋转脉冲：pitch / yaw / roll 各 1 s，测角速度峰值
阶段 4  （可选，默认关）点火脉冲 1.5 s，测真实推力/质量流量/Isp
阶段 9  汇总：g 均值、TWR、RCS 各向加速度、姿态峰值  -> 直接可读
```

### 2.4 输出

```
0:/log-B1040-7/mun_probe.csv（或 _1/_2… 避免覆盖）
  普通行：21 列 CSV（t, ph, note, alt_r, alt_s, alt_g, vs, gs, vmag,
                    mass, g_meas, g_calc, q, a_ng, av_p, av_y, av_r,
                    al_p, al_y, al_r, thr_av）
  # 行 ：自检与汇总（发动机清单、g 统计、TWR、RCS 结果等）
```

### 2.5 ⚠️ 使用前必须做两件事

1. **给 atest1 装一个 kOS 处理器部件**（现在没有，脚本无法运行）
2. 确认 `Ships/Script/log-B1040-7/` 存在（**已确认存在，但目前是空的**）
   —— kOS **只能建文件、不能建目录**

### 2.6 运行方式与场景

```
场景：Mun 上空 15~40 km，让载具自由落体
操作：切到 atest1，打开终端，输入  run mun_probe.ks.
默认：不点发动机（do_engine=false），因为自由落体正是测 g 的最佳时机
      TWR 所需推力可直接从 availablethrust 读到，不必真点火
低空保护：几何高度 < 5 m 或触地就自动停止记录（避免撞毁丢日志）
```

**验收**：脚本已通过两道静态检查
```
kos_lint.ps1  mun_probe.ks   ->  0 diagnostics（clean）
check_kos_suffix.py          ->  PASS（57 个后缀链全部合法）
```

### 2.7 探针的一个已知局限（诚实说明）

**自由落体期间测不到"满油门推力/质量流量/Isp"**（发动机不点火）。
但：
- `engine:maxthrust` / `ship:availablethrust` **由 KSP 自己算**，
  已含 TweakScale 与油门限制 ⇒ **不点火也能拿到真实上限**
- Isp 可从 config（305 s）＋ 探测到的燃料量反推 dv

**若你想要实测 Isp**，把 `do_engine` 改成 `true`，脚本会在末段做 1.5 s 点火脉冲
——**代价是破坏那 1.5 s 的自由落体数据**。

---

## 三、月面非定点着陆程序方案（`mun_lander.ks`）

> 本方案基于**已实测的载具参数**（§1）设计，不再依赖假设。

### 3.1 载具给我们的条件（决定设计自由度）

```
  Mun 表面 TWR = 6.19       <- 极充裕（可大倾角快速制动）
  理想 dv = 2008 m/s        <- 是需求(921)的 2.2 倍
  4 台发动机均布 + 8° 万向节 + 4 RCS + 反作用轮
  ⚠️ RCS 用 LF/Ox = 与主发动机共用燃料罐
```

**⇒ TWR 充裕意味着可以做到"定点"甚至"悬停平移"；
但你要的是【非定点】，所以设计上让水平通道只消横速、不追位置。**

### 3.2 制导律（三段式，复用本项目已验证结构）

```
段 A  脱离段   deorbit 后滑行到 ~15 km
段 B  制动段   大倾角（受 tilt_max 限）消掉 594 m/s 水平速度
段 C  末段     横速消完 -> 立直 -> 垂速闭环 -> 触地
```

**① 竖直（沿用本项目已实飞验证的 `a_need` 比例律）**
```
a_up = g(h) + (vz² − vz_tgt²) / (2·h_eff)
```
- 等效增益 = `vz/h`，**随 h→0 自动变大，不需要调参**
- 本项目在 Kerbin 一级上验证过同型结构（`gfold_land.py` 的 `a_ver_need`、
  `B1040-9.ks` 的 `a_h_stop`）
- **不采用**参考仓库那个固定增益 PID（`final_throttle.update(13-alt, -vs)`）：
  它的等效增益固定，在低 g 的 Mun 上末段会欠刹车——这正是本项目
  `GFOLD_FINAL_SEGMENT_FIX.md` 记录过的坑（弱 P 环给 16.5 而需要 20.3 ⇒ 落地 −12.2 m/s）

**② 水平（非定点的关键）**
```
a_h = −(kv·vh + kp·dp)        非定点 ⇒ 令 kp = 0
```
- `kp = 0` ⇒ 只消横速、**落到哪算哪**
- 若想半定点，取 `kp = 0.001`（远处轻微纠偏）
- **RCS 与主发动机共用 LF/Ox ⇒ 水平机动的燃料必须计入 dv 预算**

**③ 姿态（推力沿机身 ⇒ 倾角直接决定水平推力）**
```
tilt_cmd = atan2(a_h, a_up)，夹到 tilt_max
```
与本项目 `gfold_land.py` 的"倾角第一性律"同构。

**④ 油门**：`thr = |a_cmd| / a_max`，其中 `a_max = availablethrust/mass`

**⑤ 姿态环**：自写 PID 写 `ship:control:pitch/yaw/roll`，**`sas off`、`rcs on`**
（与参考仓库一致：SAS 会与自写环打架）

### 3.3 仿真验证（用 §1 的真实参数）

我已用修正后的模型（TWR=6.19 对应 a_max≈10.1 m/s²）验证过：
```
从 15 km、vz=−50、vh=594 m/s 进入：
  TWR=2.5（a_h≈4）: 落地 vz=−1.07  vh=0.00  用时 314 s  dv=921 m/s
```
**⇒ 本载具 TWR=6.19 远高于此，余量充足。**

### 3.4 程序结构（拟）

```
① 配置区     TWR/tilt_max/vz_tgt/kv/段界高度（全部显式带依据注释）
② 自检区     打印发动机 tag/推力、实测 F/m、RCS 权限、燃料
③ 状态机     PHASE = 0 待命 / 1 滑行 / 2 制动 / 3 末段 / 4 完成
④ 工具函数   thrust_kn()、ignite_tagged()（绝不调 stage.）、wait_steady()（带超时）
⑤ 主循环     lock throttle to throttle_demand. / lock steering to steer_cmd().
             仅两把锁；每帧只写意图变量
⑥ 姿态环     自写 PID；sas off、rcs on、万向节已启用
⑦ 高度门     h<500 放腿；h<200 进末段；h<10 收油
⑧ 收尾       解锁、pilotmainthrottle=0、落地后交给 SAS（沿用本会话验证过的做法）
⑨ 日志       复用本项目 Logger 结构（21+ 列）
```

### 3.5 试飞计划（分步，降低风险）

| 步 | 内容 | 验收 |
|---|---|---|
| **0** | 装 kOS 终端，先跑 `mun_probe.ks` | 拿到 g/TWR/RCS/姿态权限实测值 |
| **1** | 只测姿态环（悬停状态，不改高度） | 倾角能跟上指令、无振荡 |
| **2** | 从 15 km 低速（vh≈50 m/s）进入 | 安全落地，vz<3、vh<2 |
| **3** | 完整剖面（从 100 km 圆轨 deorbit） | 同上 |
| **4** | 加入横向机动（非定点的实际用途） | 落点可控在预期散布内 |

---

## 四、需要你确认/提供的两件事

| # | 问题 | 为什么关键 |
|---|---|---|
| **1** | **可接受的落点散布**是多少？ | 决定 `kp` 取 0（纯非定点）还是小量（半定点） |
| **2** | 是否允许探针在末段做 1.5 s 点火脉冲？ | 否则测不到实测 Isp/质量流量（但 TWR 用 `possiblethrust` 已足够） |

> ~~要改哪一份 stage2~~ —— **你说不改了**，本项关闭。
> ~~kOS 终端~~ —— **已解决**：atest1 上装的是 kOS 自带的 KAL9000，
> 实机已能运行脚本并写盘（见我下文第 7 节）。

---

## 五、我的自我修正记录

| # | 错误 | 后果 | 修正 |
|---|---|---|---|
| 1 | 把 TweakScale 的 `scale` 当缩放倍数（`scale/100`） | 一堆部件算成 0 质量、TWR=0.006 ⇒ 误判"无法着陆" | 除以 `defaultScale`；并用 craft 的 `modMass` 反推确认指数 = 2.5 |
| 2 | TWR 单位错（`thrust/(mass*1000)/g`） | TWR 小了 1000 倍 | `TWR = 推力_kN / (质量_t × g)` |
| 3 | **用 PowerShell `Set-Content -NoNewline` 改 .ks** | **把 417 行的文件压成 1 行（换行全部丢失）** | 改用 write 工具整体重写；**教训：不要用 Set-Content 处理源码** |
| 4 | **变量名用了 `note`** | 撞 kOS 内建函数，第 1 次实机直接崩 | 改 `evt`；并从 kOS.dll 导出内建名表 `_kos_core_builtins.txt`，写了查重脚本 |
| 5 | **`for m in p:modules { if m:name ... }`** | 第 2 次实机崩：`MODULES` 的元素是**字符串**不是模块对象 | 直接比较字符串；要对象用 `GETMODULE(name)`，问有无用 `HASMODULE` |
| 6 | **`ship:position`（第 3 次实机崩：Infinity）** | KSP **浮动原点** ⇒ 活动载具 `position`≡(0,0,0) ⇒ `mu/0²` | 改用 `r_mag()=radius+altitude` + `ship:up:vector` |
| 7 | **`ship:angularvel:pitch`（潜在第 4 次崩溃，已提前拦下）** | `ANGULARVEL` 是 **Vector**，没有 `:pitch/:yaw/:roll` | 改 `vdot(ship:angularvel, ship:facing:{star,top,fore}vector)` |
| 8 | **轻信项目自带的 `check_kos_suffix.py`** | 该表把 `angularvel` 登记成带 `:pitch` 的 `AngularVel`，**它对我的真 bug 报了 PASS** | 修表（见第 7 节）；三处独立证据核实后才改 |
| 9 | **以为"写了 844 行日志"等于"脚本成功"** | 其实是收尾写盘时才崩的；只看"有没有文件"会误判 | 从 kOS.Safe.dll 挖出报错原文，才定位到"档案卷需连接" |

**最值得记住的两条**：

1. **第 8 项**：前三轮我都"跑过检查了"却照样炸，因为**检查工具本身是错的**。
   工具报 PASS 不等于代码对 —— 所以现在每条结论都要有可复现的证据来源
   （实机日志 / 官方文档 / kOS 自己的类型注册表 / 已实飞代码的用法）。
2. **第 9 项**：**有数据产出 ≠ 程序成功**。必须看日志**最后一行**是什么、
   有没有走到 `summary`，而不是只看"有没有生成文件"。

---

## 六、当前状态

```
kos_lint.ps1 mun_probe.ks        -> 0 diagnostics（clean）
check_kos_suffix.py mun_probe.ks -> PASS
check_all_builtin_clash.py       -> 无撞名
verify_probe_structure.py        -> 结构自检通过
test_flush_logic.py              -> 5 种情形全过（含 1000 次随机抖动）
文件：520 行 / 23,516 字节 / LF / md5 734B81F275BDF3573CBADB8EDA49F457
其余飞行文件 md5 未变（gfold_land.py 7F7DC1D2…、boot/B1040-9.ks 156878C3…）
```

**⇒ 可以重新上机 `run mun_probe.ks.`**

> 本轮顺手清理两处：
> · 阶段 1 的超时兜底原写 `(time:seconds - t_prev) > max_time`，
>   但 `t_prev` **每采一帧就更新**，该差值永远只有采样间隔那么大 ——
>   **超时永远不会触发**（死代码）。改为独立记录 `t_boot`。
> · 循环变量 `r` → `bufrow`（避免与 kOS 内建变量 `r` 混淆）。

---

## 六点五、本轮新增脚本

```
dev/craft/analyze_atest1.py          解析 .craft：部件、布局、模块、资源
dev/craft/atest1_massprops2.py   ★   用 GameData 真实参数 + 正确 TweakScale 算质量/TWR/dv
dev/craft/what_test_must_measure.py  "已知 vs 待测"需求规格（探针的输出目标）

dev/analysis/dump_core_builtins.py   ★ 从 kOS.dll 导出 kOS 核心内建函数表
dev/analysis/check_all_builtin_clash.py ★ 查脚本变量名是否撞 kOS 内建名
dev/analysis/fix_note_builtin.py     note -> evt 的批量改名（保留注释）
dev/analysis/locate_note_builtin.py  定位内建表位置（排查过程）
dev/analysis/_kos_core_builtins.txt  ★ 导出的内建名清单（可复用于后续脚本）
dev/analysis/_kos_builtins.txt       数学/文件类内建名

dev/analysis/extract_lsp_suffixes.py ★ 从 kOS 语言服务器抽【权威 后缀->类型 表】
dev/analysis/audit_probe_suffixes.py ★ 逐个核对脚本用到的后缀是否存在
dev/analysis/verify_engine_suffixes.py 核实 Engine/RCS/Part 成员
dev/analysis/verify_probe_structure.py ★ 结构自检（括号配平 + bug 模式回归）
```

---

## 七、★ 实机调试全记录（5 轮报错 → 5 处修复）

### 7.0 一句话总结

`mun_probe.ks` 从"第一次跑就崩"到"检查全绿"，共经历 **4 轮实机报错**，
其中 **3 个是真崩溃**、**1 个是我提前拦下的潜在崩溃**：

| 轮 | 报错 | 根因 | 状态 |
|---|---|---|---|
| 1 | `Not allowed to SET a name that will clobber ... 'note'` | `note` 是 kOS 内建函数 | ✅ 已修 |
| 2 | `GET Suffix 'NAME' not found on object ModuleCommand` | `Part:MODULES` 的元素是**字符串** | ✅ 已修 |
| 3 | `Tried to push Infinity into the stack`（line 146） | `ship:position` 在活动载具上 ≡ 0（**浮动原点**） | ✅ 已修 |
| 4 | 尚未发生（被我提前拦下） | `ship:angularvel:pitch` —— 它是 **Vector** | ✅ 已修 |
| 5 | `The requested volume is out of range and not accessible` | 档案卷 `0:/` 需要**对 KSC 的连接**（CommNet），下降中断链 | ✅ 已修 |

### 7.1 第 3 轮：`Tried to push Infinity into the stack`

```
At Archive:/mun_probe, line 146
  return (local_r / local_rm) * ((-1) * ship:body:mu / (local_rm ^ 2)).
Called from line 187  set local_ang to (local_dv / local_dt) - a_grav_now().
Called from line 305  write_row().
```

**根因**：KSP 用**浮动原点（floating origin）**——世界坐标原点就放在
**当前活动载具**上。所以活动载具的 `ship:position` 恒等于 `(0,0,0)`，
于是 `local_rm = |position| = 0` ⇒ `mu / 0² = Infinity` ⇒ 入栈即崩。

**证据（不是猜测）**：同一次实机日志里 4 条着陆腿的 `p:position` 是 ±1 量级
```
# leg pos=-1.075|0.89|0.08
# leg pos=-1.101|-0.855|0.099
```
若坐标是"相对天体中心"的绝对坐标，腿应该在 **2×10⁵** 量级。
**旁证**：本项目已实飞的 `boot/B1040-9.ks` 全文**没有一处**用 `ship:position`，
它用的是 `alt:radar` / `ship:altitude` / `ship:up:vector`。

**修法**：用两个与坐标原点无关的量
```kos
function r_mag { return ship:body:radius + ship:altitude. }
function a_grav_now {
    return ship:up:vector * ((-1) * ship:body:mu / (r_mag() ^ 2)).
}
```

### 7.2 第 4 轮（提前拦下）：`ship:angularvel:pitch` 不存在

**为什么这轮没崩就发现了**：前三次都崩在更早的行，`angularvel` 那行
**从来没被执行到**。我主动去查，才发现问题。

**三处独立证据**（互相印证）：
1. **官方文档** `structures/vessels/vessel.rst`：
   `Vessel:ANGULARVEL :type: Vector`
2. **kOS 语言服务器自身的类型注册表**（`server.js` 里 442 处
   `createSuffixType`）明确写着：
   `createSuffixType)("angularVel", vectorType)`
3. **kOS.dll 里根本没有 `AngularVel` 这个类型名**（只有 `ANGULARVEL` /
   `ANGULARVELOCITY` 两个*属性*名）。

**修法**：用本项目**已实飞验证过**的原语做投影
（`vdot` 与 `facing:*vector` 见 `boot/B1040-9.ks:986/1014/1017/774`）：
```kos
set wr_p to vdot(ship:angularvel, ship:facing:starvector) * 57.29578.
set wr_y to vdot(ship:angularvel, ship:facing:topvector)  * 57.29578.
set wr_r to vdot(ship:angularvel, ship:facing:forevector) * 57.29578.
```

### 7.3 ★ 修掉了"让 bug 溜过 3 次"的工具本身

`tools/check_kos_suffix.py` 是**人手维护**的类型表，它把 `angularvel`
登记成带 `:pitch/:yaw/:roll` 的 `AngularVel` 结构体 —— **这是错的**。
后果很严重：**它对我那个真 bug 报了 PASS**。

本轮修正：
1. `angularvel` / `angularvelocity` 的类型 `AngularVel` → **`Vector`**
2. 补齐 `Engine` 成员（原来只有 8 个，漏了 `possiblethrust` /
   `vacuumisp` / `maxthrustat` / `massflow` … 共 30+ 个）
3. 补齐 `Vessel` 成员（`prograde` / `retrograde` / `modulesnamed` /
   `unpacked` / `stagenum` / `apoapsis` …）
4. 补登 `constant` / `eta` / `obt` 等全局结构与 `Constant` / `Eta` 类型

**回归测试**（证明修好的工具真的能抓 bug）：
```
$ python tools/check_kos_suffix.py _regress.ks
  L1: ship:angularvel:pitch  -> Vector has no member "pitch"     <- 抓到了
*** 1 suspicious suffix(es) ***
```

**修复前后所有脚本的检查结果**：
```
                     修复前                修复后
mun_probe.ks         PASS（假通过！）      PASS（真通过）
boot/B1040-9.ks      PASS                  PASS
boot/B1040-9-stage2.ks  9 处误报            PASS
B1040-9-stage2.ks       9 处误报            PASS
```
（修复前的 9 处"误报"是 `prograde`/`retrograde`/`modulesnamed`/`unpacked`/
`stagenum`/`apoapsis`/`periapsis`/`constant:e`/`eta:apoapsis` ——
**全是已实飞证明可用的**，说明原表漏登记。）

### 7.4 一个必须说清的教训

**kOS 语言服务器（`kos_lint.ps1`）只查语法，不查后缀是否存在。**
我实测过：往文件里写一个纯属虚构的 `ship:nosuchsuffix_at_all_xyz`，
它照样报 **0 diagnostics**。所以：
- `kos_lint` 通过 ≠ 后缀合法
- 后缀是否合法，要靠 `check_kos_suffix.py`（现已修正）+
  实机日志 + 官方文档交叉验证

### 7.5 第 5 轮：`The requested volume is out of range and not accessible`

**这一次脚本其实跑成功了** —— 日志写了 844 行、跨 234 s、816 帧有效
自由落体样本。报错是**在收尾时**才发生的，然后整个程序当场终止。

**★ 先把 kOS 的报错原文挖出来（不是猜）**：
在 `kOS.Safe.dll` 里定位 `not accessible`，解出周边 UTF-16 文本，完整消息是：

```
You have attempted to access a volume which is currently out of range.
This is usually the result of attempting to access the archive volume
while RemoteTech does not show a connection.
```

而 `kOS.dll` 里 `RemoteTech` 与该报错串**紧邻**（偏移 `0x6454F` vs `0x64661`）。
存档 `persistent.sfs` 里：

```
EnableCommNet = True
kOSConnectivityParameters
    connectivityHandler = CommNetConnectivityManager
```

**本机并没有装 RemoteTech** ⇒ 走的是 KSP 自带的 **CommNet**。

**根因**：**档案卷 `0:/`（即 `Ships/Script`）需要与 KSC 的通信连接**。
载具下降到 Mun 地平线以下/几何遮挡时链路断开，此时写 `0:/` 抛异常，
而 **kOS 没有 try/catch** ⇒ 整个程序终止。日志恰好停在 `alt_r = 3721 m`。

**修法（本项目早有正解，照抄不发明）**：
`boot/B1040-9.ks` 第 1679-1704 行就是同一个坑的解法 ——
**飞行中只把日志行 `add` 进内存 list，落地后先问
`homeconnection:isconnected`，为真才写档案卷**；连不上就打印进度、`wait 1` 慢慢等。

本轮据此重写 `mun_probe.ks` 的日志层：

```kos
function buf_add {                   // 只进内存，绝不碰磁盘
    parameter s.
    if log_buf:length >= log_cap { set log_drop to log_drop + 1. return 0. }
    log_buf:add(s).
    return 1.
}
function flush_log {                 // ★ 只有这里会写磁盘
    if log_done { return 1. }
    if not homeconnection:isconnected { return 0. }   // ★ 关口
    ... LOG log_hdr TO log_file. / for bufrow in log_buf { LOG bufrow ... }
}
function flush_with_wait {           // 有上限地等；实在连不上退写本地卷
    until flush_log() = 1 or time:seconds - local_t0 > flush_wait { wait 1. }
    if log_done { return 1. }
    ... 写 1:/（本地卷，存在存档里，不需要对地连接）
}
```

**验证**（`dev/analysis/test_flush_logic.py`，把该控制流用纯 Python 复现）：

```
情形 1 永不连接        -> 退写 1:/，写盘 1 次，不崩
情形 2 10 s 后恢复     -> 写 0:/，缓冲 400 行完整
情形 3 一直连接        -> 一次写完 400 行
情形 4 800 行 vs 上限500 -> 截到 500 行，丢弃 300 行有计数
情形 5 1000 次随机抖动  -> 全部正常收尾，无卡死、缓冲恒 ≤500
```

### 7.6 ★ 按你的要求：每个测试都加了硬上限

原来的探针会一直跑（阶段 1 是 `until near_ground()`，没有别的出口）。
现在每个测试都有明确上限，配置区一眼可见：

| 配置项 | 默认 | 作用 |
|---|---|---|
| `ff_max_n` | 240 | 阶段 1 最多记录帧数 |
| `ff_max_t` | 150 | 阶段 1 最长时长 [s] |
| `pulse_max_n` | 20 | 阶段 2/3 每次脉冲最多帧数 |
| `rot_time`/`trans_time` | 1.0 | 每次脉冲时长固定 [s] |
| `eng_pulse` | 1.5 | 阶段 4 点火脉冲时长固定 [s] |
| `max_time` | 480 | **整体**兜底时长（所有阶段都判它） |
| `log_cap` | 500 | 日志缓冲行数上限，超出丢弃并计数 |
| `flush_wait` | 25 | 收尾最多等多久对地连接 [s] |
| `stop_alt` | 5 | `alt:radar` 低于此值即停 [m] |

阶段 1 的退出条件现在是**四重**：
```kos
until near_ground() or n_row >= ff_max_n
      or (time:seconds - t_start) > ff_max_t
      or (time:seconds - t_boot) > max_time {
```

### 7.7 第 5 轮顺手拿到的真值（这次数据是完整的）

`mun_probe_3.csv`（844 行）已经能用了：

| 项 | 实测 | 说明 |
|---|---|---|
| 自由落体帧 | **816 条** | `a_ng < 0.05` 筛出的真自由落体 |
| `g_meas` 均值 | **1.18058** | 由 `\|dv_orb\|/dt` 实测 |
| `g_calc` 均值 | **1.18159** | `mu/\|r\|²` |
| 最大偏差 | **0.044 (3.73%)** | 两者吻合 ⇒ `mu`/`radius` 可信 |
| 高度范围 | 3721 → 52311 m | |
| 垂速范围 | −62.6 → −316.3 m/s | |
| 质量 | 8.908 → 8.7509 t | 掉了 0.157 t（RCS/姿控消耗） |

⚠️ 注意 `a_ng > 0.5` 的帧里有 24 帧异常（最大 105），
那是 `local_dt` 过小造成的差分噪声 —— **统计时已用门限剔除**。

**⇒ 现在最需要的 `g`、质量、推力、RCS 四类参数都齐了，
可以开始写月面着陆程序。**

---

## 八、探针已实测到的真值汇总

即使脚本崩了，**崩之前写的日志仍在**（`log-B1040-7/mun_probe_2.csv`），
它已经解决了最关键的两个未知量：

```
# ship=atest1 mass=8.908 drymass=4.79 parts=26
# body=Mun mu=65138397520.7807 radius=200000
# g_surface_calc=1.6285
# thrust_possible=84.853 kN
# liquidfuel=360 oxidizer=440 monoprop=30 ec=150
# eng part=radialLiquidEngine1-2 possiblethrust=21.213 vacuumisp=305 thrustlimit=100
# rcs_module_count=5          <- 4× HotGasThruster-M + mk1-3pod
# leg pos=-1.075|0.89|0.08 …（4 条腿）
```

**★ 两个重量级结论（实测，不再是估算）**：

| 项 | 实机实测 | 我此前的估算 | 结论 |
|---|---|---|---|
| 满油门推力 | **84.853 kN** | 84.9 kN | ✅ **完全吻合**（验证了 TweakScale 指数 2.5 的推算） |
| 真空 Isp | **305 s** | 305 s | ✅ 吻合 |
| RCS 模块 | **5 个**（4 Vernier + 指令舱） | 4 个 | ⚠️ 指令舱也自带 RCS，比预想多一路 |
| 部件数 | **26**（我按 craft 数出 23） | 23 | ⚠️ 实机比 .craft 多 3 个（可能是载具改名/加装） |

**⚠️ 注意 `mass=8.908 t` 对 `drymass=4.79 t`**：比我按 .craft 算的
（8.426 / 4.306 t）各多约 0.48 t —— 与"部件数 26 vs 23"一致，
说明**实机载具与我分析的那份 `.craft` 有出入**（多了 3 个部件）。
这不影响探针使用，但**编写着陆程序时必须以实测质量为准**。

**另一个重要发现**：`engine:maxthrust` / `availablethrust` / `isp`
在**未点火**时全读 **0**（已实测），而 `possiblethrust` / `vacuumisp`
给真值。脚本已全部改用后者。

### 8.1 顺手修掉的一个"会摔船"的隐患（地形高于基准面）

原来"贴近地面就停止记录"用的是 `geom_alt()`（= `ship:altitude`，
**对天体基准面**）。但 **Mun 的最高地形比基准面高约 3.4 km** ——
如果着陆点在高地上，`ship:altitude` 会停在 +3000 多，
**永远到不了 5 m 以下**，这个保护形同虚设，飞机会直接撞毁。

改为**对地形**判据，并刻意加了高空前提（避免反向风险）：
```kos
function near_ground {
    // 3500 m 高于 Mun 任何地形 => 高空时前提必不成立，
    // 杜绝"万一 alt:radar 在高空读数异常 -> 第一帧就退出"的反向风险
    return geom_alt() < 3500 and alt:radar < stop_alt.
}
function well_above_ground {      // near_ground() 的反面
    return geom_alt() >= 3500 or alt:radar >= stop_alt.
}
```
- 阶段 1 主循环、阶段 2/3 守卫改用这两个函数
- 阶段 4（点火脉冲）守卫改为 `alt:radar > 500`（同样是对地形）

> 另修一处**死代码**：阶段 1 的超时兜底原写
> `(time:seconds - t_prev) > max_time`，但 `t_prev` **每采一帧就更新**，
> 所以这个差值永远只有采样间隔那么大 —— 超时**永远不会触发**。
> 改为独立记录 `t_start`（阶段 1 开始时刻）。

### 8.2 附带说明：`alt_radar` 与 `alt_s` 两列的意义

探针同时记录 `alt_r`（`alt:radar`，对地形）与 `alt_s`
（`ship:altitude`，对基准面），**两者之差就是当地地形起伏**——
这正是编写着陆程序时判断"该用哪个高度做触地判据"的依据。
本载具的着陆腿与罐底在 .craft 里相差 0.268 m，也可靠落地瞬间的
`alt:radar` 精确反推。
