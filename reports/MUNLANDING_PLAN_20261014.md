# munlanding.ks 方案（月面非定点着陆）+ mun_probe_4 数据判定

> **状态**：程序已写好并通过全部静态检查，**已上机一次（失败，根因已定位并修复）**。
> **没有修改任何既有飞行代码**（`gfold_land.py` / `boot/*.ks` 的 md5 未变）。
> **无 git 写操作。**

---

## 零、★ 第一次实机失败：刚点火就报"已触地"

### 0.1 现象
```
到达触发高度 19940 m，开始制动
已点火 4 台发动机
已触地：下降 0 m/s，横速 0 m/s      <- 其实还在 19.8 km 下坠！
RCS 保持开，SAS 已开（radialout，径向向外）
```

### 0.2 根因（日志可证，不是猜）

看 `log-B1040-7/munlanding_1.csv`：
```
# munlanding 入口: alt_radar=19940.3 vz=203.54 vh=408.57 mass=8.282
# ===== SUMMARY =====
# status=SUB_ORBITAL alt_radar=19834.39
# final_vz=203.83 final_vh=408.77
# buffered_rows=7 dropped=0        <- 7 行 = 入口 3 行 + 汇总 4 行
```
**主循环零行数据 ⇒ 一次都没执行。** 原因是**用了未初始化的变量做循环判据**：

```kos
local h_now is 0.                          // 声明初值 = 0
...
until h_now < h_touch or ... {             // ← 这里读 h_now
    guidance().                            // ← h_now 只在这里面被赋值
```
`until` 第一次判定读到的是**声明初值 0**，而 `0 < 2.5` 恒真 ⇒ **循环零次执行**。
随后落地判定 `h_now < h_touch and abs(vz_now) < 8` 用到同样没赋值的 0，
于是打印出"已触地：下降 0 m/s"。

**kOS 语言服务器查不出这个错**（符号已声明、类型也对），
`check_kos_suffix.py` 更查不出 ⇒ 这就是为什么三轮静态检查全绿却一上机就死。

### 0.3 修法（两道一起上）

```kos
set loops to 0.
guidance().                    // ① 预热：进循环前先调一次，把所有状态量填好

// ② 判据【直接读传感器】，不再依赖缓存变量 ——
//    这样即使将来某条赋值路径漏了，也不会再"用初值做判据"
until alt:radar < h_touch or ship:status = "LANDED"
      or (time:seconds - t_brake) > max_time {
    set loops to loops + 1.
    guidance().
```

并把落地判定也改成直接读传感器：
```kos
if ship:status = "LANDED" { set landed to true. }
if alt:radar < h_touch and abs(ship:verticalspeed) < 8 { set landed to true. }
if loops = 0 { print "*** 异常：主循环一次都没执行（loops=0）***". }
```

### 0.4 ★ 我为此写了一个专门的检测器，并**先证明它有效**

这类"用未初始化变量做判据"的错，现有工具都查不出。所以写了
`dev/analysis/find_uninit_guards.py`，判据是：

> 对每个循环/分支 G：
> 读集合 R = G 条件里的变量；
> 写集合 W = G 体内 set 的变量 ∪ **体内调用过的函数（含传递闭包）**所 set 的变量；
> 若 v ∈ R ∩ W 且 v 在 G 之前**从未真正被赋值** ⇒ 可疑。

**★ 这个检测器我写坏了两版，都被我自己的测试抓住：**

| 版本 | 缺陷 | 后果 | 怎么发现的 |
|---|---|---|---|
| v1 | 只看 G 自己的花括号体 | **对真的含 bug 的代码也报 0 个问题**（赋值发生在 `guidance()` 里） | 写 `test_uninit_detector.py`，用复现原 bug 的最小样例喂它 ⇒ 它没报 |
| v2 | 用"行号更小"判断"此前是否赋值" | 把 `flush_log` 里 `set log_i to 0.` 紧接 `until … log_i` 这种**正确**写法误报（9 个误报） | 修好后跑真实文件，看那 9 个是不是真问题 |
| v3 | 按【函数作用域 + 行号】判断 | 样例 A 抓到 ✓、样例 B 放行 ✓、真实文件 0 误报 ✓ | — |

**为什么要费这个劲**：一个"永远报没问题"的检测器，比没有检测器**更危险**
—— 它会让我以为已经检查过了。前两轮我已经被"工具报干净"骗过两次
（`check_kos_suffix.py` 的错误类型表、`ksconfig.json` 的规则分组），
所以这次**先证明工具有效，再用它下结论**。

`test_uninit_detector.py` 已固化在仓库里，以后随时可回归。

---

## 一、mun_probe_4 数据判定：哪些可信、哪些不可信

你说"有些数据有问题"。逐项核实结果如下（判据都来自同一次运行的
**互相独立**的量能否对上）：

| 项 | 判定 | 依据 |
|---|---|---|
| **重力 g** | ⚠️ **本次没测到** | 自检行显示 4 台发动机 `ignited=True` —— **本来就点着火**，不是自由落体。我的 `g` 统计门限（`a_ng<0.05`）把所有帧都剔掉了 ⇒ `g_samples=0`。**不是坏了，是场景不满足**。g 用 `mu/(R+h)^2` 即可，不必实测。 |
| **满油门推力** | ✅ **可信（两条独立路径互证）** | `thr_pos` 全程只有一个值 **84.850 kN**，与自检行 `ship:availablethrust=84.853 kN`（这次点火了所以有值）一致。 |
| **质量 / 干重** | ✅ **可信** | 直接读数：湿 8.7014 t、干 4.7895 t。飞行中降 0.0495 t。 |
| **Isp** | ✅ 可信 | 4 台全报 `vacuumisp=305 s`。 |
| **RCS 平移** | ⚠️ **量级可用，方向占比没测干净** | 原始数 前1.30 / 侧2.39 / 上2.39 m/s。**没扣脉冲期间的重力**（该高度 g≈1.198）：扣掉后侧/上约 **2.07 m/s²**。而 `fore` 那次 `gs` 全程≈0.01、`vs` 从 −12.37 变到 −11.07 ⇒ 速度变化**几乎全在竖直方向**，说明那一刻机体"前向"≈铅垂向上，**不能用正交公式**。⇒ 只能当量级参考。 |
| **姿态权限** | ❌ **本次没测到，不可用** | `rot_peak_av_pitch=0.255 deg/s`，而阶段 3 原始行里 `av_p` 全程在 ±0.5 deg/s 内 —— 说明**脉冲几乎没把船转起来**。（全飞行 `av_p` 范围 −92.9…72.2 出现在**阶段 1 的长期翻滚**里，不是脉冲测的。）可能原因：反作用轮/SAS 在对抗、或 1 s 脉冲太短。 |
| **角加速度 `al_*`** | ❌ **不可信** | 用相邻两帧差分求得，而采样间隔 0.5 s、真实物理帧 0.02~0.2 s ⇒ 差分放大噪声。范围 ±98~114 deg/s² 明显是噪声。要用请改**线性拟合斜率**。 |

**⇒ 可直接用于程序的有 4 类**：
```
推力 84.853 kN（4x Mk-55，含 TweakScale 与限流）
湿重 8.7014 t / 干重 4.7895 t / Isp 305 s
Mun: mu=6.5138398e10, R=200000, g_surf=1.62846
可用 dv = ve*ln(m0/mdry) = 2991*ln(8.7014/4.7895) = 1786 m/s
表面 TWR = 5.99
```

---

## 二、munlanding.ks 流程（按你的要求）
```
阶段 0  等待：什么都不做，直到 alt:radar < 20000 m
         （每 0.5 s 打印一次高度/垂速/横速，让操作者知道还活着；
           有 max_wait=3600 s 兜底）
阶段 1  点火 + 建立两把唯一的锁（throttle / steering）
         记录入口状态，若入口垂速 > 600 m/s 打印告警
主循环  制导（见下）→ 横速 < 2 m/s 时切末段 → h < 250 m 放腿
阶段 9  收油、解锁；若确认触地则 RCS 保持开 + SAS 径向向外
         （"radialout"，与 kRPC 侧的 SASMode.radial 对应）
```

### 2.1 制导律（倾角第一性）

```kos
竖直：a_up = g(h) + (vz^2 - vz_tgt^2) / (2*h_eff)     // 等减速度剖面
水平：a_h  = vh / T_h                                  // 指数消横速
★ 推力锥限幅：a_h <= sqrt(a_max^2 - a_up^2)            // 竖直优先
倾角：tilt = atan2(a_h, a_up)，夹到 tilt_max
★ 倾角限速率：每帧最多变 tilt_rate*dt
推力方向 = up*cos(tilt) + hdir*sin(tilt)，hdir = -水平速度方向
油门：thr = sqrt(a_up^2 + a_h^2) / a_max
```

**为什么竖直用这个式子而不是 PID**：它是等减速度剖面的解析解，
等效增益 `vz/h_eff` 随高度趋零自动变大，**不需要调参**。
参考仓库用的是固定增益 PID（`final_throttle.update(13-alt, -vs)`），
在 Mun 这种低 g 环境末段会不够力 —— 这正是本项目
`GFOLD_FINAL_SEGMENT_FIX.md` 记录过的坑，所以**不照抄它的调参**。

### 2.2 ★ 非定点 = 只消横速、不追位置

参考仓库 `starShipRecovery.py:163-172` 的做法是：横速 > 50 就压倾角，
否则立直，**全程不做位置闭环**。本程序同思路，但把那个 50 m/s 的
**硬开关写成连续律**（`a_h = vh/T_h`），避免跳变。

---

## 三、★ 仿真查出的真缺陷（不是猜，是复现出来的）

我把最终代码逻辑逐条搬进 Python 闭环跑，**发现并修掉了一个会让船摔的
设计错误**：

### 3.1 缺陷：高垂速 + 高横速时，竖直与水平抢推力

从 20 km、`vz0=500`、`vh0=300` 入场：
```
制导要求：竖直 a_up = 7.58 m/s^2
          水平 a_h  = vh/T_h = 37.5 m/s^2
=> tilt = atan2(37.5, 7.58) = 78.6 度，夹到 70 度
=> |a| = 7.58/cos(70) = 22.2，而 a_max 只有 9.75
=> 油门顶死 1.0，实际竖直分量仅 9.75*cos(70) = 3.33 << 需要的 7.58
=> 刹不住，落地 vz = 174 m/s（摔）
```
**物理本质**：倾角 θ 时竖直只能拿到 `cos θ` 的推力 —— **θ=70° 只剩 34%**。

### 3.2 我第一版的错误修法（也复现了，也摔）

我原先写的是"指令直接跳 + 姿态误差超限就砍油门到 35%"。结果：
**入场时机头朝上、指令一下跳到 70°，`att_err` 立刻 70° > 12°**，
油门被砍到 35% ⇒ 更刹不住。仿真里 `vz0=500` 照样摔。

### 3.3 正确修法（已写进程序）

1. **推力锥限幅**（本项目 `gfold_land.py` 的"倾角第一性"同款）：
   竖直优先，水平预算取 `sqrt(a_max² − a_up²)` ⇒ **油门永不饱和**，
   竖直通道始终满足，代价是横速消得慢些。
2. **倾角指令限速率** `tilt_rate = 12 度/秒`：让实际姿态总能跟上，
   `att_err` 自然很小 ⇒ 姿态保护（`att_tol=25°`, 折减到 60%）退化为
   **最后一道保险**，不再误伤。

**修法验证对比**（同一批 12 个案例）：

| 写法 | 不安全案例 |
|---|---|
| 旧：无锥限幅 + 跳变指令 + 砍油门35% | **6 / 12** |
| 新：锥限幅 + 限速率12°/s | **明显改善**（见下节包线） |

---

## 四、仿真给出的真实安全入口包线

从 `alt:radar = 20000 m` 开始制动，判据「真的落地 + 落地 vz<3 + vh<2」：

```
  vz0\vh0     0   100   200   300   400   500   600   700   800   900  1000
  0         超时  超时  超时  超时  超时  超时  超时  超时  超时  超时  超时
  50         2/0  超时  超时  超时  超时  超时  超时  超时  超时  超时  超时
  100        2/0   2/0  超时  超时  超时  超时  超时  超时  超时  超时  超时
  150        2/0   2/0   2/0   2/0   2/0  超时  超时  超时  超时  超时  超时
  200        2/0   2/0   2/0   2/0   2/0   2/0   2/0  超时  超时  超时  超时
  300        2/0   2/0   2/0   2/0   2/0   2/0   2/0   2/0   2/0   2/0   2/0
  400        3/0   2/0   2/0   2/0   2/0   2/0   2/0   2/1    XX    XX    XX
  500        3/0   3/0   3/0   3/0   3/1    XX    XX    XX    XX    XX    XX
  600         XX    XX    XX    XX    XX    XX    XX    XX    XX    XX    XX
```

**★ 推荐入场条件（写进程序注释与告警）**：
```
最稳：vz0 <= 150 m/s，vh0 <= 500 m/s
可用：vz0 <= 300 m/s，vh0 <= 300 m/s
切忌：vz0 > 400 且 vh0 > 400  —— 竖直与水平抢推力，必摔其一
上限：vz0 <= 600 m/s（再大任何横速都刹不住）
```

> **"超时"含义**：那些格子是**垂速太小**（如 vz0=0 从 20 km 慢慢飘），
> 900 s 内没落地 —— 属于"太保守"，不是危险。真实飞行不会这样入场。

**dv 核对**：包线内最大需求 1676 m/s，可用 1786 m/s ⇒ **余量 1.07 倍**
（偏紧）。若入场条件收紧到「vz0≤300, vh0≤300」，需求约 1367 m/s，
余量升到 **1.31 倍**，更稳妥。

---

## 五、程序里内置的三道安全设计

1. **入口越界告警**：`vz0 > 600` 时打印并写进日志，事后能追责。
2. **姿态保护**：姿态误差 > 25° 时油门折减到 60%
   （限速率之后它很少触发，只在异常时兜底）。
3. **日志缓冲 + 连上才写**：沿用你项目的做法 —— 档案卷 `0:/` 需要与
   KSC 的连接（CommNet），断链时写会抛异常且 **kOS 无 try/catch**
   会当场终止程序。所以日志**先缓冲在内存**（上限 1200 行），
   落地后先问 `homeconnection:isconnected`，连不上就退写**本地卷 `1:/`**，
   等连接最多 30 s（有上限）。

---

## 六、静态检查结果（全过）

```
kos_lint.ps1 munlanding.ks      -> 0 diagnostics
check_kos_suffix.py             -> PASS
check_all_builtin_clash.py      -> 无撞名
verify_munlanding.py            -> 结构核对通过
check_throttle_teardown.py      -> throttle 收尾写法正确
  · 括号配平（{} / ()）
  · 无 goto / 无语句标号 / 无 `set local X is`
  · 无 ship:position / 无 angularvel:pitch
  · 全部 LOG 都在 flush_log / flush_with_wait 内
仿真自检                         -> 与解析等减速度解差 0.03%
```

### 6.1 ★ 你截图报的那个错（已修）+ 我的检查为什么漏了它

**你截图里的 3 条诊断都指向同一个根因**（`munlanding.ks:415-416`）：

```
L415 symbol-wrong-kind: Expected a variable or parameter
                         but found a Lock instead
L415 Global variable throttle was not used locally.  （附带现象）
L416 symbol-wrong-kind: Expected a lock but found a Variable instead
```

**真错在哪**：第 362 行 `lock throttle to thr.` 之后，`throttle` 就是一个
**Lock**，不再是变量。而我在收尾时写了：
```kos
set thr to 0.
set throttle to 0.        // ★ 错：Lock 不能被 set
unlock throttle.
```
第 3 条（L416 "found a Variable instead"）是**连带现象**：
因为上面错误地 `set` 了一次，类型推断把 `throttle` 当回变量，
于是 `unlock throttle` 反而抱怨"要 Lock 却给了 Variable"。

**修法**（照抄本项目已实飞的 `boot/B1040-9.ks:1664-1669`）：
```kos
set thr to 0.                              // 1. 意图变量清零
unlock throttle.                           // 2. 先解锁
unlock steering.                           // 3. 再真正归零
set ship:control:pilotmainthrottle to 0.
```
理由：`lock throttle to thr` 的语义是"throttle 读 thr"，
所以真正控制油门的是 `thr` 这个**意图变量**；归零它并解锁后，
再用 `pilotmainthrottle` 把飞行员主动油门也归零。

### 6.2 ★ 更严重的问题：我的检查脚本**漏掉了这个错**

**这是本次最值得记的教训。** 我先前跑 `kos_lint.ps1` 一直报
`0 diagnostics`，于是以为干净 —— 但你在 VS Code 里能看到这 3 条。

**根因在 `ksconfig.json`**：kOS 语言服务器的 lint 规则是**分组**的，
`symbol-wrong-kind` 属于 `symbols-strict` 组。而我的配置里：

```jsonc
"symbols-strict": "off",        // 组被关掉
"symbol-unused": "warning",     // 单独开了这条
"symbol-shadowing": "warning",
"symbol-conflict": "error",
// ← 但没有 symbol-wrong-kind
```

**组设成 `off` 时，组内规则不会继承**；只有我**显式列出来**的那几条才生效。
`symbol-wrong-kind` 没被列出 ⇒ **这条检查从来没跑过**。

对照 `server.js` 里的真实分组（已从扩展里核出）：
```
symbols-strict (off)
  ├─ symbol-may-not-exist      warning  ← 我列了
  ├─ symbol-may-not-exist-closure warning ← 我没列
  ├─ symbol-wrong-kind         warning  ← 我【没列】 => 漏检的就是它
  ├─ symbol-unused             warning  ← 我列了
  ├─ symbol-unused-locally     warning  ← 我列了(off)
  ├─ symbol-shadowing          warning  ← 我列了
  └─ symbol-conflict           warning  ← 我列了(error)
```

**已修**：在 `ksconfig.json` 里加 `"symbol-wrong-kind": "error"`。
加上之后，同一个文件立刻报出 2 条 ERROR（就是上面那两条），
**证明修复有效**。

### 6.3 修完配置后，顺带扫出另一处**既有**问题（不在本次改动范围）

用修好的配置重新扫全部 kOS 脚本：

```
mun_probe.ks            0
boot/B1040-9.ks         88（22 个 WARN，全是已停用日志段的 unused 局部变量）
boot/B1040-9-stage2.ks  0
B1040-9.ks              0
B1040-7.ks              0
B1040-9-stage2.ks       4  →  ERROR L349:15 [type-wrong] May not be a valid enumerable type
```

根目录 `B1040-9-stage2.ks:349`：
```kos
for mn in sep_list[0]:modules {
```
`sep_list[0]:modules` 返回的是**模块名字符串列表**；语言服务器对
"索引取值再取后缀"这种链式写法推断不出可枚举类型，所以报
`type-wrong`。**但这段逻辑本身是对的**（我在 `mun_probe.ks` 里
验证过：`Part:MODULES` 的元素就是字符串）。
⇒ 属于**误报**，而且**这个文件你说过不改**，所以我**没有动它**，
仅在此记录，供你判断。

> ⚠️ 注意：`check_py.ps1` 的第 4 步默认只查 `boot/B1040-9.ks`，
> 所以它仍报"全部通过" —— 它**不会**去查根目录那两个脚本。

---

## 七、试飞建议（分步，降低风险）

| 步 | 内容 | 验收 |
|---|---|---|
| **0** | 先在**低空**（如 3 km）用小垂速试 | 能点火、能切末段、能落 |
| **1** | 20 km、vz0≈100、vh0≈100 进场 | 落地 vz<3、vh<2 |
| **2** | 20 km、vz0≈150、vh0≈300 | 同上 |
| **3** | 完整剖面（deorbit 后自然入场） | 同上 |
| **4** | 加横向机动（非定点的实际用途） | 落点可控 |

---

## 八、仍缺的一项数据（诚实说明）
**姿态权限（RCS 转动能力）本次没测到**。这决定了 `tilt_max=70°` 追不追得上。
我在仿真里扫了姿态滞后 `tau = 0.2…1.2 s`：**全部安全落地**
（限姿态帧数从 6 增到 41，但因油门折减兜住）。
所以**即使姿态比预期慢，也不会摔**——但会更费燃料。

**若你想补测**：让探针的旋转脉冲改成"先 `sas off` + 加大脉冲时长到
3 s"，或直接测 `ship:availabletorque` 与 `ship:momentofinertia`
（不过这两个后缀在本机 kOS 里没找到，需实测确认）。
