# starship.ks 适配 starship.craft（2026-10-21）

> 任务：`boot/starship.ks` 由旧 `B1040-9-stage2.ks` 改来，
> 新载具 `saves/起源/Ships/VAB/starship.craft` 的**二级引擎 tag 换成了三个 `ee1` + 三个 `ee2`**，
> 程序没有适应，需要读懂逻辑后修改。

> 验证脚本：`dev/analysis/verify_starship_ignition.py`、`dev/analysis/check_starship_ks.py`

---

## 0. 结论

**原程序在新载具上只会点亮 6 台里的 2 台（缺 4 台）。**

| 写法 | `ee1` | `ee2` | 合计 |
|---|---|---|---|
| **原程序**（找到第一台就 `return`） | 1/3 | 1/3 | **2/6** |
| **修改后**（`ignite_all` 逐组点满） | 3/3 | 3/3 | **6/6** |

---

## 1. 新载具的实际结构（craft 实测）

`starship.craft` 共 **175 部件**，4 个分级：

| `istg` | 件数 | 内容 |
|---|---|---|
| **2** | 37 | 33 × SSME（`tag = e1..e33`，一级）+ 4 × `launchClamp1` |
| **1** | 61 | `restock-separator-5-1` + `restock-decoupler-1875-truss-1`（两个 `ModuleDecouple`，**均无 tag**）+ 6 × `sepMotor1`（分离火箭）+ `KAL9000_4292984650`（二级 kOS 核）+ 油箱/散热板/货舱 |
| **0** | 6 | **6 台二级发动机**：`ee1` ×3（SSME）+ `ee2` ×3（`restock-engine-galleon-1`）|
| **-1** | 71 | 一级本体：`KAL9000_4292470356`（一级 kOS 核）+ 4 条腿 + 32 × `HotGasThruster` + 气动面 + 油箱 |

**两个 kOS 核**（`istg=-1` 与 `istg=1`），所以「核数量 2 → 1」的交接判据**仍然成立**，未改动。

**二级的 RCS**：`istg=1` 有 24 台 `HotGasThruster-L`（`ModuleRCSFX`），`rcs on` 有效。

---

## 2. 三处必须改的地方

### 2.1 核心：`ignite_tagged()` 只点第一台

**原代码**：

```
function ignite_tagged {
    parameter tg.
    list engines in engs.
    for e in engs {
        if e:tag = tg {
            if not e:ignition { e:activate(). }
            return e:ignition.        // <-- 找到第一台就返回
        }
    }
    return false.
}
```

这段代码是为**旧载具的单台 `engine2`** 写的，在旧载具上完全正确。

**在新载具上的后果**：`ee1` 有 3 台，只点第 1 台就 `return`：

```
ee1: 1/3 点亮   ee2: 1/3 点亮   合计 2/6
```

后果三重：

| 后果 | 说明 |
|---|---|
| 推力只剩 1/3 | 爬升段远地点涨得极慢，可能永远到不了 `apo_tgt` |
| 推力轴偏斜 | 三联装只点一台 ⇒ 净推力偏离轴线 ⇒ 被推歪，`wait_steady` 也稳不住 |
| `thr_full` 偏小 | 火箭方程算出 `t_burn` 偏长 ⇒ 圆化烧过头 |

**修改后**：

```
function ignite_tagged {
    parameter tg.
    set n_hit to 0.  set n_on to 0.
    list engines in engs.
    for e in engs {
        if e:tag = tg {
            set n_hit to n_hit + 1.
            if not e:ignition { e:activate(). }
            if e:ignition { set n_on to n_on + 1. }
        }
    }
    if n_hit = 0 { return false. }
    return n_on = n_hit.      // 全部点亮才算成功
}

function ignite_all {
    set all_ok to true.
    for t in eng_tags {
        if not ignite_tagged(t) { set all_ok to false. }
    }
    return all_ok.
}
```

`activate()` 是幂等的，重复调用无副作用。

### 2.2 `thrust_kn()` 会把分离火箭算进来

**原代码**累加「本帧所有已点火引擎」：

```
for eng in engs {
    if eng:ignition and not eng:flameout {
        set thr_sum to thr_sum + eng:thrust.
    }
}
```

旧载具上没问题（第三级在扔载荷前没点火）。**新载具 `istg=1` 有 6 台 `sepMotor1`**，
分离事件会把它们点着 —— 一旦分离后再测推力，它们就会被算进来：

```
thr_full 高估  =>  t_burn 偏短  =>  圆化烧不够
```

**修改**：加 `tag_in_list(eng:tag)` 过滤，只累加 `eng_tags` 里的那几台。

### 2.3 `sep_tag` 已失效

原 `sep_tag = "separator2"`。`starship.craft` 的两个 `ModuleDecouple`
（`restock-separator-5-1_4291723648` / `restock-decoupler-1875-truss-1_4291663776`）
**都没有设 `nameTag`**，按 tag 找必然找不到。

本程序**从不真的执行分离**（`sep_tag` 只用于自检打印），所以直接删除该配置项与检查。

---

## 3. 配置区改动

```
// 原
local eng_tag      is "engine2".
local sep_tag      is "separator2".

// 新
local eng_tags     is list().          // 二级发动机 tag 列表
// §2b 末尾填充：
eng_tags:add("ee1").                   // 3 x SSME
eng_tags:add("ee2").                   // 3 x restock-engine-galleon-1
```

**为什么用 `list()` + `:add()` 而不是 `list("ee1","ee2")`**：
仓库里 12 处 `list(` 先例**全部是空构造后 `:add()`**，没有变参写法，
不确定 kOS 的 `LIST()` 是否支持变参，故沿用已验证的写法。

**为什么 `:add()` 放在 §2b 而不是配置区**：保持文件的「所有 `local` 声明在前、
可执行语句在后」风格（静态检查已确认修改后仍然满足）。

---

## 4. 自检强化

原自检只查单个 tag：`if ship:partstagged(eng_tag):length = 0`。
新载具 6 台分两组，**3 台缺 2 台也看不出来**。改成逐组统计：

```
本程序要点亮的发动机：合计 6 台，推力上限 X kN
```

并把每组单独打印：`ee1：级号 = 0，台数 = 3，合计推力上限 = X kN`。

---

## 5. 验证

### 5.1 用 craft 真实数据仿真的点火结果

```
=== 新写法 ignite_all（本次修改）===
    ee1  3/3
    ee2  3/3
    合计 6 / 6 台      判定 全亮 OK

=== 旧写法（找到第一台就 return）===
    ee1  1/3
    ee2  1/3
    合计 2 / 6 台      判定 **缺 4 台**
```

### 5.2 静态检查

```
check_kos_suffix.py boot/starship.ks  = ALL PASS（34 条后缀链，无可疑）
花括号配平                            = 43 { / 43 } 平衡
local 声明顺序                         = 所有 local 都在可执行语句之前
6 个函数定义 vs 调用                    = 全部有调用点，无死代码
```

---

## 6. 未改动的部分（以及为什么）

| 项 | 状态 | 理由 |
|---|---|---|
| 交接判据 `kOSProcessor:length = 1` | **未改** | 新载具实测确有 2 个 kOS 核，判据成立 |
| `apo_tgt = 100000` / `pe_circ_tgt = 100000` | **未改** | 低轨 100 km，与原改派值一致 |
| `apo_hi_k = 1.1` | **未改** | 兜底倍数，与 100 km 目标配套 |
| `ecc_tgt = 0.01` / `pe_min = 70000` | **未改** | 形状闸，与 100 km 圆轨道配套 |
| 姿态律（`steer_cmd` / `wait_steady`） | **未改** | 与载具无关 |
| 圆化燃烧时长算法 | **未改** | 全实测（`thr_full` / `flow_m`），自动适应新载具 |
| 整流罩 `AG9 on` | **未改** | 新载具无整流罩时该动作静默失败，不影响主流程 |

> ⚠️ **整流罩那条要留意**：`starship.craft` 里没找到 `ModuleProceduralFairing`。
> `AG9 on` 对不存在的动作组不会抛错（kOS 只是无效果），但若实际有整流罩而动作组编号变了，
> 需要实飞时在控制台确认。**这是遗留待观察项，不是本次修改范围。**
