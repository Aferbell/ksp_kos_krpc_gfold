# starship 一级回收（kRPC + G-FOLD）

> 新建于 2026-10-20。**不改动旧的 dev/solver 与 dev/krpc**（旧载具仍在用）。

## 为什么单独一套

旧 `dev/solver/gfold_codegen.py` 把 `MASS_GEN = 183000` 写死成编译期常量
（`z0i/z0l` 用 numpy 常数是 cvxpy 1.9.3 的 DCP 硬约束）。
starship 一级交接质量 **977.5 t** 是它的 **5.3 倍**，`s_hi0/s_lo0`（运行时按真实质量算）
与展开点严重失配 ⇒ codegen 求解器 `status=1`（15 ms 就返回）⇒ 回退 cvxpy 2.3 s/次
⇒ P3 + tf 探测 + P4 累计 **4436 ms**（`gfold_log_20261020_132426.csv` 的 `solve_ms`）。

本目录重新生成了一套 codegen（`MASS_GEN=977000`、`T_MAX=17.5e6`），
实测 **8~20 ms/次，全流程 70~340 ms**。

## 文件

| 文件 | 作用 |
|---|---|
| `starship_config.py` | 一级实测参数、交接门、控制器增益（**唯一参数入口**） |
| `starship_land.py` | 主程序：关引擎 → 等门 → 解算 → 闭环跟踪 |
| `starship_p3p4.py` | P3/P4 求解器封装（改自 `dev/solver/gfold_p3p4.py`） |
| `starship_codegen.py` | cvxpygen 生成器（改自 `dev/solver/gfold_codegen.py`） |
| `cpg_p3/`、`cpg_p4/` | 已编译的 C 求解器（`.pyd`） |
| `analyze_first_stage.py` | 从 `starship.craft` 解析一级参数 |
| `_scan_final.py` | 交接门可行域扫描（产出 config 里的门参数） |

## 载具（一级 = 分离器 y<71.7 m 以下）

| 项 | 值 |
|---|---|
| 部件 / 干重 / 推进剂 / 湿重 | 101 / **628.5 t** / 3732.5 t / **4361.0 t** |
| 33 台 SSME @250% | 82.5 MN（TWR 湿 1.93） |
| **回收保留 7 台**（e1,e2,e3,e5,e8,e10,e13） | **17.5 MN** |
| 交接 977.5 t 时 TWR / a_net | 1.82 / 8.09 m/s² |
| 横向惯量 | 1.6e9 kg·m² |
| gimbal 角加速度 | 3.8 °/s² |

> **起飞必须 33 台**：7 台在 4361 t 满油时 TWR=0.41，飞不起来。
> 关引擎发生在**回收程序启动时**，不是发射时。

## 交接门（扫描定）

```
alt < 517 m (TARGET_ALT 17 + AIM_ALT 500)
|v| < 120 m/s
vz  < 100 m/s      ← 新增，667 m 处的扫描极限
cone < 48°
```

扫描（vh=10，`vdesc=max(80, vz*1.15)`）：

| alt | vz_max |
|---|---|
| 300~400 | 60 |
| 500~600 | 80 |
| **667** | **100** |
| 800 | 100 |
| 1000 | 120 |
| 1200~2500 | 140 |

- `vz >= 130` 是 **P3 物理极限**，任何包络都 infeasible。
- `vz = 100` 若用固定 `vdesc=80` 也会 infeasible ⇒ 包络必须随 vz 自适应。

## 运行

```powershell
# 只解算，不写执行器
python dev/starship/starship_land.py --dry-run

# 正式回收（先确认 kRPC 服务器已启动）
python dev/starship/starship_land.py
```

## 与 superheavy.ks 的联动

`boot/superheavy.ks` 已同步：

- `gfold_vz_max = 100`（新增垂速门）
- `gfold_vdesc_k = 1.15`（包络系数，随交班消息广播 `vdesc_k=`）
- 交班门四条：`h_aim<150` 且 `|v|<120` 且 `v_up<100` 且 `cone<48°`

两边改一边必须同步另一边，否则会出现"kOS 以为能交班、kRPC 却解不出"的静默失败。

## 精度指标

`pos_err < 10 m`、`vel_err < 1 m/s`、触地 `vz<3`/`vh<2`/`倾角<8°`
（旧载具要求 <1 m；新载具惯量大 5 个数量级，按用户要求放宽到 10 m。）

## 重新生成 codegen

```powershell
$env:PYTHONPATH = Join-Path (Get-Location) "tools\stubs"
$env:PYTHONJULIAPKG_OFFLINE = "yes"
python dev/starship/starship_codegen.py
```

**注意**：需要写权限创建/删除 `cpg_p3`、`cpg_p4` 目录（沙箱默认会拒绝）。
