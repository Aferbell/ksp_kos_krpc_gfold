# dev 仓库上传清单（2026-10-22）

> 仓库：`dev/` 是独立 git 仓库（**不是** `Ships/Script` 根）
> remote：`https://gh-proxy.com/https://github.com/Aferbell/ksp_kos_kprc_gfold.git`
> 当前分支：**`tilt-first-control-law`**（还有 `main`）
> **本文档只做清点，不执行任何 git 操作**

---

## 0. 先说三件最重要的事

| # | 事实 | 影响 |
|---|---|---|
| **1** | **`dev/starship/` 整个目录从未提交过**（`git ls-files starship` 为空） | **最新的生产代码完全不在云端**，全部丢失过一次 |
| **2** | **`dev/video_analysis/` 有 2.7 GB**，且未被 ignore | 一旦 `git add -A` 会直接卡死 push |
| **3** | **`dev/gfold/` 是独立仓库**（`.gitignore` 已排除） | 不要动，它有自己的 remote |

---

## 1. 当前状态

```
已修改  8 个
已删除  2 个
未跟踪  4701 个（展开）  约 2.76 GB
```

### 1.1 已修改的 8 个（都在被跟踪的目录里）

| 文件 | diff 规模 | 该不该传 |
|---|---|---|
| `krpc/gfold_land.py` | **+1413 / −1799** | ⚠️ **真实逻辑改动**（tilt-first 重写版），需确认 |
| `solver/gfold_p3p4.py` | 236 行 | ✅ 传 |
| `solver/gfold_codegen.py` | 21 行 | ✅ 传 |
| `solver/cpg_p3/*.pyd` | 二进制 994 KB | ✅ **必须传**（实飞必需件）|
| `solver/cpg_p4/*.pyd` | 二进制 652 KB | ✅ **必须传** |
| `krpc/diag_track_real.py` | 22 行 | ✅ 传 |
| `krpc/verify_terminal.py` | 26 行 | ✅ 传 |
| `reports/NEW_SESSION_PROMPT.md` | +10 行 | ✅ 传 |

### 1.2 已删除的 2 个

`_notes_clean/` 下两个被删掉的 md。**如果是有意删的，提交删除即可；如果不是，用 `git restore` 恢复。**

---

## 2. ⚠️ 必须先排除的（不要上传）

| 路径 | 大小 | 为什么 |
|---|---|---|
| **`dev/video_analysis/`** | **2.69 GB** | 4058 个文件，全是提取的视频帧。**必须加进 `.gitignore`** |
| `dev/analysis/_frames_224111/` | 30.6 MB | 36 帧 PNG，一次性分析产物 |
| `dev/analysis/_frames_e_trials/` | 19.6 MB | 46 帧 PNG |
| `dev/analysis/overlap_frames/` | 2.0 MB | 17 帧 PNG |
| `dev/analysis/_kos_builtins.txt` | 小 | 从 kOS 导出的内置函数表，可重生成 |
| `dev/analysis/_kos_core_builtins.txt` | 小 | 同上 |
| `dev/recovery/` | 54 MB | `.gitignore` 已排除（历史恢复资料）|
| `dev/gfold/` | 40 MB | `.gitignore` 已排除（独立仓库）|

**`video_analysis/` 现在没被 ignore** —— 它出现在 `git status` 的 `??` 里。
**只要有人 `git add -A`，2.7 GB 会全部进暂存区。**

### 2.1 一个容易漏的点：`starship/cpg_p*/` 的 ignore 规则**没有覆盖**

实测 `git add -A --dry-run starship` 会加 **146 条 / 10.62 MB**，其中：

| 文件 | 大小 | 该传吗 |
|---|---|---|
| `cpg_p3/problem.pickle` | 2.15 MB | ❌ 编译中间件，可重生成 |
| `cpg_p3/c/src/cpg_workspace.c` | 1.82 MB | ❌ cvxpygen 生成的 C 源 |
| `cpg_p4/problem.pickle` | 1.16 MB | ❌ |
| `cpg_p4/c/src/cpg_workspace.c` | 0.91 MB | ❌ |
| `cpg_p3/README.html` | 0.26 MB | 可选 |
| 其余 `c/` `cpp/` | ~4 MB | ❌ |

**为什么没被排除**：现有 `.gitignore` 只写了 `cpg_p3/*` 和 `solver/cpg_p3/*`，
**没有 `starship/cpg_p3/*`**。而 `build/` 那条是全局的（`.gitignore:47`），所以 build 被排除了，但 `c/` 和 `problem.pickle` 没有。

**⇒ 必须补 `starship/cpg_p*/*` 规则，否则会多传 8.4 MB 垃圾。**

---

## 3. 该上传的

### 3.1 【最紧急】`dev/starship/` —— 生产代码，146 个文件 / 10.8 MB

**这是当前唯一在用的着陆程序，但从未进过仓库。**

| 文件 | 说明 | 必传 |
|---|---|---|
| `starship_land.py` | 主程序（~3.5k 行）| ✅ |
| `starship_config.py` | 全部参数 | ✅ |
| `starship_ctrl.py` | PID / Logger / 几何工具 | ✅ |
| `starship_p3p4.py` | P3/P4 求解器封装 | ✅ |
| `starship_codegen.py` | codegen 生成器 | ✅ |
| `HANDOFF.md` | 换载具必读（~620 行）| ✅ |
| `README.md` | | ✅ |
| `_selftest.py` | 自检（3 工况）| ✅ |
| `api_check.py` | kRPC API 校验 | ✅ |
| `_scan_final.py` | 交接门扫描 | ✅ |
| `_final.py` | 终版参数核算 | ✅ |
| `analyze_first_stage.py` | 一级载具分析 | ✅ |
| `cpg_p3/cpg_module_p3.cp313-win_amd64.pyd` | **实飞必需**（971 KB）| ✅ **必须** |
| `cpg_p4/cpg_module_p4.cp313-win_amd64.pyd` | **实飞必需**（637 KB）| ✅ **必须** |
| `cpg_p3/cpg_solver.py` / `cpg_p4/cpg_solver.py` | 求解器包装 | ✅ |
| `cpg_p3/__init__.py` / `cpg_p4/__init__.py` | | ✅ |
| `cpg_p*/COPYING` / `setup.py` | 许可与安装 | ✅ |
| `cpg_p*/README.html` | 422 KB 文档 | 可选 |
| `cpg_p*/problem.pickle` | 2.2+1.2 MB，编译中间件 | ❌ **不传**（可重生成）|
| `cpg_p*/build/` | 24.5 MB | ❌ **不传** |
| `cpg_p*/c/` | 9.7 MB，183 文件 | ❌ **不传**（cvxpygen 生成的 C 源）|
| `cpg_p*/cpp/` | 0.3 MB | ❌ **不传** |

**⇒ 该传 20 个文件 / 2.19 MB**（实测 `git add --dry-run` 后按下面的 ignore 规则过滤）：

```
starship/HANDOFF.md
starship/README.md
starship/_final.py
starship/_scan_final.py
starship/_selftest.py
starship/analyze_first_stage.py
starship/api_check.py
starship/cpg_p3/__init__.py
starship/cpg_p3/cpg_module_p3.cp313-win_amd64.pyd     <- 实飞必需
starship/cpg_p3/cpg_solver.py
starship/cpg_p3/setup.py
starship/cpg_p4/__init__.py
starship/cpg_p4/cpg_module_p4.cp313-win_amd64.pyd     <- 实飞必需
starship/cpg_p4/cpg_solver.py
starship/cpg_p4/setup.py
starship/starship_codegen.py
starship/starship_config.py
starship/starship_ctrl.py
starship/starship_land.py
starship/starship_p3p4.py
```

**`.gitignore` 需要新增**（`starship/` 是新路径，现有规则只覆盖 `cpg_p*` 和 `solver/cpg_p*`）：

```gitignore
starship/cpg_p3/*
starship/cpg_p4/*
!starship/cpg_p3/cpg_module_p3*.pyd
!starship/cpg_p4/cpg_module_p4*.pyd
```

### 3.2 `dev/reports/` —— 48 个新报告 / 1.1 MB

**全部该传。** 这是本项目的知识资产，包含：

| 类别 | 例子 |
|---|---|
| 失败复盘 | `C_FIX_FALSIFIED_20261021.md`、`TUNING_FEASIBILITY_20261021.md` |
| 换载具指南 | `ALTITUDE_DATUM_AUDIT_20261021.md`、`GATE_CHAIN_20261021.md` |
| 每次实飞分析 | `LANDING_184650_ANALYSIS_20261021.md`、`TGAIN12_EVAL_20261022.md` |
| 控制律详解 | `HORIZONTAL_LAW_AND_SLOWDOWN_20261022.md` |
| 图片 | `reports/img/*.png`（2 张）|

### 3.3 `dev/analysis/` —— **331 个 .py，但只有一部分值得传**

总共 430 个文件 / 54 MB，其中：

| 内容 | 数量 | 大小 | 建议 |
|---|---|---|---|
| **可复跑的分析脚本** | ~331 个 .py | 1.7 MB | ⚠️ **需要筛选** |
| `_frames_224111/` 帧数据 | 36 | 30.6 MB | ❌ ignore |
| `_frames_e_trials/` 帧数据 | 46 | 19.6 MB | ❌ ignore |
| `overlap_frames/` | 17 | 2.0 MB | ❌ ignore |
| `_kos_builtins.txt` 等 | 2 | 小 | ❌ ignore |

**331 个 .py 里很多是一次性探针**（`probe_*.py`、`diag*_*.py`、`_tmp_*.py`）。
按根 HANDOFF §5.5 的规矩「一次性的扫描/试算脚本用完删掉」，**应该先清理再传**。

**建议保留的**（有长期价值、带 `__main__` 可复跑）：

| 脚本 | 用途 |
|---|---|
| `analyze_tgain12.py` | T_GAIN 8→12 对比 |
| `analyze_224134.py` | 单趟起振分析 |
| `align_20261022.py` | 视频↔日志时间对齐 |
| `check_new_gates.py` | 门槛链自洽验证 |
| `check_gate_sync.py` | kOS/kRPC `target_alt` 同步 |
| `check_target_alt_26p5.py` | TARGET_ALT 求解器可行性 |
| `check_az_freeze.py` | 方位角冻结扫描（含失败的 C 方案）|
| `verify_log_cols.py` | 日志列核对 |
| `trace_osc_start.py` | 起振逐帧追踪 |
| `analyze_att_decomp.py` | 姿态误差分解 |
| `analyze_attitude_old_vs_new.py` | 新旧载具对比 |
| `analyze_landing_172305.py` / `analyze_205641.py` / `analyze_195348.py` | 单趟复盘 |
| `repo_inventory.py` | 本次清点（可删）|

### 3.4 `dev/sim/` —— 10 个文件 / 0.4 MB

MATLAB 验证脚本 + 4 张 PNG。**该传**（闭环验证的可复现证据）。

### 3.5 `dev/craft/` —— 5 个文件

`_split_stages.py`、`analyze_atest1.py`、`analyze_starship.py`、`atest1_massprops2.py`、`what_test_must_measure.py`

**该传** —— 一级载具质量/推力的分析脚本，换载具时要用。

### 3.6 `dev/krpc/` —— 2 个新文件

| 文件 | 建议 |
|---|---|
| `gfold_land_old.py` | ⚠️ 旧版备份，**建议不传**（避免混淆当前版本）|
| `verify_rewrite.py` | ⚠️ 一次性验证脚本，**建议不传** |

### 3.7 `dev/solver/` —— 2 个新文件

`_bench_cgen.py`、`_bench_mass.py` —— 求解器性能基准。**可传**（有参考价值）。

---

## 4. 建议的 `.gitignore` 追加内容

```gitignore
# ---- 视频分析帧（2.7 GB，绝对不进仓库）----
video_analysis/

# ---- 一次性提取的帧目录 ----
analysis/_frames_*/
analysis/overlap_frames/

# ---- kOS 内置表（可重生成）----
analysis/_kos_*.txt

# ---- starship 的 codegen 中间产物（只保留 .pyd）----
starship/cpg_p3/*
starship/cpg_p4/*
!starship/cpg_p3/cpg_module_p3*.pyd
!starship/cpg_p4/cpg_module_p4*.pyd
```

---

## 5. 上传体积估算

| 类别 | 文件数 | 体积 |
|---|---|---|
| `starship/`（筛后）| ~14 | **1.7 MB** |
| `reports/` | 48 | 1.1 MB |
| `analysis/`（筛后）| ~20 | ~0.3 MB |
| `sim/` | 10 | 0.4 MB |
| `craft/` | 5 | 0.05 MB |
| `solver/` 2 个新 py | 2 | 0.05 MB |
| 已修改的 8 个（含 2 个 .pyd）| 8 | 1.6 MB |
| **合计** | **~107** | **≈ 5.2 MB** |

**对比：如果直接 `git add -A`，会是 2.76 GB / 4701 文件。**

---

## 6. ⚠️ 两个需要你决定的问题

### 6.1 ⚠️ `krpc/gfold_land.py` 是**真实逻辑改动**，不是换行符

查证结果：

| 版本 | 行数 | 内容 |
|---|---|---|
| **HEAD（已提交）** | **5154** | 第十五步的版本 |
| **工作区（未提交）** | **4768** | **tilt-first 重写版**（文件头写着「2026-10-08 重写版，单一 tilt-first 控制律，无旧律、无开关」）|
| `gfold_land_old.py`（未跟踪） | 6076 | 旧的 18 步版 |

`-w --ignore-cr-at-eol` 后仍显示 **+1413 / −1799**，说明是**真改动**。

**这不是我这几轮改的**（我这几轮只动 `dev/starship/`）。
**提交前需要你确认**：工作区这版是不是你要的？如果要，就提交；不要就用 `git restore krpc/gfold_land.py` 回 HEAD。

### 6.2 当前分支是 `tilt-first-control-law`

`main` 和 `tilt-first-control-law` 是两个分支，最近 8 个提交都在后者上。
**要确认新代码应该提到哪个分支。**

---

## 7. 建议的执行顺序

```
1. 先加 .gitignore（第 4 节那 4 段）
2. 确认 krpc/gfold_land.py 的 3236 行 diff 是换行符还是真改动
3. 清理 analysis/ 里的一次性探针（按根 HANDOFF §5.5）
4. git add starship/ reports/ sim/ craft/ solver/_bench_*.py
5. git add analysis/（筛后）+ 已修改的 8 个
6. 确认 git status 里没有 video_analysis/ 和 _frames_*
7. 提交 + push
```
