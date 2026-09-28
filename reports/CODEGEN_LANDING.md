# Codegen 路径落地记录（2026-09-26）

> 目标：把 P3/P4 求解从「cvxpy 直接解（实飞 3.6 s）」改为「C 代码生成」
> 方案：用户选定的「方案 A」= 装 MSVC Build Tools + 用 `cvxpygen` 替代原仓库的 `cvxpy_codegen`

## 结果速览

| 指标 | 改动前 | 改动后 |
|---|---|---|
| 求解耗时（离线） | 2.17 s | **0.080 s** |
| 求解耗时（实飞推算） | 3.56 s | **~0.13 s** |
| 加速比 | — | **约 27~40 倍** |
| 轨迹质量 | 末端倾角 0.03° | 末端倾角 0.00°（不降） |
| 末端位置误差 | 0.000000 m | 0.000000 m |

---

## 1. 环境搭建（全部实测验证）

### 1.1 MSVC Build Tools ✅

```powershell
winget install Microsoft.VisualStudio.2022.BuildTools `
  --override "--quiet --wait --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended"
```

| 组件 | 路径 |
|---|---|
| VS 安装根 | `C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools` |
| MSVC 工具链 | `...\VC\Tools\MSVC\14.44.35207` |
| `cl.exe` | `...\bin\Hostx64\x64\cl.exe` |
| `vcvars64.bat` | `...\VC\Auxiliary\Build\vcvars64.bat` |
| Windows SDK | `10.0.26100.0` |

实测编译 `printf("CL_OK")` 成功。

### 1.2 cvxpygen 0.7.0 ✅

**为什么不用 1.0.0**：依赖 `pdaqp` → `juliacall` → **导入时联网下载 Julia**
（`julialang-s3.julialang.org` 不可达，实测挂起 >5 min）。

**为什么用 0.7.0**：0.6.6 声明 `cvxpy>=1.6.4`，但实测在 cvxpy 1.9.3 上抛
`AttributeError: 'tuple' object has no attribute 'cons_id_map'`。
**0.7.0 声明 `cvxpy>=1.8.0`**，与 1.9.3 匹配。

**Julia 依赖处理**（查证官方源码后）：
- `juliapkg/state.py:8 get_config` 的官方开关是 `PYTHON_JULIAPKG_OFFLINE=yes`
  → 找不到 Julia 时快速失败，不联网
- 但 `cpg.py:22` 顶层 `import mpqp` → `mpqp.py:20` `from pdaqp import MPQP`
- 查 `cpg.py:97 get_solver_and_explicit_flag`：`solver='explicit'` 才走 PDAQP，
  我们用的 `'ECOS'` 返回 `explicit=0`，**`mpqp` 从不执行**
- ⇒ 写 `tools/stubs/pdaqp.py` 空壳顶掉它，卸载 `pdaqp`/`juliacall`/`juliapkg`/`qoco`/`qocogen`

---

## 2. 踩过的 5 个坑（cvxpygen 与 cvxpy 版本差异）

| # | 现象 | 根因 | 修法 |
|---|---|---|---|
| ① | `DCPError: Problem does not follow DCP rules` | `z0i`/`z0l` 做成 Parameter（参考仓库在 cvxpy 0.4.11 下可行） | 实测 5 种等价写法，**只有 numpy 常数能过 DCP** ⇒ 固定为生成期常数 |
| ② | `DPPError: not DPP` | `z[0,0] == cp.log(m_wet)`，Parameter 的 log 非线性 | 照抄参考仓库：把 `m_wet_log` 作为**预先算好的 Parameter** |
| ③ | `DPPError: not DPP` | `s[0,0] <= r2/m_wet`，参数在分母 | 预乘成 `s_hi0`/`s_lo0` 两个标量 Parameter |
| ④ | `ModuleNotFoundError: No module named 'D:\...\cpg_p3'` | `cpg.py:94` `import_module(f'{code_dir}.cpg_solver')` 把绝对路径当模块名（上游 bug） | `wrapper=False` + 自己调公开函数 `cpg.compile_python_module()` |
| ⑤ | `from D:.path import cpg_module` 语法错 | `utils.py:1752` `code_dir.replace('\\','.')` 保留盘符冒号 | 生成后 post-process 改成 `import cpg_module` |

### 额外发现：tf 可以参数化（关键）

`tf` 原本固化在 `dt=tf/N` 里 ⇒ 每个工况都要重新生成。
**实测把 `tf` 声明成 Parameter 后 DCP 与 DPP 都保持通过**
（`dt*0.5*(变量+变量)` 是"参数×变量"的线性组合，DPP 允许）
⇒ **一份生成模型覆盖任意 tf**。

---

## 3. 两个扩展共存的坑（最费时）

**现象**：P3 与 P4 都编译成功，但同时加载时：
```
ImportError: generic_type: type "cpg_params_p4" is already registered!
```
或先前的：
```
ValueError: cannot reshape array of size 960 into shape (6,80)   # 960 = 6×160
```

**根因链**：
1. 两个 `.pyd` 都叫 `cpg_module`，初始化符号都是 `PyInit_cpg_module`
   ⇒ CPython 按名字缓存，**先加载 P3 后加载 P4 会拿到 P3**
2. 改名后 pybind11 又报冲突：它的类注册以 **C++ typeid** 为键（全局），
   两个扩展里的 `CPG_Params_cpp_t` 同名
3. 报错信息里显示的是 `cpg_params_p4`，容易误以为"改 Python 类名就行"——
   **实际要解决的是 C++ 类型重名**

**最终修法**（`rename_module()` 改三处）：
- `setup.py`：`Extension('cpg_module_p3')` / `setup(name='cpg_module_p3')`
- `cpp/src/cpg_module.cpp`：`PYBIND11_MODULE(cpg_module_p3, m)`
- **`py::class_<T>(m, "name")` → `py::class_<T>(m, "name", py::module_local())`**
  ← pybind11 官方文档 `docs/advanced/classes.rst` 的正解

---

## 4. 代码改动

### 4.1 `dev/gfold/solver/gfold_codegen.py`（**新增**）

参数化重建 P3/P4 模型并生成 C 求解器。关键常量：
```python
N_P3 = 160      # 与 gfold_p3p4 的 N3 一致（tf_m 对 N3 敏感，不降）
N_P4 = 80       # 与 N4 一致
MASS_GEN = 183000.0   # z0i/z0l 必须是常数，取交班典型质量
TF_GEN = 21.0         # 只用于算 z0i/z0l 展开点（tf 本身已参数化）
```
对外函数：`build()` / `set_params()` / `rename_module()` / `fix_generated_import()` / `generate_one()` / `main()`

### 4.2 `dev/gfold/solver/gfold_p3p4.py`（**修改**）

新增 codegen 快速路径：
- `_CGenMod` 类：包装生成的扩展（避免往 ModuleType 挂属性）
- `_load_cgen(program, N)`：加载扩展，失败**打印原因**（不再静默）
- `_solve_one_cgen(...)`：用扩展求解，返回与 `_solve_one` 同构的 dict
- `_solve_p3p4_impl()`：**P3 与 P4 都优先走 codegen**，失败回退 cvxpy
- 返回值加 `engine` 字段（`'cgen'` / `'cvxpy'`）便于确认走了哪条路

### 4.3 `tools/gen_codegen.ps1`（**新增**）

包装脚本，设好 `PYTHONPATH`（pdaqp stub）与 `PYTHON_JULIAPKG_OFFLINE=yes`。

### 4.4 `tools/stubs/pdaqp.py`（**新增**）

pdaqp 空壳，绕开 Julia 联网挂起。

---

## 5. 验证结果

```
pyright 核心文件 0 错误 / pyflakes 19 文件 / 未定义名 / kRPC 成员 / kOS 语法   全部 PASS
```

| 工况 | 状态 | engine | 耗时 | tf_m |
|---|---|---|---|---|
| 实飞交班 alt=4994 | optimal | cgen | 0.096 s | 20.8 |
| 低空备选 alt=1500 | optimal | cgen | 0.064 s | 14.8 |
| 中空 alt=2500 | optimal | cgen | 0.077 s | 17.8 |
| 高空 alt=4000 | optimal | cgen | 0.082 s | 20.5 |

**正确性对比**（同工况 cgen vs cvxpy）：
```
cgen  : 落地 150.4 t  末端倾角 0.00°  末端位置误差 0.000000 m
cvxpy : 落地 150.6 t  末端倾角 0.04°  末端位置误差 0.000000 m
u_hor 前6 cgen : [-38.33, -38.38, -38.35, -38.30, -38.24, -38.16]
u_hor 前6 cvxpy: [-36.82, -36.80, -36.76, -36.66, -36.54, -36.42]
最大方向夹角 1.33°（可跟踪）
```

---

## 6. 产物与体积

```
dev/gfold/solver/cpg_p3/cpg_module_p3.cp313-win_amd64.pyd   0.92 MB
dev/gfold/solver/cpg_p4/cpg_module_p4.cp313-win_amd64.pyd   0.60 MB
```
已删除 `build/` 中间产物（可重新生成），目录合计 **15.2 MB**。

---

## 7. 重新生成的方法

```powershell
pwsh -File tools/gen_codegen.ps1
```
（改了模型结构或 N 之后需要重跑；日常飞行不需要）
