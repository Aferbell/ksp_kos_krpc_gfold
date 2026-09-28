# Codegen 路径可行性调研（2026-09-26）

> 问题：求解要 3.6 s，原仓库用 C 代码生成，我们走 cvxpy 直接解。
> 本机能不能走 codegen？要更新什么？VS Code 的 C/C++ 扩展能不能当编译器？

---

## 1. 结论速览

| 路径 | 可行性 | 阻塞点 |
|---|---|---|
| **原仓库 `cvxpy_codegen`** | ❌ **不可行** | `cvxpy==0.4.11` 在 Python 3.13 上装不上 |
| **现代 `cvxpygen`** | ⚠️ **需先装 C 编译器** | 与 cvxpy 1.9.3 兼容，但编译 .pyd 仍需 MSVC/MinGW |
| 降 N（调参） | ✅ 已可用 | 无需新组件（详见 `TRACKING_FIX_PLAN.md`） |

---

## 2. 原仓库 codegen 路径：为什么不可行

`dev/gfold/readme.md` 的 Requirements：

```
python 3.x（推荐 3.6）
setuptools <= 57.5.0
scipy == 1.2.1
CVXcanon == 0.1.0
cvxpy == 0.4.11
cvxpy_codegen  (从 GitHub 装)
```

### 实测（本机 Python 3.13.2 / setuptools 81.0.0）

```
$ pip install "cvxpy==0.4.11" -i <清华镜像> --dry-run
error in cvxpy setup command: use_2to3 is invalid.
ERROR: Failed to build 'cvxpy' when getting requirements to build wheel
```

**根因**：`cvxpy 0.4.11`（2019 年）的 `setup.py` 用了 `use_2to3`
（setuptools 的 Python2→3 自动转换工具）。
`use_2to3` 在 **setuptools ≥ 58** 被彻底移除，而本机是 **81.0.0**。

> readme 要求 `setuptools <= 57.5.0`，但那也只支持到 Python 3.7/3.8。
> **本机 Python 3.13 上无解**，除非另装一个 Python 3.7 环境。

### 其它阻塞

| 项 | 状态 |
|---|---|
| `cvxpy_codegen` 在 PyPI | ❌ 不存在（`No matching distribution`）—— 只能从 GitHub 装 |
| `raw.githubusercontent.com` | ❌ 15 s 超时（`github.com` 与 `gh-proxy.com` 可达） |
| `scipy==1.2.1` / `CVXcanon==0.1.0` | ❌ 均为 2019 年版本，与 numpy 2.4.6 ABI 不兼容 |

**⇒ 要在本机跑原仓库 codegen，需要同时：**
1. 另装 Python 3.7/3.8（与现有 3.13 并存，环境隔离）
2. 在其中装 cvxpy 0.4.11 + scipy 1.2.1 + CVXcanon 0.1.0 + setuptools ≤57.5
3. 装 C 编译器
4. 从 GitHub 装 cvxpy_codegen

**改动面太大，不建议。**

---

## 3. 现代替代 `cvxpygen`（值得考虑）

`cvxpy_codegen` 的官方后继是 [cvxpygen](https://pypi.org/project/cvxpygen/)
（Stanford Boyd 组，论文 [Embedded Code Generation with CVXPY](https://web.stanford.edu/~boyd/papers/pdf/cvxpygen.pdf)）。

### 实测可装性 ✅

```
$ pip install cvxpygen -i <清华镜像> --dry-run
Would install cmake-4.4.3 cvxpygen-1.0.0 ecos-2.0.14 juliacall-0.9.26
              pybind11-3.1.0 qoco-0.3.2 qocogen-0.1.9
              scikit_build_core-1.1.0 ...
```

**关键优势**：
- ✅ 与 **cvxpy 1.9.3 兼容**（无需降级）
- ✅ 支持 **Python 3.13**（无需另装 Python）
- ✅ **自带 cmake 4.4.3**（解决系统无 cmake）
- ✅ 自带 `pybind11`（绑定生成）
- ✅ 可用 **ECOS**（与原仓库同源求解器）
- ✅ 从 PyPI 镜像可装

### 与原 codegen 对比

| 项 | 原 `cvxpy_codegen` | `cvxpygen` |
|---|---|---|
| cvxpy 版本 | 0.4.11（需降级） | **1.9.3（已满足）** |
| Python | 3.6/3.7 | **3.13（已满足）** |
| 安装来源 | GitHub 手装 | **PyPI 镜像** |
| cmake | 需自备 | **自带** |
| C 编译器 | 需要 | **仍需要** |

### 唯一剩下的阻塞：C 编译器

---

## 4. C 编译器现状

### 实测结果：**本机没有任何 C 编译器**

| 检查项 | 结果 |
|---|---|
| `cl.exe`（MSVC） | ❌ 未找到 |
| `gcc` / `clang` / `mingw32-make` | ❌ 未找到 |
| `cmake` | ❌ 未找到（但 cvxpygen 会自带） |
| `Windows Kits\10\Include` | ⚠️ 目录**存在但为空**（旧 VS 卸载残留，只剩 `UnionMetadata`） |
| `Windows Kits\10\bin` / `Lib` | ❌ 不存在 |
| `vswhere.exe` | ✅ 存在（但查不到任何 VS 安装） |
| Visual Studio 目录 | ⚠️ 只剩 `Installer` 与 `Shared` 两个空壳 |

`setuptools` 探测到的编译器类型是 `msvc`（默认假设），但**实际不存在**。

### VS Code 的 C/C++ 扩展**不能**当编译器

实测 `ms-vscode.cpptools-1.34.4-win32-x64` 扩展里的 `.exe`：

```
cpptools.exe / cpptools-srv.exe   ← 语言服务器（代码补全、跳转）
clang-format.exe                  ← 代码格式化
OpenDebugAD7.exe / vsdbg.exe      ← 调试适配器
msvsmon.exe                       ← 远程调试监视器
```

**全是 IDE 工具，没有编译器。** C/C++ 扩展只是「编辑器支持 + 调试器」，
它需要你**另外装**编译器（微软官方文档也是这么说的）。

---

## 5. 要装什么（若决定走 cvxpygen）

### 方案 A：MSVC Build Tools（官方、体积大、兼容性最好）

```powershell
winget install Microsoft.VisualStudio.2022.BuildTools `
  --override "--quiet --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended"
```

- 包 ID 已确认可用：`Microsoft.VisualStudio.2022.BuildTools`（17.14.41）
- 体积：**约 6~7 GB**
- 优点：与 Python 官方 `setuptools` 配合最好（自动探测）
- 缺点：下载大、耗时长

### 方案 B：MinGW-w64（体积小）

```powershell
winget install --id BrechtSanders.WinLibs.POSIX.UCRT
```

- 体积：约 200 MB
- 需设置 `distutils` 使用 mingw：
  ```
  # 在项目目录建 distutils.cfg 或在 pip 命令加 --config-settings
  [build]
  compiler=mingw32
  ```
- 优点：小、快
- 缺点：与 MSVC 编译的 Python 混用偶有兼容问题

**推荐方案 A**（兼容性优先），或先试 B（快）。

---

## 6. 若走通，预期收益

参考 `readme.md`：cvxpy 直接解「耗时可能达到 C 的 **10 倍以上**」。

本机实测（当前 cvxpy 直接解，N3/N4 = 160/80）：

| 阶段 | 离线 | 实飞 |
|---|---|---|
| P3 | 1.55 s | — |
| P4 | 0.62 s | — |
| **合计** | **2.17 s** | **3.56 s** |

若 codegen 后快 10 倍：

```
离线 2.17 s -> ~0.22 s
实飞 3.56 s -> ~0.4 s
```

**那就不需要"提前解算""降交班点"这些补偿手段了** —— 求解快到可以在交接后立刻完成。

---

## 7. 建议的决策顺序

| 优先级 | 动作 | 成本 | 收益 |
|---|---|---|---|
| 1 | **降 N3**（160→60~80），N4 不动 | 改 1 个数 | 实飞 3.56 s → ~1.5 s，**轨迹质量不降** |
| 2 | 装 MinGW（方案 B）试 cvxpygen | ~200 MB | 若成功，降到 ~0.4 s |
| 3 | 装 MSVC Build Tools（方案 A） | ~6 GB | 兼容性最好的 codegen 路径 |
| 4 | 原仓库 cvxpy_codegen | 另装 Python 3.7 + 全套降级 | **不建议** |

**建议先做 1**（零成本、立即可验证），同时可以后台装 2。
