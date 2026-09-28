# 语法检查工具链：发现并补上一个致命盲区（2026-09-27）

> 委托专门的语法子代理核查，子代理报告了一个**我自己没发现的工具盲区**。
> 本文件记录：子代理结论 → 我的独立复核 → 补上的工具。

---

## 1. 子代理结论摘要

**没有语法错误 / 未定义名 / API 误用。7 条项目不变式全部 PASS。**
但报告了一个**覆盖盲区**：

> `tools/kos_lint.ps1` **无法检测无效的 kOS 后缀**。
> 用含 `set x to ship:id.` 的文件探针，它报 **"NO DIAGNOSTICS — file is clean", exit 0**。

**这正是摔过机的失败模式**（kOS 无 try/catch，无效后缀当场终止脚本）。

---

## 2. 我的独立复核（不盲从）

### 探针 A：无效后缀

```kos
// boot/_probe_bad_suffix.ks
print "probe start".
set x to ship:id.
print "probe end".
```

```
诊断: 0 for this file
*** NO DIAGNOSTICS — file is clean ***
exit 0
```

### 探针 B：真实语法错误

```kos
// boot/_probe_syntax.ks
print "probe start"
if true {
    set x to 1.
```

```
diagnostics: 8 for this file
by severity: ERROR=2
ERROR L2:7  [parser-error] Expected ".".
ERROR L4:15 [parser-error] Expected "}" to finish statement block
exit 1
```

### ⇒ 子代理结论**成立**

| 检查项 | kos_lint.ps1 |
|---|---|
| 语法（缺句点/括号不配） | ✅ 能查（exit 1） |
| **后缀是否存在** | ❌ **不能查**（`ship:id` 判为 clean） |

**后果**：`check_py.ps1` 打印"全部通过，可以上飞"时，
**后缀名从未被验证过**。而这是会摔机的错误。

---

## 3. 补上的工具：`tools/check_kos_suffix.py`

### 做法

1. **剥注释** —— 注释里的 `ship:id` 是"记录这个坑"的散文，不是缺陷
2. **正则抓冒号链** `a:b:c`
3. **按类型图逐段走**：
   ```
   ship:geoposition:lat  ==  ((ship . geoposition) . lat)
   ```
   每段必须在**当前类型**的成员表里；若该成员有类型，它就是下一段的类型
4. **本文件定义的变量**（`local X is` / `set X to` / **`for X in`**）跳过
   —— 静态无法定类型
5. **未知项报错而不是忽略** —— 宁可人工确认一次，也不放过

### 白名单来源

官方后缀表（手工整理进 `MEMBERS` / `GLOBAL_TYPE`）：
- `https://ksp-kos.github.io/KOS/structures/vessels/vessel.html`
- 以及 `Body` / `Orbit` / `GeoPosition` / `Control` / `Direction` 等

### 自测（两个方向都验）

**① 已知坏文件必须被抓**
```kos
set x to ship:id.
set y to ship:mass.
set z to ship:geoposition:lat.
```
```
scanned 3 distinct suffix chain(s)
L1: ship:id  -> FORBIDDEN: Vessel has NO 'id' suffix -> throws
                GET Suffix 'ID' not found; kOS has no try/catch
                => the script dies mid-flight
*** 1 suspicious suffix(es) ***   exit 1
```

**② 真实文件必须通过**
```
=== boot/B1040-9.ks ===
  scanned 69 distinct suffix chain(s)
  PASS - no suspicious suffix found
*** ALL PASS ***
```

> **注**：第一版误报 36 处，因为我没处理**链式属性**（把 `ship:geoposition:lat`
> 整条拿去查 `ship` 的成员表）。改成类型图遍历 + 识别 `for` 循环变量后降到 0。

---

## 4. 顺带修掉的真问题

把新增文件加进 `check_py.ps1` 的 pyflakes 范围后，**立刻抓出 4 个未使用的 import**：

| 文件 | 问题 |
|---|---|
| `dev/krpc/diag_track_real.py:8` | `types` 未使用 |
| `dev/krpc/diag_ni_sign.py:6` | `math` 未使用 |
| `dev/krpc/verify_prewarm.py:8` | `math` 未使用 |
| `dev/krpc/verify_prewarm.py:13` | `numpy` 未使用 |

已全部删除，并验证脚本仍能正常运行。

---

## 5. `check_py.ps1` 现在的覆盖

| # | 检查 | 覆盖 |
|---|---|---|
| 0 | pyright（Pylance 同源） | 核心 2 文件，**0 错误** |
| 1 | pyflakes | **30 个文件**（原 19） |
| 2 | `check_names.py` | 顺序敏感的未定义名 |
| 3 | `check_krpc_api.py` | kRPC 成员真实存在 |
| 4 | `kos_lint.ps1` | kOS **语法** |
| 5 | **`check_kos_suffix.py`（新）** | **kOS 后缀存在性** ← 补上盲区 |

```
=== 0. pyright ===            PASS (核心文件 0 错误)
=== 1. pyflakes ===           PASS (30 个文件)
=== 2. 未定义名检查 ===        PASS
=== 3. kRPC 成员核对 ===       PASS
=== 4. kOS 语法 ===           PASS
=== 5. kOS 后缀核对 ===        PASS
*** 全部通过，可以上飞 ***
```

---

## 6. 子代理报告的另两点（已处理/说明）

| 项 | 处理 |
|---|---|
| `boot/B1040-9.ks` 日志表头写 `gfold_cone_ok`，实际写的是 `gfold_cone_flag` | **仅命名不一致**，列数与取值都正确（`check_cols.py` 验证 49=49）。不改，因为改表头会影响已有日志解析脚本；若要改，两者一起改。 |
| `cpg_p3/cpg_solver.py` 用扁平 `import cpg_module_p3`，作为包导入会失败 | **非实际缺陷**：`gfold_p3p4._load_cgen()` 直接 `importlib` 加载 `.pyd`，绕开这些文件。已实测两个扩展都能加载。 |

---

## 7. 本次新增/修改文件

| 文件 | 说明 |
|---|---|
| `tools/check_kos_suffix.py` | **新增**：kOS 后缀静态核对（类型图遍历） |
| `tools/check_py.ps1` | 新增第 5 步；pyflakes 目标 19 → **30** 文件 |
| `dev/krpc/diag_track_real.py` | 删未用 import |
| `dev/krpc/diag_ni_sign.py` | 删未用 import |
| `dev/krpc/verify_prewarm.py` | 删未用 import |
| `boot/_probe_*.ks` | 探针文件，**已删除** |
