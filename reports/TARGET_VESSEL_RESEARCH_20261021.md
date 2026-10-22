# 用地面载具作目标：kOS/kRPC 信息调研与改造方案（2026-10-21-o）

> 用户需求：现在 kOS 与 kRPC 的目标点和粗对准点经纬度都是**硬编码**，
> 想改成「选择地面上已有的载具作为目标」来控制落点。
> 本文档调研两边能拿到什么、当前程序需要什么、该怎么改。
> **本文档只做调研与方案，未改动任何飞行代码**

---

## 0. 先说结论

| 问题 | 答案 |
|---|---|
| kRPC 能拿到载具的经纬度吗 | **能，而且很完整**（`Vessel.flight().latitude/longitude`）|
| kOS 能拿到载具的经纬度吗 | **能，但必须先知道船名**（`VESSEL("name"):GEOPOSITION`）|
| kOS 能列出所有载具吗 | **不能**（kOS 1.6.0.1 无 `VESSELS` 绑定，见 §2.3）|
| 需要改什么 | 加一层「目标解析」+ 用文件把经纬度传给 kOS（见 §5）|
| 最大风险 | **目标载具不在物理范围内时，`latitude/longitude` 是否可信**（见 §4）|

---

## 1. 当前程序用了哪 8 个目标量

| 量 | 位置 | 用途 |
|---|---|---|
| `tgt_lat` / `tgt_lon` | kOS:52,53 / kRPC `TARGET_LAT/LON` | G-FOLD 目标系原点 |
| `coarse_lat` / `coarse_lon` | kOS:57,58 | **T1 RCS 粗对准目标 + 收油判据** |
| `tgt_site` = `latlng(tgt_lat, tgt_lon)` | kOS:54 | `TERRAINHEIGHT` / `ALTITUDEPOSITION` |
| `coarse_here` = `latlng(coarse_lat, coarse_lon)` | kOS:59 | `ADDONS:TR:SETTARGET` |
| `tgt_site:terrainheight` | kOS:1338 | 瞄准点绝对高度 |
| `tgt_site:altitudeposition()` | kOS:1163,1338 | 位置矢量 |
| `body.surface_position(lat,lon,frame)` | kRPC:470 | 目标系原点（+`TARGET_ALT`）|
| `TARGET_ALT` | 两边 | 质心触地高度（26.5）|

### 1.1 两个点的几何关系（实测）

```
真实目标 tgt    lat=-0.0972060952  lon=-74.5576822740
粗对准点 coarse lat=-0.0972060952  lon=-74.1002174447

经度差 0.457465 度 => 东西向 4790.6 m
纬度差 0.000000 度 => 南北 0 m
```

**粗对准点在真实目标正东 4790.6 m，纬度完全相同。**
这不是巧合 —— 它是按「载具从西往东飞」设计的：收油判据 `until imp_lon >= coarse_lon`
依赖「粗对准点在目标东侧」这个前提。

---

## 2. kOS 侧能拿到什么

### 2.1 `Vessel` 继承 `Orbitable`，有完整地理量

kOS 官方文档 `structures/orbits/orbitable.rst`：

| 后缀 | 类型 |
|---|---|
| `LATITUDE` | Scalar (deg) |
| `LONGITUDE` | Scalar (deg) |
| `ALTITUDE` | Scalar (m) |
| `GEOPOSITION` | GeoCoordinates |
| `POSITION` / `VELOCITY` / `DISTANCE` / `DIRECTION` | Vector / Scalar / Direction |

**⇒ 只要有一个 `Vessel` 对象，就能拿到它的经纬度和高度。**

### 2.2 怎么得到 `Vessel` 对象：`VESSEL("name")`

`structures/vessels/vessel.rst` 原文：

```
// Get a vessel by it s name.
// The name is Case Sensitive.
SET MY_VESS TO VESSEL("Some Ship Name").

// Save the target vessel in a variable,
// in case the current vessel changes.
SET MY_VESS TO TARGET.
```

`TARGET` 也能用，但**只能指向 KSP 当前选中的目标**，且文档明确限制：

> "This will throw an exception if called from a vessel other than the active vessel,
> as limitations in how KSP sets the target vessel limit the implementation to
> working with only the active vessel."

**⇒ `TARGET:` 只能在活动船上读写。这不够 —— 脚本要在 `SHIP` 上去读另一个载具。**

### 2.3 ⚠️ kOS **没有** `VESSELS` 列表绑定（重要发现）

我查了四处：

1. `bindings.rst` 的完整绑定变量目录 —— 只有 `SHIP` / `TARGET`，**没有 `VESSELS`**
2. `KUniverse` 的结构表 —— 有 `ACTIVEVESSEL` / `FORCESETACTIVEVESSEL` / `CRAFTLIST`，
   但**没有载具列表**
3. 直接扫 `kOS.dll` 与 `kOS.Safe.dll` 的字符串堆 —— `VESSELS` 出现 **0 次**
   （有 `TryGetVesselByName` 和 `IsValidVesselName`，印证只有「按名字取」这一条路）

**⇒ kOS 侧必须【预先知道船名】，不能「扫描地面找载具」。**

### 2.4 `GeoCoordinates` 的全部后缀

`math/geocoordinates.rst`：

| 后缀 | 类型 | 说明 |
|---|---|---|
| `LAT` / `LNG` | Scalar (deg) | |
| `TERRAINHEIGHT` | Scalar (m) | 相对海平面 |
| `DISTANCE` | Scalar (m) | **距 CPU 飞船** |
| `HEADING` / `BEARING` | Scalar (deg) | 从 CPU 飞船看过去 |
| `POSITION` | Vector | Ship-Raw 坐标 |
| `ALTITUDEPOSITION(alt)` | Vector | 指定海拔的位置 |
| `VELOCITY` | OrbitableVelocity | 该点因自转的速度 |

**⇒ `TERRAINHEIGHT` 是关键：它给地形高度，`TARGET_ALT` 必须加在它上面。**

### 2.5 kOS 侧现有的交班文件机制（可直接复用）

`superheavy.ks:1483` 已经在写：

```
LOG gfold_msg TO "0:/gfold_state.txt".
```

内容是 `GFOLD_HANDOFF lat=... lon=... alt=... t=... dist=... v=... vz=... vh=... mass=...`。

**⇒ 反方向传参数是同一个机制：一边写文件，另一边 `READ`。**

---

## 3. kRPC 侧能拿到什么

### 3.1 `SpaceCenter.vessels` 列出所有载具

```python
vessels  # list(Vessel)  —— Attribute: Read-only
```

**⇒ kRPC 能枚举，kOS 不能 —— 这是两边的根本差异。**

### 3.2 `Vessel.flight().latitude / longitude`

`Flight` 的官方文档（逐字核对了 kRPC 0.7.0 与本地 0.6.0 的 stub）：

| 属性 | 文档原文 |
|---|---|
| `latitude` | "The latitude of the vessel for the body being orbited, in degrees." |
| `longitude` | "The longitude of the vessel for the body being orbited, in degrees." |
| `elevation` | "The elevation of the terrain under the vessel, in meters. This is the height of the terrain above sea level, and is negative when the vessel is over the sea." |
| `surface_altitude` | "… Measured from the center of mass of the vessel." |
| `bedrock_altitude` | "… Measured from the center of mass of the vessel." |

**注意 `elevation` 是「船脚下地形高度」，不是船的高度。** 这正是我们需要的。

### 3.3 `CelestialBody` 的坐标工具

```python
body.surface_position(latitude, longitude, reference_frame)   # 地形表面位置
body.surface_height(latitude, longitude)                      # 地形高度（相对海平面）
body.surface_normal(latitude, longitude, reference_frame)     # 地形法线
body.position_at_altitude(latitude, longitude, altitude, rf)  # 指定海拔的位置
body.surface_reference_frame(latitude, longitude, altitude)   # 目标点固连系
```

**⇒ 当前代码 `starship_land.py:470` 用的就是 `body.surface_position`，只需把参数从常量换成变量。**

### 3.4 本地已装版本核对

`krpc 0.6.0` 的 `Vessel` 类有 **73 个属性**，含：

```
name  type  situation  loaded  packed  biome
flight  reference_frame  surface_reference_frame  orbital_reference_frame
position  velocity  direction  rotation  mass  dry_mass
```

**`loaded` / `packed` 两个属性很关键**（见 §4）。

---

## 4. ⚠️ 最大的技术风险：目标载具不在物理范围内

### 4.1 问题

地面标记载具通常离活动船**几公里**。KSP 的物理范围大约 2.25 km，
超出后载具会 **packed（on-rails）**：

```python
v.loaded   # False  => 部件没实例化，只有轨道数据
v.packed   # True   => 物理模拟暂停，按轨道解析飞行
```

**风险**：`Vessel.flight()` 在 packed 载具上返回什么？

- 如果返回**轨道推算的位置** → 经纬度正确，方案成立
- 如果返回 0 或陈旧值 → **方案失败**

### 4.2 官方文档说了什么

kRPC 文档中 `flight()` 只写：

> "Returns a Vessel.flight() object that can be used to get flight telemetry for the vessel,
> in the specified reference frame."

**文档没有明确说 packed 载具的行为。这是一个必须实测的空缺。**

### 4.3 一条可能更稳的替代路：`Vessel.orbit`

`Vessel.orbit` 是 **Read-only, Return type: Orbit**，且**不依赖物理加载**。

> ⚠️ **本文档无法确认** kRPC 的 `Orbit` 是否有 `latitude_at` 之类的工具 —— 我没查到。
> kOS 的 `Orbit` 确实有 `POSITIONAT(time)`。

**⇒ 需要实测。这也说明：本方案必须先在 KSP 在线时验证 §4.1，再动手改代码。**

### 4.4 一个规避手段（推荐）

**把目标载具的坐标【在飞行开始前】读出来并缓存。**

地面标记载具不动，所以它的经纬度**只需要读一次**：

```python
# 启动时读一次，之后全程用缓存值
fl = target_vessel.flight(sub.get(target_vessel.reference_frame))
TARGET_LAT = fl.latitude
TARGET_LON = fl.longitude
```

**这样即使中途变成 packed 也不影响** —— 我们只用启动时的快照。

唯一风险：如果读的那一刻它已经是 packed 且返回值不可信，就等于读了个垃圾值。
**所以启动时必须做有效性检查**（见 §6.3）。

---

## 5. 改造方案

### 5.1 总体思路

```
        ┌──────────────────────────────────────┐
        │ kRPC 启动时解析目标载具                │
        │  SpaceCenter.vessels -> 按名字/类型筛选│
        │  -> 读 latitude/longitude/elevation    │
        │  -> 有效性检查                          │
        └──────────────┬───────────────────────┘
                       │
           写 target_state.txt（新文件）
                       │
        ┌──────────────▼───────────────────────┐
        │ kOS 启动时 READ target_state.txt       │
        │  解析出 tgt_lat/tgt_lon/coarse_lat/lon │
        │  失败 -> 退回硬编码常量（保底）         │
        └──────────────┬───────────────────────┘
                       │
                 原有逻辑不变
```

### 5.2 为什么走文件而不是走 `TARGET:`

| 方式 | 可行性 | 问题 |
|---|---|---|
| kOS `TARGET:` | ❌ | 文档明确「只能在活动船上读写」|
| kOS `VESSEL("名")` | ✅ | 但**需要知道船名**，且船必须存在 |
| 文件传递 | ✅ | 两边都支持（`LOG` 写、`READ` 读），与现有交班机制同源 |

**选文件传递**，理由：kOS 无法枚举载具，所以「哪个载具」这个决定必须由 kRPC 做，
再由同一套文件通道下去。

### 5.3 粗对准点怎么定

**这是本方案最关键的设计问题。** 现在 `coarse` 是「目标正东 4790 m」的硬编码。

三种取法：

| 方案 | 做法 | 优点 | 缺点 |
|---|---|---|---|
| **A（推荐）** | `coarse = target + 固定东向偏移` | 保持现有收油判据语义不变 | 需要目标附近地形可容纳 |
| B | `coarse = 第二个地面载具` | 完全可配置 | 需要两个标记载具 |
| C | `coarse = target` | 最简单 | **收油判据 `until imp_lon >= coarse_lon` 会立刻成立**，功能退化 |

**方案 A 的实现**：

```python
# 保持「粗对准点在目标正东 D 米」这个已实测的几何关系
COARSE_EAST_M = 4790.6
m_per_deg = body.equatorial_radius * math.pi / 180.0
coarse_lon = TARGET_LON + COARSE_EAST_M / (m_per_deg * math.cos(math.radians(TARGET_LAT)))
coarse_lat = TARGET_LAT
```

这样**只改目标点，粗对准点自动跟随**，且几何关系与现在完全一致。

### 5.4 要改的文件与位置

| # | 文件 | 改动 | 风险 |
|---|---|---|---|
| 1 | `dev/starship/starship_config.py` | `TARGET_LAT/LON` 改为「可被运行时覆盖的默认值」 | 低 |
| 2 | `dev/starship/starship_land.py` | 新增 `resolve_target_vessel()`；`target_frame()` 用实例变量 | 中 |
| 3 | `dev/starship/starship_land.py` | 新增写 `target_state.txt` | 低 |
| 4 | `boot/superheavy.ks` | 开头 `READ target_state.txt`，失败退回硬编码 | **中** |
| 5 | `dev/analysis/probe_vessel_target.py` | 实测 §4.1 的 packed 行为 | — |

### 5.5 关键：**不破坏现有两段交班的时序**

现在的流程是：`kOS 全程 -> 交班给 kRPC`。kOS 在**发射前**就要知道 `coarse`（用来 `SETTARGET`）。

**⇒ kOS 不能等 kRPC 给参数 —— 它必须在 kOS 启动时就已经有文件。**

时序必须是：

```
① 用户手动：在 KSP 里放好地面标记载具
② 用户手动：跑 kRPC 侧的 target_resolver.py（只解析不飞）
   -> 写出 0:/target_state.txt
③ 用户手动：启动 kOS 脚本（此时文件已在）
④ 交班 -> starship_land.py 读同一个文件（或自己再解析一次）
```

**这一步「先解析再飞」不能让 `starship_land.py` 自己顺便做** ——
因为 `starship_land.py` 是在**交班后**才启动的，那时 kOS 已经用粗对准点飞了半天。

---

## 6. 实施细节

### 6.1 目标载具的识别方式

三种候选，建议同时支持并按优先级：

| 优先级 | 方式 | 例子 |
|---|---|---|
| 1 | **按名字精确匹配** | `name="LANDMARK"` |
| 2 | **按类型 + 名字正则** | `type=PROBE 且 name 匹配 "*MARK*"` |
| 3 | **按数量检查** | 全场景只有 1 个 `PROBE`/`BASE` 时自动选它 |

**必须拒绝的情况**：候选数不等于 1 时**报错退出**，不许猜。

### 6.2 `coarse` 的另一个用途：`SETTARGET`

`superheavy.ks:941` 把 `coarse_here` 设成 Trajectories 的目标：

```
ADDONS:TR:SETTARGET(coarse_here).
```

**注意**：Trajectories 的目标是给 HUD 和 `IMPACTPOS` 用的，
改成动态坐标后要确认 `IMPACTPOS:LAT/LNG` 仍正常（`superheavy.ks:839,840` 依赖它）。

### 6.3 启动时的有效性检查（必须有）

```python
def validate_target(fl, name):
    lat, lon, el = fl.latitude, fl.longitude, fl.elevation
    # ① 必须是有限数
    if not all(map(math.isfinite, (lat, lon, el))):
        raise RuntimeError("目标 %s 经纬度非有限数" % name)
    # ② 纬度范围
    if not (-90.0 <= lat <= 90.0):
        raise RuntimeError("目标 %s 纬度越界 %.4f" % (name, lat))
    # ③ 经度范围
    if not (-180.0 <= lon <= 360.0):
        raise RuntimeError("目标 %s 经度越界 %.4f" % (name, lon))
    # ④ 海拔合理（Kerbin 地形 -1500 ~ +6000 m）
    if not (-1500.0 <= el <= 6000.0):
        raise RuntimeError("目标 %s 海拔离谱 %.1f m" % (name, el))
    # ⑤ 不能在跑（地面标记应该极小速度）
    spd = math.sqrt(sum(x*x for x in fl.velocity))
    if spd > 5.0:
        raise RuntimeError("目标 %s 速度 %.1f m/s，不像地面标记" % (name, spd))
    return lat, lon, el
```

**第 5 条最关键**：如果 `flight()` 在 packed 载具上返回垃圾，速度项几乎肯定先露馅。

### 6.4 kOS 侧读文件

⚠️ **kOS 没有 try/catch**（项目里已多次踩过）。所以读文件必须：

1. 先 `EXISTS()` 判存在；
2. 解析失败时**保留硬编码默认值**，不要抛异常；
3. 用一个 `target_from_file` 布尔量标记来源，写进 HUD 方便排查。

### 6.5 默认值必须保留

**文件不存在时退回现在的硬编码**，这样：

- 老流程不受影响；
- 新机制出问题时有退路；
- 可以直接做 A/B 对比。

---

## 7. 工作量与顺序

| 步 | 内容 | 依赖 |
|---|---|---|
| **0** | 实测 §4.1（packed 载具的 `flight()` 是否可信） | **需要 KSP 在线** |
| 1 | 写 `dev/analysis/probe_vessel_target.py`（只读不飞） | 0 |
| 2 | 写 `dev/starship/target_resolver.py`（解析 + 写文件） | 1 |
| 3 | 改 `starship_config.py`：常量变默认值 | 2 |
| 4 | 改 `starship_land.py`：`target_frame()` 用实例变量 | 3 |
| 5 | 改 `boot/superheavy.ks`：`READ` + 保底 | 2 |
| 6 | 离线验证（`check_gate_sync.py` 同类脚本） | 4,5 |
| 7 | `--dry-run` 首飞 | 6 |

**第 0 步是硬门槛。** 如果 packed 载具的 `flight()` 不可信，
整个「按名字取坐标」的路线要改成「用 `Vessel.orbit` 推算」或「用 kOS 侧 `VESSEL("名"):GEOPOSITION`」。

---

## 8. 备选方案：如果 kRPC 的 packed 读数不可信

### 8.1 用 kOS 侧读（推荐备选）

kOS 的 `VESSEL("name"):GEOPOSITION` 走的是 `Orbitable` 的经纬度，
**它和 kRPC 的 `flight().latitude` 未必同源**，可能对 packed 载具更稳。

做法：在 `superheavy.ks` 里直接：

```
// 开头，在设置粗对准点之前
local mark is VESSEL("LANDMARK").
local tgt_lat is mark:GEOPOSITION:LAT.
local tgt_lon is mark:GEOPOSITION:LNG.
```

**优点**：不依赖 kRPC，不改文件通道，改动最小。
**缺点**：仍然要人工填一个船名字符串（只是从「填坐标」变成「填名字」）。

### 8.2 两者的取舍

| | 填名字（kOS 直读） | 文件传递（kRPC 解析） |
|---|---|---|
| 改动量 | **小**（改 4 行） | 大（新脚本 + 双端改） |
| 换目标的方式 | 改脚本里的字符串 | 改配置文件 / 自动识别 |
| 依赖 packed 行为 | kOS 侧 | kRPC 侧 |
| 能否自动扫描 | 不能 | **能** |

**建议：先做 §8.1（小、稳、立刻可用），把 §5 的文件通道作为第二步。**

### 8.3 混合方案（最实用）

```
kOS 侧：VESSEL("<名字>")  —— 名字从文件读，读不到就用默认字符串
kOS 解出的经纬度【同时】写回 target_state.txt
kRPC 交班后读同一个文件，保证两边口径绝对一致
```

**这样只有一处读坐标（kOS），kRPC 只消费结果 —— 彻底消除「两边不一致」的风险。**

---

## 9. 必须实测确认的 4 件事

| # | 问题 | 怎么测 | 不通过怎么办 |
|---|---|---|---|
| 1 | packed 载具的 `flight().latitude` 是否可信 | 在 3 km / 10 km / 50 km 外各读一次 | 改用 §8.1 或 `Vessel.orbit` |
| 2 | `VESSEL("名")` 对 packed 载具是否有效 | kOS 侧远处读一次 | 用 §5 文件通道 |
| 3 | kRPC 与 kOS 读同一载具的经纬度是否一致 | 同时读，比差值 | 若差 >1 m，以 kOS 为准 |
| 4 | `ADDONS:TR:SETTARGET()` 接受动态经纬度后 `IMPACTPOS` 是否仍正常 | 装 Trajectories 后看 HUD | 保留硬编码 coarse 做兜底 |

---

## 10. 建议的落地顺序

**第一步（低风险，立即可做）**：

1. 在 `superheavy.ks` 开头把 `tgt_lat/tgt_lon` 从字面量改成
   `VESSEL("LANDMARK"):GEOPOSITION:LAT/LNG`，**保留硬编码做兜底**；
2. `coarse` 由 `tgt` 按固定东向偏移算出；
3. `starship_land.py` 读 kOS 写出的 `target_state.txt`。

**第二步（需要自动扫描时）**：

4. 加 `target_resolver.py`（kRPC 枚举 + 有效性检查 + 写文件）；
5. kOS 从文件读船名。

**第三步（可选）**：

6. 支持「目标 = 另一个载具实例」（例如已落地的上一发一级）；
7. 支持按 biome / 离发射场距离自动选靶。

---

## 11. 本文档未做的事

1. **没有实测 §4.1**（KSP 当前未运行，kRPC 连接被拒 `WinError 10061`）
2. **没有确认 kRPC `Orbit` 是否有 `latitude_at`**（文档未查到）
3. **没有改任何飞行代码**

---

## 12. 参考来源

- kRPC Vessel：https://krpc.github.io/krpc/latest/python/api/space-center/vessel.html
- kRPC Flight：https://krpc.github.io/krpc/latest/python/api/space-center/flight.html
- kRPC SpaceCenter：https://krpc.github.io/krpc/latest/python/api/space-center/space-center.html
- kRPC CelestialBody：https://krpc.github.io/krpc/latest/python/api/space-center/celestial-body.html
- kRPC Waypoints：https://krpc.github.io/krpc/latest/python/api/space-center/waypoints.html
- kOS Vessel：https://ksp-kos.github.io/KOS/structures/vessels/vessel.html
- kOS Orbitable：源码 doc/source/structures/orbits/orbitable.rst
- kOS GeoCoordinates：https://ksp-kos.github.io/KOS/math/geocoordinates.html
- kOS bindings：https://raw.githubusercontent.com/KSP-KOS/KOS/master/doc/source/bindings.rst
- kOS KUniverse：https://ksp-kos.github.io/KOS/structures/misc/kuniverse.html
