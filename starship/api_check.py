# -*- coding: utf-8 -*-
"""api_check.py -- 用 krpc 的【真实类型】校验本程序用到的每个属性。

【为什么要这个文件】
  2026-10-20 实飞在 shutdown_engines 里炸：
      AttributeError: 'Part' object has no attribute 'engines'
  kRPC 的 Part 只有一个 engine（单数）属性，且当部件不是引擎时为 None。
  这类错误【静态检查抓不到】（krpc 全是动态 property），只能在
  import 阶段对着真实类做一次存在性校验。凡是用 kRPC API 的脚本，
  都应先跑这个文件。
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import krpc.services.spacecenter as sc

# (类, 属性列表, 说明)
SPEC = [
    (sc.Vessel, ["parts", "control", "auto_pilot", "available_thrust", "max_thrust",
                 "mass", "thrust", "dry_mass", "surface_reference_frame",
                 "reference_frame", "orbit", "position", "velocity", "rotation",
                 "angular_velocity", "name", "type", "flight", "direction",
                 "moment_of_inertia", "inertia_tensor", "available_torque",
                 "recoverable", "situation", "met"], "Vessel"),
    (sc.Flight, ["surface_altitude", "speed", "vertical_speed", "horizontal_speed",
                 "latitude", "longitude", "mean_altitude", "g_force", "drag",
                 "pitch", "heading", "roll"], "Flight"),
    (sc.Parts, ["all", "engines", "decouplers", "parachutes", "launch_clamps",
                "control_surfaces", "antennas", "with_tag", "with_name",
                "modules_with_name", "root"], "Parts"),
    (sc.Control, ["throttle", "pitch", "yaw", "roll", "rcs", "sas", "sas_mode",
                  "gear", "legs", "brakes", "lights", "abort", "custom_axis01",
                  "input_mode", "state", "toggle_action_group",
                  "set_action_group", "get_action_group"], "Control"),
    (sc.AutoPilot, ["target_pitch_and_heading", "target_direction", "reference_frame",
                    "error", "engaged", "sas", "sas_mode", "wait",
                    "target_pitch", "target_heading", "reset",
                    "stopping_velocity_threshold", "reset"], "AutoPilot"),
    (sc.Engine, ["active", "thrust_limit", "throttle", "max_thrust", "thrust",
                 "part", "gimbal_limit", "can_shutdown", "gimballed",
                 "propellant_names", "available_thrust", "has_fuel"], "Engine"),
    (sc.Part, ["engine", "engines", "mass", "tag", "name", "title", "modules",
               "position", "vessel", "parent", "children"], "Part"),
    (sc.CelestialBody, ["reference_frame",
                        "non_rotating_reference_frame", "surface_position",
                        "position_at_altitude", "rotational_period",
                        "surface_gravity", "name", "mass"], "CelestialBody"),
]
# Part.engines 是【已知不存在】的，从必须通过列表里移出来单独检查
MUST_NOT = [(sc.Part, "engines"), (sc.AutoPilot, "engage"),
            (sc.AutoPilot, "disengage"), (sc.Vessel, "center_of_mass")]

bad = 0
print("=" * 66)
print("kRPC API 存在性校验  (krpc %s)" % getattr(__import__("krpc"), "__version__", "?"))
print("=" * 66)
for cls, names, label in SPEC:
    miss = [n for n in names if not hasattr(cls, n)]
    if label == "Part":
        miss = [n for n in miss if n != "engines"]
    if miss:
        bad += len(miss)
        print("[FAIL] %-16s 缺: %s" % (label, ", ".join(miss)))
    else:
        print("[ ok ] %-16s %d 个属性全部存在" % (label, len(names)))

print()
print("--- 已知陷阱（必须【不存在】）---")
for cls, name in MUST_NOT:
    ok = not hasattr(cls, name)
    print("[%s] %s.%s %s" % (" ok " if ok else "FAIL",
                             cls.__name__.strip('_'), name,
                             "不存在（符合预期）" if ok else "存在（可放心使用）"))
    if not ok:
        bad += 1

print()
print("--- 嵌套类（通过 Service 实例取）---")
for label in ("ReferenceFrame",):
    print("[%s] SpaceCenter.%s" % (" ok " if label in sc.SpaceCenter._classes else "FAIL", label))

print()
print("--- 单引擎 vs 多引擎 ---")
print("Part.engine        : 单数，非引擎部件返回 None  -> 必须判空")
print("Parts.engines      : 整个载具的引擎列表        -> 可用")
print("Part.modules       : 部件模块列表，可查 ModuleEngine")
print()
print("结果: %s" % ("全部通过" if bad == 0 else "%d 处问题" % bad))
sys.exit(0 if bad == 0 else 1)
