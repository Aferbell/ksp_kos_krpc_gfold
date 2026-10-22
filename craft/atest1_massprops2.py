# atest1_massprops2.py -- 修正版：用正确的 TweakScale 语义算质量与 TWR
#
# ============================================================================
# 【上一版的错】我把 TweakScale 的 `scale` 当成缩放倍数（scale/100），
#   于是一堆部件被算成 0 质量，TWR 算出 0.001 —— 完全错。
#
# 【正确语义（已用 craft 自身的 modMass 反推验证）】
#     s = currentScale / defaultScale          （s=1 表示【未缩放】）
#     质量 = base_mass * s^2.5   （free 型）
#   验证：Mk-55 base=0.9 t、s=0.5、craft 里 modMass=-0.7409
#         => 实际 0.9-0.7409 = 0.1591 t = 0.9 * 0.5^2.5  ✓ 指数确认为 2.5
#
#   本载具实际情况（17 个有 TweakScale 的部件里只有 4 个真的缩放了）：
#     radialLiquidEngine1-2 (Mk-55)  s=0.5   <- 唯一缩放的
#     其余全部 s=1.000（未缩放）
# ============================================================================
import math
import sys

G0 = 9.80665

# ---- 从 GameData config 读到的基准值（非猜测）----
M55 = dict(mass=0.9, thrust=120.0, isp_vac=305.0, isp_sl=275.0,
           lf_ratio=0.9, ox_ratio=1.1, gimbal=8.0)
VERNIER = dict(mass=0.09, thrust=6.0, isp_vac=260.0,
               lf_ratio=0.9, ox_ratio=1.1)
POD = dict(mass=2.6, ec=150.0, mono=30.0)
TANK = dict(mass=0.5, lf=360.0, ox=440.0)
LEG = dict(mass=0.015)
DPORT = dict(mass=0.02)

#   LF/Ox 密度 = 5 kg/单位；MonoPropellant = 4 kg/单位
RHO_LF = 0.005
RHO_OX = 0.005
RHO_MONO = 0.004

# ---- 本载具部件表（含正确的 s）----
#   (名称, 数量, s, 基准, 类型)
ITEMS = [
    ('Mk-55 "Thud" 径向发动机', 4, 0.5, M55, 'engine'),
    ('HotGasThruster-M (Vernier)', 4, 1.0, VERNIER, 'rcs'),
    ('Mk1-3 指令舱', 1, 1.0, POD, 'pod'),
    ('Rockomax8BW 燃料罐', 1, 1.0, TANK, 'tank'),
    ('restock 着陆腿', 4, 1.0, LEG, 'leg'),
    ('dockingPort3', 1, 1.0, DPORT, 'port'),
]
#   其余小部件（太阳能板×2、传感器×4、天线×1）——从 config 估
MISC = [('nfs 太阳能板', 2, 0.05), ('传感器×4', 4, 0.005),
        ('SurfAntenna', 1, 0.01)]


def main():
    print('# atest1 质量属性与推重比（修正版）')
    print('')
    print('  【TweakScale 正确语义】s = currentScale/defaultScale；')
    print('     质量 = base * s^2.5（free 型）；本载具只有 4 台 Mk-55 缩放到 s=0.5')
    print('')
    print('  %-32s %4s %6s %12s %12s'
          % ('部件', '数量', 's', '单件质量[t]', '单件推力[kN]'))
    print('  ' + '-' * 74)

    dry = 0.0
    thrust = 0.0
    rcs_thrust = 0.0
    for name, n, s, base, kind in ITEMS:
        m = base['mass'] * s ** 2.5
        th = 0.0
        if kind == 'engine':
            th = base['thrust'] * s ** 2.5      # free 型推力同指数
            thrust += th * n
        elif kind == 'rcs':
            th = base['thrust'] * s ** 2.5
            rcs_thrust += th * n
        dry += m * n
        print('  %-32s %4d %6.2f %12.4f %12.1f' % (name, n, s, m, th))

    misc_m = 0.0
    for name, n, m in MISC:
        misc_m += n * m
        print('  %-32s %4d %6.2f %12.4f %12s' % (name, n, 1.0, m, '-'))

    #   燃料（tank 未缩放 => 用 config 原值）
    lf = TANK['lf']
    ox = TANK['ox']
    fuel_m = (lf + ox) * RHO_LF
    mono_m = POD['mono'] * RHO_MONO

    print('')
    print('  === 质量汇总 ===')
    print('  %-40s %10.3f t' % ('结构干重（部件空重）', dry))
    print('  %-40s %10.3f t' % ('杂项小部件', misc_m))
    print('  %-40s %10.3f t  (LF %.0f + Ox %.0f 单位)'
          % ('主燃料 LF/Ox', fuel_m, lf, ox))
    print('  %-40s %10.3f t  (%.0f 单位)' % ('单组元 MonoPropellant',
                                            mono_m, POD['mono']))
    dry_total = dry + misc_m
    wet = dry_total + fuel_m + mono_m
    print('  ' + '-' * 60)
    print('  %-40s %10.3f t' % ('干重（不含任何燃料）', dry_total))
    print('  %-40s %10.3f t' % ('湿重（满燃料）', wet))
    print('')
    print('  === 推力 ===')
    print('  %-40s %10.1f kN' % ('主发动机（4×Mk-55, s=0.5）', thrust))
    print('  %-40s %10.1f kN' % ('RCS（4×Vernier）', rcs_thrust))
    print('')
    print('  === 推重比 ===')
    #   【单位修正 —— 我第一版写错了】
    #     推力单位 kN，质量单位 t。加速度 [m/s²] = F[N]/m[kg]
    #       = (thrust_kN * 1000) / (mass_t * 1000) = thrust_kN / mass_t
    #     所以 TWR = thrust_kN / (mass_t * g)，【不能】再乘 1000。
    #     第一版写成 thrust/(wet*1000)/g => 小了 1000 倍（TWR 报 0.006）。
    print('  %-22s %8s %10s %10s %10s'
          % ('环境', 'g', 'TWR湿', 'TWR干', '净加速度湿'))
    print('  ' + '-' * 66)
    for lab, g in (('Kerbin 海平面', 9.80665), ('Kerbin 100km', 9.4),
                   ('Mun 表面', 1.628), ('Mun 100km 轨道', 0.724)):
        twr_w = thrust / (wet * g)
        twr_d = thrust / (dry_total * g)
        print('  %-22s %8.3f %10.3f %10.3f %10.3f'
              % (lab, g, twr_w, twr_d,
                 thrust / wet - g))
    print('  （净加速度 = a_max − g；>0 才能离地/悬停）')
    print('')
    print('  === dv 预算 ===')
    ve = M55['isp_vac'] * G0
    mr = wet / dry_total
    dv = ve * math.log(mr)
    print('  Isp(真空) = %.0f s -> ve = %.1f m/s' % (M55['isp_vac'], ve))
    print('  质量比 = %.4f' % mr)
    print('  理想 dv = %.1f m/s' % dv)
    print('')
    print('  === 与 Mun 着陆需求对比 ===')
    need = 921.0
    print('  Mun 动力下降需求（前次仿真，含重力损失）= %.0f m/s' % need)
    print('  本载具理想 dv = %.0f m/s  ->  %s'
          % (dv, '充足 ✓' if dv > need * 1.3 else
             ('勉强' if dv > need else '不足 ✗')))
    print('')
    twr_mun = thrust / (wet * 1.628)
    print('  【最关键：Mun 表面 TWR = %.2f】' % twr_mun)
    print('    前次仿真结论：安全着陆需 >= 1.6；推荐 >= 2.5')
    if twr_mun >= 2.5:
        print('    -> ✓ 充足，可用大倾角快速制动')
    elif twr_mun >= 1.6:
        print('    -> ✓ 够用，但需限制倾角（<= ~40 deg）')
    elif twr_mun >= 1.0:
        print('    -> ⚠ 仅能悬停/缓降，不能从轨道速度刹下来')
    else:
        print('    -> ✗ 不足 1.0，无法悬停')
    return 0


if __name__ == '__main__':
    sys.exit(main())
