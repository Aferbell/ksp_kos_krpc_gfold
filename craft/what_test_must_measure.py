# what_test_must_measure.py -- 明确"测试脚本必须实测哪些量"
#
# 【为什么先做这个】.craft + GameData 能算出【静态】参数（推力、质量、dv），
#   但月面着陆程序还需要一批【只能实测】的量：
#     · 真实的 F/m（TweakScale + 变体 + 燃料消耗都会影响）
#     · Isp 的实测值（config 只给曲线，实际随压力变化）
#     · 姿态权限（力矩/惯量），决定倾角能不能跟得上
#     · RCS 的真实可用推力与平移方向（KRE 的 Vernier 有 6 个方向）
#     · 落地判定所需的几何（腿长、雷达高度基准）
#     · 燃料余量与 dv 预算的实测标定
#   本脚本列出"已知 / 待测"清单，作为测试脚本的需求规格。
import sys


def main():
    known = [
        ('质量（干/湿）',
         '从 config 质量 + LF/Ox/Mono 密度算', '4.31 / 8.43 t',
         '需确认 mono 是否被 TweakScale 改'),
        ('主推力（真空）',
         '4 × Mk-55(120kN) × 0.5^2.5', '84.9 kN',
         '需确认 free 型推力指数确为 2.5'),
        ('Isp（真空）', 'config: 305 s', '305 s',
         '需确认实际（KRE 变体可能改）'),
        ('RCS 推力', '4 × Vernier(6kN)', '24 kN',
         '需确认 6 方向是否全开'),
        ('万向节范围', 'config: gimbalRange=8 deg',
         '8 deg',
         'craft 里 gimbalActive=False ⇒ 需确认启用'),
        ('理想 dv', '火箭方程 (mr=1.957)', '2008 m/s', '—'),
        ('Mun 表面 TWR', '84.9/(8.426*1.628)', '6.19',
         '很高，可用大倾角'),
    ]
    unknown = [
        ('真实 F/m 随时间',
         '推力与质量都随燃料变化；且 TweakScale/变体可能有偏差',
         '每帧读 Engine:THRUST 与 ship:mass'),
        ('姿态权限 α = τ/I',
         '决定倾角能否跟上指令；本项目在 Kerbin 上实测滞后 0.6~0.8 s',
         '读 availabletorque 与 momentofinertia 三轴'),
        ('RCS 各向可用推力',
         'KRE Vernier 支持 fore/starboard/top 平移；需确认哪些方向有效',
         '逐块读 thrustvectors 并按方向归类求和'),
        ('雷达高度基准',
         'alt:radar 是到"最低点"还是"腿尖"？决定触地判定',
         '对比 alt:radar / altitude / ship:altitude'),
        ('腿长与触地余量',
         '腿在 y=54.608，罐底 y=54.876 ⇒ 相差 0.268 m',
         '需实测落地瞬间的 alt:radar'),
        ('燃料→dv 标定',
         '用实测 F、flow 反算 ve，标定剩余 dv',
         '满油门测 thr/flow，得 ve = thr/flow'),
        ('质量流量',
         'config 未直接给；需实测 -dm/dt',
         '满油门期间采样 m(t)'),
        ('CoM 位置',
         '决定 RCS 力臂与姿态权限；含燃料时会移动',
         '读 ship:centerofmass（需确认 kOS 支持）'),
        ('可用推力上限',
         'TweakScale + 油门限制后的真实上限',
         '读 ship:availablethrust'),
        ('最小油门',
         '若发动机有 minThrust 限制，末段细调能力受影响',
         'config minThrust=0 ⇒ 应无限制，需确认'),
    ]
    print('# 测试脚本的需求规格：已知 vs 待测')
    print('')
    print('=== 一、已可从 .craft + GameData 确定的（不需实测）===')
    print('  %-18s %-42s %-14s %s'
          % ('项目', '来源', '值', '待确认项'))
    print('  ' + '-' * 118)
    for a, b, c, d in known:
        print('  %-18s %-42s %-14s %s' % (a, b, c, d))
    print('')
    print('=== 二、★ 必须实测的（测试脚本的输出目标）===')
    print('  %-20s %-46s %s' % ('项目', '为什么只能实测', '怎么测'))
    print('  ' + '-' * 116)
    for a, b, c in unknown:
        print('  %-20s %-46s %s' % (a, b, c))
    print('')
    print('=== 三、测试场景建议（用户说：Mun 上空自由落体时运行）===')
    print('  用户条件："在 mun 上空让着陆器自由落体时运行"')
    print('  => 场景：Mun 上空（建议 20~50 km），无动力自由落体')
    print('     程序在此期间做【被动测量】，不点发动机（或只短暂脉冲）')
    print('')
    print('  【可测到的】')
    print('   · 重力加速度 g(h) 的真实剖面  <- 自由落体是测 g 的最佳时机')
    print('   · 姿态环的响应（用 RCS 打脉冲，看角速度/滞后）')
    print('   · RCS 各向可用推力')
    print('   · 雷达高度基准、腿长')
    print('   · 燃料/质量（若不点火则不变）')
    print('')
    print('  【测不到的（自由落体期间发动机不点火）】')
    print('   · 满油门推力与质量流量  <- 需要短暂点火')
    print('   · Isp 实测')
    print('   · 姿态权限 alpha（全油门与零油门不同）')
    print('  => 建议：允许程序在【末段】做一次 1~2 s 的点火脉冲来测推力')
    print('     （用户若不允许点火，就退化为"用 config 值 + 落地后标定"）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
