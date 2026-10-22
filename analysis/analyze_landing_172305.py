# -*- coding: utf-8 -*-
"""analyze_landing_172305.py — 20261021_172305 趟着陆分析（A+B 姿态修正后首次实飞）

【这一趟发生了什么】
  姿态问题【已解决】：pid_sat_x/z 全程 <=3%，att_err 中位 2.2 deg。
  但末段出现【爬升】vz=+5.38，程序按 climb abort 退出，离地 6.4 m 未触地。

【本脚本查什么】
  §1 A+B 是否真的生效（姿态）
  §2 竖直通道跟踪精度（vz vs plan_vz）
  §3 末段为什么会爬升：av_need 上界在刹停瞬间失效
  §4 FIN_DESCENT_VZ 2.0 -> 1.0 的影响面（对本趟无效）
  §5 未定事项（必须在报告里如实标注）

用法:
  python dev/analysis/analyze_landing_172305.py
"""
import csv
import io
import os
import statistics
import sys

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
_LOGDIR = os.path.join(_ROOT, 'log-B1040-9-gfold')
_LOG = os.path.join(_LOGDIR, 'starship_log_20261021_172305.csv')
_LOG_PREV = os.path.join(_LOGDIR, 'starship_log_20261021_154351.csv')

G0 = 9.80665
TMAX = 30.754659e6


def load(path):
    if not os.path.exists(path):
        return None
    with open(path, encoding='utf-8') as fh:
        raw = [ln for ln in fh if not ln.startswith('#')]
    return list(csv.DictReader(raw))


def g(row, key):
    try:
        v = row.get(key)
        return float(v) if v not in (None, '', 'nan') else None
    except (TypeError, ValueError):
        return None


def med(vals):
    v = [x for x in vals if x is not None]
    return statistics.median(v) if v else float('nan')


def sec1(rows, prev):
    print('=' * 98)
    print('1. A+B 是否生效：姿态指标对比（改前 154351 vs 改后 172305）')
    print('=' * 98)
    hdr = '%-14s %8s %8s %9s %7s %7s %9s'
    print('  ' + hdr % ('log', 'attE50', 'attEMax', 'tiltErr50', 'satX%',
                       'satZ%', 'avelPk'))
    for tag, rr in (('改前 154351', prev), ('改后 172305', rows)):
        if not rr:
            continue
        seg = [r for r in rr if (g(r, 't') or 99) < 21.0]
        ae = [g(r, 'att_err') for r in seg if g(r, 'att_err') is not None]
        dt = [abs(g(r, 'tilt_act') - g(r, 'tilt_cmd')) for r in seg
              if g(r, 'tilt_act') is not None and g(r, 'tilt_cmd') is not None]
        sx = [g(r, 'pid_sat_x') for r in seg if g(r, 'pid_sat_x') is not None]
        sz = [g(r, 'pid_sat_z') for r in seg if g(r, 'pid_sat_z') is not None]
        av = [abs(g(r, 'avel_p')) for r in seg if g(r, 'avel_p') is not None]
        print('  ' + hdr % (
            tag, '%.1f' % med(ae), '%.1f' % max(ae), '%.1f' % med(dt),
            '%.0f%%' % (100.0 * sum(1 for v in sx if v > .5) / len(sx)),
            '%.0f%%' % (100.0 * sum(1 for v in sz if v > .5) / len(sz)),
            '%.3f' % max(av)))
    print()
    print('  判定：attE50 41.7 -> 2.2（降 95%），satX 70% -> 3%，avelPk 3.09 -> 0.28。')
    print('        A+B 完全达成设计目标，姿态不再是瓶颈。')


def sec2(rows):
    print()
    print('=' * 98)
    print('2. 竖直通道跟踪：vz 是否跟着 plan_vz 走')
    print('=' * 98)
    recs = []
    for r in rows:
        t, vz, pv = g(r, 't'), g(r, 'vz'), g(r, 'plan_vz')
        alt, pa = g(r, 'alt'), g(r, 'plan_alt')
        if None in (t, vz, pv):
            continue
        recs.append((t, vz - pv, (alt - pa) if pa is not None else None))
    print('  %-14s %11s %11s %11s %11s' % ('阶段', 'vz误差p50', 'vz误差max',
                                             '高度误差p50', '高度误差max'))
    for lo, hi, lab in ((0, 3, '早期 0~3s'), (3, 15, '中段 3~15s'),
                        (15, 18, '末段 15~18s'), (18, 99, 'FIN前 18s+')):
        seg = [x for x in recs if lo <= x[0] < hi]
        if not seg:
            continue
        e = [x[1] for x in seg]
        ea = [x[2] for x in seg if x[2] is not None]
        print('  %-14s %+11.2f %+11.2f %+11.2f %+11.2f' % (
            lab, med(e), max(e, key=abs),
            med(ea) if ea else 0.0, max(ea, key=abs) if ea else 0.0))
    print()
    area = 0.0
    for i in range(1, len(rows)):
        t0, t1 = g(rows[i - 1], 't'), g(rows[i], 't')
        if None in (t0, t1) or t1 - t0 < 1e-9:
            continue
        a0 = g(rows[i - 1], 'vz') - g(rows[i - 1], 'plan_vz')
        a1 = g(rows[i], 'vz') - g(rows[i], 'plan_vz')
        area += 0.5 * (a0 + a1) * (t1 - t0)
    print('  ∫(vz - plan_vz)dt = %+.1f m  （负 = 实际比规划多掉了这么多米）' % area)
    print()
    print('  读法：早期差 -23 m/s（开局就比规划掉得快），中段收敛到 0.5 以内，')
    print('        末段 18s 后又掉到 -3.5 m/s。前两个数是高度跟踪的面积积分，')
    print('        说明控制器一直在补这个欠账，越到后面越补不动。')


def sec3(rows):
    print()
    print('=' * 98)
    print('3. 末段为什么爬升：av_need 上界在刹停瞬间失效')
    print('=' * 98)
    print('  %-6s %-8s %-8s %-8s %-9s %-11s %-8s %-7s' % (
        't', 'alt', 'vz', 'av_plan', 'av_need', 'need-plan', 'av_up', '削?'))
    ncut = n = 0
    tail = [r for r in rows if (g(r, 't') or 0) > 17.5]
    for r in tail:
        t = g(r, 't')
        ap, an, au = g(r, 'av_plan'), g(r, 'av_need'), g(r, 'av_up')
        if None in (ap, an):
            continue
        d = an - ap
        cut = '削' if d < -0.05 else ''
        if cut:
            ncut += 1
        n += 1
        print('  %-6.2f %-8.2f %-8.2f %-8.2f %-9.2f %+-11.2f %-8.2f %-7s' % (
            t, g(r, 'alt') or 0, g(r, 'vz') or 0, ap, an, d, au or 0, cut))
    print()
    print('  av_plan > av_need 的帧：%d/%d = %.0f%%' % (ncut, n, 100.0 * ncut / max(1, n)))
    print()
    print('  机理（四步）：')
    print('    ① av_need = g0 + (vz^2 - tgt^2) / (2*h)')
    print('    ② 刹停瞬间 vz 从 -7.84 变 -0.06，分子 (vz^2 - tgt^2) 由正转负')
    print('    ③ 于是 need 从 17.53 骤降到 9.45 -> a_up 被上界削到 9.45')
    print('    ④ 但此刻油门仍在 0.53（上一帧的指令），推力撤不及时')
    print('    => 载具继续上升 -> vz 转正 -> climb abort')
    print()
    print('  【关键数据】vz 从 -15.05 到 +5.38，共用 1.14 s（t=19.61~20.75）。')


def sec4():
    print()
    print('=' * 98)
    print('4. FIN_DESCENT_VZ 2.0 -> 1.0 的影响面')
    print('=' * 98)
    cfg = os.path.join(_ROOT, 'dev', 'starship', 'starship_config.py')
    keys = ('FIN_DESCENT_VZ', 'TERM_DESCENT_VZ', 'AVER_TGT_VZ', 'FIN_ALT',
            'FIN_KD', 'FIN_UP_MIN', 'FIN_TAKEOFF_GUARD',
            'CLIMB_ABORT_VZ', 'TARGET_ALT', 'TERM_CLEARANCE', 'LAND_ALT_EPS')
    vals = {}
    if os.path.exists(cfg):
        with open(cfg, encoding='utf-8') as fh:
            for ln in fh:
                s = ln.strip()
                for k in keys:
                    if (s.startswith(k + ' ') or s.startswith(k + '=')) and '=' in s:
                        vals[k] = s.split('=', 1)[1].split('#')[0].strip()
    print('  %-20s %-12s %s' % ('常量', '当前值', '作用相位'))
    phase = {
        'FIN_DESCENT_VZ': 'FIN 段目标垂速（alt <= TARGET_ALT+FIN_ALT）',
        'TERM_DESCENT_VZ': '【死常量】只在注释里出现，代码未引用',
        'AVER_TGT_VZ': 'av_need 上界公式里的目标垂速（gfold 全段）',
        'FIN_ALT': 'FIN 触发门高度（相对 TARGET_ALT）',
        'FIN_KD': 'FIN 竖直微分项，实测最优 0',
        'FIN_UP_MIN': 'FIN 推力下限（悬停比例），保证万向节有力矩',
        'FIN_TAKEOFF_GUARD': 'FIN 段爬升保护门',
        'CLIMB_ABORT_VZ': '非 FIN 段爬升中止门',
        'TARGET_ALT': '着陆腿触点高度',
        'TERM_CLEARANCE': '末段下降律切换高度',
        'LAND_ALT_EPS': 'alt <= TARGET_ALT+该值 视为触地',
    }
    for k in keys:
        print('  %-20s %-12s %s' % (k, vals.get(k, '(未找到)'), phase.get(k, '')))
    print()
    print('  【结论】')
    print('    · FIN_DESCENT_VZ 只在 fin_control() 用，而 FIN 只在 alt<=18 m 启动。')
    print('    · 本趟最低 alt=22.63 m，【从未进入 FIN 门】=> 改它对这一趟完全无效。')
    print('    · 真正决定这一趟落地速度的是 av_need（用 AVER_TGT_VZ=2.0），')
    print('      以及 av_need 在刹停瞬间失效（见 §3）。')
    print('    · 改成 1.0 的意义是【口径统一】：让 FIN 的目标垂速与 AVER_TGT_VZ')
    print('      一致，避免 FIN 段做二次切换。但这需要同时改 AVER_TGT_VZ，')
    print('      否则两个相位目标不一致。')


def sec5(rows):
    print()
    print('=' * 98)
    print('5. 未定事项（必须在报告里如实标注，不能当成结论）')
    print('=' * 98)
    pts = [(g(r, 't'), g(r, 'vz'), g(r, 'loop_dt'), g(r, 'thrust_n'),
            g(r, 'mass')) for r in rows if (g(r, 't') or 0) > 19.5]
    print('  %-6s %-9s %-10s %-12s %-12s %-11s' % (
        't', 'loop_dt', 'dvz/dt', 'a_thrust-g', '满推上限', '判定'))
    for i in range(1, len(pts)):
        t0, v0, _, _, _ = pts[i - 1]
        t1, v1, ld, th, m = pts[i]
        if None in (t0, t1, v0, v1, ld) or ld < 1e-6:
            continue
        dvd = (v1 - v0) / ld
        anet = (th / m - G0) if (th and m) else 0.0
        cap = TMAX / m - G0 if m else 0.0
        verdict = '**超物理上限**' if dvd > cap + 1.0 else ''
        print('  %-6.2f %-9.3f %+-10.2f %+-12.2f %-12.2f %-11s' % (
            t1, ld, dvd, anet, cap, verdict))
    print()
    print('  末两帧实测 dvz/dt = +38.9 / +22.6 m/s^2，而满推理论上限只有 23.4。')
    print('  => 【物理上不可能只由推力产生】。三个可能原因，本脚本无法区分：')
    print('     (a) 载具在 t≈20.4 已触地/触地形，地面反作用力把 vz 顶成正值')
    print('     (b) 该段 loop_dt 与实际游戏时间不符（全段 sum(loop_dt) 比墙钟')
    print('         少 15%，末段尤甚），导致 dvz/dt 被高估')
    print('     (c) 参考系/速度采样异常')
    print('  【行动】下一次实飞加一列 situation（kRPC 的 landed/splashed/')
    print('         flying 枚举），并在 climb abort 时同时记录 alt 与 situation，')
    print('         才能一步判定到底是 (a) 还是 (b)。')


def main():
    rows = load(_LOG)
    if rows is None:
        print('找不到日志：%s' % _LOG)
        return 1
    prev = load(_LOG_PREV)
    print('日志：%s  (%d 帧)' % (os.path.basename(_LOG), len(rows)))
    if prev:
        print('对照：%s  (%d 帧)' % (os.path.basename(_LOG_PREV), len(prev)))
    print()
    sec1(rows, prev)
    sec2(rows)
    sec3(rows)
    sec4()
    sec5(rows)
    return 0


if __name__ == '__main__':
    sys.exit(main())
