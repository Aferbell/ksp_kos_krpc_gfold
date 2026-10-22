# -*- coding: utf-8 -*-
"""analyze_phase_handoff.py — 阶段交接点与刹车可行性（只读日志）

【回答两个问题】
  1. gfold -> 末段律 -> FIN 三个交接点分别在哪，交接时载具什么状态
  2. 在那个高度/速度下，物理上还刹得住吗

用法:
  python dev/analysis/analyze_phase_handoff.py
"""
import csv
import glob
import io
import os
import re
import sys

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
_LOGDIR = os.path.join(_ROOT, 'log-B1040-9-gfold')

G0 = 9.80665
A_CAP = 30.42                 # 13 台回收构型，实测
A_NET = A_CAP - G0            # 净减速能力 = 20.6 m/s^2


def cfg():
    """从 starship_config 读实际值，不写死。"""
    p = os.path.join(_ROOT, 'dev', 'starship', 'starship_config.py')
    want = {}
    keys = ('TARGET_ALT', 'TERM_CLEARANCE', 'TERM_DESCENT_VZ',
            'FIN_ALT', 'FIN_DESCENT_VZ', 'AVER_TGT_VZ', 'LAND_ALT_EPS',
            'CLIMB_ABORT_VZ', 'FIN_TAKEOFF_GUARD')
    if not os.path.exists(p):
        return want
    with open(p, encoding='utf-8') as fh:
        for ln in fh:
            s = ln.strip()
            for k in keys:
                if (s.startswith(k + ' ') or s.startswith(k + '=')) and '=' in s:
                    val = s.split('=', 1)[1].split('#')[0].strip()
                    try:
                        want[k] = float(val)
                    except ValueError:
                        pass
    return want


def notes(path):
    out = []
    with open(path, encoding='utf-8') as fh:
        for ln in fh:
            if ln.startswith('#'):
                out.append(ln[1:].strip())
    return out


def load(path):
    with open(path, encoding='utf-8') as fh:
        raw = [ln for ln in fh if not ln.startswith('#')]
    return list(csv.DictReader(raw))


def need_alt(vz, tgt):
    """从 vz 收到 tgt 所需的【净减速距离】（用满推净加速度 A_NET）"""
    return max(0.0, (vz * vz - tgt * tgt) / (2.0 * A_NET))


def sec_gates(c):
    print('=' * 100)
    print('1. 三个交接门（全部从 starship_config.py 现读，不写死）')
    print('=' * 100)
    ta = c.get('TARGET_ALT', 17.0)
    tc = c.get('TERM_CLEARANCE', 5.0)
    fa = c.get('FIN_ALT', 1.0)
    le = c.get('LAND_ALT_EPS', 1.0)
    print('  TARGET_ALT      = %6.1f m   （规划归零点 = 着陆腿触点）' % ta)
    print('  TERM_CLEARANCE  = %6.1f m   => 规划零速点抬到 alt = %.1f m' % (tc, ta + tc))
    print('  FIN_ALT         = %6.1f m   => FIN 接管门 alt <= %.1f m' % (fa, ta + fa))
    print('  LAND_ALT_EPS    = %6.1f m   => 非FIN触地判据 alt <= %.1f m' % (le, ta + le))
    print()
    print('  ┌─────────────────────────────────────────────────┐')
    print('  │  alt = %.1f m   规划末节点（速度为 0）              │' % (ta + tc))
    print('  │  alt = %.1f m   ★ 末段下降律接管（速度环）          │' % (ta + tc))
    print('  │  alt = %.1f m   ★ FIN 接管                          │' % (ta + fa))
    print('  │  alt = %.1f m   ★ 非FIN路径判「已落地」              │' % (ta + le))
    print('  │  alt = %.1f m   规划归零点（= 腿触点）               │' % ta)
    print('  │  alt = 0.0  m   地面                                 │')
    print('  └─────────────────────────────────────────────────┘')
    print()
    print('  末段下降律窗口 = %.1f - %.1f = %.1f m' % (ta + tc, ta + fa, tc - fa))
    print('  FIN 窗口（到触点） = %.1f - %.1f = %.1f m' % (ta + fa, ta, fa))
    if abs((ta + fa) - (ta + le)) < 1e-6:
        print('  ⚠️ FIN 接管门与「非FIN触地判据」【数值重合】= %.1f m' % (ta + fa))
    print()


def sec_fin_arrival(c):
    print('=' * 100)
    print('2. 实测：FIN 接管那一刻，载具是什么状态（全部 10 趟）')
    print('=' * 100)
    print('  %-22s %8s %9s %9s %12s %10s %s' % (
        'log', 'FIN_alt', 'FIN_vz', 'FIN_vh', '刹车需高度', '可用高度', '结果'))
    ta = c.get('TARGET_ALT', 17.0)
    worst = []
    for p in sorted(glob.glob(os.path.join(_LOGDIR, 'starship_log_*.csv'))):
        ns = notes(p)
        fin = [n for n in ns if 'FIN enter' in n]
        if not fin:
            continue
        m = re.search(r'alt=([-\d.]+) vz=([-\d.]+) vh=([-\d.]+)', fin[0])
        if not m:
            continue
        a, vz, vh = float(m.group(1)), abs(float(m.group(2))), float(m.group(3))
        land = [n for n in ns if n.startswith('LANDED')]
        ab = [n for n in ns if 'abort' in n]
        res = 'LANDED' if land else ('abort' if ab else '?')
        h_need = need_alt(vz, c.get('FIN_DESCENT_VZ', 1.0))
        h_have = max(0.0, a - ta)
        flag = 'OK' if h_need <= h_have else '**不够**'
        worst.append((vz, h_need, h_have))
        print('  %-22s %8.2f %9.2f %9.2f %12.1f %10.1f %s %s' % (
            os.path.basename(p)[13:26], a, -vz, vh, h_need, h_have, res, flag))
    print()
    print('  【读法】「刹车需高度」= vz^2/(2*(a_cap-g))，a_cap=%.1f, g=%.1f => 净 %.1f m/s^2' % (
        A_CAP, G0, A_NET))
    print('          「可用高度」= FIN 入口 alt - TARGET_ALT（= FIN 段真正的行程）')
    print('          只要「刹车需高度 > 可用高度」，FIN 段接手时就已经物理上刹不住了。')
    if worst:
        bad = sum(1 for w in worst if w[1] > w[2])
        print('          => %d/%d 趟属于「刹不住」' % (bad, len(worst)))
    print()


def sec_required(c):
    print('=' * 100)
    print('3. 给定卸压速度，需要多高才能刹到 -%.1f m/s' % c.get('FIN_DESCENT_VZ', 1.0))
    print('=' * 100)
    print('  %-12s %-14s %-24s' % ('FIN入口vz', '刹车需高度', '含义'))
    tgt = c.get('FIN_DESCENT_VZ', 1.0)
    for vz in (5, 10, 20, 30, 40, 60, 80, 100):
        h = need_alt(vz, tgt)
        note = ''
        if h < 5:
            note = 'FIN_ALT=1 也不够';
        elif h < 20:
            note = '需要 FIN_ALT >= %.0f';
        else:
            note = '需要 FIN_ALT >= %.0f';
        print('  %-12s %-14s %-24s' % ('-%.0f m/s' % vz, '%.1f m' % h,
                                       (note % h) if '%' in note else note))
    print()
    print('  【结论】FIN_ALT=1.0 只能兜住 vz ≈ -%.1f m/s 以内的入口速度。' % 
        (2.0 * A_NET * 1.0 + tgt * tgt) ** 0.5)
    print()


def sec_window(c):
    print('=' * 100)
    print('4. 末段律（alt<%.1f m）与 FIN 之间有多少余量' % (c.get('TARGET_ALT', 17.0) + c.get('TERM_CLEARANCE', 5.0)))
    print('=' * 100)
    p = os.path.join(_LOGDIR, 'starship_log_20261021_172305.csv')
    if not os.path.exists(p):
        return
    rows = load(p)
    def g(r, k):
        try:
            v = r.get(k)
            return float(v) if v not in (None, '', 'nan') else None
        except (TypeError, ValueError):
            return None
    print('  本趟（唯一到达末段但未进入 FIN 的）末段帧：')
    print('  %-6s %-8s %-8s %-9s %-9s %-8s' % (
        't', 'alt', 'vz', 'av_up', 'av_need', 'thr'))
    for r in rows:
        a = g(r, 'alt')
        if a is None or a > 32:
            continue
        print('  %-6.2f %-8.2f %-8.2f %-9.2f %-9.2f %-8.3f' % (
            g(r, 't'), a, g(r, 'vz') or 0, g(r, 'av_up') or 0,
            g(r, 'av_need') or 0, g(r, 'thr_cmd') or 0))
    print()
    print('  读数：av_need 从 17.53 骤降到 9.45（一帧内），推力撤不及时 -> 爬升。')
    print()


def main():
    c = cfg()
    sec_gates(c)
    sec_fin_arrival(c)
    sec_required(c)
    sec_window(c)
    return 0


if __name__ == '__main__':
    sys.exit(main())
