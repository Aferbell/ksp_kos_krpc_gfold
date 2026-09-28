# diag_ap.py — 探测本机 kRPC AutoPilot 的真实 API（不猜，直接问运行时）
#
# 【背景】2026-09-26 实飞报错：
#     AttributeError: 'AutoPilot' object has no attribute 'engage'.
#         Did you mean: 'engaged'?
#   我此前凭印象写了 ap.engage()，但这个 kRPC 构建的 AutoPilot 并没有该方法。
#   本脚本把 AutoPilot 对象上【真实存在】的属性/方法全部列出来，
#   并标出哪些是可调用的、哪些是只读的，避免再靠猜。
#
# 用法：python diag_ap.py
import sys

import krpc


def main():
    conn = krpc.connect(name='diag_ap')
    sc = conn.space_center
    v = sc.active_vessel
    ap = v.auto_pilot

    print('=' * 78)
    print('AutoPilot 运行时 API 探测   vessel=%r' % v.name)
    print('=' * 78)

    names = [n for n in dir(ap) if not n.startswith('_')]
    callables, others = [], []
    for n in sorted(names):
        try:
            attr = getattr(ap, n)
        except Exception as e:                       # noqa: BLE001
            others.append((n, f'<读取失败: {type(e).__name__}>'))
            continue
        if callable(attr):
            callables.append(n)
        else:
            try:
                others.append((n, repr(attr)))
            except Exception:                        # noqa: BLE001
                others.append((n, '<不可表示>'))

    print('\n[可调用方法] (%d)' % len(callables))
    for n in callables:
        print('  %s()' % n)

    print('\n[属性] (%d)' % len(others))
    for n, val in others:
        print('  %-28s = %s' % (n, val))

    # ---- 重点：与"接管/断开"相关的名字 ----
    print('\n' + '=' * 78)
    print('与 engage/disengage 相关的候选')
    print('=' * 78)
    hits = [n for n in names
            if any(k in n.lower() for k in
                   ('engage', 'diseng', 'active', 'enable', 'start', 'stop',
                    'on', 'off', 'control', 'client'))]
    for n in sorted(hits):
        try:
            a = getattr(ap, n)
            kind = 'method' if callable(a) else 'prop'
        except Exception:                            # noqa: BLE001
            kind = '?'
        print('  %-28s (%s)' % (n, kind))

    # ---- 当前 state ----
    print('\n当前状态:')
    for n in ('engaged', 'error', 'reference_frame', 'target_direction',
              'up_reference', 'roll_threshold', 'stopping_time'):
        try:
            print('  %-22s = %r' % (n, getattr(ap, n)))
        except Exception as e:                       # noqa: BLE001
            print('  %-22s <不可读: %s>' % (n, type(e).__name__))

    print('\n结论（已核实）：本 kRPC 构建的 AutoPilot【没有 engage() 方法】。')
    print('      接管 = 置属性 engaged = True；断开 = engaged = False。')
    print('      依据：GameData/kRPC/KRPC.SpaceCenter.xml 的 P:...AutoPilot.Engaged')
    print('            "Setting to true engages the auto-pilot."')


if __name__ == '__main__':
    sys.exit(main())
