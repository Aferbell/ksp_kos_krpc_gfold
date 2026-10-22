import io, sys, math
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
sys.path.insert(0, 'dev/starship')
import numpy as np
import starship_config as C
import starship_ctrl as K

print('=== C 修正：AZ_JUMP_MAX_DEG 单元验证 ===')
print('  AZ_JUMP_MAX_DEG = %.1f' % C.AZ_JUMP_MAX_DEG)
print()

def jump_reject(cur, cand, thr):
    if cur is None or thr >= 180.0:
        return cand, False, 0.0
    c = np.asarray(cur, float)
    n = np.asarray(cand, float)
    cn = float(np.linalg.norm(c)); nn = float(np.linalg.norm(n))
    if cn < 1e-9 or nn < 1e-9:
        return cand, False, 0.0
    cos_j = float(np.dot(c, n)) / (cn * nn)
    j = math.degrees(math.acos(max(-1.0, min(1.0, cos_j))))
    if j > thr:
        return np.asarray(cur, float), True, j
    return cand, False, 0.0

cases = [
    ('同向 0 deg',      (1.0, 0.0),  (1.0, 0.0),      0.0),
    ('偏 30 deg',       (1.0, 0.0),  (0.866, 0.5),   30.0),
    ('偏 59 deg',       (1.0, 0.0),  (0.515, 0.857), 59.0),
    ('偏 61 deg',       (1.0, 0.0),  (0.485, 0.875), 61.0),
    ('偏 90 deg',       (1.0, 0.0),  (0.0, 1.0),     90.0),
    ('翻转 180 deg',    (1.0, 0.0),  (-1.0, 0.0),   180.0),
]
print('  %-16s %-10s %-10s %-10s %s' % ('case', 'jump实际', '拒绝?', '跳变角', '结果'))
for tag, cur, cand, exp in cases:
    out, rej, j = jump_reject(cur, cand, C.AZ_JUMP_MAX_DEG)
    print('  %-16s %-10.1f %-10s %-10.1f %s' % (
        tag, exp, 'YES' if rej else 'no', j,
        '保持原方位' if rej else '采用新方位'))
print()
print('=== D 修正：新列 ===')
print('  Logger.COLS = %d 列' % len(K.Logger.COLS))
print('  新增:', K.Logger.COLS[-5:])
