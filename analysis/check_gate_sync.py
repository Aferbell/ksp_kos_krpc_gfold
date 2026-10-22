import io, sys, re
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
ks = open('boot/superheavy.ks', encoding='utf-8').read()

def grab(name):
    """kOS 写法是 local x is 26.5. （数字后还有一个句号），要剥掉。"""
    m = re.search(r'local\s+' + name + r'\s+is\s+([\d.]+)', ks)
    if not m:
        return None
    return float(m.group(1).rstrip('.'))

ta, aim, gh = grab('target_alt'), grab('aim_alt'), grab('gfold_h')
print('  kOS : target_alt=%s aim_alt=%s gfold_h=%s' % (ta, aim, gh))
if None not in (ta, aim, gh):
    print('  kOS 交班门 = %.1f m' % (ta + aim + gh))
sys.path.insert(0, 'dev/starship')
import starship_config as C
print('  kRPC: TARGET_ALT=%.1f AIM_ALT=%.0f GATE_AIM_H=%.0f' % (
    C.TARGET_ALT, C.AIM_ALT, C.GATE_AIM_H))
print('  kRPC GATE_ALT = %.1f m' % C.GATE_ALT)
if None not in (ta, aim, gh):
    d = abs((ta + aim + gh) - C.GATE_ALT)
    print('  差 = %.1f m   %s' % (d, 'OK' if d < 0.01 else '**不一致**'))