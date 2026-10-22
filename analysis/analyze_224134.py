import csv, io, os, sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
d = r'D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script\log-B1040-9-gfold'

print('=== 确证：三趟跑的是不是同一份代码（看列数 + ctrl 备注行）===')
for f in ('starship_log_20261021_195348.csv',
          'starship_log_20261021_205641.csv',
          'starship_log_20261021_212521.csv',
          'starship_log_20261021_224134.csv'):
    raw = open(os.path.join(d, f), encoding='utf-8').readlines()
    rows = list(csv.DictReader([l for l in raw if not l.startswith('#')]))
    ctrl = [l[1:].strip() for l in raw if 'ctrl kp=' in l]
    ncol = len(rows[0]) if rows else 0
    has_azrej = 'az_reject' in (rows[0] if rows else {})
    print('  %-34s 列数=%3d  az_reject=%s' % (f[13:27], ncol, 'YES' if has_azrej else 'no'))
    if ctrl:
        print('     %s' % ctrl[0])
