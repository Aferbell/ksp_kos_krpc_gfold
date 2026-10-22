import io, os, sys, subprocess
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
dev = r'D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script\dev'
adir = os.path.join(dev, 'analysis')

tracked = set()
out = subprocess.run(['git', 'ls-files', 'analysis'], cwd=dev,
                     capture_output=True, text=True, encoding='utf-8').stdout.splitlines()
tracked = {t.split('/')[-1] for t in out}

files = sorted(f for f in os.listdir(adir) if f.endswith('.py'))
probe = [f for f in files if f not in tracked and
         (f.startswith(('probe', 'diag', 'tmp_', '_tmp')) or f.startswith('diagnose'))]

print('拟删除 %d 个：' % len(probe))
for f in probe:
    print('  ' + f)

for f in probe:
    p = os.path.join(adir, f)
    try:
        os.remove(p)
    except OSError as e:
        print('FAIL %s: %s' % (f, e))

left = sorted(f for f in os.listdir(adir) if f.endswith('.py'))
print()
print('删除完成。analysis 根下 .py 剩 %d 个（原 %d）' % (len(left), len(files)))
