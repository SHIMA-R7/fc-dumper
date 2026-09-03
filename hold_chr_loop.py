"""CHRモードに入ったまま、$0000を無限に読み続ける。

一瞬だけモードに入って読んで抜ける通常のチェックとは違い、実際に
I2Cトランザクションが起きている「その瞬間」の電圧をテスターで
追えるようにするためのもの。

    python hold_chr_loop.py

Ctrl-Cで終了。
"""
import sys
import time
from collections import Counter

sys.path.insert(0, r"C:\FC-Dumper")
from fcdump import Dumper, MODE_CHR

d = Dumper("COM22")
try:
    d.set_mode(MODE_CHR)
    print("CHRモードで $0000 を連続読み中... Ctrl-Cで終了", flush=True)
    print("この間ずっとテスターを当てて、電圧が揺れるか見てください", flush=True)
    n = 0
    bad = 0
    t0 = time.time()
    while True:
        b = d.chr_read(0x0000, 1)[0]
        n += 1
        if not (b & 0x10):
            bad += 1
        if n % 200 == 0:
            elapsed = time.time() - t0
            print(f"  {n}回読んだ ({elapsed:.1f}秒経過)  D4=0だった回数: {bad}/{n} "
                  f"({bad/n*100:.1f}%)  直近の値: {b:#04x}", flush=True)
except KeyboardInterrupt:
    print("\n終了")
finally:
    d.close()
