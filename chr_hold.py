"""CHR アドレス線をテスターで追うための保持ツール。

CHR A12 が「効いていない」と読み出し側から見えているが、
断線なのか短絡なのかは読み出しだけでは分からない。
そこで A12 を High/Low に固定したまま止めて、実際の電圧を当たれるようにする。

    python chr_hold.py 1     -> CHR A12 = High  (CHRアドレス $1000)
    python chr_hold.py 0     -> CHR A12 = Low   (CHRアドレス $0000)

測る場所は2箇所。ここが食い違えば、その間で切れている。

    328P の PB0 (物理14番)
    カートリッジ 55番

Ctrl-C で終了。終了するまで状態を保持する。
"""
import sys
import time

sys.path.insert(0, r"C:\FC-Dumper")
from fcdump import Dumper, MODE_CHR

hi = len(sys.argv) > 1 and sys.argv[1] == "1"
addr = 0x1000 if hi else 0x0000

d = Dumper("COM4")
d.set_mode(MODE_CHR)
d.set_chr_addr(addr)
print(f"CHR アドレスを ${addr:04X} で保持中 → CHR A12 = {'High (約5V)' if hi else 'Low (約0V)'}")
print("  測る場所: 328P PB0(物理14番)  と  カートリッジ 55番")
print("  両方が上の値になっていれば結線OK。片方だけなら、その間で切れている。")
print("Ctrl-C で終了")
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    d.close()
    print("\n終了")
