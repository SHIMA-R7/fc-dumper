"""「アドレスの違い」か「実行順の後半ほど壊れる」かを分離する。
A と B を交互に読み、実行順にエラー数を並べる。
"""
import sys
from collections import Counter
sys.path.insert(0, ".")
from fcdump import Dumper, MODE_PRG

N, ROUNDS = 512, 5
A_BASE, B_BASE = 0x8000, 0x8040

d = Dumper("COM4")
d.set_mode(MODE_PRG)

seq = []   # (ラベル, データ) を実行順に
for r in range(ROUNDS):
    seq.append(("A", d.prg_read(A_BASE, N)))
    seq.append(("B", d.prg_read(B_BASE, N)))
d.close()

for label in ("A", "B"):
    reads = [x for lab, x in seq if lab == label]
    ref = bytes(Counter(p[i] for p in reads).most_common(1)[0][0] for i in range(N))
    print(f"{label}(${A_BASE if label=='A' else B_BASE:04X}) 各回のエラー数:",
          [sum(1 for i in range(N) if p[i] != ref[i]) for p in reads])

print()
print("実行順ラベル:", " ".join(lab for lab, _ in seq))
