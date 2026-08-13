"""不安定バイトが「アドレスに固着」か「読み出し列の位置に固着」かを切り分ける。

同じ物理アドレスを、ストリーム中の違う位置で読ませる。
  A = $8000 から1024バイト  -> $8100 はストリームの256番目
  B = $8040 から1024バイト  -> $8100 はストリームの192番目

重なり合う範囲で、同じアドレスが両方で壊れるかを見る。

  アドレスに固着   -> ROM/基板/配線そのものの問題(その線・その番地が悪い)
  列の位置に固着   -> 直前に何が起きたかで決まる = クロストーク/ノイズ側
"""

import sys
from collections import Counter

sys.path.insert(0, ".")
from fcdump import Dumper, MODE_PRG

N = 1024
PASSES = 5
A_BASE = 0x8000
B_BASE = 0x8040


def unstable_addrs(reads, base):
    """各アドレスについて、多数決から外れた回数を返す。"""
    out = {}
    for i in range(len(reads[0])):
        c = Counter(p[i] for p in reads)
        bad = sum(n for _, n in c.most_common()[1:])
        if bad:
            out[base + i] = bad
    return out


def main():
    d = Dumper("COM4")
    d.set_mode(MODE_PRG)
    a = [d.prg_read(A_BASE, N) for _ in range(PASSES)]
    b = [d.prg_read(B_BASE, N) for _ in range(PASSES)]
    d.close()

    ua = unstable_addrs(a, A_BASE)
    ub = unstable_addrs(b, B_BASE)

    # 両方が覆っている範囲だけで比べる
    lo, hi = B_BASE, A_BASE + N
    ka = {k for k in ua if lo <= k < hi}
    kb = {k for k in ub if lo <= k < hi}

    print(f"重なり範囲 ${lo:04X}-${hi - 1:04X} ({hi - lo} バイト)")
    print(f"  A(既定位置)で不安定だったアドレス: {len(ka)}")
    print(f"  B(64ずらし)で不安定だったアドレス: {len(kb)}")
    print(f"  両方で不安定だった共通アドレス   : {len(ka & kb)}")
    print()

    if not ka or not kb:
        print("→ 片方でエラーが出なかった。読み出し条件で消える = ノイズ側の疑い濃厚")
        return

    jaccard = len(ka & kb) / len(ka | kb)
    print(f"  一致度(Jaccard) = {jaccard:.2f}")
    if jaccard > 0.5:
        print("→ アドレスに固着。ROM/基板/その番地に繋がる配線を疑う")
    else:
        print("→ アドレスに固着していない。読み出し列の文脈で変わる = クロストーク/ノイズ側")

    print()
    print("  Aだけで壊れたアドレス(先頭10):", " ".join(f"${x:04X}" for x in sorted(ka - kb)[:10]))
    print("  Bだけで壊れたアドレス(先頭10):", " ".join(f"${x:04X}" for x in sorted(kb - ka)[:10]))
    print("  共通で壊れたアドレス(先頭10):", " ".join(f"${x:04X}" for x in sorted(ka & kb)[:10]))


if __name__ == "__main__":
    main()
