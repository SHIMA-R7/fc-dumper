"""PRG アドレス線が「カート内のROMまで届いているか」を1本ずつ確かめる。

ゼビウスは ROM選択時に 0x6F を能動的に出すのに、どのアドレスでも同じ値を返す。
一方 CHR はアドレスで値が変わる。つまりカートは挿さっているし電源も来ている。
PRG のアドレス経路だけが死んでいる、という仮説を検証する。

やること:

  1. $8000 を基準に、アドレス線を1本だけ反転して読む。値が変わればその線は
     ROM まで届いている。15本すべてで変わらなければ、アドレスがROMに
     入っていない(基板側の断線・接点不良)
  2. M2 パルス読み(set_cycle 1)でも同じことをする。静的読みでは応答しない
     基板があるため
  3. 比較用に、同じ手順をツインビーでやれば全ビット変化するはず

    python probe_prgaddr.py --port COM4
"""
import argparse
import sys

sys.path.insert(0, r"C:\FC-Dumper")
from fcdump import Dumper, MODE_PRG

BASE = 0x8000
N = 8                      # 1アドレスあたり読むバイト数


def sweep(d, tag, lines):
    def say(s=""):
        print(s)
        lines.append(s)

    base = d.prg_read(BASE, N)
    say(f"[{tag}] 基準 ${BASE:04X}: " + " ".join(f"{b:02x}" for b in base))
    say()

    moved = []
    for i in range(15):
        a = BASE ^ (1 << i)
        if a < 0x8000:                 # A15 を落とすとROMが外れるので飛ばす
            continue
        got = d.prg_read(a, N)
        ok = got != base
        if ok:
            moved.append(i)
        mark = "変化あり" if ok else "同じ    "
        say(f"  A{i:<2d} ${a:04X}: {mark}  " + " ".join(f"{b:02x}" for b in got))
    say()
    return moved, base


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--out", default=r"C:\FC-Dumper\probe_prgaddr.txt")
    args = ap.parse_args()

    lines = []

    def say(s=""):
        print(s)
        lines.append(s)

    d = Dumper(args.port)
    try:
        say(f"I2C: {d.ping()}")
        say()
        d.set_mode(MODE_PRG)

        d.set_cycle(0)
        static_moved, base = sweep(d, "静的読み", lines)

        d.set_cycle(1)
        pulsed_moved, pbase = sweep(d, "M2パルス読み", lines)
        d.set_cycle(0)

        say("── 判定 ──")
        drives = any(b != 0xFF for b in base)
        if not drives:
            say("$8000 が 0xFF。ROM が出力していない。")
            say("→ /ROMSEL がROMの /CE に届いていないか、カートが挿さっていない。")
            say("   probe_nocart.py を先に見る。")
        elif not static_moved and not pulsed_moved:
            say("15本すべてで値が変わらなかった。ROMは駆動しているのにアドレスが効かない。")
            say(f"→ 出している値は 0x{base[0]:02x} 固定。")
            say("   PRGアドレス線がROMまで届いていない。CHR側はアドレスで変化するので、")
            say("   カートの接触そのものは生きている。PRGアドレスの系統だけが切れている。")
            say()
            say("   見る順番:")
            say("   1. カート端子の PRG A0-A14 側(下段)を無水エタノールで清掃し、挿し直す")
            say("   2. それでも駄目なら、装置側で walk を使ってカート端子まで電圧が")
            say("      来ているか当たる:  python fcdump.py --port COM4 walk 0")
            say("      → カート0番側の A0 端子が High になっているか")
            say("   3. 端子まで来ているのにROMが反応しないなら、カート基板の")
            say("      パターン切れかROM自体の故障")
        elif static_moved and not pulsed_moved:
            say(f"静的読みでは A{static_moved} が効いた。パルス読みでは効かない。")
            say("→ 静的読みを使う。set_cycle(0) が正しい。")
        elif pulsed_moved and not static_moved:
            say(f"M2パルス読みでのみ A{pulsed_moved} が効いた。")
            say("→ この基板は M2 のエッジを要求する。dump 時は set_cycle(1) にする。")
        else:
            dead = [i for i in range(15) if i not in static_moved]
            if dead:
                say(f"効いた線: A{static_moved}")
                say(f"★ 効かない線: A{dead}")
                say("→ その線だけがROMまで届いていない。装置側かカート側かは")
                say("   walk で該当ビットを立ててカート端子を当たれば分かる。")
            else:
                say("15本すべて効いている。アドレス経路は正常。")
                say("→ 読めない原因は別。マッパーやタイミングを疑う。")
    finally:
        d.close()

    with open(args.out, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"\n保存: {args.out}")


if __name__ == "__main__":
    main()
