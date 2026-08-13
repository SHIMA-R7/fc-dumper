"""カートを抜いた状態で PRG データバスを見て、D4/D7 の張り付きがどちら側かを決める。

症状: あるカートで PRG が全バイト 0x6F になった。0x6F = 0110 1111 なので
D4 と D7 だけが Low。ツインビーでは同じ配線で正常に読めていた。

Nano は入力に戻すとき内部プルアップを入れる(fc_nano.ino の prgDataDir)。
つまり誰も駆動していなければ 0xFF が読めるはず。カートを抜けば
「誰も駆動していない」が保証されるので、そこで 0xFF に戻るかどうかで切り分く。

    カートを抜いて実行 → 0xFF   : カート側。装置は無罪
    カートを抜いて実行 → 0x6F   : Nano D6/D9 側。配線かはんだ

$0000 を読むと Uno は /ROMSEL を High にする(cmdPrgRead の romsel 判定)ので、
制御線は全解除 = IDLE と同じ状態になる。これが素のバス状態。

    python probe_nocart.py                  # ポート自動検出
    python probe_nocart.py --port COM8
"""
import argparse
import sys

sys.path.insert(0, r"C:\FC-Dumper")
from fcdump import Dumper, MODE_PRG

# PRG データ線 → Nano のピン → カートリッジの端子番号
PINMAP = [
    ("D0", "D2", 43), ("D1", "D3", 42), ("D2", "D4", 41), ("D3", "D5", 40),
    ("D4", "D6", 39), ("D5", "D7", 38), ("D6", "D8", 37), ("D7", "D9", 36),
]

N = 64


def find_port():
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    if not ports:
        raise SystemExit("シリアルポートが1つも見つからない。USBを挿したか確認する")
    for p in ports:
        blob = f"{p.description} {p.manufacturer or ''}".lower()
        if any(k in blob for k in ("arduino", "ch340", "ch341", "usb-serial", "wch")):
            return p.device
    raise SystemExit(
        "それらしいポートを特定できなかった。--port で指定する\n  候補: "
        + ", ".join(f"{p.device} ({p.description})" for p in ports)
    )


def bits(data):
    """ビットごとに、常に1か・常に0か・混ざっているかを返す。"""
    out = []
    for i in range(8):
        ones = sum((b >> i) & 1 for b in data)
        out.append("1" if ones == len(data) else "0" if ones == 0 else "mix")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--out", default=r"C:\FC-Dumper\probe_nocart.txt")
    args = ap.parse_args()

    port = args.port or find_port()
    lines = []

    def say(s=""):
        print(s)
        lines.append(s)

    say("★ カートリッジを抜いた状態で実行すること。挿したままでは意味がない。")
    say(f"ポート: {port}")
    say()

    d = Dumper(port)
    try:
        alive = d.ping()
        say(f"I2C: {alive}")
        if not all(alive.values()):
            say("  ⚠ 応答しないチップがある。この状態の読み値は信用できない")
        say()

        d.set_mode(MODE_PRG)
        idle = d.prg_read(0x0000, N)      # /ROMSEL=High → 制御線は全解除
        sel = d.prg_read(0x8000, N)       # /ROMSEL=Low  → 本来ROMが駆動する側

        say(f"IDLE ($0000 / 全制御線を解除) {N}バイト:")
        say("  " + " ".join(f"{b:02x}" for b in idle[:32]))
        say("  出現値: " + ", ".join(
            f"{v:02x}x{idle.count(v)}" for v in sorted(set(idle))))
        say()
        say(f"ROM選択中 ($8000 / /ROMSEL=Low) {N}バイト:")
        say("  " + " ".join(f"{b:02x}" for b in sel[:32]))
        say("  出現値: " + ", ".join(
            f"{v:02x}x{sel.count(v)}" for v in sorted(set(sel))))
        say()

        st = bits(idle)
        say("IDLE時のビットごとの状態(カートが無いので全ビット1が正常):")
        stuck = []
        for i, (sig, nano, cart) in enumerate(PINMAP):
            if st[i] == "1":
                say(f"  PRG {sig} (Nano {nano:3s} カート{cart}番): OK (常に1)")
            elif st[i] == "0":
                say(f"  PRG {sig} (Nano {nano:3s} カート{cart}番): ★ 常に0 = Lowに固定")
                stuck.append((sig, nano, cart))
            else:
                say(f"  PRG {sig} (Nano {nano:3s} カート{cart}番): ⚠ 値が暴れている")
                stuck.append((sig, nano, cart))
        say()

        say("── 判定 ──")
        if not stuck:
            say("カートを抜けば全ビットが1に戻った。")
            say("→ 装置側は正常。D4/D7 を引いていたのは【カートリッジ側】。")
            say("   そのカートの基板を疑う。端子の汚れ、パターン切れ、または")
            say("   ツインビーと違うマッパー/基板でその2本が別用途に使われている可能性。")
            say("   次: 端子を清掃して挿し直し、もう一度ツインビーが読めるか確認する。")
        else:
            names = "、".join(f"PRG {s}(Nano {n} / カート{c}番)" for s, n, c in stuck)
            say(f"カートを抜いても {names} が Low のままだった。")
            say("→ 【装置側】の不具合。カートは無罪。")
            say("   Nano の該当ピンから先を疑う。見る順番:")
            say("   1. Nano のピンとカートスロット端子の間のはんだブリッジ(隣接ピンとの短絡)")
            say("   2. UEW線の被覆が溶けてGNDや隣の線と接触していないか")
            say("   3. Nano を単体にして(カートスロットへの線を外して)同じ読みをする")
            say("      → そこで1に戻るなら配線側、0のままなら Nano のポートが壊れている")
        say()
        say("参考: ツインビーは同じ配線で読めていたので、装置側なら抜き差しの過程で")
        say("      生じた新しい不具合ということになる。")
    finally:
        d.close()

    with open(args.out, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"\n保存: {args.out}")


if __name__ == "__main__":
    main()
