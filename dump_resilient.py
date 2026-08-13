"""諦めない吸出し機。成功するまで回し続ける。

■ なぜ必要か
この装置は 0% で読める状態と、まったく読めない状態を行き来する。
Uno が USB ごと落ちることさえある。原因は掴めていないが、
待っていても直らないので「通る瞬間を捕まえる」作りにした。

■ 遠回りした経緯
最初は発熱を疑い、失敗のたび60秒冷ました。44回・計45分冷やしても回復しなかった。
次に回数で押そうとしたが、600回試して一度も通らなかった。

原因は装置ではなく**採用条件**だった。実測すると壊れるのは1バイト単位で頻度も数%。
なのに「512バイトが2回続けて完全一致」を要求していたので、
まず揃うはずがなかった。多数決に変えた途端に問題ではなくなる。

■ 方針
  - 512Bずつ、7回読んでバイトごとの多数決で決める。票が割れたバイトだけ読み足す
  - それでも決まらなければ手を変える:
    読み方を静的↔M2パルスで切り替え / モードを入れ直す / 接続を張り直す
  - 最後に検証を通ったときだけファイルを書く。壊れたものは絶対に残さない
  - 検証に落ちたら最初からやり直す
"""

import hashlib
from collections import Counter
import sys
import time
import traceback

import serial
import serial.tools.list_ports

sys.path.insert(0, r"C:\FC-Dumper")
from fcdump import (Dumper, DumperError, MODE_IDLE, MODE_PRG, MODE_CHR,
                    CMD_PRG_READ, CMD_CHR_READ, write_ines)

PORT = "COM4"
OUT = r"C:\FC-Dumper\twinbee.nes"
LOG = r"C:\FC-Dumper\dump_log.txt"
# ■ チャンクを小さく保つ理由
# 好調な波は短い。512バイトを読み切る前に不調へ落ちるので、
# 何度やっても票が割れて $8000 から一歩も進まなかった。
# 64バイトなら7回投票しても 0.2 秒程度で、ひとつの波に収まる。
# 転送回数は増えるが、全体では 32KB を数分で読める。
CHUNK = 64

# 期待される先頭16バイト。エラー率0%のときに繰り返し確認した値で、
# NESデータベースの "RC807.1.0.850328" とも一致している。
TRUTH16 = bytes([0x52, 0x43, 0x38, 0x30, 0x37, 0x00, 0x31, 0x2E,
                 0x30, 0x00, 0x38, 0x35, 0x30, 0x33, 0x32, 0x38])


def log(msg):
    line = f"[{time.strftime('%H:%M:%S')}] {msg}"
    with open(LOG, "a", encoding="utf-8") as f:
        f.write(line + "\n")
    print(line, flush=True)


def wait_port(timeout=180):
    """USBごと落ちることがある。戻ってくるまで待つ。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if PORT in [p.device for p in serial.tools.list_ports.comports()]:
            time.sleep(1.5)
            return True
        time.sleep(2)
    return False


class Rig:
    """接続が切れても自分で張り直す装置ハンドル。"""

    def __init__(self):
        self.d = None
        self.mode = MODE_IDLE
        self.reconnects = 0
        self.pulsed = 0
        self.connect()

    def connect(self):
        if self.d:
            try:
                self.d.ser.close()
            except Exception:
                pass
            self.d = None
        if not wait_port():
            raise DumperError("ポートが戻ってこない")
        for _ in range(5):
            try:
                self.d = Dumper(PORT)
                self.d.set_mode(self.mode)
                return
            except Exception:
                time.sleep(3)
        raise DumperError("接続を張り直せない")

    def set_mode(self, m):
        self.mode = m
        self.d.set_mode(m)

    def chunk(self, cmd, a, k):
        return self.d._chunk(cmd, a, k, 10.0)

    def prg_write(self, addr, val):
        self.d.prg_write(addr, val)

    def set_cycle(self, pulsed):
        self.pulsed = 1 if pulsed else 0
        self.d.set_cycle(self.pulsed)


# ■ A4 の間欠不良を打ち消す
# 実測で、誤りの正体が完全に分かった。壊れるバイトは必ず「+16 先の値」に化ける。
# 30回読んで、誤った9回はすべて同じ位置・同じ値(+16先の中身)だった。
# これは PRG A4 (Uno D6 → カート9番) がその瞬間だけ High に化けているということ。
#
# 性質が分かっているので除去できる。あるバイトの候補のうち
# 「+16 先のバイトと同じ値」は A4 が化けて紛れ込んだものと見なし、票から外す。
# 本当にその値である場合もあるので、外した結果ゼロになるなら元の最頻値を使う。
SHIFT = 16
# ■ 投票回数
# CHR の雑音は1回のセッション内では消せない。21回に増やしても誤りは減らず、
# むしろ「その時刻には誤った値が安定して多数派」という形で出る。
# 別の時刻に読み直すと満票で正しい値が返る(実測: 21回中21回)。
# だから1回あたりは軽くして早く終わらせ、ダンプそのものを何度も繰り返して
# 時刻をまたいだ多数決で潰す(dump_consensus.py)。
NVOTE = 7
NVOTE_CHR = 9
# 掃除して空のままがこの票数続いたら、化けではなく本当にその値だと認める
GHOST_OK = 25
# 拮抗したバイトを「首位のみ」で決めてよい票数。ここに達するまでは粘る。
WEAK_AFTER = 30
# 僅差で決めたバイトの件数。完成時に申告する。
WEAK = [0]


def _clean(votes, i, k):
    """位置 i の票から、A4化けで紛れ込んだ候補を除く。

    A4=1 側の番地(オフセットの bit4 が立っている位置)は化けようがない。
    A4 が High に化けても行き先は自分自身だからで、実測でも常に満票になる。
    だから A4=1 側は素の多数決でよく、A4=0 側だけこの掃除を通す。
    """
    c = votes[i]
    if (i & SHIFT) or i + SHIFT >= k:
        return c                      # A4=1側、または+16先が塊の外
    ghost = votes[i + SHIFT].most_common(1)[0][0]
    return Counter({v: n for v, n in c.items() if v != ghost})


def decided(votes, i, k):
    """採用してよいか。ゴーストを除いた票で判断する。

    以前は掃除して空になったら元の票に差し戻していたが、それでは
    「全部が化けた回」にゴーストをそのまま採用してしまう。
    空なら未確定のままにして読み足す。本当にその値なら、
    読み足しても空のままなので、最後に GHOST_OK 回で観念して採用する。
    """
    c = _clean(votes, i, k)
    top = c.most_common(2)
    if not top:
        return votes[i].total() >= GHOST_OK      # 化けではなく本当にその値だった
    # 対抗馬との差で見る。雑音が散らばると「他の合計」では厳しすぎて決まらない。
    second = top[1][1] if len(top) > 1 else 0
    if top[0][1] >= 4 and top[0][1] >= 2 * second:
        return True
    # ■ 僅差でも観念する場合
    # 拮抗するバイトが実在する(実測 d0×13 対 10×12)。原因はデータ線の接触で、
    # 多数決では原理的に割り切れない。ここで永久に粘ると1バイトのために
    # 全体が完成しないので、十分な票を集めたうえで首位を採る。
    # ただし「僅差で決めた」ことは数えて申告する。黙って通さない。
    # 判定に使うのは「実際に何回読んだか」であって、掃除後に残った票数ではない。
    # A4 が高頻度で化けている間はゴーストばかり積まれ、掃除後の票はいつまでも
    # 増えない。それを条件にしていたので永久に決まらなかった。
    if votes[i].total() >= WEAK_AFTER and top[0][1] > second:
        WEAK[0] += 1
        return True
    return False


def pick(votes, i, k):
    c = _clean(votes, i, k)
    return (c or votes[i]).most_common(1)[0][0]


def health(rig, n=10):
    """$8000 の既知16バイトを n 回読み、完全一致した割合を返す。

    装置には好調な波と不調な波がある。不調な最中に票を積んでも
    汚れた票が増えるだけなので、読む前に必ずここで状態を測る。
    """
    was = rig.mode
    if was != MODE_PRG:
        rig.set_mode(MODE_PRG)
    ok = 0
    for _ in range(n):
        try:
            if rig.chunk(CMD_PRG_READ, 0x8000, 16) == TRUTH16:
                ok += 1
        except (DumperError, serial.SerialException, OSError):
            pass
    if was != MODE_PRG:
        rig.set_mode(was)
    return ok / n


def wait_healthy(rig, label, need=0.8, restore=None):
    """好調な波が来るまで待つ。来たら True。

    実測: 好調なときは10/10で完全一致し、不調なときは A4 が96%化ける。
    間を取らずに二値で振れるので、閾値で待てば無駄読みを避けられる。
    """
    for i in range(600):
        try:
            h = health(rig)
        except (DumperError, serial.SerialException, OSError):
            h = 0.0
        if h >= need:
            if i:
                log(f"    {label}: 好調になった(一致率 {h:.0%})。読み出しを再開")
            return True
        if i % 10 == 0:
            log(f"    {label}: 待機中 一致率 {h:.0%} ({i}回目)")
        time.sleep(3)
        if i and i % 20 == 0:
            try:
                rig.connect()
                if restore:
                    restore(rig)
            except Exception:
                pass
    return False


def read_block(rig, cmd, addr, size, label, restore=None):
    """size バイトを、正しいと確信できるまで粘って読む。

    restore は接続を張り直したあとに呼ぶ後始末(CHRのバンク再選択など)。
    """
    out = bytearray()
    slow = 0
    while len(out) < size:
        want = min(CHUNK, size - len(out))
        # 末尾の16バイトは「+16先」を参照できず A4化けを見抜けない。
        # そこで毎回16バイト余分に読み、余りは捨てて次のチャンクへ進む。
        k = want + SHIFT
        a = addr + len(out)
        if cmd == CMD_CHR_READ:
            a &= 0x1FFF
            k = min(k, 0x2000 - a)        # CHRの窓(8KB)を越えない
        else:
            k = min(k, 0x10000 - a)       # 16bitのアドレス空間を越えない
        k = max(k, want)

        got = None
        # ■ 票は「好調な波の最中に積んだもの」だけを信じる
        # 貯め続ける案は失敗した。不調な波では A4 が96%化けるので、
        # 積むほど誤った値が優勢になってしまう(実測: 正値1票に対し誤値24票)。
        # 割れたら票ごと捨て、好調になるまで待ってから積み直す。
        votes = [Counter() for _ in range(k)]
        for attempt in range(60):
            try:
                base_votes = NVOTE_CHR if cmd == CMD_CHR_READ else NVOTE
                for _ in range(base_votes if not attempt else 5):
                    r = rig.chunk(cmd, a, k)
                    for i, b in enumerate(r):
                        votes[i][b] += 1

                # ■ 読み足しの上限
                # 「3票差以上」を要求する以上、票が少ないと永久に条件を満たせない。
                # 真の値がわずかに優勢なだけでも、標本を増やせば差は開く。
                # 45票程度で打ち切っていたので CHR の2バイトが決まらなかった。
                for _ in range(60):
                    weak = [i for i in range(want) if not decided(votes, i, k)]
                    if not weak:
                        break
                    r = rig.chunk(cmd, a, k)                    # 決まらない分だけ足す
                    for i in weak:
                        votes[i][r[i]] += 1

                if all(decided(votes, i, k) for i in range(want)):
                    got = bytes(pick(votes, i, k) for i in range(want))
                    break
                weak = [i for i in range(want) if not decided(votes, i, k)]

                slow += 1
                log(f"    {label} ${a:04X}: {len(weak)}バイトの票が割れた({attempt}回目)")
                # 決まらないのは装置が不調だから。ここで粘っても汚れた票が増えるだけ。
                # 好調な波を待ち、そのうえで積んだ票だけを信じる。
                votes = [Counter() for _ in range(k)]      # 不調中の票は捨てる
                if not wait_healthy(rig, label, restore=restore):
                    raise DumperError("好調な波が来ない")
                if restore:
                    restore(rig)
            except (DumperError, serial.SerialException, OSError) as e:
                log(f"    {label} ${a:04X}: 通信断 ({type(e).__name__})。張り直す")
                rig.reconnects += 1
                try:
                    rig.connect()
                    if restore:
                        restore(rig)
                except Exception as e2:
                    log(f"    張り直し失敗: {e2}。30秒待つ")
                    time.sleep(30)
                time.sleep(5)

        if got is None:
            raise DumperError(f"{label} ${a:04X} がどうしても安定しない")

        out += got
        if slow and len(out) % (4 * CHUNK) == 0:
            time.sleep(2)          # 崩れた履歴があるうちは休み休み進む
    return bytes(out)


def verify(prg, banks):
    """書き出してよいかを判定する。壊れたものを残さないための最後の関門。"""
    bad = []
    if prg[:16] != TRUTH16:
        bad.append(f"先頭16Bが既知の正解と違う: {prg[:16].hex(' ')}")

    v = prg[-6:]
    vec = {"NMI": v[1] << 8 | v[0], "RESET": v[3] << 8 | v[2], "IRQ": v[5] << 8 | v[4]}
    for name, addr in vec.items():
        if not 0x8000 <= addr <= 0xFFFF:
            bad.append(f"{name}ベクタ ${addr:04X} が範囲外")

    if prg[:1024] == prg[1024:2048]:
        bad.append("PRGが1KB周期で折り返している(アドレス線)")
    if len(prg) >= 32768 and prg[:16384] == prg[16384:]:
        bad.append("PRG前半と後半が同一(A14が効いていない)")

    # ■ CHR側の折り返し検査
    # PRG側には入れていたのに CHR 側が抜けていて、A12 が死んだまま完成扱いにしてしまった。
    # 前半4KBと後半4KBが一致するのは、A12 が効かず同じ絵柄を2回読んだということ。
    # スプライト($0000側)だけ正しく背景($1000側)が壊れる、という形で表に出る。
    for i, b in enumerate(banks):
        if b[:4096] == b[4096:8192]:
            bad.append(f"CHRバンク{i}の前半4KBと後半4KBが同一(CHR A12が効いていない)")

    hs = [hashlib.md5(b).hexdigest() for b in banks]
    if len(set(hs)) != len(hs):
        bad.append(f"CHRバンクに同じものがある: {[h[:8] for h in hs]}")
    for i, b in enumerate(banks):
        if len(set(b)) < 16:
            bad.append(f"CHRバンク{i}の中身が乏しい(種類{len(set(b))})")
    return vec, bad


def attempt(n):
    log(f"===== 試行 {n} =====")
    rig = Rig()

    rig.set_mode(MODE_PRG)
    if not wait_healthy(rig, "PRG"):
        raise DumperError("好調な波が来ない")
    log("PRG 32KB を読む")
    prg = read_block(rig, CMD_PRG_READ, 0x8000, 32768, "PRG")
    log(f"  先頭16B {prg[:16].hex(' ')}")

    # ■ ツインビーの CHR は 16KB (2バンク)
    # Mapper87 は 32KB まで扱えるが、実物は 16KB しか積んでいない。
    # NESカートDBの記載(PRG 32K / CHR 16K)とも一致する。
    # 実際、書込値 0 と 1、2 と 3 がそれぞれ同じ中身を返す。
    # 以前これを「バンク切り替えの故障」と誤判定して正しいダンプを弾いていた。
    # 4バンク別内容に見えたのは、装置が不安定だった時期の読み出しノイズだった。
    banks = []
    for val in (0, 2):                         # D1 のみが効く。D0 は未使用
        bank = (val >> 1) & 1

        def restore(r, _v=val):
            r.set_mode(MODE_PRG)
            r.prg_write(0x6000, _v)
            r.set_mode(MODE_CHR)

        rig.set_mode(MODE_PRG)
        rig.prg_write(0x6000, val)
        rig.set_mode(MODE_CHR)
        log(f"CHR bank {bank} (書込値 {val:#04x}) を読む")
        blk = read_block(rig, CMD_CHR_READ, 0, 8192, f"CHR{bank}", restore=restore)
        banks.append((bank, blk))

    banks.sort()
    ordered = [b for _, b in banks]

    vec, bad = verify(prg, ordered)
    log("  ベクタ " + " ".join(f"{k}=${v:04X}" for k, v in vec.items()))
    if bad:
        for b in bad:
            log("  ★ " + b)
        rig.d.close()
        return False

    write_ines(OUT, prg, b"".join(ordered), mapper=87, mirror="v")
    raw = open(OUT, "rb").read()
    log(f"  完成: {OUT}  {len(raw)} バイト")
    if WEAK[0]:
        log(f"  ※ {WEAK[0]} バイトは票が拮抗し、首位を採って決めた(信頼度やや低)")
    else:
        log("  全バイトが明確な多数決で決まった")
    log(f"  sha1 {hashlib.sha1(raw).hexdigest()}")
    log(f"  再接続 {rig.reconnects} 回")
    rig.d.close()
    return True


def main():
    open(LOG, "w", encoding="utf-8").close()
    log("開始")
    for n in range(1, 200):
        try:
            if attempt(n):
                log("★ 成功")
                return 0
            log("検証に落ちた。20秒置いてやり直す")
            time.sleep(20)
        except Exception:
            log("例外:\n" + traceback.format_exc())
            log("30秒置いてやり直す")
            time.sleep(30)
    log("★ 規定回数で成功せず")
    return 1


if __name__ == "__main__":
    sys.exit(main())
