"""ファミコン汎用ダンプ機のホスト側。

■ 判断はすべてここに置く
装置側(Uno/Nano/328P)はマッパーを知らない。「このCPUアドレスを読め」
「このアドレスにこの値を書け」しか受け付けない。バンク切り替えの手順も、
サイズの判定も、iNESヘッダの組み立ても、全部このファイルの仕事。

SFC-CIC で学んだこと: 装置側に判定させると誤検出が増え、直すたびに
書き込み直しで遅くなる。装置は素直な実行機に徹してもらう。

■ プロトコル
    PC  -> Uno   0x55, CMD, LEN_LO, LEN_HI, payload
    Uno -> PC    0xAA, STATUS, LEN_LO, LEN_HI, payload

■ 使い方
    python fcdump.py --port COM8 selftest
    python fcdump.py --port COM8 read prg 8000 4000
    python fcdump.py --port COM8 dump nrom out.nes
    python fcdump.py --port COM8 walk 3          # テスター用: PRG A3 だけHigh
"""

import argparse
import sys
import time

import serial

BAUD = 115200

REQ_SYNC = 0x55
RSP_SYNC = 0xAA

CMD_PING      = 0x01
CMD_SET_MODE  = 0x02
CMD_PRG_READ  = 0x03
CMD_PRG_WRITE = 0x04
CMD_CHR_READ  = 0x05
CMD_SET_ADDR  = 0x06
CMD_SET_CTRL  = 0x07
CMD_WALK_ADDR = 0x08
CMD_I2C_SCAN  = 0x09
CMD_CHR_ADDR  = 0x0A
CMD_SET_CYCLE = 0x0B

MODE_IDLE, MODE_PRG, MODE_CHR = 0, 1, 2

STATUS = {0x00: "OK", 0x01: "未知のコマンド", 0x02: "スレーブ応答なし",
          0x04: "長さが不正"}

# 1バイトあたり I2C を1〜3往復するので、実測はこのくらい遅い。
# 100kHz のとき PRG で約 0.25ms/byte、CHR で約 0.75ms/byte。
CHUNK = 1024

# 二重読みが一致しないときに諦めるまでの回数。
MAX_RETRY = 24


class DumperError(Exception):
    pass


class Dumper:
    def __init__(self, port, baud=BAUD, verbose=False):
        self.ser = serial.Serial(port, baud, timeout=1.0)
        self.verbose = verbose
        self.retries = 0
        self.mode = MODE_IDLE
        time.sleep(2.0)          # Unoは開いた瞬間にリセットが掛かる
        self.ser.reset_input_buffer()

    def close(self):
        try:
            self.set_mode(MODE_IDLE)
        finally:
            self.ser.close()

    # ------------------------------------------------------------ 下位

    def _read_exact(self, n, timeout):
        """n バイト揃うまで読む。締め切りは全体で見る。"""
        deadline = time.time() + timeout
        buf = bytearray()
        while len(buf) < n:
            chunk = self.ser.read(n - len(buf))
            if chunk:
                buf += chunk
            elif time.time() > deadline:
                raise DumperError(f"応答が {len(buf)}/{n} バイトで途切れた")
        return bytes(buf)

    def _txn(self, cmd, payload=b"", timeout=5.0):
        self.ser.reset_input_buffer()      # 同期が崩れていても次で立て直す
        frame = bytes([REQ_SYNC, cmd, len(payload) & 0xFF, len(payload) >> 8])
        self.ser.write(frame + payload)
        self.ser.flush()

        head = self._read_exact(4, timeout)
        if head[0] != RSP_SYNC:
            raise DumperError(f"同期バイトが違う: {head[0]:#04x}")
        status = head[1]
        if status != 0x00:
            raise DumperError(STATUS.get(status, f"status={status:#04x}"))
        length = head[2] | (head[3] << 8)
        if length == 0:
            return b""
        # 転送そのものが遅いので、長さに応じて締め切りを伸ばす
        return self._read_exact(length, timeout + length * 0.002)

    # ------------------------------------------------------------ 基本操作

    def ping(self):
        r = self._txn(CMD_PING)
        if len(r) != 9 or r[:7] != b"FCDUMP1":
            raise DumperError(f"PINGの応答が変: {r!r}")
        return {"nano": bool(r[7]), "chr328": bool(r[8])}

    def set_mode(self, mode):
        self._txn(CMD_SET_MODE, bytes([mode]))
        self.mode = mode

    def _recover(self):
        """詰まったときに装置を初期状態へ戻す。

        単発の読みは0%で通るのに、連続で読み続けると崩れていく。
        原因は掴めていないが、モードを入れ直すと復帰することが分かったので、
        再試行で埋まらないときはここを通す。休みを入れるのも効く。
        """
        time.sleep(0.3)
        m = getattr(self, "mode", MODE_PRG)
        try:
            self.set_mode(MODE_IDLE)
            time.sleep(0.1)
            self.set_mode(m)
        except DumperError:
            pass
        time.sleep(0.2)

    def set_cycle(self, pulsed):
        """0=静的読み(既定・速い) / 1=M2パルス読み(マッパーがエッジを欲しがるとき)"""
        self._txn(CMD_SET_CYCLE, bytes([1 if pulsed else 0]))

    def _chunk(self, cmd, a, k, timeout):
        return self._txn(cmd, bytes([a & 0xFF, (a >> 8) & 0xFF, k & 0xFF, k >> 8]),
                         timeout=timeout)

    def _read_verified(self, cmd, a, k, timeout, tag):
        """同じ範囲を2回読み、一致するまで繰り返す。

        この装置は長い読み出しで時々化ける。原因を突き止めきれていないが、
        化け方は毎回違うので「2回続けて同じ値が出た」を採用すれば通せる。
        黙って壊れたものを書き出すより、遅くても正しいものを出す。
        """
        prev = self._chunk(cmd, a, k, timeout)
        for attempt in range(MAX_RETRY):
            if attempt >= 2:
                time.sleep(0.05)          # 少し休ませるだけで通ることがある
            if attempt and attempt % 4 == 0:
                self._recover()           # 休んでも駄目ならモードごと入れ直す
                prev = self._chunk(cmd, a, k, timeout)
            cur = self._chunk(cmd, a, k, timeout)
            if cur == prev:
                if attempt:
                    self.retries += attempt
                return cur
            prev = cur
        raise DumperError(f"${a:04X} の{tag}が {MAX_RETRY} 回読んでも安定しない")

    def prg_read(self, addr, n, progress=False, verify=False):
        out = bytearray()
        while len(out) < n:
            k = min(CHUNK, n - len(out))
            a = addr + len(out)
            if verify:
                out += self._read_verified(CMD_PRG_READ, a, k, 10.0, "PRG")
            else:
                out += self._chunk(CMD_PRG_READ, a, k, 10.0)
            if progress:
                _bar("PRG", len(out), n)
        if progress:
            print()
        return bytes(out)

    def prg_write(self, addr, value):
        self._txn(CMD_PRG_WRITE, bytes([addr & 0xFF, (addr >> 8) & 0xFF, value & 0xFF]))

    def chr_read(self, addr, n, progress=False, verify=False):
        out = bytearray()
        while len(out) < n:
            k = min(CHUNK, n - len(out))
            a = (addr + len(out)) & 0x1FFF
            if verify:
                out += self._read_verified(CMD_CHR_READ, a, k, 15.0, "CHR")
            else:
                out += self._chunk(CMD_CHR_READ, a, k, 15.0)
            if progress:
                _bar("CHR", len(out), n)
        if progress:
            print()
        return bytes(out)

    # ------------------------------------------------------------ 診断

    def walk(self, idx):
        """PRGアドレス線を1本だけHighにする。0-14、それ以外で全部Low。
        テスターで当たって配線を確かめるためのもの。"""
        self._txn(CMD_WALK_ADDR, bytes([idx & 0xFF]))

    def set_addr(self, addr):
        self._txn(CMD_SET_ADDR, bytes([addr & 0xFF, (addr >> 8) & 0xFF]))

    def set_chr_addr(self, addr):
        self._txn(CMD_CHR_ADDR, bytes([addr & 0xFF, (addr >> 8) & 0xFF]))


def _bar(tag, done, total):
    pct = done * 100 // total
    sys.stdout.write(f"\r  {tag} {done:6d}/{total} ({pct:3d}%)")
    sys.stdout.flush()


# ---------------------------------------------------------------- 健全性

def selftest(d):
    """配線の一次確認。SFC-CIC の教訓に従い、判定は電圧ではなく機能で行う。

    「読めた気がする」で先へ進むのが一番高くつく。ここが通らないうちは
    マッパー対応もサイズ判定もやらない。
    """
    print("== I2C ==")
    p = d.ping()
    print(f"  Nano  (0x20): {'応答あり' if p['nano'] else '× 応答なし'}")
    print(f"  328P  (0x21): {'応答あり' if p['chr328'] else '× 応答なし'}")
    if not (p["nano"] and p["chr328"]):
        print("  -> I2Cが繋がっていない。プルアップ(3kΩ)とSDA/SCLの入れ違いを見ること")
        return False

    print("== PRG ==")
    d.set_mode(MODE_PRG)
    vec = d.prg_read(0xFFFA, 6)
    nmi = vec[0] | (vec[1] << 8)
    rst = vec[2] | (vec[3] << 8)
    irq = vec[4] | (vec[5] << 8)
    print(f"  ベクタ NMI={nmi:04X} RESET={rst:04X} IRQ={irq:04X}")

    head = d.prg_read(0x8000, 16)
    print(f"  $8000: {head.hex(' ')}")

    if all(b == 0xFF for b in head) or all(b == 0x00 for b in head):
        print("  × 全部同じ値。カートが刺さっていないか、データバスが浮いている")
        return False
    if not (0x8000 <= rst <= 0xFFFF):
        print(f"  × RESETベクタ {rst:04X} が $8000-$FFFF の外。アドレス線かデータ線の配線を疑う")
        return False

    again = d.prg_read(0x8000, 16)
    if again != head:
        print("  × 同じ場所を2回読んで値が違う。接触かノイズ。パスコンを確認")
        return False
    print("  再現性OK (同じ16バイトを2回)")

    print("== CHR ==")
    d.set_mode(MODE_CHR)
    chr0 = d.chr_read(0x0000, 16)
    print(f"  $0000: {chr0.hex(' ')}")
    if all(b == 0xFF for b in chr0):
        print("  ! 全部 FF。CHR-RAM のカートならこれで正常(要書き込みテスト)")
    elif all(b == 0x00 for b in chr0):
        print("  ! 全部 00。CHR ROM が選ばれていない可能性。56番(CHR A13)と17番を確認")

    d.set_mode(MODE_IDLE)
    print("\n健全性ゲート: 通過")
    return True


# ---------------------------------------------------------------- ダンプ

def detect_prg_size(d):
    """16KB品は A14 をデコードしないので $8000 と $C000 に同じものが見える。

    先頭512バイトだけを比べていたら誤判定した。折り返しは一過性の不具合でも
    起こりうるので、16KB全域を突き合わせる。ここを間違えるとヘッダのサイズが狂い、
    エミュレータ側でマッピングが破綻する(緑一色になる)。
    """
    lo = d.prg_read(0x8000, 16 * 1024, verify=True)
    hi = d.prg_read(0xC000, 16 * 1024, verify=True)
    return 16 * 1024 if lo == hi else 32 * 1024


def verify_prg(d, prg, base):
    """吸ったPRGが実機と一致するか、要所を抜き取って照合する。

    一度、1KB周期の折り返しに汚染されたダンプをそのまま書き出してしまった。
    「読めた」と「正しい」は別なので、書き出す前に必ずここを通す。
    """
    problems = []

    # 1. 割り込みベクタ。ここが $8000-$FFFF の外を指していたら確実に壊れている
    v = prg[-6:]
    vectors = {"NMI": v[1] << 8 | v[0], "RESET": v[3] << 8 | v[2], "IRQ": v[5] << 8 | v[4]}
    for name, addr in vectors.items():
        if not 0x8000 <= addr <= 0xFFFF:
            problems.append(f"{name}ベクタ ${addr:04X} が $8000-$FFFF の外")

    # 2. 同じ場所をもう一度読んで一致するか(数か所を抜き取り)
    for off in (0, len(prg) // 3, len(prg) // 2, len(prg) - 64):
        again = d.prg_read(base + off, 64)
        if again != prg[off:off + 64]:
            problems.append(f"${base + off:04X} の再読が不一致")

    # 3. 1KB周期の折り返しが起きていないか
    if len(prg) >= 2048 and prg[:1024] == prg[1024:2048]:
        problems.append("先頭2KBが1KB周期で重複(アドレス線の折り返し)")

    return vectors, problems


def dump_m87(d, path, mirror="v"):
    """Mapper 87 (KONAMI-74*139/74 等)。PRGは固定、CHRだけ8KB単位でバンク切り替え。

    バンク選択は $6000-$7FFF への書き込みで行う($8000以降ではない)。
    しかも D0 が上位ビット・D1 が下位ビットという入れ替わった対応になっているので、
    「書き込む値」と「バンク番号」は別物として扱う。

    バスコンフリクトは起きない。書き込み先の $6000-$7FFF は PRG ROM の窓の外で、
    /ROMSEL が High のままなので ROM は出力しない。
    """
    d.set_mode(MODE_PRG)
    prg_size = detect_prg_size(d)
    print(f"PRG {prg_size // 1024}KB と判定")
    base = 0x10000 - prg_size
    prg = d.prg_read(base, prg_size, progress=True, verify=True)

    vectors, problems = verify_prg(d, prg, base)
    print("  ベクタ " + " ".join(f"{k}=${v:04X}" for k, v in vectors.items()))
    if problems:
        print("\n★ 検証に失敗した。書き出さない:")
        for p in problems:
            print("   -", p)
        raise DumperError("PRGの検証に失敗")
    print("  PRG検証 OK")

    banks = {}
    for val in range(4):
        bank = ((val & 1) << 1) | ((val >> 1) & 1)   # D0<->D1 の入れ替え
        d.set_mode(MODE_PRG)
        d.prg_write(0x6000, val)
        d.set_mode(MODE_CHR)
        banks[bank] = d.chr_read(0, 8 * 1024, progress=True, verify=True)
        print(f"  CHR bank {bank} 取得 (書込値 {val:#04x})")

    chr_data = b"".join(banks[i] for i in sorted(banks))
    d.set_mode(MODE_IDLE)
    write_ines(path, prg, chr_data, mapper=87, mirror=mirror)


def dump_nrom(d, path, mirror="h", chr_size=8 * 1024):
    d.set_mode(MODE_PRG)
    prg_size = detect_prg_size(d)
    print(f"PRG {prg_size // 1024}KB と判定")
    prg = d.prg_read(0x10000 - prg_size, prg_size, progress=True)

    d.set_mode(MODE_CHR)
    chr_data = d.chr_read(0, chr_size, progress=True) if chr_size else b""

    d.set_mode(MODE_IDLE)
    write_ines(path, prg, chr_data, mapper=0, mirror=mirror)


def write_ines(path, prg, chr_data, mapper=0, mirror="h"):
    """iNES(.nes)として書き出す。

    ミラーリングは自動判定できない。カート18番(VRAM A10)がカート側の出力で、
    今の配線では繋いでいないため。--mirror で指定すること。
    """
    flags6 = ((mapper & 0x0F) << 4) | (0x01 if mirror == "v" else 0x00)
    flags7 = mapper & 0xF0
    header = (b"NES\x1a"
              + bytes([len(prg) // 16384, len(chr_data) // 8192, flags6, flags7])
              + b"\x00" * 8)
    with open(path, "wb") as f:
        f.write(header + prg + chr_data)
    print(f"書き出し: {path}  PRG {len(prg)//1024}KB / CHR {len(chr_data)//1024}KB "
          f"/ mapper {mapper} / mirror {mirror}")


# ---------------------------------------------------------------- CLI

def main():
    ap = argparse.ArgumentParser(description="ファミコン汎用ダンプ機")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=BAUD)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("selftest")
    sub.add_parser("ping")

    r = sub.add_parser("read")
    r.add_argument("space", choices=["prg", "chr"])
    r.add_argument("addr")
    r.add_argument("length")
    r.add_argument("--out")

    w = sub.add_parser("write")
    w.add_argument("addr")
    w.add_argument("value")

    dp = sub.add_parser("dump")
    dp.add_argument("mapper", choices=["nrom", "m87"])
    dp.add_argument("out")
    dp.add_argument("--mirror", choices=["h", "v"], default="h")
    dp.add_argument("--chr", type=lambda s: int(s, 0), default=8192)

    wk = sub.add_parser("walk")
    wk.add_argument("index", type=int)

    a = ap.parse_args()
    d = Dumper(a.port, a.baud)
    try:
        if a.cmd == "ping":
            print(d.ping())
        elif a.cmd == "selftest":
            sys.exit(0 if selftest(d) else 1)
        elif a.cmd == "read":
            addr, n = int(a.addr, 16), int(a.length, 16)
            if a.space == "prg":
                d.set_mode(MODE_PRG)
                data = d.prg_read(addr, n, progress=n > CHUNK)
            else:
                d.set_mode(MODE_CHR)
                data = d.chr_read(addr, n, progress=n > CHUNK)
            if a.out:
                open(a.out, "wb").write(data)
                print(f"書き出し: {a.out} ({len(data)} バイト)")
            else:
                _hexdump(data, addr)
        elif a.cmd == "write":
            d.set_mode(MODE_PRG)
            d.prg_write(int(a.addr, 16), int(a.value, 16))
            print("書き込み完了")
        elif a.cmd == "dump":
            if a.mapper == "m87":
                dump_m87(d, a.out, mirror=a.mirror)
            else:
                dump_nrom(d, a.out, mirror=a.mirror, chr_size=a.chr)
        elif a.cmd == "walk":
            d.walk(a.index)
            print(f"PRG A{a.index} だけ High。テスターで当たること"
                  if a.index < 15 else "PRGアドレス線を全部 Low にした")
            return          # 線を保ったまま抜けたいので close しない
    finally:
        if a.cmd != "walk":
            d.close()


def _hexdump(data, base=0):
    for i in range(0, len(data), 16):
        row = data[i:i + 16]
        txt = "".join(chr(b) if 32 <= b < 127 else "." for b in row)
        print(f"{base + i:04X}  {row.hex(' '):<47}  {txt}")


if __name__ == "__main__":
    main()
