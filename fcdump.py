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
# 以前は24回(数秒)だったが、装置の「不調の波」は数分続くことがある
# (TwinBeeのdump_resilient.pyで実測済み)ため、数分規模の粘りに広げた。
# 一致すれば即抜けるので、既に安定している読みには影響しない。
MAX_RETRY = 300


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
                time.sleep(1.0)           # 少し休ませるだけで通ることがある
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

    def prg_write(self, addr, value, fast=False):
        payload = bytes([addr & 0xFF, (addr >> 8) & 0xFF, value & 0xFF])
        if fast:
            payload += bytes([1])
        self._txn(CMD_PRG_WRITE, payload)

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

    # 4. ビットごとの分布。特定ビットが0%/100%に張り付くのは、
    # 「読めた」の顔をした実際の化け。F1レースでD0が100%固定のまま
    # 検証をすり抜けたことがある(2026-09-04)。
    problems += _sanity_bits(prg, "PRG")

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


def dump_nrom(d, path, mirror="h", chr_size=8 * 1024, chr_votes=5):
    from collections import Counter

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

    chr_data = b""
    if chr_size:
        d.set_mode(MODE_CHR)
        print(f"CHR {chr_size // 1024}KBを{chr_votes}回読んで多数決")
        runs = [d.chr_read(0, chr_size, progress=True) for _ in range(chr_votes)]
        chr_data = bytes(Counter(bs).most_common(1)[0][0] for bs in zip(*runs))
        bad = _sanity_bits(chr_data, "CHR")
        if bad:
            print("\n★ CHRのビット分布が不自然。書き出さない:")
            for b in bad:
                print("   -", b)
            raise DumperError("CHRのビット分布チェックに失敗: " + ", ".join(bad))
        print("  CHR検証 OK")

    d.set_mode(MODE_IDLE)
    write_ines(path, prg, chr_data, mapper=0, mirror=mirror)


# ---------------------------------------------------------------- CNROM (Mapper 3)
#
# PRGは32KB固定でバンク切り替えなし(NROMと同じ)。CHRだけ、$8000-$FFFFの
# どこでもよい1回の書き込みでバンクが切り替わる(MMC1のようなシリアル手順は
# 不要)。PRG ROMにマッパーロジックが無く/CEが/ROMSELに直結のままなので、
# 書き込み中はROMも出力しようとするが、prg_write()が既に/ROMSELを一旦
# 解除してから書く設計になっている(Mapper87やMMC3のときと同じ)ため、
# 追加のバスコンフリクト対策は要らない。
#
# テトリス(BPS/Japan)で確認した実際の構成(Web調査): PRG 32KB(固定)、
# CHR 16KB(8KB×2バンク)、水平ミラーリング。
def dump_cnrom(d, path, mirror="h", chr_banks=2):
    d.set_mode(MODE_PRG)
    prg_size = detect_prg_size(d)
    print(f"PRG {prg_size // 1024}KB と判定(CNROMはバンク切替なし)")
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

    d.set_mode(MODE_CHR)
    chr_data = bytearray()
    for n in range(chr_banks):
        d.set_mode(MODE_PRG)
        d.prg_write(0x8000, n)
        d.set_mode(MODE_CHR)
        chr_data += d.chr_read(0x0000, 8 * 1024, verify=True)
        print(f"  CHRバンク{n} 取得(書込値 {n})")

    bad = _sanity_bits(bytes(prg), "PRG") + _sanity_bits(bytes(chr_data), "CHR")
    if bad:
        print("★ ビット分布が不自然。書き出さない:")
        for b in bad:
            print("   -", b)
        raise DumperError("ビット分布の健全性チェックに失敗: " + ", ".join(bad))
    print("  ビット分布チェック OK")

    d.set_mode(MODE_IDLE)
    write_ines(path, prg, bytes(chr_data), mapper=3, mirror=mirror)


# ---------------------------------------------------------------- UNROM (Mapper 2)
#
# $8000-$FFFFのどこでもよい1回の書き込みでPRGバンク(16KB窓、$8000-$BFFF)を
# 切り替える。$C000-$FFFFは常に最終バンク固定。実機は「今そのアドレスに
# 出ている値と同じ値を書く」ことでバスコンフリクトを避けるが、prg_write()
# は既に/ROMSELを一旦解除してから書く設計(Mapper87/MMC3/CNROMと同じ)
# なので、その制約は気にしなくてよい。
#
# CHRはROMではなくCHR-RAM(コンソール側で書き込み可能なメモリ)なので、
# カートから読み出すデータは無い。.nesにはCHRサイズ0で書き出す。
#
# ドラゴンクエストII(Web調査)で確認した構成: PRG 128KB(16KB×8バンク)、
# CHR-RAM 8KB、水平ミラーリング。
def dump_unrom(d, path, mirror="h", prg_banks=8):
    d.set_mode(MODE_PRG)
    prg = bytearray()
    for n in range(prg_banks):
        d.prg_write(0x8000, n)
        bank = d.prg_read(0x8000, 16 * 1024, verify=True)
        prg += bank
        print(f"  PRGバンク{n} 取得(書込値 {n})")

    # $C000-$FFFFは常に最終バンク固定のはず。取得済みの最終バンクと
    # 一致するか確かめる(生の$C000直読みだが、単発の照合なので許容)。
    fixed = d.prg_read(0xC000, 16 * 1024)
    if fixed != prg[-16 * 1024:]:
        diffs = sum(1 for a, b in zip(fixed, prg[-16 * 1024:]) if a != b)
        print(f"  ★ $C000固定窓が最終バンクと不一致({diffs}バイト)。参考情報として記録するが続行する")

    bad = _sanity_bits(bytes(prg), "PRG")
    if bad:
        print("★ ビット分布が不自然。書き出さない:")
        for b in bad:
            print("   -", b)
        raise DumperError("ビット分布の健全性チェックに失敗: " + ", ".join(bad))
    print("  ビット分布チェック OK")

    d.set_mode(MODE_IDLE)
    write_ines(path, bytes(prg), b"", mapper=2, mirror=mirror)


# ---------------------------------------------------------------- Sunsoft-1 (Mapper 184)
#
# PRGはバンク切り替えなしの32KB固定。CHRだけ4KB単位2枚窓で、レジスタは
# $8000以降ではなく$6000-$7FFF(SRAM領域)にある。/ROMSELはA15依存
# (`!(A15 & M2)`)なのでA15=0のこの範囲では自然に解除されており、
# 他マッパーのようなバスコンフリクト対策は不要。
#
#   7  bit  0
#   .1HH .LLL
#    +++- $0000-0FFF窓の4KバンクL(0-7)
#    +++- $1000-1FFF窓の4KバンクH(4-7のみ)
#
# アトランチスの謎(Web調査)で確認した構成: PRG 32KB(固定)、CHR 16KB
# (4KB×4バンク、物理チップは4バンクしかないのでレジスタ上のL値0-3で
# 全域を読める)、水平ミラーリング。
def dump_sunsoft1(d, path, mirror="h", chr_banks=4):
    d.set_mode(MODE_PRG)
    prg = d.prg_read(0x8000, 32 * 1024, progress=True, verify=True)

    vectors, problems = verify_prg(d, prg, 0x8000)
    print("  ベクタ " + " ".join(f"{k}=${v:04X}" for k, v in vectors.items()))
    if problems:
        print("\n★ 検証に失敗した。書き出さない:")
        for p in problems:
            print("   -", p)
        raise DumperError("PRGの検証に失敗")
    print("  PRG検証 OK")

    chr_data = bytearray()
    for n in range(chr_banks):
        d.set_mode(MODE_PRG)
        d.prg_write(0x6000, 0x40 | (n & 0x07))
        d.set_mode(MODE_CHR)
        chr_data += d.chr_read(0x0000, 4 * 1024, verify=True)
        print(f"  CHRバンク{n} 取得(書込値 {0x40 | (n & 0x07):#04x})")

    bad = _sanity_bits(bytes(prg), "PRG") + _sanity_bits(bytes(chr_data), "CHR")
    if bad:
        print("★ ビット分布が不自然。書き出さない:")
        for b in bad:
            print("   -", b)
        raise DumperError("ビット分布の健全性チェックに失敗: " + ", ".join(bad))
    print("  ビット分布チェック OK")

    d.set_mode(MODE_IDLE)
    write_ines(path, prg, bytes(chr_data), mapper=184, mirror=mirror)


# ---------------------------------------------------------------- MMC1 (Mapper 1)
#
# $8000-$FFFFのどこでもよい同じアドレスへ、bit7=1のリセット書き込み1回 →
# 下位ビットから5回連続で1bitずつ書き込む、というシリアル方式。5回目の
# 書き込みの「アドレス」のbit14-13で、コントロール/CHR0/CHR1/PRGのどの
# 内部レジスタへ確定するかが決まる。ここでは常に同じアドレスへ5回とも
# 書くやり方(定石通り)にしている。
#
# 実機は6502の連続する2サイクル以内の書き込みを無視する癖があるが、
# I2C経由の書き込みはそれよりずっと遅いので気にしなくてよい。
#
# ドクターマリオで確認した実際の構成(Web調査): PRG 32KB(16KB×2バンク)、
# CHR 32KB(8KB×4バンク)、水平ミラーリング。バンク総数が少ないので
# 自動判定はせず、既知の値をそのまま使う。
#
# コントロールレジスタ: bit4=CHRモード(0=8KB) bit3-2=PRGモード
# (2="$C000固定・$8000で切替") bit1-0=ミラーリング(2=水平 3=垂直)
MMC1_REG_CTRL, MMC1_REG_CHR0, MMC1_REG_CHR1, MMC1_REG_PRG = 0x8000, 0xA000, 0xC000, 0xE000


def mmc1_write(d, addr, value):
    # fast=True: /ROMSEL↑とM2↓を1回のポート書き込みで同時に切る特別な
    # 書き込みサイクル。MMC1+SRAM基板($E000/$F000)でこの2遷移が離れると
    # 書き込みが化ける(sanniのcartreader実装のコメントより。2026-09-04、
    # ドラゴンクエストIIIで発見)。
    d.prg_write(addr, 0x80, fast=True)          # シフトレジスタをリセット
    time.sleep(0.002)
    for i in range(5):
        d.prg_write(addr, (value >> i) & 1, fast=True)
        time.sleep(0.002)


def dump_mmc1(d, path, mirror="h", prg_banks=2, chr_banks=4, prg_fixed=False, battery=False):
    d.set_mode(MODE_PRG)
    # PRGモード3=「$C000固定(最終バンク)・$8000で切替」(nesdev仕様で確認済み。
    # 以前は2を使っていたが、2は逆の意味「$8000固定・$C000で切替」だった。
    # このコードは$8000側を読んでいるので3が正しい)
    ctrl = (0 << 4) | (3 << 2) | (2 if mirror == "h" else 3)
    mmc1_write(d, MMC1_REG_CTRL, ctrl)

    problems = []
    if prg_fixed:
        # SEROM基板(ドクターマリオ等): MMC1からのPRGバンク切替線が
        # PRG ROMに配線されていない。PRGは32KB固定、NROM同然に直読みする
        # (sanniのcartreaderのissue#1060で報告されている既知の構成、
        # 2026-09-04確認)。CHRだけMMC1の通常のバンク切替が効く。
        prg = d.prg_read(0x8000, 32 * 1024, verify=True)
        v = prg[-6:]
        vectors = {"NMI": v[1] << 8 | v[0], "RESET": v[3] << 8 | v[2], "IRQ": v[5] << 8 | v[4]}
        for name, addr in vectors.items():
            if not 0x8000 <= addr <= 0xFFFF:
                problems.append(f"{name}ベクタ ${addr:04X} が $8000-$FFFF の外")
    else:
        # SUROM基板(512KB=32バンク、ドラゴンクエストIV等): PRGレジスタが
        # 実質4bitしか配線されておらず、上位256KB/下位256KBの切替は
        # CHR0レジスタのbit4(通称「512Kフラグ」)で行う(sanniのcartreader
        # 実装で確認、2026-09-04)。CHR-RAM機でCHR0を一度も書かないと
        # 上位256KBに届かない。
        surom = prg_banks > 16
        prg = bytearray()
        for n in range(prg_banks):
            if surom:
                mmc1_write(d, MMC1_REG_CHR0, 0x10 if n > 15 else 0x00)
            mmc1_write(d, MMC1_REG_PRG, n)
            prg += d.prg_read(0x8000, 16 * 1024, verify=True)
            _bar("PRG", len(prg), prg_banks * 16 * 1024)
        print()

        # 生の$C000直読みはしない(境界越えで化けることが分かっている、MMC3と同じ理由)。
        # 同じ$8000窓を、同じPRGバンクレジスタ経由で読み直して突き合わせる。
        v = prg[-6:]
        vectors = {"NMI": v[1] << 8 | v[0], "RESET": v[3] << 8 | v[2], "IRQ": v[5] << 8 | v[4]}
        for name, addr in vectors.items():
            if not 0x8000 <= addr <= 0xFFFF:
                problems.append(f"{name}ベクタ ${addr:04X} が $8000-$FFFF の外")
        for bank in range(prg_banks):
            if surom:
                mmc1_write(d, MMC1_REG_CHR0, 0x10 if bank > 15 else 0x00)
            mmc1_write(d, MMC1_REG_PRG, bank)
            again = d.prg_read(0x8000, 64)
            expect = prg[bank * 16384: bank * 16384 + 64]
            if again != expect:
                problems.append(f"バンク{bank}の再読が不一致($8000窓経由)")
    print("  ベクタ " + " ".join(f"{k}=${v:04X}" for k, v in vectors.items()))
    if problems:
        print("\n★ 検証に失敗した。書き出さない:")
        for p in problems:
            print("   -", p)
        raise DumperError("PRGの検証に失敗")
    print("  PRG検証 OK")

    d.set_mode(MODE_CHR)
    chr_data = bytearray()
    for n in range(chr_banks):
        # 8KB CHRモード(bit4=0)ではCHR0レジスタの下位1bitが無視されるので、
        # 8KB単位のバンクを選ぶには2ずつ進める(2026-09-04、ドクターマリオで
        # グラフィックが乱れて発覚。n をそのまま書くとバンク0/1、2/3が
        # それぞれ同じ内容になっていた)。
        mmc1_write(d, MMC1_REG_CHR0, n * 2)
        chr_data += d.chr_read(0x0000, 8 * 1024, verify=True)
        _bar("CHR", len(chr_data), chr_banks * 8 * 1024)
    print()

    bad = _sanity_bits(bytes(prg), "PRG") + _sanity_bits(bytes(chr_data), "CHR")
    if bad:
        print("★ ビット分布が不自然。書き出さない:")
        for b in bad:
            print("   -", b)
        raise DumperError("ビット分布の健全性チェックに失敗: " + ", ".join(bad))
    print("  ビット分布チェック OK")

    d.set_mode(MODE_IDLE)
    write_ines(path, bytes(prg), bytes(chr_data), mapper=1, mirror=mirror, battery=battery)


# ---------------------------------------------------------------- MMC3 (Mapper 4)
#
# レジスタは $8000/$8001 だけで足りる。書き込み自体は prg_write() が
# アドレスのbit15を見て/ROMSELの扱いを自動で切り替えてくれるので
# (cmdPrgWrite: romselActive = (a & 0x8000) ? 1 : 0)、Mapper87のときのように
# 「$6000-$7FFFという窓の外に書く」工夫は要らない。MMC3自体がASICのマッパーで、
# 書き込み中はPRG ROMの出力を自分で止める設計になっているため、実機のゲームが
# 常時これをやっている通り、素直に書けばよい。
#
# $8000(偶数アドレス、どこでもよいが$8000を使う):
#   bit7   CHR A12反転 (0=標準 / 1=反転)
#   bit6   PRGモード   (0: $8000-$9FFFがR6で切替 / 1: $C000-$DFFFがR6で切替)
#   bit2-0 次に$8001へ書く値がどのレジスタ(R0-R7)向けか
# $8001: 選んだレジスタへバンク番号を書く
#
# ダンプに使うのはR6(PRG 8KB窓)とR2(CHR 1KB窓)だけ。R7やR0/R1、
# $A000のミラーリングレジスタはダンプでは触らない(表示用の設定であって
# ROMの中身には関係ない)。IRQ関連レジスタも同様、触る必要が無い。
#
# PRGは $E000-$FFFF が常に最終バンク固定という性質を使い、R6経由で
# 読んだバンクの中身がそれと一致する回を探して総バンク数を自動判定する。
# CHRには同じ「固定窓」が無いので、バンク0とバンクKを比べて一致し始める
# 最小のK(=物理容量を超えて折り返す境目)を総バンク数とみなす。

MMC3_R_CHR_1K_A = 2   # $1000-$13FF (CHR A12反転なし時)
MMC3_R_PRG_LOW  = 6   # $8000-$9FFF (PRGモード0時)


def mmc3_select(d, mode_bits, reg):
    d.prg_write(0x8000, (mode_bits & 0xC0) | (reg & 0x07))


def mmc3_bank(d, bank):
    d.prg_write(0x8001, bank & 0xFF)


def _fuzzy_match(a, b, min_ratio=0.85):
    """$C000系境界を跨ぐ読みは先頭32-58バイト程度が化けることがある
    (ダックハントの調査で確認済み。原因未特定、うちの装置固有)。
    完全一致ではなく、大部分が一致していればよしとする。"""
    if len(a) != len(b):
        return False
    same = sum(1 for x, y in zip(a, b) if x == y)
    return same / len(a) >= min_ratio


def detect_mmc3_prg_banks(d, max_banks=64, sample=512):
    """$E000-$FFFF(常に最終バンク固定)と一致するバンク番号を探す。

    完全一致ではなくファジーマッチ: $E000読み取り自体が境界越えの影響で
    先頭が化けることがあるため、サンプルを大きめに取り大部分の一致で判定する。
    """
    fixed = d.prg_read(0xE000, sample, verify=True)
    for n in range(1, max_banks + 1):
        mmc3_select(d, 0x00, MMC3_R_PRG_LOW)
        mmc3_bank(d, n - 1)
        got = d.prg_read(0x8000, sample)
        if _fuzzy_match(got, fixed):
            return n
    raise DumperError(f"MMC3のPRGバンク数を自動判定できない({max_banks}バンク="
                       f"{max_banks * 8}KBまで試した)")


def detect_mmc3_chr_banks(d, max_banks=256, sample=64):
    """バンク0と一致し始める最小のKを、物理容量からの折り返しとみなす。"""
    mmc3_select(d, 0x00, MMC3_R_CHR_1K_A)
    mmc3_bank(d, 0)
    bank0 = d.chr_read(0x1000, sample, verify=True)
    k = 1
    while k <= max_banks:
        mmc3_select(d, 0x00, MMC3_R_CHR_1K_A)
        mmc3_bank(d, k)
        if d.chr_read(0x1000, sample) == bank0:
            return k
        k *= 2
    raise DumperError(f"MMC3のCHRバンク数を自動判定できない({max_banks}バンク="
                       f"{max_banks}KBまで試した)")


def verify_prg_mmc3(d, prg, prg_banks):
    """MMC3専用の検証。生の$C000/$E000直読みはしない。

    そこは「$Bxxx→$Cxxxのようにアドレス線の多数ビットが同時に反転する境界」で
    読み取りが化けることが分かっている(ダックハントの調査で確認済み)。
    本編のバンク読み取りはR6経由で常に$8000-$9FFFという同じアドレスしか
    使わないので、この境界を一度も踏んでいない。検証もR6経由の再読みで行う。
    """
    problems = []
    v = prg[-6:]
    vectors = {"NMI": v[1] << 8 | v[0], "RESET": v[3] << 8 | v[2], "IRQ": v[5] << 8 | v[4]}
    for name, addr in vectors.items():
        if not 0x8000 <= addr <= 0xFFFF:
            problems.append(f"{name}ベクタ ${addr:04X} が $8000-$FFFF の外")

    # 最終バンクと最終から2番目のバンクを、同じR6経由アドレスで読み直して突き合わせる
    # (R6経由の$8000読みなので$C000系境界は踏んでいない。ここは完全一致でよい)
    for bank in (prg_banks - 1, max(prg_banks - 2, 0)):
        mmc3_select(d, 0x00, MMC3_R_PRG_LOW)
        mmc3_bank(d, bank)
        again = d.prg_read(0x8000, 64)
        expect = prg[bank * 8192: bank * 8192 + 64]
        if again != expect:
            problems.append(f"バンク{bank}の再読が不一致(R6経由、$C000系境界は踏んでいない)")

    if len(prg) >= 2048 and prg[:1024] == prg[1024:2048]:
        problems.append("先頭2KBが1KB周期で重複(アドレス線の折り返し)")

    return vectors, problems


def _sanity_bits(data, label, lo=0.15, hi=0.85):
    """ビットごとの1の割合が健全な範囲(既定15-85%)に収まっているか。
    0%や100%に張り付くのは、その配線・その番地が化けている実測に基づく目安。
    CHR-RAM機(データ長0)はチェック対象がないので素通り。"""
    if not data:
        return []
    bad = []
    for bit in range(8):
        ratio = sum((b >> bit) & 1 for b in data) / len(data)
        if not (lo <= ratio <= hi):
            bad.append(f"{label} D{bit}: {ratio*100:.1f}%")
    return bad


def _bank_health(d, cmd_read, addr, n=6):
    """今選んでいるバンクの16バイトを何度か読み、自分自身と一致する割合を見る。

    MMC3はバンクごとに中身が変わるので、TwinBeeのdump_resilient.pyのような
    既知の正解値(TRUTH16)は使えない。代わりに「毎回同じ値が返るか」で
    健全性を測る。装置には好調な波と不調な波があり、不調な最中は同じ番地でも
    読むたびに違う値が返る(実測)。
    """
    vals = []
    for _ in range(n):
        try:
            vals.append(cmd_read(d, addr, 16))
        except (DumperError, Exception):
            vals.append(None)
    if not vals or vals[0] is None:
        return 0.0
    good = sum(1 for v in vals if v == vals[0])
    return good / n


def _wait_bank_healthy(d, cmd_read, addr, label, need=0.8, max_wait=600, log=print):
    t0 = time.time()
    i = 0
    while time.time() - t0 < max_wait:
        h = _bank_health(d, cmd_read, addr)
        if h >= need:
            if i:
                log(f"    {label}: 好調になった(一致率 {h:.0%})。読み出しを再開")
            return True
        if i % 10 == 0:
            log(f"    {label}: 不調(一致率 {h:.0%})。好調な波を待つ ({i}回目)")
        time.sleep(2)
        i += 1
    return False


def dump_mmc3(d, path, mirror="h", prg_banks=None, chr_banks=None):
    d.set_mode(MODE_PRG)
    if prg_banks is None:
        prg_banks = detect_mmc3_prg_banks(d)
    print(f"PRG {prg_banks}バンク({prg_banks * 8}KB)と判定")

    def _read16(dd, a, n):
        return dd.prg_read(a, n)

    prg = bytearray()
    for n in range(prg_banks):
        mmc3_select(d, 0x00, MMC3_R_PRG_LOW)
        mmc3_bank(d, n)
        if not _wait_bank_healthy(d, _read16, 0x8000, f"PRGバンク{n}"):
            raise DumperError(f"PRGバンク{n}: 好調な波が来ない(最大待機時間超過)")
        prg += d.prg_read(0x8000, 8 * 1024, verify=True)
        _bar("PRG", len(prg), prg_banks * 8 * 1024)
    print()

    vectors, problems = verify_prg_mmc3(d, bytes(prg), prg_banks)
    print("  ベクタ " + " ".join(f"{k}=${v:04X}" for k, v in vectors.items()))
    if problems:
        print("\n★ 検証に失敗した。書き出さない:")
        for p in problems:
            print("   -", p)
        raise DumperError("PRGの検証に失敗")
    print("  PRG検証 OK")

    d.set_mode(MODE_CHR)
    if chr_banks is None:
        chr_banks = detect_mmc3_chr_banks(d)
    print(f"CHR {chr_banks}バンク({chr_banks}KB)と判定")

    def _read16_chr(dd, a, n):
        return dd.chr_read(a, n)

    chr_data = bytearray()
    for n in range(chr_banks):
        mmc3_select(d, 0x00, MMC3_R_CHR_1K_A)
        mmc3_bank(d, n)
        if not _wait_bank_healthy(d, _read16_chr, 0x1000, f"CHRバンク{n}"):
            raise DumperError(f"CHRバンク{n}: 好調な波が来ない(最大待機時間超過)")
        chr_data += d.chr_read(0x1000, 1024, verify=True)
        _bar("CHR", len(chr_data), chr_banks * 1024)
    print()

    # 書き出す前の最終防波堤: ビット分布が不自然なら偽の成功として弾く
    bad = _sanity_bits(bytes(prg), "PRG") + _sanity_bits(bytes(chr_data), "CHR")
    if bad:
        print("★ ビット分布が不自然。書き出さない:")
        for b in bad:
            print("   -", b)
        raise DumperError("ビット分布の健全性チェックに失敗: " + ", ".join(bad))
    print("  ビット分布チェック OK")

    d.set_mode(MODE_IDLE)
    write_ines(path, bytes(prg), bytes(chr_data), mapper=4, mirror=mirror)


def write_ines(path, prg, chr_data, mapper=0, mirror="h", battery=False):
    """iNES(.nes)として書き出す。

    ミラーリングは自動判定できない。カート18番(VRAM A10)がカート側の出力で、
    今の配線では繋いでいないため。--mirror で指定すること。
    battery: バッテリーバックアップSRAM搭載機(ドラクエ等)で立てる。
    立てないとエミュレータがPRG-RAMを用意せず、起動時のセーブデータ
    整合性チェックが失敗してフリーズすることがある(2026-09-04、DQ3で発見)。
    """
    flags6 = ((mapper & 0x0F) << 4) | (0x01 if mirror == "v" else 0x00) | (0x02 if battery else 0x00)
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
    # ゼビウスのように、静的読みでは固定値しか返さず M2 のエッジを要求する基板がある。
    # 症状は「どのアドレスを読んでも同じ値」。probe_prgaddr.py で判別できる。
    ap.add_argument("--pulsed", action="store_true",
                    help="M2をパルスさせて読む(静的読みでアドレスが効かない基板用)")
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
    dp.add_argument("mapper", choices=["nrom", "m87", "cnrom", "mmc1", "mmc3", "sunsoft1", "unrom"])
    dp.add_argument("out")
    dp.add_argument("--mirror", choices=["h", "v"], default="h")
    dp.add_argument("--chr", type=lambda s: int(s, 0), default=8192)
    dp.add_argument("--prg-banks", type=int, default=None,
                     help="MMC3: 8KB単位のPRGバンク総数を手動指定(既定は自動判定)")
    dp.add_argument("--chr-banks", type=int, default=None,
                     help="MMC3: 1KB単位のCHRバンク総数を手動指定(既定は自動判定)")
    dp.add_argument("--prg-fixed", action="store_true",
                     help="MMC1: SEROM基板(ドクターマリオ等)向け。PRGバンク切替線が"
                          "配線されていないので32KB固定で直読みする")
    dp.add_argument("--battery", action="store_true",
                     help="バッテリーバックアップSRAM搭載機(ドラクエ等)でヘッダに立てる")

    wk = sub.add_parser("walk")
    wk.add_argument("index", type=int)

    a = ap.parse_args()
    d = Dumper(a.port, a.baud)
    try:
        if a.pulsed:
            d.set_cycle(1)
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
            elif a.mapper == "cnrom":
                dump_cnrom(d, a.out, mirror=a.mirror,
                           chr_banks=a.chr_banks if a.chr_banks else 2)
            elif a.mapper == "mmc1":
                dump_mmc1(d, a.out, mirror=a.mirror,
                           prg_banks=a.prg_banks if a.prg_banks is not None else 2,
                           chr_banks=a.chr_banks if a.chr_banks is not None else 4,
                           prg_fixed=a.prg_fixed, battery=a.battery)
            elif a.mapper == "mmc3":
                dump_mmc3(d, a.out, mirror=a.mirror,
                           prg_banks=a.prg_banks, chr_banks=a.chr_banks)
            elif a.mapper == "sunsoft1":
                dump_sunsoft1(d, a.out, mirror=a.mirror)
            elif a.mapper == "unrom":
                dump_unrom(d, a.out, mirror=a.mirror,
                           prg_banks=a.prg_banks if a.prg_banks else 8)
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
