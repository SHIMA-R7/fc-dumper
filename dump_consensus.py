"""ダンプ全体を何度も取り、バイトごとの多数決で最終版を作る。

■ なぜ必要か
1回のダンプの中で何回投票しても、CHR の誤りが 240 → 295 と減らなかった。
投票を 7回から 21回に増やしても改善しない。つまり誤りはランダムではなく、
「その瞬間は誤った値が安定して多数派」という形で出る。

ところが**別の時刻に読み直すと満票で正しい値が返る**(実測: 21回中21回)。
装置の調子が時間で変動していて、一度の読み出しセッションの中だけでは
その偏りから抜け出せない。

だから投票の単位を「1バイトの読み出し」ではなく「ダンプ1回ぶん」に上げる。
時刻を変えて何度も吸い、バイトごとに多数決を取る。
偏りは時刻ごとに変わるので、回を重ねれば正しい値が勝つ。

■ 使い方
    python dump_consensus.py [回数]      既定 5回

各回のあいだに間隔を置き、装置の状態が変わるのを待つ。
最後に、全回一致したバイトと、多数決で決めたバイトの数を申告する。
"""

import hashlib
import subprocess
import sys
import time
from collections import Counter

BASE = r"C:\FC-Dumper"
OUT = BASE + r"\twinbee.nes"
FINAL = BASE + r"\twinbee_final.nes"
GAP = 20          # 各回のあいだに置く秒数。装置の状態が変わるのを待つ


def log(m):
    print(f"[{time.strftime('%H:%M:%S')}] {m}", flush=True)


def one_pass(n):
    """dump_resilient.py を1回走らせ、出来たファイルを返す。"""
    log(f"--- {n}回目のダンプ開始 ---")
    r = subprocess.run([sys.executable, BASE + r"\dump_resilient.py"],
                       capture_output=True, text=True, encoding="utf-8", errors="replace")
    if r.returncode != 0:
        log(f"  失敗 (returncode {r.returncode})")
        return None
    data = open(OUT, "rb").read()
    log(f"  完了 sha1 {hashlib.sha1(data).hexdigest()[:16]}")
    return data


def main():
    rounds = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    dumps = []
    for n in range(1, rounds + 1):
        d = one_pass(n)
        if d:
            dumps.append(d)
        if n < rounds:
            log(f"  {GAP}秒あけて装置の状態が変わるのを待つ")
            time.sleep(GAP)

    if len(dumps) < 3:
        log("★ 成功したダンプが3回に満たない。中止")
        return 1

    sizes = {len(d) for d in dumps}
    if len(sizes) != 1:
        log(f"★ サイズが揃わない {sizes}。中止")
        return 1

    size = sizes.pop()
    out = bytearray(size)
    unanimous = split = 0
    for i in range(size):
        c = Counter(d[i] for d in dumps)
        top, n = c.most_common(1)[0]
        out[i] = top
        if n == len(dumps):
            unanimous += 1
        else:
            split += 1

    open(FINAL, "wb").write(bytes(out))
    log("")
    log(f"ダンプ {len(dumps)} 回の多数決")
    log(f"  全回一致      : {unanimous}/{size} ({100*unanimous/size:.3f}%)")
    log(f"  割れて多数決  : {split}")
    log(f"  書き出し      : {FINAL}")
    log(f"  sha1          : {hashlib.sha1(bytes(out)).hexdigest()}")
    for i, d in enumerate(dumps, 1):
        diff = sum(1 for j in range(size) if d[j] != out[j])
        log(f"  {i}回目と最終版の相違: {diff}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
