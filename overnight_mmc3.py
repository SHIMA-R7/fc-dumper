# -*- coding: utf-8 -*-
"""マリオ3(MMC3)を、失敗するたびDP100で電源サイクルしながら朝まで試す。

安全対策:
  - dp100.pyのVMAX_MV=5000ガードを経由する。ここでは電圧を一切指定しない
  - 例外(USB切断含む)で電源が入ったまま止まることがないよう、必ずfinallyで切る
  - 1回の試行に長いタイムアウトを設け、ハングしたまま無限に待たない

ログは overnight_log.txt に逐次追記。成功したら SUCCESS_MARKER ファイルを作る。
"""
import os
import subprocess
import sys
import time
import traceback

import serial.tools.list_ports as lp

sys.path.insert(0, r"C:\FC-Dumper")
from dp100 import DP100, power_on, power_off

PORT = "COM22"
OUT = r"C:\FC-Dumper\mario3.nes"
LOG = r"C:\FC-Dumper\overnight_log.txt"
SUCCESS_MARKER = r"C:\FC-Dumper\overnight_SUCCESS.txt"
MAX_ATTEMPTS = 40
GAP_AFTER_POWERON = 3.0   # ポートが出てからブートが終わるまでの余裕
# dump_mmc3内部に「好調な波を待つ」仕組みを追加した(バンクごと最大600秒待機)。
# 外側のタイムアウトがそれより短いと、待っている最中に問答無用で電源サイクル
# されてしまい意味が無いので、内部の待ちより十分長く取る。
ATTEMPT_TIMEOUT = 1800    # 1回の試行に許す時間(秒)

RETROARCH = r"C:\RetroArch\retroarch.exe"
MESEN_CORE = r"C:\RetroArch\cores\mesen_libretro.dll"
SCREENSHOT = r"C:\FC-Dumper\mario3_screenshot.png"
SCREENSHOT_FRAMES = 300   # 約5秒(60fps)。タイトル画面までは進むはず


def take_screenshot():
    """成功時にRetroArch(Mesenコア)で実際に起動し、見た目を確認する。

    内部チェック(ベクタ・ビット分布)を通っても実際は壊れているケースが
    あったため(2026-09-04未明)、最終確認として実機相当のエミュレータで
    見た目を見る。"""
    if os.path.exists(SCREENSHOT):
        os.remove(SCREENSHOT)
    try:
        r = subprocess.run(
            [RETROARCH, "-L", MESEN_CORE, OUT,
             f"--max-frames={SCREENSHOT_FRAMES}", "--max-frames-ss",
             f"--max-frames-ss-path={SCREENSHOT}"],
            capture_output=True, text=True, timeout=60,
        )
        log(f"  RetroArch終了コード {r.returncode}")
    except subprocess.TimeoutExpired:
        log("  RetroArchがタイムアウト(60秒)。強制終了扱い")
        return False
    except FileNotFoundError:
        log(f"  RetroArchが見つからない: {RETROARCH}")
        return False
    ok = os.path.exists(SCREENSHOT)
    log(f"  スクリーンショット: {'保存できた ' + SCREENSHOT if ok else '保存されなかった'}")
    return ok


def log(msg):
    line = f"[{time.strftime('%H:%M:%S')}] {msg}"
    with open(LOG, "a", encoding="utf-8") as f:
        f.write(line + "\n")
    print(line, flush=True)


def wait_port(timeout=30):
    dl = time.time() + timeout
    while time.time() < dl:
        if PORT in [q.device for q in lp.comports()]:
            return True
        time.sleep(0.4)
    return False


def power_cycle(p):
    log("  電源サイクル: OFF")
    power_off(p, log=lambda m: None)
    time.sleep(2.0)
    log("  電源サイクル: ON")
    power_on(p, log=lambda m: None)
    if not wait_port():
        raise RuntimeError(f"{PORT} が電源投入後に現れない")
    time.sleep(GAP_AFTER_POWERON)
    log(f"  {PORT} 復帰確認、ブート待ち{GAP_AFTER_POWERON}秒完了")


def one_attempt(n):
    log(f"===== 試行 {n} =====")
    try:
        r = subprocess.run(
            [sys.executable, r"C:\FC-Dumper\fcdump.py", "--port", PORT,
             "dump", "mmc3", OUT, "--mirror", "h",
             "--prg-banks", "32", "--chr-banks", "128"],
            capture_output=True, text=True, encoding="utf-8", errors="replace",
            timeout=ATTEMPT_TIMEOUT,
        )
    except subprocess.TimeoutExpired:
        log(f"  タイムアウト({ATTEMPT_TIMEOUT}秒)。ハングとみなす")
        return False

    tail = "\n".join(r.stdout.strip().splitlines()[-6:])
    log(f"  終了コード {r.returncode}")
    log("  出力末尾:\n    " + tail.replace("\n", "\n    "))
    if r.returncode != 0:
        err_tail = "\n".join(r.stderr.strip().splitlines()[-4:])
        if err_tail:
            log("  stderr末尾:\n    " + err_tail.replace("\n", "\n    "))
    return r.returncode == 0


def main():
    open(LOG, "a", encoding="utf-8").write(
        f"\n[{time.strftime('%H:%M:%S')}] ===== 一晩実験 開始 =====\n")
    log(f"最大{MAX_ATTEMPTS}回試行。1回あたり最大{ATTEMPT_TIMEOUT}秒")

    with DP100() as p:
        info = p.device_info()
        log(f"DP100: {info}")
        try:
            for n in range(1, MAX_ATTEMPTS + 1):
                if one_attempt(n):
                    log("★★★ ダンプ成功。RetroArchで見た目を確認する ★★★")
                    shot_ok = take_screenshot()
                    open(SUCCESS_MARKER, "w", encoding="utf-8").write(
                        f"attempt {n} で成功。{OUT} を確認すること。\n"
                        f"スクリーンショット: {'OK ' + SCREENSHOT if shot_ok else '失敗(手動で起動して確認すること)'}\n"
                        f"{time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                    return 0
                log("  失敗。電源サイクルして20秒待ってからやり直す")
                try:
                    power_cycle(p)
                except Exception:
                    log("  電源サイクルで例外:\n" + traceback.format_exc())
                    log("  60秒待って続行を試みる")
                    time.sleep(60)
                time.sleep(20)
            log(f"★ {MAX_ATTEMPTS}回で成功せず")
            return 1
        finally:
            log("終了処理: 電源を切る")
            try:
                power_off(p, log=log)
            except Exception:
                log("電源OFFで例外(念のため記録):\n" + traceback.format_exc())


if __name__ == "__main__":
    sys.exit(main())
