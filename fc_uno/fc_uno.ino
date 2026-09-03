// Uno: ファミコン汎用ダンプ機のマスター。
//
// ■ この firmware は何を判断しないか
// マッパーを知らない。iNESヘッダも作らない。バンク切り替えの手順も持たない。
// PCから「このCPUアドレスを読め」「このアドレスにこの値を書け」と言われた
// とおりにバスを叩いて、返ってきたバイトをそのまま返すだけ。
//
// SFC-CIC で学んだこと: 「成立したか」を装置側で判定させると誤検出が増える。
// マッパーごとの手順は PC 側(fcdump.py)に置き、こちらは素直な実行機に徹する。
//
// ■ 役割分担
//   Uno   : PRGアドレス A0-A14 を直接出す。PCとのUSB窓口。I2Cマスター
//   Nano  : PRGデータ D0-D7、制御4線、CHRデータ上位3bit          (I2C 0x20)
//   328P  : CHRアドレス A0-A12、CHRデータ下位5bit                (I2C 0x21)
//
// 1バイト読むのに I2C を1〜3回叩く。速くはないが、PRG 32KB で数秒。
//
// ■ 配線 (Uno)
//   PD2-PD7 = D2-D7   -> PRG A0-A5    (カート 13,12,11,10,9,8番)
//   PB0-PB5 = D8-D13  -> PRG A6-A11   (カート 7,6,5,4,3,2番)
//   PC0-PC2 = A0-A2   -> PRG A12-A14  (カート 33,34,35番)
//   PC4/PC5 = A4/A5   -> I2C SDA/SCL  (3kΩでVCCへプルアップ)
//   PC3     = A3      -> CHR A12      (カート55番)
//   PD0/PD1 = D0/D1   -> USB(PCへ)
//
//   ※ CHR A12 は本来 裸328P の PB0(14番)が担当していたが、
//     故障(High/Lowを命令しても電圧が動かない)が確定したため、ここへ移設した。
//     328P はカートを外して電源だけ入れた単体状態でも High/Low が振れず、
//     配線の導通は生きていたので、328Pチップ内部(出力段)の故障と判断している。
//     Uno はI2Cのマスターで、CHRアドレスが変わる瞬間を一番よく知っている当人なので、
//     ここに移すとNano/328Pのファーム改修もI2Cの追加往復も要らない。
//     328Pの14番ピン・そこへ向かう配線は今後使わない(繋いだままでも実害はない)。
//
//   ※ D13 は PRG A11 に取られているので基板上LEDは使えない。
//      LEDが点かないのは異常ではない。
//
// ■ I2Cモニター
// シリアルモニタ(115200)を直接繋いで 'M' を送ると、fcdump.py を介さない
// 手動診断モードに入る。詳細は下の monitorMode() を参照。'q' で通常の
// バイナリプロトコルへ戻る。

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

// 1.3インチ128x64 OLED (SH1106、I2C 0x3C)。バス走査で「未知」と出ていたのはこれ。
// U8g2はI2Cバスを共有できるので Wire は1つのまま。
// RAMが厳しい(Uno 2KB、PRGアドレスバスのポート直接操作等で既に食っている)ので
// フルフレームバッファ版(_F_)ではなく1ページ分だけ持つ省メモリ版(_1_)を使う。
// 引き換えに描画1回が8ページぶんのループになり遅いが、ダンプ中は毎バイト
// 描画するわけではない(チャンクの先頭でだけ更新)ので問題にならない。
U8G2_SH1106_128X64_NONAME_1_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

static void oledShow(const char *l1, const char *l2 = nullptr, const char *l3 = nullptr) {
  oled.firstPage();
  do {
    oled.setFont(u8g2_font_6x10_tr);
    if (l1) oled.drawStr(0, 11, l1);
    if (l2) oled.drawStr(0, 27, l2);
    if (l3) oled.drawStr(0, 43, l3);
  } while (oled.nextPage());
}

#define NANO_ADDR   0x20
#define CHR328_ADDR 0x21

// --- Nano へのコマンド ---
#define N_SET_MODE   0x01
#define N_SET_CTRL   0x02
#define N_PRG_WRITE  0x03
#define N_SET_CYCLE  0x04

// --- 328P へのコマンド ---
#define C_SET_ADDR   0x01
#define C_SET_MODE   0x02

// --- モード ---
#define MODE_IDLE 0
#define MODE_PRG  1
#define MODE_CHR  2

// --- ホストとのプロトコル ---
// PC -> Uno : 0x55, CMD, LEN_LO, LEN_HI, payload[LEN]
// Uno -> PC : 0xAA, STATUS, LEN_LO, LEN_HI, payload[LEN]
#define REQ_SYNC  0x55
#define RSP_SYNC  0xAA

#define CMD_PING       0x01
#define CMD_SET_MODE   0x02
#define CMD_PRG_READ   0x03
#define CMD_PRG_WRITE  0x04
#define CMD_CHR_READ   0x05
#define CMD_SET_ADDR   0x06   // 診断: PRGアドレスを固定して置く
#define CMD_SET_CTRL   0x07   // 診断: 制御線を生で固定
#define CMD_WALK_ADDR  0x08   // 診断: PRGアドレス線を1本だけHigh
#define CMD_I2C_SCAN   0x09
#define CMD_CHR_ADDR   0x0A   // 診断: CHRアドレスを固定して置く
#define CMD_SET_CYCLE  0x0B   // 0=静的読み(既定) 1=M2パルス読み

#define ST_OK        0x00
#define ST_BAD_CMD   0x01
#define ST_NO_SLAVE  0x02
#define ST_BAD_LEN   0x04

static uint8_t g_mode = MODE_IDLE;

// ---------------------------------------------------------------- ポート操作

// PRG A0-A14。bit15 は /ROMSEL になるのでここでは無視する。
static void setPrgAddr(uint16_t a) {
  PORTD = (uint8_t)((PORTD & 0x03) | ((a & 0x3F) << 2));      // A0-A5
  PORTB = (uint8_t)((PORTB & 0xC0) | ((a >> 6) & 0x3F));      // A6-A11
  PORTC = (uint8_t)((PORTC & 0xF8) | ((a >> 12) & 0x07));     // A12-A14
}

static void prgAddrOutput(bool on) {
  if (on) {
    DDRD |= 0xFC;            // PD0/PD1(USB)には触らない
    DDRB |= 0x3F;            // PB6/PB7は水晶
    DDRC |= 0x0F;            // PC0-PC2=PRG A12-14、PC3=CHR A12。PC4/PC5はI2C
  } else {
    DDRD &= 0x03;
    DDRB &= (uint8_t)~0x3F;
    DDRC &= (uint8_t)~0x0F;
  }
}

// CHR A12 (カート55番)。328PのPB0が壊れたため、こちらで直接駆動する。
// CHR ROMへの入力(コンソール→カート方向)なので、浮かせず常に確定させておく。
static inline void setChrA12(uint8_t hi) {
  if (hi) PORTC |= _BV(PC3); else PORTC &= (uint8_t)~_BV(PC3);
}

// ---------------------------------------------------------------- I2C

static bool i2cCmd(uint8_t addr, const uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(buf, n);
  return Wire.endTransmission() == 0;
}

static bool i2cCmd1(uint8_t addr, uint8_t a) { return i2cCmd(addr, &a, 1); }

static bool i2cCmd2(uint8_t addr, uint8_t a, uint8_t b) {
  const uint8_t t[2] = {a, b};
  return i2cCmd(addr, t, 2);
}

static bool i2cCmd3(uint8_t addr, uint8_t a, uint8_t b, uint8_t c) {
  const uint8_t t[3] = {a, b, c};
  return i2cCmd(addr, t, 3);
}

// スレーブに1バイト吐かせる。居なければ false。
static bool i2cGet(uint8_t addr, uint8_t *out) {
  if (Wire.requestFrom((uint8_t)addr, (uint8_t)1) != 1) return false;
  *out = (uint8_t)Wire.read();
  return true;
}

static bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// ---------------------------------------------------------------- モード

// モードを変えるとカートに向いた全ての線の向きと極性が決まる。
// カートの抜き差しは必ず MODE_IDLE で行うこと。
static bool setMode(uint8_t m) {
  bool ok = true;
  switch (m) {
    case MODE_IDLE:
      prgAddrOutput(false);
      ok &= i2cCmd2(NANO_ADDR,   N_SET_MODE, MODE_IDLE);
      ok &= i2cCmd2(CHR328_ADDR, C_SET_MODE, 0);
      break;
    case MODE_PRG:
      prgAddrOutput(true);
      setPrgAddr(0);
      // CHRアドレス線は「手を引く」のではなく 0 に駆動し続ける。
      // 入力のまま放置するとカート側 CHR ROM のアドレス入力13本が無駆動で浮き、
      // UEW空中配線ではそれがアンテナになる。浮いたCMOS入力は中間電位で
      // 発振して貫通電流を流すので、電源ごと揺れて PRG の読みを汚す。
      // CHR ROM 自体は CHR A13=H と VRAM /RD=H で止めてあるので、
      // アドレスを与えても何も出てこない。
      ok &= i2cCmd2(CHR328_ADDR, C_SET_MODE, 1);
      ok &= i2cCmd3(CHR328_ADDR, C_SET_ADDR, 0x00, 0x00);
      setChrA12(0);
      ok &= i2cCmd2(NANO_ADDR,   N_SET_MODE, MODE_PRG);
      break;
    case MODE_CHR:
      prgAddrOutput(true);
      setPrgAddr(0);
      setChrA12(0);
      ok &= i2cCmd2(NANO_ADDR,   N_SET_MODE, MODE_CHR);
      ok &= i2cCmd2(CHR328_ADDR, C_SET_MODE, 1);
      break;
    default:
      return false;
  }
  g_mode = m;

  char l2[22];
  snprintf(l2, sizeof(l2), "Nano:%s 328P:%s",
           i2cPresent(NANO_ADDR)   ? "OK" : "NG",
           i2cPresent(CHR328_ADDR) ? "OK" : "NG");
  oledShow(m == MODE_IDLE ? "mode: IDLE" : (m == MODE_PRG ? "mode: PRG" : "mode: CHR"), l2);

  return ok;
}

// ---------------------------------------------------------------- ホスト応答

static void sendHeader(uint8_t status, uint16_t len) {
  Serial.write(RSP_SYNC);
  Serial.write(status);
  Serial.write((uint8_t)(len & 0xFF));
  Serial.write((uint8_t)(len >> 8));
}

static void reply(uint8_t status) { sendHeader(status, 0); Serial.flush(); }

static void replyBuf(const uint8_t *b, uint16_t n) {
  sendHeader(ST_OK, n);
  Serial.write(b, n);
  Serial.flush();
}

// ---------------------------------------------------------------- 読み出し

// PRG。/ROMSEL は本来 !(A15 & M2) なので、A15 が変わる境界でだけ Nano に伝える。
static void cmdPrgRead(uint16_t addr, uint16_t n) {
  int8_t romselNow = -1;
  // ここでOLEDを更新しない。1回の描画は約1KBのI2C通信で、それを読み出しと
  // 同じバスへ毎チャンク流すことになる。32KBのダンプなら32回。
  // 実際、長い読みほど壊れるという症状が出た。表示より正しさを取る。
  sendHeader(ST_OK, n);
  for (uint16_t i = 0; i < n; i++) {
    const uint16_t a = (uint16_t)(addr + i);
    const uint8_t romsel = (a & 0x8000) ? 0 : 1;   // 0 = Low(選択)
    if (romsel != romselNow) {
      // bit0 R/W=1 / bit1 M2=1 / bit2 /ROMSEL / bit3 CHR/RD=1 / bit4 CHR A13=1
      i2cCmd2(NANO_ADDR, N_SET_CTRL,
              (uint8_t)(0x01 | 0x02 | (romsel ? 0x04 : 0x00) | 0x08 | 0x10));
      romselNow = (int8_t)romsel;
    }
    setPrgAddr(a);
    uint8_t d;
    if (!i2cGet(NANO_ADDR, &d)) d = 0xFF;   // Nanoが居なければ開放と同じ 0xFF
    Serial.write(d);
  }
  Serial.flush();
}

// CHR。窓は A0-A12 の 8KB 固定。それ以上はマッパーでバンクを切り替える(PC側の仕事)。
static void cmdChrRead(uint16_t addr, uint16_t n) {
  // PRG側と同じ理由でOLEDは触らない。
  sendHeader(ST_OK, n);
  for (uint16_t i = 0; i < n; i++) {
    const uint16_t a = (uint16_t)(addr + i) & 0x1FFF;
    i2cCmd3(CHR328_ADDR, C_SET_ADDR, (uint8_t)(a & 0xFF), (uint8_t)(a >> 8));
    setChrA12((uint8_t)((a >> 12) & 1));
    uint8_t lo, hi;
    if (!i2cGet(CHR328_ADDR, &lo)) lo = 0xFF;   // D0-D3 が bit0-3(328P)
    if (!i2cGet(NANO_ADDR, &hi))   hi = 0xFF;   // D4-D7 が bit4-7(Nano。D4は328PのPB5故障により移設)
    Serial.write((uint8_t)((lo & 0x0F) | (hi & 0xF0)));
  }
  Serial.flush();
}

// ---------------------------------------------------------------- コマンド処理

static bool readExact(uint8_t *dst, uint16_t n, uint32_t ms) {
  const uint32_t deadline = millis() + ms;
  uint16_t got = 0;
  while (got < n) {
    if (Serial.available()) dst[got++] = (uint8_t)Serial.read();
    else if ((int32_t)(millis() - deadline) > 0) return false;
  }
  return true;
}

static void handle(uint8_t cmd, const uint8_t *p, uint16_t len) {
  switch (cmd) {
    case CMD_PING: {
      uint8_t out[9] = {'F', 'C', 'D', 'U', 'M', 'P', '1', 0, 0};
      out[7] = i2cPresent(NANO_ADDR)   ? 1 : 0;
      out[8] = i2cPresent(CHR328_ADDR) ? 1 : 0;
      replyBuf(out, sizeof(out));
      return;
    }
    case CMD_I2C_SCAN: {
      uint8_t out[2];
      out[0] = i2cPresent(NANO_ADDR)   ? 1 : 0;
      out[1] = i2cPresent(CHR328_ADDR) ? 1 : 0;
      replyBuf(out, 2);
      return;
    }
    case CMD_SET_MODE:
      if (len != 1) { reply(ST_BAD_LEN); return; }
      reply(setMode(p[0]) ? ST_OK : ST_NO_SLAVE);
      return;

    case CMD_PRG_READ:
      if (len != 4) { reply(ST_BAD_LEN); return; }
      if (g_mode != MODE_PRG) setMode(MODE_PRG);
      cmdPrgRead((uint16_t)(p[0] | (p[1] << 8)), (uint16_t)(p[2] | (p[3] << 8)));
      return;

    case CMD_CHR_READ:
      if (len != 4) { reply(ST_BAD_LEN); return; }
      if (g_mode != MODE_CHR) setMode(MODE_CHR);
      cmdChrRead((uint16_t)(p[0] | (p[1] << 8)), (uint16_t)(p[2] | (p[3] << 8)));
      return;

    case CMD_PRG_WRITE: {
      // マッパーのレジスタ書き込み。アドレスはこちらが出し、
      // データ線と R/W の操作は Nano がやる。
      if (len != 3) { reply(ST_BAD_LEN); return; }
      if (g_mode != MODE_PRG) setMode(MODE_PRG);
      const uint16_t a = (uint16_t)(p[0] | (p[1] << 8));
      setPrgAddr(a);
      const bool ok = i2cCmd3(NANO_ADDR, N_PRG_WRITE, p[2],
                              (uint8_t)((a & 0x8000) ? 1 : 0));
      reply(ok ? ST_OK : ST_NO_SLAVE);
      return;
    }

    case CMD_SET_ADDR:
      if (len != 2) { reply(ST_BAD_LEN); return; }
      prgAddrOutput(true);
      setPrgAddr((uint16_t)(p[0] | (p[1] << 8)));
      reply(ST_OK);
      return;

    case CMD_CHR_ADDR:
      if (len != 2) { reply(ST_BAD_LEN); return; }
      setChrA12((uint8_t)((p[1] >> 4) & 1));   // p[1]の bit4 = アドレスのbit12
      reply(i2cCmd3(CHR328_ADDR, C_SET_ADDR, p[0], p[1]) ? ST_OK : ST_NO_SLAVE);
      return;

    case CMD_SET_CTRL:
      if (len != 1) { reply(ST_BAD_LEN); return; }
      reply(i2cCmd2(NANO_ADDR, N_SET_CTRL, p[0]) ? ST_OK : ST_NO_SLAVE);
      return;

    case CMD_SET_CYCLE:
      if (len != 1) { reply(ST_BAD_LEN); return; }
      reply(i2cCmd2(NANO_ADDR, N_SET_CYCLE, p[0]) ? ST_OK : ST_NO_SLAVE);
      return;

    case CMD_WALK_ADDR: {
      // テスターで当たるための診断。1本だけHigh、残りはLow。
      // 0xFF を渡すと全部Low。配線を追う前に「線が動くか」を見る。
      if (len != 1) { reply(ST_BAD_LEN); return; }
      prgAddrOutput(true);
      setPrgAddr(p[0] < 15 ? (uint16_t)(1u << p[0]) : 0);
      reply(ST_OK);
      return;
    }

    default:
      reply(ST_BAD_CMD);
      return;
  }
}

// ---------------------------------------------------------------- I2Cモニター
//
// nano3_pins.ino と同じ発想。fcdump.py を挟まず、シリアルモニタ(115200)を
// 直接繋いで手で叩くための1文字コマンド式ツール。バイナリプロトコルとは
// 起動バイトで区別する(0x55='U' は REQ_SYNC、'M' はこちら)ので同居できる。
//
// このUnoは自分がI2Cマスターなので、実際に叩いて返事があるかどうかを
// 見るのが最も確実な「モニター」になる。線に受動的に耳を当てる方式ではない。
//
//   s        バス走査 (0x03-0x77 を全部叩いてACKを見る)
//   p        Nano(0x20) / 328P(0x21) の生死
//   m 0|1|2  モード切替 (idle/prg/chr)
//   a <hex>  PRGアドレスをセットして1バイト読む (要 mode=prg)
//   x <hex>  CHRアドレスをセットして1バイト読む (要 mode=chr)
//   w <0-14> PRGアドレス線を1本だけHigh (テスターで当たる用)
//   q        モニター終了、通常プロトコルへ戻る

static bool readLine(char *buf, uint8_t maxLen, uint32_t ms) {
  const uint32_t deadline = millis() + ms;
  uint8_t n = 0;
  while (true) {
    if (Serial.available()) {
      const char c = (char)Serial.read();
      if (c == '\r') continue;
      if (c == '\n') { buf[n] = 0; return true; }
      if (n + 1 < maxLen) buf[n++] = c;
    } else if ((int32_t)(millis() - deadline) > 0) {
      buf[n] = 0;
      return n > 0;
    }
  }
}

static uint16_t parseHex(const char *s) {
  uint16_t v = 0;
  while (*s) {
    char c = *s++;
    uint8_t d;
    if (c >= '0' && c <= '9') d = (uint8_t)(c - '0');
    else if (c >= 'a' && c <= 'f') d = (uint8_t)(c - 'a' + 10);
    else if (c >= 'A' && c <= 'F') d = (uint8_t)(c - 'A' + 10);
    else continue;
    v = (uint16_t)((v << 4) | d);
  }
  return v;
}

static void monI2cScan(void) {
  Serial.println(F("  addr  応答"));
  uint8_t found = 0;
  char l3[22] = "";
  for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
    if (i2cPresent(addr)) {
      Serial.print(F("  0x")); Serial.print(addr, HEX);
      if (addr == NANO_ADDR)        Serial.println(F("  <- Nano"));
      else if (addr == CHR328_ADDR) Serial.println(F("  <- 328P"));
      else                          Serial.println(F("  <- 未知"));
      if (found < 3) {
        char tmp[8];
        snprintf(tmp, sizeof(tmp), "%02X ", addr);
        strncat(l3, tmp, sizeof(l3) - strlen(l3) - 1);
      }
      found++;
    }
  }
  if (!found) Serial.println(F("  応答なし。プルアップ(3kΩ)とSDA/SCLの配線を確認"));

  char l2[22];
  snprintf(l2, sizeof(l2), "%u devices found", found);
  oledShow("I2C scan", l2, found ? l3 : "check pullup 3kohm");
}

static void monPing(void) {
  const bool nanoOk = i2cPresent(NANO_ADDR);
  const bool c328Ok  = i2cPresent(CHR328_ADDR);
  Serial.print(F("  Nano (0x20): "));
  Serial.println(nanoOk ? F("OK") : F("× 応答なし"));
  Serial.print(F("  328P (0x21): "));
  Serial.println(c328Ok ? F("OK") : F("× 応答なし"));

  char l1[22], l2[22];
  snprintf(l1, sizeof(l1), "Nano(0x20): %s", nanoOk ? "OK" : "NG");
  snprintf(l2, sizeof(l2), "328P(0x21): %s", c328Ok ? "OK" : "NG");
  oledShow(l1, l2);
}

static void monReadPrg(uint16_t addr) {
  if (g_mode != MODE_PRG) {
    Serial.println(F("  ! mode=prg ではない。'm 1' を先に"));
    oledShow("PRG read", "! mode!=PRG", "send: m 1");
    return;
  }
  const uint8_t romsel = (addr & 0x8000) ? 0 : 1;
  i2cCmd2(NANO_ADDR, N_SET_CTRL,
          (uint8_t)(0x01 | 0x02 | (romsel ? 0x04 : 0x00) | 0x08 | 0x10));
  setPrgAddr(addr);
  uint8_t d;
  const bool ok = i2cGet(NANO_ADDR, &d);
  Serial.print(F("  PRG $")); Serial.print(addr, HEX);
  Serial.print(F(" = "));
  char l1[22], l2[22];
  snprintf(l1, sizeof(l1), "PRG $%04X", addr);
  if (ok) {
    Serial.print(F("0x")); Serial.println(d, HEX);
    snprintf(l2, sizeof(l2), "= 0x%02X", d);
  } else {
    Serial.println(F("読めず(Nano応答なし)"));
    snprintf(l2, sizeof(l2), "read failed");
  }
  oledShow(l1, l2);
}

static void monReadChr(uint16_t addr) {
  if (g_mode != MODE_CHR) {
    Serial.println(F("  ! mode=chr ではない。'm 2' を先に"));
    oledShow("CHR read", "! mode!=CHR", "send: m 2");
    return;
  }
  addr &= 0x1FFF;
  const bool okAddr = i2cCmd3(CHR328_ADDR, C_SET_ADDR, (uint8_t)(addr & 0xFF), (uint8_t)(addr >> 8));
  uint8_t lo = 0, hi = 0;
  const bool okLo = i2cGet(CHR328_ADDR, &lo);
  const bool okHi = i2cGet(NANO_ADDR, &hi);
  Serial.print(F("  CHR $")); Serial.print(addr, HEX);
  Serial.print(F(" = "));
  char l1[22], l2[22];
  snprintf(l1, sizeof(l1), "CHR $%04X", addr);
  if (okAddr && okLo && okHi) {
    const uint8_t v = (uint8_t)((lo & 0x0F) | (hi & 0xF0));
    Serial.print(F("0x")); Serial.println(v, HEX);
    snprintf(l2, sizeof(l2), "= 0x%02X", v);
  } else {
    Serial.println(F("読めず(328PかNanoの応答なし)"));
    snprintf(l2, sizeof(l2), "read failed");
  }
  oledShow(l1, l2);
}

static void monitorMode(void) {
  Serial.println(F("== I2Cモニター =="));
  Serial.println(F("s=走査 p=生死 m<0-2>=モード a<hex>=PRG読み x<hex>=CHR読み w<0-14>=アドレス線1本 q=終了"));
  oledShow("I2C Monitor", "s/p/m/a/x/w/q", "waiting for cmd...");
  char line[16];
  while (true) {
    Serial.print(F("> "));
    if (!readLine(line, sizeof(line), 30000)) { Serial.println(); continue; }
    if (!line[0]) continue;

    const char c = line[0];
    char *arg = line + 1;
    while (*arg == ' ') arg++;

    if (c == 'q') { Serial.println(F("通常モードへ戻る")); return; }
    else if (c == 's') monI2cScan();
    else if (c == 'p') monPing();
    else if (c == 'm') {
      const uint8_t m = (uint8_t)parseHex(arg);
      Serial.println(setMode(m) ? F("  OK") : F("  ! スレーブ応答なし"));
    }
    else if (c == 'c') {
      // 328P だけを idle/active させ、その前後でバスが生きているかを見る。
      // active にした瞬間にバスが死ぬなら、328P が出力にしたピンのどれかが
      // SDA か SCL に落ちている。PC3(CHR A11, 26番)と PC4(SDA, 27番)は隣同士なので、
      // そこの短絡が最有力。
      const uint8_t on = (uint8_t)parseHex(arg) ? 1 : 0;
      Serial.print(F("  328P -> "));
      Serial.println(on ? F("active") : F("idle"));
      const bool sent = i2cCmd2(CHR328_ADDR, C_SET_MODE, on);
      Serial.println(sent ? F("  コマンド送信OK") : F("  コマンド送信 失敗"));
      Serial.print(F("  直後のバス: Nano="));
      Serial.print(i2cPresent(NANO_ADDR) ? F("OK") : F("死"));
      Serial.print(F(" 328P="));
      Serial.print(i2cPresent(CHR328_ADDR) ? F("OK") : F("死"));
      Serial.print(F(" OLED="));
      Serial.println(i2cPresent(0x3C) ? F("OK") : F("死"));
    }
    else if (c == 'a') monReadPrg(parseHex(arg));
    else if (c == 'x') monReadChr(parseHex(arg));
    else if (c == 'w') {
      const uint16_t idx = parseHex(arg);
      prgAddrOutput(true);
      setPrgAddr(idx < 15 ? (uint16_t)(1u << idx) : 0);
      Serial.print(F("  PRG A")); Serial.print(idx);
      Serial.println(F(" だけ High (0-14以外で全部Low)"));
      char l1[22];
      if (idx < 15) snprintf(l1, sizeof(l1), "PRG A%u = High", (unsigned)idx);
      else          snprintf(l1, sizeof(l1), "PRG all Low");
      oledShow("walk addr", l1, "check with tester");
    }
    else Serial.println(F("  ? s/p/m/a/x/w/q のどれか"));
  }
}

// ---------------------------------------------------------------- 本体

void setup() {
  // 電源投入直後はカートに何も出さない。差したまま起動されても壊さないため。
  prgAddrOutput(false);
  PORTD &= 0x03;
  PORTB &= 0xC0;
  PORTC &= 0xF8;

  Serial.begin(115200);
  oled.begin();                // Wire.begin() はこの中で呼ばれる
  oledShow("FC Dumper", "init...");
  Wire.setClock(100000);      // 3kΩプルアップ + ジャンパ配線。まず100kHzで確実に
  // 誰かが SDA/SCL を Low に握ったままだと、Wire は解放を永久に待ってハングする。
  // 実際 328P のポートを出力にした瞬間にそれが起きた。黙って固まるのが一番たちが悪いので、
  // 25ms で諦めて TWI をリセットさせ、「応答なし」として上へ返す。
  Wire.setWireTimeout(25000, true);
  delay(50);
  setMode(MODE_IDLE);
}

void loop() {
  uint8_t b;
  if (!readExact(&b, 1, 1000)) return;
  if (b == 'M' || b == 'm') { monitorMode(); return; }
  if (b != REQ_SYNC) return;                    // 同期が崩れたら 0x55 まで捨てる

  uint8_t head[3];
  if (!readExact(head, 3, 500)) return;
  const uint8_t  cmd = head[0];
  const uint16_t len = (uint16_t)(head[1] | (head[2] << 8));

  uint8_t payload[8];
  if (len > sizeof(payload)) { reply(ST_BAD_LEN); return; }
  if (len && !readExact(payload, len, 500)) return;

  handle(cmd, payload, len);
}
