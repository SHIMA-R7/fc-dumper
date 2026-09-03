// Nano: PRGデータバスと制御線、それに CHR データの上位3bit。I2Cスレーブ 0x20。
//
// ■ この firmware も判断しない
// アドレスは Uno が出す。こちらは「いま出ているアドレスに対して読め/書け」と
// 言われたとおりに制御線を動かし、データバスの値を返すだけ。
//
// ■ 制御線の意味 (カート側の正式名は括弧内)
//   PRG R/W       (14番)  High = 読み。書くときだけ Low
//   PRG M2        (32番 φ2)  CPUクロックの後半。実機は約1.79MHz だが、
//                            マッパーは静的論理なのでゆっくりでよい
//   PRG ROM /CE   (44番 /ROMSEL)  実機では !(A15 & M2)。ここでは自前で作る
//   VRAM /RD      (17番 CHR /RD)  CHR ROM の出力イネーブル
//   CHR A13       (56番)  ★アドレス線ではなく実質チップイネーブル。
//                         PPU空間で A13=0 がパターンテーブル($0000-$1FFF)。
//                         標準基板は CHR ROM の /CE をここに繋いでいる。
//                         CHRを読むあいだ Low に張り続ける。
//
// ■ 読みは静的が既定
// ROM は静的デバイスなので、M2=High / /ROMSEL=Low を保ったままアドレスだけ
// 変えれば読める。1バイトあたり I2C 1往復で済む。マッパーが M2 のエッジを
// 欲しがる場合のために、パルス読み(SET_CYCLE 1)も用意してある。
//
// ■ 配線 (Nano)
//   PD2-PD7 = D2-D7   -> PRG D0-D5    (カート 43,42,41,40,39,38番)
//   PB0-PB1 = D8-D9   -> PRG D6-D7    (カート 37,36番)
//   PB2     = D10     -> PRG R/W      (カート 14番)
//   PB3     = D11     -> PRG φ2       (カート 32番)
//   PB4     = D12     -> PRG ROM /CE  (カート 44番)
//   PB5     = D13     -> VRAM /RD     (カート 17番)
//   PC0-PC2 = A0-A2   -> CHR D5-D7    (カート 59,58,57番)
//   PC3     = A3      -> CHR A13      (カート 56番)
//   PC4/PC5 = A4/A5   -> I2C
//   PD0     = D0      -> 空き(USBシリアルのRX。ここは使わない、下記参照)
//   PD1     = D1      -> CHR D4       (カート 60番。328PのPB5が壊れたため移設)
//
//   ※ D13 は VRAM /RD に取られている。基板上LEDはこの線と一緒に光るだけで、
//      状態表示には使えない。常に出力なので線を乱すことはない。
//
//   ※ CHR D4 は本来 裸328P の PB5(19番/SCK兼用)が担当していたが、
//      3枚のチップ全てで同じ壊れ方(内部プルアップを有効にしても常にLow固定、
//      配線を完全に外して孤立させても直らない)が確定したため、Nano へ移設した。
//      D0(RX)ではなく D1(TX)を選んだ理由:
//      USBシリアル変換チップ(CH340等)からD0(RX)は常に駆動されている
//      (UARTのアイドル時もHighを能動的に押している)ので、そこにカートの
//      信号を重ねるとバス競合になり、別の固定化けを生むだけ。
//      D1(TX)側はCH340から見て受信専用の入力で、こちらのUART送信回路も
//      Serial.begin()を呼ばない限り眠ったままなので、駆動が衝突しない。

#include <Arduino.h>
#include <Wire.h>

#define I2C_ADDR 0x20

#define N_SET_MODE   0x01
#define N_SET_CTRL   0x02
#define N_PRG_WRITE  0x03
#define N_SET_CYCLE  0x04

#define MODE_IDLE 0
#define MODE_PRG  1
#define MODE_CHR  2

// 制御線のビット。1 = High。/ROMSEL と /RD は負論理なので 1 が「解除」。
#define CTL_RW     0x01
#define CTL_M2     0x02
#define CTL_ROMSEL 0x04
#define CTL_CHRRD  0x08
#define CTL_A13    0x10

#define CTL_ALL_OFF (CTL_RW | CTL_M2 | CTL_ROMSEL | CTL_CHRRD | CTL_A13)
// PRG読み: R/W=H, M2=H, /ROMSEL=L, /RD=H, A13=H(CHR ROMは止める)
#define CTL_PRG_READ (CTL_RW | CTL_M2 | CTL_CHRRD | CTL_A13)
// CHR読み: R/W=H, M2=L, /ROMSEL=H, /RD=L, A13=L(CHR ROMを選ぶ)
#define CTL_CHR_READ (CTL_RW | CTL_ROMSEL)

static volatile uint8_t g_mode  = MODE_IDLE;
static volatile uint8_t g_ctrl  = CTL_ALL_OFF;
static volatile uint8_t g_pulse = 0;      // 1 = M2パルス読み

// ---------------------------------------------------------------- ポート操作

static inline void applyCtrl(uint8_t c) {
  PORTB = (uint8_t)((PORTB & ~0x3C) | ((c & 0x0F) << 2));
  if (c & CTL_A13) PORTC |= _BV(PC3); else PORTC &= (uint8_t)~_BV(PC3);
}

static inline uint8_t readPrgData(void) {
  return (uint8_t)(((PIND >> 2) & 0x3F) | ((PINB & 0x03) << 6));
}

static inline void writePrgData(uint8_t d) {
  PORTD = (uint8_t)((PORTD & 0x03) | (uint8_t)(d << 2));
  PORTB = (uint8_t)((PORTB & 0xFC) | (uint8_t)(d >> 6));
}

// 入力に戻すときは内部プルアップを入れておく。
// ROM が駆動している場面では ROM 側が勝つので読み値は変わらない。
// 誰も駆動していない場面(CHRモード中のPRGデータ線など)で線が浮くのを防ぐのが目的で、
// UEW空中配線ではこの「浮き」がそのままアンテナと貫通電流の原因になる。
static void prgDataDir(bool out) {
  if (out) {
    DDRD |= 0xFC; DDRB |= 0x03;
  } else {
    DDRD &= 0x03; DDRB &= (uint8_t)~0x03;
    PORTD |= 0xFC; PORTB |= 0x03;
  }
}

// CHR D5-D7 は PC0-PC2、CHR D4 は PD1(328PのPB5故障により移設)。
// 返す位置(bit4-7)に合わせて詰める。
static inline uint8_t readChrHigh(void) {
  return (uint8_t)(((PINC & 0x07) << 5) | ((PIND & 0x02) << 3));
}

static void chrDataDir(bool out) {
  if (out) {
    DDRC |= 0x07;
  } else {
    DDRC &= (uint8_t)~0x07;
    PORTC |= 0x07;          // 同上。CHR ROM が止まっている間の浮き止め
  }
}

// ---------------------------------------------------------------- モード

static void setMode(uint8_t m) {
  switch (m) {
    case MODE_PRG:
      prgDataDir(false);
      chrDataDir(false);
      DDRB |= 0x3C;              // 制御4線は出力
      DDRC |= _BV(PC3);          // CHR A13 も出力
      g_ctrl = CTL_PRG_READ;
      break;
    case MODE_CHR:
      prgDataDir(false);
      chrDataDir(false);
      DDRB |= 0x3C;
      DDRC |= _BV(PC3);
      g_ctrl = CTL_CHR_READ;
      break;
    default:                     // MODE_IDLE
      prgDataDir(false);
      chrDataDir(false);
      DDRB |= 0x3C;
      DDRC |= _BV(PC3);
      g_ctrl = CTL_ALL_OFF;      // 全て解除。カートの抜き差しはこの状態で
      break;
  }
  g_mode = m;
  applyCtrl(g_ctrl);
}

// ---------------------------------------------------------------- バスサイクル

// マッパーのレジスタ書き込み。アドレスは Uno が既に出している。
// romselActive = 1 なら $8000-$FFFF なので /ROMSEL を落とす。
//
// 実機では /ROMSEL = !(A15 & M2) なので、/ROMSEL の立ち上がりと M2 の
// 立ち下がりは同時に起きる。多くのマッパーはその立ち上がりで値を確定する。
// データ線はそのエッジを跨いで有効に保つ。
static void prgWriteCycle(uint8_t d, uint8_t romselActive) {
  const uint8_t base = CTL_CHRRD | CTL_A13;      // CHR側は止めたまま

  applyCtrl(base | CTL_RW | CTL_ROMSEL);          // まず解除。ROMの出力を止める
  prgDataDir(true);
  writePrgData(d);
  applyCtrl(base | CTL_ROMSEL);                   // R/W を Low へ
  delayMicroseconds(1);

  applyCtrl(base | CTL_M2 | (romselActive ? 0 : CTL_ROMSEL));   // M2↑ /ROMSEL↓
  delayMicroseconds(2);
  applyCtrl(base | CTL_M2 | CTL_ROMSEL);          // /ROMSEL↑  ← ここで確定
  applyCtrl(base | CTL_ROMSEL);                   // M2↓
  delayMicroseconds(1);

  prgDataDir(false);
  applyCtrl(g_ctrl);                              // 読みの状態へ戻す
}

static uint8_t prgReadCycle(void) {
  if (!g_pulse) return readPrgData();             // 静的: もう出力は出ている

  const uint8_t base = CTL_RW | CTL_CHRRD | CTL_A13;
  applyCtrl(base | CTL_ROMSEL);                   // M2↓ /ROMSEL↑
  delayMicroseconds(1);
  applyCtrl(base | CTL_M2);                       // M2↑ /ROMSEL↓
  delayMicroseconds(2);                           // ROMのアクセス時間
  const uint8_t d = readPrgData();
  applyCtrl(g_ctrl);
  return d;
}

// ---------------------------------------------------------------- I2C

static void onReceive(int n) {
  if (n < 1) return;
  const uint8_t cmd = (uint8_t)Wire.read();
  uint8_t a = 0, b = 0;
  if (n >= 2) a = (uint8_t)Wire.read();
  if (n >= 3) b = (uint8_t)Wire.read();
  while (Wire.available()) Wire.read();

  switch (cmd) {
    case N_SET_MODE:  setMode(a); break;
    case N_SET_CTRL:  g_ctrl = a; applyCtrl(g_ctrl); break;
    case N_PRG_WRITE: prgWriteCycle(a, b); break;
    case N_SET_CYCLE: g_pulse = a ? 1 : 0; break;
    default: break;
  }
}

// 1バイト返す。CHRモードなら上位3bit、それ以外は PRG データ。
// クロックストレッチが効くので、ここでバスサイクルを回してよい。
static void onRequest(void) {
  Wire.write(g_mode == MODE_CHR ? readChrHigh() : prgReadCycle());
}

// ---------------------------------------------------------------- 本体

void setup() {
  // 電源投入直後はカートに何も出さない。差したまま起動されても壊さないため。
  DDRD &= 0x03;
  DDRB &= (uint8_t)~0x3F;
  DDRC &= (uint8_t)~0x0F;

  // CHR D4(PD1)は常に入力・常にプルアップでよい。他のデータ線と違って
  // 出力に切り替える場面が無いので、モードごとに触らずここで固定する。
  PORTD |= 0x02;

  setMode(MODE_IDLE);

  Wire.begin(I2C_ADDR);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
}

void loop() {
  // 何もしない。すべて I2C の割り込みで動く。
}
