// 裸 ATmega328P-PU: CHRアドレス A0-A12 と CHRデータ下位5bit。I2Cスレーブ 0x21。
//
// ■ 水晶の周波数に依存しない
// SFC-CIC 用に組んだ個体(AD21L4 / 実測21.585MHz)をそのまま流用しても動く。
// 理由は二つ。
//   1. TWIのスレーブは SCL をマスターに叩かれて動くので、こちらのクロック精度は
//      関係ない。必要なのは「F_CPU が SCL の16倍以上」だけで、16でも21.5でも満たす
//   2. この firmware は delay()/micros() を一切使わない。ROMのアクセス時間は
//      次の I2C トランザクション(数十µs)が勝手に稼いでくれる
// したがって F_CPU を 16MHz と誤って書き込んでも動作は変わらない。
// UART は使っていない(PD0/PD1 は CHR A0/A1 に取られている)。
//
// ■ CHR の窓は 8KB 固定
// PPU のアドレス空間は $0000-$1FFF がパターンテーブル、$2000-$3FFF が
// ネームテーブル。A13 がその境目で、CHR ROM の窓は A0-A12 の13本しかない。
// 8KBより大きい CHR ROM はマッパーでバンクを切り替える(PC側の仕事)。
// A13(カート56番)は Nano が Low に張っている。
//
// ■ 配線 (DIP28 の物理ピン番号)
//   PD0-PD7 = 2,3,4,5,6,11,12,13   -> CHR A0-A7   (カート 25,24,23,22,21,20,19,50番)
//   PC0-PC3 = 23,24,25,26          -> CHR A8-A11  (カート 51,52,53,54番)
//   PB0     = 14                   -> CHR A12     (カート 55番)
//   PB1-PB5 = 15,16,17,18,19       -> CHR D0-D4   (カート 26,27,28,29,60番)
//   PC4/PC5 = 27,28                -> I2C SDA/SCL
//   PB6/PB7 = 9,10                 -> 水晶
//   PC6     = 1                    -> RESET (10kΩプルアップ)

#include <Arduino.h>
#include <Wire.h>

#define I2C_ADDR 0x21

#define C_SET_ADDR 0x01
#define C_SET_MODE 0x02

static volatile uint8_t g_active = 0;

// ---------------------------------------------------------------- ポート操作

static inline void setChrAddr(uint16_t a) {
  PORTD = (uint8_t)(a & 0xFF);                              // A0-A7
  PORTC = (uint8_t)((PORTC & 0xF0) | ((a >> 8) & 0x0F));    // A8-A11
  if (a & 0x1000) PORTB |= _BV(PB0); else PORTB &= (uint8_t)~_BV(PB0);
}

// CHR D0-D4 は PB1-PB5。呼び出し側が期待する bit0-4 に詰め直す。
static inline uint8_t readChrLow(void) {
  return (uint8_t)((PINB >> 1) & 0x1F);
}

static void setActive(uint8_t on) {
  if (on) {
    DDRB &= (uint8_t)~0x3E;      // データは入力
    PORTB |= 0x3E;               // 浮き止めのプルアップ。ROMが駆動すればROMが勝つ
    DDRD  = 0xFF;                // アドレスは出力
    DDRC |= 0x0F;
    DDRB |= _BV(PB0);
  } else {
    // カートに何も出さない状態。抜き差しはこの状態で。
    DDRD  = 0x00;
    DDRC &= (uint8_t)~0x0F;
    DDRB &= (uint8_t)~0x3F;
    PORTD = 0x00;
    PORTC &= (uint8_t)0xF0;
    PORTB &= (uint8_t)~0x3F;
  }
  g_active = on;
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
    case C_SET_ADDR: setChrAddr((uint16_t)(a | (b << 8)) & 0x1FFF); break;
    case C_SET_MODE: setActive(a ? 1 : 0); break;
    default: break;
  }
}

static void onRequest(void) {
  // アドレスを変えてからここへ来るまでに I2C が1トランザクション挟まるので、
  // ROM のアクセス時間(数百ns)は確実に経過している。待たなくてよい。
  Wire.write(readChrLow());
}

// ---------------------------------------------------------------- 本体

void setup() {
  setActive(0);
  Wire.begin(I2C_ADDR);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
}

void loop() {
  // 何もしない。すべて I2C の割り込みで動く。
}
