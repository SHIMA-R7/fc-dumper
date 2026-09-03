// 裸 ATmega328P-PU: CHRアドレス A0-A11 と CHRデータ下位4bit。I2Cスレーブ 0x21。
//
// ■ このチップは2本のピンが壊れている
// PB0(14番、CHR A12) — High/Lowを命令しても電圧が動かない。
// PB5(19番、CHR D4、SCKと兼用) — 内部プルアップを有効にしても常にLow固定。
//   配線を完全に外して孤立させても、隣のPB4(18番)との接触を疑って清掃しても
//   直らなかった。3枚のチップ全てで再現したため、チップ固有の製造不良という
//   よりは、ISP書き込み時にSCKとして酷使される影響を疑っている(未確証)。
// どちらも「カートを外した単体状態でも症状が出る」ことを確認済みで、
// 配線ではなくチップ内部の故障と判断した。
//
// PB0の代わりは Uno の A3(PC3、カート55番)、
// PB5の代わりは Nano の D1(カート60番)へ、それぞれ移設した。
// 詳細は fc_uno.ino / fc_nano.ino のコメント参照。このファイルでは
// PB0・PB5 のどちらにも一切触らない。
// ★ 配線側の作業として、328Pの14番・19番から出ている線は両方とも外すこと。
//   繋いだままだと、故障したピンと移設先の新しい駆動が同じ線を
//   取り合う形になる。
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
//   PB0     = 14                   -> (未使用。故障によりUno A3へ移設。配線は外す)
//   PB1-PB4 = 15,16,17,18          -> CHR D0-D3   (カート 26,27,28,29番)
//   PB5     = 19                   -> (未使用。故障によりNano D1へ移設。配線は外す)
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
  // A12(旧PB0)は Uno の A3 が直接駆動するので、ここでは何もしない。
}

// CHR D0-D3 は PB1-PB4。呼び出し側が期待する bit0-3 に詰め直す。
// D4(旧PB5)はこのチップの別ピンと同じ壊れ方(常にLow固定)をしたため、
// Nano の D1 へ移設した。ここでは触らない。
static inline uint8_t readChrLow(void) {
  return (uint8_t)((PINB >> 1) & 0x0F);
}

static void setActive(uint8_t on) {
  if (on) {
    DDRB &= (uint8_t)~0x3E;      // データは入力(PB5含む。使わないが害はない)
    PORTB |= 0x3E;               // 浮き止めのプルアップ。ROMが駆動すればROMが勝つ
    DDRD  = 0xFF;                // アドレスは出力
    DDRC |= 0x0F;
    // PB0(旧CHR A12)・PB5(旧CHR D4)はもう使わない。入力のまま放置してよい(配線も外す)。
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
