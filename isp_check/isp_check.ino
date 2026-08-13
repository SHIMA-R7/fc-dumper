// PRGアドレス線15本：駆動して読み返す(競合の直接検出)。
//
// 前の版は「開放した線が自然に戻るか」を見ていたが、あれは筋が悪かった。
// どこにも繋がっていない線は電荷の漏れで勝手に漂うので、数値がばらついて
// 「何かが引いている」ように見えてしまう。実際 A0 も中途半端な値を出した。
//
// AVR は出力中でも PINx で**実際のピン電圧**を読める。
// だから「Highを出しているのに Low が読める」なら、誰かが押し負かしている
// ということが確実に分かる。漂いに惑わされない。
//
// カートの入力はハイインピーダンスなので、正常なら読み返しは必ず一致する。

#include <Arduino.h>

struct Line { volatile uint8_t *ddr, *port, *pin; uint8_t bit; const char *name; };

static const Line LINES[15] = {
  {&DDRD, &PORTD, &PIND, PD2, "A0  D2 (cart13)"},
  {&DDRD, &PORTD, &PIND, PD3, "A1  D3 (cart12)"},
  {&DDRD, &PORTD, &PIND, PD4, "A2  D4 (cart11)"},
  {&DDRD, &PORTD, &PIND, PD5, "A3  D5 (cart10)"},
  {&DDRD, &PORTD, &PIND, PD6, "A4  D6 (cart 9)"},
  {&DDRD, &PORTD, &PIND, PD7, "A5  D7 (cart 8)"},
  {&DDRB, &PORTB, &PINB, PB0, "A6  D8 (cart 7)"},
  {&DDRB, &PORTB, &PINB, PB1, "A7  D9 (cart 6)"},
  {&DDRB, &PORTB, &PINB, PB2, "A8  D10(cart 5)"},
  {&DDRB, &PORTB, &PINB, PB3, "A9  D11(cart 4)"},
  {&DDRB, &PORTB, &PINB, PB4, "A10 D12(cart 3)"},
  {&DDRB, &PORTB, &PINB, PB5, "A11 D13(cart 2)"},
  {&DDRC, &PORTC, &PINC, PC0, "A12 A0 (cart33)"},
  {&DDRC, &PORTC, &PINC, PC1, "A13 A1 (cart34)"},
  {&DDRC, &PORTC, &PINC, PC2, "A14 A2 (cart35)"},
};

// その線だけを目的の値にし、他14本は逆の値で駆動する。
// 隣と短絡していれば、多数派に引きずられて読み返しが食い違う。
static uint16_t driveAndCheck(uint8_t idx, bool high) {
  for (uint8_t i = 0; i < 15; i++) {
    const uint8_t m = (uint8_t)_BV(LINES[i].bit);
    *LINES[i].ddr |= m;
    const bool v = (i == idx) ? high : !high;
    if (v) *LINES[i].port |= m; else *LINES[i].port &= (uint8_t)~m;
  }
  delayMicroseconds(50);

  const uint8_t m = (uint8_t)_BV(LINES[idx].bit);
  uint16_t bad = 0;
  for (uint16_t k = 0; k < 500; k++) {
    const bool got = (*LINES[idx].pin & m) != 0;
    if (got != high) bad++;
  }
  return bad;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  delay(300);

  Serial.println(F("== 駆動して読み返す (500回中の食い違い数) =="));
  Serial.println(F("その線だけ目的値、他14本は逆値。0なら競合なし"));
  Serial.println(F("信号線            High出力   Low出力    判定"));

  for (uint8_t i = 0; i < 15; i++) {
    const uint16_t bh = driveAndCheck(i, true);
    const uint16_t bl = driveAndCheck(i, false);
    Serial.print(F("  "));
    Serial.print(LINES[i].name);
    Serial.print(F("   "));
    Serial.print(bh); Serial.print(F("\t     "));
    Serial.print(bl); Serial.print(F("\t    "));
    if (bh && bl)      Serial.println(F("★ 両方向で負けている(短絡)"));
    else if (bh)       Serial.println(F("★ Highを出せない(GND側へ短絡)"));
    else if (bl)       Serial.println(F("★ Lowを出せない(VCC側へ短絡)"));
    else               Serial.println(F("正常"));
  }

  for (uint8_t i = 0; i < 15; i++) {
    *LINES[i].ddr &= (uint8_t)~_BV(LINES[i].bit);
    *LINES[i].port &= (uint8_t)~_BV(LINES[i].bit);
  }
  Serial.println(F("完了"));
}

void loop() {}
