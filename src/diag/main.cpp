/*
  ADS1220 diagnostic for the TAS501 / Nano ESP32 rocket motor DAQ
  ----------------------------------------------------
  Uses the ADS1220's INTERNAL 2.048 V reference at gain 1 (PGA bypassed),
  so it works even if REFP0/REFN0 are not connected. It measures each
  node of the load cell circuit and prints the voltages.

  Serial Monitor: 115200 baud. Results repeat every 2 seconds.

  Expected with correct wiring (3.3 V supply, no load):
    AVDD supply          ~3.3 V
    AIN1 (green, S+)     ~1.65 V
    AIN2 (white, S-)     ~1.65 V
    AIN1 - AIN2          a few mV (changes when you press on the cell)
    Shorted inputs       ~0 V
*/

#include <Arduino.h>
#include <SPI.h>

const int PIN_CS   = D10;
const int PIN_DRDY = D2;

const uint8_t CMD_RESET = 0x06;
const uint8_t CMD_START = 0x08;
const uint8_t CMD_RDATA = 0x10;
const uint8_t CMD_WREG  = 0x40;

const float VREF_INT = 2.048;

SPISettings adsSPI(1000000, MSBFIRST, SPI_MODE1);

void adsCommand(uint8_t c) {
  SPI.beginTransaction(adsSPI); digitalWrite(PIN_CS, LOW);
  SPI.transfer(c);
  digitalWrite(PIN_CS, HIGH); SPI.endTransaction();
}

void adsWriteReg(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(adsSPI); digitalWrite(PIN_CS, LOW);
  SPI.transfer(CMD_WREG | (reg << 2));     // write 1 register
  SPI.transfer(val);
  digitalWrite(PIN_CS, HIGH); SPI.endTransaction();
}

bool readOnce(uint8_t mux, float &volts) {
  // REG0: MUX, gain 1 (000), PGA bypassed (1)
  adsWriteReg(0, (mux << 4) | 0x01);
  adsCommand(CMD_START);                   // single-shot conversion
  uint32_t t = millis();
  while (digitalRead(PIN_DRDY) == HIGH) {
    if (millis() - t > 500) return false;
    delay(1);
  }
  SPI.beginTransaction(adsSPI); digitalWrite(PIN_CS, LOW);
  SPI.transfer(CMD_RDATA);
  uint32_t v = ((uint32_t)SPI.transfer(0) << 16);
  v |= ((uint32_t)SPI.transfer(0) << 8);
  v |= SPI.transfer(0);
  digitalWrite(PIN_CS, HIGH); SPI.endTransaction();
  if (v & 0x800000) v |= 0xFF000000;
  volts = (int32_t)v * VREF_INT / 8388608.0;
  return true;
}

void show(const char *name, uint8_t mux, float scale, const char *expect) {
  float v;
  if (!readOnce(mux, v)) { Serial.printf("%-22s  no data (DRDY timeout)\n", name); return; }
  v *= scale;
  const char *flag = (fabs(v / scale) > 2.0) ? "  <-- at/over range" : "";
  if (fabs(v) < 0.1) Serial.printf("%-22s %10.3f mV   expect %s%s\n", name, v * 1000.0, expect, flag);
  else               Serial.printf("%-22s %10.4f V    expect %s%s\n", name, v, expect, flag);
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);

  pinMode(PIN_CS, OUTPUT); digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_DRDY, INPUT_PULLUP);
  SPI.begin();

  adsCommand(CMD_RESET);
  delay(5);
  adsWriteReg(1, 0x00);   // 20 SPS, normal mode, single-shot
  adsWriteReg(2, 0x10);   // internal 2.048 V reference, 50/60 Hz rejection
  adsWriteReg(3, 0x00);
  Serial.println(F("\nADS1220 diagnostic (internal reference, gain 1)"));
}

void loop() {
  Serial.println(F("------------------------------------------------------------"));
  show("AVDD supply",        0b1101, 4.0, "~3.3 V");      // (AVDD-AVSS)/4 monitor
  show("AIN1 (green, S+)",   0b1001, 1.0, "~1.65 V");     // AIN1 vs AVSS
  show("AIN2 (white, S-)",   0b1010, 1.0, "~1.65 V");     // AIN2 vs AVSS
  show("AIN1 - AIN2 (signal)",0b0011, 1.0, "a few mV");
  show("Shorted inputs",     0b1110, 1.0, "~0 mV");       // internal short to mid-supply
  delay(2000);
}
