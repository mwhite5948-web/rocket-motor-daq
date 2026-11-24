/*
  Rocket motor static-test DAQ
  ----------------------------
  TAS501 200 kg S-beam load cell -> ADS1220 24-bit ADC -> Arduino Nano ESP32,
  logging thrust to an Adafruit MicroSD Card Breakout Board+ (ADA254).

  Wiring: see docs/RocketDAQ_wiring.pdf
    ADS1220 (default SPI bus)          MicroSD ADA254 (second SPI bus)
      CS   -> D10   DRDY -> D2           5V  -> 3V3 (on-board LDO takes 3.3-6 V)
      SCLK -> D13   3V3  -> 3V3          GND -> GND
      DIN  -> D11   GND  -> GND          CLK -> D5    DI -> D6
      DOUT -> D12                        DO  -> D7    CS -> D4
                                         CD  -> D8 (card detect)
    Load cell: Red -> AVDD + REFP0, Black -> AGND + REFN0,
               Green -> AIN1, White -> AIN2, Shield -> AGND

  How it records
    The ADS1220 runs at 1000 samples/s. A DRDY interrupt wakes a high-priority
    task on core 0 that reads each sample, timestamps it, and queues it. The
    main loop on core 1 converts samples to force and writes them to the SD
    card, so a slow SD write never causes a missed sample (the queue holds
    about 8 s of data).

    Recording STARTS when thrust rises above 1 lbf and STOPS when it falls
    below 1 lbf. Each burn goes to its own file, /BURN_000.CSV, /BURN_001.CSV ...
    Short dips below 1 lbf (shorter than STOP_HOLD_MS, e.g. a chuff or noise
    at tail-off) do not split the file; the below-threshold tail at the end
    of a burn is discarded, so every file runs from the first to the last
    sample above 1 lbf. A one-line summary of each burn (peak, average, total
    impulse, motor class) is appended to /SUMMARY.CSV and printed on Serial.

  Status LED (on-board RGB)
    blue          not calibrated, or taring / calibrating
    green         armed: calibrated, SD card ready, waiting for thrust
    red (solid)   recording a burn
    red (blink)   SD card missing or failed (thrust is still measured and
                  summarised on Serial, but nothing is saved)

  Serial Monitor: 115200 baud, line ending "Newline".
    t            Tare (zero) with the motor mounted and nothing firing
    c <kg>       Calibrate with a known mass, e.g.  c 20
    c <lb> lb    Calibrate with a known mass in pounds, e.g.  c 25 lb
    r            Toggle raw ADC counts in the live display
    d            Dump registers, calibration and SD status
    l            List files on the SD card
    x            Erase saved calibration
    h            Help
  Calibrate in the direction the motor pushes. The calibration keeps its
  sign, so thrust reads positive whichever way the cell is loaded.

  The stand auto-tares at power-up (AUTO_TARE_ON_BOOT), so mount the motor
  first, then power the DAQ. Calibration is kept in flash.
*/

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Preferences.h>

// ======================================================================
// User settings
// ======================================================================
const float    TRIGGER_LBF       = 1.0;     // start above / stop below this thrust
const int      TRIGGER_AVG_N     = 5;       // samples averaged for the trigger decision (5 ms)
const uint32_t STOP_HOLD_MS      = 100;     // thrust must stay below TRIGGER_LBF this long to end a burn
const bool     AUTO_TARE_ON_BOOT = true;    // tare automatically at power-up
const uint32_t PRINT_MS          = 250;     // live display interval
const uint32_t SD_FLUSH_MS       = 500;     // flush the open file at least this often while recording
const float    CAPACITY_KG       = 200.0;   // TAS501 rated capacity
const bool     CD_ACTIVE_HIGH    = true;    // ADA254: CD reads HIGH (via pull-up) when a card is inserted

// ======================================================================
// Pins (D-names work in either Nano ESP32 pin-numbering mode)
// ======================================================================
const int PIN_ADS_CS   = D10;   // ADS1220 on the default SPI bus (D13/D12/D11)
const int PIN_ADS_DRDY = D2;
const int PIN_SD_CS    = D4;    // MicroSD on a second SPI bus
const int PIN_SD_SCK   = D5;
const int PIN_SD_MOSI  = D6;    // ADA254 "DI"
const int PIN_SD_MISO  = D7;    // ADA254 "DO"
const int PIN_SD_CD    = D8;    // ADA254 "CD"
const uint32_t SD_SPI_HZ = 10000000;

// ======================================================================
// ADS1220
// ======================================================================
const uint8_t CMD_RESET = 0x06;
const uint8_t CMD_START = 0x08;
const uint8_t CMD_RDATA = 0x10;
const uint8_t CMD_RREG  = 0x20;
const uint8_t CMD_WREG  = 0x40;

// REG0: MUX = 0011 (AIN1+, AIN2-), GAIN = 111 (x128), PGA enabled
const uint8_t REG0 = 0x3E;
// REG1: DR = 110 (1000 SPS in normal mode), MODE = 00, CM = 1 (continuous), TS = 0, BCS = 0
const uint8_t REG1 = 0xC4;
// REG2: VREF = 01 (external REFP0/REFN0), 50/60 Hz filter off (only valid at 20 SPS), IDAC off
const uint8_t REG2 = 0x40;
// REG3: IDACs off, DRDYM = 0 (data ready only on DRDY)
const uint8_t REG3 = 0x00;
const float   SAMPLE_RATE_HZ = 1000.0;

const float KG_TO_LB = 2.20462;
const float KG_TO_N  = 9.80665;

SPISettings adsSPI(2000000, MSBFIRST, SPI_MODE1);
SPIClass    sdSPI(HSPI);
Preferences prefs;

// ======================================================================
// Sample acquisition (DRDY interrupt -> task on core 0 -> queue)
// ======================================================================
struct Sample {
  uint32_t tUs;   // micros() at DRDY
  int32_t  raw;   // signed 24-bit ADC result
};

const int     QUEUE_LEN = 8192;              // ~8 s at 1000 SPS
QueueHandle_t sampleQueue;
TaskHandle_t  sampleTask;
volatile uint32_t drdyTimeUs   = 0;
volatile uint32_t droppedCount = 0;          // queue full: samples lost
volatile bool     adsTimeout   = false;

// ======================================================================
// State
// ======================================================================
int32_t tareOffset  = 0;
float   countsPerKg = 0.0;     // 0 = not calibrated
bool    showRaw     = false;
String  inputLine;

// Tare / calibration collect samples from the stream instead of blocking
enum CollectMode { COLLECT_NONE, COLLECT_TARE, COLLECT_CAL };
const int   CAL_SAMPLES = 2000;              // 2 s at 1000 SPS
CollectMode collectMode = COLLECT_NONE;
int64_t     collectSum  = 0;
int         collectCount = 0;
float       calKnownKg  = 0;

// Trigger filter
int32_t trigRing[TRIGGER_AVG_N];
int64_t trigSum = 0;
int     trigIdx = 0, trigFill = 0;

// SD card
bool     sdOk = false;
uint32_t lastSdTry = 0;
int      nextBurnNum = 0;

// Burn recording
bool     recording = false;
File     burnFile;
char     burnName[20];
uint32_t burnStartUs = 0, lastSampleUs = 0, lastFlush = 0;
uint32_t burnSamples = 0, burnSaturated = 0, burnDroppedAtStart = 0;
float    burnPeakN = 0, burnImpulseNs = 0, prevForceN = 0;
bool     burnWriteFailed = false;

// Tail of samples below the threshold, held until we know the burn is over
const int PENDING_MAX = 1024;
Sample    pending[PENDING_MAX];
int       pendingCount = 0;

// Write buffer: SD likes big writes
const size_t WBUF_SIZE = 8192;
char   wbuf[WBUF_SIZE];
size_t wlen = 0;

// Live display accumulators
int64_t dispSum = 0;
int32_t dispCount = 0, dispLo = INT32_MAX, dispHi = INT32_MIN;

// ======================================================================
// ADS1220 low level
// ======================================================================
void adsCommand(uint8_t cmd) {
  SPI.beginTransaction(adsSPI);
  digitalWrite(PIN_ADS_CS, LOW);
  SPI.transfer(cmd);
  digitalWrite(PIN_ADS_CS, HIGH);
  SPI.endTransaction();
}

void adsWriteRegs(uint8_t r0, uint8_t r1, uint8_t r2, uint8_t r3) {
  SPI.beginTransaction(adsSPI);
  digitalWrite(PIN_ADS_CS, LOW);
  SPI.transfer(CMD_WREG | (0 << 2) | 3);
  SPI.transfer(r0); SPI.transfer(r1); SPI.transfer(r2); SPI.transfer(r3);
  digitalWrite(PIN_ADS_CS, HIGH);
  SPI.endTransaction();
}

void adsReadRegs(uint8_t *regs) {
  SPI.beginTransaction(adsSPI);
  digitalWrite(PIN_ADS_CS, LOW);
  SPI.transfer(CMD_RREG | (0 << 2) | 3);
  for (int i = 0; i < 4; i++) regs[i] = SPI.transfer(0x00);
  digitalWrite(PIN_ADS_CS, HIGH);
  SPI.endTransaction();
}

int32_t adsReadRaw() {
  SPI.beginTransaction(adsSPI);
  digitalWrite(PIN_ADS_CS, LOW);
  SPI.transfer(CMD_RDATA);
  uint32_t v = ((uint32_t)SPI.transfer(0x00) << 16);
  v |= ((uint32_t)SPI.transfer(0x00) << 8);
  v |= (uint32_t)SPI.transfer(0x00);
  digitalWrite(PIN_ADS_CS, HIGH);
  SPI.endTransaction();
  if (v & 0x800000) v |= 0xFF000000;
  return (int32_t)v;
}

bool adsInit() {
  pinMode(PIN_ADS_CS, OUTPUT);
  digitalWrite(PIN_ADS_CS, HIGH);
  pinMode(PIN_ADS_DRDY, INPUT_PULLUP);
  SPI.begin();

  adsCommand(CMD_RESET);
  delay(5);
  adsWriteRegs(REG0, REG1, REG2, REG3);
  delay(1);

  uint8_t r[4];
  adsReadRegs(r);
  if (r[0] != REG0 || r[1] != REG1 || r[2] != REG2 || r[3] != REG3) {
    Serial.printf("ADS1220 register check FAILED: read %02X %02X %02X %02X, expected %02X %02X %02X %02X\n",
                  r[0], r[1], r[2], r[3], REG0, REG1, REG2, REG3);
    return false;
  }
  return true;
}

void IRAM_ATTR drdyIsr() {
  drdyTimeUs = micros();
  BaseType_t woke = pdFALSE;
  vTaskNotifyGiveFromISR(sampleTask, &woke);
  if (woke) portYIELD_FROM_ISR();
}

void sampleTaskFn(void *) {
  for (;;) {
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200)) == 0) {
      adsTimeout = true;                     // no DRDY for 200 ms
      continue;
    }
    Sample s;
    s.tUs = drdyTimeUs;
    s.raw = adsReadRaw();
    if (xQueueSend(sampleQueue, &s, 0) != pdTRUE) droppedCount = droppedCount + 1;   // only this task writes it
  }
}

void startAcquisition() {
  sampleQueue = xQueueCreate(QUEUE_LEN, sizeof(Sample));
  xTaskCreatePinnedToCore(sampleTaskFn, "ads", 4096, nullptr, configMAX_PRIORITIES - 1, &sampleTask, 0);
  attachInterrupt(digitalPinToInterrupt(PIN_ADS_DRDY), drdyIsr, FALLING);
  adsCommand(CMD_START);                     // begin continuous conversions
}

// ======================================================================
// Status LED (on-board RGB, active low)
// ======================================================================
void setLed(bool r, bool g, bool b) {
  digitalWrite(LED_RED,   r ? LOW : HIGH);
  digitalWrite(LED_GREEN, g ? LOW : HIGH);
  digitalWrite(LED_BLUE,  b ? LOW : HIGH);
}

void updateLed() {
  if (recording)                                       setLed(true, false, false);
  else if (countsPerKg == 0 || collectMode != COLLECT_NONE) setLed(false, false, true);
  else if (!sdOk)                                      setLed((millis() / 250) % 2, false, false);
  else                                                 setLed(false, true, false);
}

// ======================================================================
// Calibration storage (same namespace as the original scale sketch)
// ======================================================================
void loadCalibration() {
  prefs.begin("scale", true);
  tareOffset  = prefs.getInt("offset", 0);
  countsPerKg = prefs.getFloat("cpk", 0.0);
  prefs.end();
}

void saveCalibration() {
  prefs.begin("scale", false);
  prefs.putInt("offset", tareOffset);
  prefs.putFloat("cpk", countsPerKg);
  prefs.end();
}

void clearCalibration() {
  prefs.begin("scale", false);
  prefs.clear();
  prefs.end();
  tareOffset = 0;
  countsPerKg = 0.0;
}

inline float rawToKg(int32_t raw) { return (raw - tareOffset) / countsPerKg; }

// ======================================================================
// SD card
// ======================================================================
bool cardPresent() {
  return (digitalRead(PIN_SD_CD) == HIGH) == CD_ACTIVE_HIGH;
}

void findNextBurnNumber() {
  nextBurnNum = 0;
  File root = SD.open("/");
  if (!root) return;
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    int n;
    const char *name = f.name();
    if (name[0] == '/') name++;
    if (sscanf(name, "BURN_%d.CSV", &n) == 1 && n >= nextBurnNum) nextBurnNum = n + 1;
    f.close();
  }
  root.close();
}

void sdMount() {
  lastSdTry = millis();
  if (!cardPresent()) {
    if (sdOk) Serial.println(F("SD card removed."));
    if (sdOk) SD.end();
    sdOk = false;
    return;
  }
  if (sdOk) return;
  if (!SD.begin(PIN_SD_CS, sdSPI, SD_SPI_HZ)) {
    Serial.println(F("SD card present but mount failed (FAT32/exFAT formatted? wiring?)."));
    SD.end();
    return;
  }
  sdOk = true;
  findNextBurnNumber();
  Serial.printf("SD card mounted, %llu MB. Next file: /BURN_%03d.CSV\n",
                SD.cardSize() / (1024ULL * 1024ULL), nextBurnNum);
}

void sdFail(const char *why) {
  Serial.printf("SD ERROR: %s. Recording continues on Serial only.\n", why);
  burnWriteFailed = true;
  if (burnFile) burnFile.close();
  SD.end();
  sdOk = false;
}

void wbufFlush() {
  if (wlen == 0) return;
  if (burnFile && !burnWriteFailed) {
    if (burnFile.write((const uint8_t *)wbuf, wlen) != wlen) sdFail("write failed");
  }
  wlen = 0;
}

void wbufAppend(const char *s, size_t n) {
  if (wlen + n > WBUF_SIZE) wbufFlush();
  memcpy(wbuf + wlen, s, n);
  wlen += n;
}

void listFiles() {
  if (!sdOk) { Serial.println(F("No SD card mounted.")); return; }
  File root = SD.open("/");
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    Serial.printf("  %-16s %10u bytes\n", f.name(), (unsigned)f.size());
    f.close();
  }
  root.close();
}

// ======================================================================
// Burn recording
// ======================================================================
String motorClass(float impulseNs) {
  if (impulseNs <= 0.3125) return "micro";
  if (impulseNs <= 0.625)  return "1/4A";
  if (impulseNs <= 1.25)   return "1/2A";
  int letter = 0;                             // A = 1.26 - 2.5 N*s, each letter doubles
  float top = 2.5;
  while (impulseNs > top && letter < 25) { top *= 2; letter++; }
  return String((char)('A' + letter));
}

void beginBurn() {
  recording = true;
  burnSamples = burnSaturated = 0;
  burnPeakN = burnImpulseNs = prevForceN = 0;
  burnWriteFailed = false;
  burnDroppedAtStart = droppedCount;
  pendingCount = 0;
  wlen = 0;
  lastFlush = millis();

  snprintf(burnName, sizeof(burnName), "/BURN_%03d.CSV", nextBurnNum);
  if (sdOk) {
    burnFile = SD.open(burnName, FILE_WRITE);
    if (!burnFile) sdFail("could not create file");
    else nextBurnNum++;
  } else {
    burnWriteFailed = true;
  }
  const char *hdr = "time_s,thrust_N,thrust_lbf,raw_counts\n";
  wbufAppend(hdr, strlen(hdr));
  Serial.printf("\n*** IGNITION: thrust > %.2f lbf, recording to %s\n",
                TRIGGER_LBF, burnWriteFailed ? "(no SD card)" : burnName);
}

void recordSample(const Sample &s) {
  float kg = rawToKg(s.raw);
  float n  = kg * KG_TO_N;
  if (burnSamples == 0) {
    burnStartUs = s.tUs;
  } else {
    float dt = (uint32_t)(s.tUs - lastSampleUs) * 1e-6f;
    burnImpulseNs += 0.5f * (n + prevForceN) * dt;     // trapezoidal integration
  }
  lastSampleUs = s.tUs;
  prevForceN = n;
  if (n > burnPeakN) burnPeakN = n;
  if (s.raw >= 0x7FFFFF || s.raw <= -0x800000) burnSaturated++;
  burnSamples++;

  char line[64];
  int len = snprintf(line, sizeof(line), "%.4f,%.3f,%.3f,%ld\n",
                     (uint32_t)(s.tUs - burnStartUs) * 1e-6, n, kg * KG_TO_LB, (long)s.raw);
  wbufAppend(line, len);
}

void endBurn() {
  recording = false;
  pendingCount = 0;                          // discard the below-threshold tail
  wbufFlush();
  uint32_t dropped = droppedCount - burnDroppedAtStart;
  float durS = (uint32_t)(lastSampleUs - burnStartUs) * 1e-6f;
  float avgN = durS > 0 ? burnImpulseNs / durS : 0;
  String cls = motorClass(burnImpulseNs);

  bool saved = false;
  if (burnFile) {
    burnFile.close();
    saved = !burnWriteFailed;
  }
  if (sdOk) {
    bool newSummary = !SD.exists("/SUMMARY.CSV");
    File sum = SD.open("/SUMMARY.CSV", FILE_APPEND);
    if (sum) {
      if (newSummary)
        sum.println(F("file,samples,duration_s,peak_N,peak_lbf,avg_N,impulse_Ns,class,saturated,dropped"));
      sum.printf("%s,%lu,%.4f,%.2f,%.2f,%.2f,%.3f,%s,%lu,%lu\n",
                 saved ? burnName + 1 : "(not saved)", (unsigned long)burnSamples, durS,
                 burnPeakN, burnPeakN / KG_TO_N * KG_TO_LB, avgN, burnImpulseNs, cls.c_str(),
                 (unsigned long)burnSaturated, (unsigned long)dropped);
      sum.close();
    }
  }

  Serial.println(F("\n*** BURNOUT: thrust < trigger"));
  Serial.printf("    file        %s\n", saved ? burnName : "NOT SAVED (no SD card or write error)");
  Serial.printf("    samples     %lu\n", (unsigned long)burnSamples);
  Serial.printf("    burn time   %.3f s\n", durS);
  Serial.printf("    peak        %.2f N  (%.2f lbf)\n", burnPeakN, burnPeakN / KG_TO_N * KG_TO_LB);
  Serial.printf("    average     %.2f N\n", avgN);
  Serial.printf("    impulse     %.3f N*s  (%s class)\n", burnImpulseNs, cls.c_str());
  if (burnSaturated) Serial.printf("    WARNING: %lu samples saturated the ADC\n", (unsigned long)burnSaturated);
  if (dropped)       Serial.printf("    WARNING: %lu samples dropped (queue overflow)\n", (unsigned long)dropped);
  if (burnPeakN / KG_TO_N > CAPACITY_KG) Serial.println(F("    WARNING: peak exceeded load cell capacity"));
  Serial.println();
}

// ======================================================================
// Per-sample processing
// ======================================================================
void finishCollect() {
  int32_t avg = (int32_t)(collectSum / collectCount);
  if (collectMode == COLLECT_TARE) {
    tareOffset = avg;
    saveCalibration();
    Serial.printf("Tare done. Offset = %ld counts\n", (long)tareOffset);
  } else {
    int32_t delta = avg - tareOffset;
    if (abs(delta) < 1000) {
      Serial.println(F("Calibration failed: almost no change from tare. Check the load and the signal wires."));
    } else {
      countsPerKg = (float)delta / calKnownKg;
      saveCalibration();
      Serial.printf("Calibration done. %.2f counts per kg (%.2f counts per N)\n",
                    countsPerKg, countsPerKg / KG_TO_N);
    }
  }
  collectMode = COLLECT_NONE;
  trigFill = 0; trigSum = 0; trigIdx = 0;    // restart the trigger filter
}

void processSample(const Sample &s) {
  // live display accumulators
  dispSum += s.raw; dispCount++;
  if (s.raw < dispLo) dispLo = s.raw;
  if (s.raw > dispHi) dispHi = s.raw;

  if (collectMode != COLLECT_NONE) {
    collectSum += s.raw;
    if (++collectCount >= CAL_SAMPLES) finishCollect();
    return;
  }

  // moving average used only for the trigger decision
  if (trigFill == TRIGGER_AVG_N) trigSum -= trigRing[trigIdx];
  else trigFill++;
  trigRing[trigIdx] = s.raw;
  trigSum += s.raw;
  trigIdx = (trigIdx + 1) % TRIGGER_AVG_N;

  if (countsPerKg == 0 || trigFill < TRIGGER_AVG_N) return;
  float filtLbf = rawToKg((int32_t)(trigSum / trigFill)) * KG_TO_LB;
  bool above = filtLbf > TRIGGER_LBF;

  if (!recording) {
    if (above) { beginBurn(); recordSample(s); }
    return;
  }

  if (above) {
    for (int i = 0; i < pendingCount; i++) recordSample(pending[i]);  // it was only a dip
    pendingCount = 0;
    recordSample(s);
  } else {
    if (pendingCount < PENDING_MAX) pending[pendingCount++] = s;
    if (pendingCount >= PENDING_MAX ||
        (uint32_t)(s.tUs - pending[0].tUs) >= STOP_HOLD_MS * 1000UL) {
      endBurn();
    }
  }

  if (recording && millis() - lastFlush >= SD_FLUSH_MS) {
    lastFlush = millis();
    wbufFlush();
    if (burnFile && !burnWriteFailed) burnFile.flush();
  }
}

// ======================================================================
// Commands
// ======================================================================
void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  t          tare (motor mounted, not firing)"));
  Serial.println(F("  c <kg>     calibrate with known mass, e.g. c 20"));
  Serial.println(F("  c <lb> lb  calibrate with known mass in pounds, e.g. c 25 lb"));
  Serial.println(F("  r          toggle raw counts"));
  Serial.println(F("  d          dump registers, calibration, SD status"));
  Serial.println(F("  l          list files on SD card"));
  Serial.println(F("  x          erase saved calibration"));
  Serial.println(F("  h          help"));
}

void startCollect(CollectMode mode) {
  collectMode = mode;
  collectSum = 0;
  collectCount = 0;
}

void dumpState() {
  uint8_t r[4];
  adsReadRegs(r);
  Serial.printf("ADS1220 regs: %02X %02X %02X %02X (expected %02X %02X %02X %02X)\n",
                r[0], r[1], r[2], r[3], REG0, REG1, REG2, REG3);
  Serial.printf("Sample rate: %.0f SPS, trigger %.2f lbf, stop hold %lu ms\n",
                SAMPLE_RATE_HZ, TRIGGER_LBF, (unsigned long)STOP_HOLD_MS);
  Serial.printf("Tare offset: %ld counts\n", (long)tareOffset);
  if (countsPerKg != 0) Serial.printf("Scale factor: %.2f counts/kg\n", countsPerKg);
  else                  Serial.println(F("Scale factor: not calibrated"));
  Serial.printf("SD card: %s%s, next file /BURN_%03d.CSV\n",
                cardPresent() ? "present" : "not detected", sdOk ? ", mounted" : ", not mounted", nextBurnNum);
  Serial.printf("Dropped samples since boot: %lu\n", (unsigned long)droppedCount);
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  char cmd = tolower(line.charAt(0));
  if (recording && cmd != 'd' && cmd != 'h') {
    Serial.println(F("Busy recording a burn."));
    return;
  }
  if (collectMode != COLLECT_NONE) {
    Serial.println(F("Busy taring / calibrating."));
    return;
  }
  switch (cmd) {
    case 't':
      Serial.println(F("Taring... keep the stand still, nothing firing."));
      startCollect(COLLECT_TARE);
      break;
    case 'c': {
      String arg = line.substring(1);
      arg.toLowerCase();
      float v = arg.toFloat();
      if (arg.indexOf("lb") >= 0) v /= KG_TO_LB;
      if (v <= 0) { Serial.println(F("Give the known mass, e.g. c 20  or  c 25 lb")); break; }
      calKnownKg = v;
      Serial.printf("Calibrating with %.3f kg (%.3f lb)... keep the load still.\n", v, v * KG_TO_LB);
      startCollect(COLLECT_CAL);
      break;
    }
    case 'r': showRaw = !showRaw; Serial.printf("Raw display %s\n", showRaw ? "on" : "off"); break;
    case 'd': dumpState(); break;
    case 'l': listFiles(); break;
    case 'x': clearCalibration(); Serial.println(F("Calibration erased. Tare and calibrate again.")); break;
    case 'h': printHelp(); break;
    default:  Serial.println(F("Unknown command. Send h for help."));
  }
}

void pollSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      handleCommand(inputLine);
      inputLine = "";
    } else if (inputLine.length() < 32) {
      inputLine += ch;
    }
  }
}

void printLive() {
  if (dispCount == 0) {
    if (adsTimeout) Serial.println(F("No data from ADS1220 (DRDY never went low). Check DRDY -> D2 and power."));
    adsTimeout = false;
    return;
  }
  int32_t avg = (int32_t)(dispSum / dispCount);
  bool saturated = dispHi >= 0x7FFFFF || dispLo <= -0x800000;
  dispSum = 0; dispCount = 0; dispLo = INT32_MAX; dispHi = INT32_MIN;
  adsTimeout = false;

  if (saturated) {
    Serial.println(F("ADC saturated: check load cell wiring (signal wires shorted or open?)"));
    return;
  }
  if (collectMode != COLLECT_NONE) return;
  if (countsPerKg == 0) {
    Serial.printf("Raw: %ld counts (uncalibrated)\n", (long)avg);
    return;
  }
  float kg = rawToKg(avg);
  const char *state = recording ? "RECORDING" : (sdOk ? "ARMED" : "ARMED (NO SD)");
  Serial.printf("%9.2f N  %8.2f lbf  %s", kg * KG_TO_N, kg * KG_TO_LB, state);
  if (showRaw) Serial.printf("  raw %ld", (long)avg);
  if (fabs(kg) > CAPACITY_KG) Serial.print(F("  OVERLOAD"));
  Serial.println();
}

// ======================================================================
// Setup and loop
// ======================================================================
void setup() {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  setLed(false, false, true);

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  Serial.println(F("\nRocket motor DAQ: TAS501 / ADS1220 / Nano ESP32 / MicroSD"));

  pinMode(PIN_SD_CD, INPUT_PULLUP);
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);
  sdSPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  sdMount();
  if (!sdOk && !cardPresent()) Serial.println(F("No SD card detected. Insert one to log burns."));

  while (!adsInit()) {
    Serial.println(F("Check SPI wiring and 3V3/GND to the ADS1220. Retrying in 2 s..."));
    delay(2000);
  }
  Serial.println(F("ADS1220 OK: AIN1-AIN2, gain 128, ext ref, 1000 SPS continuous"));

  loadCalibration();
  startAcquisition();

  if (countsPerKg == 0) {
    Serial.println(F("Not calibrated. Send t with the stand unloaded, then c <kg> with a known mass."));
  } else {
    Serial.printf("Loaded calibration: offset %ld, %.2f counts/kg\n", (long)tareOffset, countsPerKg);
    if (AUTO_TARE_ON_BOOT) {
      Serial.println(F("Auto-tare... keep the stand still."));
      startCollect(COLLECT_TARE);
    }
  }
  printHelp();
}

void loop() {
  Sample s;
  while (xQueueReceive(sampleQueue, &s, 0) == pdTRUE) processSample(s);

  pollSerial();

  if (!recording && millis() - lastSdTry > 1000) sdMount();   // card inserted / removed

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= PRINT_MS) {
    lastPrint = millis();
    printLive();
  }
  updateLed();
  delay(1);                                   // let the idle task run
}
