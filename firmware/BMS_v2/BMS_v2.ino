// BMS controller v2 firmware.
// Board: BMS controller PCB v2 with an ESP32-S3-DevKitC-1(U)-N8R8 and a Waveshare 2" ST7789V LCD.
// Pin map: pcb_v2/REVIEW.md and pcb_v2/gen/design.py.
// CMU naming and CAN handling are ported from StamPLC_SCIB.ino (validated 16-CMU timing, see cmucoms.md).
// Remote link: ESP-NOW on Wi-Fi channel 1 (see BMS_v2_interface.md). The same commands work on USB serial.

#include <Arduino.h>
#include <algorithm>
#include <M5GFX.h>
#include <lgfx/v1/panel/Panel_ST7789.hpp>
#include <Preferences.h>
#include <SCIBCMUComs.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>
#include <strings.h>

#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "rom/ets_sys.h"

using SCIBCMUComs::CommandFamily;
using SCIBCMUComs::Manager;
using SCIBCMUComs::ModuleData;
using SCIBCMUComs::kCellsPerModule;
using SCIBCMUComs::kMaxModules;
using SCIBCMUComs::kTempsPerModule;

// ------------------------------------------------------------------ pins (design.py)
static const int8_t kPinCmuIsense = 1;   // BTS5090 IS, 1.2k to GND
static const int8_t kPinTemp2 = 2;       // air probe, 1-Wire or NTC
static const int8_t kPinFan1Tach = 3;
static const int8_t kCanRxPin = 4;
static const int8_t kCanTxPin = 5;
static const int8_t kVEDirectRxPin = 6;  // from SmartShunt TX (ISO7321C)
static const int8_t kPinVmon = 7;        // 24 V monitor, 100k/10k
static const int8_t kPinCmuPower = 8;    // BTS5090 IN; also takes the TJA1042 out of standby
static const int8_t kPinEmsEnable = 9;   // high = EMS300 solenoids allowed to close
static const int8_t kPinTemp1 = 10;      // battery probe, 1-Wire or NTC
static const int8_t kPinTftMosi = 11;
static const int8_t kPinTftSck = 12;
static const int8_t kPinTftCs = 13;
static const int8_t kPinTftDc = 14;
static const int8_t kNamingTxPin = 15;   // L9637D TX -> K-line
static const int8_t kNamingRxPin = 16;   // naming return (RET divider), idles high
static const int8_t kPinEmsStatus = 17;  // opto, low while the EMS300 LED output is on
static const int8_t kPinFan2Tach = 18;
static const int8_t kPinFan1Pwm = 21;    // inverted: GPIO high = fan PWM line low (slower)
static const int8_t kPinBtnA = 39;
static const int8_t kPinBtnB = 40;
static const int8_t kPinBtnC = 41;
static const int8_t kVEDirectTxPin = 42; // to SmartShunt RX
static const int8_t kPinTftBl = 47;
static const int8_t kPinFan2Pwm = 48;    // inverted, as fan 1

// ------------------------------------------------------------------ CMU timing (unchanged from StamPLC_SCIB)
static const uint32_t kCanBitrate = 500000;
static const uint32_t kNamingBaud = 1200;
static const uint16_t kNamingCanPeriodMs = 120;
static const uint16_t kNamingCanPreArmMs = 1400;
static const uint16_t kNamingCanGapBeforeUartMs = 60;
static const uint16_t kNamingCanReplyWaitMs = 4000;
static const uint8_t kNamingCanDisarmFrames = 8;
static const uint8_t kNamingAttempts = 1;
static const uint16_t kNamingInterAttemptDelayMs = 600;
static const uint16_t kNamingPostFlushDelayMs = 25;
static const uint16_t kNamingReplyWindowMs = 150;
static const uint16_t kPowerRailSettleMs = 700;
static const uint16_t kPostNamingTxHoldoffMs = 6000;

static const uint32_t kVEDirectBaud = 19200;
static const uint32_t kVEDirectStaleMs = 5000;

static const uint32_t kFanPwmHz = 25000;
static const uint8_t kFanPwmBits = 8;
static const uint8_t kEspNowChannel = 1;
static const size_t kEspNowMaxLen = 250;
static const uint32_t kScreenTimeoutMs = 5UL * 60UL * 1000UL;
static const uint32_t kDrawPeriodMs = 250;
static const uint32_t kBroadcastPeriodMs = 1000;
static const uint32_t kLongPressMs = 700;
static const uint32_t kVmonDividerRatio = 11;  // (100k + 10k) / 10k
// VMON divides the rail after the input diode. Bench check: 19.78 V at the battery read 19.33 V.
static const uint32_t kVmonDiodeDropMv = 450;
// BTS5090-1EJA current sense: IIS = IL / kILIS into the 1.2k IS resistor. kILIS is 1460 typical from 0.5 A up
// (+/-16 % at 0.5 A) and 1650 +/-50 % at 50 mA (datasheet Rev 2.2, P_7.5.8 to P_7.5.12). In a fault the IS pin
// sources 6 to 35 mA, which pins the reading at the top of the ADC range.
static const float kCmuKilis = 1460.0f;
static const float kCmuIsOhms = 1200.0f;
static const uint16_t kCmuIsFaultMv = 2900;

// ------------------------------------------------------------------ display
class LGFX_BMS : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 panel_;
  lgfx::Bus_SPI bus_;
  lgfx::Light_PWM light_;

 public:
  LGFX_BMS() {
    auto b = bus_.config();
    b.spi_host = SPI2_HOST;
    b.spi_mode = 0;
    b.freq_write = 40000000;
    b.freq_read = 16000000;
    b.spi_3wire = true;
    b.use_lock = true;
    b.dma_channel = SPI_DMA_CH_AUTO;
    b.pin_sclk = kPinTftSck;
    b.pin_mosi = kPinTftMosi;
    b.pin_miso = -1;
    b.pin_dc = kPinTftDc;
    bus_.config(b);
    panel_.setBus(&bus_);

    auto p = panel_.config();
    p.pin_cs = kPinTftCs;
    p.pin_rst = -1;  // RST is tied to ESP EN
    p.pin_busy = -1;
    p.panel_width = 240;
    p.panel_height = 320;
    p.offset_x = 0;
    p.offset_y = 0;
    p.offset_rotation = 0;
    p.readable = false;
    p.invert = true;
    p.rgb_order = false;
    p.dlen_16bit = false;
    p.bus_shared = false;
    panel_.config(p);

    auto l = light_.config();
    l.pin_bl = kPinTftBl;
    l.invert = false;
    l.freq = 12000;
    l.pwm_channel = 7;
    light_.config(l);
    panel_.setLight(&light_);

    setPanel(&panel_);
  }
};

// ------------------------------------------------------------------ types
struct AppConfig {
  float tempOffsetC = -5.6f;
  bool keepaliveEnabled = true;
  bool balanceAllEnabled = true;
  CommandFamily family = CommandFamily::Family0A;
  uint8_t rotation = 1;
  uint8_t backlightPct = 100;
};

struct NamingReply {
  bool valid = false;
  uint64_t moduleMask = 0;
  uint8_t moduleCount = 0;
  uint8_t raw[8] = {0};
};

struct NamingAccumulator {
  uint8_t buf[8] = {0};
  uint8_t len = 0;
  NamingReply reply;
};

struct NamingResult {
  bool canStarted = false;
  bool uartStarted = false;
  bool replyValid = false;
  bool ok = false;
  uint8_t attemptsRun = 0;
  uint16_t armFrames = 0;
  uint16_t disarmFrames = 0;
  uint16_t uartFrames = 0;
  uint16_t txFailures = 0;
  NamingReply reply;
};

// Each field keeps its own timestamp. The SmartShunt alternates two text blocks (live values, then history
// H1..H18), each with its own Checksum, so a field is only stale once it hasn't been seen for kVEDirectStaleMs.
struct VEDirectLive {
  uint32_t lastFrameMs = 0;
  int32_t socDeciPct = -1;
  int32_t currentMa = 0;
  int32_t batteryMv = -1;
  uint32_t socMs = 0;
  uint32_t currentMs = 0;
  uint32_t batteryMs = 0;
};

struct VEDirectPending {
  int32_t socDeciPct = -1;
  int32_t currentMa = 0;
  int32_t batteryMv = -1;
  bool sawSoc = false;
  bool sawCurrent = false;
  bool sawBattery = false;
};

struct VEDirectDiag {
  uint32_t bytes = 0;
  uint32_t lines = 0;
  uint32_t goodFrames = 0;
  uint32_t badFrames = 0;
  uint32_t hexFrames = 0;
  uint32_t lastByteMs = 0;
  uint32_t lastGoodFrameMs = 0;
  uint32_t lastReportMs = 0;
  char lastLine[64] = {0};
};

struct SummaryView {
  uint8_t uartCount = 0;
  uint8_t canCount = 0;
  int32_t moduleHighMv = -1;
  int32_t moduleAvgMv = -1;
  int32_t moduleLowMv = -1;
  int32_t tempHighDeciF = -1;
  int32_t tempAvgDeciF = -1;
  int32_t tempLowDeciF = -1;
  int32_t socDeciPct = -1;
  int32_t currentMa = 0;
  int32_t batteryMv = -1;
  bool socValid = false;
  bool currentValid = false;
  bool batteryValid = false;
  uint16_t maxDeltaMv = 0;
};

enum class ProbeKind : uint8_t { None = 0, DS18B20, NTC };

struct TempProbe {
  int8_t pin;
  const char* name;
  ProbeKind kind = ProbeKind::None;
  bool converting = false;
  uint32_t convertStartMs = 0;
  uint32_t lastReadMs = 0;
  bool firstAfterPresence = true;
  float tempC = NAN;
  uint16_t ntcMv = 0;
  uint32_t crcErrors = 0;
};

struct Fan {
  int8_t pwmPin;
  int8_t tachPin;
  uint8_t pct = 0;
  volatile uint32_t pulses = 0;     // accepted edges
  volatile uint32_t rawEdges = 0;   // every falling edge the GPIO saw
  volatile uint32_t lastEdgeUs = 0;
  volatile uint32_t hist[4] = {0};  // raw edge spacing: <100 us, <1 ms, <lockout, >= lockout
  volatile uint32_t ring[64] = {0};  // accepted edge spacing, us
  volatile uint8_t ringPos = 0;
  uint32_t lastPulses = 0;
  uint32_t lastRaw = 0;
  uint16_t rpm = 0;
  uint16_t rawRpm = 0;
};

struct Button {
  int8_t pin;
  bool stableDown = false;
  bool lastRaw = false;
  uint32_t changedMs = 0;
  uint32_t downSinceMs = 0;
  bool longFired = false;
  bool clicked = false;
  bool longPressed = false;
};

// One coloured text run on a main-page row.
struct RowPart {
  uint16_t color;
  const char* text;
};

enum class Page : uint8_t { Main = 0, Debug };

enum DebugItem : uint8_t {
  kItemCmuPower = 0,
  kItemNaming,
  kItemEms,
  kItemFan1,
  kItemFan2,
  kItemFanBoth,
  kItemBalance,
  kItemBacklight,
  kItemFlip,
  kItemCount
};

// Command reply sink: USB serial, or ESP-NOW unicast back to the sender.
struct ReplyTarget {
  bool espnow = false;
  uint8_t mac[6] = {0};
};

struct PendingEspNowCommand {
  volatile bool ready = false;
  uint8_t mac[6] = {0};
  char text[160] = {0};
};

// ------------------------------------------------------------------ globals
LGFX_BMS g_lcd;
LGFX_Sprite g_line(&g_lcd);
Preferences g_prefs;
Manager g_cmu;
HardwareSerial g_namingUart(1);
HardwareSerial g_veUart(2);

AppConfig g_config;
VEDirectLive g_veLive;
VEDirectPending g_vePending;
VEDirectDiag g_veDiag;
PendingEspNowCommand g_pendingNow;

TempProbe g_probes[2] = {{kPinTemp1, "Batt"}, {kPinTemp2, "Air"}};
Fan g_fans[2] = {{kPinFan1Pwm, kPinFan1Tach}, {kPinFan2Pwm, kPinFan2Tach}};
Button g_btnA{kPinBtnA};
Button g_btnB{kPinBtnB};
Button g_btnC{kPinBtnC};

bool g_commPowerOn = false;
bool g_contactorEnabled = false;
bool g_cmuActive = false;
bool g_screenOn = true;
bool g_namingInProgress = false;
uint8_t g_uartDetectedCount = 0;
uint64_t g_uartDetectedMask = 0;
bool g_uartReplyValid = false;
NamingResult g_lastNaming;

bool g_espNowReady = false;
uint32_t g_espNowTx = 0;
uint32_t g_espNowTxFail = 0;
uint32_t g_espNowRx = 0;
uint32_t g_espNowLastRxMs = 0;

Page g_page = Page::Main;
uint8_t g_debugCursor = 0;

char g_statusLine[96] = "Ready";
char g_nodeId[20] = "bms-000000";
uint32_t g_statusUntilMs = 0;

uint32_t g_lastInteractionMs = 0;
uint32_t g_lastDrawMs = 0;
uint32_t g_lastBroadcastMs = 0;
uint32_t g_lastTachMs = 0;
uint32_t g_lastSerialReportMs = 0;

uint16_t g_vmonMv = 0;
uint16_t g_cmuIsMv = 0;
uint16_t g_cmuMa = 0;
bool g_cmuIsFault = false;
bool g_emsLedOn = false;

char g_veLine[96] = {0};
size_t g_veLineLen = 0;
uint8_t g_veChecksum = 0;
bool g_veHexMode = false;
bool g_veRawEcho = false;

char g_serialLine[160] = {0};
size_t g_serialLen = 0;

static const uint16_t COLOR_BG = TFT_BLACK;
static const uint16_t COLOR_TEXT = TFT_WHITE;
static const uint16_t COLOR_DIM = TFT_DARKGREY;
static const uint16_t COLOR_GOOD = TFT_GREENYELLOW;
static const uint16_t COLOR_WARN = TFT_ORANGE;
static const uint16_t COLOR_BAD = TFT_RED;
static const uint16_t COLOR_ACCENT = TFT_CYAN;
static const uint16_t COLOR_SELECT = 0x18E3;  // dark grey row highlight

// ------------------------------------------------------------------ helpers
static void setStatus(const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vsnprintf(g_statusLine, sizeof(g_statusLine), fmt, args);
  va_end(args);
  g_statusUntilMs = millis() + 5000;
  Serial.printf("[BMS] %s\n", g_statusLine);
}

static void setPersistentStatus(const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vsnprintf(g_statusLine, sizeof(g_statusLine), fmt, args);
  va_end(args);
  g_statusUntilMs = 0;
}

static bool veStreamActive() {
  return g_veDiag.lastByteMs != 0 && (millis() - g_veDiag.lastByteMs) <= kVEDirectStaleMs;
}

static bool veFieldFresh(uint32_t seenMs) {
  return seenMs != 0 && (millis() - seenMs) <= kVEDirectStaleMs;
}

static const char* familyLabel(CommandFamily family) {
  switch (family) {
    case CommandFamily::Family09:
      return "09";
    case CommandFamily::Family0A:
      return "0A";
    default:
      return "AUTO";
  }
}

static CommandFamily familyFromToken(const char* token) {
  if (strcasecmp(token, "09") == 0 || strcasecmp(token, "0x09") == 0) {
    return CommandFamily::Family09;
  }
  if (strcasecmp(token, "0A") == 0 || strcasecmp(token, "0x0A") == 0) {
    return CommandFamily::Family0A;
  }
  return CommandFamily::Auto;
}

static bool isTruthyToken(const char* token, bool currentValue, bool& parsedValue) {
  if (!token) {
    return false;
  }
  if (strcasecmp(token, "ON") == 0 || strcmp(token, "1") == 0 || strcasecmp(token, "TRUE") == 0) {
    parsedValue = true;
    return true;
  }
  if (strcasecmp(token, "OFF") == 0 || strcmp(token, "0") == 0 || strcasecmp(token, "FALSE") == 0) {
    parsedValue = false;
    return true;
  }
  if (strcasecmp(token, "TOGGLE") == 0) {
    parsedValue = !currentValue;
    return true;
  }
  return false;
}

static void updateNodeIdentity() {
  const uint64_t efuseMac = ESP.getEfuseMac();
  const unsigned long suffix = static_cast<unsigned long>(efuseMac & 0xFFFFFFULL);
  snprintf(g_nodeId, sizeof(g_nodeId), "bms-%06lX", suffix);
}

static float cToF(float c) {
  return c * 9.0f / 5.0f + 32.0f;
}

// ------------------------------------------------------------------ config
static void loadConfig() {
  g_prefs.begin("bms-v2", true);
  g_config.tempOffsetC = g_prefs.getFloat("tempoff", -5.6f);
  g_config.keepaliveEnabled = g_prefs.getBool("keepalive", true);
  g_config.balanceAllEnabled = g_prefs.getBool("balall", true);
  const uint8_t familyRaw = g_prefs.getUChar("family", static_cast<uint8_t>(CommandFamily::Family0A));
  g_config.rotation = g_prefs.getUChar("rot", 1) == 3 ? 3 : 1;  // landscape only: 1, or 3 for upside down
  g_config.backlightPct = g_prefs.getUChar("bl", 100);
  g_prefs.end();

  if (familyRaw == static_cast<uint8_t>(CommandFamily::Family09)) {
    g_config.family = CommandFamily::Family09;
  } else if (familyRaw == static_cast<uint8_t>(CommandFamily::Family0A)) {
    g_config.family = CommandFamily::Family0A;
  } else {
    g_config.family = CommandFamily::Auto;
  }
  if (g_config.backlightPct < 10 || g_config.backlightPct > 100) {
    g_config.backlightPct = 100;
  }
}

static void saveConfig() {
  g_prefs.begin("bms-v2", false);
  g_prefs.putFloat("tempoff", g_config.tempOffsetC);
  g_prefs.putBool("keepalive", g_config.keepaliveEnabled);
  g_prefs.putBool("balall", g_config.balanceAllEnabled);
  g_prefs.putUChar("family", static_cast<uint8_t>(g_config.family));
  g_prefs.putUChar("rot", g_config.rotation);
  g_prefs.putUChar("bl", g_config.backlightPct);
  g_prefs.end();
}

// ------------------------------------------------------------------ outputs: fans, EMS300, screen
static void setFanPct(uint8_t index, int pct) {
  if (index > 1) {
    return;
  }
  pct = constrain(pct, 0, 100);
  g_fans[index].pct = static_cast<uint8_t>(pct);
  // Open-drain FET inverts: full gate duty holds the fan PWM line low (0 %).
  const uint32_t maxDuty = (1U << kFanPwmBits) - 1U;
  const uint32_t duty = maxDuty - (static_cast<uint32_t>(pct) * maxDuty + 50U) / 100U;
  ledcWrite(g_fans[index].pwmPin, duty);
}

// The tach line rises slowly (10k pull-up, 1 nF) and the GPIO has no hysteresis, so the rising edge often triggers a
// false falling interrupt about half a period after the real one, plus short bursts. Measured on fan 2 at 100 %: real
// edges every ~10.1 ms (2970 rpm) with extra edges at ~5.1 ms and <100 us. An edge only counts if the line is still
// low when the ISR runs, and not within the lockout of the last accepted edge.
volatile uint32_t g_tachLockoutUs = 1000;

static inline void IRAM_ATTR onTachEdge(Fan& fan) {
  const uint32_t now = micros();
  const uint32_t dt = now - fan.lastEdgeUs;
  fan.rawEdges = fan.rawEdges + 1;
  if (gpio_get_level(static_cast<gpio_num_t>(fan.tachPin)) != 0) {
    fan.hist[0] = fan.hist[0] + 1;  // line already high: glitch on the rising edge
    return;
  }
  const uint8_t bucket = dt < 100 ? 0 : dt < 1000 ? 1 : dt < g_tachLockoutUs ? 2 : 3;
  fan.hist[bucket] = fan.hist[bucket] + 1;
  if (dt >= g_tachLockoutUs) {
    fan.lastEdgeUs = now;
    fan.pulses = fan.pulses + 1;
    fan.ring[fan.ringPos & 63] = dt;
    fan.ringPos = fan.ringPos + 1;
  }
}

static void IRAM_ATTR onFan1Tach() {
  onTachEdge(g_fans[0]);
}

static void IRAM_ATTR onFan2Tach() {
  onTachEdge(g_fans[1]);
}

static void initFans() {
  // Gate high = fan off. Drive it before LEDC takes over so the fans stop as early as possible.
  for (Fan& fan : g_fans) {
    pinMode(fan.pwmPin, OUTPUT);
    digitalWrite(fan.pwmPin, HIGH);
  }
  for (uint8_t i = 0; i < 2; ++i) {
    ledcAttach(g_fans[i].pwmPin, kFanPwmHz, kFanPwmBits);
    setFanPct(i, 0);
    pinMode(g_fans[i].tachPin, INPUT);  // 10k pull-up on the board
  }
  attachInterrupt(digitalPinToInterrupt(kPinFan1Tach), onFan1Tach, FALLING);
  attachInterrupt(digitalPinToInterrupt(kPinFan2Tach), onFan2Tach, FALLING);
}

static void serviceFanTach() {
  const uint32_t now = millis();
  const uint32_t elapsed = now - g_lastTachMs;
  if (elapsed < 1000) {
    return;
  }
  g_lastTachMs = now;
  for (Fan& fan : g_fans) {
    const uint32_t pulses = fan.pulses;
    const uint32_t delta = pulses - fan.lastPulses;
    fan.lastPulses = pulses;
    // 2 pulses per revolution, so rpm = 30e6 / edge spacing in us. The median of the latest spacings ignores the
    // odd false edge that still gets through the ISR checks.
    if (delta < 2) {
      fan.rpm = 0;
    } else {
      const uint8_t n = static_cast<uint8_t>(min<uint32_t>(delta, 16));
      const uint8_t end = fan.ringPos;
      uint32_t recent[16];
      for (uint8_t k = 0; k < n; ++k) {
        recent[k] = fan.ring[static_cast<uint8_t>(end - 1 - k) & 63];
      }
      std::sort(recent, recent + n);
      const uint32_t median = recent[n / 2];
      fan.rpm = median ? static_cast<uint16_t>(min<uint32_t>(30000000UL / median, 65535)) : 0;
    }
    const uint32_t raw = fan.rawEdges;
    fan.rawRpm = static_cast<uint16_t>(((raw - fan.lastRaw) * 30000UL) / elapsed);
    fan.lastRaw = raw;
  }
}

static void setContactorEnabled(bool enabled) {
  if (g_contactorEnabled == enabled) {
    return;
  }
  g_contactorEnabled = enabled;
  digitalWrite(kPinEmsEnable, enabled ? HIGH : LOW);
  setStatus("Contactor %s", enabled ? "ENABLED" : "DISABLED");
}

static void applyBacklight() {
  g_lcd.setBrightness(g_screenOn ? static_cast<uint8_t>((g_config.backlightPct * 255U) / 100U) : 0);
}

static void wakeScreen() {
  g_lastInteractionMs = millis();
  if (!g_screenOn) {
    g_screenOn = true;
    applyBacklight();
  }
}

static void sleepScreen() {
  g_screenOn = false;
  applyBacklight();
}

// ------------------------------------------------------------------ analog inputs
static void serviceAnalog() {
  static uint32_t lastMs = 0;
  const uint32_t now = millis();
  if (now - lastMs < 200) {
    return;
  }
  lastMs = now;
  const uint32_t railMv = analogReadMilliVolts(kPinVmon) * kVmonDividerRatio;
  // Only add the diode drop when the 24 V input is actually powering the rail (not back-fed from USB).
  g_vmonMv = static_cast<uint16_t>(railMv > 6000 ? railMv + kVmonDiodeDropMv : railMv);
  g_cmuIsMv = static_cast<uint16_t>(analogReadMilliVolts(kPinCmuIsense));
  g_cmuMa = static_cast<uint16_t>(lroundf(g_cmuIsMv * kCmuKilis / kCmuIsOhms));
  g_cmuIsFault = g_commPowerOn && g_cmuIsMv >= kCmuIsFaultMv;
  g_emsLedOn = digitalRead(kPinEmsStatus) == LOW;
}

// ------------------------------------------------------------------ 1-Wire DS18B20 (one probe per bus, skip ROM)
// The pin runs as open-drain input/output; the board has a 4.7k pull-up to 3V3.
static portMUX_TYPE g_owMux = portMUX_INITIALIZER_UNLOCKED;

static void owConfigure(int8_t pin) {
  gpio_reset_pin(static_cast<gpio_num_t>(pin));
  gpio_set_direction(static_cast<gpio_num_t>(pin), GPIO_MODE_INPUT_OUTPUT_OD);
  gpio_set_pull_mode(static_cast<gpio_num_t>(pin), GPIO_FLOATING);
  gpio_set_level(static_cast<gpio_num_t>(pin), 1);
}

static bool owReset(int8_t pin) {
  const gpio_num_t g = static_cast<gpio_num_t>(pin);
  if (gpio_get_level(g) == 0) {
    return false;  // line held low (shorted, or an NTC to GND)
  }
  gpio_set_level(g, 0);
  ets_delay_us(480);
  portENTER_CRITICAL(&g_owMux);
  gpio_set_level(g, 1);
  ets_delay_us(70);
  const bool present = gpio_get_level(g) == 0;
  portEXIT_CRITICAL(&g_owMux);
  ets_delay_us(410);
  return present;
}

static void owWriteBit(gpio_num_t g, bool bit) {
  portENTER_CRITICAL(&g_owMux);
  gpio_set_level(g, 0);
  ets_delay_us(bit ? 6 : 60);
  gpio_set_level(g, 1);
  ets_delay_us(bit ? 64 : 10);
  portEXIT_CRITICAL(&g_owMux);
}

static bool owReadBit(gpio_num_t g) {
  portENTER_CRITICAL(&g_owMux);
  gpio_set_level(g, 0);
  ets_delay_us(6);
  gpio_set_level(g, 1);
  ets_delay_us(9);
  const bool bit = gpio_get_level(g) != 0;
  ets_delay_us(55);
  portEXIT_CRITICAL(&g_owMux);
  return bit;
}

static void owWriteByte(int8_t pin, uint8_t v) {
  for (uint8_t i = 0; i < 8; ++i) {
    owWriteBit(static_cast<gpio_num_t>(pin), (v >> i) & 1U);
  }
}

static uint8_t owReadByte(int8_t pin) {
  uint8_t v = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    if (owReadBit(static_cast<gpio_num_t>(pin))) {
      v |= static_cast<uint8_t>(1U << i);
    }
  }
  return v;
}

static uint8_t owCrc8(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0;
  while (len--) {
    uint8_t in = *data++;
    for (uint8_t i = 0; i < 8; ++i) {
      const uint8_t mix = (crc ^ in) & 0x01;
      crc >>= 1;
      if (mix) {
        crc ^= 0x8C;
      }
      in >>= 1;
    }
  }
  return crc;
}

// No DS18B20: read the pin as a 10k B3950 NTC under the 4.7k pull-up. An open input reads about 3.1 V on the
// ESP32-S3 ADC (measured on the v2 board), so anything above 3.05 V counts as no probe. That limits NTC readings to
// about -12 C and up.
static void readNtc(TempProbe& p) {
  const uint32_t mv = analogReadMilliVolts(p.pin);
  owConfigure(p.pin);
  p.ntcMv = static_cast<uint16_t>(mv);
  if (mv < 100 || mv > 3050) {
    p.kind = ProbeKind::None;
    p.tempC = NAN;
    return;
  }
  const float r = 4700.0f * mv / (3300.0f - mv);
  const float invT = 1.0f / 298.15f + logf(r / 10000.0f) / 3950.0f;
  p.kind = ProbeKind::NTC;
  p.tempC = 1.0f / invT - 273.15f;
}

static void serviceProbe(TempProbe& p) {
  const uint32_t now = millis();
  if (p.converting) {
    if (now - p.convertStartMs < 800) {
      return;
    }
    p.converting = false;
    p.lastReadMs = now;
    if (!owReset(p.pin)) {
      p.kind = ProbeKind::None;
      p.tempC = NAN;
      return;
    }
    owWriteByte(p.pin, 0xCC);  // skip ROM
    owWriteByte(p.pin, 0xBE);  // read scratchpad
    uint8_t sp[9];
    for (uint8_t i = 0; i < 9; ++i) {
      sp[i] = owReadByte(p.pin);
    }
    if (owCrc8(sp, 8) != sp[8]) {
      p.crcErrors++;
      return;
    }
    const int16_t raw = static_cast<int16_t>((sp[1] << 8) | sp[0]);
    const float c = raw / 16.0f;
    // 85.0 is the power-on value; ignore it on the first read after the probe appears.
    if (p.firstAfterPresence && raw == 0x0550) {
      p.firstAfterPresence = false;
      return;
    }
    p.firstAfterPresence = false;
    p.kind = ProbeKind::DS18B20;
    p.tempC = c;
    return;
  }

  if (now - p.lastReadMs < 1000) {
    return;
  }
  p.lastReadMs = now;
  if (!owReset(p.pin)) {
    p.firstAfterPresence = true;
    readNtc(p);
    return;
  }
  owWriteByte(p.pin, 0xCC);
  owWriteByte(p.pin, 0x44);  // convert T, 12-bit, up to 750 ms
  p.converting = true;
  p.convertStartMs = now;
}

static void initProbes() {
  for (TempProbe& p : g_probes) {
    owConfigure(p.pin);
  }
}

static void serviceProbes() {
  for (TempProbe& p : g_probes) {
    serviceProbe(p);
  }
}

static const char* probeKindLabel(ProbeKind k) {
  switch (k) {
    case ProbeKind::DS18B20:
      return "DS18";
    case ProbeKind::NTC:
      return "NTC";
    default:
      return "none";
  }
}

// ------------------------------------------------------------------ CMU naming (ported from StamPLC_SCIB)
static void clearNamingUartRx() {
  while (g_namingUart.available() > 0) {
    g_namingUart.read();
  }
}

static void dumpTwaiStatus(const char* tag) {
  twai_status_info_t info;
  if (twai_get_status_info(&info) != ESP_OK) {
    Serial.printf("[BMS] %s: twai status unavailable\n", tag);
    return;
  }
  Serial.printf("[BMS] %s: state=%d tx_err=%u rx_err=%u tx_failed=%u rx_missed=%u rx_overrun=%u arb_lost=%u bus_err=%u msgs_tx=%u msgs_rx=%u\n",
                tag, static_cast<int>(info.state), static_cast<unsigned>(info.tx_error_counter),
                static_cast<unsigned>(info.rx_error_counter), static_cast<unsigned>(info.tx_failed_count),
                static_cast<unsigned>(info.rx_missed_count), static_cast<unsigned>(info.rx_overrun_count),
                static_cast<unsigned>(info.arb_lost_count), static_cast<unsigned>(info.bus_error_count),
                static_cast<unsigned>(info.msgs_to_tx), static_cast<unsigned>(info.msgs_to_rx));
}

static bool parseNamingReplyPacket(const uint8_t in[8], NamingReply& out) {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < 7; ++i) {
    checksum = static_cast<uint8_t>(checksum + in[i]);
  }
  if (checksum != in[7]) {
    return false;
  }
  if (in[0] != 0x00) {
    return false;
  }

  uint64_t mask = 0;
  uint8_t count = 0;
  for (uint8_t i = 0; i < 6; ++i) {
    mask |= (static_cast<uint64_t>(in[1 + i]) << (8U * i));
    count = static_cast<uint8_t>(count + __builtin_popcount(static_cast<unsigned>(in[1 + i])));
  }

  out.valid = true;
  out.moduleMask = mask;
  out.moduleCount = count;
  memcpy(out.raw, in, 8);
  return true;
}

static void pollNamingReply(NamingAccumulator& acc) {
  while (g_namingUart.available() > 0 && !acc.reply.valid) {
    const int raw = g_namingUart.read();
    if (raw < 0) {
      return;
    }
    const uint8_t b = static_cast<uint8_t>(raw);

    if (acc.len == 0) {
      if (b != 0x00) {
        continue;
      }
      acc.buf[acc.len++] = b;
      continue;
    }

    acc.buf[acc.len++] = b;
    if (acc.len < sizeof(acc.buf)) {
      continue;
    }

    NamingReply reply;
    if (parseNamingReplyPacket(acc.buf, reply)) {
      acc.reply = reply;
      return;
    }

    memmove(acc.buf, acc.buf + 1, sizeof(acc.buf) - 1);
    acc.len = sizeof(acc.buf) - 1;
    while (acc.len > 0 && acc.buf[0] != 0x00) {
      memmove(acc.buf, acc.buf + 1, acc.len - 1);
      acc.len--;
    }
  }
}

static twai_timing_config_t timingForBitrate(uint32_t bitrate) {
  switch (bitrate) {
    case 1000000:
      return TWAI_TIMING_CONFIG_1MBITS();
    case 800000:
      return TWAI_TIMING_CONFIG_800KBITS();
    case 500000:
      return TWAI_TIMING_CONFIG_500KBITS();
    case 250000:
      return TWAI_TIMING_CONFIG_250KBITS();
    case 125000:
      return TWAI_TIMING_CONFIG_125KBITS();
    case 100000:
      return TWAI_TIMING_CONFIG_100KBITS();
    default:
      return TWAI_TIMING_CONFIG_500KBITS();
  }
}

static bool bootCanStart() {
  twai_stop();
  twai_driver_uninstall();

  twai_general_config_t g_config_twai =
      TWAI_GENERAL_CONFIG_DEFAULT(static_cast<gpio_num_t>(kCanTxPin), static_cast<gpio_num_t>(kCanRxPin), TWAI_MODE_NORMAL);
  g_config_twai.rx_queue_len = 32;
  g_config_twai.tx_queue_len = 16;

  const twai_timing_config_t t_config = timingForBitrate(kCanBitrate);
  const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&g_config_twai, &t_config, &f_config);
  if (err == ESP_ERR_INVALID_STATE) {
    twai_stop();
    twai_driver_uninstall();
    err = twai_driver_install(&g_config_twai, &t_config, &f_config);
  }
  if (err != ESP_OK) {
    return false;
  }

  err = twai_start();
  if (err != ESP_OK) {
    twai_driver_uninstall();
    return false;
  }
  dumpTwaiStatus("bootCanStart");
  return true;
}

static void bootCanStop() {
  twai_stop();
  twai_driver_uninstall();
}

static bool bootSend3C3(bool arm) {
  uint8_t payload[8] = {0, 0, 0, static_cast<uint8_t>(arm ? 0x04 : 0x00), 0, 0, 0, 0};

  twai_message_t msg;
  memset(&msg, 0, sizeof(msg));
  msg.identifier = 0x3C3;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = 8;
  memcpy(msg.data, payload, sizeof(payload));
  const esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(20));
  return err == ESP_OK;
}

static void namingDelayPoll(NamingAccumulator& acc, uint16_t durationMs) {
  const uint32_t untilMs = millis() + durationMs;
  while (millis() < untilMs) {
    pollNamingReply(acc);
    delay(5);
  }
}

static void collectNamingReply(NamingAccumulator& acc, uint16_t durationMs) {
  if (kNamingRxPin < 0) {
    return;
  }
  namingDelayPoll(acc, durationMs);
}

static bool runNamingAttempt(NamingResult& result) {
  static const uint8_t kNamingFrame[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  NamingAccumulator acc;
  clearNamingUartRx();

  // CAN arm ("listen") on 0x3C3 every 120 ms for 1400 ms before the UART frame.
  bool ok = true;
  const uint32_t preStart = millis();
  while (millis() - preStart < kNamingCanPreArmMs) {
    const bool sent = bootSend3C3(true);
    ok = sent && ok;
    if (!sent) {
      result.txFailures++;
    }
    result.armFrames++;
    delay(kNamingCanPeriodMs);
  }

  {
    const bool sent = bootSend3C3(true);
    ok = sent && ok;
    if (!sent) {
      result.txFailures++;
    }
  }
  result.armFrames++;
  delay(kNamingCanGapBeforeUartMs);

  const size_t written = g_namingUart.write(kNamingFrame, sizeof(kNamingFrame));
  g_namingUart.flush();
  result.uartFrames++;
  ok = (written == sizeof(kNamingFrame)) && ok;
  namingDelayPoll(acc, kNamingPostFlushDelayMs);

  // Hold the arm until the chain replies (large chains reply late) or the wait times out.
  const uint32_t replyWaitStart = millis();
  while ((millis() - replyWaitStart) < kNamingCanReplyWaitMs && !acc.reply.valid) {
    const bool sent = bootSend3C3(true);
    ok = sent && ok;
    if (!sent) {
      result.txFailures++;
    }
    result.armFrames++;
    namingDelayPoll(acc, kNamingCanPeriodMs);
  }

  for (uint8_t i = 0; i < kNamingCanDisarmFrames; ++i) {
    const bool sent = bootSend3C3(false);
    ok = sent && ok;
    if (!sent) {
      result.txFailures++;
    }
    result.disarmFrames++;
    namingDelayPoll(acc, kNamingCanPeriodMs);
  }

  collectNamingReply(acc, kNamingReplyWindowMs);
  if (acc.reply.valid) {
    result.replyValid = true;
    result.reply = acc.reply;
  }

  return ok;
}

static void stopCmuManager() {
  if (!g_cmuActive) {
    return;
  }
  g_cmu.end();
  g_cmuActive = false;
}

static bool startCmuManager() {
  if (!g_cmu.begin(kCanBitrate, kCanTxPin, kCanRxPin)) {
    return false;
  }

  g_cmu.setTempOffsetC(g_config.tempOffsetC);
  g_cmu.setTxPeriodMs(120);
  g_cmu.setKeepaliveEnabled(g_config.keepaliveEnabled);
  g_cmu.setTelemetryTxEnabled(true);
  g_cmu.setBalanceAllEnabled(g_config.balanceAllEnabled);
  g_cmu.setCommandFamily(g_config.family);
  g_cmu.setTxInhibitForMs(kPostNamingTxHoldoffMs);
  g_cmuActive = true;
  return true;
}

static void drawScreen();

static NamingResult runNamingSequence() {
  NamingResult result;

  stopCmuManager();
  bootCanStop();
  delay(20);

  g_namingUart.end();
  delay(10);
  g_namingUart.begin(kNamingBaud, SERIAL_8E1, kNamingRxPin, kNamingTxPin);
  result.uartStarted = true;
  clearNamingUartRx();

  if (!bootCanStart()) {
    setStatus("Naming failed: CAN init");
    g_lastNaming = result;
    return result;
  }
  result.canStarted = true;

  for (uint8_t attempt = 1; attempt <= kNamingAttempts; ++attempt) {
    result.attemptsRun = attempt;
    const bool attemptOk = runNamingAttempt(result);
    dumpTwaiStatus("after naming attempt");
    Serial.printf("[BMS] naming attempt=%u arm=%u disarm=%u uart=%u tx_fail=%u reply=%u ok=%u\n",
                  static_cast<unsigned>(attempt), static_cast<unsigned>(result.armFrames),
                  static_cast<unsigned>(result.disarmFrames), static_cast<unsigned>(result.uartFrames),
                  static_cast<unsigned>(result.txFailures), result.replyValid ? 1U : 0U, attemptOk ? 1U : 0U);
    if (attemptOk) {
      result.ok = true;
    }
    if (result.replyValid) {
      break;
    }
    if (attempt < kNamingAttempts) {
      delay(kNamingInterAttemptDelayMs);
    }
  }

  bootCanStop();

  g_uartReplyValid = result.replyValid;
  if (result.replyValid) {
    g_uartDetectedMask = result.reply.moduleMask;
    g_uartDetectedCount = result.reply.moduleCount;
    setStatus("Naming OK: %u CMUs", static_cast<unsigned>(g_uartDetectedCount));
    Serial.printf("[BMS] naming reply %02X %02X %02X %02X %02X %02X %02X %02X\n", result.reply.raw[0],
                  result.reply.raw[1], result.reply.raw[2], result.reply.raw[3], result.reply.raw[4],
                  result.reply.raw[5], result.reply.raw[6], result.reply.raw[7]);
  } else if (result.ok) {
    g_uartDetectedMask = 0;
    g_uartDetectedCount = 0;
    setStatus("Naming sent: no UART reply");
  } else {
    g_uartDetectedMask = 0;
    g_uartDetectedCount = 0;
    setStatus("Naming sequence failed");
  }

  if (g_commPowerOn) {
    if (!startCmuManager()) {
      setStatus("CMU CAN manager start failed");
    }
  }

  g_lastNaming = result;
  return result;
}

static void runNamingNow() {
  g_namingInProgress = true;
  setPersistentStatus("Naming CMUs...");
  drawScreen();
  runNamingSequence();
  g_namingInProgress = false;
}

static void setCommPower(bool enabled) {
  if (g_commPowerOn == enabled) {
    return;
  }

  g_commPowerOn = enabled;
  // GPIO8 switches the BTS5090 (CMU 12 V, K-line supply) and releases TJA1042 standby.
  digitalWrite(kPinCmuPower, enabled ? HIGH : LOW);

  if (!enabled) {
    stopCmuManager();
    bootCanStop();
    g_uartDetectedCount = 0;
    g_uartDetectedMask = 0;
    g_uartReplyValid = false;
    setStatus("CMU power OFF");
    return;
  }

  setPersistentStatus("CMU power ON, naming...");
  drawScreen();
  delay(kPowerRailSettleMs);
  runNamingNow();
}

// ------------------------------------------------------------------ VE.Direct text protocol
static void commitVeFrame() {
  const uint32_t now = millis();
  g_veLive.lastFrameMs = now;
  if (g_vePending.sawSoc) {
    g_veLive.socDeciPct = g_vePending.socDeciPct;
    g_veLive.socMs = now;
  }
  if (g_vePending.sawCurrent) {
    g_veLive.currentMa = g_vePending.currentMa;
    g_veLive.currentMs = now;
  }
  if (g_vePending.sawBattery) {
    g_veLive.batteryMv = g_vePending.batteryMv;
    g_veLive.batteryMs = now;
  }
  g_veDiag.goodFrames++;
  g_veDiag.lastGoodFrameMs = now;
  g_vePending = VEDirectPending();
}

static void recordVeLineForDiag(const char* line) {
  size_t n = 0;
  while (line[n] != '\0' && n < (sizeof(g_veDiag.lastLine) - 1)) {
    const char c = line[n];
    g_veDiag.lastLine[n] = (c >= 32 && c <= 126) ? c : '.';
    ++n;
  }
  g_veDiag.lastLine[n] = '\0';
}

// "---" means the SmartShunt has no value yet (SOC before the first full charge synchronises it).
static bool parseVeNumber(const char* value, int32_t& out) {
  char* end = nullptr;
  const long v = strtol(value, &end, 10);
  if (end == value || *end != '\0') {
    return false;
  }
  out = static_cast<int32_t>(v);
  return true;
}

static void handleVeLine(const char* line) {
  if (line[0] == '\0') {
    return;
  }

  recordVeLineForDiag(line);

  const char* tab = strchr(line, '\t');
  if (!tab) {
    return;
  }

  const size_t keyLen = static_cast<size_t>(tab - line);
  if (keyLen == 0 || keyLen >= 16) {
    return;
  }

  char key[16];
  memcpy(key, line, keyLen);
  key[keyLen] = '\0';
  const char* value = tab + 1;

  if (strcmp(key, "PID") == 0) {
    g_vePending = VEDirectPending();
    return;
  }
  if (strcmp(key, "SOC") == 0) {
    g_vePending.sawSoc = parseVeNumber(value, g_vePending.socDeciPct);
    return;
  }
  if (strcmp(key, "I") == 0) {
    g_vePending.sawCurrent = parseVeNumber(value, g_vePending.currentMa);
    return;
  }
  if (strcmp(key, "V") == 0) {
    g_vePending.sawBattery = parseVeNumber(value, g_vePending.batteryMv);
    return;
  }
  if (strcmp(key, "Checksum") == 0) {
    if (g_veChecksum == 0) {
      commitVeFrame();
    } else {
      g_veDiag.badFrames++;
      if (g_veRawEcho) {
        Serial.printf("[VE] checksum mismatch (sum %02X)\n", g_veChecksum);
      }
      g_vePending = VEDirectPending();
    }
    return;
  }
}

static void pollVeDirect() {
  while (g_veUart.available() > 0) {
    const int raw = g_veUart.read();
    if (raw < 0) {
      return;
    }
    const uint8_t b = static_cast<uint8_t>(raw);
    g_veDiag.bytes++;
    g_veDiag.lastByteMs = millis();

    // Asynchronous hex messages (":...\n") can appear between text lines and are not part of the checksum.
    if (g_veHexMode) {
      if (b == '\n') {
        g_veHexMode = false;
        g_veDiag.hexFrames++;
      }
      continue;
    }
    if (b == ':' && g_veLineLen == 0) {
      g_veHexMode = true;
      continue;
    }

    g_veChecksum = static_cast<uint8_t>(g_veChecksum + b);

    if (b == '\n') {
      g_veDiag.lines++;
      g_veLine[g_veLineLen] = '\0';
      // The Checksum line's value byte is binary; it is already in the running sum.
      const bool isChecksumLine = strncmp(g_veLine, "Checksum", 8) == 0;
      if (g_veRawEcho) {
        Serial.printf("[VE] %s\n", isChecksumLine ? "Checksum" : g_veLine);
      }
      handleVeLine(g_veLine);
      g_veLineLen = 0;
      if (isChecksumLine) {
        g_veChecksum = 0;
      }
      continue;
    }

    if (b == '\r') {
      continue;
    }

    if (g_veLineLen < (sizeof(g_veLine) - 1)) {
      g_veLine[g_veLineLen++] = static_cast<char>(b);
    }
  }
}

// ------------------------------------------------------------------ summary
static SummaryView buildSummaryView() {
  SummaryView view;
  view.uartCount = g_commPowerOn ? g_uartDetectedCount : 0;
  view.socValid = veFieldFresh(g_veLive.socMs);
  view.currentValid = veFieldFresh(g_veLive.currentMs);
  view.batteryValid = veFieldFresh(g_veLive.batteryMs);
  view.socDeciPct = view.socValid ? g_veLive.socDeciPct : -1;
  view.currentMa = view.currentValid ? g_veLive.currentMa : 0;
  view.batteryMv = view.batteryValid ? g_veLive.batteryMv : -1;

  uint32_t totalModuleMv = 0;
  uint32_t tempSumDeciF = 0;
  uint16_t tempCount = 0;
  bool haveModule = false;
  bool haveTemp = false;

  if (!g_cmuActive) {
    return view;
  }

  for (uint8_t moduleNo = 1; moduleNo <= kMaxModules; ++moduleNo) {
    const ModuleData* module = g_cmu.module(moduleNo);
    if (!module || !module->discovered || !module->online) {
      continue;
    }

    ++view.canCount;

    float totalV = NAN;
    float avgCellV = NAN;
    uint16_t deltaMv = 0;
    float avgTempF = NAN;
    uint8_t validCells = 0;
    uint8_t validTemps = 0;
    if (g_cmu.moduleMetrics(moduleNo, totalV, avgCellV, deltaMv, avgTempF, validCells, validTemps)) {
      const int32_t totalMv = static_cast<int32_t>(lroundf(totalV * 1000.0f));
      totalModuleMv += static_cast<uint32_t>(totalMv);
      if (!haveModule) {
        view.moduleHighMv = totalMv;
        view.moduleLowMv = totalMv;
        haveModule = true;
      } else {
        view.moduleHighMv = max(view.moduleHighMv, totalMv);
        view.moduleLowMv = min(view.moduleLowMv, totalMv);
      }
      if (deltaMv > view.maxDeltaMv) {
        view.maxDeltaMv = deltaMv;
      }
    }

    for (uint8_t i = 0; i < kTempsPerModule; ++i) {
      if (!module->tempValid[i] || isnan(module->temps_C[i])) {
        continue;
      }
      const int32_t tempDeciF = static_cast<int32_t>(lroundf(SCIBCMUComs::Manager::cToF(module->temps_C[i]) * 10.0f));
      tempSumDeciF += static_cast<uint32_t>(tempDeciF);
      tempCount++;
      if (!haveTemp) {
        view.tempHighDeciF = tempDeciF;
        view.tempLowDeciF = tempDeciF;
        haveTemp = true;
      } else {
        view.tempHighDeciF = max(view.tempHighDeciF, tempDeciF);
        view.tempLowDeciF = min(view.tempLowDeciF, tempDeciF);
      }
    }
  }

  if (view.canCount > 0 && haveModule) {
    view.moduleAvgMv = static_cast<int32_t>(totalModuleMv / view.canCount);
  }
  if (tempCount > 0) {
    view.tempAvgDeciF = static_cast<int32_t>(tempSumDeciF / tempCount);
  }

  return view;
}

// ------------------------------------------------------------------ display
static const int16_t kScreenW = 320;
static const int16_t kMainLineH = 26;
static const int16_t kDebugLineH = 16;

// Each line is drawn into a one-line sprite and pushed, so updates don't flicker.
static void drawLine(int16_t y, int16_t h, uint16_t bg, const lgfx::IFont* font, uint16_t color, const char* fmt, ...) {
  char buf[96];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  if (g_line.height() != h && !g_line.createSprite(kScreenW, h)) {
    return;
  }
  g_line.fillScreen(bg);
  g_line.setFont(font);
  g_line.setTextColor(color, bg);
  g_line.setCursor(4, (h - g_line.fontHeight()) / 2);
  g_line.print(buf);
  const int32_t width = 4 + g_line.textWidth(buf);
  static int16_t s_overflowLoggedY = -1;
  if (width > kScreenW && s_overflowLoggedY != y) {
    s_overflowLoggedY = y;
    Serial.printf("[BMS] line y=%d is %ld px wide, clipped at %d: %s\n", y, static_cast<long>(width), kScreenW, buf);
  }
  g_line.pushSprite(0, y);
}

static void fmtTempF(char* out, size_t len, float c) {
  if (isnan(c)) {
    snprintf(out, len, "--");
  } else {
    snprintf(out, len, "%.1fF", cToF(c));
  }
}

// Main page: one coloured section per device. The tag column on the left names the device, and that device's
// values are drawn in the same colour.
static const int16_t kRowH = 22;  // 10 rows + a 20 px status line
static const int16_t kTagW = 58;
static const int16_t kValueX = kTagW + 6;
static const uint16_t kColorCmu = TFT_CYAN;
static const uint16_t kColorShunt = TFT_YELLOW;
static const uint16_t kColorBoard = TFT_GREENYELLOW;
static const uint16_t kColorCtrl = TFT_ORANGE;

// One row: the tag strip (with its label on the section's first row), then text parts left to right.
static void drawRow(int16_t y, uint16_t tagColor, const char* tag, const RowPart* parts, uint8_t count) {
  if (g_line.height() != kRowH && !g_line.createSprite(kScreenW, kRowH)) {
    return;
  }
  g_line.fillScreen(COLOR_BG);
  g_line.fillRect(0, 0, kTagW, kRowH, tagColor);
  if (tag) {
    g_line.setFont(&fonts::Font2);
    g_line.setTextColor(COLOR_BG, tagColor);
    g_line.setTextDatum(lgfx::middle_center);
    g_line.drawString(tag, kTagW / 2, kRowH / 2);
  }
  g_line.setFont(&fonts::FreeSansBold9pt7b);
  g_line.setTextDatum(lgfx::middle_left);
  int16_t x = kValueX;
  for (uint8_t i = 0; i < count; ++i) {
    g_line.setTextColor(parts[i].color, COLOR_BG);
    x += g_line.drawString(parts[i].text, x, kRowH / 2);
  }
  static int16_t s_overflowLoggedY = -1;
  if (x > kScreenW && s_overflowLoggedY != y) {
    s_overflowLoggedY = y;
    Serial.printf("[BMS] main row y=%d is %d px wide, clipped at %d\n", y, x, kScreenW);
  }
  g_line.setTextDatum(lgfx::top_left);
  g_line.pushSprite(0, y);
}

static void drawRow1(int16_t y, uint16_t tagColor, const char* tag, uint16_t color, const char* fmt, ...) {
  char buf[64];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  const RowPart part = {color, buf};
  drawRow(y, tagColor, tag, &part, 1);
}

static void fmtTriple(char* out, size_t len, int32_t hi, int32_t avg, int32_t lo, float scale, const char* numFmt) {
  if (hi < 0 || avg < 0 || lo < 0) {
    snprintf(out, len, "--");
    return;
  }
  char fmt[32];
  snprintf(fmt, sizeof(fmt), "%s/%s/%s", numFmt, numFmt, numFmt);
  snprintf(out, len, fmt, hi / scale, avg / scale, lo / scale);
}

static void drawMainPage() {
  const SummaryView v = buildSummaryView();
  int16_t y = 0;
  char a[40];
  char b[24];
  char c[24];

  // CMUs: count from the naming reply, count live on CAN, module voltage and temperature hi/avg/lo, cell spread.
  const bool cmuOk = g_commPowerOn && v.canCount > 0 && v.canCount == v.uartCount;
  if (g_commPowerOn) {
    drawRow1(y, kColorCmu, "CMUs", cmuOk ? kColorCmu : COLOR_WARN, "Named %u   On CAN %u",
             static_cast<unsigned>(v.uartCount), static_cast<unsigned>(v.canCount));
  } else {
    drawRow1(y, kColorCmu, "CMUs", COLOR_DIM, "power off");
  }
  y += kRowH;
  fmtTriple(a, sizeof(a), v.moduleHighMv, v.moduleAvgMv, v.moduleLowMv, 1000.0f, "%.2f");
  drawRow1(y, kColorCmu, nullptr, kColorCmu, "Mod V  %s", a);
  y += kRowH;
  fmtTriple(a, sizeof(a), v.tempHighDeciF, v.tempAvgDeciF, v.tempLowDeciF, 10.0f, "%.1f");
  drawRow1(y, kColorCmu, nullptr, kColorCmu, "Temp F  %s", a);
  y += kRowH;
  if (v.canCount > 0) {
    drawRow1(y, kColorCmu, nullptr, kColorCmu, "Spread %u mV     hi/avg/lo", static_cast<unsigned>(v.maxDeltaMv));
  } else {
    drawRow1(y, kColorCmu, nullptr, COLOR_DIM, "hi/avg/lo");
  }
  y += kRowH;

  // Shunt: pack voltage, current (+ = charging), state of charge.
  snprintf(a, sizeof(a), v.batteryValid ? "%.2f V" : "-- V", v.batteryMv / 1000.0f);
  snprintf(b, sizeof(b), v.currentValid ? "%+.2f A" : "-- A", v.currentMa / 1000.0f);
  {
    const RowPart parts[] = {{v.batteryValid ? kColorShunt : COLOR_DIM, a},
                             {COLOR_DIM, "    "},
                             {v.currentValid ? (v.currentMa >= 0 ? COLOR_GOOD : COLOR_WARN) : COLOR_DIM, b}};
    drawRow(y, kColorShunt, "SHUNT", parts, 3);
  }
  y += kRowH;
  if (v.socValid) {
    drawRow1(y, kColorShunt, nullptr, kColorShunt, "SOC  %.1f %%", v.socDeciPct / 10.0f);
  } else {
    drawRow1(y, kColorShunt, nullptr, COLOR_DIM, veStreamActive() ? "SOC  -- (not synced)" : "no VE.Direct data");
  }
  y += kRowH;

  // Board: its own probes and fans.
  fmtTempF(b, sizeof(b), g_probes[0].tempC);
  fmtTempF(c, sizeof(c), g_probes[1].tempC);
  drawRow1(y, kColorBoard, "BOARD", kColorBoard, "Batt %s   Air %s", b, c);
  y += kRowH;
  drawRow1(y, kColorBoard, nullptr, kColorBoard, "Fan %u/%u%%   %u/%u rpm", static_cast<unsigned>(g_fans[0].pct),
           static_cast<unsigned>(g_fans[1].pct), static_cast<unsigned>(g_fans[0].rpm),
           static_cast<unsigned>(g_fans[1].rpm));
  y += kRowH;
  if (g_cmuIsFault) {
    drawRow1(y, kColorBoard, nullptr, COLOR_BAD, "CMU 12V FAULT   in %.1f V", g_vmonMv / 1000.0f);
  } else {
    drawRow1(y, kColorBoard, nullptr, kColorBoard, "CMU 12V %u mA   In %.1f V", static_cast<unsigned>(g_cmuMa),
             g_vmonMv / 1000.0f);
  }
  y += kRowH;

  // Control: the two outputs, green when on.
  {
    const RowPart parts[] = {{g_commPowerOn ? COLOR_GOOD : COLOR_DIM, g_commPowerOn ? "CMU ON" : "CMU off"},
                             {COLOR_DIM, "    "},
                             {g_contactorEnabled ? COLOR_GOOD : COLOR_DIM, g_contactorEnabled ? "CONT ON" : "CONT off"},
                             {COLOR_DIM, "   "},
                             {g_config.balanceAllEnabled ? kColorCtrl : COLOR_DIM, g_config.balanceAllEnabled ? "BAL" : ""}};
    drawRow(y, kColorCtrl, "CTRL", parts, 5);
  }
  y += kRowH;

  const bool showStatus = g_statusUntilMs == 0 || millis() <= g_statusUntilMs;
  drawLine(y, 240 - y, COLOR_BG, &fonts::Font2, showStatus ? COLOR_TEXT : COLOR_DIM, "%s",
           showStatus ? g_statusLine : "A: wake/hold debug   B: CMU pwr   C: contactor");
}

static void drawDebugPage() {
  const lgfx::IFont* f = &fonts::Font2;
  int16_t y = 0;
  char a[24];
  char b[24];

  drawLine(y, kDebugLineH, COLOR_ACCENT, f, COLOR_BG, "DEBUG  A:next  B:-/off  C:+/on  hold A:exit");
  y += kDebugLineH;

  for (uint8_t i = 0; i < kItemCount; ++i) {
    const bool sel = i == g_debugCursor;
    const uint16_t bg = sel ? COLOR_SELECT : COLOR_BG;
    const char* mark = sel ? ">" : " ";
    switch (i) {
      case kItemCmuPower:
        if (g_cmuIsFault) {
          drawLine(y, kDebugLineH, bg, f, COLOR_BAD, "%s CMU 12V power    %s   FAULT (%umV)", mark,
                   g_commPowerOn ? "ON" : "OFF", static_cast<unsigned>(g_cmuIsMv));
        } else {
          drawLine(y, kDebugLineH, bg, f, g_commPowerOn ? COLOR_GOOD : COLOR_TEXT, "%s CMU 12V power    %s   %umA (%umV)",
                   mark, g_commPowerOn ? "ON" : "OFF", static_cast<unsigned>(g_cmuMa), static_cast<unsigned>(g_cmuIsMv));
        }
        break;
      case kItemNaming:
        drawLine(y, kDebugLineH, bg, f, COLOR_TEXT, "%s Rerun naming (C)  last: %s, %u CMUs, arm %u", mark,
                 g_lastNaming.replyValid ? "reply" : (g_lastNaming.attemptsRun ? "no reply" : "-"),
                 static_cast<unsigned>(g_uartDetectedCount), static_cast<unsigned>(g_lastNaming.armFrames));
        break;
      case kItemEms:
        drawLine(y, kDebugLineH, bg, f, g_contactorEnabled ? COLOR_GOOD : COLOR_TEXT, "%s EMS300 enable     %s   LED %s",
                 mark, g_contactorEnabled ? "ON" : "OFF", g_emsLedOn ? "on" : "off");
        break;
      case kItemFan1:
        drawLine(y, kDebugLineH, bg, f, COLOR_TEXT, "%s Fan 1              %3u%%  %u rpm", mark,
                 static_cast<unsigned>(g_fans[0].pct), static_cast<unsigned>(g_fans[0].rpm));
        break;
      case kItemFan2:
        drawLine(y, kDebugLineH, bg, f, COLOR_TEXT, "%s Fan 2              %3u%%  %u rpm", mark,
                 static_cast<unsigned>(g_fans[1].pct), static_cast<unsigned>(g_fans[1].rpm));
        break;
      case kItemFanBoth:
        drawLine(y, kDebugLineH, bg, f, COLOR_TEXT, "%s Both fans (B off, C 100%%)", mark);
        break;
      case kItemBalance:
        drawLine(y, kDebugLineH, bg, f, COLOR_TEXT, "%s Balance all        %s   fam %s", mark,
                 g_config.balanceAllEnabled ? "ON" : "OFF", familyLabel(g_config.family));
        break;
      case kItemBacklight:
        drawLine(y, kDebugLineH, bg, f, COLOR_TEXT, "%s Backlight          %3u%%", mark,
                 static_cast<unsigned>(g_config.backlightPct));
        break;
      case kItemFlip:
        drawLine(y, kDebugLineH, bg, f, COLOR_TEXT, "%s Flip screen 180    %s", mark,
                 g_config.rotation == 3 ? "FLIPPED" : "normal");
        break;
    }
    y += kDebugLineH;
  }

  for (uint8_t i = 0; i < 2; ++i) {
    const TempProbe& p = g_probes[i];
    fmtTempF(a, sizeof(a), p.tempC);
    snprintf(b, sizeof(b), isnan(p.tempC) ? "--" : "%.2fC", p.tempC);
    drawLine(y, kDebugLineH, COLOR_BG, f, COLOR_ACCENT, "T%u %-4s %-4s  %s  %s   pin %umV crc %u", i + 1, p.name,
             probeKindLabel(p.kind), b, a, static_cast<unsigned>(p.ntcMv), static_cast<unsigned>(p.crcErrors));
    y += kDebugLineH;
  }

  if (g_cmuActive) {
    const SCIBCMUComs::BusStats& s = g_cmu.stats();
    drawLine(y, kDebugLineH, COLOR_BG, f, COLOR_TEXT, "CAN rx %lu tx %lu err %lu  found %u", static_cast<unsigned long>(s.rxFrames),
             static_cast<unsigned long>(s.txFrames), static_cast<unsigned long>(s.txErrors),
             static_cast<unsigned>(g_cmu.discoveredCount()));
  } else {
    drawLine(y, kDebugLineH, COLOR_BG, f, COLOR_DIM, "CAN manager stopped");
  }
  y += kDebugLineH;

  drawLine(y, kDebugLineH, COLOR_BG, f, veStreamActive() ? COLOR_TEXT : COLOR_DIM, "VE good %lu bad %lu hex %lu    24V in %.2fV",
           static_cast<unsigned long>(g_veDiag.goodFrames), static_cast<unsigned long>(g_veDiag.badFrames),
           static_cast<unsigned long>(g_veDiag.hexFrames), g_vmonMv / 1000.0f);
  y += kDebugLineH;

  drawLine(y, 240 - y, COLOR_BG, f, COLOR_TEXT, "NOW ch%u tx %lu fail %lu rx %lu   up %lus", kEspNowChannel,
           static_cast<unsigned long>(g_espNowTx), static_cast<unsigned long>(g_espNowTxFail),
           static_cast<unsigned long>(g_espNowRx), static_cast<unsigned long>(millis() / 1000));
}

static void setRotation(uint8_t rotation) {
  g_config.rotation = rotation == 3 ? 3 : 1;
  saveConfig();
  g_lcd.setRotation(g_config.rotation);
  g_lcd.fillScreen(COLOR_BG);
}

static void drawScreen() {
  if (!g_screenOn) {
    return;
  }
  if (g_page == Page::Main) {
    drawMainPage();
  } else {
    drawDebugPage();
  }
}

static void initDisplay() {
  g_lcd.init();
  g_lcd.setRotation(g_config.rotation);
  g_lcd.fillScreen(COLOR_BG);
  g_line.setColorDepth(16);
  if (!g_line.createSprite(kScreenW, kMainLineH)) {
    Serial.println("[BMS] line sprite alloc failed");
  }
  g_screenOn = true;
  applyBacklight();
}

// ------------------------------------------------------------------ buttons
static void pollButton(Button& b, uint32_t now) {
  b.clicked = false;
  b.longPressed = false;
  const bool raw = digitalRead(b.pin) == LOW;  // 10k pull-up, pressed = low
  if (raw != b.lastRaw) {
    b.lastRaw = raw;
    b.changedMs = now;
  }
  if ((now - b.changedMs) < 30 || raw == b.stableDown) {
    if (b.stableDown && !b.longFired && (now - b.downSinceMs) >= kLongPressMs) {
      b.longFired = true;
      b.longPressed = true;
    }
    return;
  }
  b.stableDown = raw;
  if (raw) {
    b.downSinceMs = now;
    b.longFired = false;
  } else if (!b.longFired) {
    b.clicked = true;
  }
}

static void debugAdjust(int dir) {
  switch (g_debugCursor) {
    case kItemCmuPower:
      setCommPower(dir > 0);
      break;
    case kItemNaming:
      if (dir > 0) {
        if (g_commPowerOn) {
          runNamingNow();
        } else {
          setStatus("Naming needs CMU power");
        }
      }
      break;
    case kItemEms:
      setContactorEnabled(dir > 0);
      break;
    case kItemFan1:
      setFanPct(0, g_fans[0].pct + dir * 10);
      break;
    case kItemFan2:
      setFanPct(1, g_fans[1].pct + dir * 10);
      break;
    case kItemFanBoth:
      setFanPct(0, dir > 0 ? 100 : 0);
      setFanPct(1, dir > 0 ? 100 : 0);
      break;
    case kItemBalance:
      g_config.balanceAllEnabled = dir > 0;
      saveConfig();
      if (g_cmuActive) {
        g_cmu.setBalanceAllEnabled(g_config.balanceAllEnabled);
      }
      break;
    case kItemBacklight:
      g_config.backlightPct = static_cast<uint8_t>(constrain(g_config.backlightPct + dir * 10, 10, 100));
      saveConfig();
      applyBacklight();
      break;
    case kItemFlip:
      setRotation(dir > 0 ? 3 : 1);
      break;
  }
}

static void handleButtons() {
  const uint32_t now = millis();
  pollButton(g_btnA, now);
  pollButton(g_btnB, now);
  pollButton(g_btnC, now);

  const bool any = g_btnA.clicked || g_btnB.clicked || g_btnC.clicked || g_btnA.longPressed;
  if (!any) {
    return;
  }
  // While the screen is dark, a press only wakes it, so nothing is switched blind.
  if (!g_screenOn) {
    wakeScreen();
    return;
  }
  wakeScreen();

  if (g_btnA.longPressed) {
    g_page = (g_page == Page::Main) ? Page::Debug : Page::Main;
    g_lcd.fillScreen(COLOR_BG);
    return;
  }

  if (g_page == Page::Main) {
    if (g_btnB.clicked) {
      setCommPower(!g_commPowerOn);
    } else if (g_btnC.clicked) {
      setContactorEnabled(!g_contactorEnabled);
    }
    return;
  }

  if (g_btnA.clicked) {
    g_debugCursor = static_cast<uint8_t>((g_debugCursor + 1) % kItemCount);
  } else if (g_btnB.clicked) {
    debugAdjust(-1);
  } else if (g_btnC.clicked) {
    debugAdjust(+1);
  }
}

// ------------------------------------------------------------------ ESP-NOW and serial commands
static const uint8_t kBroadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static bool appendFormat(char* dst, size_t len, size_t& used, const char* fmt, ...) {
  if (used >= len) {
    return false;
  }
  va_list args;
  va_start(args, fmt);
  const int written = vsnprintf(dst + used, len - used, fmt, args);
  va_end(args);
  if (written < 0) {
    return false;
  }
  if (static_cast<size_t>(written) >= (len - used)) {
    used = len;
    return false;
  }
  used += static_cast<size_t>(written);
  return true;
}

static bool ensurePeer(const uint8_t mac[6]) {
  if (esp_now_is_peer_exist(mac)) {
    return true;
  }
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = kEspNowChannel;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

static void sendNow(const uint8_t mac[6], const char* payload) {
  if (!g_espNowReady) {
    return;
  }
  const size_t len = strlen(payload);
  if (len > kEspNowMaxLen) {
    Serial.printf("[BMS] NOW: payload too long (%u)\n", static_cast<unsigned>(len));
    return;
  }
  if (!ensurePeer(mac) || esp_now_send(mac, reinterpret_cast<const uint8_t*>(payload), len) != ESP_OK) {
    g_espNowTxFail++;
    return;
  }
  g_espNowTx++;
}

static void sendReply(const ReplyTarget& to, const char* payload) {
  if (to.espnow) {
    sendNow(to.mac, payload);
  } else {
    Serial.println(payload);
  }
}

static void sendAck(const ReplyTarget& to, const char* cmd, bool ok, const char* msg) {
  char payload[192];
  snprintf(payload, sizeof(payload), "{\"type\":\"ack\",\"cmd\":\"%s\",\"ok\":%u,\"msg\":\"%s\"}", cmd, ok ? 1U : 0U,
           msg ? msg : "");
  sendReply(to, payload);
}

// Two packets, each under the 250-byte ESP-NOW limit.
static bool buildStatusJson(char* out, size_t len) {
  const SummaryView v = buildSummaryView();
  size_t used = 0;
  return appendFormat(out, len, used,
                      "{\"type\":\"status\",\"cmus_uart\":%u,\"cmus_can\":%u,\"v_hi_mv\":%ld,\"v_avg_mv\":%ld,"
                      "\"v_lo_mv\":%ld,\"t_hi_df\":%ld,\"t_avg_df\":%ld,\"t_lo_df\":%ld,\"delta_max_mv\":%u,"
                      "\"soc_dpct\":%ld,\"cur_ma\":%ld,\"batt_mv\":%ld}",
                      static_cast<unsigned>(v.uartCount), static_cast<unsigned>(v.canCount),
                      static_cast<long>(v.moduleHighMv), static_cast<long>(v.moduleAvgMv),
                      static_cast<long>(v.moduleLowMv), static_cast<long>(v.tempHighDeciF),
                      static_cast<long>(v.tempAvgDeciF), static_cast<long>(v.tempLowDeciF),
                      static_cast<unsigned>(v.maxDeltaMv), static_cast<long>(v.socDeciPct),
                      static_cast<long>(v.currentValid ? v.currentMa : 0), static_cast<long>(v.batteryMv));
}

static long probeDeciC(const TempProbe& p) {
  return isnan(p.tempC) ? -1000L : static_cast<long>(lroundf(p.tempC * 10.0f));
}

static bool buildBoardJson(char* out, size_t len) {
  char status[33];
  strlcpy(status, g_statusLine, sizeof(status));
  for (char* p = status; *p; ++p) {
    if (*p == '"' || *p == '\\') {
      *p = '\'';
    }
  }
  size_t used = 0;
  return appendFormat(out, len, used,
                      "{\"type\":\"board\",\"power\":%u,\"contactor\":%u,\"ems_led\":%u,\"bal_all\":%u,\"family\":\"%s\","
                      "\"t_batt_dc\":%ld,\"t_air_dc\":%ld,\"fan_pct\":[%u,%u],\"fan_rpm\":[%u,%u],\"v24_mv\":%u,"
                      "\"status\":\"%s\"}",
                      g_commPowerOn ? 1U : 0U, g_contactorEnabled ? 1U : 0U, g_emsLedOn ? 1U : 0U,
                      g_config.balanceAllEnabled ? 1U : 0U, familyLabel(g_config.family), probeDeciC(g_probes[0]),
                      probeDeciC(g_probes[1]), static_cast<unsigned>(g_fans[0].pct), static_cast<unsigned>(g_fans[1].pct),
                      static_cast<unsigned>(g_fans[0].rpm), static_cast<unsigned>(g_fans[1].rpm),
                      static_cast<unsigned>(g_vmonMv), status);
}

static void sendStatusTo(const ReplyTarget& to) {
  char payload[kEspNowMaxLen + 1];
  if (buildStatusJson(payload, sizeof(payload))) {
    sendReply(to, payload);
  }
  if (buildBoardJson(payload, sizeof(payload))) {
    sendReply(to, payload);
  }
}

static void sendModuleTo(const ReplyTarget& to, uint8_t moduleNo) {
  const ModuleData* module = g_cmuActive ? g_cmu.module(moduleNo) : nullptr;
  if (!module || !module->discovered) {
    sendAck(to, "MODULE", false, "module_not_found");
    return;
  }
  float totalV = NAN;
  float avgCellV = NAN;
  uint16_t deltaMv = 0;
  float avgTempF = NAN;
  uint8_t validCells = 0;
  uint8_t validTemps = 0;
  const bool haveMetrics = g_cmu.moduleMetrics(moduleNo, totalV, avgCellV, deltaMv, avgTempF, validCells, validTemps);
  const long totalMv = haveMetrics ? static_cast<long>(lroundf(totalV * 1000.0f)) : -1;
  const long avgTempDeciF = (haveMetrics && !isnan(avgTempF)) ? static_cast<long>(lroundf(avgTempF * 10.0f)) : -1;

  char payload[kEspNowMaxLen + 1];
  size_t used = 0;
  bool ok = appendFormat(payload, sizeof(payload), used,
                         "{\"type\":\"module\",\"slot\":%u,\"can\":%u,\"online\":%u,\"status_word\":%u,\"total_mv\":%ld,"
                         "\"delta_mv\":%u,\"cells_mv\":[",
                         static_cast<unsigned>(moduleNo), static_cast<unsigned>(module->canModuleNo),
                         module->online ? 1U : 0U, static_cast<unsigned>(module->statusWord), totalMv,
                         static_cast<unsigned>(deltaMv));
  for (uint8_t i = 0; ok && i < kCellsPerModule; ++i) {
    ok = appendFormat(payload, sizeof(payload), used, "%s%d", i ? "," : "", module->cellValid[i] ? module->cells_mV[i] : -1);
  }
  ok = ok && appendFormat(payload, sizeof(payload), used, "]}");
  if (ok) {
    sendReply(to, payload);
  }

  used = 0;
  ok = appendFormat(payload, sizeof(payload), used, "{\"type\":\"module_t\",\"slot\":%u,\"avg_temp_df\":%ld,\"temps_df\":[",
                    static_cast<unsigned>(moduleNo), avgTempDeciF);
  for (uint8_t i = 0; ok && i < kTempsPerModule; ++i) {
    const int tempDeciF = (module->tempValid[i] && !isnan(module->temps_C[i]))
                              ? static_cast<int>(lroundf(SCIBCMUComs::Manager::cToF(module->temps_C[i]) * 10.0f))
                              : -1;
    ok = appendFormat(payload, sizeof(payload), used, "%s%d", i ? "," : "", tempDeciF);
  }
  ok = ok && appendFormat(payload, sizeof(payload), used, "]}");
  if (ok) {
    sendReply(to, payload);
  }
}

static void processCommand(char* command, const ReplyTarget& to) {
  for (char* p = command; *p; ++p) {
    if (*p == '\r' || *p == '\n') {
      *p = '\0';
      break;
    }
  }

  char* savePtr = nullptr;
  char* verb = strtok_r(command, " \t", &savePtr);
  if (!verb) {
    return;
  }

  if (strcasecmp(verb, "STATUS") == 0 || strcasecmp(verb, "GET") == 0) {
    sendStatusTo(to);
    return;
  }

  if (strcasecmp(verb, "MODULE") == 0) {
    const char* token = strtok_r(nullptr, " \t", &savePtr);
    const int moduleNo = token ? atoi(token) : 0;
    if (moduleNo < 1 || moduleNo > kMaxModules) {
      sendAck(to, "MODULE", false, "use_1_to_16");
      return;
    }
    sendModuleTo(to, static_cast<uint8_t>(moduleNo));
    return;
  }

  if (strcasecmp(verb, "POWER") == 0) {
    bool value = false;
    if (!isTruthyToken(strtok_r(nullptr, " \t", &savePtr), g_commPowerOn, value)) {
      sendAck(to, "POWER", false, "use_on_off_toggle");
      return;
    }
    setCommPower(value);
    sendAck(to, "POWER", true, value ? "on" : "off");
    sendStatusTo(to);
    return;
  }

  if (strcasecmp(verb, "CONTACTOR") == 0 || strcasecmp(verb, "EMS") == 0) {
    bool value = false;
    if (!isTruthyToken(strtok_r(nullptr, " \t", &savePtr), g_contactorEnabled, value)) {
      sendAck(to, "CONTACTOR", false, "use_on_off_toggle");
      return;
    }
    setContactorEnabled(value);
    sendAck(to, "CONTACTOR", true, value ? "on" : "off");
    sendStatusTo(to);
    return;
  }

  if (strcasecmp(verb, "FAN") == 0) {
    const char* which = strtok_r(nullptr, " \t", &savePtr);
    const char* pctTok = strtok_r(nullptr, " \t", &savePtr);
    if (!which || !pctTok) {
      sendAck(to, "FAN", false, "use_FAN_1|2|ALL_pct");
      return;
    }
    const int pct = constrain(atoi(pctTok), 0, 100);
    if (strcasecmp(which, "ALL") == 0) {
      setFanPct(0, pct);
      setFanPct(1, pct);
    } else if (atoi(which) == 1 || atoi(which) == 2) {
      setFanPct(static_cast<uint8_t>(atoi(which) - 1), pct);
    } else {
      sendAck(to, "FAN", false, "use_FAN_1|2|ALL_pct");
      return;
    }
    sendAck(to, "FAN", true, "ok");
    return;
  }

  if (strcasecmp(verb, "BAL") == 0 || strcasecmp(verb, "BALANCE") == 0) {
    bool value = false;
    if (!isTruthyToken(strtok_r(nullptr, " \t", &savePtr), g_config.balanceAllEnabled, value)) {
      sendAck(to, "BAL", false, "use_on_off_toggle");
      return;
    }
    g_config.balanceAllEnabled = value;
    saveConfig();
    if (g_cmuActive) {
      g_cmu.setBalanceAllEnabled(value);
    }
    setStatus("Balance all %s", value ? "ON" : "OFF");
    sendAck(to, "BAL", true, value ? "on" : "off");
    return;
  }

  if (strcasecmp(verb, "KEEP") == 0) {
    bool value = false;
    if (!isTruthyToken(strtok_r(nullptr, " \t", &savePtr), g_config.keepaliveEnabled, value)) {
      sendAck(to, "KEEP", false, "use_on_off_toggle");
      return;
    }
    g_config.keepaliveEnabled = value;
    saveConfig();
    if (g_cmuActive) {
      g_cmu.setKeepaliveEnabled(value);
    }
    setStatus("Keepalive %s", value ? "ON" : "OFF");
    sendAck(to, "KEEP", true, value ? "on" : "off");
    return;
  }

  if (strcasecmp(verb, "FAMILY") == 0) {
    const char* token = strtok_r(nullptr, " \t", &savePtr);
    if (!token) {
      sendAck(to, "FAMILY", false, "missing_auto_09_0A");
      return;
    }
    g_config.family = familyFromToken(token);
    saveConfig();
    if (g_cmuActive) {
      g_cmu.setCommandFamily(g_config.family);
    }
    setStatus("Family %s", familyLabel(g_config.family));
    sendAck(to, "FAMILY", true, familyLabel(g_config.family));
    return;
  }

  if (strcasecmp(verb, "TEMPOFFSET") == 0) {
    const char* token = strtok_r(nullptr, " \t", &savePtr);
    if (!token) {
      sendAck(to, "TEMPOFFSET", false, "missing_value");
      return;
    }
    g_config.tempOffsetC = strtof(token, nullptr);
    saveConfig();
    if (g_cmuActive) {
      g_cmu.setTempOffsetC(g_config.tempOffsetC);
    }
    setStatus("Temp offset %.1fC", g_config.tempOffsetC);
    sendAck(to, "TEMPOFFSET", true, "ok");
    return;
  }

  if (strcasecmp(verb, "NAME") == 0) {
    if (!g_commPowerOn) {
      sendAck(to, "NAME", false, "power_off");
      return;
    }
    runNamingNow();
    sendAck(to, "NAME", g_lastNaming.ok, g_lastNaming.replyValid ? "reply_ok" : "no_reply");
    sendStatusTo(to);
    return;
  }

  if (strcasecmp(verb, "SCREEN") == 0) {
    bool value = false;
    if (!isTruthyToken(strtok_r(nullptr, " \t", &savePtr), g_screenOn, value)) {
      sendAck(to, "SCREEN", false, "use_on_off_toggle");
      return;
    }
    if (value) {
      wakeScreen();
    } else {
      sleepScreen();
    }
    sendAck(to, "SCREEN", true, value ? "on" : "off");
    return;
  }

  if (strcasecmp(verb, "ROTATE") == 0 || strcasecmp(verb, "FLIP") == 0) {
    bool flipped = g_config.rotation == 3;
    const char* token = strtok_r(nullptr, " \t", &savePtr);
    if (!token) {
      flipped = !flipped;
    } else if (!isTruthyToken(token, flipped, flipped)) {
      sendAck(to, "FLIP", false, "use_on_off_toggle");
      return;
    }
    setRotation(flipped ? 3 : 1);
    sendAck(to, "FLIP", true, flipped ? "flipped" : "normal");
    return;
  }

  if (strcasecmp(verb, "PAGE") == 0) {
    const char* token = strtok_r(nullptr, " \t", &savePtr);
    g_page = (token && strcasecmp(token, "DEBUG") == 0) ? Page::Debug : Page::Main;
    wakeScreen();
    g_lcd.fillScreen(COLOR_BG);
    sendAck(to, "PAGE", true, g_page == Page::Debug ? "debug" : "main");
    return;
  }

  if (strcasecmp(verb, "TACH") == 0 && !to.espnow) {
    const char* token = strtok_r(nullptr, " \t", &savePtr);
    if (token) {
      g_tachLockoutUs = static_cast<uint32_t>(atol(token));
    }
    for (uint8_t i = 0; i < 2; ++i) {
      Fan& f = g_fans[i];
      Serial.printf("[TACH] fan%u pct=%u rpm=%u raw_rpm=%u lockout=%luus high_at_isr+<100us=%lu <1ms=%lu <lockout=%lu ok=%lu\n",
                    i + 1, f.pct, f.rpm, f.rawRpm, static_cast<unsigned long>(g_tachLockoutUs),
                    static_cast<unsigned long>(f.hist[0]), static_cast<unsigned long>(f.hist[1]),
                    static_cast<unsigned long>(f.hist[2]), static_cast<unsigned long>(f.hist[3]));
      for (uint8_t b = 0; b < 4; ++b) {
        f.hist[b] = 0;
      }
      uint32_t sorted[64];
      for (uint8_t k = 0; k < 64; ++k) {
        sorted[k] = f.ring[k];
      }
      std::sort(sorted, sorted + 64);
      Serial.printf("[TACH] fan%u spacing us: min=%lu p10=%lu median=%lu p90=%lu max=%lu\n", i + 1,
                    static_cast<unsigned long>(sorted[0]), static_cast<unsigned long>(sorted[6]),
                    static_cast<unsigned long>(sorted[32]), static_cast<unsigned long>(sorted[57]),
                    static_cast<unsigned long>(sorted[63]));
    }
    return;
  }

  if (strcasecmp(verb, "VE") == 0 && !to.espnow) {
    const char* token = strtok_r(nullptr, " \t", &savePtr);
    if (token && strcasecmp(token, "RAW") == 0) {
      token = strtok_r(nullptr, " \t", &savePtr);
      bool value = !g_veRawEcho;
      if (!token || isTruthyToken(token, g_veRawEcho, value)) {
        g_veRawEcho = value;
      }
    }
    const uint32_t now = millis();
    Serial.printf("[VE] bytes=%lu lines=%lu good=%lu bad=%lu hex=%lu raw=%u | V=%ldmV age=%ldms I=%ldmA age=%ldms "
                  "SOC=%ld age=%ldms last=\"%s\"\n",
                  static_cast<unsigned long>(g_veDiag.bytes), static_cast<unsigned long>(g_veDiag.lines),
                  static_cast<unsigned long>(g_veDiag.goodFrames), static_cast<unsigned long>(g_veDiag.badFrames),
                  static_cast<unsigned long>(g_veDiag.hexFrames), g_veRawEcho, static_cast<long>(g_veLive.batteryMv),
                  g_veLive.batteryMs ? static_cast<long>(now - g_veLive.batteryMs) : -1L,
                  static_cast<long>(g_veLive.currentMa),
                  g_veLive.currentMs ? static_cast<long>(now - g_veLive.currentMs) : -1L,
                  static_cast<long>(g_veLive.socDeciPct), g_veLive.socMs ? static_cast<long>(now - g_veLive.socMs) : -1L,
                  g_veDiag.lastLine);
    return;
  }

  if (strcasecmp(verb, "HELP") == 0 && !to.espnow) {
    Serial.println(
        "STATUS | MODULE n | POWER on|off|toggle | CONTACTOR on|off|toggle | FAN 1|2|ALL pct | BAL on|off |"
        " KEEP on|off | FAMILY AUTO|09|0A | TEMPOFFSET c | NAME | SCREEN on|off | FLIP [on|off] | PAGE MAIN|DEBUG | VE [RAW on|off]");
    return;
  }

  sendAck(to, verb, false, "unknown_command");
}

static void onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (g_pendingNow.ready || len <= 0 || !info) {
    return;
  }
  if (data[0] == '{') {
    return;  // another node's JSON, not a command
  }
  const size_t n = min(static_cast<size_t>(len), sizeof(g_pendingNow.text) - 1);
  memcpy(g_pendingNow.text, data, n);
  g_pendingNow.text[n] = '\0';
  memcpy(g_pendingNow.mac, info->src_addr, 6);
  g_pendingNow.ready = true;
}

static void initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(kEspNowChannel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  if (esp_now_init() != ESP_OK) {
    Serial.println("[BMS] NOW: init failed");
    return;
  }
  esp_now_register_recv_cb(onEspNowRecv);
  g_espNowReady = ensurePeer(kBroadcastMac);
  Serial.printf("[BMS] NOW: %s on channel %u, MAC %s\n", g_espNowReady ? "ready" : "peer add failed",
                static_cast<unsigned>(kEspNowChannel), WiFi.macAddress().c_str());
}

static void serviceEspNowCommand() {
  if (!g_pendingNow.ready) {
    return;
  }
  char text[sizeof(g_pendingNow.text)];
  ReplyTarget to;
  to.espnow = true;
  memcpy(text, g_pendingNow.text, sizeof(text));
  memcpy(to.mac, g_pendingNow.mac, 6);
  g_pendingNow.ready = false;
  g_espNowRx++;
  g_espNowLastRxMs = millis();
  if (strncasecmp(text, "MASTER_HELLO", 12) == 0) {
    return;
  }
  Serial.printf("[BMS] NOW cmd from %02X:%02X:%02X:%02X:%02X:%02X: %s\n", to.mac[0], to.mac[1], to.mac[2], to.mac[3],
                to.mac[4], to.mac[5], text);
  processCommand(text, to);
}

static void serviceSerialCommand() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (g_serialLen > 0) {
        g_serialLine[g_serialLen] = '\0';
        g_serialLen = 0;
        ReplyTarget to;
        processCommand(g_serialLine, to);
      }
      continue;
    }
    if (g_serialLen < sizeof(g_serialLine) - 1) {
      g_serialLine[g_serialLen++] = c;
    }
  }
}

static void serviceSerialReport() {
  const uint32_t now = millis();
  if (now - g_lastSerialReportMs < 5000) {
    return;
  }
  g_lastSerialReportMs = now;
  char a[16];
  char b[16];
  snprintf(a, sizeof(a), isnan(g_probes[0].tempC) ? "--" : "%.2f", g_probes[0].tempC);
  snprintf(b, sizeof(b), isnan(g_probes[1].tempC) ? "--" : "%.2f", g_probes[1].tempC);
  Serial.printf("[BMS] pwr=%u ems=%u ems_led=%u cmus=%u can=%u fans=%u%%/%u%% rpm=%u/%u T1(%s)=%sC T2(%s)=%sC "
                "pins=%u/%umV 24V=%umV cmu12V=%umA(IS %umV) ve_bytes=%lu now_tx=%lu/%lu rx=%lu btn=%u%u%u\n",
                g_commPowerOn, g_contactorEnabled, g_emsLedOn, g_uartDetectedCount,
                g_cmuActive ? g_cmu.discoveredCount() : 0, g_fans[0].pct, g_fans[1].pct, g_fans[0].rpm, g_fans[1].rpm,
                probeKindLabel(g_probes[0].kind), a, probeKindLabel(g_probes[1].kind), b, g_probes[0].ntcMv,
                g_probes[1].ntcMv, g_vmonMv, g_cmuMa, g_cmuIsMv, static_cast<unsigned long>(g_veDiag.bytes),
                static_cast<unsigned long>(g_espNowTx), static_cast<unsigned long>(g_espNowTxFail),
                static_cast<unsigned long>(g_espNowRx), digitalRead(kPinBtnA), digitalRead(kPinBtnB),
                digitalRead(kPinBtnC));
}

// ------------------------------------------------------------------ setup / loop
void setup() {
  // Safe states first: fans off, CMU power off, EMS300 contact open.
  initFans();
  pinMode(kPinCmuPower, OUTPUT);
  digitalWrite(kPinCmuPower, LOW);
  pinMode(kPinEmsEnable, OUTPUT);
  digitalWrite(kPinEmsEnable, LOW);

  Serial.begin(115200);
  delay(100);
  Serial.println("\n[BMS] BMS controller v2 starting");

  pinMode(kPinEmsStatus, INPUT);  // 10k pull-up on the board
  pinMode(kPinBtnA, INPUT);
  pinMode(kPinBtnB, INPUT);
  pinMode(kPinBtnC, INPUT);
  analogSetPinAttenuation(kPinVmon, ADC_11db);
  analogSetPinAttenuation(kPinCmuIsense, ADC_11db);

  loadConfig();
  updateNodeIdentity();
  Serial.printf("[BMS] node=%s temp offset C=%.2f family=%s bal=%u keep=%u\n", g_nodeId, g_config.tempOffsetC,
                familyLabel(g_config.family), g_config.balanceAllEnabled, g_config.keepaliveEnabled);

  initDisplay();
  initProbes();

  g_namingUart.begin(kNamingBaud, SERIAL_8E1, kNamingRxPin, kNamingTxPin);
  g_veUart.begin(kVEDirectBaud, SERIAL_8N1, kVEDirectRxPin, kVEDirectTxPin);
  Serial.printf("[BMS] naming UART RX=%d TX=%d 1200 8E1, VE UART RX=%d TX=%d 19200 8N1\n", kNamingRxPin, kNamingTxPin,
                kVEDirectRxPin, kVEDirectTxPin);

  initEspNow();
  g_lastInteractionMs = millis();
  setPersistentStatus("Ready: B power, C contactor, hold A debug");
  drawScreen();
}

void loop() {
  pollVeDirect();
  serviceEspNowCommand();
  serviceSerialCommand();
  handleButtons();
  serviceAnalog();
  serviceProbes();
  serviceFanTach();

  if (g_cmuActive && !g_namingInProgress) {
    g_cmu.update();
    if (g_config.balanceAllEnabled != g_cmu.balanceAllEnabled()) {
      g_cmu.setBalanceAllEnabled(g_config.balanceAllEnabled);
    }
    if (g_config.keepaliveEnabled != g_cmu.keepaliveEnabled()) {
      g_cmu.setKeepaliveEnabled(g_config.keepaliveEnabled);
    }
  }

  if (g_screenOn && (millis() - g_lastInteractionMs) >= kScreenTimeoutMs) {
    sleepScreen();
  }

  if ((millis() - g_lastDrawMs) >= kDrawPeriodMs) {
    drawScreen();
    g_lastDrawMs = millis();
  }

  if ((millis() - g_lastBroadcastMs) >= kBroadcastPeriodMs) {
    ReplyTarget bc;
    bc.espnow = true;
    memcpy(bc.mac, kBroadcastMac, 6);
    sendStatusTo(bc);
    g_lastBroadcastMs = millis();
  }

  serviceSerialReport();
  delay(5);
}
