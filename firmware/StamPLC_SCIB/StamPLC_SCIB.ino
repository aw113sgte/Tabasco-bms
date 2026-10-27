#include <Arduino.h>
#include <M5StamPLC.h>
#include <Preferences.h>
#include <SCIBCMUComs.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>
#include <strings.h>

#include "driver/twai.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

using SCIBCMUComs::CommandFamily;
using SCIBCMUComs::Manager;
using SCIBCMUComs::ModuleData;
using SCIBCMUComs::kCellsPerModule;
using SCIBCMUComs::kMaxModules;
using SCIBCMUComs::kTempsPerModule;

struct AppConfig {
  float tempOffsetC = -5.6f;
  bool keepaliveEnabled = true;
  bool balanceAllEnabled = true;
  CommandFamily family = CommandFamily::Family0A;
  char wifiSsid[33] = "";
  char wifiPass[65] = "";
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

struct VEDirectLive {
  bool valid = false;
  uint32_t lastFrameMs = 0;
  int32_t socDeciPct = -1;
  int32_t currentMa = 0;
  int32_t batteryMv = -1;
  bool socValid = false;
  bool currentValid = false;
  bool batteryValid = false;
};

struct VEDirectPending {
  int32_t socDeciPct = -1;
  int32_t currentMa = 0;
  int32_t batteryMv = -1;
  bool sawSoc = false;
  bool sawCurrent = false;
  bool sawBattery = false;
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
  bool powerOn = false;
  bool contactorEnabled = false;
  bool balanceAllEnabled = false;
  bool screenOn = true;
  bool namingOk = false;
  bool namingReplyValid = false;
  bool cmuActive = false;
  CommandFamily family = CommandFamily::Family0A;
};

enum class NetworkState : uint8_t {
  Reconnecting = 0,
  Connected,
  RecoveryAp,
};

enum class NodeLinkState : uint8_t {
  RouterConnect = 0,
  RouterWaitMaster,
  ScanMasterAp,
  MasterConnect,
  MasterWait,
};

struct PendingUdpCommand {
  bool ready = false;
  IPAddress remoteIp;
  uint16_t remotePort = 0;
  char text[160] = {0};
};

struct PendingNetcfgApply {
  bool active = false;
  uint32_t applyAtMs = 0;
};

struct VEDirectDiag {
  uint32_t bytes = 0;
  uint32_t lines = 0;
  uint32_t goodFrames = 0;
  uint32_t badFrames = 0;
  uint32_t lastByteMs = 0;
  uint32_t lastGoodFrameMs = 0;
  uint32_t lastReportMs = 0;
  char lastLine[64] = {0};
};

static const uint32_t kCanBitrate = 500000;
static const int8_t kCanTxPin = 42;
static const int8_t kCanRxPin = 43;

static const uint32_t kNamingBaud = 1200;
static const int8_t kNamingTxPin = 1;  // Port A G1 -> K-line TX
static const int8_t kNamingRxPin = 2;  // Port A G2 <- K-line RX
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
static const int8_t kVEDirectRxPin = 4;  // Port C G4 <- SmartShunt TX
static const int8_t kVEDirectTxPin = 5;  // Port C G5 -> SmartShunt RX
static const uint32_t kVEDirectStaleMs = 5000;

static const uint8_t kRelayCommPower = 0;   // Relay 1
static const uint8_t kRelayContactor = 1;   // Relay 2
static const uint32_t kScreenTimeoutMs = 5UL * 60UL * 1000UL;
static const uint32_t kDrawPeriodMs = 250;
static const uint32_t kBroadcastPeriodMs = 1000;
static const uint16_t kUdpPort = 40120;
static const uint32_t kNetcfgApplyDelayMs = 1200;
static const uint32_t kMasterPresenceTimeoutMs = 20000;
static const uint32_t kMasterScanRetryMs = 10000;
static const uint32_t kWifiConnectTimeoutMs = 12000;
static const char* kDefaultWifiSsid = "";
static const char* kDefaultWifiPass = "";
static const char* kRecoveryApPass = "scibsetup";
static const char* kMasterApPrefix = "SCIB-Master-";
static const IPAddress kMasterRecoveryIp(192, 168, 4, 1);
static const IPAddress kMasterRecoveryBroadcast(192, 168, 4, 255);

Preferences g_prefs;
Manager g_cmu;
HardwareSerial g_namingUart(1);
HardwareSerial g_veUart(2);
WiFiUDP g_udp;

AppConfig g_config;
VEDirectLive g_veLive;
VEDirectPending g_vePending;
PendingUdpCommand g_pendingUdp;
PendingNetcfgApply g_pendingNetcfgApply;
VEDirectDiag g_veDiag;

bool g_commPowerOn = false;
bool g_contactorEnabled = false;
bool g_cmuActive = false;
bool g_screenOn = true;
bool g_namingInProgress = false;
bool g_udpReady = false;
uint8_t g_uartDetectedCount = 0;
uint64_t g_uartDetectedMask = 0;
bool g_uartReplyValid = false;
NetworkState g_networkState = NetworkState::Reconnecting;
NodeLinkState g_linkState = NodeLinkState::RouterConnect;
uint32_t g_lastMasterSeenMs = 0;
uint32_t g_lastMasterScanMs = 0;
uint32_t g_linkStateSinceMs = 0;
IPAddress g_masterIp;
bool g_masterIpValid = false;
uint32_t g_statusSendCount = 0;

char g_statusLine[96] = "Ready";
char g_nodeId[20] = "plc-000000";
char g_masterApSsid[33] = "";
uint32_t g_statusUntilMs = 0;

uint32_t g_lastInteractionMs = 0;
uint32_t g_lastDrawMs = 0;
uint32_t g_lastBroadcastMs = 0;

char g_veLine[96] = {0};
size_t g_veLineLen = 0;
uint8_t g_veChecksum = 0;

static const uint16_t COLOR_BG = TFT_BLACK;
static const uint16_t COLOR_TEXT = TFT_WHITE;
static const uint16_t COLOR_DIM = TFT_DARKGREY;
static const uint16_t COLOR_GOOD = TFT_GREENYELLOW;
static const uint16_t COLOR_WARN = TFT_ORANGE;
static const uint16_t COLOR_BAD = TFT_RED;
static const uint16_t COLOR_ACCENT = TFT_CYAN;

static void setStatus(const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vsnprintf(g_statusLine, sizeof(g_statusLine), fmt, args);
  va_end(args);
  g_statusUntilMs = millis() + 5000;
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

static bool veHasRetainedData() {
  return g_veLive.socValid || g_veLive.currentValid || g_veLive.batteryValid;
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

static const char* networkStateLabel(NetworkState state) {
  switch (state) {
    case NetworkState::Connected:
      return "ROUTER";
    case NetworkState::RecoveryAp:
      return "MASTER_AP";
    default:
      return "SEARCHING";
  }
}

static void updateNodeIdentity() {
  const uint64_t efuseMac = ESP.getEfuseMac();
  const unsigned long suffix = static_cast<unsigned long>(efuseMac & 0xFFFFFFULL);
  snprintf(g_nodeId, sizeof(g_nodeId), "plc-%06lX", suffix);
}

static bool isMasterApSsid(const String& ssid) {
  return ssid.length() > 0 && ssid.startsWith(kMasterApPrefix);
}

static void logNetf(const char* fmt, ...) {
  char buf[160];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.printf("[StamPLC_SCIB] NET: %s\n", buf);
}

static void setLinkState(NodeLinkState state) {
  g_linkState = state;
  g_linkStateSinceMs = millis();
}

static bool masterSeenRecently(uint32_t now) {
  return g_lastMasterSeenMs != 0 && (now - g_lastMasterSeenMs) < kMasterPresenceTimeoutMs;
}

static void rememberMasterIp(const IPAddress& ip) {
  if (ip[0] == 0 && ip[1] == 0 && ip[2] == 0 && ip[3] == 0) {
    return;
  }
  g_masterIp = ip;
  g_masterIpValid = true;
}

static void prepareStaMode() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(false);
}

static void startRouterAttempt() {
  if (g_config.wifiSsid[0] == '\0') {
    setLinkState(NodeLinkState::ScanMasterAp);
    return;
  }
  WiFi.scanDelete();
  prepareStaMode();
  WiFi.disconnect();
  delay(20);
  g_lastMasterSeenMs = 0;
  WiFi.begin(g_config.wifiSsid, g_config.wifiPass);
  g_networkState = NetworkState::Reconnecting;
  setLinkState(NodeLinkState::RouterConnect);
  setStatus("Join router %s", g_config.wifiSsid);
  logNetf("try router ssid=%s", g_config.wifiSsid);
}

static bool beginMasterApConnect(const char* ssid) {
  if (!ssid || ssid[0] == '\0') {
    return false;
  }
  strlcpy(g_masterApSsid, ssid, sizeof(g_masterApSsid));
  WiFi.scanDelete();
  prepareStaMode();
  WiFi.disconnect();
  delay(20);
  g_lastMasterSeenMs = 0;
  WiFi.begin(g_masterApSsid, kRecoveryApPass);
  g_networkState = NetworkState::Reconnecting;
  setLinkState(NodeLinkState::MasterConnect);
  setStatus("Join %s", g_masterApSsid);
  logNetf("try master ap ssid=%s", g_masterApSsid);
  return true;
}

static bool processMasterScanResults(int found) {
  int bestIndex = -1;
  int32_t bestRssi = -1000;
  for (int i = 0; i < found; ++i) {
    const String ssid = WiFi.SSID(i);
    if (!isMasterApSsid(ssid)) {
      continue;
    }
    const int32_t rssi = WiFi.RSSI(i);
    if (bestIndex < 0 || rssi > bestRssi) {
      bestIndex = i;
      bestRssi = rssi;
    }
  }

  if (bestIndex < 0) {
    WiFi.scanDelete();
    g_masterApSsid[0] = '\0';
    return false;
  }

  const String targetSsid = WiFi.SSID(bestIndex);
  WiFi.scanDelete();
  return beginMasterApConnect(targetSsid.c_str());
}

static void startMasterScan(bool force = false) {
  const uint32_t now = millis();
  prepareStaMode();
  setLinkState(NodeLinkState::ScanMasterAp);
  g_networkState = NetworkState::Reconnecting;
  if (!force && g_lastMasterScanMs != 0 && (now - g_lastMasterScanMs) < kMasterScanRetryMs) {
    return;
  }
  WiFi.scanDelete();
  const int rc = WiFi.scanNetworks(false, true);
  g_lastMasterScanMs = now;
  logNetf("scan master ap rc=%d", rc);
  if (rc >= 0) {
    if (!processMasterScanResults(rc) && g_config.wifiSsid[0] != '\0') {
      logNetf("master ap not found, return to router");
      startRouterAttempt();
    }
    return;
  }
  setStatus("Master AP scan failed");
  logNetf("master ap scan failed");
}

static void initNetwork() {
  prepareStaMode();
  if (g_config.wifiSsid[0] != '\0') {
    startRouterAttempt();
  } else {
    startMasterScan(true);
  }
  if (g_udp.begin(kUdpPort)) {
    g_udpReady = true;
  } else {
    g_udpReady = false;
    setStatus("UDP port %u failed", static_cast<unsigned>(kUdpPort));
  }
}

static void serviceNetwork() {
  const uint32_t now = millis();
  const wl_status_t status = WiFi.status();
  const String currentSsid = (status == WL_CONNECTED) ? WiFi.SSID() : String();

  switch (g_linkState) {
    case NodeLinkState::RouterConnect:
      if (status == WL_CONNECTED) {
        if (isMasterApSsid(currentSsid)) {
          g_networkState = NetworkState::RecoveryAp;
          setLinkState(NodeLinkState::MasterWait);
          setStatus("Master AP %s", currentSsid.c_str());
          logNetf("connected master ap ssid=%s", currentSsid.c_str());
        } else {
          g_networkState = NetworkState::Connected;
          setLinkState(NodeLinkState::RouterWaitMaster);
          setStatus("Router %s", currentSsid.c_str());
          logNetf("connected router ssid=%s", currentSsid.c_str());
        }
      } else if ((now - g_linkStateSinceMs) >= kWifiConnectTimeoutMs) {
        logNetf("router connect timeout");
        startMasterScan(true);
      }
      break;

    case NodeLinkState::RouterWaitMaster:
      g_networkState = NetworkState::Connected;
      if (status != WL_CONNECTED) {
        startRouterAttempt();
        break;
      }
      if (isMasterApSsid(currentSsid)) {
        g_networkState = NetworkState::RecoveryAp;
        setLinkState(NodeLinkState::MasterWait);
        break;
      }
      if (!masterSeenRecently(now) && (now - g_linkStateSinceMs) >= kMasterPresenceTimeoutMs) {
        logNetf("master heartbeat missing on router, scan master ap");
        WiFi.disconnect();
        delay(20);
        startMasterScan(true);
      }
      break;

    case NodeLinkState::ScanMasterAp:
      g_networkState = NetworkState::Reconnecting;
      if ((now - g_lastMasterScanMs) >= kMasterScanRetryMs) {
        startMasterScan(true);
      }
      break;

    case NodeLinkState::MasterConnect:
      if (status == WL_CONNECTED) {
        if (isMasterApSsid(currentSsid)) {
          g_networkState = NetworkState::RecoveryAp;
          setLinkState(NodeLinkState::MasterWait);
          setStatus("Master AP %s", currentSsid.c_str());
          logNetf("connected master ap ssid=%s", currentSsid.c_str());
        } else {
          g_networkState = NetworkState::Connected;
          setLinkState(NodeLinkState::RouterWaitMaster);
          logNetf("unexpected router while connecting master, ssid=%s", currentSsid.c_str());
        }
      } else if ((now - g_linkStateSinceMs) >= kWifiConnectTimeoutMs) {
        logNetf("master ap connect timeout");
        if (g_config.wifiSsid[0] != '\0') {
          startRouterAttempt();
        } else {
          startMasterScan(true);
        }
      }
      break;

    case NodeLinkState::MasterWait:
      g_networkState = NetworkState::RecoveryAp;
      if (status != WL_CONNECTED || !isMasterApSsid(currentSsid)) {
        logNetf("master ap lost, restart search");
        startMasterScan(true);
        break;
      }
      break;
  }
}

static void loadConfig() {
  g_prefs.begin("stamplc-scib", false);
  const bool tempoffMigrated = g_prefs.getBool("tempoff_v2", false);
  const bool tempoffMigratedV3 = g_prefs.getBool("tempoff_v3", false);
  g_config.tempOffsetC = g_prefs.getFloat("tempoff", -5.6f);
  if (!tempoffMigrated) {
    if (fabsf(g_config.tempOffsetC - 10.0f) < 0.01f) {
      g_config.tempOffsetC = 0.0f;
      g_prefs.putFloat("tempoff", g_config.tempOffsetC);
    }
    g_prefs.putBool("tempoff_v2", true);
  }
  if (!tempoffMigratedV3) {
    if (fabsf(g_config.tempOffsetC - 0.0f) < 0.01f) {
      g_config.tempOffsetC = -5.6f;
      g_prefs.putFloat("tempoff", g_config.tempOffsetC);
    }
    g_prefs.putBool("tempoff_v3", true);
  }
  g_config.keepaliveEnabled = g_prefs.getBool("keepalive", true);
  g_config.balanceAllEnabled = g_prefs.getBool("balall", true);
  const String wifiSsid = g_prefs.getString("wifi_ssid", kDefaultWifiSsid);
  const String wifiPass = g_prefs.getString("wifi_pass", kDefaultWifiPass);
  const uint8_t familyRaw = g_prefs.getUChar("family", static_cast<uint8_t>(CommandFamily::Family0A));
  g_prefs.end();

  strlcpy(g_config.wifiSsid, wifiSsid.c_str(), sizeof(g_config.wifiSsid));
  strlcpy(g_config.wifiPass, wifiPass.c_str(), sizeof(g_config.wifiPass));

  if (familyRaw == static_cast<uint8_t>(CommandFamily::Family09)) {
    g_config.family = CommandFamily::Family09;
  } else if (familyRaw == static_cast<uint8_t>(CommandFamily::Family0A)) {
    g_config.family = CommandFamily::Family0A;
  } else {
    g_config.family = CommandFamily::Auto;
  }
}

static void saveConfig() {
  g_prefs.begin("stamplc-scib", false);
  g_prefs.putFloat("tempoff", g_config.tempOffsetC);
  g_prefs.putBool("keepalive", g_config.keepaliveEnabled);
  g_prefs.putBool("balall", g_config.balanceAllEnabled);
  g_prefs.putUChar("family", static_cast<uint8_t>(g_config.family));
  g_prefs.putString("wifi_ssid", g_config.wifiSsid);
  g_prefs.putString("wifi_pass", g_config.wifiPass);
  g_prefs.end();
}

static void wakeScreen() {
  g_screenOn = true;
  g_lastInteractionMs = millis();
  M5StamPLC.setBacklight(true);
}

static void sleepScreen() {
  g_screenOn = false;
  M5StamPLC.setBacklight(false);
}

static void clearNamingUartRx() {
  while (g_namingUart.available() > 0) {
    g_namingUart.read();
  }
}

static void dumpTwaiStatus(const char* tag) {
  twai_status_info_t info;
  if (twai_get_status_info(&info) != ESP_OK) {
    Serial.printf("[StamPLC_SCIB] %s: twai status unavailable\n", tag);
    return;
  }
  Serial.printf("[StamPLC_SCIB] %s: state=%d tx_err=%u rx_err=%u tx_failed=%u rx_missed=%u rx_overrun=%u arb_lost=%u bus_err=%u msgs_tx=%u msgs_rx=%u\n",
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
    Serial.println("[StamPLC_SCIB] Naming failed: CAN init");
    return result;
  }
  result.canStarted = true;

  for (uint8_t attempt = 1; attempt <= kNamingAttempts; ++attempt) {
    result.attemptsRun = attempt;
    const bool attemptOk = runNamingAttempt(result);
    dumpTwaiStatus("after naming attempt");
    Serial.printf("[StamPLC_SCIB] naming attempt=%u arm=%u disarm=%u uart=%u tx_fail=%u reply=%u ok=%u\n",
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

  return result;
}

static void setCommPower(bool enabled) {
  if (g_commPowerOn == enabled) {
    return;
  }

  g_commPowerOn = enabled;
  M5StamPLC.writePlcRelay(kRelayCommPower, enabled);

  if (!enabled) {
    stopCmuManager();
    g_uartDetectedCount = 0;
    g_uartDetectedMask = 0;
    g_uartReplyValid = false;
    setStatus("CMU power OFF");
    return;
  }

  setStatus("CMU power ON, naming...");
  delay(kPowerRailSettleMs);
  g_namingInProgress = true;
  runNamingSequence();
  g_namingInProgress = false;
}

static void setContactorEnabled(bool enabled) {
  if (g_contactorEnabled == enabled) {
    return;
  }
  g_contactorEnabled = enabled;
  M5StamPLC.writePlcRelay(kRelayContactor, enabled);
  setStatus("Contactor %s", enabled ? "ENABLED" : "DISABLED");
}

static void updateStatusLight() {
  if (g_namingInProgress) {
    M5StamPLC.setStatusLight(255, 48, 0);
    return;
  }
  if (!g_commPowerOn) {
    M5StamPLC.setStatusLight(0, 0, 32);
    return;
  }
  if (!g_cmuActive || g_cmu.discoveredCount() == 0) {
    M5StamPLC.setStatusLight(255, 0, 0);
    return;
  }
  if (g_config.balanceAllEnabled) {
    M5StamPLC.setStatusLight(0, 128, 0);
    return;
  }
  M5StamPLC.setStatusLight(0, 32, 96);
}

static void commitVeFrame() {
  g_veLive.valid = g_vePending.sawSoc || g_vePending.sawCurrent || g_vePending.sawBattery;
  g_veLive.lastFrameMs = millis();
  g_veLive.socValid = g_vePending.sawSoc;
  g_veLive.currentValid = g_vePending.sawCurrent;
  g_veLive.batteryValid = g_vePending.sawBattery;
  if (g_vePending.sawSoc) {
    g_veLive.socDeciPct = g_vePending.socDeciPct;
  }
  if (g_vePending.sawCurrent) {
    g_veLive.currentMa = g_vePending.currentMa;
  }
  if (g_vePending.sawBattery) {
    g_veLive.batteryMv = g_vePending.batteryMv;
  }
  if (g_veLive.valid) {
    g_veDiag.goodFrames++;
    g_veDiag.lastGoodFrameMs = g_veLive.lastFrameMs;
  }
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
    g_vePending.socDeciPct = strtol(value, nullptr, 10);
    g_vePending.sawSoc = true;
    return;
  }
  if (strcmp(key, "I") == 0) {
    g_vePending.currentMa = strtol(value, nullptr, 10);
    g_vePending.sawCurrent = true;
    return;
  }
  if (strcmp(key, "V") == 0) {
    g_vePending.batteryMv = strtol(value, nullptr, 10);
    g_vePending.sawBattery = true;
    return;
  }
  if (strcmp(key, "Checksum") == 0) {
    if (g_veChecksum == 0) {
      commitVeFrame();
    } else {
      g_veDiag.badFrames++;
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
    g_veChecksum = static_cast<uint8_t>(g_veChecksum + b);

    if (b == '\n') {
      g_veDiag.lines++;
      g_veLine[g_veLineLen] = '\0';
      const bool isChecksumLine = (strncmp(g_veLine, "Checksum\t", 9) == 0) || (strcmp(g_veLine, "Checksum") == 0);
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

  g_veLive.valid = veStreamActive() && veHasRetainedData();
}

static void serviceVeDiagnostics() {
  const uint32_t nowMs = millis();
  if ((nowMs - g_veDiag.lastReportMs) < 5000) {
    return;
  }
  g_veDiag.lastReportMs = nowMs;

  if (veStreamActive() && veHasRetainedData()) {
    Serial.printf("[StamPLC_SCIB] VE: OK soc=%.1f%% current=%+.3fA good=%u bad=%u bytes=%u lines=%u\n",
                  g_veLive.socDeciPct / 10.0f, g_veLive.currentMa / 1000.0f,
                  static_cast<unsigned>(g_veDiag.goodFrames), static_cast<unsigned>(g_veDiag.badFrames),
                  static_cast<unsigned>(g_veDiag.bytes), static_cast<unsigned>(g_veDiag.lines));
    return;
  }

  if (g_veDiag.bytes == 0) {
    Serial.println("[StamPLC_SCIB] VE: no bytes received on G4 (19200 8N1)");
    return;
  }

  Serial.printf("[StamPLC_SCIB] VE: no valid frame yet bytes=%u lines=%u good=%u bad=%u last_line=\"%s\"\n",
                static_cast<unsigned>(g_veDiag.bytes), static_cast<unsigned>(g_veDiag.lines),
                static_cast<unsigned>(g_veDiag.goodFrames), static_cast<unsigned>(g_veDiag.badFrames),
                g_veDiag.lastLine[0] ? g_veDiag.lastLine : "-");
}

static SummaryView buildSummaryView() {
  SummaryView view;
  view.uartCount = g_commPowerOn ? g_uartDetectedCount : 0;
  const bool veActive = veStreamActive();
  view.socValid = veActive && g_veLive.socValid;
  view.currentValid = veActive && g_veLive.currentValid;
  view.batteryValid = veActive && g_veLive.batteryValid;
  view.socDeciPct = view.socValid ? g_veLive.socDeciPct : -1;
  view.currentMa = view.currentValid ? g_veLive.currentMa : 0;
  view.batteryMv = view.batteryValid ? g_veLive.batteryMv : -1;
  view.powerOn = g_commPowerOn;
  view.contactorEnabled = g_contactorEnabled;
  view.balanceAllEnabled = g_config.balanceAllEnabled;
  view.screenOn = g_screenOn;
  view.namingReplyValid = g_uartReplyValid;
  view.namingOk = g_uartReplyValid;
  view.cmuActive = g_cmuActive;
  view.family = g_config.family;

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
        if (totalMv > view.moduleHighMv) {
          view.moduleHighMv = totalMv;
        }
        if (totalMv < view.moduleLowMv) {
          view.moduleLowMv = totalMv;
        }
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
        if (tempDeciF > view.tempHighDeciF) {
          view.tempHighDeciF = tempDeciF;
        }
        if (tempDeciF < view.tempLowDeciF) {
          view.tempLowDeciF = tempDeciF;
        }
      }
    }
  }

  if (view.canCount > 0) {
    view.moduleAvgMv = static_cast<int32_t>(totalModuleMv / view.canCount);
  }
  if (tempCount > 0) {
    view.tempAvgDeciF = static_cast<int32_t>(tempSumDeciF / tempCount);
  }

  return view;
}

static void formatVoltageTriple(char* out, size_t len, const SummaryView& view) {
  if (view.moduleHighMv < 0 || view.moduleAvgMv < 0 || view.moduleLowMv < 0) {
    snprintf(out, len, "--/--/--");
    return;
  }
  snprintf(out, len, "%.2f/%.2f/%.2f", view.moduleHighMv / 1000.0f, view.moduleAvgMv / 1000.0f, view.moduleLowMv / 1000.0f);
}

static void formatTempTriple(char* out, size_t len, const SummaryView& view) {
  if (view.tempHighDeciF < 0 || view.tempAvgDeciF < 0 || view.tempLowDeciF < 0) {
    snprintf(out, len, "--/--/--");
    return;
  }
  snprintf(out, len, "%.1f/%.1f/%.1f", view.tempHighDeciF / 10.0f, view.tempAvgDeciF / 10.0f, view.tempLowDeciF / 10.0f);
}

static void formatSoc(char* out, size_t len, const SummaryView& view) {
  if (view.socDeciPct < 0) {
    snprintf(out, len, "--");
    return;
  }
  snprintf(out, len, "%.1f", view.socDeciPct / 10.0f);
}

static void formatCurrent(char* out, size_t len, const SummaryView& view) {
  snprintf(out, len, "%+.2fA", view.currentMa / 1000.0f);
}

static void drawScreen() {
  if (!g_screenOn) {
    return;
  }

  const SummaryView view = buildSummaryView();
  char volts[32];
  char temps[32];
  char soc[16];
  char current[16];

  formatVoltageTriple(volts, sizeof(volts), view);
  formatTempTriple(temps, sizeof(temps), view);
  formatSoc(soc, sizeof(soc), view);
  formatCurrent(current, sizeof(current), view);

  M5StamPLC.Display.fillScreen(COLOR_BG);
  M5StamPLC.Display.setTextSize(1);
  const int16_t textX = 3;
  const int16_t lineH = M5StamPLC.Display.fontHeight() + 1;
  int16_t y = 0;

  M5StamPLC.Display.setTextColor(COLOR_TEXT, COLOR_BG);
  M5StamPLC.Display.setCursor(textX, y);
  M5StamPLC.Display.printf("CMUs:%u  CAN:%u", static_cast<unsigned>(view.uartCount), static_cast<unsigned>(view.canCount));
  y += lineH;

  M5StamPLC.Display.setTextColor(COLOR_DIM, COLOR_BG);
  M5StamPLC.Display.setCursor(textX, y);
  if (WiFi.status() == WL_CONNECTED) {
    M5StamPLC.Display.printf("NET:%s  %s", networkStateLabel(g_networkState), WiFi.SSID().c_str());
  } else {
    M5StamPLC.Display.printf("NET:%s  waiting", networkStateLabel(g_networkState));
  }
  y += lineH;

  M5StamPLC.Display.setCursor(textX, y);
  M5StamPLC.Display.printf("V:%s", volts);
  y += lineH;

  M5StamPLC.Display.setCursor(textX, y);
  M5StamPLC.Display.printf("T:%s F", temps);
  y += lineH;

  M5StamPLC.Display.setCursor(textX, y);
  M5StamPLC.Display.printf("%%C:%s", soc);
  y += lineH;

  M5StamPLC.Display.setCursor(textX, y);
  M5StamPLC.Display.printf("I:%s", current);
  y += lineH;

  const uint16_t relayColor = view.powerOn ? COLOR_GOOD : COLOR_DIM;
  M5StamPLC.Display.setTextColor(relayColor, COLOR_BG);
  M5StamPLC.Display.setCursor(textX, y);
  M5StamPLC.Display.printf("PWR:%s  ", view.powerOn ? "ON" : "OFF");
  M5StamPLC.Display.setTextColor(view.contactorEnabled ? COLOR_GOOD : COLOR_WARN, COLOR_BG);
  M5StamPLC.Display.printf("CT:%s", view.contactorEnabled ? "ON" : "OFF");
  y += lineH;

  M5StamPLC.Display.setTextColor(view.balanceAllEnabled ? COLOR_GOOD : COLOR_DIM, COLOR_BG);
  M5StamPLC.Display.setCursor(textX, y);
  M5StamPLC.Display.printf("BAL:%s", view.balanceAllEnabled ? "ON" : "OFF");
  M5StamPLC.Display.setTextColor(COLOR_TEXT, COLOR_BG);
  M5StamPLC.Display.printf("  F:%s  d:%umV", familyLabel(view.family), static_cast<unsigned>(view.maxDeltaMv));
  y += lineH;

  M5StamPLC.Display.setTextColor(COLOR_DIM, COLOR_BG);
  M5StamPLC.Display.setCursor(textX, y);
  if (g_statusUntilMs == 0 || millis() <= g_statusUntilMs) {
    M5StamPLC.Display.print(g_statusLine);
  } else {
    M5StamPLC.Display.print("A:wake  B:power  C:cont");
  }
}

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

static bool buildStatusJson(char* out, size_t len) {
  const SummaryView view = buildSummaryView();
  const String ip = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : String("0.0.0.0");
  size_t used = 0;
  return appendFormat(out, len, used,
                      "{\"type\":\"status\",\"node_id\":\"%s\",\"name\":\"StampPLC_SCIB\",\"ip\":\"%s\","
                      "\"net_state\":\"%s\",\"cmus_uart\":%u,\"cmus_can\":%u,\"v_hi_mv\":%ld,\"v_avg_mv\":%ld,"
                      "\"v_lo_mv\":%ld,\"t_hi_df\":%ld,\"t_avg_df\":%ld,\"t_lo_df\":%ld,\"soc_dpct\":%ld,"
                      "\"soc_valid\":%u,\"cur_ma\":%ld,\"cur_valid\":%u,\"batt_mv\":%ld,\"batt_valid\":%u,"
                      "\"power\":%u,\"contactor\":%u,\"bal_all\":%u,\"family\":\"%s\","
                      "\"screen\":%u,\"delta_max_mv\":%u,\"status\":\"%s\"}",
                      g_nodeId, ip.c_str(), networkStateLabel(g_networkState),
                      static_cast<unsigned>(view.uartCount), static_cast<unsigned>(view.canCount),
                      static_cast<long>(view.moduleHighMv), static_cast<long>(view.moduleAvgMv),
                      static_cast<long>(view.moduleLowMv), static_cast<long>(view.tempHighDeciF),
                      static_cast<long>(view.tempAvgDeciF), static_cast<long>(view.tempLowDeciF),
                      static_cast<long>(view.socDeciPct), view.socValid ? 1U : 0U,
                      static_cast<long>(view.currentMa), view.currentValid ? 1U : 0U,
                      static_cast<long>(view.batteryMv), view.batteryValid ? 1U : 0U,
                      view.powerOn ? 1U : 0U, view.contactorEnabled ? 1U : 0U, view.balanceAllEnabled ? 1U : 0U, familyLabel(view.family),
                      view.screenOn ? 1U : 0U, static_cast<unsigned>(view.maxDeltaMv), g_statusLine);
}

static bool buildModuleJson(uint8_t moduleNo, char* out, size_t len) {
  const ModuleData* module = g_cmu.module(moduleNo);
  if (!module || !module->discovered) {
    snprintf(out, len, "{\"type\":\"error\",\"msg\":\"module_not_found\"}");
    return false;
  }

  float totalV = NAN;
  float avgCellV = NAN;
  uint16_t deltaMv = 0;
  float avgTempF = NAN;
  uint8_t validCells = 0;
  uint8_t validTemps = 0;
  const bool haveMetrics = g_cmu.moduleMetrics(moduleNo, totalV, avgCellV, deltaMv, avgTempF, validCells, validTemps);
  const long totalMv = haveMetrics ? static_cast<long>(lroundf(totalV * 1000.0f)) : -1;
  const long avgCellMv = haveMetrics ? static_cast<long>(lroundf(avgCellV * 1000.0f)) : -1;
  const long avgTempDeciF = (haveMetrics && !isnan(avgTempF)) ? static_cast<long>(lroundf(avgTempF * 10.0f)) : -1;

  size_t used = 0;
  if (!appendFormat(out, len, used,
                    "{\"type\":\"module\",\"slot\":%u,\"can\":%u,\"online\":%u,\"status_word\":%u,\"total_mv\":%ld,"
                    "\"avg_cell_mv\":%ld,\"delta_mv\":%u,\"avg_temp_df\":%ld,\"cells_mv\":[",
                    static_cast<unsigned>(moduleNo), static_cast<unsigned>(module->canModuleNo), module->online ? 1U : 0U,
                    static_cast<unsigned>(module->statusWord), totalMv, avgCellMv, static_cast<unsigned>(deltaMv),
                    avgTempDeciF)) {
    return false;
  }

  for (uint8_t i = 0; i < kCellsPerModule; ++i) {
    if (!appendFormat(out, len, used, "%s%d", (i == 0) ? "" : ",", module->cellValid[i] ? module->cells_mV[i] : -1)) {
      return false;
    }
  }

  if (!appendFormat(out, len, used, "],\"temps_df\":[")) {
    return false;
  }

  for (uint8_t i = 0; i < kTempsPerModule; ++i) {
    const int tempDeciF =
        (module->tempValid[i] && !isnan(module->temps_C[i]))
            ? static_cast<int>(lroundf(SCIBCMUComs::Manager::cToF(module->temps_C[i]) * 10.0f))
            : -1;
    if (!appendFormat(out, len, used, "%s%d", (i == 0) ? "" : ",", tempDeciF)) {
      return false;
    }
  }

  return appendFormat(out, len, used, "]}");
}

static bool sendUdpPayload(const IPAddress& destIp, uint16_t destPort, const char* payload) {
  if (!g_udpReady || !payload || destPort == 0) {
    return false;
  }
  if (!g_udp.beginPacket(destIp, destPort)) {
    return false;
  }
  g_udp.write(reinterpret_cast<const uint8_t*>(payload), strlen(payload));
  return g_udp.endPacket() == 1;
}

static void sendStatusDirect(const IPAddress& destIp, uint16_t destPort) {
  char payload[768];
  if (!buildStatusJson(payload, sizeof(payload))) {
    Serial.printf("[StamPLC_SCIB] NET: buildStatusJson overflow len=%u\n",
                  static_cast<unsigned>(sizeof(payload)));
    return;
  }
  sendUdpPayload(destIp, destPort, payload);
}

static void sendStatusHeartbeat() {
  char payload[768];
  if (!buildStatusJson(payload, sizeof(payload))) {
    Serial.printf("[StamPLC_SCIB] NET: buildStatusJson overflow len=%u\n",
                  static_cast<unsigned>(sizeof(payload)));
    return;
  }
  sendUdpPayload(IPAddress(255, 255, 255, 255), kUdpPort, payload);
  if (WiFi.status() == WL_CONNECTED) {
    const String ssid = WiFi.SSID();
    if (isMasterApSsid(ssid)) {
      sendUdpPayload(kMasterRecoveryIp, kUdpPort, payload);
      sendUdpPayload(kMasterRecoveryBroadcast, kUdpPort, payload);
      const IPAddress gw = WiFi.gatewayIP();
      if (!(gw[0] == 0 && gw[1] == 0 && gw[2] == 0 && gw[3] == 0)) {
        sendUdpPayload(gw, kUdpPort, payload);
      }
    } else if (g_masterIpValid && masterSeenRecently(millis())) {
      sendUdpPayload(g_masterIp, kUdpPort, payload);
    }
  }
  ++g_statusSendCount;
  if ((g_statusSendCount % 10U) == 0U) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[StamPLC_SCIB] NET: status tx ssid=%s ip=%s gw=%s master=%s valid=%u\n",
                    WiFi.SSID().c_str(),
                    WiFi.localIP().toString().c_str(),
                    WiFi.gatewayIP().toString().c_str(),
                    g_masterIpValid ? g_masterIp.toString().c_str() : "0.0.0.0",
                    g_masterIpValid ? 1U : 0U);
    } else {
      Serial.printf("[StamPLC_SCIB] NET: status tx disconnected\n");
    }
  }
}

static void sendUdpAck(const IPAddress& ip, uint16_t port, const char* cmd, bool ok, const char* msg) {
  char payload[192];
  snprintf(payload, sizeof(payload), "{\"type\":\"ack\",\"node_id\":\"%s\",\"cmd\":\"%s\",\"ok\":%u,\"msg\":\"%s\"}",
           g_nodeId, cmd, ok ? 1U : 0U, msg ? msg : "");
  sendUdpPayload(ip, port, payload);
}

static void pollUdpPacket() {
  if (g_pendingUdp.ready || !g_udpReady) {
    return;
  }

  const int packetLen = g_udp.parsePacket();
  if (packetLen <= 0) {
    return;
  }

  const size_t copyLen = (static_cast<size_t>(packetLen) >= sizeof(g_pendingUdp.text))
                             ? (sizeof(g_pendingUdp.text) - 1)
                             : static_cast<size_t>(packetLen);
  g_pendingUdp.remoteIp = g_udp.remoteIP();
  g_pendingUdp.remotePort = g_udp.remotePort();
  const int actual = g_udp.read(reinterpret_cast<uint8_t*>(g_pendingUdp.text), copyLen);
  if (actual <= 0) {
    return;
  }
  g_pendingUdp.text[actual] = '\0';
  if (g_pendingUdp.text[0] == '{') {
    return;
  }
  g_pendingUdp.ready = true;
}

static void processUdpCommand() {
  if (!g_pendingUdp.ready) {
    return;
  }

  char command[sizeof(g_pendingUdp.text)];
  IPAddress replyIp = g_pendingUdp.remoteIp;
  const uint16_t replyPort = g_pendingUdp.remotePort;
  memcpy(command, g_pendingUdp.text, sizeof(command));
  g_pendingUdp.ready = false;

  for (char* p = command; *p; ++p) {
    if (*p == '\r' || *p == '\n') {
      *p = '\0';
      break;
    }
  }

  char* savePtr = nullptr;
  char* verb = strtok_r(command, " \t", &savePtr);
  if (!verb) {
    sendUdpAck(replyIp, replyPort, "NONE", false, "empty_command");
    return;
  }

  if (strcasecmp(verb, "MASTER_HELLO") == 0) {
    g_lastMasterSeenMs = millis();
    rememberMasterIp(replyIp);
    return;
  }

  rememberMasterIp(replyIp);

  if (strcasecmp(verb, "STATUS") == 0 || strcasecmp(verb, "GET") == 0) {
    sendStatusDirect(replyIp, replyPort);
    return;
  }

  if (strcasecmp(verb, "MODULE") == 0) {
    char* token = strtok_r(nullptr, " \t", &savePtr);
    if (!token) {
      sendUdpAck(replyIp, replyPort, "MODULE", false, "missing_slot");
      return;
    }
    const int moduleNo = atoi(token);
    char payload[320];
    if (moduleNo < 1 || moduleNo > kMaxModules || !buildModuleJson(static_cast<uint8_t>(moduleNo), payload, sizeof(payload))) {
      sendUdpAck(replyIp, replyPort, "MODULE", false, "invalid_or_missing");
      return;
    }
    sendUdpPayload(replyIp, replyPort, payload);
    return;
  }

  if (strcasecmp(verb, "POWER") == 0) {
    bool value = false;
    if (!isTruthyToken(strtok_r(nullptr, " \t", &savePtr), g_commPowerOn, value)) {
      sendUdpAck(replyIp, replyPort, "POWER", false, "use_on_off_toggle");
      return;
    }
    wakeScreen();
    setCommPower(value);
    sendUdpAck(replyIp, replyPort, "POWER", true, value ? "on" : "off");
    sendStatusDirect(replyIp, replyPort);
    return;
  }

  if (strcasecmp(verb, "CONTACTOR") == 0) {
    bool value = false;
    if (!isTruthyToken(strtok_r(nullptr, " \t", &savePtr), g_contactorEnabled, value)) {
      sendUdpAck(replyIp, replyPort, "CONTACTOR", false, "use_on_off_toggle");
      return;
    }
    wakeScreen();
    setContactorEnabled(value);
    sendUdpAck(replyIp, replyPort, "CONTACTOR", true, value ? "on" : "off");
    sendStatusDirect(replyIp, replyPort);
    return;
  }

  if (strcasecmp(verb, "BAL") == 0 || strcasecmp(verb, "BALANCE") == 0) {
    bool value = false;
    if (!isTruthyToken(strtok_r(nullptr, " \t", &savePtr), g_config.balanceAllEnabled, value)) {
      sendUdpAck(replyIp, replyPort, "BAL", false, "use_on_off_toggle");
      return;
    }
    g_config.balanceAllEnabled = value;
    saveConfig();
    if (g_cmuActive) {
      g_cmu.setBalanceAllEnabled(value);
    }
    setStatus("Balance all %s", value ? "ON" : "OFF");
    sendUdpAck(replyIp, replyPort, "BAL", true, value ? "on" : "off");
    sendStatusDirect(replyIp, replyPort);
    return;
  }

  if (strcasecmp(verb, "KEEP") == 0) {
    bool value = false;
    if (!isTruthyToken(strtok_r(nullptr, " \t", &savePtr), g_config.keepaliveEnabled, value)) {
      sendUdpAck(replyIp, replyPort, "KEEP", false, "use_on_off_toggle");
      return;
    }
    g_config.keepaliveEnabled = value;
    saveConfig();
    if (g_cmuActive) {
      g_cmu.setKeepaliveEnabled(value);
    }
    setStatus("Keepalive %s", value ? "ON" : "OFF");
    sendUdpAck(replyIp, replyPort, "KEEP", true, value ? "on" : "off");
    sendStatusDirect(replyIp, replyPort);
    return;
  }

  if (strcasecmp(verb, "FAMILY") == 0) {
    char* token = strtok_r(nullptr, " \t", &savePtr);
    if (!token) {
      sendUdpAck(replyIp, replyPort, "FAMILY", false, "missing_auto_09_0A");
      return;
    }
    g_config.family = familyFromToken(token);
    saveConfig();
    if (g_cmuActive) {
      g_cmu.setCommandFamily(g_config.family);
    }
    setStatus("Family %s", familyLabel(g_config.family));
    sendUdpAck(replyIp, replyPort, "FAMILY", true, familyLabel(g_config.family));
    sendStatusDirect(replyIp, replyPort);
    return;
  }

  if (strcasecmp(verb, "TEMPOFFSET") == 0) {
    char* token = strtok_r(nullptr, " \t", &savePtr);
    if (!token) {
      sendUdpAck(replyIp, replyPort, "TEMPOFFSET", false, "missing_value");
      return;
    }
    g_config.tempOffsetC = strtof(token, nullptr);
    saveConfig();
    if (g_cmuActive) {
      g_cmu.setTempOffsetC(g_config.tempOffsetC);
    }
    setStatus("Temp offset %.1fC", g_config.tempOffsetC);
    sendUdpAck(replyIp, replyPort, "TEMPOFFSET", true, "ok");
    sendStatusDirect(replyIp, replyPort);
    return;
  }

  if (strcasecmp(verb, "NETCFG") == 0) {
    while (savePtr && (*savePtr == ' ' || *savePtr == '\t')) {
      ++savePtr;
    }
    if (!savePtr || !*savePtr) {
      sendUdpAck(replyIp, replyPort, "NETCFG", false, "missing_ssid_tab_pass");
      return;
    }
    char* split = strchr(savePtr, '\t');
    if (!split) {
      sendUdpAck(replyIp, replyPort, "NETCFG", false, "use_ssid_tab_pass");
      return;
    }
    *split++ = '\0';
    if (*savePtr == '\0') {
      sendUdpAck(replyIp, replyPort, "NETCFG", false, "empty_ssid");
      return;
    }
    strlcpy(g_config.wifiSsid, savePtr, sizeof(g_config.wifiSsid));
    strlcpy(g_config.wifiPass, split, sizeof(g_config.wifiPass));
    saveConfig();
    g_pendingNetcfgApply.active = true;
    g_pendingNetcfgApply.applyAtMs = millis() + kNetcfgApplyDelayMs;
    setStatus("WiFi config saved");
    sendUdpAck(replyIp, replyPort, "NETCFG", true, "saved");
    return;
  }

  if (strcasecmp(verb, "NAME") == 0) {
    if (!g_commPowerOn) {
      sendUdpAck(replyIp, replyPort, "NAME", false, "power_off");
      return;
    }
    wakeScreen();
    g_namingInProgress = true;
    const NamingResult result = runNamingSequence();
    g_namingInProgress = false;
    sendUdpAck(replyIp, replyPort, "NAME", result.ok, result.replyValid ? "reply_ok" : "no_reply");
    sendStatusDirect(replyIp, replyPort);
    return;
  }

  if (strcasecmp(verb, "SCREEN") == 0) {
    bool value = false;
    if (!isTruthyToken(strtok_r(nullptr, " \t", &savePtr), g_screenOn, value)) {
      sendUdpAck(replyIp, replyPort, "SCREEN", false, "use_on_off_toggle");
      return;
    }
    if (value) {
      wakeScreen();
    } else {
      sleepScreen();
    }
    sendUdpAck(replyIp, replyPort, "SCREEN", true, value ? "on" : "off");
    sendStatusDirect(replyIp, replyPort);
    return;
  }

  sendUdpAck(replyIp, replyPort, verb, false, "unknown_command");
}

static void servicePendingNetcfgApply() {
  if (!g_pendingNetcfgApply.active) {
    return;
  }
  if (millis() < g_pendingNetcfgApply.applyAtMs) {
    return;
  }
  g_pendingNetcfgApply.active = false;
  if (g_config.wifiSsid[0] != '\0') {
    startRouterAttempt();
  } else {
    startMasterScan(true);
  }
}

static void handleButtons() {
  if (M5StamPLC.BtnA.wasClicked()) {
    wakeScreen();
    return;
  }

  if (M5StamPLC.BtnB.wasClicked()) {
    wakeScreen();
    setCommPower(!g_commPowerOn);
    return;
  }

  if (M5StamPLC.BtnC.wasClicked()) {
    wakeScreen();
    setContactorEnabled(!g_contactorEnabled);
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  loadConfig();
  Serial.printf("[StamPLC_SCIB] Temp offset C=%.2f\n", g_config.tempOffsetC);
  updateNodeIdentity();

  {
    auto plcConfig = M5StamPLC.config();
    plcConfig.enableCan = true;
    plcConfig.canBaudRate = static_cast<long>(kCanBitrate);
    M5StamPLC.config(plcConfig);
  }
  M5StamPLC.begin();
  M5StamPLC.Display.setRotation(1);
  M5StamPLC.Display.setTextWrap(false);
  M5StamPLC.Display.setTextSize(1);
  M5StamPLC.Display.setFont(&fonts::Font2);
  wakeScreen();

  M5StamPLC.writePlcRelay(kRelayCommPower, false);
  M5StamPLC.writePlcRelay(kRelayContactor, false);

  g_namingUart.begin(kNamingBaud, SERIAL_8E1, kNamingRxPin, kNamingTxPin);
  g_veUart.begin(kVEDirectBaud, SERIAL_8N1, kVEDirectRxPin, kVEDirectTxPin);
  Serial.printf("[StamPLC_SCIB] VE UART on Port C: RX=G%d TX=G%d @ %lu 8N1\n",
                static_cast<int>(kVEDirectRxPin), static_cast<int>(kVEDirectTxPin),
                static_cast<unsigned long>(kVEDirectBaud));

  initNetwork();
  updateStatusLight();
  setPersistentStatus("Ready: B power, C contactor, UDP %u", static_cast<unsigned>(kUdpPort));
  drawScreen();
}

void loop() {
  M5StamPLC.update();
  serviceNetwork();
  pollUdpPacket();
  pollVeDirect();
  serviceVeDiagnostics();
  processUdpCommand();
  servicePendingNetcfgApply();
  handleButtons();

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
    sendStatusHeartbeat();
    g_lastBroadcastMs = millis();
  }

  updateStatusLight();
  delay(10);
}
