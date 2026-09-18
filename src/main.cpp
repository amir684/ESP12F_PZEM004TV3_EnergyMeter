/*
  ---------------------------------------------------------------
  Project: ESP12F Energy Meter with OTA + MQTT + Reset + WebUI
  Version: V6
  Author: AmirY
  Date: 2025
  ---------------------------------------------------------------

  ⚙️ Description:
  Smart energy meter on ESP8266 (ESP-12F) reading a PZEM-004T-V3,
  showing data on an ST7567 128x64 LCD, publishing to MQTT.
  Purpose-built to log washing-machine cycles by power draw.

  Features:
    - Non-blocking main loop (millis based, no delay() in loop)
    - Web management UI with tabs: Status / Sessions / WiFi / MQTT
    - Editable hostname (appears in the router) + MQTT settings,
      persisted to LittleFS
    - NTP real-time clock (Israel TZ) for session timestamps
    - Washing-session logging: a session starts when power rises
      above ~10W and ends after power stays below ~10W for 5 min;
      start/end/duration/energy/peak saved to flash (last 40 kept)
    - TTP223 capacitive touch (GPIO3/RX) cycles 4 screens:
        Main readings -> Power graph -> Status -> Info
    - Live power history graph on the LCD

  ---------------------------------------------------------------
  🔌 Hardware Wiring:

  🧠 ESP12F (ESP8266):
    • D1  (GPIO5)  → TX of PZEM-004T-V3
    • D2  (GPIO4)  → RX of PZEM-004T-V3
    • D6  (GPIO12) → LCD backlight control (PWM)
    • GPIO0  (CS)  → ST7567 LCD Chip Select
    • GPIO2  (DC)  → ST7567 LCD Data/Command
    • GPIO16 (RST) → ST7567 LCD Reset
    • D5/D7 (SPI CLK/MOSI) shared hardware SPI lines to LCD
    • RX  (GPIO3/D9) → TTP223 touch sensor OUT (idle LOW, touch HIGH)
      NOTE: GPIO3 is the UART RX pin. Using it as touch input disables
      serial *input* (debug prints via TX still work). Disconnect the
      sensor before flashing over USB (OTA is unaffected).

  ---------------------------------------------------------------
  📊 MQTT Topics (base topic is configurable, default "home/energy"):
    <base>/voltage  <base>/current  <base>/power
    <base>/energy   <base>/frequency <base>/pf
    <base>/reset          ← send "RESET" to reset energy counter
    <base>/reset_status   ← returns "SUCCESS" or "FAILED"

  🌐 Web UI:  http://<device-ip>/
  ---------------------------------------------------------------
*/

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <PZEM004Tv30.h>
#include <U8g2lib.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include <time.h>

// ---------------- User settings ----------------
#define LCD_LED         D6
#define LCD_BRIGHTNESS  500
#define LCD_CONTRAST    20
#define LCD_ROTATION    U8G2_R3

// Backlight auto-dim (PWM range is 0..1023)
#define LCD_DIM_LEVEL    45      // idle brightness
#define LCD_DIM_TIMEOUT  60000UL // ms without a touch before dimming
#define LCD_FADE_STEP    20      // PWM steps per fade tick
#define LCD_FADE_MS      12      // ms between fade ticks

// Canvas is 64 x 128: a 128x64 panel rotated by U8G2_R3
#define SCR_W           64
#define SCR_H          128
#define HEADER_H        11
#define DOTS_Y         124

#define FONT_MICRO      u8g2_font_4x6_tr
#define FONT_SMALL      u8g2_font_5x8_tr
#define FONT_LABEL      u8g2_font_6x12_tr
#define FONT_VALUE      u8g2_font_7x13B_tr

// Non-blocking timing intervals (ms)
#define READ_INTERVAL          2000    // PZEM read + display refresh
#define MQTT_INTERVAL          5000    // MQTT publish
#define MQTT_RECONNECT_INTERVAL 5000   // spacing between reconnect attempts

// ---------------- Touch (TTP223) ----------------
#define TOUCH_PIN        3      // GPIO3 / RX / D9, idle LOW (UART RX repurposed)
#define TOUCH_DEBOUNCE   200    // ms
#define TOUCH_LONG_PRESS 800    // ms held = jump back to the main screen

// ---------------- Network identity ----------------
#define DEFAULT_HOSTNAME "EnergyMeter"   // shown in the router; editable via web

// ---------------- NTP (real-time clock over WiFi) ----------------
#define NTP_TZ       "IST-2IDT,M3.4.4/26,M10.5.0"   // Israel (auto DST)
#define NTP_SERVER1  "pool.ntp.org"
#define NTP_SERVER2  "time.google.com"

// ---------------- Washing session detection ----------------
#define SESSION_POWER_THRESHOLD 10.0f      // W: above => running, below => (maybe) idle
#define SESSION_END_GRACE       300000UL   // ms of sustained low power to end a session
#define SESSION_MIN_DURATION    60UL       // s: ignore blips shorter than this
#define MAX_SESSIONS            40         // rolling history kept in flash

// ---------------- OTA ----------------
#define OTA_PASSWORD    "12345678"

// ---------------- MQTT defaults (used on first boot only) ----------------
#define MQTT_SERVER     "192.168.1.175"
#define MQTT_PORT       1883
#define MQTT_USER       "mqtt_user"
#define MQTT_PASS       "password"
#define MQTT_TOPIC      "home/energy"

// ---------------- Persistent configuration ----------------
#define CFG_MAGIC 0xE12F0006UL
struct DeviceConfig {
  uint32_t magic;
  char     hostname[32];
  char     mqtt_server[40];
  uint16_t mqtt_port;
  char     mqtt_user[24];
  char     mqtt_pass[24];
  char     mqtt_topic[40];
};
DeviceConfig config;

// ---------------- LCD ST7567 ----------------
U8G2_ST7567_JLX12864_F_4W_HW_SPI u8g2(LCD_ROTATION, /* cs=*/ 0, /* dc=*/ 2, /* reset=*/ 16);

// ---------------- PZEM ----------------
#define PZEM_RX D2
#define PZEM_TX D1
SoftwareSerial pzemSW(PZEM_RX, PZEM_TX);
PZEM004Tv30 pzem(pzemSW);

// ---------------- WiFi / MQTT / Web ----------------
WiFiClient       espClient;
PubSubClient     mqtt(espClient);
ESP8266WebServer server(80);

// ---------------- Runtime state ----------------
// Latest readings (shared by all screens + web API)
float gV = 0, gI = 0, gP = 0, gE = 0, gF = 0, gPF = 0;
float maxPower = 0;

unsigned long lastRead = 0;
unsigned long lastMqttSend = 0;
unsigned long lastMqttReconnect = 0;

// Screen state machine
enum Screen { SCR_MAIN = 0, SCR_GRAPH, SCR_STATUS, SCR_INFO, SCREEN_COUNT };
uint8_t currentScreen = SCR_MAIN;

// Touch state
int           lastTouchState = LOW;
unsigned long lastTouchTime = 0;
unsigned long touchDownTime = 0;
bool          longPressFired = false;
bool          touchWokeDisplay = false;   // this press only un-dimmed the LCD

// Backlight state
int           blCurrent = LCD_BRIGHTNESS;
int           blTarget  = LCD_BRIGHTNESS;
unsigned long blLastActivity = 0;
unsigned long blLastFade = 0;

// Power history for the graph
#define GRAPH_POINTS 64
float powerHist[GRAPH_POINTS];
int   histCount = 0;

// ---- Washing session logging ----
struct WashSession {
  uint32_t start;      // epoch seconds (0 if clock not yet synced)
  uint32_t end;        // epoch seconds
  uint32_t duration;   // seconds
  float    energy;     // kWh consumed during the session
  float    peak;       // peak power (W)
};
WashSession sessions[MAX_SESSIONS];
int     sessionCount = 0;

// Live session tracking
bool          sessionActive = false;
uint32_t      curStartEpoch = 0;
unsigned long curStartMillis = 0;
float         curEnergyStart = 0;
float         curPeak = 0;
unsigned long curLowSince = 0;     // millis when power first dropped low (0 = not low)
unsigned long curLowMillis = 0;    // millis snapshot when low started
uint32_t      curLowEpoch = 0;     // epoch snapshot when low started
float         curLowEnergy = 0;    // energy snapshot when low started
float         lastSessionEnergy = 0; // kWh of the most recently finished session

bool timeSynced() { return time(nullptr) > 1600000000UL; }  // ~2020+

// ============================================================
//  Configuration storage (LittleFS)
// ============================================================
void loadDefaults() {
  config.magic = CFG_MAGIC;
  strlcpy(config.hostname, DEFAULT_HOSTNAME, sizeof(config.hostname));
  strlcpy(config.mqtt_server, MQTT_SERVER, sizeof(config.mqtt_server));
  config.mqtt_port = MQTT_PORT;
  strlcpy(config.mqtt_user,  MQTT_USER,  sizeof(config.mqtt_user));
  strlcpy(config.mqtt_pass,  MQTT_PASS,  sizeof(config.mqtt_pass));
  strlcpy(config.mqtt_topic, MQTT_TOPIC, sizeof(config.mqtt_topic));
}

void saveConfig() {
  File f = LittleFS.open("/config.bin", "w");
  if (f) {
    f.write((uint8_t*)&config, sizeof(config));
    f.close();
    Serial.println("Config saved");
  } else {
    Serial.println("Config save FAILED");
  }
}

void loadConfig() {
  if (LittleFS.exists("/config.bin")) {
    File f = LittleFS.open("/config.bin", "r");
    if (f) {
      if (f.size() == sizeof(config)) {
        f.read((uint8_t*)&config, sizeof(config));
        f.close();
        if (config.magic == CFG_MAGIC) {
          Serial.println("Config loaded from flash");
          return;
        }
      } else {
        f.close();
      }
    }
  }
  Serial.println("Using default config");
  loadDefaults();
  saveConfig();
}

// Apply MQTT config to the client (call after changes)
void applyMqttConfig() {
  mqtt.disconnect();
  mqtt.setServer(config.mqtt_server, config.mqtt_port);
  lastMqttReconnect = 0;  // trigger reconnect ASAP
}

// ============================================================
//  Session storage (LittleFS)
// ============================================================
void saveSessions() {
  File f = LittleFS.open("/sessions.bin", "w");
  if (!f) { Serial.println("Sessions save FAILED"); return; }
  f.write((uint8_t*)&sessionCount, sizeof(sessionCount));
  f.write((uint8_t*)sessions, sizeof(WashSession) * sessionCount);
  f.close();
}

void loadSessions() {
  sessionCount = 0;
  if (!LittleFS.exists("/sessions.bin")) return;
  File f = LittleFS.open("/sessions.bin", "r");
  if (!f) return;
  int n = 0;
  if (f.read((uint8_t*)&n, sizeof(n)) == sizeof(n) && n >= 0 && n <= MAX_SESSIONS) {
    f.read((uint8_t*)sessions, sizeof(WashSession) * n);
    sessionCount = n;
  }
  f.close();
  if (sessionCount > 0) lastSessionEnergy = sessions[sessionCount - 1].energy;
  Serial.printf("Loaded %d sessions\n", sessionCount);
}

// Append a session, keeping only the most recent MAX_SESSIONS
void addSession(const WashSession &s) {
  if (sessionCount < MAX_SESSIONS) {
    sessions[sessionCount++] = s;
  } else {
    for (int k = 0; k < MAX_SESSIONS - 1; k++) sessions[k] = sessions[k + 1];
    sessions[MAX_SESSIONS - 1] = s;
  }
  saveSessions();
  Serial.printf("Session saved: %lus, %.3f kWh, peak %.0fW\n",
                (unsigned long)s.duration, s.energy, s.peak);
}

void clearSessions() {
  sessionCount = 0;
  LittleFS.remove("/sessions.bin");
}

// Feed each new reading into the wash-session state machine
void updateSession(float p, float e) {
  if (isnan(p)) return;
  unsigned long now = millis();

  if (!sessionActive) {
    if (p >= SESSION_POWER_THRESHOLD) {
      sessionActive  = true;
      curStartEpoch  = timeSynced() ? (uint32_t)time(nullptr) : 0;
      curStartMillis = now;
      curEnergyStart = isnan(e) ? 0 : e;
      curPeak        = p;
      curLowSince    = 0;
      Serial.println("Wash session STARTED");
    }
    return;
  }

  // Running
  if (!isnan(p) && p > curPeak) curPeak = p;

  if (p < SESSION_POWER_THRESHOLD) {
    if (curLowSince == 0) {              // just dropped low -> snapshot the end point
      curLowSince  = now;
      curLowMillis = now;
      curLowEpoch  = timeSynced() ? (uint32_t)time(nullptr) : 0;
      curLowEnergy = isnan(e) ? curEnergyStart : e;
    } else if (now - curLowSince >= SESSION_END_GRACE) {
      uint32_t durS = (curLowMillis - curStartMillis) / 1000;
      if (durS >= SESSION_MIN_DURATION) {
        WashSession s;
        s.start    = curStartEpoch;
        s.end      = curLowEpoch;
        s.duration = durS;
        float en   = curLowEnergy - curEnergyStart;
        s.energy   = (en < 0) ? 0 : en;       // guard a mid-session counter reset
        s.peak     = curPeak;
        lastSessionEnergy = s.energy;
        addSession(s);
      } else {
        Serial.println("Session too short, ignored");
      }
      sessionActive = false;
      curLowSince   = 0;
    }
  } else {
    curLowSince = 0;                     // power back up -> still running
  }
}

// ============================================================
//  Display helpers
//
//  Canvas is 64 x 128 (a 128x64 panel rotated by U8G2_R3), so every
//  layout below is portrait. All drawing goes through the o*()
//  wrappers, which add a horizontal offset -- that is what lets a
//  whole screen be slid sideways for the touch transition without
//  the layout code knowing anything about it.
// ============================================================

// Horizontal draw offset, non-zero only while a slide is running
static int16_t gOX = 0;
// Screen currently being drawn (differs from currentScreen mid-slide)
static uint8_t gDrawScreen = SCR_MAIN;
// Axis top of the last graph drawn, so the caller can label it
static float   gGraphMax = 1;

static inline void oPixel(int x, int y)                  { u8g2.drawPixel(x + gOX, y); }
static inline void oHLine(int x, int y, int w)           { u8g2.drawHLine(x + gOX, y, w); }
static inline void oVLine(int x, int y, int h)           { u8g2.drawVLine(x + gOX, y, h); }
static inline void oBox(int x, int y, int w, int h)      { u8g2.drawBox(x + gOX, y, w, h); }
static inline void oFrame(int x, int y, int w, int h)    { u8g2.drawFrame(x + gOX, y, w, h); }
static inline void oRBox(int x, int y, int w, int h, int r)   { u8g2.drawRBox(x + gOX, y, w, h, r); }
static inline void oRFrame(int x, int y, int w, int h, int r) { u8g2.drawRFrame(x + gOX, y, w, h, r); }
static inline void oLine(int x0, int y0, int x1, int y1) { u8g2.drawLine(x0 + gOX, y0, x1 + gOX, y1); }
static inline void oDisc(int x, int y, int r)            { u8g2.drawDisc(x + gOX, y, r); }
static inline void oTri(int x0, int y0, int x1, int y1, int x2, int y2) {
  u8g2.drawTriangle(x0 + gOX, y0, x1 + gOX, y1, x2 + gOX, y2);
}
static inline void oStr(int x, int y, const char* s)     { u8g2.drawStr(x + gOX, y, s); }
// Right-aligned: xr is the last pixel column the text may occupy
static inline void oStrR(int xr, int y, const char* s)   { oStr(xr - u8g2.getStrWidth(s) + 1, y, s); }
static inline void oStrC(int xc, int y, const char* s)   { oStr(xc - u8g2.getStrWidth(s) / 2, y, s); }

// Small rotating scratch buffers, so several formatted values can be
// used in one expression without each caller declaring its own array.
static char    fmtBuf[4][20];
static uint8_t fmtIdx = 0;
static const char* fmtF(float v, int dec) {
  fmtIdx = (fmtIdx + 1) & 3;
  if (isnan(v) || isinf(v)) v = 0;
  snprintf(fmtBuf[fmtIdx], sizeof(fmtBuf[0]), "%.*f", dec, v);
  return fmtBuf[fmtIdx];
}
// Compact form for large numbers: 940 -> "940", 2150 -> "2.1k"
static const char* fmtK(float v) {
  fmtIdx = (fmtIdx + 1) & 3;
  if (isnan(v) || isinf(v) || v < 0) v = 0;
  if (v >= 1000.0f) snprintf(fmtBuf[fmtIdx], sizeof(fmtBuf[0]), "%.1fk", v / 1000.0f);
  else              snprintf(fmtBuf[fmtIdx], sizeof(fmtBuf[0]), "%.0f",  v);
  return fmtBuf[fmtIdx];
}

// ---- primitives -------------------------------------------------
// 50% checkerboard fill of one column, used for the graph area fill.
// The (x ^ y) parity keeps the dither aligned between columns.
static void ditherCol(int x, int yTop, int yBottom) {
  int y = yTop + (((yTop ^ x) & 1) ? 1 : 0);
  for (; y <= yBottom; y += 2) oPixel(x, y);
}

static void dashedHLine(int x0, int x1, int y) {
  for (int x = x0; x <= x1; x += 3) oPixel(x, y);
}

// Rounds up to the next "nice" axis value. The ladder is deliberately finer
// than 1/2/5: a 2148 W peak against a 5000 W axis would waste half the plot,
// so the steps in between keep the trace filling the available height.
static float niceCeil(float v) {
  if (v <= 0) return 1;
  static const float steps[] = { 1.0f, 1.2f, 1.5f, 2.0f, 2.5f, 3.0f,
                                 4.0f, 5.0f, 6.0f, 8.0f, 10.0f };
  float e = powf(10, floorf(log10f(v)));
  float m = v / e;
  for (uint8_t i = 0; i < sizeof(steps) / sizeof(steps[0]); i++)
    if (m <= steps[i] + 0.0001f) return steps[i] * e;
  return 10.0f * e;
}

// ---- shared chrome ----------------------------------------------
// WiFi bars + MQTT square. Drawn inside the inverted header, so the
// caller owns the draw colour.
static void drawHeaderIcons() {
  bool wl = (WiFi.status() == WL_CONNECTED);
  int  bars = 0;
  if (wl) {
    int rssi = WiFi.RSSI();
    if      (rssi >= -55) bars = 4;
    else if (rssi >= -65) bars = 3;
    else if (rssi >= -75) bars = 2;
    else                  bars = 1;
  }

  const int nBars = 4, barW = 2, pitch = 3;
  const int startX = SCR_W - 2 - (nBars * pitch - 1);   // 4 bars span 11px
  for (int b = 0; b < nBars; b++) {
    int h = 2 + b * 2;                                  // 2, 4, 6, 8
    int x = startX + b * pitch;
    if (b < bars) oBox(x, 9 - h, barW, h);
    else          oBox(x, 8, barW, 1);                  // empty bar = foot only
  }

  if (mqtt.connected()) oBox(startX - 7, 4, 4, 4);
  else                  oFrame(startX - 7, 4, 4, 4);
}

static void drawHeader(const char* title) {
  oBox(0, 0, SCR_W, HEADER_H);
  u8g2.setDrawColor(0);
  u8g2.setFont(FONT_SMALL);
  oStr(3, 8, title);
  drawHeaderIcons();
  u8g2.setDrawColor(1);
}

// Page indicator: one dot per screen, filled for the current one
static void drawPageDots() {
  const int pitch = 10;
  int x0 = (SCR_W - (SCREEN_COUNT - 1) * pitch) / 2;
  for (int i = 0; i < SCREEN_COUNT; i++) {
    int cx = x0 + i * pitch;
    if (i == gDrawScreen) oDisc(cx, DOTS_Y, 2);
    else                  oBox(cx - 1, DOTS_Y - 1, 2, 2);
  }
}

// Label on the left, value right-aligned on the same baseline. The value
// drops to the micro font, and is truncated as a last resort, so that it
// can never run into the label on this 64px-wide canvas.
static void drawKV(int y, const char* key, const char* val) {
  u8g2.setFont(FONT_MICRO);
  oStr(2, y, key);
  int avail = (SCR_W - 3) - (2 + u8g2.getStrWidth(key) + 3);

  u8g2.setFont(FONT_SMALL);
  if ((int)u8g2.getStrWidth(val) > avail) u8g2.setFont(FONT_MICRO);

  if ((int)u8g2.getStrWidth(val) <= avail) { oStrR(SCR_W - 3, y, val); return; }

  char cut[24];
  strncpy(cut, val, sizeof(cut) - 1);
  cut[sizeof(cut) - 1] = 0;
  for (int n = strlen(cut); n > 0 && (int)u8g2.getStrWidth(cut) > avail; n--) cut[n - 1] = 0;
  oStrR(SCR_W - 3, y, cut);
}

// Rounded badge: filled when on, outlined when off
static void drawPill(int yBase, const char* text, bool on) {
  u8g2.setFont(FONT_MICRO);
  int w = u8g2.getStrWidth(text) + 6;
  int x = SCR_W - 2 - w;
  if (on) {
    oRBox(x, yBase - 7, w, 9, 2);
    u8g2.setDrawColor(0);
    oStr(x + 3, yBase - 1, text);
    u8g2.setDrawColor(1);
  } else {
    oRFrame(x, yBase - 7, w, 9, 2);
    oStr(x + 3, yBase - 1, text);
  }
}

static void drawStateRow(int y, const char* key, bool on, const char* onTxt, const char* offTxt) {
  u8g2.setFont(FONT_MICRO);
  oStr(2, y, key);
  drawPill(y + 1, on ? onTxt : offTxt, on);
}

// Largest of the big digit fonts that still fits, falling back to the
// regular bold value font when even the smallest one would overflow.
static void drawHeroValue(int yBase, int yTopLimit, const char* num, const char* unit) {
  u8g2.setFont(FONT_LABEL);
  int uw = (unit && unit[0]) ? u8g2.getStrWidth(unit) + 3 : 0;
  int avail = SCR_W - 4 - uw;

  const uint8_t* fonts[4] = { u8g2_font_logisoso24_tn, u8g2_font_logisoso20_tn,
                              u8g2_font_logisoso16_tn, FONT_VALUE };
  const uint8_t* best = fonts[3];
  for (int i = 0; i < 4; i++) {
    u8g2.setFont(fonts[i]);
    if ((int)u8g2.getStrWidth(num) <= avail &&
        yBase - u8g2.getAscent() >= yTopLimit) { best = fonts[i]; break; }
  }

  u8g2.setFont(best);
  int nw = u8g2.getStrWidth(num);
  oStr(SCR_W - 2 - uw - nw, yBase, num);
  if (uw) {
    u8g2.setFont(FONT_LABEL);
    oStr(SCR_W - 2 - uw + 3, yBase, unit);
  }
}

// Filled area chart over the power history: a dithered body under a
// solid trace. Columns map 1:1 to history slots, newest on the right.
static void drawPowerArea(int x0, int yTop, int yBottom, int points, bool grid) {
  int h = yBottom - yTop;
  if (h < 2 || points < 2) return;

  float maxP = 1.0f;
  for (int k = 0; k < GRAPH_POINTS; k++)
    if (powerHist[k] > maxP) maxP = powerHist[k];
  maxP = niceCeil(maxP);
  gGraphMax = maxP;

  if (grid) {
    for (int g = 1; g <= 3; g++) dashedHLine(x0, x0 + points - 1, yBottom - (h * g) / 4);
    dashedHLine(x0, x0 + points - 1, yTop);
  }

  const int first = GRAPH_POINTS - points;        // leftmost history slot shown
  const int oldest = GRAPH_POINTS - histCount;    // first slot holding real data
  int prevX = -1, prevY = 0;
  for (int i = 0; i < points; i++) {
    int idx = first + i;
    if (idx < oldest) continue;                   // not sampled yet
    float v = powerHist[idx];
    if (isnan(v) || v < 0) v = 0;
    int y = yBottom - (int)((v / maxP) * h + 0.5f);
    if (y > yBottom) y = yBottom;
    if (y < yTop)    y = yTop;

    ditherCol(x0 + i, y, yBottom);                  // body
    if (prevX >= 0) oLine(prevX, prevY, x0 + i, y); // solid trace on top
    else            oPixel(x0 + i, y);
    prevX = x0 + i;
    prevY = y;
  }

  oHLine(x0, yBottom + 1, points);                  // baseline
  if (prevX >= 0) oDisc(prevX - 1, prevY, 1);       // "now" marker
}

// ---- boot / status screens --------------------------------------
void splashScreen() {
  const int cx = SCR_W / 2;

  for (int p = 0; p <= 100; p += 4) {
    u8g2.clearBuffer();

    // lightning bolt
    oTri(cx + 4, 20, cx - 6, 42, cx + 1, 42);
    oTri(cx - 1, 40, cx + 6, 40, cx - 4, 62);

    u8g2.setFont(u8g2_font_7x13B_tr);
    oStrC(cx, 82, "ENERGY");
    oStrC(cx, 95, "METER");

    u8g2.setFont(FONT_MICRO);
    oStrC(cx, 106, "AmirY  V7");

    oRFrame(6, 112, SCR_W - 12, 7, 2);
    if (p > 2) oBox(8, 114, ((SCR_W - 16) * p) / 100, 3);

    u8g2.sendBuffer();
    delay(14);
  }
  delay(300);
}

// Shared layout for the boot / OTA messages, with an optional bar
static void drawBanner(const char* title, const char* msg, int progress) {
  u8g2.clearBuffer();
  drawHeader(title);

  u8g2.setFont(FONT_MICRO);
  size_t len = strlen(msg);
  if ((int)u8g2.getStrWidth(msg) <= SCR_W - 4 || len >= 24) {
    oStrC(SCR_W / 2, 52, msg);
  } else {
    // split on the space nearest the middle so long messages fit
    char line[24];
    size_t cut = len / 2;
    while (cut > 0 && msg[cut] != ' ') cut--;
    if (cut == 0) cut = len / 2;
    strncpy(line, msg, cut);
    line[cut] = 0;
    oStrC(SCR_W / 2, 46, line);
    oStrC(SCR_W / 2, 56, msg + cut + (msg[cut] == ' ' ? 1 : 0));
  }

  if (progress >= 0) {
    oRFrame(6, 68, SCR_W - 12, 9, 2);
    if (progress > 2) oBox(8, 70, ((SCR_W - 16) * progress) / 100, 5);
    char pct[12];
    snprintf(pct, sizeof(pct), "%d%%", progress % 1000);
    u8g2.setFont(FONT_SMALL);
    oStrC(SCR_W / 2, 92, pct);
  }
  u8g2.sendBuffer();
}

void showWiFiStatus(const char* msg)                   { drawBanner("WIFI", msg, -1); }
void showOTAStatus(const char* msg, int progress = -1) { drawBanner("OTA",  msg, progress); }

// ---- Screen 1: main readings ----
// One grid cell: unit lives in the label so the value stays a big number
static void drawCell(int x, int y, const char* label, const char* val) {
  u8g2.setFont(FONT_MICRO);
  oStr(x + 3, y + 6, label);
  u8g2.setFont(FONT_VALUE);
  if ((int)u8g2.getStrWidth(val) > 28) u8g2.setFont(FONT_LABEL);
  oStrR(x + 29, y + 18, val);
}

void drawMainScreen() {
  drawHeader("ENERGY");

  u8g2.setFont(FONT_MICRO);
  oStr(2, 17, "POWER");
  drawHeroValue(42, 20, fmtF(gP, 0), "W");
  oHLine(0, 44, SCR_W);

  drawCell(0,  46, "VOLT V",  fmtF(gV, 0));
  drawCell(32, 46, "CURR A",  fmtF(gI, 2));
  oVLine(31, 48, 15);
  oHLine(0, 65, SCR_W);

  drawCell(0,  67, "FREQ Hz", fmtF(gF, 1));
  drawCell(32, 67, "POW.F",   fmtF(gPF, 2));
  oVLine(31, 69, 15);
  oHLine(0, 86, SCR_W);

  drawKV(96, "kWh", fmtF(gE, 2));

  // glanceable sparkline over the same history the graph screen plots
  drawPowerArea(2, 103, 116, 60, false);

  drawPageDots();
}

// ---- Screen 2: power graph ----
void drawGraphScreen() {
  drawHeader("POWER");

  drawHeroValue(36, 13, fmtF(gP, 0), "W");
  oHLine(0, 40, SCR_W);

  drawPowerArea(0, 50, 102, GRAPH_POINTS, true);

  // scale row, drawn after the chart so gGraphMax is known
  char buf[16];
  u8g2.setFont(FONT_MICRO);
  oStr(2, 46, "2 min");
  snprintf(buf, sizeof(buf), "max %s", fmtK(gGraphMax));
  oStrR(SCR_W - 3, 46, buf);

  // average and peak over the window on screen
  float sum = 0, peak = 0;
  for (int k = 0; k < histCount; k++) {
    float v = powerHist[GRAPH_POINTS - 1 - k];
    if (isnan(v) || v < 0) v = 0;
    sum += v;
    if (v > peak) peak = v;
  }
  float avg = histCount ? sum / histCount : 0;

  drawKV(111, "AVG W",  fmtK(avg));
  drawKV(119, "PEAK W", fmtK(peak));

  drawPageDots();
}

// ---- Screen 3: general status ----
void formatUptime(char* buf, size_t n) {
  unsigned long s = millis() / 1000;
  unsigned long h = s / 3600;
  unsigned long m = (s % 3600) / 60;
  unsigned long sec = s % 60;
  snprintf(buf, n, "%luh%02lum%02lus", h, m, sec);
}

// Two-part form for the LCD, where the full string does not fit
void formatUptimeShort(char* buf, size_t n) {
  unsigned long s = millis() / 1000;
  unsigned long h = s / 3600;
  if (h) snprintf(buf, n, "%luh%02lum", h, (s % 3600) / 60);
  else   snprintf(buf, n, "%lum%02lus", s / 60, s % 60);
}

void drawStatusScreen() {
  drawHeader("STATUS");

  bool wl = (WiFi.status() == WL_CONNECTED);

  drawStateRow(21, "WIFI", wl, "LINK", "DOWN");
  dashedHLine(2, SCR_W - 3, 25);

  String ssid = wl ? WiFi.SSID() : String("-");
  drawKV(35, "SSID", ssid.c_str());
  dashedHLine(2, SCR_W - 3, 39);

  // the IP needs the full width, so it gets a centred line of its own
  String ip = wl ? WiFi.localIP().toString() : String("not connected");
  u8g2.setFont(FONT_MICRO);
  oStrC(SCR_W / 2, 49, ip.c_str());
  dashedHLine(2, SCR_W - 3, 53);

  // signal strength: value plus a bar
  int rssi = wl ? WiFi.RSSI() : -100;
  int q = (rssi + 100) * 2;                       // -100..-50 dBm -> 0..100%
  if (q < 0)   q = 0;
  if (q > 100) q = 100;
  char buf[16];
  snprintf(buf, sizeof(buf), "%d dBm", rssi);
  drawKV(63, "RSSI", buf);
  oRFrame(2, 66, SCR_W - 5, 7, 2);
  if (q > 3) oBox(4, 68, ((SCR_W - 9) * q) / 100, 3);

  drawStateRow(85, "MQTT", mqtt.connected(), "CONN", "OFF");
  dashedHLine(2, SCR_W - 3, 89);

  formatUptimeShort(buf, sizeof(buf));
  drawKV(99, "UPTIME", buf);
  dashedHLine(2, SCR_W - 3, 103);

  drawKV(113, "HEAP", fmtK(ESP.getFreeHeap()));

  drawPageDots();
}

// ---- Screen 4: general info ----
void drawInfoScreen() {
  drawHeader("INFO");

  u8g2.setFont(FONT_MICRO);
  oStr(2, 17, "TOTAL");
  drawHeroValue(42, 20, fmtF(gE, 1), "kWh");
  oHLine(0, 44, SCR_W);

  drawKV(56, "PEAK W", fmtF(maxPower, 0));
  dashedHLine(2, SCR_W - 3, 60);

  drawKV(70, "SESSIONS", fmtF(sessionCount, 0));
  dashedHLine(2, SCR_W - 3, 74);

  drawStateRow(84, "WASH", sessionActive, "RUN", "IDLE");
  dashedHLine(2, SCR_W - 3, 88);

  drawKV(98, "HEAP", fmtK(ESP.getFreeHeap()));
  dashedHLine(2, SCR_W - 3, 102);

  drawKV(112, "FIRMWARE", "V7");

  drawPageDots();
}

// ---- renderer + slide transition --------------------------------
static void renderTo(uint8_t s) {
  gDrawScreen = s;
  switch (s) {
    case SCR_MAIN:   drawMainScreen();   break;
    case SCR_GRAPH:  drawGraphScreen();  break;
    case SCR_STATUS: drawStatusScreen(); break;
    case SCR_INFO:   drawInfoScreen();   break;
  }
}

void renderScreen() {
  gOX = 0;
  u8g2.clearBuffer();
  renderTo(currentScreen);
  u8g2.sendBuffer();
}

// Slides `next` in from the right (dir = +1) or from the left (dir = -1)
static void slideToScreen(uint8_t next, int8_t dir) {
  if (next == currentScreen) { renderScreen(); return; }

  const int steps = 5;
  for (int i = 1; i <= steps; i++) {
    int off = (SCR_W * i) / steps;
    u8g2.clearBuffer();
    gOX = -off * dir;          renderTo(currentScreen);
    gOX = (SCR_W - off) * dir; renderTo(next);
    gOX = 0;
    u8g2.sendBuffer();
    yield();
  }
  currentScreen = next;
  gDrawScreen   = next;
}

// ============================================================
//  Backlight (PWM on LCD_LED)
//  Full brightness while someone is at the meter, faded down after
//  a stretch with no touch. Any touch brings it straight back.
// ============================================================
void backlightWake() {
  blLastActivity = millis();
  blTarget = LCD_BRIGHTNESS;
}

void handleBacklight() {
  unsigned long now = millis();

  if (blTarget != LCD_DIM_LEVEL && now - blLastActivity >= LCD_DIM_TIMEOUT)
    blTarget = LCD_DIM_LEVEL;

  if (blCurrent != blTarget && now - blLastFade >= LCD_FADE_MS) {
    blLastFade = now;
    if (blCurrent < blTarget) {
      blCurrent += LCD_FADE_STEP;
      if (blCurrent > blTarget) blCurrent = blTarget;
    } else {
      blCurrent -= LCD_FADE_STEP;
      if (blCurrent < blTarget) blCurrent = blTarget;
    }
    analogWrite(LCD_LED, blCurrent);
  }
}

// ============================================================
//  Touch handling
//    tap        -> next screen (animated)
//    long press -> back to the main screen
//  A touch on a dimmed display only wakes it, so the first tap in the
//  dark never changes the screen out from under you.
// ============================================================
void handleTouch() {
  int s = digitalRead(TOUCH_PIN);
  unsigned long now = millis();

  if (s != lastTouchState && (now - lastTouchTime) > TOUCH_DEBOUNCE) {
    lastTouchTime  = now;
    lastTouchState = s;

    if (s == HIGH) {                          // press
      touchWokeDisplay = (blTarget == LCD_DIM_LEVEL);
      touchDownTime    = now;
      longPressFired   = false;
      backlightWake();
    } else {                                  // release
      backlightWake();
      if (!longPressFired && !touchWokeDisplay)
        slideToScreen((currentScreen + 1) % SCREEN_COUNT, +1);
    }
  }

  if (s == HIGH && !longPressFired && (now - touchDownTime) >= TOUCH_LONG_PRESS) {
    longPressFired = true;
    backlightWake();
    if (currentScreen != SCR_MAIN) slideToScreen(SCR_MAIN, -1);
  }
}

// ============================================================
//  PZEM reading + power history
// ============================================================
void pushPower(float p) {
  if (isnan(p) || p < 0) p = 0;
  for (int k = 0; k < GRAPH_POINTS - 1; k++) powerHist[k] = powerHist[k + 1];
  powerHist[GRAPH_POINTS - 1] = p;
  if (histCount < GRAPH_POINTS) histCount++;
}

void readSensors() {
  gV  = pzem.voltage();
  gI  = pzem.current();
  gP  = pzem.power();
  gE  = pzem.energy();
  gF  = pzem.frequency();
  gPF = pzem.pf();

  if (!isnan(gP) && gP > maxPower) maxPower = gP;
  pushPower(gP);
  updateSession(gP, gE);

  int rssi = WiFi.RSSI();
  Serial.print("V:");    Serial.print(gV, 1);
  Serial.print(" | I:"); Serial.print(gI, 2);
  Serial.print(" | P:"); Serial.print(gP, 1);
  Serial.print(" | E:"); Serial.print(gE, 2);
  Serial.print(" | F:"); Serial.print(gF, 1);
  Serial.print(" | PF:");Serial.print(gPF, 2);
  Serial.print(" | RSSI:"); Serial.print(rssi); Serial.println(" dBm");
}

// ============================================================
//  MQTT
// ============================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) message += (char)payload[i];

  String resetTopic = String(config.mqtt_topic) + "/reset";
  if (String(topic) == resetTopic && message == "RESET") {
    Serial.println("Resetting energy counter...");
    backlightWake();
    drawBanner("RESET", "Resetting counter", -1);

    bool ok = pzem.resetEnergy();
    drawBanner("RESET", ok ? "Reset OK" : "Reset failed", -1);
    delay(1500);

    mqtt.publish((String(config.mqtt_topic) + "/reset_status").c_str(),
                 ok ? "SUCCESS" : "FAILED");
    if (ok) maxPower = 0;
  }
}

// Unique node id for Home Assistant (stable per chip)
String haNodeId() { return "energymeter_" + String(ESP.getChipId(), HEX); }
String availTopic() { return String(config.mqtt_topic) + "/status"; }

// Publish a Home Assistant MQTT-discovery config for one sensor (retained).
// Abbreviated keys keep each payload small enough for the client buffer.
void publishHaSensor(const char* comp, const char* id, const char* name,
                     const String &stateTopic, const char* unit,
                     const char* devClass, const char* stateClass,
                     const char* onPl = nullptr, const char* offPl = nullptr) {
  String node  = haNodeId();
  String topic = String("homeassistant/") + comp + "/" + node + "/" + id + "/config";
  String p = "{";
  p += "\"name\":\"" + String(name) + "\"";
  p += ",\"uniq_id\":\"" + node + "_" + id + "\"";
  p += ",\"stat_t\":\"" + stateTopic + "\"";
  p += ",\"avty_t\":\"" + availTopic() + "\"";
  if (unit && strlen(unit))             p += ",\"unit_of_meas\":\"" + String(unit) + "\"";
  if (devClass && strlen(devClass))     p += ",\"dev_cla\":\"" + String(devClass) + "\"";
  if (stateClass && strlen(stateClass)) p += ",\"stat_cla\":\"" + String(stateClass) + "\"";
  if (onPl)  p += ",\"pl_on\":\""  + String(onPl)  + "\"";
  if (offPl) p += ",\"pl_off\":\"" + String(offPl) + "\"";
  p += ",\"dev\":{\"ids\":[\"" + node + "\"],\"name\":\"" + String(config.hostname) +
       "\",\"mdl\":\"ESP12F PZEM-004T\",\"mf\":\"AmirY\"}";
  p += "}";
  mqtt.publish(topic.c_str(), p.c_str(), true);   // retained
}

void publishDiscovery() {
  String b = config.mqtt_topic;
  publishHaSensor("sensor", "voltage",   "Voltage",   b + "/voltage",   "V",   "voltage",      "measurement");
  publishHaSensor("sensor", "current",   "Current",   b + "/current",   "A",   "current",      "measurement");
  publishHaSensor("sensor", "power",     "Power",     b + "/power",     "W",   "power",        "measurement");
  publishHaSensor("sensor", "energy",    "Energy",    b + "/energy",    "kWh", "energy",       "total_increasing");
  publishHaSensor("sensor", "frequency", "Frequency", b + "/frequency", "Hz",  "frequency",    "measurement");
  publishHaSensor("sensor", "pf",        "Power Factor", b + "/pf",     "",    "power_factor", "measurement");
  // Washing session
  publishHaSensor("binary_sensor", "washing", "Washing", b + "/session", "", "running", "", "ON", "OFF");
  publishHaSensor("sensor", "session_energy", "Session Energy", b + "/session_energy", "kWh", "energy", "measurement");
  Serial.println("HA discovery published");
}

void reconnectMQTT() {
  if (mqtt.connected()) return;
  Serial.print("Connecting to MQTT...");
  String clientId = "EnergyMeter-" + String(ESP.getChipId());
  String avty = availTopic();
  // Last Will: broker marks us offline if the connection drops
  if (mqtt.connect(clientId.c_str(), config.mqtt_user, config.mqtt_pass,
                   avty.c_str(), 0, true, "offline")) {
    Serial.println("Connected!");
    mqtt.publish(avty.c_str(), "online", true);       // retained availability
    String resetTopic = String(config.mqtt_topic) + "/reset";
    mqtt.subscribe(resetTopic.c_str());
    publishDiscovery();
  } else {
    Serial.print("Failed, rc=");
    Serial.println(mqtt.state());
  }
}

void publishMQTT() {
  if (!mqtt.connected()) return;
  String base = config.mqtt_topic;
  mqtt.publish((base + "/voltage").c_str(),   String(gV, 1).c_str());
  mqtt.publish((base + "/current").c_str(),   String(gI, 2).c_str());
  mqtt.publish((base + "/power").c_str(),     String(gP, 1).c_str());
  mqtt.publish((base + "/energy").c_str(),    String(gE, 2).c_str());
  mqtt.publish((base + "/frequency").c_str(), String(gF, 1).c_str());
  mqtt.publish((base + "/pf").c_str(),        String(gPF, 2).c_str());
  // Washing session state for Home Assistant
  mqtt.publish((base + "/session").c_str(), sessionActive ? "ON" : "OFF", true);
  float se = sessionActive ? (gE - curEnergyStart) : lastSessionEnergy;
  if (se < 0) se = 0;
  mqtt.publish((base + "/session_energy").c_str(), String(se, 3).c_str(), true);
  Serial.println("MQTT data sent!");
}

// ============================================================
//  Web server
// ============================================================
const char INDEX_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Energy Meter</title>
<style>
:root{--bg:#0f1420;--card:#1b2130;--acc:#3ba3ff;--txt:#e6ecf5;--mut:#8a97ad;--ok:#38d39f;--bad:#ff5d6c}
*{box-sizing:border-box}body{margin:0;font-family:system-ui,Segoe UI,Roboto,sans-serif;background:var(--bg);color:var(--txt)}
header{padding:16px;text-align:center;font-size:20px;font-weight:600;border-bottom:1px solid #26304a}
.tabs{display:flex;background:var(--card);position:sticky;top:0}
.tabs button{flex:1;padding:14px;border:0;background:transparent;color:var(--mut);font-size:15px;cursor:pointer;border-bottom:3px solid transparent}
.tabs button.active{color:var(--txt);border-bottom-color:var(--acc)}
.wrap{max-width:520px;margin:0 auto;padding:16px}
.panel{display:none}.panel.active{display:block}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.card{background:var(--card);border-radius:12px;padding:14px}
.card .lbl{color:var(--mut);font-size:12px;text-transform:uppercase;letter-spacing:.5px}
.card .val{font-size:26px;font-weight:600;margin-top:4px}
.card .unit{font-size:14px;color:var(--mut);margin-left:4px}
label{display:block;margin:12px 0 4px;color:var(--mut);font-size:13px}
input{width:100%;padding:11px;border-radius:9px;border:1px solid #2c3550;background:#141a28;color:var(--txt);font-size:15px}
button.act{width:100%;margin-top:18px;padding:13px;border:0;border-radius:9px;background:var(--acc);color:#02101f;font-size:16px;font-weight:600;cursor:pointer}
button.warn{background:var(--bad);color:#fff}
.row{display:flex;justify-content:space-between;padding:9px 0;border-bottom:1px solid #26304a}
.row:last-child{border:0}.row .k{color:var(--mut)}.dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:6px}
.msg{margin-top:12px;padding:10px;border-radius:8px;background:#12351f;color:var(--ok);display:none}
.hint{color:var(--mut);font-size:12px;margin-top:6px}
table{width:100%;border-collapse:collapse;font-size:13px}
th{color:var(--mut);text-align:left;font-weight:500;padding:10px 12px;border-bottom:1px solid #26304a}
td{padding:9px 12px;border-bottom:1px solid #232c44}
tr:last-child td{border:0}td.r,th.r{text-align:right}
.empty{padding:22px;text-align:center;color:var(--mut)}
</style></head><body>
<header id="hdr">⚡ Energy Meter</header>
<div class="tabs">
  <button class="active" onclick="tab(0)">Status</button>
  <button onclick="tab(1)">Sessions</button>
  <button onclick="tab(2)">WiFi</button>
  <button onclick="tab(3)">MQTT</button>
</div>
<div class="wrap">

  <div class="panel active" id="p0">
    <div class="grid">
      <div class="card"><div class="lbl">Voltage</div><div class="val"><span id="v">-</span><span class="unit">V</span></div></div>
      <div class="card"><div class="lbl">Current</div><div class="val"><span id="i">-</span><span class="unit">A</span></div></div>
      <div class="card"><div class="lbl">Power</div><div class="val"><span id="p">-</span><span class="unit">W</span></div></div>
      <div class="card"><div class="lbl">Energy</div><div class="val"><span id="e">-</span><span class="unit">kWh</span></div></div>
      <div class="card"><div class="lbl">Frequency</div><div class="val"><span id="f">-</span><span class="unit">Hz</span></div></div>
      <div class="card"><div class="lbl">Power Factor</div><div class="val"><span id="pf">-</span></div></div>
    </div>
    <div class="card" style="margin-top:12px">
      <div class="row"><span class="k">WiFi</span><span><span class="dot" id="wd"></span><span id="ssid">-</span></span></div>
      <div class="row"><span class="k">IP</span><span id="ip">-</span></div>
      <div class="row"><span class="k">RSSI</span><span id="rssi">-</span></div>
      <div class="row"><span class="k">MQTT</span><span><span class="dot" id="md"></span><span id="mq">-</span></span></div>
      <div class="row"><span class="k">Uptime</span><span id="up">-</span></div>
      <div class="row"><span class="k">Free heap</span><span id="heap">-</span></div>
    </div>
    <div class="card" style="margin-top:12px">
      <div class="row"><span class="k">Wash session</span><span><span class="dot" id="sd"></span><span id="sstate">-</span></span></div>
      <div class="row"><span class="k">This session</span><span id="senergy">-</span></div>
      <div class="row"><span class="k">Peak</span><span id="speak">-</span></div>
    </div>
    <button class="act warn" onclick="resetEnergy()">Reset Energy Counter</button>
    <div class="msg" id="m0"></div>
  </div>

  <div class="panel" id="p1">
    <div class="card" style="padding:0"><table id="stbl"><thead><tr>
      <th>Start</th><th>Dur</th><th>kWh</th><th>Peak</th></tr></thead>
      <tbody id="sbody"></tbody></table></div>
    <div class="hint" id="scount">-</div>
    <button class="act warn" onclick="clearSessions()">Clear History</button>
    <div class="msg" id="m3"></div>
  </div>

  <div class="panel" id="p2">
    <div class="card">
      <div class="row"><span class="k">Status</span><span><span class="dot" id="wd2"></span><span id="ssid2">-</span></span></div>
      <div class="row"><span class="k">IP</span><span id="ip2">-</span></div>
      <div class="row"><span class="k">MAC</span><span id="mac">-</span></div>
      <div class="row"><span class="k">RSSI</span><span id="rssi2">-</span></div>
    </div>
    <label>Device name (shown in your router)</label>
    <input id="whost" placeholder="EnergyMeter">
    <label>Network name (SSID)</label>
    <input id="wssid" placeholder="Your WiFi SSID">
    <label>Password</label>
    <input id="wpass" type="password" placeholder="Leave empty to keep current">
    <div class="hint">Name change fully applies after a reboot. Changing SSID reconnects now.</div>
    <button class="act" onclick="saveWifi()">Save &amp; Connect</button>
    <div class="msg" id="m1"></div>
  </div>

  <div class="panel" id="p3">
    <label>Broker address</label><input id="msrv" placeholder="192.168.1.175">
    <label>Port</label><input id="mport" type="number" placeholder="1883">
    <label>Username</label><input id="muser" placeholder="mqtt_user">
    <label>Password</label><input id="mpass" type="password" placeholder="Leave empty to keep current">
    <label>Base topic</label><input id="mtopic" placeholder="home/energy">
    <div class="hint">Publishes to &lt;topic&gt;/voltage, /power, ... Reset via &lt;topic&gt;/reset</div>
    <button class="act" onclick="saveMqtt()">Save MQTT Settings</button>
    <div class="msg" id="m2"></div>
  </div>

</div>
<script>
function tab(n){
  document.querySelectorAll('.tabs button').forEach((b,i)=>b.classList.toggle('active',i==n));
  document.querySelectorAll('.panel').forEach((p,i)=>p.classList.toggle('active',i==n));
}
function fmt(x,d){return (x==null||isNaN(x))?'-':Number(x).toFixed(d);}
function show(id,txt){var m=document.getElementById(id);m.textContent=txt;m.style.display='block';setTimeout(()=>m.style.display='none',3000);}
function dt(ep){ if(!ep||ep<1000000000)return 'n/a'; var d=new Date(ep*1000);
  return d.toLocaleDateString([], {day:'2-digit',month:'2-digit'})+' '+d.toLocaleTimeString([], {hour:'2-digit',minute:'2-digit'}); }
function dur(s){ var h=Math.floor(s/3600),m=Math.floor((s%3600)/60); return h>0?(h+'h'+String(m).padStart(2,'0')):(m+'m'); }
async function refresh(){
  try{
    const s=await (await fetch('/api/status')).json();
    hdr.textContent='⚡ '+(s.host||'Energy Meter');
    v.textContent=fmt(s.v,0); i.textContent=fmt(s.i,2); p.textContent=fmt(s.p,1);
    e.textContent=fmt(s.e,1); f.textContent=fmt(s.f,1); pf.textContent=fmt(s.pf,2);
    ssid.textContent=s.ssid; ssid2.textContent=s.ssid; ip.textContent=s.ip; ip2.textContent=s.ip;
    rssi.textContent=s.rssi+' dBm'; rssi2.textContent=s.rssi+' dBm'; mac.textContent=s.mac;
    up.textContent=s.up; heap.textContent=s.heap+' B';
    mq.textContent=s.mqtt?'Connected':'Offline';
    wd.style.background=wd2.style.background=s.wifi?'var(--ok)':'var(--bad)';
    md.style.background=s.mqtt?'var(--ok)':'var(--bad)';
    sstate.textContent=s.session?'RUNNING':'Idle';
    sd.style.background=s.session?'var(--ok)':'var(--mut)';
    senergy.textContent=fmt(s.sess_energy,3)+' kWh';
    speak.textContent=fmt(s.sess_peak,0)+' W';
  }catch(err){}
}
async function loadSessions(){
  try{
    const a=await (await fetch('/api/sessions')).json();
    const b=document.getElementById('sbody'); b.innerHTML='';
    if(!a.length){ b.innerHTML='<tr><td colspan="4" class="empty">No sessions recorded yet</td></tr>'; }
    let tot=0;
    a.forEach(s=>{ tot+=s.energy;
      b.insertAdjacentHTML('beforeend',
        '<tr><td>'+dt(s.start)+'</td><td class="r">'+dur(s.dur)+'</td><td class="r">'+
        fmt(s.energy,3)+'</td><td class="r">'+fmt(s.peak,0)+'</td></tr>'); });
    document.getElementById('scount').textContent=a.length+' sessions  ·  total '+fmt(tot,2)+' kWh';
  }catch(err){}
}
async function loadCfg(){
  try{
    const c=await (await fetch('/api/config')).json();
    msrv.value=c.mqtt_server; mport.value=c.mqtt_port; muser.value=c.mqtt_user; mtopic.value=c.mqtt_topic;
    whost.value=c.hostname; wssid.value=c.ssid;
  }catch(err){}
}
async function post(url,data,msgid,ok){
  const b=new URLSearchParams(data);
  const r=await fetch(url,{method:'POST',body:b});
  show(msgid, r.ok?ok:'Error');
}
function saveWifi(){post('/api/wifi',{hostname:whost.value,ssid:wssid.value,pass:wpass.value},'m1','Saved');}
function saveMqtt(){post('/api/mqtt',{server:msrv.value,port:mport.value,user:muser.value,pass:mpass.value,topic:mtopic.value},'m2','MQTT settings saved');}
function resetEnergy(){if(confirm('Reset the energy counter?'))post('/api/reset_energy',{},'m0','Reset command sent');}
async function clearSessions(){ if(confirm('Delete all saved sessions?')){ await post('/api/sessions/clear',{},'m3','History cleared'); loadSessions(); } }
loadCfg();refresh();loadSessions();setInterval(refresh,2000);setInterval(loadSessions,15000);
</script>
</body></html>)HTML";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  char up[24];
  formatUptime(up, sizeof(up));
  bool wl = (WiFi.status() == WL_CONNECTED);
  String j = "{";
  j += "\"v\":"    + String(isnan(gV) ? 0 : gV, 1);
  j += ",\"i\":"   + String(isnan(gI) ? 0 : gI, 2);
  j += ",\"p\":"   + String(isnan(gP) ? 0 : gP, 1);
  j += ",\"e\":"   + String(isnan(gE) ? 0 : gE, 2);
  j += ",\"f\":"   + String(isnan(gF) ? 0 : gF, 1);
  j += ",\"pf\":"  + String(isnan(gPF) ? 0 : gPF, 2);
  j += ",\"wifi\":"  + String(wl ? "true" : "false");
  j += ",\"ssid\":\"" + WiFi.SSID() + "\"";
  j += ",\"ip\":\""   + WiFi.localIP().toString() + "\"";
  j += ",\"mac\":\""  + WiFi.macAddress() + "\"";
  j += ",\"rssi\":"   + String(wl ? WiFi.RSSI() : 0);
  j += ",\"mqtt\":"   + String(mqtt.connected() ? "true" : "false");
  j += ",\"heap\":"   + String(ESP.getFreeHeap());
  j += ",\"up\":\""   + String(up) + "\"";
  j += ",\"host\":\"" + String(config.hostname) + "\"";
  j += ",\"time\":"   + String((uint32_t)time(nullptr));
  j += ",\"synced\":" + String(timeSynced() ? "true" : "false");
  // live session
  j += ",\"session\":" + String(sessionActive ? "true" : "false");
  float liveE = 0;
  if (sessionActive) { liveE = gE - curEnergyStart; if (liveE < 0) liveE = 0; }
  j += ",\"sess_energy\":" + String(liveE, 3);
  j += ",\"sess_peak\":"   + String(sessionActive ? curPeak : 0, 0);
  j += "}";
  server.send(200, "application/json", j);
}

void handleSessions() {
  String j = "[";
  // most recent first
  for (int k = sessionCount - 1; k >= 0; k--) {
    if (k != sessionCount - 1) j += ",";
    WashSession &s = sessions[k];
    j += "{\"start\":"   + String(s.start);
    j += ",\"end\":"     + String(s.end);
    j += ",\"dur\":"     + String(s.duration);
    j += ",\"energy\":"  + String(s.energy, 3);
    j += ",\"peak\":"    + String(s.peak, 0);
    j += "}";
  }
  j += "]";
  server.send(200, "application/json", j);
}

void handleClearSessions() {
  clearSessions();
  server.send(200, "text/plain", "OK");
}

void handleGetConfig() {
  String j = "{";
  j += "\"hostname\":\""    + String(config.hostname) + "\"";
  j += ",\"mqtt_server\":\"" + String(config.mqtt_server) + "\"";
  j += ",\"mqtt_port\":"    + String(config.mqtt_port);
  j += ",\"mqtt_user\":\""  + String(config.mqtt_user) + "\"";
  j += ",\"mqtt_topic\":\"" + String(config.mqtt_topic) + "\"";
  j += ",\"ssid\":\""       + WiFi.SSID() + "\"";
  j += "}";   // NOTE: passwords are intentionally never sent back
  server.send(200, "application/json", j);
}

void handleSaveWifi() {
  // Hostname can be changed on its own (takes full effect after a reboot)
  if (server.hasArg("hostname") && server.arg("hostname").length() > 0) {
    strlcpy(config.hostname, server.arg("hostname").c_str(), sizeof(config.hostname));
    saveConfig();
    WiFi.hostname(config.hostname);
    ArduinoOTA.setHostname(config.hostname);
  }

  String ssid = server.arg("ssid");
  String pass = server.arg("pass");
  server.send(200, "text/plain", "OK");

  if (ssid.length() > 0) {
    Serial.printf("Switching WiFi to: %s\n", ssid.c_str());
    // If password field left blank, reuse the currently stored one
    if (pass.length() == 0) WiFi.begin(ssid.c_str(), WiFi.psk().c_str());
    else                    WiFi.begin(ssid.c_str(), pass.c_str());
  }
}

void handleSaveMqtt() {
  if (server.hasArg("server")) strlcpy(config.mqtt_server, server.arg("server").c_str(), sizeof(config.mqtt_server));
  if (server.hasArg("port"))   config.mqtt_port = (uint16_t)server.arg("port").toInt();
  if (server.hasArg("user"))   strlcpy(config.mqtt_user,  server.arg("user").c_str(),  sizeof(config.mqtt_user));
  if (server.hasArg("topic"))  strlcpy(config.mqtt_topic, server.arg("topic").c_str(), sizeof(config.mqtt_topic));
  // Only overwrite the password when a new one was actually provided
  if (server.hasArg("pass") && server.arg("pass").length() > 0)
    strlcpy(config.mqtt_pass, server.arg("pass").c_str(), sizeof(config.mqtt_pass));
  saveConfig();
  applyMqttConfig();
  server.send(200, "text/plain", "OK");
}

void handleResetEnergy() {
  bool ok = pzem.resetEnergy();
  if (ok) maxPower = 0;
  server.send(200, "text/plain", ok ? "OK" : "FAILED");
}

void setupWebServer() {
  server.on("/",               HTTP_GET,  handleRoot);
  server.on("/api/status",     HTTP_GET,  handleStatus);
  server.on("/api/config",     HTTP_GET,  handleGetConfig);
  server.on("/api/sessions",   HTTP_GET,  handleSessions);
  server.on("/api/sessions/clear", HTTP_POST, handleClearSessions);
  server.on("/api/wifi",       HTTP_POST, handleSaveWifi);
  server.on("/api/mqtt",       HTTP_POST, handleSaveMqtt);
  server.on("/api/reset_energy", HTTP_POST, handleResetEnergy);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();
  Serial.println("Web server started");
}

// ============================================================
//  OTA
// ============================================================
void setupOTA() {
  ArduinoOTA.setHostname(config.hostname);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() { backlightWake(); showOTAStatus("Starting..."); });
  ArduinoOTA.onEnd([]()   { showOTAStatus("Complete!"); delay(1000); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    showOTAStatus("Uploading...", progress / (total / 100));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    String m = "Error: ";
    if      (error == OTA_AUTH_ERROR)    m += "Auth";
    else if (error == OTA_BEGIN_ERROR)   m += "Begin";
    else if (error == OTA_CONNECT_ERROR) m += "Connect";
    else if (error == OTA_RECEIVE_ERROR) m += "Receive";
    else if (error == OTA_END_ERROR)     m += "End";
    showOTAStatus(m.c_str());
    delay(3000);
  });
  ArduinoOTA.begin();
  Serial.println("OTA Ready");
}

// ============================================================
//  Setup / Loop
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n=== Energy Meter V5 Started ===");

  // Touch input. On GPIO3 this also detaches the pin from UART0 RX,
  // switching it to plain GPIO (serial TX / debug output still works).
  pinMode(TOUCH_PIN, INPUT);

  // Filesystem + config
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed, formatting...");
    LittleFS.format();
    LittleFS.begin();
  }
  loadConfig();
  loadSessions();

  // LCD
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.setContrast(LCD_CONTRAST);
  pinMode(LCD_LED, OUTPUT);
  analogWriteRange(1023);
  analogWrite(LCD_LED, LCD_BRIGHTNESS);
  backlightWake();

  splashScreen();

  // WiFi (set hostname first so it appears in the router's DHCP list)
  showWiFiStatus("Starting WiFi...");
  WiFi.hostname(config.hostname);
  WiFiManager wm;
  wm.setConfigPortalTimeout(180);
  if (!wm.autoConnect("EnergyMeter-Setup")) {
    Serial.println("Failed to connect");
    showWiFiStatus("Connection Failed!");
    delay(3000);
    ESP.restart();
  }
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  showWiFiStatus("Connected!");

  // Real-time clock via NTP (for session timestamps)
  configTime(NTP_TZ, NTP_SERVER1, NTP_SERVER2);

  delay(1500);

  setupOTA();
  setupWebServer();

  // MQTT (larger buffer needed for HA discovery payloads)
  mqtt.setBufferSize(640);
  mqtt.setServer(config.mqtt_server, config.mqtt_port);
  mqtt.setCallback(mqttCallback);
  reconnectMQTT();
}

void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  mqtt.loop();
  handleTouch();
  handleBacklight();

  unsigned long now = millis();

  // Read sensors + refresh display
  if (now - lastRead >= READ_INTERVAL) {
    lastRead = now;
    readSensors();
    renderScreen();
  }

  // MQTT reconnect (spaced, non-blocking cadence)
  if (!mqtt.connected() && now - lastMqttReconnect >= MQTT_RECONNECT_INTERVAL) {
    lastMqttReconnect = now;
    reconnectMQTT();
  }

  // MQTT publish
  if (mqtt.connected() && now - lastMqttSend >= MQTT_INTERVAL) {
    lastMqttSend = now;
    publishMQTT();
  }
}
