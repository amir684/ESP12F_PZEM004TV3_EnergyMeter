/*
  ---------------------------------------------------------------
  Project: ESP12F Energy Meter with OTA + MQTT + Reset
  Version: V4
  Author: Amir684684
  Date: 2025
  ---------------------------------------------------------------

  ⚙️ Description:
  This project implements a smart energy meter using an ESP8266 (ESP-12F)
  that reads electrical parameters from a PZEM-004T-V3 module,
  displays them on an ST7567 128x64 LCD, and publishes the readings
  to an MQTT broker.  

  The device supports:
    - OTA firmware updates (Over-The-Air)
    - WiFiManager captive portal setup
    - MQTT data publishing
    - Remote energy counter reset via MQTT command

  ---------------------------------------------------------------
  🔌 Hardware Wiring:

  🧠 ESP12F (ESP8266):
    • D1 (GPIO5)  → TX of PZEM-004T-V3
    • D2 (GPIO4)  → RX of PZEM-004T-V3
    • D6 (GPIO12) → LCD backlight control (PWM)
    • GPIO0 (CS)  → ST7567 LCD Chip Select
    • GPIO2 (DC)  → ST7567 LCD Data/Command
    • GPIO16 (RST)→ ST7567 LCD Reset
    • SPI (CLK, MOSI) shared hardware SPI lines to LCD

  ⚡ Energy Meter Module: PZEM-004T-V3
    • RX ↔ D1 (TX from ESP)
    • TX ↔ D2 (RX to ESP)
    • 5V and GND to suitable power source
    • Measurement inputs connected to AC line and load

  🖥️ Display: ST7567 128x64 LCD
    • Connected via hardware SPI
    • Backlight controlled via PWM on D6
    • Contrast and rotation adjustable in code

  🌐 Communication:
    • WiFi configuration via WiFiManager (AP name: “EnergyMeter-Setup”)
    • MQTT broker sends data for voltage, current, power, energy, frequency, PF
    • MQTT reset command: `home/energy/reset` → message “RESET”
    • OTA updates enabled with password protection

  ---------------------------------------------------------------
  🧩 Required Libraries:
    - ArduinoOTA
    - WiFiManager
    - PubSubClient
    - U8g2lib
    - PZEM004Tv30
    - SoftwareSerial

  ---------------------------------------------------------------
  📊 MQTT Topics:
    home/energy/voltage
    home/energy/current
    home/energy/power
    home/energy/energy
    home/energy/frequency
    home/energy/pf
    home/energy/reset         ← send “RESET” to reset energy counter
    home/energy/reset_status  ← returns “SUCCESS” or “FAILED”

  ---------------------------------------------------------------
*/

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <PZEM004Tv30.h>
#include <U8g2lib.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>

// ---------------- הגדרות משתמש ----------------
#define LCD_LED         D6
#define LCD_BRIGHTNESS  500
#define LCD_CONTRAST    20
#define LCD_ROTATION    U8G2_R3
#define FONT_LABEL      u8g2_font_6x12_tr
#define FONT_VALUE      u8g2_font_7x13B_tr
#define LINE_GAP        21
#define REFRESH_DELAY   2000
#define MQTT_INTERVAL   5000

// ---------------- הגדרות OTA ----------------
#define OTA_HOSTNAME    "EnergyMeter"
#define OTA_PASSWORD    "12345678"

// ---------------- הגדרות MQTT ----------------
#define MQTT_SERVER     "192.168.1.175"
#define MQTT_PORT       1883
#define MQTT_USER       "mqtt_user"
#define MQTT_PASS       "amir3080"
#define MQTT_TOPIC      "home/energy"
#define MQTT_RESET_TOPIC "home/energy/reset"

// ---------------- LCD ST7567 ----------------
U8G2_ST7567_JLX12864_F_4W_HW_SPI u8g2(LCD_ROTATION, /* cs=*/ 0, /* dc=*/ 2, /* reset=*/ 16);

// ---------------- PZEM ----------------
#define PZEM_RX D2
#define PZEM_TX D1
SoftwareSerial pzemSW(PZEM_RX, PZEM_TX);
PZEM004Tv30 pzem(pzemSW);

// ---------------- WiFi & MQTT ----------------
WiFiClient espClient;
PubSubClient mqtt(espClient);
unsigned long lastMqttSend = 0;

// ---------------- פונקציות עזר ----------------

void splashScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_9x15B_tr);
  u8g2.setCursor(4, 15);
  u8g2.print("Energy");
  u8g2.setCursor(8, 35);
  u8g2.print("Meter");
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.setCursor(2, 70);
  u8g2.print("Amir Yahud");
  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.setCursor(25, 90);
  u8g2.print("2025");
  u8g2.sendBuffer();
  delay(3000);
}

void showWiFiStatus(const char* msg) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.setCursor(0, 15);
  u8g2.print("WiFi Status:");
  u8g2.setCursor(0, 35);
  u8g2.print(msg);
  u8g2.sendBuffer();
}

void showOTAStatus(const char* msg, int progress = -1) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.setCursor(0, 15);
  u8g2.print("OTA Update");
  u8g2.setCursor(0, 35);
  u8g2.print(msg);
  
  if (progress >= 0) {
    u8g2.drawFrame(0, 50, 60, 10);
    u8g2.drawBox(2, 52, (progress * 56) / 100, 6);
    u8g2.setCursor(20, 75);
    u8g2.print(progress);
    u8g2.print("%");
  }
  
  u8g2.sendBuffer();
}

void setupOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {
      type = "filesystem";
    }
    Serial.println("Start updating " + type);
    showOTAStatus("Starting...");
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    showOTAStatus("Complete!");
    delay(1000);
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    int percent = (progress / (total / 100));
    Serial.printf("Progress: %u%%\r", percent);
    showOTAStatus("Uploading...", percent);
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    String errorMsg = "Error: ";
    if (error == OTA_AUTH_ERROR) errorMsg += "Auth Failed";
    else if (error == OTA_BEGIN_ERROR) errorMsg += "Begin Failed";
    else if (error == OTA_CONNECT_ERROR) errorMsg += "Connect Failed";
    else if (error == OTA_RECEIVE_ERROR) errorMsg += "Receive Failed";
    else if (error == OTA_END_ERROR) errorMsg += "End Failed";
    
    Serial.println(errorMsg);
    showOTAStatus(errorMsg.c_str());
    delay(3000);
  });
  
  ArduinoOTA.begin();
  Serial.println("OTA Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void drawParameter(int &y, const char* label, float value, const char* unit, int decimals = 1) {
  u8g2.setFont(FONT_LABEL);
  u8g2.setCursor(0, y + 2);
  u8g2.print(label);
  u8g2.setFont(FONT_VALUE);
  u8g2.setCursor(0, y + 14);
  u8g2.print(isnan(value) ? 0 : value, decimals);
  u8g2.print(" ");
  u8g2.print(unit);
  y += LINE_GAP;
}

// ★★★ Callback למסרים נכנסים מ-MQTT ★★★
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);
  
  // בדיקה אם זה פקודת איפוס
  if (String(topic) == MQTT_RESET_TOPIC && message == "RESET") {
    Serial.println("Resetting energy counter...");
    
    // הצגה על המסך
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_7x13B_tr);
    u8g2.setCursor(5, 30);
    u8g2.print("Resetting");
    u8g2.setCursor(5, 50);
    u8g2.print("Counter...");
    u8g2.sendBuffer();
    
    // איפוס המונה ב-PZEM
    if (pzem.resetEnergy()) {
      Serial.println("Energy counter reset successfully!");
      
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_7x13B_tr);
      u8g2.setCursor(10, 40);
      u8g2.print("Reset OK!");
      u8g2.sendBuffer();
      delay(2000);
      
      // שליחת אישור חזרה ל-MQTT
      mqtt.publish((String(MQTT_TOPIC) + "/reset_status").c_str(), "SUCCESS");
    } else {
      Serial.println("Reset failed!");
      
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_7x13B_tr);
      u8g2.setCursor(5, 40);
      u8g2.print("Reset Failed");
      u8g2.sendBuffer();
      delay(2000);
      
      mqtt.publish((String(MQTT_TOPIC) + "/reset_status").c_str(), "FAILED");
    }
  }
}

void reconnectMQTT() {
  if (!mqtt.connected()) {
    Serial.print("Connecting to MQTT...");
    String clientId = "EnergyMeter-" + String(ESP.getChipId());
    
    if (mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
      Serial.println("Connected!");
      // ★★★ הרשמה לטופיק האיפוס ★★★
      mqtt.subscribe(MQTT_RESET_TOPIC);
      Serial.print("Subscribed to: ");
      Serial.println(MQTT_RESET_TOPIC);
    } else {
      Serial.print("Failed, rc=");
      Serial.println(mqtt.state());
    }
  }
}

void publishMQTT(float v, float i, float p, float e, float f, float pf) {
  if (!mqtt.connected()) {
    reconnectMQTT();
  }
  
  if (mqtt.connected()) {
    mqtt.publish((String(MQTT_TOPIC) + "/voltage").c_str(), String(v, 1).c_str());
    mqtt.publish((String(MQTT_TOPIC) + "/current").c_str(), String(i, 2).c_str());
    mqtt.publish((String(MQTT_TOPIC) + "/power").c_str(), String(p, 1).c_str());
    mqtt.publish((String(MQTT_TOPIC) + "/energy").c_str(), String(e, 2).c_str());
    mqtt.publish((String(MQTT_TOPIC) + "/frequency").c_str(), String(f, 1).c_str());
    mqtt.publish((String(MQTT_TOPIC) + "/pf").c_str(), String(pf, 2).c_str());
    
    Serial.println("MQTT data sent!");
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n=== Energy Meter Started ===");

  // Init LCD
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.setContrast(LCD_CONTRAST);

  // תאורה אחורית
  pinMode(LCD_LED, OUTPUT);
  analogWriteRange(1023);
  analogWrite(LCD_LED, LCD_BRIGHTNESS);

  splashScreen();

  // ============ WiFi Manager ============
  showWiFiStatus("Starting WiFi...");
  
  WiFiManager wm;
  wm.setConfigPortalTimeout(180);
  
  bool res = wm.autoConnect("EnergyMeter-Setup");
  
  if (!res) {
    Serial.println("Failed to connect");
    showWiFiStatus("Connection Failed!");
    delay(3000);
    ESP.restart();
  } else {
    Serial.println("Connected to WiFi!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    
    showWiFiStatus("Connected!");
    delay(2000);
  }

  // ============ OTA Setup ============
  setupOTA();

  // ============ MQTT Setup ============
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);  // ★★★ חשוב! הגדרת callback ★★★
  reconnectMQTT();
}

void loop() {
  // טיפול ב-OTA
  ArduinoOTA.handle();
  
  // שמירה על חיבור MQTT
  if (!mqtt.connected()) {
    reconnectMQTT();
  }
  mqtt.loop();  // ★★★ חשוב לטיפול במסרים נכנסים ★★★

  // קריאה מה-PZEM
  float v  = pzem.voltage();
  float i  = pzem.current();
  float p  = pzem.power();
  float e  = pzem.energy();
  float f  = pzem.frequency();
  float pf = pzem.pf();

  // קריאה טרייה של RSSI
  WiFi.mode(WIFI_STA);
  int currentRSSI = WiFi.RSSI();
  
  // הדפסה לסיריאל
  Serial.print("V:"); Serial.print(v, 1);
  Serial.print(" | I:"); Serial.print(i, 2);
  Serial.print(" | P:"); Serial.print(p, 1);
  Serial.print(" | E:"); Serial.print(e, 2);
  Serial.print(" | F:"); Serial.print(f, 1);
  Serial.print(" | PF:"); Serial.print(pf, 2);
  Serial.print(" | RSSI:"); Serial.print(currentRSSI);
  Serial.print(" dBm (");
  
  if (currentRSSI >= -50) Serial.print("Excellent");
  else if (currentRSSI >= -60) Serial.print("Good");
  else if (currentRSSI >= -70) Serial.print("Fair");
  else if (currentRSSI >= -80) Serial.print("Weak");
  else Serial.print("Poor");
  Serial.println(")");

  // עדכון מסך
  u8g2.clearBuffer();
  
  int y = 8;
  drawParameter(y, "Voltage:", v, "V", 0);
  drawParameter(y, "Current:", i, "A", 2);
  drawParameter(y, "Power:", p, "W");
  drawParameter(y, "Energy:", e, "kWh", 2);
  drawParameter(y, "Freq:", f, "Hz");
  drawParameter(y, "PF:", pf, "", 2);

  // אייקון WiFi בפינה
  if (WiFi.status() == WL_CONNECTED) {
    int rssi = WiFi.RSSI();
    int bars = 0;
    
    if (rssi >= -55) bars = 4;
    else if (rssi >= -65) bars = 3;
    else if (rssi >= -75) bars = 2;
    else if (rssi >= -85) bars = 1;
    
    int startX = 50;
    int startY = 2;
    int maxHeight = 8;
    
    for (int i = 0; i < 4; i++) {
      int h = (i + 1) * 2;
      if (i < bars) {
        u8g2.drawBox(startX + i * 3, startY + (maxHeight - h), 2, h);
      } else {
        u8g2.drawFrame(startX + i * 3, startY + (maxHeight - h), 2, h);
      }
    }
    
    if (mqtt.connected()) {
      u8g2.setFont(u8g2_font_4x6_tr);
      u8g2.setCursor(startX, startY + maxHeight + 6);
      u8g2.print("MQ");
    }
  }

  u8g2.sendBuffer();

  // שליחת MQTT
  if (millis() - lastMqttSend >= MQTT_INTERVAL) {
    publishMQTT(v, i, p, e, f, pf);
    lastMqttSend = millis();
  }

  delay(REFRESH_DELAY);
}
