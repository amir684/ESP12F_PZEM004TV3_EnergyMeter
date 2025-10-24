# ⚡ ESP12F Energy Meter – OTA + MQTT + Reset

**Version:** V4  
**Author:** Amir684684
**Year:** 2025  

---

## 📖 Overview

This project is a **smart energy monitoring system** built around an **ESP8266 (ESP-12F)** microcontroller.  
It reads data from a **PZEM-004T V3** energy meter module, displays the information on an **ST7567 128×64 LCD**,  
and publishes live readings via **MQTT** for integration with Home Assistant, Node-RED, or any MQTT-based dashboard.

### Main Capabilities
- Real-time display of voltage, current, power, frequency, power factor, and energy  
- Wi-Fi setup via **WiFiManager** captive portal  
- **OTA** (Over-The-Air) firmware updates  
- **MQTT** integration for live data streaming  
- Remote **energy counter reset** via MQTT command  

---

## 🔌 Hardware Connections

| Component | ESP8266 Pin | Description |
|------------|-------------|-------------|
| **PZEM-004T-V3 RX** | D1 (GPIO5) | TX from ESP |
| **PZEM-004T-V3 TX** | D2 (GPIO4) | RX to ESP |
| **LCD ST7567 CS** | GPIO0 | Chip Select |
| **LCD ST7567 DC** | GPIO2 | Data/Command |
| **LCD ST7567 RESET** | GPIO16 | LCD Reset |
| **LCD Backlight (LED)** | D6 (GPIO12) | PWM brightness control |
| **Power Supply** | 5V / GND | Common ground for all modules |

---

## 📡 MQTT Topics

| Topic | Description |
|-------|-------------|
| `home/energy/voltage` | Voltage (V) |
| `home/energy/current` | Current (A) |
| `home/energy/power` | Power (W) |
| `home/energy/energy` | Energy (kWh) |
| `home/energy/frequency` | Frequency (Hz) |
| `home/energy/pf` | Power Factor |
| `home/energy/reset` | Send `"RESET"` to clear energy counter |
| `home/energy/reset_status` | Returns `"SUCCESS"` or `"FAILED"` after reset |

---

## ⚙️ OTA Configuration

| Parameter | Value |
|------------|--------|
| Hostname | `EnergyMeter` |
| Password | `amir3080` |

To upload new firmware:
1. Connect your PC to the same Wi-Fi network as the device.  
2. In Arduino IDE, go to **Tools → Port**, and select `network:EnergyMeter.local`.  
3. Upload the sketch normally — OTA will handle the update and show progress on the LCD.

---

## 🧰 Required Libraries

Install these via **Arduino Library Manager**:

- [PZEM004Tv30](https://github.com/mandulaj/PZEM-004T-v30)
- [U8g2](https://github.com/olikraus/u8g2)
- [PubSubClient](https://github.com/knolleary/pubsubclient)
- [WiFiManager](https://github.com/tzapu/WiFiManager)
- [ArduinoOTA](https://arduino-esp8266.readthedocs.io/en/latest/ota_updates/readme.html)

---

## 📶 Network and MQTT Setup

- Default MQTT broker: `192.168.1.175:1883`
- Default credentials:
  - Username: `mqtt_user`
  - Password: `amir3080`
- The device automatically opens a configuration portal named:
