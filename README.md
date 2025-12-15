# SurRon StormBee / Greenway BMS ESP32 Interface  
Firmware for ESP32-U / ESP32-WROOM  
Licensed under **GNU GPLv3**

This project provides an ESP32-based Bluetooth interface for reading data from the **Greenway battery** used in the **SurRon StormBee** motorcycle.  
It exposes live BMS data through a **local web dashboard**, sends telemetry to **MQTT**, and allows charge control through a **configurable charge-stop threshold (75–100%)** with optional remote MQTT commands.

The goal of this project is to give StormBee owners an open and flexible tool to monitor and manage their battery during charging.

---

## 🚀 Features

### 🔵 **Greenway BMS Bluetooth Interface**
- Automatic BLE connection to Greenway BMS  
- Reads:
  - State of Charge (SOC)
  - Pack Voltage
  - Current
  - Cell Voltages
  - Temperatures (T1, T2, Mos)
  - SOH, cycle count
  - Inter-cell delta
  
### 🌐 **Built-in Web Interface**
- Hosted directly on the ESP32  
- Real-time dashboard  
- Charge threshold configuration (75–100%)  
- Manual / MQTT control mode  
- Cell detail page  
- Network & MQTT configuration pages  
- Factory Reset & Debug options available via Serial Terminal

### 📡 **MQTT Integration**
- Publishes all major BMS data to MQTT topics  
- Allows external control:
  - Enable / Disable charging
  - Change maximum charge percentage  
- Compatible with:
  - Home Assistant
  - Node-RED
  - Domoticz
  - Any MQTT client

### 📶 **Access Point at First Boot**
On first boot (or after factory reset), the ESP32:
1. Starts in **Wi-Fi Access Point** mode  SSID "Stormbee_Setup"
2. Hosts a configuration webpage  AP IP: 192.168.4.1
3. Stores settings   
4. Reboots into normal mode once configured  

### ⚡ **Charging Control**
- Automatic stop at configured SOC limit  
- Relay output control (optional)  PIN 2
- Manual override mode  
- MQTT-based authorization mode

---

## 🛠️ Requirements

### **Arduino IDE Configuration**
- Board: **ESP32 Dev Module**
- Partition Scheme: **Huge APP**
- Flash Frequency: **80 MHz** (recommended)

### **Libraries**
Install the following:

| Library        | Author          | Version |
|----------------|-----------------|---------|
| PubSubClient   | Nick O'Leary    | 2.8.0   |

### **ESP32 Arduino Core**
Recommended version:  
**3.3.3**

---

## 📡 MQTT Usage

### **Published topics:**
stormbee/charge/relay_status
stormbee/bms/status
stormbee/bms/cells

### **Subscribed commands:**

stormbee/charge/set_power  ON/OFF
stormbee/charge/set_limit  0 to 100

## 🔧 First-time Setup

1. Power the ESP32  
2. It automatically starts in **Access Point mode**  
3. Connect to the ESP32 Wi-Fi network  
4. Open the configuration page  
5. Enter:
   - Wi-Fi credentials 
    auto reboot
6. Go to "config" page on web interface and configure BLE and MQTT(optional)  
   - Charge limit percentage  
7. The ESP32 connects normally and starts sending data

## 🔁 Factory Reset / Debug

Available from:
- **Serial Monitor** (menu with commands)


## 📜 License

This project is released under the **GNU General Public License v3.0 (GPLv3)**.  
Any modified or redistributed version **must remain open-source** under the same license.


## ⚠️ Disclaimer

This project is **not affiliated** with SurRon, Greenway, or any manufacturer.  
Use this firmware at your own risk.  

## Screenshots

<p align="center">
  <img src="IMG/front_web_01.png" width="400">
</p>
<p align="center">
  <img src="IMG/front_web_02.png" width="400">
</p>
<p align="center">
  <img src="IMG/front_web_03.png" width="400">
</p>
