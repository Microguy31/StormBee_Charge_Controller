#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h> 
#include <Preferences.h> 
#include <PubSubClient.h> 
#include <esp_task_wdt.h> 

// --- PINS --
const int RELAY_PIN = 2; // Pin connected to the charging relay (Active LOW)
#define RXD2 16
#define TXD2 17

// --- LED PINS (STATUS INDICATORS) ---
const int LED_RELAY = 25;  // Relay status indicator
const int LED_WIFI = 26;   // WiFi status indicator
const int LED_BT = 27;     // BLE connection status indicator (flashes on data)
const int LED_MQTT = 32;   // MQTT connection status indicator
const int LED_ZIGBEE = 33; // Zigbee connection status indicator

// ==================================================================
// GLOBAL VARIABLES AND OBJECTS
// ==================================================================
WebServer server(80);
WiFiClient espClient;
PubSubClient mqttClient(espClient);
Preferences preferences;
HardwareSerial SerialZigbee(2); 
DNSServer dnsServer; 

// --- BLUETOOTH CONFIGURATION ---
String targetDeviceMAC = ""; // MAC address of the target BMS
static BLEUUID SVC_UUID("0000fff0-0000-1000-8000-00805f9b34fb"); // Service UUID
static BLEUUID CHR_UUID("0000fff1-0000-1000-8000-00805f9b34fb"); // Characteristic UUID (Data)
static BLEUUID DSC_CCCD("00002902-0000-1000-8000-00805f9b34fb"); // Client Characteristic Configuration Descriptor

// BMS registers to request in rotation (Greenway protocol)
uint8_t registersToRequest[] = {0x09, 0x0A, 0x08, 0x24, 0x25, 0x17}; 
int requestIndex = 0;
unsigned long lastRequestTime = 0;
const unsigned long REQUEST_INTERVAL_MS = 500; // Interval between requests

static BLEClient* pClient = nullptr;
static BLERemoteCharacteristic* chrData = nullptr;
static bool connected = false; // BLE connection status
bool isScanning = false;
String scanResults = ""; // Used to display BLE scan results on the web page
String wifiScanResults = ""; // Used to display WiFi scan results on the web page

// --- WIFI NON-BLOCKING RECONNECT ---
unsigned long lastWifiAttempt = 0;
#define WIFI_RETRY_INTERVAL 30000UL // Time between WiFi reconnection attempts (30s)

// --- LED BLE FLASH ---
bool bluetoothDataReceived = false; 
unsigned long lastBleFlashTime = 0;
const unsigned long BLE_FLASH_DURATION = 100; // Duration of the BLE LED flash

// --- WATCHDOG TIMER ---
const int WDT_TIMEOUT_S = 45; // Watchdog timeout in seconds

// --- CONFIGURATION SAVED IN PREFERENCES ---
String wifi_ssid = "";
String wifi_pass = "";
String mqtt_server = "";
String mqtt_port = "1883";
String mqtt_user = "";
String mqtt_pass = "";
String lang = "fr"; // Default language
bool isApMode = false; // Is the ESP in Access Point mode?
bool debugMode = false; // Serial debug mode toggle

// --- SYSTEM STATE ---
int chargeTargetPercent = 80; // User-defined charge limit SOC
bool chargeMasterSwitch = true; // Main charging ON/OFF switch
bool modeMQTT = false; // Control mode: false=Manual/Web, true=MQTT
bool relayActive = false; // Current state of the physical relay (true=Charging, false=Off)
bool useZigbee = false; // Reserved for Zigbee functionality
bool zigbeeConnected = false; 
bool mqttForceOff = false; // Flag to force relay off via MQTT command

unsigned long lastZigbeeHeartbeat = 0;
unsigned long lastPublish = 0;
const long publishInterval = 10000; // Interval for MQTT/Zigbee data publication (10s)

// --- BMS DATA STRUCTURE ---
struct BmsData {
    float totalVoltage = 0.0;
    float current = 0.0;
    uint8_t soc = 0;
    uint8_t soh = 0;
    uint16_t cycleCount = 0;
    float remainingCapacity = 0.0;
    float fullCapacity = 0.0;
    int tempT1 = 0;
    int tempT2 = 0;
    int tempMOS = 0;
    float minVoltage = 0.0;
    float maxVoltage = 0.0;
    uint8_t minCellIndex = 0;
    uint8_t maxCellIndex = 0;
    float cellVolts[32];
    unsigned long lastUpdateTime = 0; // Timestamp of the last successful data reception
} bmsData;

// --- DECODING VARIABLES ---
static uint8_t main_packet_shift_byte = 0;
static bool expect_suite_24 = false;
static bool expect_suite_25 = false;

// --- MQTT TOPICS ---
const char* topic_status = "stormbee/bms/status";
const char* topic_cells  = "stormbee/bms/cells";
const char* topic_set_limit = "stormbee/charge/set_limit"; // Topic to set SOC limit
const char* topic_set_power = "stormbee/charge/set_power"; // Topic to force charge ON/OFF

// --- FreeRTOS Task Handle for BLE
TaskHandle_t hBleTask = NULL; 

// ==================================================================
// UTILITY FUNCTIONS
// ==================================================================

/**
 * @brief Translates a string based on the current language setting.
 * @param fr French string.
 * @param en English string.
 * @return Translated string.
 */
String tr(String fr, String en) { return (lang == "fr") ? fr : en; }

/**
 * @brief Retrieves a 32-bit Little-Endian value from a byte array.
 */
template <typename T>
T get32BitLEValue(const uint8_t* data, size_t index) {
    if (index + 3 >= 20) return 0;
    uint32_t raw = data[index] | (data[index + 1] << 8) | (data[index + 2] << 16) | (data[index + 3] << 24);
    return (T)raw;
}

/**
 * @brief Updates the state of the LED status indicators.
 */
void updateLeds() {
    digitalWrite(LED_RELAY, relayActive ? HIGH : LOW);

    if (isApMode) {
        digitalWrite(LED_WIFI, HIGH); 
    } else {
        digitalWrite(LED_WIFI, WiFi.status() == WL_CONNECTED ? HIGH : LOW);
    }
    
    // BLE LED flashes briefly upon data reception
    if (connected) {
        if (bluetoothDataReceived) {
            digitalWrite(LED_BT, HIGH);
            lastBleFlashTime = millis(); 
            bluetoothDataReceived = false;
        } else if (millis() - lastBleFlashTime < BLE_FLASH_DURATION) {
            digitalWrite(LED_BT, HIGH);
        } else {
            digitalWrite(LED_BT, LOW);
        }
    } else {
        digitalWrite(LED_BT, LOW);
    }

    digitalWrite(LED_MQTT, (mqtt_server != "" && mqttClient.connected()) ? HIGH : LOW);
    digitalWrite(LED_ZIGBEE, zigbeeConnected ? HIGH : LOW);
}

// ==================================================================
// BMS DATA DECODING
// ==================================================================

/**
 * @brief Decodes the cell voltage packets (registers 0x24, 0x25 and their follow-up packets).
 */
void printDetailedCellTable(int cellStartNum, int cellEndNum, const uint8_t* data, size_t startDataIndex, size_t dataMaxIndex, uint8_t shiftLSB, bool isShiftedBlock) {
    size_t currentDataIndex = startDataIndex;
    int currentCell = cellStartNum;

    // Handle the first cell of a follow-up packet (shifted due to the shift byte)
    if (isShiftedBlock) {
        uint8_t msb = data[0];
        uint16_t val_mv = (msb << 8) | shiftLSB;
        bmsData.cellVolts[currentCell - 1] = (float)val_mv / 1000.0;
        currentCell++;
        currentDataIndex++;
    }

    // Decode remaining cells in the packet
    for (size_t i = currentDataIndex; i < dataMaxIndex; i += 2) {
        if (currentCell > cellEndNum) break;
        uint8_t lsb = data[i];
        uint8_t msb = data[i + 1];
        uint16_t val_mv = (msb << 8) | lsb;
        if (!isShiftedBlock && i + 1 >= dataMaxIndex) break;
        bmsData.cellVolts[currentCell - 1] = (float)val_mv / 1000.0;
        currentCell++;
    }
}

/**
 * @brief Prints detailed BMS data to the serial console for debugging.
 */
void printBMSDataDebug() {
    if(!debugMode || bmsData.lastUpdateTime == 0) return;

    Serial.println("=============================================");
    Serial.println("         STORM BEE BMS DATA DEBUG            ");
    Serial.println("=============================================");
    Serial.printf("| Voltage: %.2fV | Current: %.2fA | SOC: %d%% |\n", 
                  bmsData.totalVoltage, bmsData.current, bmsData.soc);
    Serial.printf("| SOH: %d%% | Cycles: %d | Capacity (Wh): %.0f |\n",
                  bmsData.soh, bmsData.cycleCount, bmsData.fullCapacity);
    Serial.printf("| Temps (C): T1=%d | T2=%d | MOS=%d |\n",
                  bmsData.tempT1, bmsData.tempT2, bmsData.tempMOS);
    Serial.printf("| Min V: %.3fV (C%02d) | Max V: %.3fV (C%02d) | Delta: %.3fV |\n",
                  bmsData.minVoltage, bmsData.minCellIndex + 1, bmsData.maxVoltage, bmsData.maxCellIndex + 1,
                  bmsData.maxVoltage - bmsData.minVoltage);
    Serial.println("=============================================");
    Serial.println("       CELL VOLTAGES (V) :        ");
    Serial.println("---------------------------------------------");

    for(int i=0; i<4; i++) {
        Serial.printf("| ");
        for(int j=0; j<7; j++) {
            int cellIndex = i*7 + j;
            if(cellIndex < 28) {
                if(cellIndex == bmsData.minCellIndex || cellIndex == bmsData.maxCellIndex) Serial.print(">>"); else Serial.print(" ");
                Serial.printf("C%02d:%.3f ", cellIndex + 1, bmsData.cellVolts[cellIndex]);
                if(cellIndex == bmsData.minCellIndex || cellIndex == bmsData.maxCellIndex) Serial.print("<<|"); else Serial.print("  |");
            } else {
                Serial.print("                     |");
            }
        }
        Serial.println();
    }
    Serial.println("---------------------------------------------");
}


/**
 * @brief Main function for decoding incoming BLE notification data.
 */
void decodeGreenway(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* data, size_t len, bool isNotify) {
    bluetoothDataReceived = true; 
    
    if (len < 6) return;
    uint8_t regId = data[3];
    bmsData.lastUpdateTime = millis();
    
    if(debugMode) { 
        Serial.printf("PKT %02X (len %d) RAW: ", regId, len); 
        for(int i=0;i<len;i++) Serial.printf("%02X ",data[i]); 
        Serial.println(); 
    }

    if (regId == 0x09 && len >= 9) {
        int32_t vRaw = get32BitLEValue<int32_t>(data + 5, 0);
        bmsData.totalVoltage = (float)vRaw / 1000.0;
        if (debugMode) Serial.printf("[09] Total Voltage: %.2f V\n", bmsData.totalVoltage);
    } 
    else if (regId == 0x0A && len >= 18) {
        int32_t iRaw = get32BitLEValue<int32_t>(data + 5, 0);
        bmsData.current = (float)iRaw / 1000.0;
        bmsData.soc = data[9];
        bmsData.soh = data[13];
        uint16_t cap = data[17] | (data[18] << 8);
        bmsData.fullCapacity = (float)cap; 
        
        if (debugMode) Serial.printf("[0A] Current: %.2f A | SOC: %d %% | SOH: %d %% | Capacity: %.0f Wh\n", 
                                     bmsData.current, bmsData.soc, bmsData.soh, bmsData.fullCapacity);
    } 
    else if (regId == 0x08 && len >= 9) { 
        bmsData.tempT1 = (int8_t)data[5];
        bmsData.tempT2 = (int8_t)data[7]; 
        bmsData.tempMOS = (int8_t)data[9]; 
        if (debugMode) Serial.printf("[08] Temperatures: T1=%d, T2=%d, MOS=%d C\n", 
                                     bmsData.tempT1, bmsData.tempT2, bmsData.tempMOS);
    }
    else if (regId == 0x17 && len >= 7) {
        uint16_t cycleCountRaw = data[5] | (data[6] << 8); 
        bmsData.cycleCount = cycleCountRaw;
        
        if (debugMode) Serial.printf("[17] Cycle Count: %d\n", bmsData.cycleCount);
    } 
    else if (regId == 0x25 && len >= 20) {
        // Cells 17-23 (first packet)
        printDetailedCellTable(17, 23, data, 5, 19, 0, false);
        main_packet_shift_byte = data[19];
        expect_suite_24 = false; expect_suite_25 = true; 
    }
    else if (regId == 0x24 && len >= 20) {
        // Cells 1-7 (first packet)
        printDetailedCellTable(1, 7, data, 5, 19, 0, false);
        main_packet_shift_byte = data[19];
        expect_suite_24 = true; expect_suite_25 = false;
    }
    else if (len == 18) {
        if (expect_suite_25) {
            // Cells 24, 25-28 and Min/Max V (follow-up packet for 0x25)
            printDetailedCellTable(24, 24, data, 0, 18, main_packet_shift_byte, true);
            printDetailedCellTable(25, 28, data, 1, 9, 0, false); 
            uint16_t maxV = data[13] | (data[14] << 8);
            uint16_t minV = data[15] | (data[16] << 8);
            bmsData.maxVoltage = (float)maxV / 1000.0;
            bmsData.minVoltage = (float)minV / 1000.0;
            expect_suite_25 = false;
        }
        else if (expect_suite_24) {
            // Cells 8, 9-16 (follow-up packet for 0x24)
            printDetailedCellTable(8, 8, data, 0, 18, main_packet_shift_byte, true);
            printDetailedCellTable(9, 16, data, 1, 17, 0, false); 
            expect_suite_24 = false;
        }
    }

    // Determine absolute Min/Max cell voltage and indices
    float minV=10.0, maxV=0.0;
    for(int i=0; i<28; i++) {
        float v = bmsData.cellVolts[i];
        if(v > 0.1) {
            if(v < minV) { minV = v; bmsData.minCellIndex = i; }
            if(v > maxV) { maxV = v; bmsData.maxCellIndex = i; }
        }
    }
}

// ==================================================================
// CHARGE CONTROL LOGIC
// ==================================================================
/**
 * @brief Checks BMS SOC against the target limit and controls the relay.
 */
void checkChargeControl() {
    bool desiredState = false;

    // 1. Master Switch OFF always means OFF
    if (!chargeMasterSwitch) {
        desiredState = false;
    } 
    else {
        // 2. Control logic based on SOC limit
        if (chargeTargetPercent == 100) {
            desiredState = true; // No limit, charge until BMS/external factors stop it
        } 
        else {
            if (bmsData.soc >= (chargeTargetPercent + 1)) {
                desiredState = false; // Over the limit + 1% hysteresis -> stop charging
            }
            else if (bmsData.soc <= (chargeTargetPercent - 1)) {
                desiredState = true; // Under the limit - 1% hysteresis -> start charging
            }
            else {
                desiredState = relayActive; // Maintain current state within the hysteresis band
            }
        }
        
        // 3. MQTT Force OFF override
        if (modeMQTT && mqttForceOff) {
            desiredState = false;
        }
    }

    // Apply the new state if different
    if (desiredState != relayActive) {
        Serial.printf("RELAY STATE CHANGE: %s -> %s (SOC=%d, Limit=%d, Master=%s, Mode=%s)\n",
                      relayActive ? "ON" : "OFF", desiredState ? "ON" : "OFF",
                      bmsData.soc, chargeTargetPercent, chargeMasterSwitch ? "ON" : "OFF", modeMQTT ? "MQTT" : "MANUAL");

        digitalWrite(RELAY_PIN, desiredState ? LOW : HIGH); // RELAY_PIN is Active LOW
        relayActive = desiredState;
    }
}

// ==================================================================
// WEB SERVER HANDLERS (I18N IMPLEMENTED)
// ==================================================================

/**
 * @brief Generates the common HTML header and CSS/JS for the web interface.
 */
String getHeader(String title) {
    String h = (String)F("<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><meta http-equiv='refresh' content='5'><title>") + title + (String)F("</title>");
    h += (String)F("<style>body{font-family:sans-serif;margin:0;padding:5px;background:#f0f2f5;color:#333}h1{font-size:1.5em;margin-top:0;margin-bottom:15px;text-align:center}.card{background:#fff;padding:15px;border-radius:12px;box-shadow:0 2px 8px rgba(0,0,0,0.05);margin-bottom:15px}.row{display:flex;justify-content:space-between;align-items:center;padding:10px 0;border-bottom:1px solid #eee}.row:last-child{border:none}.badge{flex:1;text-align:center;padding:8px 0;margin:0 2px;border-radius:6px;color:#fff;font-size:11px;font-weight:700}.btn{display:block;width:180px;max-width:100%;padding:7px;margin:8px auto;border:none;border-radius:6px;color:#fff;font-size:14px;text-align:center;text-decoration:none;cursor:pointer}.btn-grn{background:#34c759}.btn-blu{background:#007aff}.btn-gry{background:#8e8e93}.switch{position:relative;display:inline-block;width:50px;height:28px}.switch input{opacity:0;width:0;height:0}.slider{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background-color:#e9e9ea;border-radius:34px;transition:.4s}.slider:before{position:absolute;content:'';height:20px;width:20px;left:4px;bottom:4px;background-color:white;border-radius:50%;transition:.4s;box-shadow:0 2px 4px rgba(0,0,0,0.2)}input:checked + .slider{background-color:#34c759}input:checked + .slider:before{transform:translateX(22px)}input[type=text],input[type=password],input[type=number],select{width:100%;padding:10px;margin:5px 0 10px 0;display:block;border:1px solid #ccc;border-radius:4px;box-sizing:border-box}table{width:100%;border-collapse:collapse;margin-top:5px;font-size:13px}th,td{padding:5px;border-bottom:1px solid #eee;text-align:left}th{font-weight:700}</style>");
    h += (String)F("<script>function sw(ep,p){fetch('/'+ep+'?'+p);setTimeout(()=>location.reload(),500)}</script></head><body>");
    return h;
}

/**
 * @brief Handles the main status page (/).
 */
void handleRoot() {
    String html = getHeader(tr("Stormbee V102", "Stormbee V102"));
    html += (String)F("<h1>🔋 ") + tr("Contrôleur de Charge Stormbee", "Stormbee Charge Controller") + (String)F("</h1>");

    html += (String)F("<div class='card' style='display:flex'>");
    html += (String)F("<span class='badge' style='background:") + (relayActive?"#34c759":"#ff3b30") + (String)F("'>") + tr("RELAIS", "RELAY") + (String)F("</span>");
    html += (String)F("<span class='badge' style='background:") + (isApMode ? "#ffcc00" : (WiFi.status()==WL_CONNECTED?"#34c759":"#ff3b30")) + (String)F("'>WIFI</span>");
    html += (String)F("<span class='badge' style='background:") + (!useZigbee && mqttClient.connected()?"#34c759":"#8e8e93") + (String)F("'>MQTT</span>");
    html += (String)F("<span class='badge' style='background:") + (zigbeeConnected?"#34c759":"#ffcc00") + (String)F("'>ZIGBEE</span>");
    html += (String)F("<span class='badge' style='background:") + (connected?"#34c759":"#ff3b30") + (String)F("'>BT</span>");
    html += (String)F("</div>");

    html += (String)F("<div class='card'><div class='row'><b>") + tr("CHARGE (MAÎTRE)", "CHARGE (MASTER)") + (String)F("</b>");
    html += (String)F("<label class='switch'><input type='checkbox' onclick=\"sw('set_master','')\" ") + (chargeMasterSwitch?"checked":"") + (String)F("><span class='slider'></span></label></div></div>");

    html += (String)F("<div class='card'>");
    html += (String)F("<div class='row'><span>") + tr("Cible BMS", "BMS Target") + (String)F("</span><small>") + targetDeviceMAC + (String)F("</small></div>");
    
    // Status data display with I18N
    String status_V = (bmsData.lastUpdateTime > 0) ? String(bmsData.totalVoltage, 2) + (String)F(" V") : tr("N/A (Connexion en cours)", "N/A (Connecting)");
    String status_A = (bmsData.lastUpdateTime > 0) ? String(bmsData.current, 2) + (String)F(" A") : (String)F("N/A");
    String status_SOC = (bmsData.lastUpdateTime > 0) ? String(bmsData.soc) + (String)F(" %") : (String)F("N/A");
    String status_Temp = (bmsData.lastUpdateTime > 0) ? 
        String(bmsData.tempT1) + "/" + String(bmsData.tempT2) + "/" + String(bmsData.tempMOS) + (String)F(" °C") : 
        (String)F("N/A");
    
    html += (String)F("<div class='row'><span>") + tr("Tension", "Voltage") + (String)F("</span><b style='color:#007aff'>") + status_V + (String)F("</b></div>");
    html += (String)F("<div class='row'><span>") + tr("Courant", "Current") + (String)F("</span><b>") + status_A + (String)F("</b></div>");
    html += (String)F("<div class='row'><span>SOC</span><b>") + status_SOC + (String)F("</b></div>");
    html += (String)F("<div class='row'><span>") + tr("Temp (T1/T2/M)", "Temp (T1/T2/M)") + (String)F("</span><b>") + status_Temp + (String)F("</b></div>");
    
    html += (String)F("</div>");

    html += (String)F("<div class='card'>");
    html += (String)F("<div class='row'><span>") + tr("Mode (MAN/MQTT)", "Mode (MAN/MQTT)") + (String)F("</span><label class='switch'><input type='checkbox' onclick=\"sw('set_mode','')\" ") + (modeMQTT?"checked":"") + (String)F("><span class='slider'></span></label></div>");
    
    html += (String)F("<div class='row'><span>") + tr("Limite de Charge", "Charge Limit") + (String)F(" (") + chargeTargetPercent + (String)F("%)</span>");
    html += (String)F("<form action='/set_limit' method='GET' style='display:inline'><select name='val' onchange='this.form.submit()'>");
    int opts[] = {70,75,80,85,90,95,100};
    for(int i : opts) {
        html += (String)F("<option value='") + i + (String)F("'") + (chargeTargetPercent==i?" selected":"") + (String)F(">") + i + (String)F("%</option>");
    }
    html += (String)F("</select></form></div></div>");

    html += (String)F("<div class='card'>");
    html += (String)F("<a href='/cells' class='btn btn-grn'>") + tr("Détails Cellules", "Cell Details") + (String)F("</a>");
    html += (String)F("<a href='/config' class='btn btn-gry'>") + tr("Configuration", "Configuration") + (String)F("</a>");
    html += (String)F("</div></body></html>");
    server.send(200, "text/html", html);
}

/**
 * @brief Handles the cell details page (/cells).
 */
void handleCells() {
    String html = getHeader(tr("Détails des Cellules", "Cell Details"));
    html += (String)F("<h1>") + tr("Détails des Cellules", "Cell Details") + (String)F("</h1>");
    
    if (bmsData.lastUpdateTime == 0) {
         html += (String)F("<div class='card'><p>") + tr("Données non disponibles. Connexion Bluetooth en cours...", "Data not available. Bluetooth connecting...") + (String)F("</p></div>");
    } else {
        // FIX V102.38: Changed "Cycles" to "Cycle Count" for the English translation to prevent a subtle memory issue
        // caused by identical FR/EN strings in the previous version, which only manifested in the EN language.
        html += (String)F("<div class='card'><table><tr><th>SOH</th><th>") + tr("Cycles", "Cycle Count") + (String)F("</th><th>Delta</th></tr><tr>");
        html += (String)F("<td>") + bmsData.soh + (String)F("%</td><td>") + bmsData.cycleCount + (String)F("</td><td>") + String(bmsData.maxVoltage - bmsData.minVoltage, 3) + (String)F("V</td></tr></table></div>");
    
        html += (String)F("<div class='card'><h3>") + tr("Tensions (V)", "Voltages (V)") + (String)F("</h3><table style='width:100%; text-align:center;'>");
        for(int r=0; r<7; r++) { 
            html += (String)F("<tr>");
            for(int c=0; c<4; c++) { 
                int cellIndex = c*7 + r;
                if (cellIndex < 28) {
                    String colStyle = "";
                    if(cellIndex == bmsData.minCellIndex) colStyle = "color:#ff3b30;font-weight:bold"; 
                    else if(cellIndex == bmsData.maxCellIndex) colStyle = "color:#34c759;font-weight:bold"; 
                    
                    html += (String)F("<td style='padding:5px 0;'>C") + (cellIndex+1) + (String)F("</td>");
                    html += (String)F("<td style='padding:5px 0; ") + colStyle + (String)F("'>") + String(bmsData.cellVolts[cellIndex],3) + (String)F("</td>");
                }
            }
            html += (String)F("</tr>");
        }
        html += (String)F("</table></div>");
    }
    
    html += (String)F("<a href='/' class='btn btn-gry'>") + tr("Retour", "Back") + (String)F("</a>");
    server.send(200, "text/html", html);
}

/**
 * @brief Handles the configuration page (/config).
 */
void handleConfig() {
    String html = getHeader(tr("Configuration", "Configuration"));
    html += (String)F("<h1>") + tr("Configuration", "Configuration") + (String)F("</h1>");
    
    html += (String)F("<div class='card'><h3>Bluetooth</h3>");
    html += (String)F("<a href='/scan_ble' class='btn btn-blu'>") + tr("SCAN BLE", "SCAN BLE") + (String)F("</a>");
    if(scanResults != "") html += scanResults;
    html += (String)F("</div>");

    html += (String)F("<div class='card'><h3>WiFi</h3>");
    html += (String)F("<a href='/scan_wifi' class='btn btn-blu'>") + tr("SCAN WIFI", "SCAN WIFI") + (String)F("</a>"); 
    if(wifiScanResults != "") html += wifiScanResults; 
    wifiScanResults = ""; 
    html += (String)F("<form action='/save_wifi' method='POST'>");
    html += (String)F("<label>SSID:</label><input type='text' name='s' value='") + wifi_ssid + (String)F("'>");
    html += (String)F("<label>") + tr("PASS", "PASS") + (String)F(":</label><input type='password' name='p' value='") + wifi_pass + (String)F("'>");
    html += (String)F("<input type='submit' class='btn btn-grn' value='") + tr("SAUVEGARDER", "SAVE") + (String)F("'>");
    html += (String)F("</form></div>");

    html += (String)F("<div class='card'><h3>MQTT</h3><form action='/save_mqtt' method='POST'>");
    html += (String)F("<label>Server:</label><input type='text' name='s' value='") + mqtt_server + (String)F("'>");
    html += (String)F("<label>") + tr("Port", "Port") + (String)F(":</label><input type='number' name='p' value='") + mqtt_port + (String)F("'>");
    html += (String)F("<label>User:</label><input type='text' name='u' value='") + mqtt_user + (String)F("'>");
    html += (String)F("<label>") + tr("Pass", "Pass") + (String)F(":</label><input type='password' name='a' value='") + mqtt_pass + (String)F("'>");
    html += (String)F("<input type='submit' class='btn btn-grn' value='") + tr("SAUVEGARDER", "SAVE") + (String)F("'>");
    html += (String)F("</form></div>");

    html += (String)F("<div class='card'><h3>Zigbee</h3>");
    html += (String)F("<a href='/zigbee_join' class='btn btn-grn'>") + tr("PAIR / JOIN", "PAIR / JOIN") + (String)F("</a>");
    html += (String)F("<p>") + tr("Statut", "Status") + (String)F(": ") + (zigbeeConnected ? tr("CONNECTÉ", "CONNECTED") : tr("DÉCONNECTÉ", "DISCONNECTED")) + (String)F("</p>");
    html += (String)F("</div>");

    html += (String)F("<div class='card'><h3>") + tr("Langue", "Language") + (String)F("</h3>");
    html += (String)F("<a href='/set_lang?l=fr' class='badge' style='background:#007aff;display:inline-block;width:40px'>FR</a> ");
    html += (String)F("<a href='/set_lang?l=en' class='badge' style='background:#007aff;display:inline-block;width:40px'>EN</a></div>");
    
    html += (String)F("<a href='/' class='btn btn-gry'>") + tr("Retour", "Back") + (String)F("</a>");
    server.send(200, "text/html", html);
}

// --- CONFIGURATION HANDLERS ---
void handleSetMaster() { chargeMasterSwitch = !chargeMasterSwitch; checkChargeControl(); server.send(200, "text/plain", "OK"); }
void handleSetMode() { modeMQTT = !modeMQTT; checkChargeControl(); server.send(200, "text/plain", "OK"); }
void handleSetLimit() { chargeTargetPercent = server.arg("val").toInt(); preferences.putInt("lim", chargeTargetPercent); checkChargeControl(); server.sendHeader("Location","/"); server.send(303); }
void handleSetLang() { 
    lang = server.arg("l"); 
    preferences.putString("lang", lang); 
    server.sendHeader("Location","/config"); 
    server.send(303); 
}
void handleSaveWifi() { 
    preferences.putString("w_ssid", server.arg("s")); 
    preferences.putString("w_pass", server.arg("p")); 
    server.send(200,"text/plain","Saved. Rebooting..."); delay(1000); ESP.restart(); 
}
void handleSaveMqtt() { 
    preferences.putString("m_srv", server.arg("s")); 
    preferences.putString("m_port", server.arg("p")); 
    preferences.putString("m_user", server.arg("u")); 
    preferences.putString("m_pass", server.arg("a")); 
    server.send(200,"text/plain","MQTT Saved. Rebooting..."); delay(1000); ESP.restart(); 
}

void handleScanWifi() {
    wifiScanResults = "<ul>";
    int n = WiFi.scanNetworks();
    if (n == 0) {
        wifiScanResults += "<li>" + tr("Aucun réseau trouvé", "No network found") + (String)F("</li>");
    } else {
        for (int i = 0; i < n; ++i) {
            String ssid = WiFi.SSID(i);
            wifiScanResults += "<li>" + ssid + " (" + String(WiFi.RSSI(i)) + " dBm) " + 
                               "<a href='/sel_wifi?s=" + server.urlDecode(ssid) + "' class='badge' style='background:#007aff; display:inline; margin-left:10px;'>SELECT</a></li>";
        }
    }
    wifiScanResults += "</ul>";
    server.sendHeader("Location", "/config"); 
    server.send(303);
}

void handleSelWifi() {
    wifi_ssid = server.arg("s"); 
    server.sendHeader("Location", "/config"); 
    server.send(303);
}

void handleZigbeeJoin() { SerialZigbee.println("{\"cmd\":\"JOIN\"}"); server.sendHeader("Location","/config"); server.send(303); }

void handleScanBLE() {
    if(connected && pClient) { pClient->disconnect(); connected=false; }
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setActiveScan(true);
    BLEScanResults* foundDevices = pBLEScan->start(10, false); 
    
    scanResults = "<ul>";
    if (foundDevices) {
        for(int i=0; i<foundDevices->getCount(); i++) {
            BLEAdvertisedDevice d = foundDevices->getDevice(i);
            
            if (d.haveServiceUUID() && d.isAdvertisingService(SVC_UUID)) {
                 scanResults += "<li>" + String(d.getName().c_str()) + 
                               " [" + d.getAddress().toString().c_str() + "] " +
                               "(" + String(d.getRSSI()) + " dBm) " + 
                               "<a href='/sel_mac?m=" + d.getAddress().toString().c_str() + "'>SELECT</a></li>";
            }
        }
    }
    scanResults += "</ul>";
    if(scanResults == "<ul></ul>") scanResults = "<li>" + tr("Aucun appareil BMS pertinent trouvé.", "No relevant BMS device found.") + (String)F("</li>");
    
    pBLEScan->clearResults(); 
    server.sendHeader("Location","/config"); server.send(303);
}
void handleSelMac() { targetDeviceMAC = server.arg("m"); preferences.putString("mac", targetDeviceMAC); server.sendHeader("Location","/"); server.send(303); }

// ==================================================================
// BLE CALLBACKS
// ==================================================================
class CB : public BLEClientCallbacks {
  void onConnect(BLEClient* p) { connected=true; p->setMTU(256); Serial.println("BLE OK"); }
  void onDisconnect(BLEClient* p) { 
      connected=false; 
      chrData=nullptr; 
      bmsData.lastUpdateTime = 0; // Reset last data time to trigger connection attempts
      Serial.println("BLE LOST"); 
  }
};

// ==================================================================
// MQTT/ZIGBEE PUBLISHING
// ==================================================================
/**
 * @brief Formats BMS data into JSON and publishes it via MQTT or Zigbee.
 */
void publishData() {
    if(bmsData.lastUpdateTime == 0) return;
    
    printBMSDataDebug();

    char jsonStatus[512];
    snprintf(jsonStatus, sizeof(jsonStatus), 
        "{\"v\":%.2f,\"i\":%.2f,\"soc\":%d,\"soh\":%d,\"cap\":%.0f,\"cyc\":%d,\"t1\":%d,\"t2\":%d,\"tm\":%d,\"rly\":%d,\"lim\":%d}",
        bmsData.totalVoltage, bmsData.current, bmsData.soc, bmsData.soh, bmsData.fullCapacity, bmsData.cycleCount,
        bmsData.tempT1, bmsData.tempT2, bmsData.tempMOS, relayActive, chargeTargetPercent
    );

    String jsonCells = "{\"cells\":[";
    for(int i=0; i<28; i++) { jsonCells += String(bmsData.cellVolts[i], 3); if(i<27) jsonCells += ","; }
    jsonCells += "]}";

    if(useZigbee && zigbeeConnected) {
        SerialZigbee.println(jsonStatus);
        SerialZigbee.println(jsonCells);
        if(mqttClient.connected()) mqttClient.publish(topic_status, "{\"zb\":\"UP\",\"data\":\"UART\"}");
    } else {
        if(!useZigbee || (useZigbee && !zigbeeConnected)) {
            if(mqttClient.connected()) {
                 if(useZigbee) mqttClient.publish(topic_status, "{\"zb\":\"DOWN\"}"); 
                 mqttClient.publish(topic_status, jsonStatus);
                 mqttClient.publish(topic_cells, jsonCells.c_str());
            }
        }
    }
}

/**
 * @brief MQTT callback function to handle incoming commands.
 */
void mqttCb(char* topic, byte* payload, unsigned int length) {
    String msg; for(int i=0;i<length;i++) msg += (char)payload[i];
    if(String(topic) == topic_set_limit) {
        int v = msg.toInt(); 
        if(v>=50 && v<=100) { 
            chargeTargetPercent=v; 
            checkChargeControl(); 
        }
    }
    if(String(topic) == topic_set_power) {
        if(msg=="OFF") mqttForceOff=true; 
        else mqttForceOff=false;
        checkChargeControl();
    }
}

// ==================================================================
// DEDICATED BLE TASK (CORE 1)
// ==================================================================
/**
 * @brief FreeRTOS task running on Core 1 dedicated to BLE communication.
 * Manages connection, reconnection, and data request cycle.
 */
void ble_task_core1(void *parameter) {
    unsigned long lastTry = 0;
    while(1) {
        // Reset WDT for this task's main loop (outside blocking calls)
        esp_task_wdt_reset(); 

        if(!isScanning && !isApMode) {
            // --- RECONNECTION LOGIC ---
            if(!connected && targetDeviceMAC != "") {
                if(millis()-lastTry > 5000) {
                    Serial.printf("Connecting to BLE MAC: %s (Core 1)\n", targetDeviceMAC.c_str()); 
                    
                    if(pClient->isConnected()) {
                        pClient->disconnect();
                        delay(100); 
                    }
                    
                    // Reset WDT just before the potentially long-blocking pClient->connect() call
                    esp_task_wdt_reset(); 

                    // Connection attempt (this function is blocking, ~30s max)
                    if(pClient->connect(BLEAddress(targetDeviceMAC.c_str()))) {
                        BLERemoteService* s = pClient->getService(SVC_UUID);
                        if(s) {
                            chrData = s->getCharacteristic(CHR_UUID);
                            if(chrData && chrData->canNotify()) {
                                // Enable notifications
                                chrData->registerForNotify(decodeGreenway);
                                const uint8_t on[]={0x01,0x00};
                                chrData->getDescriptor(DSC_CCCD)->writeValue((uint8_t*)on,2,true);
                            }
                        }
                    } else {
                        Serial.println("BLE Connection Failed.");
                    }
                    lastTry=millis();
                }
            } 
            // --- ACTIVE CONNECTION LOGIC ---
            else if(connected && chrData) {
                
                // Check if the physical link was lost during active communication
                if (!pClient->isConnected()) {
                    Serial.println("BLE physical link lost (inside active loop). Forcing reconnect.");
                    connected = false;
                    chrData = nullptr;
                    bmsData.lastUpdateTime = 0; 
                    lastTry = millis(); 
                } else {
                    
                    // --- BMS Data Request Cycle ---
                    if(millis()-lastRequestTime > REQUEST_INTERVAL_MS) {
                        // Prepare the command (Greenway read register command)
                        uint8_t cmd[] = {0x46, 0x16, 0x01, registersToRequest[requestIndex%6], 0x20, 0x00};
                        uint8_t s=0; for(int i=0;i<5;i++)s+=cmd[i]; cmd[5]=s;
                        
                        // Reset WDT before the blocking writeValue() call.
                        // If BLE link is lost, this call blocks until NimBLE's internal timeout (~90s),
                        // triggering WDT alarm, but without fatal panic (as configured in setup()).
                        esp_task_wdt_reset(); 
                        chrData->writeValue(cmd, 6, true); 
                        
                        requestIndex++;
                        lastRequestTime=millis();
                    }

                    // --- Publishing and Control ---
                    if(millis()-lastPublish > publishInterval) { 
                        publishData(); 
                        lastPublish=millis(); 
                    }
                    checkChargeControl();
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Yield to FreeRTOS scheduler
    }
}

// ==================================================================
// SETUP (CORE 0)
// ==================================================================
void setup() {
    Serial.begin(115200);
    SerialZigbee.begin(115200, SERIAL_8N1, RXD2, TXD2);
    pinMode(RELAY_PIN, OUTPUT); 
    digitalWrite(RELAY_PIN, HIGH); // Initialize relay to OFF (Active LOW)
    
    // --- WATCHDOG TIMER SETUP ---
    esp_task_wdt_deinit(); 
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT_S * 1000, 
        .trigger_panic = false // V102.38: Non-fatal WDT on CPU 1 to prevent full system reboot.
    };
    esp_task_wdt_init(&wdt_config); 
    esp_task_wdt_add(NULL);         // Add LoopTask (Core 0)
    
    Serial.printf("Watchdog Timer initialized to %d seconds (V102.38 - Panic Mode OFF).\n", WDT_TIMEOUT_S);
    
    // --- LED INITIALIZATION ---
    pinMode(LED_RELAY, OUTPUT);
    pinMode(LED_WIFI, OUTPUT);
    pinMode(LED_BT, OUTPUT);
    pinMode(LED_MQTT, OUTPUT);
    pinMode(LED_ZIGBEE, OUTPUT);
    digitalWrite(LED_RELAY, LOW);
    digitalWrite(LED_WIFI, LOW);
    digitalWrite(LED_BT, LOW);
    digitalWrite(LED_MQTT, LOW);
    digitalWrite(LED_ZIGBEE, LOW);
    
    // --- LOAD SAVED PREFERENCES ---
    preferences.begin("sb", false);
    chargeTargetPercent = preferences.getInt("lim", 80);
    targetDeviceMAC = preferences.getString("mac", "");
    wifi_ssid = preferences.getString("w_ssid", "");
    wifi_pass = preferences.getString("w_pass", "");
    lang = preferences.getString("lang", "fr");
    mqtt_server = preferences.getString("m_srv", "");
    mqtt_port = preferences.getString("m_port", "1883");
    mqtt_user = preferences.getString("m_user", "");
    mqtt_pass = preferences.getString("m_pass", "");

    Serial.println("Stormbee V102.38 Started");
    Serial.println("-------------------------------------");
    Serial.println("AVAILABLE COMMANDS (Serial):");
    Serial.printf("  - debug    : Toggle debug mode (ON/OFF) [Current: %s]\n", debugMode ? "ON" : "OFF");
    Serial.println("  - recovery : Clear all preferences and restart.");
    Serial.println("-------------------------------------");

    // --- WIFI SETUP ---
    if(wifi_ssid == "") {
        isApMode = true;
        WiFi.softAP("Stormbee_Setup"); 
        dnsServer.start(53, "*", WiFi.softAPIP()); 
        Serial.println("AP MODE STARTED (Recovery/First Boot)");
        Serial.print("AP IP: "); Serial.println(WiFi.softAPIP()); 
        Serial.flush(); 
    }
    else {
        isApMode = false;
        WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());
        lastWifiAttempt = millis(); 
        Serial.println("WIFI STATION MODE STARTED (Background attempt)");
    }
    
    // --- MQTT SETUP ---
    if(!isApMode && mqtt_server != "") {
        mqttClient.setServer(mqtt_server.c_str(), mqtt_port.toInt());
        mqttClient.setCallback(mqttCb);
        mqttClient.setBufferSize(1024); 
    }

    // --- BLE SETUP ---
    BLEDevice::init("ESP32-Master");
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new CB());

    // --- START BLE TASK ON CORE 1 ---
    xTaskCreatePinnedToCore(
        ble_task_core1,
        "BLE_Task",
        10000, // Stack depth
        NULL,
        1, // Priority
        &hBleTask, 
        1 // Core 1
    );
    esp_task_wdt_add(hBleTask); // Add Core 1 to WDT

    // --- WEB SERVER ROUTING ---
    server.on("/", handleRoot);
    server.on("/config", handleConfig);
    server.on("/cells", handleCells);
    server.on("/scan_ble", handleScanBLE);
    server.on("/sel_mac", handleSelMac);
    server.on("/set_master", handleSetMaster);
    server.on("/set_mode", handleSetMode);
    server.on("/set_limit", handleSetLimit);
    server.on("/set_lang", handleSetLang);
    server.on("/save_wifi", HTTP_POST, handleSaveWifi);
    server.on("/zigbee_join", handleZigbeeJoin);
    server.on("/scan_wifi", handleScanWifi); 
    server.on("/sel_wifi", handleSelWifi); 
    server.on("/save_mqtt", HTTP_POST, handleSaveMqtt); 
    server.begin();
}

// ==================================================================
// MAIN LOOP (CORE 0)
// ==================================================================
void loop() {
    esp_task_wdt_reset(); // Reset WDT for the LoopTask (Core 0)
    
    if(isApMode) {
        dnsServer.processNextRequest(); 
    }
    server.handleClient(); // Handle web requests non-blockingly

    // --- WIFI & MQTT CONNECTIVITY MANAGEMENT ---
    if (!isApMode) {
        if (WiFi.status() == WL_CONNECTED) {
            static bool wasConnected = false;
            if (!wasConnected) {
                Serial.printf("WiFi CONNECTED. IP: %s\n", WiFi.localIP().toString().c_str());
                wasConnected = true;
            }
            if(mqtt_server != "") {
                if(!mqttClient.connected()) {
                    String id = "SB-" + WiFi.macAddress();
                    if(mqttClient.connect(id.c_str(), mqtt_user.c_str(), mqtt_pass.c_str())) {
                        mqttClient.subscribe(topic_set_limit);
                        mqttClient.subscribe(topic_set_power);
                    }
                }
                mqttClient.loop();
            }
        } else {
            // Attempt to reconnect to WiFi periodically
            static bool initialConnectionAttempt = true;
            if (initialConnectionAttempt || millis() - lastWifiAttempt > WIFI_RETRY_INTERVAL) {
                if (!initialConnectionAttempt) Serial.println("Re-attempting WiFi connection...");
                WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str()); 
                lastWifiAttempt = millis();
                initialConnectionAttempt = false; 
            }
        }
    }

    // --- SERIAL COMMANDS (DEBUG/RECOVERY) ---
    if(Serial.available()) {
        String s = Serial.readStringUntil('\n'); s.trim();
        if(s=="recovery") { preferences.clear(); ESP.restart(); }
        if(s=="debug") { 
            debugMode = !debugMode; 
            Serial.printf("Debug Mode is now %s\n", debugMode ? "ON" : "OFF");
        }
    }
    
    // --- ZIGBEE STATUS CHECK ---
    if(SerialZigbee.available()) {
        String s = SerialZigbee.readStringUntil('\n');
        if(s.indexOf("heartbeat") >= 0 || s.indexOf("zb_beat") >= 0) { // Check for heartbeat messages
            zigbeeConnected=true; 
            lastZigbeeHeartbeat=millis(); 
        }
    }
    // Timeout Zigbee connection if no heartbeat received for 30s
    if(millis() - lastZigbeeHeartbeat > 30000) zigbeeConnected=false;
    
    updateLeds(); 

    delay(10); // Standard loop delay
}
