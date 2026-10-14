#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_I2C_Multiplexer.h>
#include <SensirionI2cScd4x.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <HTTPClient.h>
#include <esp_sleep.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include <Adafruit_NeoPixel.h>
#include <WebServer.h>
#include <Preferences.h>
#include <time.h>
#include <sys/time.h>

// Firmware version
#define SW_VERSION "0.60"

// SET TO true TO OVERWRITE ALL STORED CONFIG VALUES (WIFI, SENSORS, INTERVALS) WITH THE DEFAULTS BELOW ONCE.
// MUST BE SET BACK TO false AFTER THE FIRST UPLOAD, OTHERWISE PORTAL UPDATES WILL BE OVERWRITTEN ON EVERY COLD BOOT!
bool forceUpdateDefaults = false;

// ==================== SECTION 1: TEXT-BUFFERS (PRIMARY CONFIGURATION) ====================
// Server Settings
char serverURL[256]       = "httpbin.org";
char serverResource[128]  = "/post";

// WiFi Connection
char wifiSSID[33]         = "your-wifi-ssid"; // WiFi SSIDs can be up to 32 chars + null terminator
char wifiPass[64]         = "your-wifi-password";

// LTE Specific Connection Buffers
char lteAPN[64]           = "iot.1nce.net";
char lteUser[32]          = "";
char ltePass[32]          = "";
char netModeStr[2]        = "0";      // 0 = WiFi Only, 1 = LTE-M, 2 = NB-IoT
char modemLedStr[2]       = "1";      // 1 = LED active, 0 = LED off

// Core Sensor and Timing Allocations
char sensorCountStr[4]    = "3";      // Active connected sensors (1-8)
char measSamplesStr[4]    = "3";      // Single-shot samples to average per cycle (after discarding first)
char sleepMinutesStr[6]   = "5";      // Deep Sleep interval duration
char maxMeasurementsStr[4]= "6";      // Cycles to buffer before upload
char sensorAltitudeStr[6] = "50";     // Height above sea level (NN) for live pressure correction

// System Diagnostic Flags
char serialDebugStr[2]    = "1";      // Serial console: 0=OFF, 1=ON (default), 2=ON + extended cellular
char rgbDebugStr[2]       = "1";      // Onboard NeoPixel LED flashes (1=ON, 0=OFF)

// Device Identification and Security Access Keys
char otaHostname[32]      = "beehive-co2-sensor";
char sharedCrypto[32]     = "openhive";        // Access key for AP Hotspot & OTA wireless flashing

// ==================== SECTION 2: MODEM / LTE CONFIGURATION ====================
#define TINY_GSM_MODEM_SIM7080
#include <TinyGsmClient.h>

#define MODEM_TX             14
#define MODEM_RX             13
#define MODEM_PWRKEY         26
#define uS_TO_S_FACTOR       1000000ULL

HardwareSerial SerialAT(2);   // ESP32 UART2 for AT commands to the SIM7080
TinyGsm modem(SerialAT);     // TinyGSM driver bound to SerialAT

// ==================== SECTION 3: HARDWARE PIN DEFINITIONS ====================
const uint8_t maxI2CDevices = 8;    // Limit boundary of physical TCA9548A hardware channels
const int batteryPin = 34;          // Onboard voltage divider connection pin mapping
#define SCD41_I2C_ADDR 0x62         // Sensirion SCD41 default I2C address
#define MULTIPLEXER_I2C_ADDR 0x70   // TCA9548A I2C multiplexer address
#define RGB_LED_PIN 5               // NeoPixel data pin
#define RGB_LED_COUNT 1             // Single status LED on board
#define USER_BUTTON_PIN 27          // Physical USER button (active LOW, portal skip)
#define NO_ERROR 0                  // Sensirion library success code
#define CO2_MIN_PLAUSIBLE 50        // Reject CO₂ samples below this [ppm] (near-zero = typical I2C/sensor glitch)
#define SCD41_GROUP_WAIT_MS 5000          // Shared wait after triggering a sensor group (parallel single-shots)
#define SCD41_DATA_READY_EXTRA_MS 2000    // Extra per-sensor data-ready poll after group wait (total max ~7 s)
#define SCD41_FRC_PREP_SHOTS 5            // Sensirion: 5 single-shots before FRC in single-shot / power-cycled mode
#define SCD41_FRC_PREP_INTERVAL_MS 60000UL // 1 minute between FRC prep measurements
#define SCD41_FRC_TARGET_FRESH_PPM 400    // Fresh-air FRC target (datasheet ASC default)
#define SCD41_FRC_TARGET_MIN_PPM 400      // Sensirion testing guide: known reference gas 400–2000 ppm
#define SCD41_FRC_TARGET_MAX_PPM 2000

// Maximum time window allocated for active user interaction (in seconds)
#define PORTAL_TIMEOUT_SECONDS 120

// ==================== SECTION 4: OPERATIONAL RUNTIME VARIABLES ====================
uint32_t sleepSeconds      = 0;
uint8_t activeSensors      = 0;
uint8_t measurementSamples = 3;       // Averaged samples per cycle (first shot always discarded)
uint16_t sensorAltitude    = 0;
bool systemDebugEnabled    = false;   // serialDebugLevel >= 1
bool cellularDebugEnabled  = false;   // serialDebugLevel == 2 (step-by-step modem connect logs)
bool rgbDebugEnabled       = false;
uint8_t serialDebugLevel   = 1;       // 0=OFF, 1=ON default, 2=ON + extended cellular
uint8_t telemetryMode      = 0;
bool enableModemLed        = true;
uint32_t maxMeasurements   = 6;
unsigned long lastPortalActivity = 0;
volatile bool cancelCalTriggered = false; // Tracks if the user pressed CANCEL in the browser

// Hardware driver object instances
DFRobot_I2C_Multiplexer I2CMulti(&Wire, MULTIPLEXER_I2C_ADDR);
SensirionI2cScd4x sensors[maxI2CDevices]; // Allocated safely matching upper hardware boundaries
Adafruit_NeoPixel rgbLed(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
WebServer server(80);
Preferences prefs;

// Accumulated log text sent to the browser calibration console via /cal-log
String calLogBuffer = "";

// Defined above the first function: the Arduino builder inserts prototypes
// before calLog(), so a type used in those signatures must already exist.
struct WifiScanHit {
    bool found;
    int32_t channel;
    int32_t rssi;
    uint8_t bssid[6];
    int auth;
};

// Appends a line to the calibration log buffer and optionally echoes it to Serial
void calLog(String text) {
    if (systemDebugEnabled) Serial.println(text);
    calLogBuffer += text + "\n";
}

static char errorMessage[128]; // reusable buffer for Sensirion error string conversion
float voltageBattery = 0.0;

// WiFi channel and BSSID cached in RTC memory to speed up reconnection after deep sleep
RTC_DATA_ATTR uint8_t cachedChannel = 0;
RTC_DATA_ATTR uint8_t cachedBSSID[6] = {0, 0, 0, 0, 0, 0};
RTC_DATA_ATTR bool cachedValid = false;

// ==================== SECTION 5: RTC MEASUREMENT BUFFER ====================
struct SensorDataset {
  uint32_t timestamp;            // Unix timestamp of the measurement
  uint16_t co2[maxI2CDevices];   // CO₂ readings per sensor port [ppm]
  float temp[maxI2CDevices];     // Temperature readings per sensor port [°C]
  float hum[maxI2CDevices];      // Humidity readings per sensor port [%RH]
  bool valid[maxI2CDevices];     // True if sensor was reachable and data is plausible
  float battery;                 // Battery voltage at time of measurement [V]
};

constexpr uint32_t RTC_BUFFER_CAP = 24; // Physical queue of unsent frames. maxMeasurements only starts an upload.

RTC_DATA_ATTR SensorDataset rtcStorage[RTC_BUFFER_CAP];
RTC_DATA_ATTR uint32_t      rtcCounter = 0; // Number of buffered datasets waiting to be uploaded
RTC_DATA_ATTR bool          rtcTimeIsSynced = false; // Set after first successful NTP/cell time sync
RTC_DATA_ATTR uint32_t      rtcRetryPending = 0; // 1 after an upload left frames behind; retry every wake until empty

// Forward declarations (must appear before first use; definitions follow below)
void loadConfigPrefs();
void saveConfigPrefs();
void handleRoot();
void handleSave();
void handlePing();
void handleStartCal();
void handleCalLog();
void handleFactoryReset();
void handleUpdate();
void handleUpdateUpload();
void setRgbColor(uint8_t r, uint8_t g, uint8_t b);
void rgbOff();
void PrintUint64(uint64_t& value);
void printExtendedError(uint8_t port, const char* functionName, int16_t errorCode);
void selectSensorPort(uint8_t port);
int16_t triggerRawSingleShot(uint8_t port);
bool waitSensorDataReady(uint8_t port, unsigned long timeoutMs);
bool connectWiFi();
bool connectCellular();
void setupOTA();
void initSensor(uint8_t port, bool firstBoot);
void measureBattery();
void uploadData();
void printLogHeader(const char* category);
void shutdownModem();
bool syncNetworkTime();
int readHttpStatusAndDrain(Client& client, unsigned long timeoutMs);

// Prints a formatted log header with elapsed time and category tag to Serial
void printLogHeader(const char* category) {
  double currentSec = millis() / 1000.0;
  Serial.print("[");
  if (currentSec < 10.0)       Serial.print("00");
  else if (currentSec < 100.0) Serial.print("0");
  Serial.print(currentSec, 3);
  Serial.print("s] ");
  Serial.print(category);
}

// Reads all configuration values from NVS flash into RAM buffers.
// On a fresh board with no stored keys, writes the code defaults first.
void loadConfigPrefs() {
    prefs.begin("beehive", true); // read-only

    // First-time board detection: if no config exists yet, burn the code defaults
    if (!prefs.isKey("ssid")) {
        prefs.end();
        if (systemDebugEnabled) {
            printLogHeader("[STORAGE] ");
            Serial.println("No stored config found. Writing code defaults to NVS ...");
        }
        saveConfigPrefs();
        prefs.begin("beehive", true);
    }

    // Copy each stored string into its RAM buffer, guarding against overflow
    if (prefs.isKey("ssid"))     strncpy(wifiSSID, prefs.getString("ssid").c_str(), sizeof(wifiSSID) - 1);
    if (prefs.isKey("pass"))     strncpy(wifiPass, prefs.getString("pass").c_str(), sizeof(wifiPass) - 1);
    if (prefs.isKey("server"))   strncpy(serverURL, prefs.getString("server").c_str(), sizeof(serverURL) - 1);
    if (prefs.isKey("resource")) strncpy(serverResource, prefs.getString("resource").c_str(), sizeof(serverResource) - 1);
    if (prefs.isKey("lteapn"))   strncpy(lteAPN, prefs.getString("lteapn").c_str(), sizeof(lteAPN) - 1);
    if (prefs.isKey("lteuser"))  strncpy(lteUser, prefs.getString("lteuser").c_str(), sizeof(lteUser) - 1);
    if (prefs.isKey("ltepass"))  strncpy(ltePass, prefs.getString("ltepass").c_str(), sizeof(ltePass) - 1);
    if (prefs.isKey("netmode"))  strncpy(netModeStr, prefs.getString("netmode").c_str(), sizeof(netModeStr) - 1);
    if (prefs.isKey("mled"))     strncpy(modemLedStr, prefs.getString("mled").c_str(), sizeof(modemLedStr) - 1);
    if (prefs.isKey("minutes"))  strncpy(sleepMinutesStr, prefs.getString("minutes").c_str(), sizeof(sleepMinutesStr) - 1);
    if (prefs.isKey("maxmeas"))  strncpy(maxMeasurementsStr, prefs.getString("maxmeas").c_str(), sizeof(maxMeasurementsStr) - 1);
    if (prefs.isKey("altitude")) strncpy(sensorAltitudeStr, prefs.getString("altitude").c_str(), sizeof(sensorAltitudeStr) - 1);
    if (prefs.isKey("scount"))   strncpy(sensorCountStr, prefs.getString("scount").c_str(), sizeof(sensorCountStr) - 1);
    if (prefs.isKey("msamples")) strncpy(measSamplesStr, prefs.getString("msamples").c_str(), sizeof(measSamplesStr) - 1);
    if (prefs.isKey("sdebug"))   strncpy(serialDebugStr, prefs.getString("sdebug").c_str(), sizeof(serialDebugStr) - 1);
    if (prefs.isKey("rgbdebug")) strncpy(rgbDebugStr, prefs.getString("rgbdebug").c_str(), sizeof(rgbDebugStr) - 1);
    if (prefs.isKey("hostname")) strncpy(otaHostname, prefs.getString("hostname").c_str(), sizeof(otaHostname) - 1);
    if (prefs.isKey("crypto"))   strncpy(sharedCrypto, prefs.getString("crypto").c_str(), sizeof(sharedCrypto) - 1);
    
    // Convert string buffers to typed runtime variables with range clamping
    sleepSeconds       = atoi(sleepMinutesStr) * 60;
    if (atoi(sleepMinutesStr) == 0) {
        sleepSeconds = 10; // interval = 0 activates rapid test mode (10s cycles)
    } else if (sleepSeconds < 10) {
        sleepSeconds = 300; // minimum 5 minutes if value is implausibly low
    }
    
    sensorAltitude     = atoi(sensorAltitudeStr);
    telemetryMode      = atoi(netModeStr);
    enableModemLed     = (atoi(modemLedStr) == 1);
    maxMeasurements    = atoi(maxMeasurementsStr);
    if (maxMeasurements == 0 || maxMeasurements > RTC_BUFFER_CAP) maxMeasurements = 6;
    
    activeSensors      = atoi(sensorCountStr);
    if (activeSensors == 0 || activeSensors > 8) {
        activeSensors = 3; 
    }

    measurementSamples = (uint8_t)atoi(measSamplesStr);
    if (measurementSamples < 1 || measurementSamples > 10) {
        measurementSamples = 3;
    }
    
    serialDebugLevel     = (uint8_t)atoi(serialDebugStr);
    if (serialDebugLevel > 2) serialDebugLevel = 1;
    systemDebugEnabled   = (serialDebugLevel >= 1);
    cellularDebugEnabled = (serialDebugLevel == 2);
    rgbDebugEnabled      = (atoi(rgbDebugStr) == 1);
    
    prefs.end();
}

// Writes all current RAM buffer values to NVS flash, then reloads them into runtime variables.
void saveConfigPrefs() {
    prefs.begin("beehive", false); // read-write
    prefs.putString("ssid", wifiSSID);
    prefs.putString("pass", wifiPass);
    prefs.putString("server", serverURL);
    prefs.putString("resource", serverResource);
    prefs.putString("lteapn", lteAPN);
    prefs.putString("lteuser", lteUser);
    prefs.putString("ltepass", ltePass);
    prefs.putString("netmode", netModeStr);
    prefs.putString("mled", modemLedStr);
    prefs.putString("minutes", sleepMinutesStr);
    prefs.putString("maxmeas", maxMeasurementsStr);
    prefs.putString("altitude", sensorAltitudeStr);
    prefs.putString("scount", sensorCountStr);
    prefs.putString("msamples", measSamplesStr);
    prefs.putString("sdebug", serialDebugStr);
    prefs.putString("rgbdebug", rgbDebugStr);
    prefs.putString("hostname", otaHostname);
    prefs.putString("crypto", sharedCrypto);
    prefs.end();
    
    loadConfigPrefs();
    if (systemDebugEnabled) {
        printLogHeader("[STORAGE] ");
        Serial.println("Configuration successfully burned into native NVS cells!");
    }
}

// Renders the full web configuration portal page and sends it to the connected browser
void handleRoot() {
    lastPortalActivity = millis(); 
    
    String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:sans-serif;background:#f4f4f9;padding:20px;color:#333;}";
    html += ".card{background:#fff;padding:20px;border-radius:8px;box-shadow:0 4px 6px rgba(0,0,0,0.1);max-width:420px;margin:0 auto;}";
    html += "h2{color:#e67e22;text-align:center;margin-bottom:25px;}";
    
    html += "fieldset{border:1px solid #ddd; border-radius:6px; margin-bottom:22px; padding:15px 18px; background:#fafafa;}";
    html += "legend{color:#2c3e50; font-size:14px; font-weight:bold; padding:0 8px; background:#fff; border-radius:3px;}";
    html += "label{display:block; font-size:13px; font-weight:bold; margin-top:10px; color:#555;}";
    html += "input, select{width:100%;padding:10px;margin:6px 0 12px 0;box-sizing:border-box;border:1px solid #ccc;border-radius:4px;background:#fff;}";
    html += ".hint{font-size:11px; color:#7f8c8d; font-weight:normal; margin-top:-9px; margin-bottom:12px; display:block;}";
    html += "button{width:100%;padding:12px;background:#e67e22;color:#fff;border:none;border-radius:4px;font-size:16px;cursor:pointer;margin-top:5px;font-weight:bold;}";
    html += "button:hover{background:#d35400;}";
    
    html += ".modal {display:none; position:fixed; z-index:9999; left:0; top:0; width:100%; height:100%; background-color:rgba(0,0,0,0.6); border-radius:0;}";
    html += ".modal-content {background-color:#fff; padding:25px; border-radius:10px; width:85%; max-width:400px; text-align:center; box-shadow:0 5px 15px rgba(0,0,0,0.3); border:none; animation:fadeIn 0.3s; box-sizing:border-box; margin:10px;}";
    html += ".modal-icon {font-size:40px; color:#e67e22; display:block; margin-bottom:10px;}";
    html += ".modal h3{margin:0 0 10px 0;color:#2c3e50;}";
    html += ".modal p{font-size:14px;color:#555;line-height:1.5;margin:0 0 20px 0;}";
    html += ".btn-close {background:#2c3e50; color:#fff; border:none; padding:10px 20px; border-radius:4px; cursor:pointer; font-weight:bold; width:100%; margin-top:10px;}";
    html += ".btn-close:disabled {background:#7f8c8d; cursor:not-allowed;}";
    html += "@keyframes fadeIn{from{opacity:0;transform:translateY(-10px);}to{opacity:1;transform:translateY(0);}}";
    html += "#busyLayer{z-index:10001;background:rgba(0,0,0,0.82);cursor:not-allowed;}";
    html += "</style></head><body>";
    
    html += "<div class='card'><h2>⬡ Open Hive<br>CO₂ Monitoring System</h2>";
    html += "<form id='configForm' onsubmit='saveConfig(event)'>";

    // Connection mode selector (WiFi / LTE-M / NB-IoT)
    html += "<fieldset><legend>Connection</legend>";
    html += "<label>Connection Type</label>";
    html += "<select name='netmode'>";
    html += "<option value='0'" + String(telemetryMode == 0 ? " selected" : "") + ">WiFi</option>";
    html += "<option value='1'" + String(telemetryMode == 1 ? " selected" : "") + ">LTE-M (Cat-M1)</option>";
    html += "<option value='2'" + String(telemetryMode == 2 ? " selected" : "") + ">NB-IoT (Cat-NB1)</option>";
    html += "</select>";
    html += "</fieldset>";

    // Remote server address and API endpoint
    html += "<fieldset><legend>Server Settings</legend>";
    html += "<label>Server Address</label><input type='text' name='server' id='serverField' value='" + String(serverURL) + "' required>";
    html += "<label>API Endpoint Path</label><input type='text' name='resource' value='" + String(serverResource) + "' required>";
    html += "</fieldset>";

    // WiFi credentials
    html += "<fieldset><legend>WiFi Connection</legend>";
    html += "<label>WiFi SSID</label><input type='text' name='ssid' id='ssidField' value='" + String(wifiSSID) + "'>";
    html += "<label>WiFi Password</label><input type='password' name='pass' id='wifi_password_field' value='" + String(wifiPass) + "'>";
    html += "<div style='text-align:right; margin:-4px 0 0 0; font-size:13px; color:#555;'><input type='checkbox' style='width:auto; margin:0 6px 0 0; vertical-align:middle;' onclick='var x=document.getElementById(\"wifi_password_field\"); if(this.checked){x.type=\"text\";}else{x.type=\"password\";}'>Show Password</div>";
    html += "</fieldset>";

    // Cellular APN and modem LED settings
    html += "<fieldset><legend>Cellular Connection</legend>";
    html += "<label>APN Gateway</label><input type='text' name='lteapn' value='" + String(lteAPN) + "'>";
    
    html += "<label>APN Username</label><input type='text' name='lteuser' value='" + String(lteUser) + "'>";
    html += "<span class='hint'>Leave blank if not required by provider</span>";
    
    html += "<label>APN Password</label><input type='password' name='ltepass' value='" + String(ltePass) + "'>";
    html += "<span class='hint'>Leave blank if not required by provider</span>";
    
    html += "<label>Status LED Cellular</label>";
    html += "<select name='mled'>";
    html += "<option value='1'" + String(enableModemLed ? " selected" : "") + ">ON</option>";
    html += "<option value='0'" + String(!enableModemLed ? " selected" : "") + ">OFF</option>";
    html += "</select>";
    html += "</fieldset>";
    
    // Sensor count, measurement interval and buffer depth
    html += "<fieldset><legend>Sensors & Intervals</legend>";
    html += "<label>Active Sensors (1-8)</label><input type='number' name='scount' value='" + String(sensorCountStr) + "' min='1' max='8'>";
    html += "<label>Samples per Measurement (1-10)</label><input type='number' name='msamples' value='" + String(measSamplesStr) + "' min='1' max='10'>";
    html += "<span class='hint'>Averaged single-shots after discarding the first reading. Group wait 5 s, then data-ready poll up to +2 s (max ~7 s).</span>";
    html += "<label>Interval (Minutes)</label><input type='number' name='minutes' value='" + String(sleepMinutesStr) + "'>";
    html += "<label>Buffer Size (Cycles before Upload)</label><input type='number' name='maxmeas' value='" + String(maxMeasurementsStr) + "' min='1' max='24'>";
    html += "<label>Sensor Altitude (Meters)</label><input type='number' name='altitude' value='" + String(sensorAltitudeStr) + "'>";
    html += "</fieldset>";

    // Debug output flags
    html += "<fieldset><legend>Hardware Logs</legend>";
    html += "<label>Serial Debug</label>";
    html += "<select name='sdebug'>";
    html += "<option value='1'" + String(serialDebugLevel == 1 ? " selected" : "") + ">ON — Default</option>";
    html += "<option value='2'" + String(serialDebugLevel == 2 ? " selected" : "") + ">ON — Default + Extended Cellular Debug</option>";
    html += "<option value='0'" + String(serialDebugLevel == 0 ? " selected" : "") + ">OFF</option>";
    html += "</select>";
    html += "<span class='hint'>Extended Cellular Debug adds step-by-step modem connect details (init, network, GPRS)</span>";
    
    html += "<label>RGB LED Status</label>";
    html += "<select name='rgbdebug'>";
    html += "<option value='1'" + String(rgbDebugEnabled ? " selected" : "") + ">ON</option>";
    html += "<option value='0'" + String(!rgbDebugEnabled ? " selected" : "") + ">OFF</option>";
    html += "</select>";
    html += "</fieldset>";
    
    // Device hostname and shared security key (OTA / AP)
    html += "<fieldset><legend>Device Identity</legend>";
    html += "<label>Device Name | WLAN SSID configuration mode</label><input type='text' name='hostname' value='" + String(otaHostname) + "'>";
    html += "<label>Security Key | OTA AP Password</label><input type='text' name='crypto' value='" + String(sharedCrypto) + "'>";
    html += "</fieldset>";

    // Save button — submits the form via AJAX. Measurement after reboot is opt-in.
    html += "<div style='margin: 20px 0 35px 0;'>";
    html += "<label style='font-weight:normal; font-size:13px; margin-top:0;'>";
    html += "<input type='checkbox' name='measure' value='1' style='width:auto; margin:0 8px 0 0; vertical-align:middle;'>";
    html += "Start measurement after saving</label>";
    html += "<span class='hint' style='margin-top:4px;'>Unchecked: the device opens the portal again after reboot.</span>";
    html += "<button type='submit' style='background:#e67e22;'>Save Configuration</button>";
    html += "</div>";
    html += "</form>"; 

    // FRC calibration: 400 ppm fresh air, or a known reference in the Sensirion 400–2000 ppm window
    html += "<fieldset style='margin-top:48px;'><legend>Sensor Calibration</legend>";
    html += "<label>Reference</label>";
    html += "<select id='calMode' onchange='updateCalMode()'>";
    html += "<option value='fresh' selected>Fresh air — 400 ppm</option>";
    html += "<option value='ref'>Known concentration</option>";
    html += "</select>";
    html += "<div id='calPpmWrap' style='display:none;'>";
    html += "<label>Reference CO₂ (ppm)</label>";
    html += "<input type='number' id='calPpm' min='400' max='2000' step='1' inputmode='numeric' value='400'>";
    html += "</div>";
    html += "<span class='hint'>Sensirion testing guide: a known reference should be 400–2000 ppm and must stay constant for the whole preparation. FRC takes one value, not a span between two concentrations. There is no documented limit at 1500 ppm.</span>";
    html += "<div id='calWarn' style='background:#fff3cd; color:#2c3e50; border:1px solid #ffeeba; padding:12px; border-radius:6px; font-size:13px; margin:5px 0 15px 0; line-height:1.4;'>";
    html += "<strong>⚠️ IMPORTANT:</strong> Use fresh air only when the air really is about 400 ppm. Indoor or stale air will permanently shift all future readings.";
    html += "<br><br>FRC prep: 5 idle single-shots at 1-minute intervals (sensors stay awake), then FRC (~5–6 min per group). Field measurements stay power-cycled afterwards.";
    html += "</div>";
    html += "<button type='button' id='btnStartCal' onclick='startCalibration()' style='background:#2c3e50; margin-top:0;'>Start Sensor Calibration</button>";
    html += "</fieldset>";

    // Per-sensor factory reset: resets SCD41 EEPROM and clears FRC/ASC history
    html += "<fieldset style='margin-top:48px;'><legend>Sensor Factory Reset</legend>";
    html += "<label>Sensor Number</label>";
    html += "<select id='factorySensor'>";
    for (uint8_t i = 1; i <= activeSensors; i++) {
        html += "<option value='" + String(i) + "'>Sensor " + String(i) + "</option>";
    }
    html += "<option value='all'>All sensors</option>";
    html += "</select>";
    html += "<span class='hint'>Resets EEPROM settings and FRC/ASC history. All sensors runs them one after another.</span>";
    html += "<button type='button' id='btnFactoryReset' onclick='doFactoryReset()' style='background:#c0392b; margin-top:8px;'>Factory Reset</button>";
    html += "</fieldset>";

    // Web OTA: flash a precompiled .bin from the browser (Arduino sketch .ino is not supported)
    html += "<fieldset style='margin-top:48px;'><legend>Firmware Update</legend>";
    html += "<label>Compiled Firmware (.bin)</label>";
    html += "<input type='file' id='fwFile' accept='.bin,application/octet-stream'>";
    html += "<span class='hint'>Upload a compiled .bin from Arduino IDE (Sketch → Export compiled Binary). Raw .ino source cannot be flashed.</span>";
    html += "<div id='fwProgress' style='font-size:13px; color:#555; margin:8px 0 0 0; min-height:18px;'></div>";
    html += "<button type='button' id='btnFwUpdate' onclick='uploadFirmware()' style='background:#d35400; margin-top:8px;'>Upload & Flash Firmware</button>";
    html += "</fieldset>";

    html += "<div style='text-align:center; margin-top:25px; padding-top:15px; border-top:1px solid #eee; font-size:13px; color:#888;'>";
    html += "⬡ Open Hive | CO₂ Monitoring System | Firmware Version v" + String(SW_VERSION) + "</div>";
    html += "</div>";

    // Modals
    html += "<div id='busyLayer' class='modal'><div class='modal-content'><span class='modal-icon'>⬢</span><h3>Saving Configuration</h3><p id='busyText'>The configuration is being written. The device will reboot.</p></div></div>";
    html += "<div id='calModal' class='modal'><div class='modal-content' style='max-width: 650px; width: 92%; padding: 20px; box-sizing: border-box;'><span class='modal-icon'>⬢</span><h3>Forced Calibration Processing</h3><p id='calModalNote' style='font-size:13px; color:#555;'>Idle single-shot FRC prep (~5–6 min per group). Keep the CO₂ concentration stable. Cancel aborts safely if nothing was persisted yet.</p><textarea id='calConsole' readonly style='width:100%; height:220px; background:#111; color:#ff9f43; font-family:monospace; font-size:11px; padding:12px; box-sizing:border-box; border-radius:4px; border:1px solid #222; resize:none; white-space:pre; overflow-x:auto;' placeholder='Awaiting baseline frame allocation...'></textarea><button id='btnCalClose' onclick='handleCalButtonClick()' class='btn-close' style='background:#d35400; margin-top:15px;'>Cancel Calibration</button></div></div>";
    html += "<div id='factoryResetModal' class='modal'><div class='modal-content'><span class='modal-icon'>⬢</span><h3 id='factoryResetTitle'>Factory Reset Done</h3><p id='factoryResetMsg'>OK</p><button id='btnFactoryResetOk' onclick='document.getElementById(\"factoryResetModal\").style.display=\"none\"' class='btn-close'>OK</button></div></div>";
    html += "<div id='fwModal' class='modal'><div class='modal-content'><span class='modal-icon'>⬢</span><h3>Firmware Update Complete</h3><p>The new firmware was written successfully. The device will reboot now.</p><button onclick='closeModal(\"fwModal\")' class='btn-close'>OK</button></div></div>";
    
    // Client-side JavaScript: form handling, calibration polling, factory reset
    html += "<script>var logInterval = null; var isFormDirty = false; var isCalFinished = false; var activeSensorCount = " + String(activeSensors) + ";";
    html += "function updateWifiRequired() { var mode = document.querySelector(\"select[name='netmode']\"); var ssid = document.getElementById(\"ssidField\"); if(!mode || !ssid) return; var isWifi = (mode.value === \"0\"); ssid.required = isWifi; }";
    html += "window.addEventListener(\"DOMContentLoaded\", function() { updateWifiRequired(); var modeSelect = document.querySelector(\"select[name='netmode']\"); if(modeSelect) modeSelect.addEventListener(\"change\", updateWifiRequired); var form = document.getElementById(\"configForm\"); if(form) { var inputs = form.getElementsByTagName(\"input\"); for(var i = 0; i < inputs.length; i++) { if(inputs[i].type !== \"checkbox\" && inputs[i].type !== \"submit\" && inputs[i].type !== \"button\") { inputs[i].addEventListener(\"input\", function() { isFormDirty = true; }); } } } });";
    html += "function updateCalMode() { var mode = document.getElementById(\"calMode\"); var wrap = document.getElementById(\"calPpmWrap\"); var warn = document.getElementById(\"calWarn\"); if(!mode || !wrap || !warn) return; var isRef = (mode.value === \"ref\"); wrap.style.display = isRef ? \"block\" : \"none\"; if(isRef) { warn.innerHTML = \"<strong>⚠️ IMPORTANT:</strong> Enter the concentration at the sensors for the whole preparation, from a reference meter or certified gas. Sensirion’s window for a known reference is 400–2000 ppm. Keep the reference next to the sensors, and keep humidity, temperature and pressure stable.\"; } else { warn.innerHTML = \"<strong>⚠️ IMPORTANT:</strong> Use fresh air only when the air really is about 400 ppm. Indoor or stale air will permanently shift all future readings.<br><br>FRC prep: 5 idle single-shots at 1-minute intervals (sensors stay awake), then FRC (~5–6 min per group). Field measurements stay power-cycled afterwards.\"; } }";
    html += "function readCalTargetPpm() { var mode = document.getElementById(\"calMode\"); if(!mode || mode.value !== \"ref\") return 400; var el = document.getElementById(\"calPpm\"); var ppm = el ? parseInt(el.value, 10) : NaN; if(!el || el.value === \"\" || isNaN(ppm) || ppm < 400 || ppm > 2000) return -1; return ppm; }";
    html += "function startCalibration() { if(isFormDirty) { alert(\"⚠️ Unsaved changes detected! Please save your configuration first.\"); return; } var ppm = readCalTargetPpm(); if(ppm < 400) { alert(\"Enter a reference from 400 to 2000 ppm. That is the known-concentration window in the Sensirion testing guide.\"); return; } if(!confirm(\"Calibrate all sensors so this air reads \" + ppm + \" ppm?\")) return; isCalFinished = false; var btn = document.getElementById(\"btnCalClose\"); if(btn) { btn.innerHTML = \"Cancel Calibration\"; btn.style.background = \"#d9534f\"; btn.disabled = false; } var note = document.getElementById(\"calModalNote\"); if(note) note.innerHTML = \"Target \" + ppm + \" ppm. Idle single-shot FRC prep (~5–6 min per group). Keep that concentration stable. Cancel aborts safely if nothing was persisted yet.\"; var modal = document.getElementById(\"calModal\"); if(modal) { modal.style.display = \"flex\"; modal.style.alignItems = \"center\"; modal.style.justifyContent = \"center\"; } var con = document.getElementById(\"calConsole\"); if(con) con.value = \"Connecting to calibration engine...\\n\"; fetch(\"/start-cal?ppm=\" + ppm).then(function(r) { if(!r.ok) { clearInterval(logInterval); if(modal) modal.style.display = \"none\"; alert(\"Calibration was not started. Reference must be 400–2000 ppm.\"); } }); if(logInterval) clearInterval(logInterval); logInterval = setInterval(pollCalLog, 1500); }";
    html += "function pollCalLog() { fetch(\"/cal-log\").then(r=>r.text()).then(text=> { var con = document.getElementById(\"calConsole\"); if(con) { con.value = text; con.scrollTop = con.scrollHeight; } if(text.indexOf(\"=== END ===\") !== -1) { clearInterval(logInterval); var maxId = setInterval(function(){}, 9999); for (var i = 1; i <= maxId; i++) { clearInterval(i); } isCalFinished = true; var btn = document.getElementById(\"btnCalClose\"); if(btn) { btn.innerHTML = \"OK (Reboot Device)\"; btn.style.background = \"#2c3e50\"; } setTimeout(function() { document.getElementById(\"calModal\").style.display = \"none\"; window.location.replace(\"about:blank\"); }, 15500); } }); }";
    html += "function handleCalButtonClick() { if(!isCalFinished) { if(confirm(\"⚠️ HALT: Abort hardware calibration safely?\")) { clearInterval(logInterval); fetch(\"/cancel-cal\").then(()=> { document.getElementById(\"calModal\").style.display = \"none\"; window.location.href = \"/\"; }); } } else { document.getElementById(\"calModal\").style.display = \"none\"; fetch(\"/save\"); window.location.replace(\"about:blank\"); } }";
    html += "function doFactoryReset() { var sel = document.getElementById(\"factorySensor\"); if(!sel) return; var n = sel.value; var all = (n === \"all\"); var ask = all ? \"Factory reset ALL sensors? This erases EEPROM settings and calibration history.\" : (\"Factory reset Sensor \" + n + \"? This erases EEPROM settings and calibration history.\"); if(!confirm(ask)) return; var btn = document.getElementById(\"btnFactoryReset\"); var msg = document.getElementById(\"factoryResetMsg\"); var title = document.getElementById(\"factoryResetTitle\"); var modal = document.getElementById(\"factoryResetModal\"); var okBtn = document.getElementById(\"btnFactoryResetOk\"); if(btn) { btn.disabled = true; btn.innerHTML = \"Resetting...\"; } if(title) title.innerHTML = \"Factory Reset\"; if(okBtn) okBtn.disabled = true; if(modal) { modal.style.display = \"flex\"; modal.style.alignItems = \"center\"; modal.style.justifyContent = \"center\"; } var list = []; if(all) { for(var i = 1; i <= activeSensorCount; i++) list.push(String(i)); } else { list.push(n); } function finishOk() { if(btn) { btn.disabled = false; btn.innerHTML = \"Factory Reset\"; } if(title) title.innerHTML = \"Factory Reset Done\"; if(okBtn) okBtn.disabled = false; if(msg) msg.innerHTML = all ? \"OK — All sensors factory reset completed.\" : (\"OK — Sensor \" + n + \" factory reset completed.\"); } function fail(text) { if(btn) { btn.disabled = false; btn.innerHTML = \"Factory Reset\"; } if(title) title.innerHTML = \"Factory Reset Failed\"; if(okBtn) okBtn.disabled = false; if(msg) msg.innerHTML = text; } function resetNext(idx) { if(idx >= list.length) { finishOk(); return; } var cur = list[idx]; if(msg) msg.innerHTML = \"Sensor \" + cur + \" is being reset...\"; fetch(\"/factory-reset?sensor=\" + cur).then(function(r) { return r.text().then(function(t) { return { ok: r.ok, text: (t || \"\").trim() }; }); }).then(function(res) { if(!(res.ok && res.text === \"OK\")) { fail(\"Factory reset failed on sensor \" + cur + \": \" + (res.text || \"unknown error\")); return; } resetNext(idx + 1); }).catch(function() { fail(\"Factory reset failed: no response from device (sensor \" + cur + \").\"); }); } resetNext(0); }";
    html += "function uploadFirmware() { var inp = document.getElementById(\"fwFile\"); var prog = document.getElementById(\"fwProgress\"); var btn = document.getElementById(\"btnFwUpdate\"); if(!inp || !inp.files || !inp.files.length) { alert(\"Please select a compiled .bin firmware file.\"); return; } var file = inp.files[0]; if(!/\\.bin$/i.test(file.name)) { alert(\"Only compiled .bin files are supported. Export via Arduino IDE: Sketch → Export compiled Binary.\"); return; } if(!confirm(\"Flash firmware \\\"\" + file.name + \"\\\"? The device will reboot afterwards.\")) return; if(btn) { btn.disabled = true; btn.innerHTML = \"Flashing...\"; } if(prog) prog.innerHTML = \"Starting upload...\"; var xhr = new XMLHttpRequest(); xhr.open(\"POST\", \"/update\", true); xhr.upload.onprogress = function(e) { if(!prog) return; if(e.lengthComputable && e.total > 0) { prog.innerHTML = \"Uploading: \" + Math.round(e.loaded / e.total * 100) + \"%\"; } else { prog.innerHTML = \"Uploading: \" + Math.round(e.loaded / 1024) + \" KB\"; } }; xhr.onload = function() { if(xhr.status === 200) { if(prog) prog.innerHTML = \"Flash OK — rebooting...\"; var modal = document.getElementById(\"fwModal\"); if(modal) { modal.style.display = \"flex\"; modal.style.alignItems = \"center\"; modal.style.justifyContent = \"center\"; } setTimeout(function() { window.location.replace(\"about:blank\"); }, 4000); } else { if(btn) { btn.disabled = false; btn.innerHTML = \"Upload & Flash Firmware\"; } if(prog) prog.innerHTML = \"\"; alert(\"Firmware update failed: \" + (xhr.responseText || (\"HTTP \" + xhr.status))); } }; xhr.onerror = function() { if(btn) { btn.disabled = false; btn.innerHTML = \"Upload & Flash Firmware\"; } if(prog) prog.innerHTML = \"\"; alert(\"Firmware update failed: connection lost.\"); }; var fd = new FormData(); fd.append(\"firmware\", file); xhr.send(fd); }";
    html += "function showBusy(msg) { var layer = document.getElementById(\"busyLayer\"); var text = document.getElementById(\"busyText\"); if(text && msg) text.innerHTML = msg; if(layer) { layer.style.display = \"flex\"; layer.style.alignItems = \"center\"; layer.style.justifyContent = \"center\"; } }";
    html += "function hideBusy() { var layer = document.getElementById(\"busyLayer\"); if(layer) layer.style.display = \"none\"; }";
    html += "function afterSave(measureNow) { if(measureNow) return; var sawDown = false; var n = 0; function poll() { n++; if(n > 60) return; var ctrl = new AbortController(); var timer = setTimeout(function() { ctrl.abort(); }, 1500); fetch(\"/ping\", { cache: \"no-store\", signal: ctrl.signal }).then(function(r) { clearTimeout(timer); if(r.ok && sawDown) { window.location.replace(\"/\"); return; } setTimeout(poll, 700); }).catch(function() { clearTimeout(timer); sawDown = true; setTimeout(poll, 700); }); } setTimeout(poll, 400); }";
    html += "function saveConfig(e) { e.preventDefault(); var f = document.getElementById(\"configForm\"); if(!f) return; var box = f.querySelector(\"input[name='measure']\"); var measureNow = !!(box && box.checked); showBusy(measureNow ? \"Measurement starts after reboot. This page stays inactive.\" : \"The page reloads when the portal is back.\"); var d = new FormData(f); d.set(\"measure\", measureNow ? \"1\" : \"0\"); fetch(\"/save\", { method: \"POST\", body: d }).then(function(r) { if(r.ok) { isFormDirty = false; afterSave(measureNow); } else { hideBusy(); alert(\"Save error.\"); } }).catch(function() { isFormDirty = false; afterSave(measureNow); }); }";
    html += "function closeModal(modalId) { document.getElementById(modalId).style.display = \"none\"; window.location.href = \"/\"; }";
    html += "setInterval(function(){fetch(\"/ping\");}, 10000);</script></body></html>";
    
    server.send(200, "text/html; charset=utf-8", html);
}

// Saves portal form fields to NVS, then reboots so the new config takes effect.
// measure=1 is a one-shot opt-in: the next boot skips the portal and measures.
void handleSave() {
    if (server.hasArg("ssid"))     strncpy(wifiSSID, server.arg("ssid").c_str(), sizeof(wifiSSID) - 1);
    if (server.hasArg("pass"))     strncpy(wifiPass, server.arg("pass").c_str(), sizeof(wifiPass) - 1);
    if (server.hasArg("server"))   strncpy(serverURL, server.arg("server").c_str(), sizeof(serverURL) - 1);
    if (server.hasArg("resource")) strncpy(serverResource, server.arg("resource").c_str(), sizeof(serverResource) - 1);
    if (server.hasArg("lteapn"))   strncpy(lteAPN, server.arg("lteapn").c_str(), sizeof(lteAPN) - 1);
    if (server.hasArg("lteuser"))  strncpy(lteUser, server.arg("lteuser").c_str(), sizeof(lteUser) - 1);
    if (server.hasArg("ltepass"))  strncpy(ltePass, server.arg("ltepass").c_str(), sizeof(ltePass) - 1);
    if (server.hasArg("netmode"))  strncpy(netModeStr, server.arg("netmode").c_str(), sizeof(netModeStr) - 1);
    if (server.hasArg("mled"))     strncpy(modemLedStr, server.arg("mled").c_str(), sizeof(modemLedStr) - 1);
    if (server.hasArg("minutes"))  strncpy(sleepMinutesStr, server.arg("minutes").c_str(), sizeof(sleepMinutesStr) - 1);
    if (server.hasArg("maxmeas"))  strncpy(maxMeasurementsStr, server.arg("maxmeas").c_str(), sizeof(maxMeasurementsStr) - 1);
    if (server.hasArg("altitude")) strncpy(sensorAltitudeStr, server.arg("altitude").c_str(), sizeof(sensorAltitudeStr) - 1);
    if (server.hasArg("scount"))   strncpy(sensorCountStr, server.arg("scount").c_str(), sizeof(sensorCountStr) - 1);
    if (server.hasArg("msamples")) strncpy(measSamplesStr, server.arg("msamples").c_str(), sizeof(measSamplesStr) - 1);
    if (server.hasArg("sdebug"))   strncpy(serialDebugStr, server.arg("sdebug").c_str(), sizeof(serialDebugStr) - 1);
    if (server.hasArg("rgbdebug")) strncpy(rgbDebugStr, server.arg("rgbdebug").c_str(), sizeof(rgbDebugStr) - 1);
    if (server.hasArg("hostname")) strncpy(otaHostname, server.arg("hostname").c_str(), sizeof(otaHostname) - 1);
    if (server.hasArg("crypto"))   strncpy(sharedCrypto, server.arg("crypto").c_str(), sizeof(sharedCrypto) - 1);

    // NVS, not RTC: ESP.restart() reloads RTC_DATA_ATTR initializers, so a RAM flag
    // would be false again and the portal would always reopen.
    bool measureNow = server.hasArg("measure") && (server.arg("measure") == "1");
    saveConfigPrefs();
    prefs.begin("beehive", false);
    if (measureNow) prefs.putString("measnow", "1");
    else prefs.remove("measnow");
    prefs.end();
    if (systemDebugEnabled) {
        printLogHeader("[PORTAL ] ");
        Serial.println(measureNow
            ? "Configuration saved. Next boot starts measurement."
            : "Configuration saved. Next boot opens the portal.");
    }
    server.send(200, "text/plain", "OK");
    delay(1500);
    ESP.restart(); 
}

// Keep-alive endpoint: resets the portal inactivity timer while the browser is open
void handlePing() {
    lastPortalActivity = millis(); 
    server.send(200, "text/plain", "pong");
}

// Returns the calibration console log buffer for browser polling
void handleCalLog() {
    server.send(200, "text/plain; charset=utf-8", calLogBuffer);
}

// Runs SCD41 factory reset on the sensor number requested by the portal
void handleFactoryReset() {
    lastPortalActivity = millis();

    if (!server.hasArg("sensor")) {
        server.send(400, "text/plain", "MISSING_SENSOR");
        return;
    }

    int sensorNum = server.arg("sensor").toInt();
    if (sensorNum < 1 || sensorNum > (int)activeSensors) {
        server.send(400, "text/plain", "INVALID_SENSOR");
        return;
    }

    uint8_t port = (uint8_t)(sensorNum - 1);

    selectSensorPort(port);
    sensors[port].begin(Wire, SCD41_I2C_ADDR);
    delay(20);
    sensors[port].wakeUp();
    delay(50);
    sensors[port].stopPeriodicMeasurement();
    delay(500);

    int16_t err = sensors[port].performFactoryReset();
    delay(1200); // SCD41 datasheet: max. 1200 ms command duration after factory reset

    if (err != NO_ERROR) {
        printExtendedError(port, "performFactoryReset", err);
        server.send(500, "text/plain", "ERROR");
        return;
    }

    if (systemDebugEnabled) {
        printLogHeader("[SENSOR ] ");
        Serial.print("Factory reset OK on sensor ");
        Serial.println(sensorNum);
    }

    server.send(200, "text/plain", "OK");
}

// Completes a browser firmware flash: reply OK/FAIL, then reboot into the new image
void handleUpdate() {
    lastPortalActivity = millis();
    server.sendHeader("Connection", "close");
    if (Update.hasError()) {
        if (systemDebugEnabled) {
            printLogHeader("[OTA    ] ");
            Serial.print("Web firmware update failed: ");
            Update.printError(Serial);
        }
        server.send(500, "text/plain", "FAIL");
        return;
    }
    if (systemDebugEnabled) {
        printLogHeader("[OTA    ] ");
        Serial.println("Web firmware update OK. Rebooting ...");
    }
    server.send(200, "text/plain", "OK");
    delay(1500);
    ESP.restart();
}

// Streams multipart .bin chunks from the portal into the ESP32 Update partition
void handleUpdateUpload() {
    HTTPUpload& upload = server.upload();
    lastPortalActivity = millis();

    if (upload.status == UPLOAD_FILE_START) {
        if (systemDebugEnabled) {
            printLogHeader("[OTA    ] ");
            Serial.print("Web firmware upload started: ");
            Serial.println(upload.filename);
        }
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            if (systemDebugEnabled) {
                printLogHeader("[OTA    ] ");
                Update.printError(Serial);
            }
        }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            if (systemDebugEnabled) {
                printLogHeader("[OTA    ] ");
                Update.printError(Serial);
            }
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            if (systemDebugEnabled) {
                printLogHeader("[OTA    ] ");
                Serial.printf("Web firmware written: %u bytes\n", upload.totalSize);
            }
        } else if (systemDebugEnabled) {
            printLogHeader("[OTA    ] ");
            Update.printError(Serial);
        }
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
        Update.abort();
        if (systemDebugEnabled) {
            printLogHeader("[OTA    ] ");
            Serial.println("Web firmware upload aborted");
        }
    }
}

// Sets the NeoPixel RGB color when RGB debug is enabled
void setRgbColor(uint8_t r, uint8_t g, uint8_t b) {
    if (rgbDebugEnabled) {
        rgbLed.setPixelColor(0, rgbLed.Color(r, g, b));
        rgbLed.show();
    }
}

// Turns the NeoPixel off when RGB debug is enabled
void rgbOff() {
    if (rgbDebugEnabled) {
        rgbLed.setPixelColor(0, rgbLed.Color(0, 0, 0));
        rgbLed.show();
    }
}

// Prints a 64-bit value as hex to Serial (debug helper)
void PrintUint64(uint64_t& value) {
    if(systemDebugEnabled) {
        Serial.print("0x");
        Serial.print((uint32_t)(value >> 32), HEX);
        Serial.print((uint32_t)(value & 0xFFFFFFFF), HEX);
    }
}

// Logs a Sensirion library error with port and function name
void printExtendedError(uint8_t port, const char* functionName, int16_t errorCode) {
    if (systemDebugEnabled && errorCode != NO_ERROR) {
        errorToString(errorCode, errorMessage, sizeof(errorMessage));
        Serial.print("[ERROR ] port ");
        Serial.print(port);
        Serial.print(" - ");
        Serial.print(functionName);
        Serial.print(" failed: ");
        Serial.println(errorMessage);
    }
}

// Selects a TCA9548A channel and waits for the I2C bus to settle before sensor I/O
void selectSensorPort(uint8_t port) {
    I2CMulti.selectPort(port);
    delay(20);
}

// Triggers a raw SCD41 single-shot measurement on the given multiplexer port
int16_t triggerRawSingleShot(uint8_t port) {
    selectSensorPort(port);
    Wire.beginTransmission(SCD41_I2C_ADDR);
    Wire.write(0x21); 
    Wire.write(0x9D);
    return Wire.endTransmission();
}

// Polls SCD41 data-ready flag after a single-shot. Returns false on timeout (does not wait forever).
bool waitSensorDataReady(uint8_t port, unsigned long timeoutMs) {
    selectSensorPort(port);
    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        bool dataReady = false;
        int16_t err = sensors[port].getDataReadyStatus(dataReady);
        if (err == NO_ERROR && dataReady) {
            return true;
        }
        delay(50);
        yield();
    }
    return false;
}

// Last 802.11 disconnect reason (201 = AP not found, 15/204 = handshake/password).
static volatile uint8_t wifiLastDiscReason = 0;

static void onWifiStaEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
        wifiLastDiscReason = info.wifi_sta_disconnected.reason;
    }
}

static const char* wifiStatusName(wl_status_t status) {
    switch (status) {
        case WL_IDLE_STATUS: return "IDLE";
        case WL_NO_SSID_AVAIL: return "NO_SSID_AVAIL";
        case WL_SCAN_COMPLETED: return "SCAN_COMPLETED";
        case WL_CONNECTED: return "CONNECTED";
        case WL_CONNECT_FAILED: return "CONNECT_FAILED";
        case WL_CONNECTION_LOST: return "CONNECTION_LOST";
        case WL_DISCONNECTED: return "DISCONNECTED";
        default: return "UNKNOWN";
    }
}

static const char* wifiReasonName(uint8_t reason) {
    switch (reason) {
        case 2: return "AUTH_EXPIRE";
        case 8: return "ASSOC_LEAVE"; // often our own disconnect(), not an AP reject
        case 15: return "4WAY_HANDSHAKE_TIMEOUT";
        case 39: return "TIMEOUT";
        case 201: return "NO_AP_FOUND";
        case 202: return "AUTH_FAIL";
        case 203: return "ASSOC_FAIL";
        case 204: return "HANDSHAKE_TIMEOUT";
        default: return "";
    }
}

static const char* wifiAuthName(int mode) {
    switch (mode) {
        case WIFI_AUTH_OPEN: return "OPEN";
        case WIFI_AUTH_WEP: return "WEP";
        case WIFI_AUTH_WPA_PSK: return "WPA";
        case WIFI_AUTH_WPA2_PSK: return "WPA2";
        case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2";
        case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-ENT";
        case WIFI_AUTH_WPA3_PSK: return "WPA3";
        case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3";
        default: return "OTHER";
    }
}

// Bring STA up with modem-sleep off and a fixed DE channel map (1–13).
// The ESP32 default country often scans only channels 1–11, so a router on 12/13
// is visible to phones and invisible to the ESP. WIFI_MODE_NULL is intentionally
// not used here: it de-inits the driver and the following internal scan comes back empty.
static void wifiApplyRadio() {
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(false);
    WiFi.setHostname(otaHostname);
    WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
    WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);

    wifi_country_t country;
    memset(&country, 0, sizeof(country));
    memcpy(country.cc, "DE", 2);
    country.schan = 1;
    country.nchan = 13;
    country.max_tx_power = 80; // 20 dBm, unit is 0.25 dBm
    country.policy = WIFI_COUNTRY_POLICY_MANUAL;
    esp_err_t err = esp_wifi_set_country(&country);
    if (systemDebugEnabled && err != ESP_OK) {
        printLogHeader("[WIFI   ] ");
        Serial.printf("Country DE not applied (%d) — channels 12/13 may be missing\n", (int)err);
    }
    delay(250); // PHY needs a moment after start before the first scan is reliable
}

static void wifiRememberBest(WifiScanHit& hit, int index) {
    if (!WiFi.SSID(index).equals(wifiSSID)) return;
    int32_t rssi = WiFi.RSSI(index);
    if (hit.found && rssi <= hit.rssi) return;
    hit.found = true;
    hit.rssi = rssi;
    hit.channel = WiFi.channel(index);
    hit.auth = (int)WiFi.encryptionType(index);
    const uint8_t* bssid = WiFi.BSSID(index);
    if (bssid) memcpy(hit.bssid, bssid, 6);
}

// Active scan across every channel the country allows. If the SSID is still
// missing, one passive scan — some routers do not answer probe requests.
static WifiScanHit wifiScanForSsid() {
    WifiScanHit hit = {};
    hit.rssi = -999;

    int foundCount = WiFi.scanNetworks(false, true, false, 300);
    if (foundCount <= 0) {
        delay(200);
        foundCount = WiFi.scanNetworks(false, true, false, 300);
    }
    int usable = foundCount > 0 ? foundCount : 0;
    for (int i = 0; i < usable; i++) wifiRememberBest(hit, i);

    if (!hit.found) {
        WiFi.scanDelete();
        foundCount = WiFi.scanNetworks(false, true, true, 350);
        usable = foundCount > 0 ? foundCount : 0;
        for (int i = 0; i < usable; i++) wifiRememberBest(hit, i);
    }

    if (systemDebugEnabled) {
        printLogHeader("[WIFI   ] ");
        if (hit.found) {
            // No newline: dots from wifiWaitConnected continue on this same line.
            Serial.printf("Found ch=%d rssi=%d auth=%s ", hit.channel, hit.rssi, wifiAuthName(hit.auth));
        } else {
            Serial.printf("SSID not visible on 2.4 GHz ch 1-13 (%d other network(s))\n", usable);
            int shown = usable > 8 ? 8 : usable;
            for (int i = 0; i < shown; i++) {
                printLogHeader("[WIFI   ] ");
                Serial.printf("  seen ch=%d rssi=%d ", WiFi.channel(i), WiFi.RSSI(i));
                String seen = WiFi.SSID(i);
                Serial.println(seen.length() ? seen : "(hidden)");
            }
        }
    }

    WiFi.scanDelete();
    return hit;
}

// Leave scan mode before begin(). Calling begin() while the radio is still in a
// post-scan state often keeps WiFi.status() at IDLE for the whole timeout.
static void wifiLeaveScanMode() {
    WiFi.scanDelete();
    WiFi.disconnect(false, false);
    delay(200);
}

static bool wifiWaitConnected(unsigned long timeoutMs) {
    unsigned long startAttemptTime = millis();
    bool toggleBlink = false;
    while (WiFi.status() != WL_CONNECTED && (millis() - startAttemptTime < timeoutMs)) {
        delay(500);
        toggleBlink = !toggleBlink;
        if (toggleBlink) setRgbColor(0, 0, 50);
        else rgbOff();
        if (systemDebugEnabled) Serial.print(".");
    }
    return WiFi.status() == WL_CONNECTED;
}

static void wifiNoteConnected() {
    if (systemDebugEnabled) {
        Serial.println();
        printLogHeader("[WIFI   ] ");
        Serial.print("Connected. IP: ");
        Serial.println(WiFi.localIP());
    }
    setRgbColor(0, 0, 50);
    cachedChannel = WiFi.channel();
    memcpy(cachedBSSID, WiFi.BSSID(), 6);
    cachedValid = true;
    WiFi.setAutoReconnect(true);
}

static void wifiNoteFailed(const char* why) {
    if (!systemDebugEnabled) return;
    Serial.println();
    printLogHeader("[WIFI   ] ");
    Serial.printf("%s (status=%s", why, wifiStatusName(WiFi.status()));
    if (wifiLastDiscReason != 0) {
        const char* reasonName = wifiReasonName(wifiLastDiscReason);
        Serial.printf(", reason=%u", wifiLastDiscReason);
        if (reasonName[0] != '\0') {
            Serial.print(" ");
            Serial.print(reasonName);
        }
    }
    Serial.println(")");
}

// Connects to the configured WiFi network. After deep sleep the cached channel/BSSID
// is tried first. Otherwise scan to confirm the AP, then join without locking BSSID —
// forcing BSSID right after scanNetworks often leaves STA in IDLE (reason 8 = leave).
// Returns true on success, false when both attempts time out.
bool connectWiFi() {
    if (strlen(wifiSSID) == 0) return false;

    static bool eventsHooked = false;
    if (!eventsHooked) {
        WiFi.onEvent(onWifiStaEvent);
        eventsHooked = true;
    }

    for (uint8_t attempt = 0; attempt < 2; attempt++) {
        bool useCache = cachedValid && (attempt == 0);
        wifiLastDiscReason = 0;

        // Erase the previous BSSID/channel while STA is up — disconnect() before
        // WiFi.mode(STA) cannot clear NVS, and a leftover channel locks the next scan.
        WiFi.persistent(false);
        WiFi.setAutoReconnect(false);
        WiFi.mode(WIFI_STA);
        WiFi.disconnect(false, true);
        delay(100);
        if (!useCache) {
            WiFi.mode(WIFI_OFF);
            delay(200);
        }
        wifiApplyRadio();

        if (useCache) {
            if (systemDebugEnabled) {
                printLogHeader("[WIFI   ] ");
                Serial.printf("Connecting (cached ch=%u) ", cachedChannel);
            }
            WiFi.begin(wifiSSID, wifiPass, cachedChannel, cachedBSSID);
            if (wifiWaitConnected(8000)) {
                wifiNoteConnected();
                return true;
            }
            wifiNoteFailed("Cached connect timed out.");
            cachedValid = false;
            rgbOff();
            continue;
        }

        if (systemDebugEnabled) {
            printLogHeader("[WIFI   ] ");
            Serial.print(attempt == 0 ? "Scanning all 2.4 GHz channels for " : "Retrying with full scan for ");
            Serial.println(wifiSSID);
        }
        setRgbColor(0, 0, 50);
        WifiScanHit hit = wifiScanForSsid();
        wifiLeaveScanMode();

        if (hit.found) {
            // Prefer channel hint without BSSID lock. Second pass locks BSSID only if
            // the open join timed out — some APs reject a forced BSSID right after scan.
            if (attempt == 0) {
                WiFi.begin(wifiSSID, wifiPass, hit.channel);
            } else {
                WiFi.begin(wifiSSID, wifiPass, hit.channel, hit.bssid);
            }
        } else {
            // Hidden SSID: beacons omit the name, a plain begin() still sends a directed probe.
            WiFi.begin(wifiSSID, wifiPass);
        }

        if (wifiWaitConnected(15000)) {
            wifiNoteConnected();
            return true;
        }

        wifiNoteFailed(hit.found ? "AP heard but association failed." : "Connection timed out.");
        cachedValid = false;
    }

    rgbOff(); // clear last connect-blink frame so blue does not stick into the portal
    return false;
}

// Powers up the SIM7080, registers on LTE-M/NB-IoT and opens a GPRS session.
// Returns true when GPRS is connected; powers the modem off again on failure.
bool connectCellular() {
    String techStr = (telemetryMode == 2) ? "LTE Cat-NB1 (NB-IoT)" : "LTE Cat-M1 (LTE-M)";
    if (systemDebugEnabled) {
        printLogHeader("[CELL   ] ");
        if (cellularDebugEnabled) {
            Serial.printf("Connecting via %s (telemetryMode=%u)\n", techStr.c_str(), telemetryMode);
        } else {
            Serial.println("Connecting via " + techStr);
        }
    }

    // Cyan while modem is up — distinct from WiFi blue, portal purple, measure orange
    setRgbColor(0, 45, 50);

    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
    delay(50);

    // Flush any stale UART data (boot echoes, AT+CPOWD leftovers) before checking
    while (SerialAT.available()) SerialAT.read();

    // Send AT and wait for a real "OK" — not just any byte in the buffer
    SerialAT.print("AT\r\n");
    delay(300);
    String initReply = "";
    while (SerialAT.available()) initReply += (char)SerialAT.read();
    bool modemAlreadyOn = initReply.indexOf("OK") >= 0;
    if (cellularDebugEnabled) {
        printLogHeader("[CELL   ] ");
        Serial.println(modemAlreadyOn ? "Modem already responding to AT." : "Modem not responding — PWRKEY power-on ...");
    }

    if (!modemAlreadyOn) {
        // Modem is off — trigger PWRKEY sequence then poll for ready
        pinMode(MODEM_PWRKEY, OUTPUT);
        digitalWrite(MODEM_PWRKEY, HIGH); 
        delay(100);
        digitalWrite(MODEM_PWRKEY, LOW); 
        delay(1500); 
        digitalWrite(MODEM_PWRKEY, HIGH);

        // Poll every second: flush boot noise, send AT, check for OK — up to 20 seconds
        bool modemResponding = false;
        for (int attempt = 0; attempt < 20; attempt++) {
            delay(1000);
            while (SerialAT.available()) SerialAT.read(); // discard boot messages (RDY, +CFUN etc.)
            SerialAT.print("AT\r\n");
            delay(300);
            String reply = "";
            while (SerialAT.available()) reply += (char)SerialAT.read();
            if (reply.indexOf("OK") >= 0) {
                modemResponding = true;
                if (systemDebugEnabled) {
                    printLogHeader("[CELL   ] ");
                    Serial.printf("Modem ready after %ds\n", attempt + 1);
                }
                break;
            }
        }
        if (!modemResponding) {
            if (systemDebugEnabled) { printLogHeader("[CELL   ] "); Serial.println("Modem did not respond within 20s."); }
            shutdownModem();
            return false;
        }
    }

    if (cellularDebugEnabled) { printLogHeader("[CELL   ] "); Serial.print("modem.init() ... "); }
    if (!modem.init()) {
        if (cellularDebugEnabled) Serial.println("FAIL");
        shutdownModem();
        return false;
    }
    if (cellularDebugEnabled) Serial.println("OK");

    SerialAT.println("ATE0");
    delay(50); 

    if (enableModemLed) {
        SerialAT.println("AT+CNETLIGHT=1");
    } else {
        SerialAT.println("AT+CNETLIGHT=0");
    }
    delay(100);

    // CMNB is AUTO_SAVE in modem NVM — only write when current mode differs
    if (telemetryMode == 1 || telemetryMode == 2) {
        int desiredCmnb = (telemetryMode == 2) ? 2 : 1;
        int currentCmnb = -1;

        while (SerialAT.available()) SerialAT.read();
        SerialAT.print("AT+CMNB?\r\n");
        delay(300);
        String cmnbReply = "";
        while (SerialAT.available()) cmnbReply += (char)SerialAT.read();
        int cmnbIdx = cmnbReply.indexOf("+CMNB:");
        if (cmnbIdx >= 0) {
            currentCmnb = cmnbReply.substring(cmnbIdx + 6).toInt();
        }

        if (currentCmnb == desiredCmnb) {
            if (cellularDebugEnabled) {
                printLogHeader("[CELL   ] ");
                Serial.printf("Preferred mode already AT+CMNB=%d — skip write\n", desiredCmnb);
            }
        } else {
            if (cellularDebugEnabled) {
                printLogHeader("[CELL   ] ");
                Serial.printf("Preferred mode AT+CMNB?=%d → setting AT+CMNB=%d\n", currentCmnb, desiredCmnb);
            }
            modem.sendAT("+CMNB=", desiredCmnb);
            modem.waitResponse();
        }
    }

    // NB-IoT only. Default CNBS=3 also scans weak cells at 200 ms and 500 ms per
    // raster; that search is what makes a cold attach cost more than LTE-M.
    // CNBS=1 keeps the strong-cell pass (30 ms per raster). AUTO_SAVE: the new
    // limit applies on the next modem power-up, not in this session.
    if (telemetryMode == 2) {
        while (SerialAT.available()) SerialAT.read();
        SerialAT.print("AT+CNBS?\r\n");
        delay(300);
        String cnbsReply = "";
        while (SerialAT.available()) cnbsReply += (char)SerialAT.read();
        int cnbsIdx = cnbsReply.indexOf("+CNBS:");
        int currentCnbs = (cnbsIdx >= 0) ? cnbsReply.substring(cnbsIdx + 6).toInt() : -1;
        if (currentCnbs == 1) {
            if (cellularDebugEnabled) {
                printLogHeader("[CELL   ] ");
                Serial.println("NB-IoT band scan already AT+CNBS=1 (strong cells only)");
            }
        } else {
            if (cellularDebugEnabled) {
                printLogHeader("[CELL   ] ");
                Serial.printf("NB-IoT band scan AT+CNBS?=%d → AT+CNBS=1 (active after next power-up)\n", currentCnbs);
            }
            modem.sendAT("+CNBS=1");
            modem.waitResponse();
        }
    }

    if (cellularDebugEnabled) { printLogHeader("[CELL   ] "); Serial.println("Waiting for network (max 180s) ..."); }
    if (!modem.waitForNetwork(180000L)) {
        if (cellularDebugEnabled) {
            printLogHeader("[CELL   ] ");
            Serial.printf("FAIL: waitForNetwork (reg=%d, signal=%d)\n",
                          modem.getRegistrationStatus(), modem.getSignalQuality());
        }
        shutdownModem();
        return false;
    }
    if (cellularDebugEnabled) {
        printLogHeader("[CELL   ] ");
        Serial.printf("Network registered (reg=%d, signal=%d)\n",
                      modem.getRegistrationStatus(), modem.getSignalQuality());
    }

    if (cellularDebugEnabled) {
        printLogHeader("[CELL   ] ");
        Serial.printf("gprsConnect(APN=\"%s\") ... ", lteAPN);
    }
    if (!modem.gprsConnect(lteAPN, lteUser, ltePass)) {
        if (cellularDebugEnabled) Serial.println("FAIL");
        shutdownModem();
        return false;
    }
    if (cellularDebugEnabled) Serial.println("OK");

    if (!modem.isGprsConnected()) {
        if (cellularDebugEnabled) { printLogHeader("[CELL   ] "); Serial.println("FAIL: isGprsConnected=false after connect"); }
        shutdownModem();
        return false;
    }
    if (cellularDebugEnabled) { printLogHeader("[CELL   ] "); Serial.println("GPRS connected."); }
    return true;
}

// Normal power-off: the modem detaches from the network, then shuts down.
// Urgent CPOWD=0 skipped that detach and did not reduce the measured charge.
void shutdownModem() {
    if (systemDebugEnabled) {
        printLogHeader("[CELL   ] ");
        Serial.print("Powering modem off ... ");
    }
    while (SerialAT.available()) SerialAT.read();
    SerialAT.print("AT+CPOWD=1\r\n");

    unsigned long start = millis();
    String reply = "";
    bool poweredDown = false;
    while (millis() - start < 10000) {
        while (SerialAT.available()) {
            reply += (char)SerialAT.read();
            if (reply.indexOf("NORMAL POWER DOWN") >= 0) {
                poweredDown = true;
                break;
            }
            if (reply.length() > 80) reply.remove(0, reply.length() - 40);
        }
        if (poweredDown) break;
        delay(10);
    }
    if (systemDebugEnabled) Serial.println(poweredDown ? "OK" : "timeout");
    rgbOff();
}

// Configures ArduinoOTA callbacks and starts the OTA service for the portal window
void setupOTA() {
    ArduinoOTA.setHostname(otaHostname);
    ArduinoOTA.setPassword(sharedCrypto);

    // Reset the portal activity timer on every OTA event so the portal loop
    // cannot time out and exit while a firmware upload is in progress.
    ArduinoOTA.onStart([]() {
        lastPortalActivity = millis();
        if (systemDebugEnabled) {
            String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
            printLogHeader("[OTA    ] "); Serial.println("OTA upload started (" + type + ")");
        }
    });

    ArduinoOTA.onEnd([]() {
        lastPortalActivity = millis();
        if (systemDebugEnabled) { printLogHeader("[OTA    ] "); Serial.println("OTA upload complete. Rebooting ..."); }
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        lastPortalActivity = millis(); // keep portal alive during multi-second flash
        if (systemDebugEnabled) Serial.printf("[OTA    ] Progress: %u%%\r", (progress / (total / 100)));
    });

    ArduinoOTA.onError([](ota_error_t error) {
        if (systemDebugEnabled) {
            printLogHeader("[OTA    ] "); Serial.printf("OTA error [%u]: ", error);
            if      (error == OTA_AUTH_ERROR)    Serial.println("Authentication failed — check OTA password");
            else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin failed — not enough flash space?");
            else if (error == OTA_CONNECT_ERROR) Serial.println("Connect failed");
            else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive failed");
            else if (error == OTA_END_ERROR)     Serial.println("End failed");
        }
    });

    ArduinoOTA.begin();
}

// Brings one SCD41 into idle, ready for single-shot measurement (Sensirion §3.11).
// Every cycle: wake_up only (sensor was power_down'd last cycle; NACK on wake is normal).
// Cold start: stop + reinit clean state, then ASC off / altitude / persist once.
void initSensor(uint8_t port, bool firstBoot) {
    selectSensorPort(port);
    sensors[port].begin(Wire, SCD41_I2C_ADDR);

    // wake_up: library waits 30 ms and ignores I2C NACK (datasheet: not acknowledged)
    sensors[port].wakeUp();

    if (firstBoot) {
        // Clean idle state once after power-on (matches Sensirion example setup)
        sensors[port].stopPeriodicMeasurement(); // library waits 500 ms
        sensors[port].reinit();                  // library waits 30 ms; reloads EEPROM into RAM
        sensors[port].setAutomaticSelfCalibrationEnabled(false); // manual FRC only
        sensors[port].setSensorAltitude(sensorAltitude);
        sensors[port].persistSettings(); // library waits 800 ms for EEPROM write
    }
}

// Reads the battery ADC and converts it to voltageBattery [V]
void measureBattery() {
    int rawValue = analogRead(batteryPin);        
    voltageBattery = (rawValue * 2.0) / 1177.0;  
}

// Starts outdoor forced recalibration (FRC) for all active sensors via the portal.
// Sensirion FRC prep for single-shot / power-cycled field use: idle single-shot at
// 1-minute intervals × 5 (sensors stay awake), then FRC immediately on the last reading.
// Normal field measurements may still powerDown between cycles; FRC prep must not
// powerDown before the FRC command or the operated-state / last sample is lost.
void handleStartCal() {
    uint16_t frcTargetPpm = SCD41_FRC_TARGET_FRESH_PPM;
    if (server.hasArg("ppm")) {
        int requestedPpm = server.arg("ppm").toInt();
        if (requestedPpm < SCD41_FRC_TARGET_MIN_PPM || requestedPpm > SCD41_FRC_TARGET_MAX_PPM) {
            server.send(400, "text/plain", "PPM_OUT_OF_RANGE");
            return;
        }
        frcTargetPpm = (uint16_t)requestedPpm;
    }
    server.send(200, "text/plain", "STARTED");
    delay(500);
    cancelCalTriggered = false;
    setRgbColor(35, 0, 50);
    calLogBuffer = "=== INITIALIZING SINGLE-SHOT FRC CALIBRATION ===\n";

    auto dualLog = [](String text) {
        Serial.println(text);
        calLogBuffer += text + "\n";
    };

    auto waitWithPortal = [&](unsigned long durationMs) {
        unsigned long start = millis();
        int lastLoggedSecond = -1;
        while (millis() - start < durationMs) {
            server.handleClient();
            yield();
            if (cancelCalTriggered) return;
            int secondsRemaining = (int)((durationMs - (millis() - start) + 999) / 1000);
            if (secondsRemaining % 10 == 0 && secondsRemaining != lastLoggedSecond && secondsRemaining > 0) {
                dualLog(String(secondsRemaining) + " seconds until next prep shot ...");
                lastLoggedSecond = secondsRemaining;
                digitalWrite(2, !digitalRead(2));
            }
            delay(10);
        }
    };

    dualLog("FRC target: " + String(frcTargetPpm) + " ppm. Keep this concentration stable until calibration finishes.");
    dualLog("FRC prep mode: idle single-shot (stay awake) — required before FRC.");
    dualLog("Field measurements remain power-cycled single-shot after calibration.");
    uint8_t numI2CDevices = activeSensors;
    uint8_t totalGroups = (numI2CDevices > 4) ? 2 : 1;
    uint8_t splitIndex = (totalGroups == 2) ? 4 : numI2CDevices;
    int firstPersistedPort = -1;
    int lastPersistedPort = -1;

    for (uint8_t g = 0; g < totalGroups; g++) {
        if (cancelCalTriggered) break;
        uint8_t startPort = (g == 0) ? 0 : splitIndex;
        uint8_t endPort = (g == 0) ? splitIndex : numI2CDevices;
        if (startPort == endPort) continue;

        dualLog("\n----------------------------------------");
        dualLog("starting calibration for group " + String(g + 1));
        dualLog("ports: " + String(startPort) + " to " + String(endPort - 1));
        dualLog("----------------------------------------");

        for (uint8_t port = startPort; port < endPort; port++) {
            if (cancelCalTriggered) break;
            selectSensorPort(port);
            sensors[port].begin(Wire, SCD41_I2C_ADDR);
            delay(20);
            sensors[port].wakeUp();
            delay(50);
            sensors[port].stopPeriodicMeasurement();
            delay(500);
            sensors[port].reinit();
            delay(1000);

            int16_t tErr = sensors[port].setTemperatureOffset(0.0);
            if (tErr == 0) {
                dualLog("port " + String(port) + ": temperature offset cleared to 0.0 °C");
            } else {
                dualLog("port " + String(port) + ": failed to clear temperature offset!");
            }

            int16_t aErr = sensors[port].setSensorAltitude(sensorAltitude);
            if (aErr == 0) {
                dualLog("port " + String(port) + ": sensor altitude set to " + String(sensorAltitude) + " meters");
            } else {
                dualLog("port " + String(port) + ": failed to set sensor altitude!");
            }

            sensors[port].setAutomaticSelfCalibrationEnabled(false);
            // Stay awake for idle single-shot FRC prep (do not powerDown here)
        }

        // One discard after initial wake, then 5 timed prep shots while powered
        dualLog("discarding first single-shot after wake ...");
        for (uint8_t port = startPort; port < endPort; port++) triggerRawSingleShot(port);
        delay(SCD41_GROUP_WAIT_MS);
        for (uint8_t port = startPort; port < endPort; port++) {
            server.handleClient();
            yield();
            if (!waitSensorDataReady(port, SCD41_DATA_READY_EXTRA_MS)) {
                dualLog("port " + String(port) + ": discard data-ready timeout");
                continue;
            }
            uint16_t dCo2 = 0; float dTemp = 0.0f; float dHum = 0.0f;
            sensors[port].readMeasurement(dCo2, dTemp, dHum);
        }

        dualLog("FRC prep: " + String(SCD41_FRC_PREP_SHOTS) +
                " idle single-shots at 1-minute intervals (no powerDown) ...");

        for (uint8_t prep = 0; prep < SCD41_FRC_PREP_SHOTS; prep++) {
            if (cancelCalTriggered) break;
            unsigned long cycleStart = millis();
            dualLog("\nprep shot " + String(prep + 1) + "/" + String(SCD41_FRC_PREP_SHOTS) + " ...");

            for (uint8_t port = startPort; port < endPort; port++) triggerRawSingleShot(port);
            delay(SCD41_GROUP_WAIT_MS);
            String prepLine = "prep " + String(prep + 1) + "/" + String(SCD41_FRC_PREP_SHOTS) + ": ";
            for (uint8_t port = startPort; port < endPort; port++) {
                server.handleClient();
                yield();
                uint16_t co2Val = 0; float tempVal = 0.0f; float humVal = 0.0f;
                if (!waitSensorDataReady(port, SCD41_DATA_READY_EXTRA_MS)) {
                    prepLine += "[P" + String(port) + "] TIMEOUT";
                } else {
                    sensors[port].readMeasurement(co2Val, tempVal, humVal);
                    prepLine += "[P" + String(port) + "] " + String(co2Val) + "ppm | " +
                                String(tempVal, 1) + " °C | " + String(humVal, 1) + "%";
                }
                if (port < endPort - 1) prepLine += "  ||  ";
            }
            dualLog(prepLine);

            // Keep awake — FRC needs operated state + last sample still valid
            if (prep + 1 < SCD41_FRC_PREP_SHOTS && !cancelCalTriggered) {
                unsigned long elapsed = millis() - cycleStart;
                if (elapsed < SCD41_FRC_PREP_INTERVAL_MS) {
                    waitWithPortal(SCD41_FRC_PREP_INTERVAL_MS - elapsed);
                }
            }
        }

        // FRC immediately after last prep shot (still awake, idle — not after powerDown)
        for (uint8_t port = startPort; port < endPort; port++) {
            if (cancelCalTriggered) break;
            selectSensorPort(port);
            sensors[port].stopPeriodicMeasurement();
            delay(500);
            uint16_t frcCorrection = 0;
            int16_t err = sensors[port].performForcedRecalibration(frcTargetPpm, frcCorrection);
            if (err == 0 && frcCorrection != 0xFFFF) {
                int16_t corrPpm = (int16_t)((int32_t)frcCorrection - 0x8000);
                dualLog("port " + String(port) + " CO₂ success! correction offset: " + String(corrPpm) + " ppm");
            } else {
                dualLog("port " + String(port) + " failed CO₂ calibration! (err=" + String(err) +
                        ", raw=" + String(frcCorrection) + ")");
            }
            sensors[port].setAutomaticSelfCalibrationEnabled(false);
            sensors[port].persistSettings();
            if (firstPersistedPort < 0) firstPersistedPort = port;
            lastPersistedPort = port;
            delay(500);
            sensors[port].powerDown();
        }
    }

    if (cancelCalTriggered) {
        dualLog("\n⚠️ CALIBRATION ABORTED BY USER FROM BROWSER INTERFACE!");
        if (firstPersistedPort < 0) {
            dualLog("no hardware values were permanently altered. entering standby mode.");
        } else {
            dualLog("WARNING: calibration incomplete — ports " + String(firstPersistedPort) +
                    " to " + String(lastPersistedPort) + " already persisted to EEPROM.");
            dualLog("re-run outdoor calibration (or factory-reset affected sensors) before trusting readings.");
        }
        dualLog("=== END ===");
        for (uint8_t port = 0; port < numI2CDevices; port++) {
            selectSensorPort(port);
            sensors[port].wakeUp();
            delay(20);
            sensors[port].powerDown();
        }
        return;
    }

    dualLog("\n=== CALIBRATION COMPLETE! RUNNING SINGLE-SHOT VERIFICATION ===");
    setRgbColor(60, 20, 0);

    for (uint8_t port = 0; port < numI2CDevices; port++) {
        selectSensorPort(port);
        sensors[port].wakeUp();
    }

    // Discard first shot after wake
    for (uint8_t port = 0; port < numI2CDevices; port++) triggerRawSingleShot(port);
    delay(SCD41_GROUP_WAIT_MS);
    for (uint8_t port = 0; port < numI2CDevices; port++) {
        server.handleClient();
        yield();
        if (waitSensorDataReady(port, SCD41_DATA_READY_EXTRA_MS)) {
            uint16_t dCo2 = 0; float dTemp = 0.0f; float dHum = 0.0f;
            sensors[port].readMeasurement(dCo2, dTemp, dHum);
        }
    }

    for (int readCycle = 1; readCycle <= 5; readCycle++) {
        unsigned long cycleStart = millis();
        for (uint8_t port = 0; port < numI2CDevices; port++) triggerRawSingleShot(port);
        delay(SCD41_GROUP_WAIT_MS);

        String dataLine = "Data Cycle " + String(readCycle) + ": ";
        for (uint8_t port = 0; port < numI2CDevices; port++) {
            server.handleClient();
            yield();
            uint16_t co2Val = 0; float tempVal = 0.0f; float humidVal = 0.0f;
            if (!waitSensorDataReady(port, SCD41_DATA_READY_EXTRA_MS)) {
                dataLine += "[P" + String(port) + "] TIMEOUT";
            } else {
                sensors[port].readMeasurement(co2Val, tempVal, humidVal);
                dataLine += "[P" + String(port) + "] " + String(co2Val) + "ppm | " +
                            String(tempVal, 1) + " °C | " + String(humidVal, 1) + "%";
            }
            if (port < numI2CDevices - 1) dataLine += "  ||  ";
        }
        dualLog(dataLine);
        digitalWrite(2, !digitalRead(2));

        if (readCycle < 5) {
            while (millis() - cycleStart < 5000) {
                server.handleClient();
                yield();
                delay(10);
            }
        }
    }

    for (uint8_t port = 0; port < numI2CDevices; port++) {
        selectSensorPort(port);
        sensors[port].powerDown();
    }

    dualLog("\n=== Verification completed successfully. ===");
    dualLog("Readings in this same air should be near ~" + String(frcTargetPpm) + " ppm. Then press OK or wait 15 s for reboot.");
    dualLog("=== END ===");

    uint32_t autoRebootStart = millis();
    while (millis() - autoRebootStart < 15000) {
        server.handleClient(); yield(); delay(10);
    }
    setRgbColor(0, 0, 0);
    delay(500);
    ESP.restart();
}

// Reads the HTTP status line from a TCP client, drains headers/body, returns the status code
int readHttpStatusAndDrain(Client& client, unsigned long timeoutMs) {
    unsigned long start = millis();
    int statusCode = 0;
    int contentLength = 0;
    bool gotStatus = false;
    bool headersDone = false;

    while (millis() - start < timeoutMs) {
        while (client.available()) {
            String line = client.readStringUntil('\n');
            line.trim();
            if (!gotStatus) {
                if (line.startsWith("HTTP/1.")) {
                    int sp = line.indexOf(' ');
                    if (sp > 0) {
                        statusCode = line.substring(sp + 1).toInt();
                    }
                    gotStatus = true;
                }
                continue;
            }
            if (!headersDone) {
                if (line.length() == 0) {
                    headersDone = true;
                    break;
                }
                String lower = line;
                lower.toLowerCase();
                if (lower.startsWith("content-length:")) {
                    contentLength = lower.substring(15).toInt();
                }
            }
        }
        if (gotStatus && headersDone) break;
        delay(10);
        yield();
    }

    unsigned long bodyStart = millis();
    while (contentLength > 0 && millis() - bodyStart < timeoutMs) {
        if (client.available()) {
            client.read();
            contentLength--;
        } else {
            delay(5);
            yield();
        }
    }
    return statusCode;
}

// Uploads every buffered frame. Frames that get HTTP 2xx are removed.
// Frames that fail stay in rtcStorage, in order, and are retried on the next wake together with newer measurements.
void uploadData() {
    uint32_t frameCount = rtcCounter;
    if (frameCount > RTC_BUFFER_CAP) frameCount = RTC_BUFFER_CAP;
    bool frameSent[RTC_BUFFER_CAP] = {};

    if (systemDebugEnabled) { printLogHeader("[HTTP   ] "); Serial.printf("Uploading %u buffered data frame(s) ...\n", frameCount); }

    if (telemetryMode == 0) { // ==================== WIFI UPLOAD ====================
        if (connectWiFi()) {
            for (uint32_t d = 0; d < frameCount; d++) {
                HTTPClient http;
                String fullURL = "http://" + String(serverURL) + String(serverResource);
                http.begin(fullURL);
                http.addHeader("Content-Type", "application/x-www-form-urlencoded");

                String postData = "time=" + String(rtcStorage[d].timestamp);
                for (uint8_t i = 0; i < activeSensors; i++) {
                    if (rtcStorage[d].valid[i]) {
                        postData += "&co2-" + String(i + 1) + "=" + String(rtcStorage[d].co2[i]);
                        postData += "&temp-" + String(i + 1) + "=" + String(rtcStorage[d].temp[i], 2);
                        postData += "&hum-" + String(i + 1) + "=" + String(rtcStorage[d].hum[i], 2);
                    }
                }
                postData += "&battery=" + String(rtcStorage[d].battery, 2);

                if (systemDebugEnabled) { printLogHeader("[WIFI   ] "); Serial.printf("Sending data frame %u/%u (Time: %u) ... ", d + 1, frameCount, rtcStorage[d].timestamp); }
                
                int responseCode = http.POST(postData);
                if (responseCode >= 200 && responseCode < 300) {
                    frameSent[d] = true;
                    if (systemDebugEnabled) Serial.printf("OK (HTTP %d)\n", responseCode);
                } else {
                    if (systemDebugEnabled) Serial.printf("FAILED (HTTP %d)\n", responseCode);
                }
                http.end();
                delay(100); 
            }
            WiFi.disconnect(true);
            rgbOff();
        }
    } else { // ==================== CELLULAR UPLOAD (single persistent TCP socket) ====================
        if (connectCellular()) {
            TinyGsmClient cellularClient(modem);

            // Open one TCP socket and reuse it for all frames to minimise overhead
            if (cellularClient.connect(serverURL, 80)) {
                
                for (uint32_t d = 0; d < frameCount; d++) {
                    String postData = "time=" + String(rtcStorage[d].timestamp);
                    for (uint8_t i = 0; i < activeSensors; i++) {
                        if (rtcStorage[d].valid[i]) {
                            postData += "&co2-" + String(i + 1) + "=" + String(rtcStorage[d].co2[i]);
                            postData += "&temp-" + String(i + 1) + "=" + String(rtcStorage[d].temp[i], 2);
                            postData += "&hum-" + String(i + 1) + "=" + String(rtcStorage[d].hum[i], 2);
                        }
                    }
                    postData += "&battery=" + String(rtcStorage[d].battery, 2);

                    if (systemDebugEnabled) { printLogHeader("[CELL   ] "); Serial.printf("Streaming data frame %u/%u (Time: %u) ... ", d + 1, frameCount, rtcStorage[d].timestamp); }

                    // NB-IoT: ask the network to drop RRC once this POST's reply arrives,
                    // instead of sitting out the carrier inactivity timer.
                    if (telemetryMode == 2 && d + 1 == frameCount) {
                        modem.sendAT("+CRAI=2,1");
                        modem.waitResponse(1000L);
                    }

                    String purePath = String(serverResource);
                    if (purePath.indexOf("/api-notls/") != -1) {
                        purePath = purePath.substring(purePath.indexOf("/api-notls/"));
                    }
                    if (purePath.length() == 0 || purePath.charAt(0) != '/') {
                        purePath = "/" + purePath;
                    }

                    String connectionHeader = (d == frameCount - 1) ? "close" : "keep-alive";
                    int payloadLength = postData.length();

                    String httpHeaders = "";
                    httpHeaders += "POST " + purePath + " HTTP/1.1\r\n";
                    httpHeaders += "Host: " + String(serverURL) + "\r\n";
                    httpHeaders += "Content-Type: application/x-www-form-urlencoded\r\n";
                    httpHeaders += "Content-Length: " + String(payloadLength) + "\r\n";
                    httpHeaders += "Connection: " + connectionHeader + "\r\n\r\n";

                    cellularClient.print(httpHeaders);
                    cellularClient.print(postData);

                    int statusCode = readHttpStatusAndDrain(cellularClient, 8000);
                    bool httpOk = (statusCode >= 200 && statusCode < 300);
                    if (httpOk) frameSent[d] = true;
                    if (systemDebugEnabled) {
                        if (httpOk) {
                            Serial.printf("OK (HTTP %d)\n", statusCode);
                        } else if (statusCode > 0) {
                            Serial.printf("FAILED (HTTP %d)\n", statusCode);
                        } else {
                            Serial.println("FAILED (No Response)");
                        }
                    }

                    delay(300);
                }

                cellularClient.stop(); // close TCP socket after all frames have been sent
                
            } else {
                if (systemDebugEnabled) { printLogHeader("[CELL   ] "); Serial.println("Persistent socket tunnel connection failed."); }
            }
            
            shutdownModem();
        }
    }

    uint32_t kept = 0;
    for (uint32_t d = 0; d < frameCount; d++) {
        if (!frameSent[d]) {
            if (kept != d) rtcStorage[kept] = rtcStorage[d];
            kept++;
        }
    }
    uint32_t sentCount = frameCount - kept;
    rtcCounter = kept;

    if (kept == 0) {
        rtcRetryPending = 0;
        if (systemDebugEnabled) { printLogHeader("[HTTP   ] "); Serial.println("All data frames transmitted successfully. Clearing buffer."); }
        setRgbColor(0, 60, 0);
        delay(800);
        rgbOff();
    } else {
        rtcRetryPending = 1;
        if (systemDebugEnabled) {
            printLogHeader("[HTTP   ] ");
            if (sentCount == 0) {
                Serial.printf("Transmission failed. Kept %u frame(s) for next cycle:", kept);
            } else {
                Serial.printf("Sent %u of %u frame(s). Kept %u for next cycle:", sentCount, frameCount, kept);
            }
            for (uint32_t d = 0; d < kept; d++) {
                Serial.printf(" %u", rtcStorage[d].timestamp);
            }
            Serial.println();
        }
        setRgbColor(60, 0, 0);
        delay(800);
        rgbOff();
    }
}

// Syncs the ESP32 clock once via NTP (WiFi) or cell-tower time (LTE); skips if already synced
bool syncNetworkTime() {
    if (rtcTimeIsSynced) return true;

    if (telemetryMode == 0) {
        if (connectWiFi()) {
            if (systemDebugEnabled) { printLogHeader("[SYSTEM ] "); Serial.print("Requesting network time from NTP server ... "); }
            configTime(0, 0, "pool.ntp.org");
            struct tm timeinfo;
            if (getLocalTime(&timeinfo)) {
                time_t wifi_unix_time = mktime(&timeinfo);
                struct timeval tv = { .tv_sec = wifi_unix_time, .tv_usec = 0 };
                settimeofday(&tv, NULL);
                rtcTimeIsSynced = true;
                if (systemDebugEnabled) { Serial.printf("SUCCESS (Unix: %lu)\n", (unsigned long)wifi_unix_time); }
            } else if (systemDebugEnabled) {
                Serial.println("FAILED!");
            }
            WiFi.disconnect(true);
            rgbOff();
        }
    } else if (connectCellular()) {
        if (systemDebugEnabled) { printLogHeader("[SYSTEM ] "); Serial.print("Requesting network time from cell tower ... "); }
        int year = 0, month = 0, day = 0, hour = 0, min = 0, sec = 0;
        float timezone = 0;
        if (modem.getNetworkTime(&year, &month, &day, &hour, &min, &sec, &timezone)) {
            struct tm tm_info = {};
            tm_info.tm_year = year - 1900;
            tm_info.tm_mon = month - 1;
            tm_info.tm_mday = day;
            tm_info.tm_hour = hour;
            tm_info.tm_min = min;
            tm_info.tm_sec = sec;
            tm_info.tm_isdst = 0;

            // CCLK is network local time. With default UTC TZ, mktime() treats it as UTC,
            // so subtract the reported GMT offset to store a real unix timestamp.
            time_t network_unix_time = mktime(&tm_info);
            network_unix_time -= (time_t)(timezone * 3600.0f);

            struct timeval tv = { .tv_sec = network_unix_time, .tv_usec = 0 };
            settimeofday(&tv, NULL);
            rtcTimeIsSynced = true;
            if (systemDebugEnabled) { Serial.printf("SUCCESS (Unix: %lu)\n", (unsigned long)network_unix_time); }
        } else if (systemDebugEnabled) {
            Serial.println("FAILED!");
        }
        shutdownModem();
    } else if (cellularDebugEnabled) {
        printLogHeader("[SYSTEM ] ");
        Serial.println("Cellular connect failed — cannot sync time.");
    }

    return rtcTimeIsSynced;
}

// Main boot path: load config, optional portal, time sync, measure, buffer/upload, deep sleep
void setup() {
    Serial.begin(115200);
    delay(1000); 
    Wire.begin();
    pinMode(2, OUTPUT); digitalWrite(2, LOW);
    rgbLed.begin(); rgbLed.show();

    // Load configuration from NVS. If forceUpdateDefaults is true, overwrite stored values first.
    if (forceUpdateDefaults) {
        if (systemDebugEnabled) {
            printLogHeader("[STORAGE] ");
            Serial.println("forceUpdateDefaults active — overwriting NVS with code defaults ...");
        }
        saveConfigPrefs();
    }
    loadConfigPrefs();

    esp_sleep_wakeup_cause_t wakeupReason = esp_sleep_get_wakeup_cause();
    bool firstBoot = !(wakeupReason == ESP_SLEEP_WAKEUP_TIMER);

    if(systemDebugEnabled) {
        printLogHeader("[SYSTEM ] "); Serial.print("Firmware Version: v"); Serial.println(SW_VERSION); 
        printLogHeader("[SYSTEM ] "); Serial.println(firstBoot ? "Cold start - initial setup" : "Wakeup from deep sleep");
        if (rtcCounter > 0 && rtcCounter <= RTC_BUFFER_CAP) {
            printLogHeader("[HTTP   ] ");
            Serial.printf("Unsent from earlier cycle(s): %u frame(s):", rtcCounter);
            for (uint32_t d = 0; d < rtcCounter; d++) {
                Serial.printf(" %u", rtcStorage[d].timestamp);
            }
            Serial.println();
        }
    }

    // Soft/hard reset keeps RTC WiFi cache, but the AP often still holds the old STA
    // association — forced BSSID reconnect then fails. Only deep-sleep wakes keep the cache.
    if (firstBoot) {
        cachedValid = false;
    }

    time_t nowTime;

    // Restart after save is not a timer wake, so it would reopen the portal.
    // measnow is a one-shot NVS flag from the save checkbox; default stays portal.
    bool startMeasurement = false;
    if (firstBoot) {
        prefs.begin("beehive", false);
        startMeasurement = prefs.getString("measnow", "0") == "1";
        prefs.remove("measnow");
        prefs.end();
        if (startMeasurement && systemDebugEnabled) {
            printLogHeader("[PORTAL ] ");
            Serial.println("Measurement opted in at save — skipping portal");
        }
    }

    if (firstBoot && !startMeasurement) {
        pinMode(USER_BUTTON_PIN, INPUT_PULLUP);
        
        // Attempt WiFi connection; fall back to AP mode if unavailable
        bool connectedLocal = connectWiFi();
        
        if (!connectedLocal) {
            if (systemDebugEnabled) { printLogHeader("[PORTAL ] "); Serial.println("No local network found. Starting fallback Access Point ..."); }
            WiFi.softAP(otaHostname, sharedCrypto);
            // In AP mode mDNS is not available — connect to 192.168.4.1 directly.
            // OTA still works; target the device by IP in the Arduino IDE.
            if (systemDebugEnabled) { printLogHeader("[PORTAL ] "); Serial.println("AP active. Web/OTA reachable at http://192.168.4.1 (no mDNS in AP mode)"); }
        }
        
        server.on("/", handleRoot);
        server.on("/save", HTTP_POST, handleSave);
        server.on("/ping", handlePing); 
        server.on("/start-cal", handleStartCal);
        server.on("/cancel-cal", [&]() { cancelCalTriggered = true; server.send(200, "text/plain", "CANCELED"); });
        server.on("/cal-log", handleCalLog);
        server.on("/factory-reset", handleFactoryReset);
        server.on("/update", HTTP_POST, handleUpdate, handleUpdateUpload);
        server.begin();
        setupOTA();

        if (systemDebugEnabled) {
            int portalMinutes = PORTAL_TIMEOUT_SECONDS / 60;
            printLogHeader("[PORTAL ] "); Serial.println("-------------------------------------------------------------------------");
            if (connectedLocal) {
                printLogHeader("[PORTAL ] "); Serial.print("Web portal: http://"); Serial.print(WiFi.localIP()); Serial.println("/");
                printLogHeader("[PORTAL ] "); Serial.print("            http://"); Serial.print(otaHostname); Serial.println(".local/");
            }
            printLogHeader("[PORTAL ] "); Serial.print("OTA and web portal active for "); Serial.print(portalMinutes); Serial.println(" minutes");
            printLogHeader("[PORTAL ] "); Serial.println("Press USER button or send 'c' via serial to skip to measurement");
            printLogHeader("[PORTAL ] "); Serial.println("-------------------------------------------------------------------------");
        }

        lastPortalActivity = millis();

        while (millis() - lastPortalActivity < ((unsigned long)PORTAL_TIMEOUT_SECONDS * 1000)) {
            yield(); vTaskDelay(1);
            server.handleClient();
            ArduinoOTA.handle();   

            if (Serial.available() > 0) {
                char inputChar = Serial.read();
                if (inputChar == 'c' || inputChar == 'C') {
                    if (systemDebugEnabled) { printLogHeader("[PORTAL ] "); Serial.println("Skip triggered via Serial command!"); }
                    break; 
                }
            }
            if (digitalRead(USER_BUTTON_PIN) == LOW) {
                delay(50);
                if (digitalRead(USER_BUTTON_PIN) == LOW) {
                    if (systemDebugEnabled) { printLogHeader("[PORTAL ] "); Serial.println("Skip triggered via physical USER button!"); }
                    break; 
                }
            }

            if ((millis() - lastPortalActivity) % 300 < 150) {
                digitalWrite(2, HIGH); setRgbColor(35, 0, 50); 
            } else {
                digitalWrite(2, LOW); rgbOff();
            }
        }
        WiFi.softAPdisconnect(true); server.stop(); digitalWrite(2, LOW); rgbOff();
    }

    uint16_t co2[maxI2CDevices] = {0};
    float temperature[maxI2CDevices] = {0.0};
    float humidity[maxI2CDevices] = {0.0};
    bool sensorValid[maxI2CDevices] = {false};

    // Sync network time BEFORE measuring so the stored timestamp is always valid.
    // On subsequent wakeups rtcTimeIsSynced = true and this returns immediately (no network needed).
    // If sync fails (e.g. modem cold-start issue), skip this measurement cycle entirely
    // and go back to sleep — never store a measurement with an invalid timestamp.
    if (!syncNetworkTime()) {
        if (systemDebugEnabled) {
            printLogHeader("[SYSTEM ] ");
            Serial.println("Time sync failed. Skipping measurement this cycle, retrying next wakeup.");
        }
        I2CMulti.selectPort(8); delay(20); Wire.end(); delay(20);
        pinMode(21, INPUT); pinMode(22, INPUT);
        WiFi.disconnect(true); WiFi.mode(WIFI_OFF); delay(50);
        rgbOff(); delay(10); // NeoPixel latches color — floating the data pin alone keeps it lit
        pinMode(RGB_LED_PIN, INPUT);
        if (systemDebugEnabled) {
            printLogHeader("[POWER  ] "); Serial.printf("Enter ESP32 deep sleep. Timer set to: %ds\n", sleepSeconds);
            Serial.println("--- Entering Deep Sleep ---\n"); Serial.flush();
        }
        esp_sleep_enable_timer_wakeup((uint64_t)sleepSeconds * uS_TO_S_FACTOR);
        esp_deep_sleep_start();
    }

    // Orange LED marks active measurement — only after time sync succeeded
    setRgbColor(60, 20, 0);
    if (systemDebugEnabled) { printLogHeader("[SENSOR ] "); Serial.println(">>> Starting measurement array now ..."); }

    for (uint8_t port = 0; port < activeSensors; port++) initSensor(port, firstBoot);

    uint8_t totalGroups = (activeSensors > 4) ? 2 : 1;
    uint8_t splitIndex = (totalGroups == 2) ? 4 : activeSensors;

    uint32_t co2Sum[maxI2CDevices] = {0};
    float tempSum[maxI2CDevices] = {0.0f};
    float humSum[maxI2CDevices] = {0.0f};
    uint8_t sampleOk[maxI2CDevices] = {0};

    if (systemDebugEnabled) {
        printLogHeader("[SENSOR ] ");
        Serial.printf("Averaging %u sample(s) after discarding first single-shot\n", measurementSamples);
    }

    for (uint8_t g = 0; g < totalGroups; g++) {
        uint8_t startPort = (g == 0) ? 0 : splitIndex;
        uint8_t endPort = (g == 0) ? splitIndex : activeSensors;
        if (startPort == endPort) continue;

        // Always discard the first single-shot after wake (Sensirion recommendation)
        for (uint8_t port = startPort; port < endPort; port++) triggerRawSingleShot(port);
        delay(SCD41_GROUP_WAIT_MS); // whole group measures in parallel
        for (uint8_t port = startPort; port < endPort; port++) {
            if (!waitSensorDataReady(port, SCD41_DATA_READY_EXTRA_MS)) {
                if (systemDebugEnabled) {
                    printLogHeader("[SENSOR ] ");
                    Serial.printf("Port %u discard: data-ready timeout after %lu+%lu ms\n",
                                  port, (unsigned long)SCD41_GROUP_WAIT_MS,
                                  (unsigned long)SCD41_DATA_READY_EXTRA_MS);
                }
                continue;
            }
            uint16_t discardCo2 = 0; float discardTemp = 0.0f; float discardHum = 0.0f;
            sensors[port].readMeasurement(discardCo2, discardTemp, discardHum);
        }

        // Take measurementSamples readings and accumulate; keep sensors awake between shots
        for (uint8_t s = 0; s < measurementSamples; s++) {
            for (uint8_t port = startPort; port < endPort; port++) triggerRawSingleShot(port);
            delay(SCD41_GROUP_WAIT_MS); // whole group measures in parallel

            for (uint8_t port = startPort; port < endPort; port++) {
                if (!waitSensorDataReady(port, SCD41_DATA_READY_EXTRA_MS)) {
                    if (systemDebugEnabled) {
                        printLogHeader("[SENSOR ] ");
                        Serial.printf("Port %u sample %u REJECT data-ready timeout after %lu+%lu ms\n",
                                      port, s + 1, (unsigned long)SCD41_GROUP_WAIT_MS,
                                      (unsigned long)SCD41_DATA_READY_EXTRA_MS);
                    }
                    continue;
                }

                uint16_t sampleCo2 = 0; float sampleTemp = 0.0f; float sampleHum = 0.0f;
                int16_t readError = sensors[port].readMeasurement(sampleCo2, sampleTemp, sampleHum);
                bool sampleValid = (readError == NO_ERROR && sampleCo2 >= CO2_MIN_PLAUSIBLE);
                if (sampleValid) {
                    co2Sum[port] += sampleCo2;
                    tempSum[port] += sampleTemp;
                    humSum[port] += sampleHum;
                    sampleOk[port]++;
                } else if (systemDebugEnabled) {
                    printLogHeader("[SENSOR ] ");
                    if (readError != NO_ERROR) {
                        Serial.printf("Port %u sample %u REJECT readError=%d\n", port, s + 1, readError);
                    } else {
                        Serial.printf("Port %u sample %u REJECT CO₂=%u ppm (min %u)\n",
                                      port, s + 1, sampleCo2, CO2_MIN_PLAUSIBLE);
                    }
                }
            }
        }

        for (uint8_t port = startPort; port < endPort; port++) {
            if (sampleOk[port] > 0) {
                co2[port] = (uint16_t)((co2Sum[port] + (sampleOk[port] / 2)) / sampleOk[port]); // integer average
                temperature[port] = roundf((tempSum[port] / sampleOk[port]) * 100.0f) / 100.0f; // 2 decimals
                humidity[port] = roundf((humSum[port] / sampleOk[port]) * 100.0f) / 100.0f;     // 2 decimals
                sensorValid[port] = true;
            } else if (systemDebugEnabled) {
                printLogHeader("[SENSOR ] ");
                Serial.printf("Port %u: no plausible samples — marked invalid\n", port);
            }
            selectSensorPort(port);
            sensors[port].powerDown();
        }
    }
    rgbOff(); measureBattery();

    if (systemDebugEnabled) {
        for (uint8_t port = 0; port < activeSensors; port++) {
            printLogHeader("[DATA   ] ");
            if (sensorValid[port]) {
                Serial.printf("[Sensor-%d/Port-%d] CO₂: %d ppm | Temp: %.2f °C | Hum: %.2f %%RH (avg of %u)\n",
                              port+1, port, co2[port], temperature[port], humidity[port], sampleOk[port]);
            } else {
                Serial.printf("[Sensor-%d/Port-%d] UNPLUGGED or ERROR -> Skipping payload entry.\n", port+1, port);
            }
        }
    }

    time(&nowTime);

    if (rtcCounter > RTC_BUFFER_CAP) rtcCounter = RTC_BUFFER_CAP;
    if (rtcCounter == RTC_BUFFER_CAP) {
        if (systemDebugEnabled) {
            printLogHeader("[HTTP   ] ");
            Serial.printf("Unsent buffer full (%u). Dropping oldest frame (Time: %u) to keep the current measurement.\n",
                          rtcCounter, rtcStorage[0].timestamp);
        }
        for (uint32_t i = 1; i < rtcCounter; i++) rtcStorage[i - 1] = rtcStorage[i];
        rtcCounter--;
    }

    uint32_t pending = rtcCounter;
    rtcStorage[rtcCounter].timestamp = (uint32_t)nowTime;
    rtcStorage[rtcCounter].battery   = voltageBattery;
    for(uint8_t p = 0; p < activeSensors; p++) {
        rtcStorage[rtcCounter].co2[p]   = co2[p]; 
        rtcStorage[rtcCounter].temp[p]  = temperature[p];
        rtcStorage[rtcCounter].hum[p]   = humidity[p]; 
        rtcStorage[rtcCounter].valid[p] = sensorValid[p];
    }
    rtcCounter++;

    if (systemDebugEnabled && pending > 0) {
        printLogHeader("[HTTP   ] ");
        Serial.printf("Appending current measurement (Time: %u) to %u unsent frame(s). Buffer now %u.\n",
                      (uint32_t)nowTime, pending, rtcCounter);
    }

    // Upload once the threshold is reached, and keep uploading every wake while a previous attempt left frames behind.
    if (rtcCounter >= maxMeasurements) rtcRetryPending = 1;
    if (rtcRetryPending == 1 && rtcCounter > 0) {
        uploadData();
    } else {
        if (systemDebugEnabled) {
            printLogHeader("[POWER  ] "); 
            Serial.printf("Buffer not full yet (%u/%u). Keeping network interfaces off and entering deep sleep\n", rtcCounter, maxMeasurements);
        }
    }

    // Deselect all I2C multiplexer channels, stop the I2C bus and float SDA/SCL
    // to prevent parasitic current draw through the sensors during deep sleep
    I2CMulti.selectPort(8); delay(20); Wire.end(); delay(20);
    pinMode(21, INPUT); pinMode(22, INPUT);

    // Disconnect WiFi and fully power down the radio
    WiFi.disconnect(true); WiFi.mode(WIFI_OFF); delay(50);

    // Clear NeoPixel latch first, then float the data pin (INPUT alone leaves last color on)
    rgbOff(); delay(10);
    pinMode(RGB_LED_PIN, INPUT);

    if (systemDebugEnabled) {
        printLogHeader("[POWER  ] "); Serial.printf("Enter ESP32 deep sleep. Timer set to: %ds\n", sleepSeconds);
        Serial.println("--- Entering Deep Sleep ---\n"); Serial.flush();
    }

    esp_sleep_enable_timer_wakeup((uint64_t)sleepSeconds * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
}

// Unused: all work happens in setup() before deep sleep
void loop() {}
