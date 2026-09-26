#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_I2C_Multiplexer.h>
#include <SensirionI2cScd4x.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_sleep.h>
#include <ArduinoOTA.h>
#include <Adafruit_NeoPixel.h>
#include <WebServer.h>
#include <Preferences.h>
#include <esp_task_wdt.h>

// software version 
#define SW_VERSION "0.20"  // Current firmware version identifier

// Forward declarations for compile-time safety mappings
void loadConfigPrefs();
void saveConfigPrefs();
void handleRoot();
void handleSave();
void handlePing();
void setRgbColor(uint8_t r, uint8_t g, uint8_t b);
void rgbOff();
void PrintUint64(uint64_t& value);
void printExtendedError(uint8_t port, const char* functionName, int16_t errorCode);
int16_t triggerRawSingleShot(uint8_t port);
bool connectWiFi();
void setupOTA();
void initSensor(uint8_t port, bool firstBoot);
void measureBattery();
void uploadData(uint16_t* co2, float* temp, float* hum, bool* sensorValid);

// SET TO true TO OVERWRITE ALL STORED CONFIG VALUES (WIFI, SENSORS, INTERVALS) WITH THE DEFAULTS BELOW ONCE.
// MUST BE SET BACK TO false AFTER THE FIRST UPLOAD, OTHERWISE PORTAL UPDATES WILL BE OVERWRITTEN ON EVERY COLD BOOT!
bool forceUpdateDefaults = false; 

// ==================== SECTION 1: TEXT-BUFFERS (PRIMARY CONFIGURATION) ====================
// Network Connection Settings
char wifiSSID[32]         = "your-wifi-ssid";
char wifiPass[64]         = "your-wifi-password";   
char serverURL[256]       = "http://example.org";


// Core Sensor and Timing Allocations
char sensorCountStr[4]    = "3";      // Active connected sensors (1-8)
char sleepMinutesStr[6]   = "5";      // Deep Sleep interval duration
char sensorAltitudeStr[6] = "50";     // Height above sea level (NN) for live pressure correction

// System Diagnostic Flags
char serialDebugStr[2]    = "1";      // Hardware Serial Console logs (1=ON, 0=OFF)
char rgbDebugStr[2]       = "1";      // Onboard NeoPixel LED flashes (1=ON, 0=OFF)

// Device Identification and Security Access Keys
char otaHostname[32]      = "beehive-co2-sensor"; 
char sharedCrypto[32]     = "openhive";        // Access key for AP Hotspot & OTA wireless flashing

// ==================== SECTION 2: OPERATIONAL RUNTIME VARIABLES ====================
// These fields are dynamically overwritten on boot by loadConfigPrefs()! Changing them here has no effect.
uint32_t sleepSeconds      = 0;       
uint8_t activeSensors      = 0;       
uint16_t sensorAltitude    = 0;       
bool systemDebugEnabled    = false;   
bool rgbDebugEnabled       = false;   
unsigned long lastPortalActivity = 0; 

// Hardware pin definitions and static chip registry constraints
const uint8_t maxI2CDevices = 8;    // Limit boundary of physical TCA9548A hardware channels
const int batteryPin = 34;          // Onboard voltage divider connection pin mapping
#define SCD41_I2C_ADDR 0x62         
#define MULTIPLEXER_I2C_ADDR 0x70   
#define RGB_LED_PIN 5               
#define RGB_LED_COUNT 1             
#define USER_BUTTON_PIN 27          
#define NO_ERROR 0

// Maximum time window allocated for active user interaction (in seconds)
#define PORTAL_TIMEOUT_SECONDS 120

// Hardware driver object instances
DFRobot_I2C_Multiplexer I2CMulti(&Wire, MULTIPLEXER_I2C_ADDR);
SensirionI2cScd4x sensors[maxI2CDevices]; // Allocated safely matching upper hardware boundaries
Adafruit_NeoPixel rgbLed(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
WebServer server(80); 
Preferences prefs;    

static char errorMessage[128]; // Puffer allocation array for error string generation
static int16_t error;
float voltageBattery = 0.0;

// Non-volatile RTC data cache memory map registers to bypass full WiFi scans on wakeups
RTC_DATA_ATTR uint8_t cachedChannel = 0;
RTC_DATA_ATTR uint8_t cachedBSSID[6] = {0, 0, 0, 0, 0, 0}; 
RTC_DATA_ATTR bool cachedValid = false;

// Pull variables from non-volatile NVS cells or enforce code defaults upon initial run
void loadConfigPrefs() {
    prefs.begin("beehive", true); // Open flash namespace in read-only protection mode
    
    // Automatically intercept blank factory states to burn fallback default matrices
    if (!prefs.isKey("ssid")) {
        prefs.end(); 
        if (systemDebugEnabled) Serial.println("\n[Storage] Factory new board detected! Automatically burning code defaults...");
        saveConfigPrefs(); 
        prefs.begin("beehive", true); 
    }
    
    // Linearly copy strings directly into memory buffers safely preventing overflow issues
    if (prefs.isKey("ssid"))     strncpy(wifiSSID, prefs.getString("ssid").c_str(), sizeof(wifiSSID) - 1);
    if (prefs.isKey("pass"))     strncpy(wifiPass, prefs.getString("pass").c_str(), sizeof(wifiPass) - 1);
    if (prefs.isKey("server"))   strncpy(serverURL, prefs.getString("server").c_str(), sizeof(serverURL) - 1);
    if (prefs.isKey("minutes"))  strncpy(sleepMinutesStr, prefs.getString("minutes").c_str(), sizeof(sleepMinutesStr) - 1);
    if (prefs.isKey("altitude")) strncpy(sensorAltitudeStr, prefs.getString("altitude").c_str(), sizeof(sensorAltitudeStr) - 1);
    if (prefs.isKey("scount"))   strncpy(sensorCountStr, prefs.getString("scount").c_str(), sizeof(sensorCountStr) - 1);
    if (prefs.isKey("sdebug"))   strncpy(serialDebugStr, prefs.getString("sdebug").c_str(), sizeof(serialDebugStr) - 1);
    if (prefs.isKey("rgbdebug")) strncpy(rgbDebugStr, prefs.getString("rgbdebug").c_str(), sizeof(rgbDebugStr) - 1);
    if (prefs.isKey("hostname")) strncpy(otaHostname, prefs.getString("hostname").c_str(), sizeof(otaHostname) - 1);
    if (prefs.isKey("crypto"))   strncpy(sharedCrypto, prefs.getString("crypto").c_str(), sizeof(sharedCrypto) - 1);
    
    // Parse runtime tracking integers securely from text variables
    sleepSeconds       = atoi(sleepMinutesStr) * 60;
    if (sleepSeconds < 10) sleepSeconds = 300; 
    sensorAltitude     = atoi(sensorAltitudeStr);
    
    activeSensors      = atoi(sensorCountStr);
    if (activeSensors == 0 || activeSensors > maxI2CDevices) activeSensors = 3; 
    
    systemDebugEnabled = (atoi(serialDebugStr) == 1);
    rgbDebugEnabled    = (atoi(rgbDebugStr) == 1);
    
    prefs.end();
}

// Permanently burn parameters into internal Espressif NVS storage
void saveConfigPrefs() {
    prefs.begin("beehive", false); // Open namespace in read-write mode
    
    // FIXED: Now it takes your real variables from Block 1 instead of hardcoded placeholder text
    if (forceUpdateDefaults) {
        prefs.putString("ssid", wifiSSID);       // Takes your real WiFi name from Block 1
        prefs.putString("pass", wifiPass);       // Takes your real WiFi password from Block 1
        prefs.putString("server", serverURL);   // Takes your real Server URL from Block 1
        prefs.putString("minutes", sleepMinutesStr);
        prefs.putString("altitude", sensorAltitudeStr);
        prefs.putString("scount", sensorCountStr);
        prefs.putString("sdebug", serialDebugStr);
        prefs.putString("rgbdebug", rgbDebugStr);
        prefs.putString("hostname", otaHostname);
        prefs.putString("crypto", sharedCrypto);
    } else {
        // Standard smartphone save path from the web browser fields
        prefs.putString("ssid", wifiSSID);
        prefs.putString("pass", wifiPass);
        prefs.putString("server", serverURL);
        prefs.putString("minutes", sleepMinutesStr);
        prefs.putString("altitude", sensorAltitudeStr);
        prefs.putString("scount", sensorCountStr);
        prefs.putString("sdebug", serialDebugStr);
        prefs.putString("rgbdebug", rgbDebugStr);
        prefs.putString("hostname", otaHostname);
        prefs.putString("crypto", sharedCrypto);
    }
    
    prefs.end();
    
    // Force immediate RAM synchronization
    loadConfigPrefs();

    if(systemDebugEnabled) {
        Serial.println("[Storage] Configuration successfully burned into native NVS cells!");
    }
}

// Generate the fully native HTML configuration page layout for connected browsers
void handleRoot() {
    lastPortalActivity = millis(); 
    
    String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:sans-serif;background:#f4f4f9;padding:20px;color:#333;}";
    html += ".card{background:#fff;padding:20px;border-radius:8px;box-shadow:0 4px 6px rgba(0,0,0,0.1);max-width:400px;margin:0 auto;}";
    html += "h2{color:#e67e22;text-align:center;} input{width:100%;padding:10px;margin:8px 0;box-sizing:border-box;border:1px solid #ccc;border-radius:4px;}";
    html += "button{width:100%;padding:12px;background:#e67e22;color:#fff;border:none;border-radius:4px;font-size:16px;cursor:pointer;}";
    html += "button:hover{background:#d35400;}</style></head><body>";
    
    html += "<div class='card'><h2>⬡ Open Hive CO2 Monitoring System</h2>";
    html += "<form action='/save' method='POST'>";
    
    // 1. Network Connection Settings
    html += "<label>WiFi SSID Name:</label><input type='text' name='ssid' value='" + String(wifiSSID) + "' required>";
    html += "<label>WiFi Password:</label><input type='password' name='pass' id='wifi_password_field' value='" + String(wifiPass) + "'>";
    html += "<div style='text-align:right; margin:-4px 0 12px 0; font-size:14px; color:#555;'><input type='checkbox' style='width:auto; margin-right:6px;' onclick='var x=document.getElementById(\"wifi_password_field\"); if(this.checked){x.type=\"text\";}else{x.type=\"password\";}'>Show Password</div>";
    html += "<label>Server URL (Endpoint):</label><input type='text' name='server' value='" + String(serverURL) + "'>";
    
    // 2. Core Sensor and Timing Allocations
    html += "<label>Connected Sensors (1-8):</label><input type='number' name='scount' value='" + String(sensorCountStr) + "' min='1' max='8'>";
    html += "<label>Interval (Minutes):</label><input type='number' name='minutes' value='" + String(sleepMinutesStr) + "'>";
    html += "<label>Sensor Altitude (Meters):</label><input type='number' name='altitude' value='" + String(sensorAltitudeStr) + "'>";
    
    // 3. System Diagnostic Flags
    html += "<label>Serial Debug (1=ON, 0=OFF):</label><input type='number' name='sdebug' value='" + String(serialDebugStr) + "' min='0' max='1'>";
    html += "<label>RGB LED Debug (1=ON, 0=OFF):</label><input type='number' name='rgbdebug' value='" + String(rgbDebugStr) + "' min='0' max='1'>";
    
    // 4. Device Identification and Security Access Keys
    html += "<label>Device Name (WLAN SSID / Hostname):</label><input type='text' name='hostname' value='" + String(otaHostname) + "'>";
    html += "<label>Device Security Key (AP / OTA Password):</label><input type='text' name='crypto' value='" + String(sharedCrypto) + "'>";
    
    html += "<br><br><button type='submit'>Save Configuration</button></form>";
    
    // Integrated Footer inside the card container boundaries
    html += "<div style='text-align:center; margin-top:25px; padding-top:15px; border-top:1px solid #eee; font-size:13px; color:#888;'>";
    html += "⬡ Firmware Version v" + String(SW_VERSION) + "</div>";
    
    html += "</div>"; 
    html += "<script>setInterval(function(){fetch('/ping');}, 10000);</script>"; 
    html += "</body></html>";
    
    server.send(200, "text/html; charset=utf-8", html);
}

// Map parameters transmitted via HTTP POST securely into the string buffers
void handleSave() {
    if (server.hasArg("ssid"))     strncpy(wifiSSID, server.arg("ssid").c_str(), sizeof(wifiSSID) - 1);
    if (server.hasArg("pass"))     strncpy(wifiPass, server.arg("pass").c_str(), sizeof(wifiPass) - 1);
    if (server.hasArg("server"))   strncpy(serverURL, server.arg("server").c_str(), sizeof(serverURL) - 1);
    if (server.hasArg("minutes"))  strncpy(sleepMinutesStr, server.arg("minutes").c_str(), sizeof(sleepMinutesStr) - 1);
    if (server.hasArg("altitude")) strncpy(sensorAltitudeStr, server.arg("altitude").c_str(), sizeof(sensorAltitudeStr) - 1);
    if (server.hasArg("scount"))   strncpy(sensorCountStr, server.arg("scount").c_str(), sizeof(sensorCountStr) - 1);
    if (server.hasArg("sdebug"))   strncpy(serialDebugStr, server.arg("sdebug").c_str(), sizeof(serialDebugStr) - 1);
    if (server.hasArg("rgbdebug")) strncpy(rgbDebugStr, server.arg("rgbdebug").c_str(), sizeof(rgbDebugStr) - 1);
    if (server.hasArg("hostname")) strncpy(otaHostname, server.arg("hostname").c_str(), sizeof(otaHostname) - 1);
    if (server.hasArg("crypto"))   strncpy(sharedCrypto, server.arg("crypto").c_str(), sizeof(sharedCrypto) - 1);
    
    saveConfigPrefs(); 
    
    // BEAUTIFULLY FORMATTED RESPONSE MATCHING THE DESIGN OF PAGE 1
    String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:sans-serif;background:#f4f4f9;padding:20px;color:#333;}";
    html += ".card{background:#fff;padding:30px;border-radius:8px;box-shadow:0 4px 6px rgba(0,0,0,0.1);max-width:400px;margin:40px auto;text-align:center;}";
    html += "h2{color:#e67e22;margin-bottom:15px;} p{font-size:16px;line-height:1.5;color:#555;}</style></head><body>";
    
    html += "<div class='card'><h2>⬡ Successfully Saved!</h2>";
    html += "<p>The sensor is now restarting to apply your changes...</p>";
    
    // CHANGED: Clean footer layout inside the box displaying only the firmware version
    html += "<div style='text-align:center; margin-top:25px; padding-top:15px; border-top:1px solid #eee; font-size:13px; color:#888;'>";
    html += "⬡ Firmware Version v" + String(SW_VERSION) + "</div>";
    
    html += "</div></body></html>"; 
    
    server.send(200, "text/html; charset=utf-8", html);
    delay(2000);
    ESP.restart(); 
}

// Invisible heartbeat listener route to lock countdown timer execution during configurations
void handlePing() {
    lastPortalActivity = millis(); 
    server.send(200, "text/plain", "pong");
}

// Drive colors into NeoPixel channel mapping applying protective dampening limits
void setRgbColor(uint8_t r, uint8_t g, uint8_t b) {
    if (rgbDebugEnabled) {
        rgbLed.setPixelColor(0, rgbLed.Color(r, g, b));
        rgbLed.show();
    }
}

// Extinguish NeoPixel array immediately to cut off microamps power drain
void rgbOff() {
    if (rgbDebugEnabled) {
        rgbLed.setPixelColor(0, rgbLed.Color(0, 0, 0));
        rgbLed.show();
    }
}

// Output unique 64-bit device signatures via hexadecimal console structures
void PrintUint64(uint64_t& value) {
    if(systemDebugEnabled) {
        Serial.print("0x");
        Serial.print((uint32_t)(value >> 32), HEX);
        Serial.print((uint32_t)(value & 0xFFFFFFFF), HEX);
    }
}

// Decrypt and process extended Sensirion register failure flags
void printExtendedError(uint8_t port, const char* functionName, int16_t errorCode) {
    if (systemDebugEnabled && errorCode != NO_ERROR) {
        errorToString(errorCode, errorMessage, sizeof(errorMessage));
        Serial.print("[error] port ");
        Serial.print(port);
        Serial.print(" - ");
        Serial.print(functionName);
        Serial.print(" failed: ");
        Serial.println(errorMessage);
    }
}

// Dispatch hardware command triggering single shot acquisition loops asynchronously
int16_t triggerRawSingleShot(uint8_t port) {
    I2CMulti.selectPort(port);
    Wire.beginTransmission(SCD41_I2C_ADDR);
    Wire.write(0x21); 
    Wire.write(0x9D);
    return Wire.endTransmission();
}

// Authenticate wireless link sessions utilizing hardware network channel caching
bool connectWiFi() {
    // Clean the background radio driver state to prevent "cannot set config" collision bugs
    WiFi.disconnect(false); 
    delay(100);

    if (strlen(wifiSSID) == 0) {
        if(systemDebugEnabled) Serial.println("WiFi connect skipped: No SSID saved yet.");
        return false;
    }

    // FIXED: Only execute the radical hostname injection if no valid channel cache exists yet!
    if (!cachedValid) {
        if(systemDebugEnabled) Serial.println("No valid cache found. Executing deep hostname registration...");
        WiFi.disconnect(true, true); 
        WiFi.mode(WIFI_MODE_NULL);
        delay(150);
        
        WiFi.setHostname(otaHostname); 
        WiFi.mode(WIFI_STA);
        
        if(systemDebugEnabled) Serial.print("connecting to wifi using standard autoconnect ");
        WiFi.begin(wifiSSID, wifiPass);
    } else {
        // MAXIMUM PERFORMANCE PATH: Keep the hardware register cache alive!
        WiFi.mode(WIFI_STA);
        WiFi.setHostname(otaHostname); // Keep registry in sync gently
        
        if(systemDebugEnabled) Serial.print("connecting to wifi using cached channel/bssid ");
        WiFi.begin(wifiSSID, wifiPass, cachedChannel, cachedBSSID);
    }
    
    unsigned long startAttemptTime = millis();
    const unsigned long wifiTimeout = 15000; 
    unsigned long lastBlinkTime = 0;
    bool toggleBlink = false;
    
    while (WiFi.status() != WL_CONNECTED && (millis() - startAttemptTime < wifiTimeout)) {
        delay(100); 
        if (millis() - lastBlinkTime >= 500) {
            lastBlinkTime = millis();
            toggleBlink = !toggleBlink;
            if (toggleBlink) setRgbColor(0, 0, 50); 
            else rgbOff(); 
        }
        if(systemDebugEnabled && (millis() - startAttemptTime) % 1000 < 100) Serial.print(" .");
    }
    
    if(WiFi.status() == WL_CONNECTED) {
        if(systemDebugEnabled) {
            Serial.print("\nwifi connected! IP: ");
            Serial.println(WiFi.localIP());
        }
        setRgbColor(0, 0, 50); 
        cachedChannel = WiFi.channel();
        memcpy(cachedBSSID, WiFi.BSSID(), 6);
        cachedValid = true;
        return true;
    } else {
        if(systemDebugEnabled) Serial.println(" wifi connection timeout");
        rgbOff();
        cachedValid = false; 
        return false;
    }
}

// Initialize native background firmware flashing framework
void setupOTA() {
    ArduinoOTA.setHostname(otaHostname);
    ArduinoOTA.setPassword(sharedCrypto);
    ArduinoOTA.onStart([]() { if(systemDebugEnabled) Serial.println("start ota updating..."); });
    ArduinoOTA.onEnd([]() { if(systemDebugEnabled) Serial.println("\nota end success"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) { if(systemDebugEnabled) Serial.printf("progress: %u%%\r", (progress / (total / 100))); });
    ArduinoOTA.onError([](ota_error_t error) { if(systemDebugEnabled) Serial.printf("error[%u]\n", error); });
    ArduinoOTA.begin();
}

// Wake up and transmit environment tuning tables to individual sensors
void initSensor(uint8_t port, bool firstBoot) {
    I2CMulti.selectPort(port);
    delay(5);
    sensors[port].begin(Wire, SCD41_I2C_ADDR);
    delay(10);
    error = sensors[port].wakeUp();
    printExtendedError(port, "wakeUp()", error);
    delay(25);

    if(firstBoot) {
        error = sensors[port].stopPeriodicMeasurement();
        printExtendedError(port, "stopPeriodicMeasurement()", error);
        error = sensors[port].reinit();
        printExtendedError(port, "reinit()", error);
        error = sensors[port].setAutomaticSelfCalibrationEnabled(false);
        printExtendedError(port, "setAutomaticSelfCalibrationEnabled(false)", error);
        error = sensors[port].setSensorAltitude(sensorAltitude);
        printExtendedError(port, "setSensorAltitude()", error);
        error = sensors[port].persistSettings();
        printExtendedError(port, "persistSettings()", error);
    }
}

// Capture current cell voltage potentials via analog mapping channels
void measureBattery() {
    int rawValue = analogRead(batteryPin);        
    voltageBattery = (rawValue * 2.0) / 1177.0;  
    if(systemDebugEnabled) {
        Serial.print("battery voltage: ");
        Serial.print(voltageBattery);
        Serial.println(" v");
    }
}

// Format dataset payload structure and dispatch via HTTP POST routines
void uploadData(uint16_t* co2, float* temp, float* hum, bool* sensorValid) {
    HTTPClient http;
    http.begin(serverURL);
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    String postData = "";
    bool firstParam = true;

    for(uint8_t i = 0; i < activeSensors; i++) {
        // Only append parameters if the sensor is physically present and valid!
        if (sensorValid[i]) {
            if (!firstParam) postData += "&";
            postData += "co2-" + String(i+1) + "=" + String(co2[i]);
            postData += "&temp-" + String(i+1) + "=" + String(temp[i], 2);
            postData += "&hum-" + String(i+1) + "=" + String(hum[i], 2);
            firstParam = false;
        }
    }
    
    // Kept identically: Battery gets appended here at the end of the query string
    if (!firstParam) postData += "&";
    postData += "battery=" + String(voltageBattery, 2);

    int httpResponseCode = http.POST(postData);
    if (httpResponseCode >= 200 && httpResponseCode < 300) {
        if (systemDebugEnabled) {
            Serial.print("[HTTP] Payload uploaded successfully! Response code: ");
            Serial.println(httpResponseCode);
        }
        setRgbColor(0, 50, 0); 
        delay(2000);
    } else {
        if (systemDebugEnabled) {
            Serial.print("[HTTP] Critical upload failure! Response code: ");
            Serial.println(httpResponseCode);
        }
        setRgbColor(50, 0, 0); 
        delay(3000);
    }
    rgbOff();
    http.end();
}

void setup() {
    Serial.begin(115200);
    delay(1000); // Allow hardware power levels to stabilize completely

    Wire.begin();

    pinMode(2, OUTPUT); // FireBeetle internal onboard diagnostic green LED
    digitalWrite(2, LOW);

    rgbLed.begin();
    rgbLed.show(); // Clear and initialize NeoPixel registry

    // Force burn code variables into NVS *BEFORE* loading old values from flash!
    if (forceUpdateDefaults) {
        if (systemDebugEnabled) {
            Serial.println("\n[Storage] FORCING code defaults into NVS memory cells...");
        }
        saveConfigPrefs(); // Takes your fresh variables from Block 1 and burns them into flash
    }

    // Now safely load (either the fresh burned ones or the old ones if force was false)
    loadConfigPrefs(); 

    // Determine sleep state context to evaluate if warm configuration is needed
    esp_sleep_wakeup_cause_t wakeupReason = esp_sleep_get_wakeup_cause();
    bool firstBoot = !(wakeupReason == ESP_SLEEP_WAKEUP_TIMER);

    if(systemDebugEnabled) {
        Serial.println("\n--- start ---");
        Serial.print("Firmware Version: v");
        Serial.println(SW_VERSION); 
        Serial.println(firstBoot ? "cold start - initial setup" : "wakeup from deep sleep");
    }

    // Interactive user gateway - only active on true manual power-on resets
    if (firstBoot) {
        pinMode(USER_BUTTON_PIN, INPUT_PULLUP);
        
        // Try local router first to keep device visible inside domestic network
        bool connectedLocal = connectWiFi();
        
        // Fallback option if off-grid or router is missing -> host standalone AP
        if (!connectedLocal) {
            if(systemDebugEnabled) Serial.println("No local network found. Launching local fallback Access Point...");
            WiFi.softAP(otaHostname, sharedCrypto);
            if(systemDebugEnabled) {
                Serial.print("Local AP hosted! SSID: "); Serial.print(otaHostname);
                Serial.print(" IP: "); Serial.println(WiFi.softAPIP());
            }
        }
        
        // Bind URL routes to background server engine and ignite framework tasks
        server.on("/", handleRoot);
        server.on("/save", HTTP_POST, handleSave);
        server.on("/ping", handlePing); 
        server.begin();
        setupOTA();

        // Print interactive user warning banner to serial monitor
        if(systemDebugEnabled) {
            int portalMinutes = PORTAL_TIMEOUT_SECONDS / 60; 

            Serial.println("-------------------------------------------------------------------------");
            Serial.print("OTA wireless upload or web configuration via IP are active for ");
            Serial.print(portalMinutes);
            Serial.println(" minutes");
            Serial.println("Press USER button on Firebeetle or input and send 'c' in serial");
            Serial.println("to skip configuration and start measurement");
            Serial.println("-------------------------------------------------------------------------");
        }

        unsigned long portalStart = millis(); 
        lastPortalActivity = millis(); // Arm interactive inactivity anchor       

        // Dynamic loop monitoring browser requests and wireless flash update streams
        while (millis() - lastPortalActivity < ((unsigned long)PORTAL_TIMEOUT_SECONDS * 1000)) {
            yield(); 
            vTaskDelay(1); // Yield context execution slots to background network stacks
            
            server.handleClient(); // Pump native web engine ticks
            ArduinoOTA.handle();   // Evaluate incoming network firmware data streams

            // Skip trigger via Serial terminal input
            if (Serial.available() > 0) {
                char inputChar = Serial.read();
                if (inputChar == 'c' || inputChar == 'C') {
                    if (systemDebugEnabled) {
                        Serial.println("[Config Mode] Skip triggered via Serial command!");
                    }
                    break; 
                }
            }
            // Skip trigger via hardware USER button
            if (digitalRead(USER_BUTTON_PIN) == LOW) {
                delay(50); // Hard debounce check
                if (digitalRead(USER_BUTTON_PIN) == LOW) {
                    if (systemDebugEnabled) {
                        Serial.println("[Config Mode] Skip triggered via physical USER button!");
                    }
                    break; 
                }
            }

            // Rapid violet oscillation sequence indicating system setup window is active
            if ((millis() - lastPortalActivity) % 300 < 150) {
                digitalWrite(2, HIGH);
                setRgbColor(35, 0, 50); 
            } else {
                digitalWrite(2, LOW);
                rgbOff();
            }
        }
        
        // Clean and terminate network infrastructure before moving to sensor execution
        WiFi.softAPdisconnect(true);
        server.stop();
        digitalWrite(2, LOW);
        rgbOff();
    }

    if (systemDebugEnabled) {
        Serial.println(">>> Starting measurement array now...\n");
    }

    // Warm amber orange glow indicating sensor heater matrices are ramping up
    setRgbColor(60, 20, 0); 

    // Allocate stack memory space arrays based on maximum physical limits
    uint16_t co2[maxI2CDevices] = {0};
    float temperature[maxI2CDevices] = {0.0};
    float humidity[maxI2CDevices] = {0.0};

    // Array definition directly above the measurement loops
    bool sensorValid[maxI2CDevices] = {false};

    // Cycle multiplexer channels to wake up active sensors
    for (uint8_t port = 0; port < activeSensors; port++) {
        initSensor(port, firstBoot);
    }

    // Split port index processing into separate calculation steps to prevent peak milliamp loads
    uint8_t totalGroups = (activeSensors > 4) ? 2 : 1;
    uint8_t splitIndex = (totalGroups == 2) ? 4 : activeSensors;

    for (uint8_t g = 0; g < totalGroups; g++) {
        uint8_t startPort = (g == 0) ? 0 : splitIndex;
        uint8_t endPort = (g == 0) ? splitIndex : activeSensors;
        if (startPort == endPort) continue;

        // Perform double raw single-shot executions to flush internal sensor logic buffers
        for (uint8_t port = startPort; port < endPort; port++) triggerRawSingleShot(port);
        delay(5800); 
        for (uint8_t port = startPort; port < endPort; port++) triggerRawSingleShot(port);
        delay(5800); 

        // Extract physical climate values from targeted sensor registers and power down hardware
        for (uint8_t port = startPort; port < endPort; port++) {
            I2CMulti.selectPort(port);
            delay(5); 
            
            // Capture the error code of the physical read command
            int16_t readError = sensors[port].readMeasurement(co2[port], temperature[port], humidity[port]);
            
            // Check if reading was successful and sensor returned real bytes
            if (readError == NO_ERROR && co2[port] > 0) {
                sensorValid[port] = true; // Sensor exists and is healthy!
            } else {
                sensorValid[port] = false; // dead port or unplugged sensor
            }
            
            sensors[port].powerDown(); // Deep freeze sensor to consume zero microamps
        }
    }
    rgbOff();

    // Print beautifully structured climate results to the serial terminal
    if (systemDebugEnabled) {
        for (uint8_t port = 0; port < activeSensors; port++) {
            // Only print real measurements or a clear skip message in the serial console
            if (sensorValid[port]) {
                Serial.print("[Sensor-");
                Serial.print(port + 1); 
                Serial.print("/Port-");
                Serial.print(port);
                Serial.print("] CO2: ");
                Serial.print(co2[port]);
                Serial.print(" ppm | Temp: ");
                Serial.print(temperature[port], 2);
                Serial.print(" °C | Hum: ");
                Serial.print(humidity[port], 2);
                Serial.println(" %RH");
            } else {
                Serial.print("[Sensor-");
                Serial.print(port + 1);
                Serial.print("/Port-");
                Serial.print(port);
                Serial.println("] UNPLUGGED or ERROR -> Skipping payload entry.");
            }
        }
        Serial.println(); 
    }

    // Collect internal analog battery supply values
    measureBattery();

    // Establish transmission bridge to transmit dataset to the cloud backend
    if (WiFi.status() != WL_CONNECTED) connectWiFi();
    else setRgbColor(0, 0, 50); // Steady blue indicating wireless sync window is open
    
    // Dispatch encoded data block or gracefully disconnect radio core upon timeout
    if (WiFi.status() == WL_CONNECTED) {
        // Pass sensorValid array to the upload function!
        uploadData(co2, temperature, humidity, sensorValid);
        WiFi.disconnect(true);
    } else {
        setRgbColor(50, 0, 0); // Red pulse indicating fatal link layer failure
        delay(3000);
        rgbOff();
        WiFi.disconnect(true);
    }

    // ==================== MAXIMUM POWER SAVING INTERFACES ====================
    if (systemDebugEnabled) {
        Serial.println("Going to deep sleep for " + String(sleepSeconds) + " seconds");
        Serial.flush();
    }

    // 1. DFRobot Lib Directive: Safely disconnect all downstream sensor channels
    I2CMulti.selectPort(8); 
    delay(20);

    // 2. Shut down the core ESP32 I2C peripheral hardware engine
    Wire.end(); 
    delay(20);

    // 3. Force physical hardware pins to clean, high-impedance inputs
    // This stops the multiplexer's 10k pull-up resistors from leaking current
    pinMode(21, INPUT); // SDA Pin
    pinMode(22, INPUT); // SCL Pin
    pinMode(RGB_LED_PIN, INPUT); // Solves the cut-pad leakage path completely!

    // 4. Standard 2.x Wireless Shutdown Sequence
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(50);

    // Engage ultra-low power deep sleep state to preserve battery inside the hive
    esp_sleep_enable_timer_wakeup((uint64_t)sleepSeconds * 1000000ULL);
    esp_deep_sleep_start();
}

void loop() {
    // Execution context blocked safely by hardware deep sleep mechanics
}

