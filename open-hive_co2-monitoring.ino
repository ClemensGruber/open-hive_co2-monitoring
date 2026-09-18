#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_I2C_Multiplexer.h>
#include <SensirionI2cScd4x.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_sleep.h>

// user configuration
// ------------------
#define SLEEP_SECONDS 5 * 60      // deep sleep interval in minutes * seconds
#define DEBUG true                // enable/disable serial debug output

// wifi credentials
const char* ssid     = "[your--ssid]";
const char* password = "[your-pw]";

// server endpoint
const char* serverURL = "http://example.org/api-notls/hiveeyes/testdrive/beekeeper-name/hive-number/data";

// hardware settings
const uint8_t numI2CDevices = 3;    // flexible: change to any number (max 8)
const int batteryPin = 34;          // analog pin for battery measurement
#define SCD41_I2C_ADDR 0x62         // default I2C address for Sensirion SCD41
#define MULTIPLEXER_I2C_ADDR 0x70   // default I2C address for TCA9548A I2C multiplexer

// system constants (do not modify)
// --------------------------------
#define NO_ERROR 0

// objects & global variables
// --------------------------
DFRobot_I2C_Multiplexer I2CMulti(&Wire, MULTIPLEXER_I2C_ADDR);
SensirionI2cScd4x sensors[numI2CDevices];

static char errorMessage[64]; // safe buffer size for human-readable error messages
static int16_t error;
float voltageBattery = 0.0;

// rtc memory variables to cache wifi connection data between deep sleep cycles
RTC_DATA_ATTR uint8_t cachedChannel = 0;
RTC_DATA_ATTR uint8_t cachedBSSID[6] = {0, 0, 0, 0, 0, 0};
RTC_DATA_ATTR bool cachedValid = false;

// helper function: print 64-bit serial number in hex format
void PrintUint64(uint64_t& value) {
    if(DEBUG) {
        Serial.print("0x");
        Serial.print((uint32_t)(value >> 32), HEX);
        Serial.print((uint32_t)(value & 0xFFFFFFFF), HEX);
    }
}

// helper function: print detailed error message
void printExtendedError(uint8_t port, const char* functionName, int16_t errorCode) {
    if (DEBUG && errorCode != NO_ERROR) {
        errorToString(errorCode, errorMessage, sizeof(errorMessage));
        Serial.print("[error] port ");
        Serial.print(port);
        Serial.print(" - ");
        Serial.print(functionName);
        Serial.print(" failed: ");
        Serial.println(errorMessage);
    }
}

// helper function: send raw i2c command to start single shot without blocking delay
int16_t triggerRawSingleShot(uint8_t port) {
    I2CMulti.selectPort(port);
    Wire.beginTransmission(SCD41_I2C_ADDR);
    Wire.write(0x21); // command: 0x219D (measure single shot)
    Wire.write(0x9D);
    return Wire.endTransmission();
}

// initialize sensor on a specific multiplexer port
void initSensor(uint8_t port, bool firstBoot) {
    I2CMulti.selectPort(port);
    delay(5);

    sensors[port].begin(Wire, SCD41_I2C_ADDR);
    delay(10);

    // wake up from deep sleep (required after every boot)
    error = sensors[port].wakeUp();
    printExtendedError(port, "wakeUp()", error);
    delay(25);    // give the sensor time to wake up fully before the next command

    if(firstBoot) {
        error = sensors[port].stopPeriodicMeasurement();
        printExtendedError(port, "stopPeriodicMeasurement()", error);

        error = sensors[port].reinit();
        printExtendedError(port, "reinit()", error);

        // disable automatic self-calibration for beehive environment
        error = sensors[port].setAutomaticSelfCalibrationEnabled(false);
        printExtendedError(port, "setAutomaticSelfCalibrationEnabled(false)", error);

        // permanently save settings to eeprom so powerDown doesn't wipe them
        error = sensors[port].persistSettings();
        printExtendedError(port, "persistSettings()", error);
        
        uint64_t serialNumber = 0;
        error = sensors[port].getSerialNumber(serialNumber);
        if (error == NO_ERROR) {
            if(DEBUG) {
                Serial.print("sensor port ");
                Serial.print(port);
                Serial.print(" serial: ");
                PrintUint64(serialNumber);
                Serial.println();
            }
        } else {
            printExtendedError(port, "getSerialNumber()", error);
        }
    }
}

bool connectWiFi() {
    if (cachedValid) {
        if(DEBUG) Serial.print("connecting to wifi using cached channel/bssid ");
        WiFi.begin(ssid, password, cachedChannel, cachedBSSID);
    } else {
        if(DEBUG) Serial.print("connecting to wifi using full scan ");
        WiFi.begin(ssid, password);
    }
    
    uint8_t attempts = 0;
    while(WiFi.status() != WL_CONNECTED && attempts < 40) { // max 20 seconds timeout
        delay(500);
        if(DEBUG) Serial.print(".");
        attempts++;
    }
    
    if(WiFi.status() == WL_CONNECTED) {
        if(DEBUG) {
            Serial.print(" wifi connected! ip: ");
            Serial.println(WiFi.localIP());
        }
        cachedChannel = WiFi.channel();
        memcpy(cachedBSSID, WiFi.BSSID(), 6);
        cachedValid = true;
        return true;
    } else {
        if(DEBUG) Serial.println(" wifi connection timeout");
        cachedValid = false; 
        return false;
    }
}

void measureBattery() {
    int rawValue = analogRead(batteryPin);        // adc value 0-4095 (esp32)
    voltageBattery = (rawValue * 2.0) / 1177.0;  // calculation according to formula
    if(DEBUG) {
        Serial.print("battery voltage: ");
        Serial.print(voltageBattery);
        Serial.println(" v");
    }
}

void uploadData(uint16_t* co2, float* temp, float* hum) {
    HTTPClient http;
    http.begin(serverURL);
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    String postData = "";
    for(uint8_t i=0; i<numI2CDevices; i++) {
        if(i > 0) postData += "&";
        postData += "co2-" + String(i+1) + "=" + String(co2[i]);
        postData += "&temp-" + String(i+1) + "=" + String(temp[i], 2);
        postData += "&hum-" + String(i+1) + "=" + String(hum[i], 2);
    }
    postData += "&battery=" + String(voltageBattery, 2);

    if(DEBUG) {
        Serial.println("post data: " + postData);
    }

    int httpResponseCode = http.POST(postData);
    if(DEBUG) {
        Serial.print("http response code: ");
        Serial.println(httpResponseCode);
        Serial.println();
    }

    http.end();
}

void setup() {
    if(DEBUG) Serial.begin(115200);
    delay(1000);

    Wire.begin();

    esp_sleep_wakeup_cause_t wakeupReason = esp_sleep_get_wakeup_cause();
    bool firstBoot = !(wakeupReason == ESP_SLEEP_WAKEUP_TIMER);

    if(DEBUG) {
        Serial.println("\n--- start ---");
        Serial.println(firstBoot ? "cold start - initial setup" : "wakeup from deep sleep");
    }

    uint16_t co2[numI2CDevices] = {0};
    float temperature[numI2CDevices] = {0.0};
    float humidity[numI2CDevices] = {0.0};

    for (uint8_t port = 0; port < numI2CDevices; port++) {
        initSensor(port, firstBoot);
    }

    // dynamic group logic to prevent max current overloads
    uint8_t totalGroups = (numI2CDevices > 4) ? 2 : 1;
    uint8_t splitIndex = (totalGroups == 2) ? 4 : numI2CDevices;

    for (uint8_t g = 0; g < totalGroups; g++) {
        uint8_t startPort = (g == 0) ? 0 : splitIndex;
        uint8_t endPort = (g == 0) ? splitIndex : numI2CDevices;

        if (startPort == endPort) continue;

        if(DEBUG) {
            Serial.print("starting parallel measurement group ");
            Serial.print(g + 1);
            Serial.print(" (ports "); Serial.print(startPort);
            Serial.print(" to "); Serial.print(endPort - 1);
            Serial.println(")...");
        }

        // measurement 1: trigger parallelly and wait 5.8s (discarded)
        for (uint8_t port = startPort; port < endPort; port++) {
            triggerRawSingleShot(port);
        }
        delay(5800); 

        // measurement 2: trigger parallelly and wait 5.8s (actual data)
        for (uint8_t port = startPort; port < endPort; port++) {
            triggerRawSingleShot(port);
        }
        delay(5800); 

        // read data and immediately power down this group
        for (uint8_t port = startPort; port < endPort; port++) {
            I2CMulti.selectPort(port);
            delay(5); 
            sensors[port].readMeasurement(co2[port], temperature[port], humidity[port]);
            sensors[port].powerDown();
        }
    }

    // phase C: print debug, battery & upload
    // --------------------------------------
    if(DEBUG) {
        for(uint8_t port = 0; port < numI2CDevices; port++) {
            Serial.print("sensor port "); Serial.println(port);
            Serial.print("  co2 [ppm]: "); Serial.println(co2[port]);
            Serial.print("  temp [°c]: "); Serial.println(temperature[port]);
            Serial.print("  hum [%rh]: "); Serial.println(humidity[port]);
        }
    }

    measureBattery();

    if(DEBUG) Serial.println("sensors are asleep. activating wifi for upload...");
    
    if (connectWiFi()) {
        uploadData(co2, temperature, humidity);
        WiFi.disconnect(true);
    } else {
        if(DEBUG) Serial.println("skipping upload due to missing connection. entering rescue sleep.");
        WiFi.disconnect(true);
    }

    if(DEBUG) {
        Serial.println("going to deep sleep for " + String(SLEEP_SECONDS) + " seconds");
        Serial.flush();
    }

    esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_SECONDS * 1000000ULL);
    esp_deep_sleep_start();
}

void loop() {
    // never reached
}
