#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_I2C_Multiplexer.h>
#include <SensirionI2cScd4x.h>

// configuration
// -------------
const uint8_t numI2CDevices = 3;    // flexible: change to any number up to 8
const float tempOffsets[numI2CDevices] = {0.0, 0.0, 0.0}; // setting all offsets to 0.0 clears the factory 4°c offset for single shot mode
const uint16_t sensorAltitude = 50;  // set your height above sea level in meters (e.g., 50m for Berlin/Neuenhagen)

// hardware settings
// -----------------
#define SCD41_I2C_ADDR 0x62
#define MULTIPLEXER_I2C_ADDR 0x70

// objects & global variables
// --------------------------
DFRobot_I2C_Multiplexer I2CMulti(&Wire, MULTIPLEXER_I2C_ADDR);
SensirionI2cScd4x sensors[numI2CDevices];

void setup() {
    Serial.begin(115200);
    delay(1000);
    Wire.begin();
    pinMode(2, OUTPUT); // internal esp32 firebeetle led for status

    Serial.println("\n=== scd41 grouped forced calibration tool (co2 + temp + altitude) ===");
    Serial.println("make sure sensors are in fresh outdoor air for co2 calibration.");
    
    // dynamic group logic (up to 4 sensors per group are safe)
    uint8_t totalGroups = (numI2CDevices > 4) ? 2 : 1;
    uint8_t splitIndex = (totalGroups == 2) ? 4 : numI2CDevices;

    for (uint8_t g = 0; g < totalGroups; g++) {
        uint8_t startPort = (g == 0) ? 0 : splitIndex;
        uint8_t endPort = (g == 0) ? splitIndex : numI2CDevices;

        if (startPort == endPort) continue;

        Serial.println("\n----------------------------------------");
        Serial.print("starting calibration for group "); Serial.println(g + 1);
        Serial.print("ports: "); Serial.print(startPort); Serial.print(" to "); Serial.println(endPort - 1);
        Serial.println("----------------------------------------");

        // 1. prepare sensors, set temperature offsets to 0.0 and configure altitude
        for (uint8_t port = startPort; port < endPort; port++) {
            I2CMulti.selectPort(port);
            sensors[port].begin(Wire, SCD41_I2C_ADDR);
            
            sensors[port].wakeUp();
            delay(25); // hardware wakeup stabilization
            
            sensors[port].stopPeriodicMeasurement();
            sensors[port].reinit();
            
            // write 0.0 to disable the factory continuous-mode temperature subtraction
            int16_t tErr = sensors[port].setTemperatureOffset(tempOffsets[port]);
            if (tErr == 0) {
                Serial.print("port "); Serial.print(port); 
                Serial.print(": temperature offset cleared to "); Serial.print(tempOffsets[port]); Serial.println(" °c");
            } else {
                Serial.print("port "); Serial.print(port); Serial.println(": failed to clear temperature offset!");
            }

            // set the sensor altitude to compensate for air density changes
            int16_t aErr = sensors[port].setSensorAltitude(sensorAltitude);
            if (aErr == 0) {
                Serial.print("port "); Serial.print(port);
                Serial.print(": sensor altitude set to "); Serial.print(sensorAltitude); Serial.println(" meters");
            } else {
                Serial.print("port "); Serial.print(port); Serial.println(": failed to set sensor altitude!");
            }
            
            sensors[port].startPeriodicMeasurement();
        }

        // 2. mandatory 3 minute warm up phase for co2 baseline
        Serial.println("warming up this group in periodic mode for 3 minutes...");
        for (int i = 180; i > 0; i--) {
            if (i % 10 == 0) {
                Serial.print(i); Serial.println(" seconds remaining for this group...");
            }
            digitalWrite(2, !digitalRead(2));
            delay(1000);
        }

        // 3. execute co2 calibration and save everything permanently
        for (uint8_t port = startPort; port < endPort; port++) {
            I2CMulti.selectPort(port);
            
            sensors[port].stopPeriodicMeasurement();
            delay(500);
            
            uint16_t frcCorrection = 0;
            // calibrate to 400 ppm (outdoor baseline)
            int16_t error = sensors[port].performForcedRecalibration(400, frcCorrection);
            
            if (error == 0) {
                Serial.print("port "); Serial.print(port);
                Serial.print(" co2 success! correction offset: "); Serial.println((int16_t)frcCorrection);
            } else {
                Serial.print("port "); Serial.print(port); Serial.println(" failed co2 calibration!");
            }

            // disable asc permanently for beehive environment
            sensors[port].setAutomaticSelfCalibrationEnabled(false);
            
            // persist settings (saves 0.0 offset, altitude and co2 calibration to eeprom)
            sensors[port].persistSettings();
            
            // MODIFIED: do not power down, instead restart periodic measurement for live logging
            sensors[port].startPeriodicMeasurement();
        }
    }

    Serial.println("\n=== calibration complete! starting continuous live log ===");
    Serial.println("format: [port] co2(ppm) | temp(°c) | humid(%rh)");
    Serial.println("------------------------------------------------------------------");
}

void loop() {
    uint16_t co2[numI2CDevices] = {0};
    float temperature[numI2CDevices] = {0.0};
    float humidity[numI2CDevices] = {0.0};

    // read current continuous data from all sensors
    for (uint8_t port = 0; port < numI2CDevices; port++) {
        I2CMulti.selectPort(port);
        // read measurement (does not block because sensors run in periodic mode)
        sensors[port].readMeasurement(co2[port], temperature[port], humidity[port]);
    }

    // print values neatly side by side in the serial monitor
    Serial.print("data: ");
    for (uint8_t port = 0; port < numI2CDevices; port++) {
        Serial.print("[p"); Serial.print(port); Serial.print("] ");
        Serial.print(co2[port]); Serial.print("ppm | ");
        Serial.print(temperature[port], 2); Serial.print("°c | ");
        Serial.print(humidity[port], 2); Serial.print("%");
        
        if (port < numI2CDevices - 1) {
            Serial.print("  ||  "); // divider between sensors
        }
    }
    Serial.println();

    digitalWrite(2, HIGH);
    delay(100);
    digitalWrite(2, LOW); // quick flash indicating a read cycle

    delay(9900); // repeat every 10 seconds (9900ms + 100ms flash = 10s)
}
