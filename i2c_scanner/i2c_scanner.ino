#include <Wire.h>

// Create an array of pointers to the Wire instances we want to scan
TwoWire* const buses[] = { &Wire, &Wire1, &Wire2 };
const char* const busNames[] = { "Wire", "Wire1", "Wire2" };
const uint8_t numBuses = sizeof(buses) / sizeof(buses[0]);

void setup() {
    Serial.begin(115200);
    while (!Serial);

    // Initialize each I2C bus
    Wire.begin();       // default Wire (pins 18=SDA, 19=SCL on Teensy 4.1)
    Wire1.begin();      // Wire1 (pins 17=SDA1, 16=SCL1)
    Wire2.begin();      // Wire2 (pins 9=SDA2, 10=SCL2)

    Serial.println();
    Serial.println("I2C Multi-Scanner on Teensy 4.1");
    Serial.println("--------------------------------");
}

void loop() {
    for (uint8_t busIdx = 0; busIdx < numBuses; busIdx++) {
        TwoWire &wire = *buses[busIdx];
        Serial.print("Scanning I2C devices on ");
        Serial.print(busNames[busIdx]);
        Serial.println("...");

        for (uint8_t address = 0x03; address <= 0x77; address++) {
            wire.beginTransmission(address);
            if (wire.endTransmission() == 0) {
                Serial.print("  -> Found device at 0x");
                if (address < 16) Serial.print('0');
                Serial.println(address, HEX);
            }
        }

        Serial.println("Scan complete.\n");
        delay(200);  // short pause between buses
    }

    Serial.println("Waiting 2 seconds before next full scan...\n");
    delay(2000);
}
