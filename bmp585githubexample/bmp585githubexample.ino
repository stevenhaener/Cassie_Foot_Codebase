#include <Wire.h>
#include "SparkFun_BMP581_Arduino_Library.h"

#define TCA9548A_ADDR 0x70  // Default I2C address of the TCA9548A
#define NUM_CHANNELS 8      // Number of multiplexer channels

// Create a new sensor object
BMP581 pressureSensor;

// I2C address selection
uint8_t i2cAddress = BMP581_I2C_ADDRESS_DEFAULT; // 0x47
// uint8_t i2cAddress = BMP581_I2C_ADDRESS_SECONDARY; // 0x46

// Function to select the desired channel on the multiplexer
void tcaselect(uint8_t channel) {
    if (channel > 7) return;  // Ensure channel is valid (0-7)
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << channel);  // Select the channel by writing a bitmask
    Wire.endTransmission();
}

void setup() {
    // Start serial
    Serial.begin(115200);
    while (!Serial);
    Serial.println("BMP581 Multiplexer Example begin!");

    // Initialize the I2C library
    Wire.begin();

    // Check each multiplexer channel for BMP581
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        tcaselect(i);
        delay(50);  // Allow stabilization

        if (pressureSensor.beginI2C(i2cAddress) == BMP5_OK) {
            Serial.print("BMP581 detected on channel: ");
            Serial.println(i);
        } else {
            Serial.print("No BMP581 detected on channel: ");
            Serial.println(i);
        }
    }
}

void loop() {
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        tcaselect(i);
        delay(50);  // Allow stabilization after switching channels

        bmp5_sensor_data data = {0, 0};
        int8_t err = pressureSensor.getSensorData(&data);

        if (err == BMP5_OK) {
            Serial.print("Channel ");
            Serial.print(i);
            Serial.print(" -> Temperature (C): ");
            Serial.print(data.temperature);
            Serial.print("\tPressure (Pa): ");
            Serial.println(data.pressure);
        } else {
            Serial.print("Error getting data from channel ");
            Serial.print(i);
            Serial.print(". Error code: ");
            Serial.println(err);
        }

        delay(1000);  // Delay between channel readings
    }
}
