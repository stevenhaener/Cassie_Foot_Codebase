#include <Wire.h>
#include "SparkFun_BMP581_Arduino_Library.h"

#define TCA9548A_ADDR 0x70  // Default I2C address of the TCA9548A
// I2C address selection
#define i2cAddress 0x47

// Create a new sensor object
BMP581 pressureSensor;

// Function to select the desired channel on the multiplexer
void tcaselect(uint8_t channel) {
  if (channel > 7) return;  // Ensure channel is valid (0-7)
  Wire1.beginTransmission(TCA9548A_ADDR);
  Wire1.write(1 << channel);  // Select the channel by writing a bitmask
  Wire1.endTransmission();
}

void setup()
{
    // Start serial
    Serial.begin(115200);
    while (!Serial);
    Serial.println("BMP581 Example1 begin!");

    // Initialize the I2C library
    Wire1.begin();

  for (uint8_t i = 0; i < 8; i++) {
    tcaselect(i);  // Select channel on the multiplexer
    Serial.print("Selected "); Serial.print(i); Serial.println("-th sensor.");  
    // Check if sensor is connected and initialize
    delay(10);
    if (Wire1.requestFrom(0x47, 1)) {
      Serial.println("Device detected on channel 0.");
    } else {
      Serial.println("No device detected on channel 0.");
    }
    // Address is optional (defaults to 0x47)
    int rst = pressureSensor.beginI2C(i2cAddress, Wire1);
    while (rst != BMP5_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMP581 not connected, try again after 1 sec!");
        Serial.printf("The error code is %d.\n", rst);
        delay(1000);
    }
    Serial.println("Successfully connected.");
  }
}

void loop()
{
  String outputLine = ""; // Create a string to store the line of data

  for (uint8_t i = 0; i < 8; i++) {
    tcaselect(i);  // Select channel on the multiplexer

    // Get measurements from the sensor
    bmp5_sensor_data data = {0, 0};
    int8_t err = pressureSensor.getSensorData(&data);

    // Check whether data was acquired successfully
    if (err == BMP5_OK) {
      // Append the pressure data to the output string
      outputLine += String(data.pressure);
    } else {
      // Append an error placeholder if acquisition failed
      outputLine += "ERROR";
    }

    // Add a comma after every reading except the last one
    if (i < 7) {
      outputLine += ",";
    }
  }

  // Print the complete line of data
  Serial.println(outputLine);

  // Wait for a short time before the next iteration
  delay(100);
}
