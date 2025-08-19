#include <Wire.h>
#include "SparkFun_BMP581_Arduino_Library.h"

// Actuator definitions and constants
#define start_byte1 0x55
#define start_byte2 0xAA
#define response_byte1 0xAA
#define response_byte2 0x55
#define actuator1_ID 0x01
#define actuator2_ID 0x02
#define inst_type 0x32

#define ACTUATOR_1 Serial6
#define ACTUATOR_2 Serial1

// TCA9548A multiplexer address and sensor I2C address
#define TCA9548A_ADDR 0x70  // Default I2C address for the TCA9548A
#define i2cAddress    0x47  // I2C address for the BMP581 sensor

// Create sensor objects for each bus
//BMP581 pressureSensor1; // Sensor on Bus0 (Wire)
BMP581 pressureSensor2; // Sensor on Bus1 (Wire1)
BMP581 pressureSensor3; // Sensor on Bus2 (Wire2)

// // Separate tcaselect functions for each I2C bus:
// void tcaselectWire0(uint8_t channel) {
//   if (channel > 7) return;
//   Wire.beginTransmission(TCA9548A_ADDR);
//   Wire.write(1 << channel);
//   Wire.endTransmission();
// }

void tcaselectWire1(uint8_t channel) {
  if (channel > 7) return;
  Wire1.beginTransmission(TCA9548A_ADDR);
  Wire1.write(1 << channel);
  Wire1.endTransmission();
}

void tcaselectWire2(uint8_t channel) {
  if (channel > 7) return;
  Wire2.beginTransmission(TCA9548A_ADDR);
  Wire2.write(1 << channel);
  Wire2.endTransmission();
}

void setup() {
  Serial.begin(921600);
  while (!Serial);

  // Initialize I2C buses for the multiplexers
  //Wire.begin();   // Bus0 for sensor array 1
  Wire1.begin();  // Bus1 for sensor array 2
  Wire2.begin();  // Bus2 for sensor array 3

  // For sensor initialization, assume the sensor is connected on channel 0.
  // Select channel 0 before initializing each sensor:
  
  // // Bus0 sensor initialization:
  // tcaselectWire0(0);
  // int rst1 = pressureSensor1.beginI2C(i2cAddress);
  // while (rst1 != BMP5_OK) {
  //   Serial.println("Error: BMP581 not connected on Bus0. Retrying...");
  //   delay(1000);
  //   tcaselectWire0(0); // Ensure the sensor channel is selected
  //   rst1 = pressureSensor1.beginI2C(i2cAddress);
  // }
  // Serial.println("Sensor on Bus0 connected on channel 0.");

  // Bus1 sensor initialization:
  tcaselectWire1(0);
  int rst2 = pressureSensor2.beginI2C(i2cAddress, Wire1);
  while (rst2 != BMP5_OK) {
    Serial.println("Error: BMP581 not connected on Bus1. Retrying...");
    delay(1000);
    tcaselectWire1(0);
    rst2 = pressureSensor2.beginI2C(i2cAddress, Wire1);
  }
  Serial.println("Sensor on Bus1 connected on channel 0.");

  // Bus2 sensor initialization:
  tcaselectWire2(0);
  int rst3 = pressureSensor3.beginI2C(i2cAddress, Wire2);
  while (rst3 != BMP5_OK) {
    Serial.println("Error: BMP581 not connected on Bus2. Retrying...");
    delay(1000);
    tcaselectWire2(0);
    rst3 = pressureSensor3.beginI2C(i2cAddress, Wire2);
  }
  Serial.println("Sensor on Bus2 connected on channel 0.");

  // Initialize actuator serial ports
  ACTUATOR_1.begin(921600);
  while (!ACTUATOR_1);
  ACTUATOR_2.begin(921600);
  while (!ACTUATOR_2);

  // Fault clearance command for actuators
  uint8_t faultclr[] = {0x55, 0xAA, 0x05, 0xFF,
                        0x32, 0x18, 0x00, 0x01,
                        0x00, 0x4F};
  Serial.println("Performing fault clearance on actuators.");
  ACTUATOR_1.write(faultclr, 10);
  ACTUATOR_2.write(faultclr, 10);
  delay(100);
}

void loop() {
  // // Read sensor data from Bus0 (pressureSensor1)
  // String outputLine1 = "";
  // float total1 = 0.0;
  // for (uint8_t i = 0; i < 8; i++) {
  //   tcaselectWire0(i); // Select each channel on Bus0
  //   bmp5_sensor_data data = {0, 0};
  //   int8_t err = pressureSensor1.getSensorData(&data);
  //   if (err == BMP5_OK) {
  //     outputLine1 += String(data.pressure);
  //     total1 += data.pressure;
  //   } else {
  //     outputLine1 += "ERROR";
  //   }
  //   if (i < 7) outputLine1 += ",";
  //   delay(10);
  // }
  // float avg1 = total1 / 8.0;

  // Read sensor data from Bus1 (pressureSensor2)
  String outputLine2 = "";
  float total2 = 0.0;
  for (uint8_t i = 0; i < 8; i++) {
    tcaselectWire1(i); // Select each channel on Bus1
    bmp5_sensor_data data = {0, 0};
    int8_t err = pressureSensor2.getSensorData(&data);
    if (err == BMP5_OK) {
      outputLine2 += String(data.pressure);
      total2 += data.pressure;
    } else {
      outputLine2 += "ERROR";
    }
    if (i < 7) outputLine2 += ",";
    delay(10);
  }
  float avg2 = total2 / 8.0;

  // Read sensor data from Bus2 (pressureSensor3)
  String outputLine3 = "";
  float total3 = 0.0;
  for (uint8_t i = 0; i < 8; i++) {
    tcaselectWire2(i); // Select each channel on Bus2
    bmp5_sensor_data data = {0, 0};
    int8_t err = pressureSensor3.getSensorData(&data);
    if (err == BMP5_OK) {
      outputLine3 += String(data.pressure);
      total3 += data.pressure;
    } else {
      outputLine3 += "ERROR";
    }
    if (i < 7) outputLine3 += ",";
    delay(10);
  }
  float avg3 = total3 / 8.0;

  // Print out sensor readings and averages
  // Serial.println("Sensor Array on Bus0: " + outputLine1);
  // Serial.println("Average Bus0: " + String(avg1));
  Serial.println("Sensor Array on Bus1: " + outputLine2);
  Serial.println("Average Bus1: " + String(avg2));
  Serial.println("Sensor Array on Bus2: " + outputLine3);
  Serial.println("Average Bus2: " + String(avg3));

  // Example actuator control logic:
  // If any sensor array’s average pressure exceeds 98000.0, send a force command; otherwise, retract.
  if (avg2 > 98000.0 || avg3 > 98000.0) {
    uint8_t forcemode[] = {0x55, 0xAA, 0x05, 0xFF,
                           0x32, 0x25, 0x00, 0x00,
                           0x00, 0x5B};
    ACTUATOR_1.write(forcemode, 10);
    ACTUATOR_2.write(forcemode, 10);
    delay(100);
    uint8_t force[] = {0x55, 0xAA, 0x05, 0xFF,
                       0x32, 0x29, 0x00, 0xE8,
                       0x03, 0x4A};
    delay(100);
    ACTUATOR_1.write(force, 10);
    ACTUATOR_2.write(force, 10);
  } else {
    uint8_t posmode[] = {0x55, 0xAA, 0x05, 0xFF,
                         0x32, 0x25, 0x00, 0x00,
                         0x00, 0x5B};
    ACTUATOR_1.write(posmode, 10);
    ACTUATOR_2.write(posmode, 10);
    delay(100);
    uint8_t fullretract[] = {0x55, 0xAA, 0x05, 0xFF,
                             0x32, 0x29, 0x00, 0x00,
                             0x00, 0x5F};
    ACTUATOR_1.write(fullretract, 10);
    ACTUATOR_2.write(fullretract, 10);
  }
  delay(100); // Wait one second before the next loop iteration
}
