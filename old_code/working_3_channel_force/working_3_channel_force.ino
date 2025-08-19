#include <Wire.h>
#include "SparkFun_BMP581_Arduino_Library.h"

// Actuator and protocol definitions
#define start_byte1 0x55
#define start_byte2 0xAA
#define response_byte1 0xAA
#define response_byte2 0x55
#define actuator1_ID 0x01
#define actuator2_ID 0x02
#define inst_type 0x32

#define ACTUATOR_1 Serial1
#define ACTUATOR_2 Serial1

// Multiplexer and sensor I2C addresses
#define TCA9548A_ADDR 0x70  // Default address for TCA9548A
#define TCA9548A_ADDR_2 0x71  // Default address for TCA9548A
#define i2cAddress    0x47  // I2C address for BMP585 sensor

// Create sensor arrays – one for each channel on each I²C bus
//BMP581 sensors0[8];  // Bus0 (Wire)
BMP581 sensors1[8];  // Bus1 (Wire1)
BMP581 sensors2[8];  // Bus2 (Wire2)

// Soft reset function for BMP585 (datasheet section 4.3.10)
// Resets the sensor by writing 0xB6 to the CMD register (address 0x7E)
void resetBMP585(TwoWire &wirePort, uint8_t address) {
  wirePort.beginTransmission(address);
  wirePort.write(0x7E);  // CMD register address
  wirePort.write(0xB6);  // Soft reset command
  wirePort.endTransmission();
  delay(2);  // Wait for t_soft_res (~2ms)
}

// Multiplexer channel select functions for each bus
void tcaselectWire0(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void tcaselectWire1(uint8_t channel) {
  if (channel > 7) return;
  Wire1.beginTransmission(TCA9548A_ADDR);
  Wire1.write(1 << channel);
  Wire1.endTransmission();
}


// Uncomment this function if you later use a third I²C bus (Wire2)
void tcaselectWire2(uint8_t channel) {
  if (channel > 7) return;
  Wire2.beginTransmission(TCA9548A_ADDR);
  Wire2.write(1 << channel);
  Wire2.endTransmission();
}


void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Initialize I²C buses
  Wire.begin();    // Bus0 (Wire)
  Wire1.begin();   // Bus1 (Wire1)
  Wire2.begin();   // Bus2 (Wire2) - commented out

  // Initialize actuator serial ports
  ACTUATOR_1.begin(921600);
  while (!ACTUATOR_1);
  ACTUATOR_2.begin(921600);
  while (!ACTUATOR_2);

  // Initialize sensors on Bus0 (Wire)
  for (uint8_t ch = 0; ch < 8; ch++) {
    tcaselectWire0(ch);
    resetBMP585(Wire, i2cAddress);
    delay(50); // Allow extra settling time after reset
    int status = sensors0[ch].beginI2C(i2cAddress, Wire);
    while (status != BMP5_OK) {
      Serial.print("Error: BMP585 not connected on Bus0 channel ");
      Serial.print(ch);
      Serial.println(". Retrying...");
      delay(1000);
      tcaselectWire0(ch);
      status = sensors0[ch].beginI2C(i2cAddress, Wire);
    }
    Serial.print("Sensor on Bus0 connected on channel ");
    Serial.println(ch);
  }

  // Initialize sensors on Bus1 (Wire1)
  for (uint8_t ch = 0; ch < 8; ch++) {
    tcaselectWire1(ch);
    resetBMP585(Wire1, i2cAddress);
    delay(50);
    int status = sensors1[ch].beginI2C(i2cAddress, Wire1);
    while (status != BMP5_OK) {
      Serial.print("Error: BMP585 not connected on Bus1 channel ");
      Serial.print(ch);
      Serial.println(". Retrying...");
      delay(1000);
      tcaselectWire1(ch);
      status = sensors1[ch].beginI2C(i2cAddress, Wire1);
    }
    Serial.print("Sensor on Bus1 connected on channel ");
    Serial.println(ch);
  }

  
  // Initialize sensors on Bus2 (Wire2)
  for (uint8_t ch = 0; ch < 8; ch++) {
    tcaselectWire2(ch);
    resetBMP585(Wire2, i2cAddress);
    delay(50);
    int status = sensors2[ch].beginI2C(i2cAddress, Wire2);
    while (status != BMP5_OK) {
      Serial.print("Error: BMP585 not connected on Bus2 channel ");
      Serial.print(ch);
      Serial.println(". Retrying...");
      delay(1000);
      tcaselectWire2(ch);
      status = sensors2[ch].beginI2C(i2cAddress, Wire2);
    }
    Serial.print("Sensor on Bus2 connected on channel ");
    Serial.println(ch);
  }
  

  // Fault clearance for actuators
  uint8_t faultclr[] = {0x55, 0xAA, 0x05, 0xFF,
                        0x32, 0x18, 0x00, 0x01,
                        0x00, 0x4F};
  Serial.println("Performing fault clearance on actuators.");
  ACTUATOR_1.write(faultclr, 10);
  ACTUATOR_2.write(faultclr, 10);
  delay(100);
}

void loop() {
  // Read sensors on Bus0 (Wire)
  String outputLine0 = "";
  float total0 = 0.0;
  for (uint8_t ch = 0; ch < 8; ch++) {
    tcaselectWire0(ch);
    delay(10); // Ensure settling after channel switching
    bmp5_sensor_data data = {0, 0};
    int8_t err = sensors0[ch].getSensorData(&data);
    if (err == BMP5_OK) {
      outputLine0 += String(data.pressure);
      total0 += data.pressure;
    } else {
      outputLine0 += "ERROR";
    }
    if (ch < 7) outputLine0 += ",";
  }
  float avg0 = total0 / 8.0;

  // Debug output:
  Serial.println("Bus2 Sensor Data: " + outputLine2);
  Serial.println("Average Bus2: " + String(avg2));

  // Read sensors on Bus1 (Wire1)
  String outputLine1 = "";
  float total1 = 0.0;
  for (uint8_t ch = 0; ch < 8; ch++) {
    tcaselectWire1(ch);
    delay(10);
    bmp5_sensor_data data = {0, 0};
    int8_t err = sensors1[ch].getSensorData(&data);
    if (err == BMP5_OK) {
      outputLine1 += String(data.pressure);
      total1 += data.pressure;
    } else {
      outputLine1 += "ERROR";
    }
    if (ch < 7) outputLine1 += ",";
  }
  float avg1 = total1 / 8.0;

  // Debug output:
  Serial.println("Bus2 Sensor Data: " + outputLine2);
  Serial.println("Average Bus2: " + String(avg2));

  // Read sensors on Bus2 (Wire2)
  String outputLine2 = "";
  float total2 = 0.0;
  for (uint8_t ch = 0; ch < 8; ch++) {
    tcaselectWire2(ch);
    delay(10);
    bmp5_sensor_data data = {0, 0};
    int8_t err = sensors2[ch].getSensorData(&data);
    if (err == BMP5_OK) {
      outputLine1 += String(data.pressure);
      total2 += data.pressure;
    } else {
      outputLine2 += "ERROR";
    }
    if (ch < 7) outputLine1 += ",";
  }
  float avg2 = total2 / 8.0;

  // Debug output:
  Serial.println("Bus2 Sensor Data: " + outputLine2);
  Serial.println("Average Bus2: " + String(avg2));

  // Actuator control example based on sensor averages:
  // avg0 > 100000.0 || 
  if (avg0 > 100000.0 || avg1 > 100000.0 || avg2 > 100000.0) {    
    uint8_t forcemode[] = {0x55, 0xAA, 0x05, 0xFF, 
                        0x32, 0x25, 0x00, 0x03,
                        0x00, 0x5E};

    // sets the control mode as the positioning mode

    ACTUATOR_1.write(forcemode,10);
    ACTUATOR_2.write(forcemode,10);

    uint8_t force[] = {0x55, 0xAA, 0x05, 0xFF, 
                        0x32, 0x27, 0x00, 0xD0,
                        0x07, 0x34};


    delay(10);

    ACTUATOR_1.write(force,10);
    ACTUATOR_2.write(force,10);

  } else {
    uint8_t posmode[] = {0x55, 0xAA, 0x05, 0xFF,
                         0x32, 0x25, 0x00, 0x00,
                         0x00, 0x5B};
    ACTUATOR_1.write(posmode, 10);
    ACTUATOR_2.write(posmode, 10);
    delay(10);
    uint8_t fullretract[] = {0x55, 0xAA, 0x05, 0xFF,
                             0x32, 0x29, 0x00, 0x00,
                             0x00, 0x5F};
    ACTUATOR_1.write(fullretract, 10);
    ACTUATOR_2.write(fullretract, 10);
  }
  
  delay(10);
}



// position mode to 1500
    // uint8_t forcemode[] = {0x55, 0xAA, 0x05, 0xFF,
    //                        0x32, 0x25, 0x00, 0x00,
    //                        0x00, 0x5B};
    // ACTUATOR_1.write(forcemode, 10);
    // ACTUATOR_2.write(forcemode, 10);
    // delay(10);
    // uint8_t force[] = {0x55, 0xAA, 0x05, 0xFF,
    //                    0x32, 0x29, 0x00, 0xE8,
    //                    0x03, 0x4A};
    // delay(10);
    // ACTUATOR_1.write(force, 10);
    // ACTUATOR_2.write(force, 10);
