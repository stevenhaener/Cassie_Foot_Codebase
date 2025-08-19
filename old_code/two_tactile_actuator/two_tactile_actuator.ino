

#define start_byte1 0x55
#define start_byte2 0xAA
#define response_byte1 0xAA
#define response_byte2 0x55
#define actuator1_ID 0x01
#define actuator2_ID 0x02
#define inst_type 0x32

#define ACTUATOR_1 Serial6
#define ACTUATOR_2 Serial7
#include <Wire.h>
#include "SparkFun_BMP581_Arduino_Library.h"

#define TCA9548A_ADDR 0x70  // Default I2C address of the TCA9548A
// I2C address selection
#define i2cAddress 0x47

BMP581 pressureSensor;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void tcaselect(uint8_t channel) {
  if (channel > 7) return;  // Ensure channel is valid (0-7)
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);  // Select the channel by writing a bitmask
  Wire.endTransmission();
}

void tcaselect1(uint8_t channel) {
  if (channel > 7) return;  // Ensure channel is valid (0-7)
  Wire1.beginTransmission(TCA9548A_ADDR);
  Wire1.write(1 << channel);  // Select the channel by writing a bitmask
  Wire1.endTransmission();
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
   
  Serial.begin(9600);
  while (!Serial);
  // NUC.begin(9600);
  // while (!Serial);    // Wait for the serial port to connect (for native USB)

   // Initialize the I2C library
    Wire.begin();
    Wire1.begin();
    
  for (uint8_t i = 0; i < 8; i++) {
    tcaselect(i);  // Select channel on the multiplexer
    Serial.print("Selected "); Serial.print(i); Serial.println("-th sensor.");  
    // Check if sensor is connected and initialize
    delay(10);
    if (Wire.requestFrom(0x47, 1)) {
      Serial.println("Device detected on channel 0.");
    } else {
      Serial.println("No device detected on channel 0.");
    }
    // Address is optional (defaults to 0x47)
    int rst = pressureSensor.beginI2C(i2cAddress);
    while (rst != BMP5_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMP581 not connected, try again after 1 sec!");
        Serial.printf("The error code is %d.\n", rst);
        delay(1000);
    }
    Serial.println("Successfully connected.");
  }

  for (uint8_t i = 0; i < 8; i++) {
    tcaselect1(i);  // Select channel on the multiplexer
    Serial.print("Selected "); Serial.print(i); Serial.println("-th sensor.");  
    // Check if sensor is connected and initialize
    delay(10);
    if (Wire1.requestFrom(0x47, 1)) {
      Serial.println("Device detected on channel 0.");
    } else {
      Serial.println("No device detected on channel 0.");
    }
    // Address is optional (defaults to 0x47)
    int rst = pressureSensor.beginI2C(i2cAddress);
    while (rst != BMP5_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMP581 not connected, try again after 1 sec!");
        Serial.printf("The error code is %d.\n", rst);
        delay(1000);
    }
    Serial.println("Successfully connected.");
  }

  Serial.println("Ready to receive CSV data.");
  // Initializes serial communication with linear actuator
  ACTUATOR_1.begin(921600);
  while (!Serial);
  ACTUATOR_2.begin(921600);
  while (!Serial);


  // Fault clearance instruction frame
  // Header (2B) + Data length (1B) + ID (1B) + Instruction Type (1B) + register address (2B) + data (NB)
  uint8_t faultclr[] = {0x55, 0xAA, 0x05, 0xFF, 
                        0x32, 0x18, 0x00, 0x01,
                        0x00, 0x4F};

  //  Fault clearance for linear actuator
  // if (ACTUATOR.availableForWrite() > 0) {

  Serial.println("Fault clearance");
  ACTUATOR_1.write(faultclr,10);
  ACTUATOR_2.write(faultclr,10);
    
  delay(100);
  // }
  
 }


//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////


void loop() {

  String outputLine_1 = ""; // Create a string to store the line of data
  float total_1 = 0.0;

  for (uint8_t i = 0; i < 8; i++) {
    tcaselect(i);  // Select channel on the multiplexer
    
    // Get measurements from the sensor
    bmp5_sensor_data data = {0, 0};
    int8_t err = pressureSensor.getSensorData(&data);

    // Check whether data was acquired successfully
    if (err == BMP5_OK) {
      // Append the pressure data to the output string
      outputLine_1 += String(data.pressure);
      total_1 += float(data.pressure);
    } else {
      // Append an error placeholder if acquisition failed
      outputLine_1 += "ERROR";
    }

    // Add a comma after every reading except the last one
    if (i < 7) {
      outputLine_1 += ",";
    }
  }

  float avg_1 = total_1/8.0; 

/////////////////////////////////////////////////////////////////////////////////////////////////

  String outputLine_2 = ""; // Create a string to store the line of data
  float total_2 = 0.0;

  for (uint8_t i = 0; i < 8; i++) {
    tcaselect1(i);  // Select channel on the multiplexer
    
    // Get measurements from the sensor
    bmp5_sensor_data data = {0, 0};
    int8_t err = pressureSensor.getSensorData(&data);

    // Check whether data was acquired successfully
    if (err == BMP5_OK) {
      // Append the pressure data to the output string
      outputLine_2 += String(data.pressure);
      total_2 += float(data.pressure);
    } else {
      // Append an error placeholder if acquisition failed
      outputLine_2 += "ERROR";
    }

    // Add a comma after every reading except the last one
    if (i < 7) {
      outputLine_2 += ",";
    }
  }

  float avg_2 = total_2/8.0; 

/////////////////////////////////////////////////////////////////////////////////////////////////

  // Print the complete line of data
  Serial.println(outputLine_2);
  Serial.println(avg_2);

if (avg_1 > 99000.0 || avg_2 > 99000.0) {
  // Header (2B) + Data length (1B) + ID (1B) + Instruction Type (1B) + register address (2B) + data (NB)
  uint8_t forcemode[] = {0x55, 0xAA, 0x05, 0xFF, 
                        0x32, 0x25, 0x00, 0x03,
                        0x00, 0x5E};

  // sets the control mode as the positioning mode

  ACTUATOR_1.write(forcemode,10);
  ACTUATOR_2.write(forcemode,10);

  uint8_t force[] = {0x55, 0xAA, 0x05, 0xFF, 
                      0x32, 0x27, 0x00, 0xD0,
                      0x07, 0x34};


  delay(500);

  ACTUATOR_1.write(force,10);
  ACTUATOR_2.write(force,10);
} else {
  
  // sets the control mode as the positioning mode
  uint8_t posmode[] = {0x55, 0xAA, 0x05, 0xFF, 
                        0x32, 0x25, 0x00, 0x00,
                        0x00, 0x5B};


  ACTUATOR_1.write(posmode,10);
  ACTUATOR_2.write(posmode,10);

  delay(500);

  uint8_t fullretract[] = {0x55, 0xAA, 0x05, 0xFF, 
                           0x32, 0x29, 0x00, 0x00,
                           0x00, 0x5F};


  ACTUATOR_1.write(fullretract,10);
  ACTUATOR_2.write(fullretract,10);

}

}





