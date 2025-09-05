

#define start_byte1 0x55
#define start_byte2 0xAA
#define response_byte1 0xAA
#define response_byte2 0x55
#define actuator1_ID 0x01
#define actuator2_ID 0x02
#define inst_type 0x32

#define ACTUATOR_1 Serial1
#define ACTUATOR_2 Serial1
#include <Wire.h>
#include "SparkFun_BMP581_Arduino_Library.h"

#define TCA9548A_ADDR 0x70  // Default I2C address of the TCA9548A
// I2C address selection
#define i2cAddress 0x47

BMP581 pressureSensor;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() {
   
  Serial.begin(9600);
  while (!Serial);
  // NUC.begin(9600);
  // while (!Serial);    // Wait for the serial port to connect (for native USB)

   // Initialize the I2C library
    // Wire.begin();
    Wire1.begin();
  
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

  // sets the control mode as the positioning mode
  uint8_t posmode[] = {0x55, 0xAA, 0x05, 0xFF, 
                        0x32, 0x25, 0x00, 0x00,
                        0x00, 0x5B};


  ACTUATOR_1.write(posmode,10);
  ACTUATOR_2.write(posmode,10);

  delay(100);

  uint8_t fullretract[] = {0x55, 0xAA, 0x05, 0xFF, 
                           0x32, 0x29, 0x00, 0x00,
                           0x00, 0x5F};


  ACTUATOR_1.write(fullretract,10);
  ACTUATOR_2.write(fullretract,10);
  
 }


//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////


void loop() {  

}






