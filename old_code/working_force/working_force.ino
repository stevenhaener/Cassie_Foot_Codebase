

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

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t calculateChecksum(uint8_t *data, int length) {
    uint16_t sum = 0;  // Use a larger type to avoid overflow
    for (int i = 2; i < length; i++) {
        sum += data[i];
      }  
    return static_cast<uint8_t>(sum & 0xFF);  // Return only the least significant byte
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void sendCommand(uint16_t cmnd_1) {

    unsigned char lowByte1, highByte1;
    unsigned char lowByte2, highByte2;

    // Mask the low-order byte and high-order byte
    lowByte1 = cmnd_1 & 0xFF;        // Extract the low-order byte (last 8 bits)
    highByte1 = (cmnd_1 >> 8) & 0xFF; // Extract the high-order byte (first 8 bits)

    Serial.println(static_cast<uint8_t>(lowByte1));
    Serial.println(static_cast<uint8_t>(highByte1));

    // adds offset to calibrate both actuators

    
    int cmnd_2 = cmnd_1*1.06 + 30;

    // Mask the low-order byte and high-order byte
    lowByte2 = cmnd_2 & 0xFF;        // Extract the low-order byte (last 8 bits)
    highByte2 = (cmnd_2 >> 8) & 0xFF; // Extract the high-order byte (first 8 bits)

    uint8_t csvcommand_1[] = {0x29, 0x00, lowByte1, highByte1};
    uint8_t csvcommand_2[] = {0x29, 0x00, lowByte2, highByte2};

    Serial.print("Moving to position: ");
    Serial.println((static_cast<uint16_t>(csvcommand_1[3]) << 8) | static_cast<uint16_t>(csvcommand_1[2]));

    uint8_t cmndLength_1 = sizeof(csvcommand_1)+1 / sizeof(csvcommand_1[0]);
    uint8_t cmndLength_2 = sizeof(csvcommand_2)+1 / sizeof(csvcommand_2[0]);
    
    uint8_t package_1[4+cmndLength_1] = {start_byte1,
                                         start_byte2,
                                         cmndLength_1,
                                         actuator1_ID,
                                         inst_type};

    uint8_t package_2[4+cmndLength_2] = {start_byte1,
                                         start_byte2,
                                         cmndLength_2,
                                         actuator2_ID,
                                         inst_type};

    for (int i = 0; i < cmndLength_1; i++) {
      package_1[5+i] = csvcommand_1[i];
      package_2[5+i] = csvcommand_2[i];
    }                             

    int packageLength_1 = sizeof(package_1) / sizeof(package_1[0]);
    uint8_t checksum_1 = calculateChecksum(package_1, packageLength_1);
    ACTUATOR_1.write(package_1, packageLength_1);
    ACTUATOR_1.write(checksum_1); // Send checksum as the last byte

    int packageLength_2 = sizeof(package_2) / sizeof(package_1[0]);
    uint8_t checksum_2 = calculateChecksum(package_2, packageLength_2);
    ACTUATOR_2.write(package_2, packageLength_2);
    ACTUATOR_2.write(checksum_2); // Send checksum as the last byte

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Function to read incoming data from the actuator
void readResponse() {

    uint8_t response[20];
    size_t bytesRead = 0;
    int length = 20;
    while (ACTUATOR_2.available() > 0) {
      uint8_t incomingByte = ACTUATOR_2.read();  // Read one byte from serial

      // Add the byte to the array if there's space
      if (bytesRead < static_cast<size_t>(length - 1)) {
        response[bytesRead] = incomingByte;  // Store the byte in the array
        bytesRead++;                          // Increment the index
    } 
  }

  // Checks if the first and second byte of the response frame are valid
  handleResponse(response, length);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void handleResponse(byte* response, int length) {
      // Validate and process the response frame here
      if (response[0] != response_byte1 || response[1] != response_byte2) {
           Serial.println("Invalid response frame");
           return;
      }
    
      // Process the response frame
      uint16_t pos = (static_cast<uint16_t>(response[8]) << 8) | static_cast<uint16_t>(response[9]);

      // Further processing...
      Serial.print("Received position: ");
      Serial.println(pos, DEC); // Print the response ID in hexadecimal
      Serial.println("");
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int parseCSV(String csvData) {
    // This function assumes the CSV data is a single-item CSV (one integer in a row)
    // We can directly convert the CSV string to an integer
    int value = csvData.toInt(); // Convert the CSV string to an integer
    //int value = value;
    return value;                // Return the extracted integer
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

  // Reads fault clearance response frame
  readResponse();


// uint8_t force[] = {0x55, 0xAA, 0x05, 0xFF, 
//                       0x32, 0x27, 0x00, 0xE8,
//                       0x03, 0x48};

// delay(10);

// Serial.println("Positioning mode actuator 2 begun...");
// ACTUATOR_1.write(force,10);
// ACTUATOR_2.write(force,10);

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
  // Print the complete line of data
  Serial.println(outputLine_1);
  Serial.println(avg_1);

if (avg > 102000.0) {
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


  delay(100);

  ACTUATOR_1.write(force,10);
  ACTUATOR_2.write(force,10);
} else {
  
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

}





