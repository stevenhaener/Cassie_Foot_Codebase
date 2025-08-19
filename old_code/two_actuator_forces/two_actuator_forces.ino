

#define start_byte1 0x55
#define start_byte2 0xAA
#define response_byte1 0xAA
#define response_byte2 0x55
#define actuator1_ID 0x01
#define actuator2_ID 0x02
#define inst_type 0x32

#define ACTUATOR_1 Serial6
#define ACTUATOR_2 Serial7


//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////


uint8_t checksum(uint8_t *data, int length) {
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
    uint8_t checksum_1 = checksum(package_1, packageLength_1);
    ACTUATOR_1.write(package_1, packageLength_1);
    ACTUATOR_1.write(checksum_1); // Send checksum as the last byte

    int packageLength_2 = sizeof(package_2) / sizeof(package_1[0]);
    uint8_t checksum_2 = checksum(package_2, packageLength_2);
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
                        0x00};

  //  Fault clearance for linear actuator
  // if (ACTUATOR.availableForWrite() > 0) {

  int faultclrLength = sizeof(faultclr) / sizeof(faultclr[0]);
  uint8_t checksum_fc = checksum(faultclr, faultclrLength);
  
  Serial.println("Fault clearance");
  ACTUATOR_1.write(faultclr,faultclrLength);
  ACTUATOR_1.write(checksum_fc); 
  ACTUATOR_2.write(faultclr,faultclrLength);
  ACTUATOR_2.write(checksum_fc); 
    
  delay(100);
  // }

  // Reads fault clearance response frame
  readResponse();



  // Control mode to positioning mode instruction frame
  // Header (2B) + Data length (1B) + ID (1B) + Instruction Type (1B) + register address (2B) + data (NB)
  uint8_t ctrlmode[] = {0x55, 0xAA, 0x05, 0xFF, 
                        0x32, 0x25, 0x00, 0x03,
                        0x00};

  // sets the control mode as the positioning mode


int ctrlmodeLength = sizeof(ctrlmode) / sizeof(ctrlmode[0]);
uint8_t checksum_cm = checksum(ctrlmode, ctrlmodeLength);

Serial.println("Positioning mode actuator 1 begun...");
ACTUATOR_1.write(ctrlmode,ctrlmodeLength);
ACTUATOR_1.write(checksum_cm);
ACTUATOR_2.write(ctrlmode,ctrlmodeLength);
ACTUATOR_2.write(checksum_cm);

// uint8_t force[] = {0x55, 0xAA, 0x05, 0xFF, 
//                       0x32, 0x27, 0x00, 0xE8,
//                       0x03, 0x48};

delay(10);

Serial.println("Positioning mode actuator 2 begun...");
// ACTUATOR_1.write(force,10);
  

 }


//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////


void loop() {

  // uint8_t force[] = {0x55, 0xAA, 0x05, 0xFF, 
  //                     0x32, 0x27, 0x00, 0xE8,
  //                     0x03, 0x48};


uint8_t force[] = {0x55, 0xAA, 0x05, 0xFF, 
                   0x32, 0x27, 0x00, 0x82,
                   0x00};


  delay(100);
  int i = 0;

  int forceLength = sizeof(force) / sizeof(force[0]);
  uint8_t checksum_f = checksum(force, forceLength);
  
  //Serial.println("Positioning mode actuator 2 begun...")
  ACTUATOR_1.write(force,forceLength);
  ACTUATOR_1.write(checksum_f);
  //Serial.println(checksum_f);
  ACTUATOR_2.write(force,forceLength);
  ACTUATOR_1.write(checksum_f);
  

  
}









