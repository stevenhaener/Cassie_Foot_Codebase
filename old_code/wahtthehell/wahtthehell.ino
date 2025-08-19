
#define start_byte1 0x55
#define start_byte2 0xAA
#define response_byte1 0xAA
#define response_byte2 0x55
#define actuator1_ID 0xFF
#define actuator2_ID 0xFF
#define inst_type 0x32
#define force_mode 0x03
#define pos_mode 0x00

#define ACTUATOR_1 Serial4
#define ACTUATOR_2 Serial5


//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t calculateChecksum(uint8_t *data, int length) {
    uint16_t sum = 0;  // Use a larger type to avoid overflow
    for (int i = 2; i < length; i++) {
        sum += data[i];
      }  
    return static_cast<uint8_t>(sum & 0xFF);  // Return only the least significant byte
}


void setup() {
  
  Serial.begin(9600);
  while (!Serial);
  // while (!Serial);    // Wait for the serial port to connect (for native USB)
  Serial.println("Ready to receive CSV data.");
  // Initializes serial communication with linear actuator
  ACTUATOR_1.begin(921600);
  while (!Serial);
  ACTUATOR_2.begin(921600);
  while (!Serial);


  uint8_t faultclr[10] = {0x55, 0xAA, 0x05, 0xFF, 
                        0x32, 0x18, 0x00, 0x01,
                        0x00, 0x55};

  //  Fault clearance for linear actuator
  // if (ACTUATOR.availableForWrite() > 0) {

  Serial.println("Fault clearance");
  ACTUATOR_1.write(faultclr,10);
  ACTUATOR_2.write(faultclr,10);
    
  delay(100);

  // Control mode to positioning mode instruction frame
  // Header (2B) + Data length (1B) + ID (1B) + Instruction Type (1B) + register address (2B) + data (NB)
  uint8_t forcemode[10] = {0x55, 0xAA, 0x05, 0xFF, 
                        0x32, 0x25, 0x00, 0x00,
                        0x00, 0x5B};

  

  Serial.println("Force control mode actuator 1 begun...");
  ACTUATOR_1.write(forcemode,10);
  delay(100);
  Serial.println("Force control mode actuator 2 begun...");
  ACTUATOR_2.write(forcemode,10);
  
  delay(1000);
  
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////


void loop() {

  uint8_t package_1[9] = {0x55, 0xAA, 0x05, 0xFF, 
                           0x32, 0x29, 0x00, 0xE8, 
                           0x03};
  

  int packageLength_1 = sizeof(package_1) / sizeof(package_1[0]);
  uint8_t checksum_1 = calculateChecksum(package_1, packageLength_1);
  ACTUATOR_1.write(package_1, packageLength_1);
  ACTUATOR_1.write(checksum_1);

  ACTUATOR_2.write(package_1, packageLength_1);
  ACTUATOR_2.write(checksum_1);

  delay(1000);

  uint8_t package_2[9] = {0x55, 0xAA, 0x05, 0xFF, 
                          0x32, 0x29, 0x00, 0x00, 
                          0x00};
  

  int packageLength_2 = sizeof(package_2) / sizeof(package_1[0]);
  uint8_t checksum_2 = calculateChecksum(package_2, packageLength_2);
  ACTUATOR_1.write(package_2, packageLength_2);
  ACTUATOR_1.write(checksum_2);

  ACTUATOR_2.write(package_2, packageLength_2);
  ACTUATOR_2.write(checksum_2);

  delay(1000);

}













