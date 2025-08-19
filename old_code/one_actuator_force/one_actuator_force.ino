
// #define ACTUATOR_1 Serial5


// //////////////////////////////////////////////////////////////////////////////////////////////////////////
// //////////////////////////////////////////////////////////////////////////////////////////////////////////

// uint8_t calculateChecksum(uint8_t *data, int length) {
//     uint16_t sum = 0;  // Use a larger type to avoid overflow
//     for (int i = 2; i < length; i++) {
//         sum += data[i];
//       }  
//     return static_cast<uint8_t>(sum & 0xFF);  // Return only the least significant byte
// }


// void setup() {
  
//   Serial.begin(9600);
//   while (!Serial);
//   // while (!Serial);    // Wait for the serial port to connect (for native USB)
//   Serial.println("Ready to receive CSV data.");
//   // Initializes serial communication with linear actuator
//   ACTUATOR_1.begin(921600);
//   while (!Serial);


//   uint8_t faultclr[] = {0x55, 0xAA, 0x05, 0xFF, 
//                         0x32, 0x18, 0x00, 0x01,
//                         0x00};

  

//   int faultLength_1 = sizeof(faultclr) / sizeof(faultclr[0]);
//   uint8_t checksum_4 = calculateChecksum(faultclr, faultLength_1);
//   ACTUATOR_1.write(faultclr, faultLength_1);
//   ACTUATOR_1.write(checksum_4);


//   delay(1000);
    

//   // Control mode to positioning mode instruction frame
//   // Header (2B) + Data length (1B) + ID (1B) + Instruction Type (1B) + register address (2B) + data (NB)
//   uint8_t forcemode[] = {0x55, 0xAA, 0x05, 0xFF, 
//                          0x32, 0x25, 0x00, 0x03,
//                          0x00};

  

//   Serial.println("Force control mode actuator 1 begun...");
//   int forcemodeLength_1 = sizeof(forcemode) / sizeof(forcemode[0]);
//   uint8_t checksum_1 = calculateChecksum(forcemode, forcemodeLength_1);
//   ACTUATOR_1.write(forcemode, forcemodeLength_1);
//   ACTUATOR_1.write(checksum_1);

//   delay(1000);
  
//   uint8_t package_3[] = {0x55, 0xAA, 0x05, 0xFF, 
//                          0x32, 0x27, 0x00, 0xE8, 
//                          0x03};
  

//   int packageLength_3 = sizeof(package_3) / sizeof(package_3[0]);
//   uint8_t checksum_3 = calculateChecksum(package_3, packageLength_3);
//   ACTUATOR_1.write(package_3, packageLength_3);
//   ACTUATOR_1.write(checksum_3);

//   delay(1000);

// }


// //////////////////////////////////////////////////////////////////////////////////////////////////////////
// //////////////////////////////////////////////////////////////////////////////////////////////////////////


// void loop() {

//   // uint16_t force_in_grams = 100;
//   // sendCommand(force_in_grams);
  

// }


// #define ACTUATOR_1 Serial4

// //////////////////////////////////////////////////////////////////////////////////////////////////////////
// //////////////////////////////////////////////////////////////////////////////////////////////////////////

// uint8_t calculateChecksum(uint8_t *data, int length) {
//     uint16_t sum = 0;
//     for (int i = 2; i < length; i++) {
//         sum += data[i];
//     }
//     return static_cast<uint8_t>(sum & 0xFF);
// }

// void sendFrame(uint8_t *frame, int length) {
//     uint8_t checksum = calculateChecksum(frame, length);
//     ACTUATOR_1.write(frame, length);
//     ACTUATOR_1.write(checksum);
//     delay(100);  // Allow some time for the actuator to process
// }

// void readResponse() {
//     Serial.print("Response: ");
//     while (ACTUATOR_1.available()) {
//         uint8_t byteReceived = ACTUATOR_1.read();
//         Serial.print("0x");
//         Serial.print(byteReceived, HEX);
//         Serial.print(" ");
//     }
//     Serial.println();
// }

// void setup() {
//     Serial.begin(9600);
//     while (!Serial);
//     ACTUATOR_1.begin(921600);
//     while (!ACTUATOR_1);

//     // 1. Fault Clearance Command
//     // uint8_t faultclr[] = {0x55, 0xAA, 0x05, 0x03, 0x32, 0x18, 0x00, 0x01, 0x00};
//     // sendFrame(faultclr, sizeof(faultclr));
//     // readResponse();

//     // 2. Set Force Control Mode Command
//     uint8_t forcemode[] = {0x55, 0xAA, 0x05, 0x01, 0x32, 0x25, 0x00, 0x03, 0x00};
//     sendFrame(forcemode, sizeof(forcemode));
//     Serial.println("Force control mode enabled");
//     readResponse();

//     delay(100);

    
// }

// void loop() {
//     // If additional force values or adjustments are needed, they can be sent here
//   // 3. Set Target Force to 1000g Command
//     uint8_t targetForce[] = {0x55, 0xAA, 0x05, 0x01, 0x32, 0x27, 0x00, 0xE8, 0x03};
//     sendFrame(targetForce, sizeof(targetForce));
//     Serial.println("Target force set to 1000g");
//     readResponse();

//     delay(10);


// }



#define ACTUATOR_1 Serial5

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t calculateChecksum(uint8_t *data, int length) {
    uint16_t sum = 0;
    for (int i = 2; i < length; i++) {
        sum += data[i];
    }
    return static_cast<uint8_t>(sum & 0xFF);
}

void printFrame(const char* label, uint8_t *frame, int length) {
    Serial.print(label);
    for (int i = 0; i < length; i++) {
        Serial.print(" 0x");
        if (frame[i] < 0x10) Serial.print("0");  // Add leading zero for single-digit hex values
        Serial.print(frame[i], HEX);
    }
    Serial.println();
}

void sendFaultClearCommand() {
    // Construct fault-clear command frame
    uint8_t fault_clear_frame[] = {
        0x55, 0xAA,      // Frame header
        0x05,            // Data length
        0xFF,            // ID (broadcast)
        0x32,            // CMD_WR_REGISTER
        0x18, 0x00,      // Register address for fault clear
        0x01,            // Fault clear command
        0x00             // Placeholder for checksum
    };

    int frameLength = sizeof(fault_clear_frame) / sizeof(fault_clear_frame[0]);
    uint8_t checksum = calculateChecksum(fault_clear_frame, frameLength);
    
    // Print the fault-clear command
    printFrame("Sending Fault Clear Command:", fault_clear_frame, frameLength);
    Serial.print(" Checksum: 0x");
    if (checksum < 0x10) Serial.print("0");
    Serial.println(checksum, HEX);

    // Send fault-clear command frame with checksum
    ACTUATOR_1.write(fault_clear_frame, frameLength);
    ACTUATOR_1.write(checksum);
}

void sendForceControlCommand(uint16_t target_force) {
    // Construct frame based on Method 2 for force control
    uint8_t command_frame[] = {
        0x55, 0xAA,      // Frame header
        0x09,            // Data length
        0xFF,            // ID
        0x32,            // CMD_WR_REGISTER
        0x25,            // Register address for control mode
        0x00,            // Control mode: force control
        0x03, 0x00,           // Set control mode to force control
        0x00, 0x00,      // Motor output voltage (set to 0x0000 in force control mode)
        static_cast<uint8_t>(target_force & 0xFF),       // Target force low byte
        static_cast<uint8_t>((target_force >> 8) & 0xFF) // Target force high byte
    };

    int frameLength = sizeof(command_frame) / sizeof(command_frame[0]);
    uint8_t checksum = calculateChecksum(command_frame, frameLength);

    // Print the force control command being sent
    printFrame("Sending Force Control Command:", command_frame, frameLength);
    Serial.print(" Checksum: 0x");
    if (checksum < 0x10) Serial.print("0");
    Serial.println(checksum, HEX);

    // Send force control command frame with checksum
    ACTUATOR_1.write(command_frame, frameLength);
    ACTUATOR_1.write(checksum);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void readResponse() {
    if (ACTUATOR_1.available()) {
        uint8_t response[20];  // Assuming response length is within 20 bytes
        int i = 0;
        while (ACTUATOR_1.available() && i < 20) {
            response[i++] = ACTUATOR_1.read();
        }

        // Print the received response frame
        printFrame("Received Response:", response, i);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
    Serial.begin(9600);
    while (!Serial);
    ACTUATOR_1.begin(921600);
    while (!ACTUATOR_1);

    // Send fault clear command
    sendFaultClearCommand();
    delay(500);  // Delay to ensure fault clear is processed before sending force control command

    Serial.println("Starting continuous force control command transmission.");

    sendForceControlCommand(100);
}

void loop() {
    // Continuously send force control command with target of 1000g (0x03E8)
    
    
    // Read and print response frames if any
    readResponse();
    
    delay(100);  // Short delay to control the frequency of command sending
}

