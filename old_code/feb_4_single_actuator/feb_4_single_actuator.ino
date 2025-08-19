#define start_byte1 0x55
#define start_byte2 0xAA
#define response_byte1 0xAA
#define response_byte2 0x55
#define actuator1_ID 0x01  // Change as needed
#define inst_type 0x32

#define ACTUATOR_1 Serial1

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function to calculate checksum
//////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t calculateChecksum(uint8_t *data, int length) {
    uint16_t sum = 0;
    for (int i = 2; i < length; i++) {  
        sum += data[i];
    }  
    return static_cast<uint8_t>(sum & 0xFF);  // Return only the least significant byte
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function to send a command
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void sendCommand(uint16_t cmnd_1) {
    uint8_t lowByte1 = cmnd_1 & 0xFF;
    uint8_t highByte1 = (cmnd_1 >> 8) & 0xFF;

    uint8_t csvcommand_1[] = {0x29, 0x00, lowByte1, highByte1};

    Serial.print("Moving to position: ");
    Serial.println(cmnd_1);

    uint8_t cmndLength = sizeof(csvcommand_1);

    uint8_t package_1[5 + cmndLength] = {start_byte1, start_byte2, cmndLength + 3, actuator1_ID, inst_type};

    for (int i = 0; i < cmndLength; i++) {
        package_1[5 + i] = csvcommand_1[i];
    }

    uint8_t checksum_1 = calculateChecksum(package_1, sizeof(package_1));

    ACTUATOR_1.write(package_1, sizeof(package_1));
    ACTUATOR_1.write(checksum_1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function to read actuator response
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void readResponse(HardwareSerial &actuator) {
    uint8_t response[20];
    size_t bytesRead = 0;

    unsigned long startTime = millis();
    while (millis() - startTime < 100) {  
        if (actuator.available()) {
            if (bytesRead < sizeof(response)) {
                response[bytesRead++] = actuator.read();
            }
        }
    }

    if (bytesRead > 0) {
        handleResponse(response, bytesRead);
    } else {
        Serial.println("No response received.");
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function to handle actuator response
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void handleResponse(uint8_t* response, int length) {
    if (length < 10 || response[0] != response_byte1 || response[1] != response_byte2 || response[4] != inst_type) {
        Serial.print("Invalid response received. Raw data: ");
        for (int i = 0; i < length; i++) {
            Serial.print("0x");
            Serial.print(response[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        return;
    }

    uint16_t pos = (static_cast<uint16_t>(response[8]) << 8) | static_cast<uint16_t>(response[9]);
    Serial.print("Received position: ");
    Serial.println(pos);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function to parse CSV input
//////////////////////////////////////////////////////////////////////////////////////////////////////////
int parseCSV(String csvData) {
    csvData.trim();
    if (csvData.length() == 0) return -1;
    return csvData.toInt();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function to send a command with checksum calculation
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void sendCommandWithChecksum(uint8_t* command, uint8_t length) {
    uint8_t checksum = calculateChecksum(command, length);
    ACTUATOR_1.write(command, length);
    ACTUATOR_1.write(checksum);
    Serial.println(checksum);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup function
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
    Serial.begin(115200);
    while (!Serial);

    ACTUATOR_1.begin(921600);

    Serial.println("Initializing actuator...");

    // // Fault Clearance Command
    // uint8_t faultclr[] = {start_byte1, start_byte2, 0x05, actuator1_ID, inst_type, 0x18, 0x00, 0x01, 0x00};
    // Serial.println("Clearing faults...");
    // sendCommandWithChecksum(faultclr, sizeof(faultclr));
    // delay(100);
    // readResponse(ACTUATOR_1);

    // Set Positioning Mode Command
    // 
    // Serial.println("Setting to positioning mode...");
    // sendCommandWithChecksum(ctrlmode, sizeof(ctrlmode));
    uint8_t ctrlmode[] = {start_byte1, start_byte2, 0x05, actuator1_ID, inst_type, 0x25, 0x00, 0x00, 0x00, 0x5D};
    delay(1000);
    
    readResponse(ACTUATOR_1);
    
    delay(3000);

    // Move Actuator to 20 Steps
    Serial.println("Moving actuator to 20 steps...");
    sendCommand(20);
    delay(2000);
    readResponse(ACTUATOR_1);
    
    // Move Actuator to 1500 Steps
    Serial.println("Moving actuator to 1500 steps...");
    sendCommand(1500);
    delay(2000);
    readResponse(ACTUATOR_1);

    // Retract Actuator to 20 Steps
    Serial.println("Retracting actuator to 20 steps...");
    sendCommand(20);
    delay(2000);
    readResponse(ACTUATOR_1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Loop function
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
    if (Serial.available() > 0) {
        String csvData = Serial.readStringUntil('\n');
        csvData.trim();
        int receivedValue = parseCSV(csvData);

        if (receivedValue >= 0) {
            Serial.print("Received integer: ");
            Serial.println(receivedValue);
            sendCommand(receivedValue);
        } else {
            Serial.println("Invalid input.");
        }
    }
}
