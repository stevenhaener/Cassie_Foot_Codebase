#define start_byte1 0x55
#define start_byte2 0xAA
#define response_byte1 0xAA
#define response_byte2 0x55
#define actuator1_ID 0xFF   // Set unique actuator IDs
#define actuator2_ID 0xFF
#define inst_type 0x32

#define ACTUATOR_1 Serial5
#define ACTUATOR_2 Serial6

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
// Function to send position command
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void sendCommand(uint16_t cmnd_1) {
    uint8_t lowByte1 = cmnd_1 & 0xFF;
    uint8_t highByte1 = (cmnd_1 >> 8) & 0xFF;

    int cmnd_2 = cmnd_1 * 1.06 + 30;  // Apply actuator-specific offset
    uint8_t lowByte2 = cmnd_2 & 0xFF;
    uint8_t highByte2 = (cmnd_2 >> 8) & 0xFF;

    uint8_t csvcommand_1[] = {0x29, 0x00, lowByte1, highByte1};
    uint8_t csvcommand_2[] = {0x29, 0x00, lowByte2, highByte2};

    Serial.print("Moving to position: ");
    Serial.println(cmnd_1);

    uint8_t cmndLength = sizeof(csvcommand_1);

    uint8_t package_1[5 + cmndLength] = {start_byte1, start_byte2, cmndLength + 3, actuator1_ID, inst_type};
    uint8_t package_2[5 + cmndLength] = {start_byte1, start_byte2, cmndLength + 3, actuator2_ID, inst_type};

    for (int i = 0; i < cmndLength; i++) {
        package_1[5 + i] = csvcommand_1[i];
        package_2[5 + i] = csvcommand_2[i];
    }

    uint8_t checksum_1 = calculateChecksum(package_1, sizeof(package_1));
    uint8_t checksum_2 = calculateChecksum(package_2, sizeof(package_2));

    ACTUATOR_1.write(package_1, sizeof(package_1));
    ACTUATOR_1.write(checksum_1);

    ACTUATOR_2.write(package_2, sizeof(package_2));
    ACTUATOR_2.write(checksum_2);
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
        Serial.println("Invalid response frame");
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
// Setup function
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
    Serial.begin(115200);
    while (!Serial);

    ACTUATOR_1.begin(921600);
    ACTUATOR_2.begin(921600);

    Serial.println("Initializing actuators...");

    uint8_t faultclr[] = {0x55, 0xAA, 0x05, actuator1_ID, 0x32, 0x18, 0x00, 0x01, 0x00, 0x55};

    Serial.println("Clearing faults...");
    ACTUATOR_1.write(faultclr, sizeof(faultclr));
    ACTUATOR_2.write(faultclr, sizeof(faultclr));
    delay(100);
    readResponse(ACTUATOR_1);
    readResponse(ACTUATOR_2);

    uint8_t ctrlmode[] = {0x55, 0xAA, 0x05, actuator1_ID, 0x32, 0x25, 0x00, 0x00, 0x00, 0x5E};

    Serial.println("Setting to positioning mode...");
    ACTUATOR_1.write(ctrlmode, sizeof(ctrlmode));
    ACTUATOR_2.write(ctrlmode, sizeof(ctrlmode));
    delay(1000);
    readResponse(ACTUATOR_1);
    readResponse(ACTUATOR_2);

    delay(3000);

    Serial.println("Moving actuators to 20 steps...");
    sendCommand(20);
    delay(2000);
    readResponse(ACTUATOR_1);
    readResponse(ACTUATOR_2);

    Serial.println("Moving actuators to 1500 steps...");
    sendCommand(1500);
    delay(2000);
    readResponse(ACTUATOR_1);
    readResponse(ACTUATOR_2);

    Serial.println("Retracting actuators to 20 steps...");
    sendCommand(20);
    delay(2000);
    readResponse(ACTUATOR_1);
    readResponse(ACTUATOR_2);
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
