#include <Arduino.h>

// Constants for framing and actuator control
#define START_BYTE1 0x55
#define START_BYTE2 0xAA
#define RESPONSE_BYTE1 0xAA
#define RESPONSE_BYTE2 0x55
#define ACTUATOR_ID 0x01  // Based on documentation example
#define CMD_WR_REGISTER 0x32
#define INST_TYPE 0x32
#define FORCE_MODE 0x03
#define POS_MODE 0x00

// Register Addresses
#define CTRL_MODE_REGISTER 0x25
#define TARGET_POS_REGISTER 0x29
#define TARGET_FORCE_REGISTER 0x27

// Serial definitions for actuators
#define ACTUATOR_1 Serial4
#define ACTUATOR_2 Serial5

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// Function to calculate checksum
uint8_t calculateChecksum(uint8_t *data, int length) {
    uint16_t sum = 0;
    for (int i = 2; i < length; i++) {
        sum += data[i];
    }
    return static_cast<uint8_t>(sum & 0xFF);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Function to parse CSV data
int parseCSV(String csvData) {
    return csvData.toInt();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Function to set positioning control mode
void setPositionControlMode() {
    uint8_t positionModeFrame[] = {
        START_BYTE1, START_BYTE2, 0x05, ACTUATOR_ID,
        CMD_WR_REGISTER, CTRL_MODE_REGISTER, 0x00, POS_MODE, 0x00
    };
    positionModeFrame[8] = calculateChecksum(positionModeFrame, 9);
    
    Serial.println("Setting positioning mode...");
    ACTUATOR_1.write(positionModeFrame, 9);
    ACTUATOR_2.write(positionModeFrame, 9);
    delay(100);
    readResponse();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Function to set target position in steps
void setTargetPosition(uint16_t targetSteps) {
    uint8_t lowByte = targetSteps & 0xFF;
    uint8_t highByte = (targetSteps >> 8) & 0xFF;

    uint8_t targetPosFrame[] = {
        START_BYTE1, START_BYTE2, 0x05, ACTUATOR_ID,
        CMD_WR_REGISTER, TARGET_POS_REGISTER, 0x00, highByte, lowByte
    };
    targetPosFrame[8] = calculateChecksum(targetPosFrame, 9);
    
    Serial.print("Setting target position: ");
    Serial.print(targetSteps);
    Serial.println(" steps");
    ACTUATOR_1.write(targetPosFrame, 9);
    ACTUATOR_2.write(targetPosFrame, 9);
    delay(100);
    readResponse();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Function to set force control mode
void setForceControlMode() {
    uint8_t forceModeFrame[] = {
        START_BYTE1, START_BYTE2, 0x05, ACTUATOR_ID,
        CMD_WR_REGISTER, CTRL_MODE_REGISTER, 0x00, FORCE_MODE, 0x00
    };
    forceModeFrame[8] = calculateChecksum(forceModeFrame, 9);
    
    Serial.println("Setting force control mode...");
    ACTUATOR_1.write(forceModeFrame, 9);
    ACTUATOR_2.write(forceModeFrame, 9);
    delay(100);
    readResponse();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Function to set target force in grams
void setTargetForce(uint16_t targetForce) {
    uint8_t lowByte = targetForce & 0xFF;
    uint8_t highByte = (targetForce >> 8) & 0xFF;

    uint8_t forceTargetFrame[] = {
        START_BYTE1, START_BYTE2, 0x05, ACTUATOR_ID,
        CMD_WR_REGISTER, TARGET_FORCE_REGISTER, 0x00, highByte, lowByte
    };
    forceTargetFrame[8] = calculateChecksum(forceTargetFrame, 9);
    
    Serial.print("Setting target force: ");
    Serial.print(targetForce);
    Serial.println(" grams");
    ACTUATOR_1.write(forceTargetFrame, 9);
    ACTUATOR_2.write(forceTargetFrame, 9);
    delay(100);
    readResponse();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Function to read response from the actuator
void readResponse() {
    uint8_t response[20];
    size_t bytesRead = 0;
    int length = 20;

    while (ACTUATOR_2.available() > 0) {
        uint8_t incomingByte = ACTUATOR_2.read();
        if (bytesRead < static_cast<size_t>(length - 1)) {
            response[bytesRead] = incomingByte;
            bytesRead++;
        }
    }
    handleResponse(response, bytesRead);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Function to handle and parse actuator response
void handleResponse(uint8_t* response, int length) {
    if (length < 20 || response[0] != RESPONSE_BYTE1 || response[1] != RESPONSE_BYTE2) {
        Serial.println("Invalid response frame or insufficient length");
        return;
    }

    // Extract key data based on the response format
    uint16_t actualPosition = (static_cast<uint16_t>(response[8]) << 8) | static_cast<uint16_t>(response[9]);
    uint16_t currentForce = (static_cast<uint16_t>(response[14]) << 8) | static_cast<uint16_t>(response[15]);
    uint8_t temperature = response[18];
    uint8_t errorCode = response[19];

    // Print the parsed data
    Serial.print("Actual Position: ");
    Serial.println(actualPosition);
    Serial.print("Current Force: ");
    Serial.println(currentForce);
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" °C, Error Code: ");
    Serial.println(errorCode);
    Serial.println("");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
    Serial.begin(9600);
    while (!Serial);

    ACTUATOR_1.begin(921600);
    ACTUATOR_2.begin(921600);

    // Step 1: Set Positioning Mode
    setPositionControlMode();

    // Step 2: Set initial target position to 1000 steps in positioning mode
    uint16_t initialPosition = 1000;
    setTargetPosition(initialPosition);

    // Step 3: Switch to Force Control Mode
    setForceControlMode();

    // Step 4: Set target force value to 1000 grams in force control mode
    uint16_t targetForce = 1000;
    setTargetForce(targetForce);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
    static uint32_t lastUpdate = 0;
    uint32_t now = millis();
    
    // Resend target force every 500 ms (adjust interval as needed)
    if (now - lastUpdate > 500) {
        uint16_t targetForce = 1000;  // Target force in grams
        setTargetForce(targetForce);
        lastUpdate = now;
    }

    // Check for new CSV data input and parse it if available
    if (Serial.available() > 0) {
        String csvData = Serial.readStringUntil('\n');
        csvData.trim();
        int receivedValue = parseCSV(csvData);

        Serial.print("Received integer from CSV: ");
        Serial.println(receivedValue);

        // Update the target position based on CSV input in positioning mode
        setTargetPosition(receivedValue);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
