#define BAUD_RATE 115200
#define TIMEOUT_MS 100  // Timeout for response in milliseconds

// Define all available UART ports
HardwareSerial* serialPorts[] = { 
    &Serial1, &Serial2, &Serial3, &Serial4, 
    &Serial5, &Serial6, &Serial7, &Serial8 
};
const char* portNames[] = { 
    "Serial1", "Serial2", "Serial3", "Serial4", 
    "Serial5", "Serial6", "Serial7", "Serial8" 
};

void setup() {
    Serial.begin(BAUD_RATE);  // USB Serial
    while (!Serial);  // Wait for Serial Monitor

    // Initialize all available UART ports
    for (int i = 0; i < 8; i++) {
        serialPorts[i]->begin(921600);
    }

    delay(1000);  // Allow devices to initialize
    Serial.println("Starting UART scan...");
}

void loop() {
    for (int i = 0; i < 8; i++) {
        scanSerialLine(serialPorts[i], portNames[i]);
    }

    Serial.println("Scan complete.\n");
    delay(2000);  // Wait before next scan
}

void scanSerialLine(HardwareSerial *serialPort, const char *portName) {
    serialPort->flush();  // Clear buffer
    while (serialPort->available()) serialPort->read();  // Empty input buffer

    uint8_t ctrlmode[] = {0x55, 0xAA, 0x05, 0x01, 0x32, 0x25, 0x00, 0x00, 0x00, 0x5D};
    serialPort->write(ctrlmode, sizeof(ctrlmode));  // Send test message
    
    unsigned long startTime = millis();
    uint8_t responseBuffer[64];  // Buffer to store response
    size_t bytesRead = 0;

    while (millis() - startTime < TIMEOUT_MS) {
        while (serialPort->available() && bytesRead < sizeof(responseBuffer)) {
            responseBuffer[bytesRead++] = serialPort->read();
        }
    }

    if (bytesRead > 0) {
        Serial.print("Device found on ");
        Serial.print(portName);
        Serial.print(": Response (Raw) -> \"");

        // Print response as raw characters
        for (size_t i = 0; i < bytesRead; i++) {
            Serial.write(responseBuffer[i]);  // Print raw bytes as characters
        }
        Serial.println("\"");

        Serial.print("Device found on ");
        Serial.print(portName);
        Serial.print(": Response (Hex) -> ");

        // Print response in HEX format
        for (size_t i = 0; i < bytesRead; i++) {
            Serial.print("0x");
            if (responseBuffer[i] < 0x10) Serial.print("0");  // Add leading zero for single-digit hex
            Serial.print(responseBuffer[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    } else {
        Serial.print(portName);
        Serial.println(" - No response");
    }
}
