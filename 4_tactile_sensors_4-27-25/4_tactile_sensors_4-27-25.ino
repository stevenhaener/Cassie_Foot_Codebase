#include <Wire.h>
#include "SparkFun_BMP581_Arduino_Library.h"

// Actuator and protocol definitions
#define start_byte1     0x55
#define start_byte2     0xAA
#define response_byte1  0xAA
#define response_byte2  0x55
#define actuator1_ID    0x01
#define actuator2_ID    0x02
#define inst_type       0x32

#define ACTUATOR_1 Serial1
#define ACTUATOR_2 Serial1

// I²C addresses
#define TCA9548A_ADDR    0x70
#define TCA9548A_ADDR_2  0x71
#define BMP_ADDR         0x47  // BMP585

// One array per bank of 8 sensors (total 32)
BMP581 sensors0[8];   // bank 0: Wire + mux @0x70
BMP581 sensors1[8];   // bank 1: Wire1 + mux @0x70
BMP581 sensors2[8];   // bank 2: Wire2 + mux @0x70
BMP581 sensors3[8];   // bank 3: Wire  + mux @0x71

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
// Function to create and send a command
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
// Function to send a command with checksum calculation
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void sendCommandWithChecksum(uint8_t* command, uint8_t length) {
    uint8_t checksum = calculateChecksum(command, length);
    ACTUATOR_1.write(command, length);
    ACTUATOR_1.write(checksum);
    Serial.println(checksum);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Soft-reset BMP585
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void resetBMP585(TwoWire &bus, uint8_t addr) {
  bus.beginTransmission(addr);
  bus.write(0x7E);
  bus.write(0xB6);
  bus.endTransmission();
  delay(2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Select one channel on the given mux
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void tcaselect(TwoWire &bus, uint8_t muxAddr, uint8_t ch) {
  if (ch > 7) return;
  bus.beginTransmission(muxAddr);
  bus.write(1 << ch);
  bus.endTransmission();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialize a bank of 8 BMP585 sensors
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void initSensors(TwoWire &bus, BMP581 sensors[], uint8_t muxAddr, const char *label) {
  for (uint8_t ch = 0; ch < 8; ch++) {
    tcaselect(bus, muxAddr, ch);
    resetBMP585(bus, BMP_ADDR);
    delay(50);
    while (sensors[ch].beginI2C(BMP_ADDR, bus) != BMP5_OK) {
      Serial.print("Retrying ");
      Serial.print(label);
      Serial.print(" ch ");
      Serial.println(ch);
      delay(500);
      tcaselect(bus, muxAddr, ch);
    }
    Serial.print(label);
    Serial.print(" sensor ch ");
    Serial.println(ch);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// void setup
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Initialize I²C busses
  Wire.begin();
  Wire1.begin();
  Wire2.begin();

  // Initialize actuator serial ports
  ACTUATOR_1.begin(921600);
  while (!ACTUATOR_1);
  ACTUATOR_2.begin(921600);
  while (!ACTUATOR_2);

  // Bring up all 4 banks (8 sensors each = 32 total)
  initSensors(Wire,  sensors0, TCA9548A_ADDR,   "Bank0 (Wire @0x70)");
  initSensors(Wire1, sensors1, TCA9548A_ADDR,   "Bank1 (Wire1 @0x70)");
  initSensors(Wire2, sensors2, TCA9548A_ADDR,   "Bank2 (Wire2 @0x70)");
  initSensors(Wire,  sensors3, TCA9548A_ADDR_2, "Bank3 (Wire  @0x71)");

  // Fault clearance on actuators
  uint8_t faultclr[] = {
    0x55,0xAA,0x05,0xFF,
    inst_type,0x18,0x00,0x01,
    0x00,0x4F
  };
  Serial.println("Performing fault clearance on actuators.");
  ACTUATOR_1.write(faultclr, sizeof(faultclr));
  ACTUATOR_2.write(faultclr, sizeof(faultclr));
  delay(100);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// void loop
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  // Helper to read a bank and return its average pressure
  auto readAvg = [&](TwoWire &bus, BMP581 sensors[], uint8_t muxAddr) {
    float total = 0;
    for (uint8_t ch = 0; ch < 8; ch++) {
      tcaselect(bus, muxAddr, ch);
      delay(10);
      bmp5_sensor_data d;
      if (sensors[ch].getSensorData(&d) == BMP5_OK) {
        total += d.pressure;
      }
    }
    return total / 8.0;
  };

  // Read banks in numeric order
  float avg0 = readAvg(Wire,  sensors0, TCA9548A_ADDR);
  float avg1 = readAvg(Wire1, sensors1, TCA9548A_ADDR);
  float avg2 = readAvg(Wire2, sensors2, TCA9548A_ADDR);
  float avg3 = readAvg(Wire,  sensors3, TCA9548A_ADDR_2);

  // Debug prints
  Serial.print("Avg0: "); Serial.println(avg0);
  Serial.print("Avg1: "); Serial.println(avg1);
  Serial.print("Avg2: "); Serial.println(avg2);
  Serial.print("Avg3: "); Serial.println(avg3);

  // Actuator decision using all four averages
  if (avg0 > 100000.0 || avg1 > 100000.0 ||
      avg2 > 100000.0 || avg3 > 100000.0) {
    uint8_t forcemode[] = {0x55, 0xAA, 0x05, 0xFF, 
                        0x32, 0x25, 0x00, 0x03,
                        0x00, 0x5E};

    // sets the control mode as the positioning mode

    ACTUATOR_1.write(forcemode,10);
    ACTUATOR_2.write(forcemode,10);

    uint8_t force[] = {0x55, 0xAA, 0x05, 0xFF, 
                       0x32, 0x27, 0x00, 0xD0,
                       0x07};


    //old checksum 0x34
    uint8_t checksum_1 = calculateChecksum(force, sizeof(force));

    Serial.println(checksum_1);

    delay(10);

    ACTUATOR_1.write(force,10);
    ACTUATOR_2.write(force,10);

  } else {
    uint8_t posmode[] = {0x55, 0xAA, 0x05, 0xFF,
                         0x32, 0x25, 0x00, 0x00,
                         0x00, 0x5B};
    ACTUATOR_1.write(posmode, 10);
    ACTUATOR_2.write(posmode, 10);
    
    delay(10);

    uint8_t fullretract[] = {0x55, 0xAA, 0x05, 0xFF,
                             0x32, 0x29, 0x00, 0x00,
                             0x00, 0x5F};
    ACTUATOR_1.write(fullretract, 10);
    ACTUATOR_2.write(fullretract, 10);
  }
  
  delay(10);
}
