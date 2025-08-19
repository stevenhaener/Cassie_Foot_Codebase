#include <Wire.h>

#define TCA9548A_ADDR 0x70  // Default I2C address of the TCA9548A
#define IIC_ADDRESS 0x47    // I2C address of BMP585

#define PRESS_DATA_MSB 0x22
#define PRESS_DATA_LSB 0x21
#define PRESS_DATA_XLSB 0x20

// Function to select the desired channel on the multiplexer
void tcaselect(uint8_t channel) {
  if (channel > 7) return;  // Ensure channel is valid (0-7)
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);  // Select the channel by writing a bitmask
  Wire.endTransmission();
}

// Function to read a single byte from a register
uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(IIC_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(IIC_ADDRESS, 1);
  return Wire.available() ? Wire.read() : 0;
}

// Function to read pressure data
float readPressure() {
  uint8_t msb = readRegister(PRESS_DATA_MSB);
  uint8_t lsb = readRegister(PRESS_DATA_LSB);
  uint8_t xlsb = readRegister(PRESS_DATA_XLSB);

  // Combine the three bytes into a 20-bit value
  uint32_t rawPressure = ((uint32_t)msb << 16) | ((uint32_t)lsb << 8) | xlsb;

  // Convert to pressure in Pa (divide by 64)
  return rawPressure / 64.0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("BMP585 Initialization");

  Wire.begin();

  for (uint8_t i = 0; i < 8; i++) {
    tcaselect(i);
    delay(50);  // Allow stabilization
    Serial.print("Selected channel: "); Serial.println(i);

    // Dummy read to ensure communication
    uint8_t chip_id = readRegister(0x00);  // CHIP_ID register
    Serial.print("CHIP_ID: 0x"); Serial.println(chip_id, HEX);
  }
}

void loop() {
  for (uint8_t i = 0; i < 8; i++) {
    tcaselect(i);
    delay(50);  // Stabilization delay
    Serial.print("Selected channel: "); Serial.println(i);

    float pressure = readPressure();
    if (pressure > 0) {
      Serial.print("Pressure (Pa): ");
      Serial.println(pressure);
    } else {
      Serial.println("Error: Failed to read valid pressure data.");
    }

    delay(500);  // Adjust delay as needed
  }
}
