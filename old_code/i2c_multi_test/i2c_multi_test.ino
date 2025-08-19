#include <Wire.h>

#define BMP390_ADDR 0x76  // Update this to 0x77 if SDO is tied to VDDIO
#define TCAADDR 0x70      // Multiplexer address

// Register addresses
#define REG_PWR_CTRL 0x1B
#define REG_OSR 0x1C
#define REG_CONFIG 0x1F
#define REG_PRESS_MSB 0x04
#define REG_TEMP_MSB 0x07

void initBMP390() {
  // Set power control and forced mode
  Wire2.beginTransmission(BMP390_ADDR);
  Wire2.write(REG_PWR_CTRL);
  Wire2.write(0x13);  // press_en = 1, temp_en = 1, mode = forced
  Wire2.endTransmission();

  // Set oversampling configuration
  Wire2.beginTransmission(BMP390_ADDR);
  Wire2.write(REG_OSR);
  Wire2.write(0x02);  // osr_p = x4, osr_t = x1 for moderate resolution
  Wire2.endTransmission();

  // Set IIR filter configuration
  Wire2.beginTransmission(BMP390_ADDR);
  Wire2.write(REG_CONFIG);
  Wire2.write(0x02);  // Enable IIR filter
  Wire2.endTransmission();
}

void readBMP390(uint8_t channel) {
  // First, send the register address in write mode
  Wire2.beginTransmission(BMP390_ADDR);
  Wire2.write(REG_PRESS_MSB);  // Start from pressure MSB register
  Wire2.endTransmission(false); // End transmission with a restart

  // Now, request 6 bytes of data from the BMP390
  Wire2.requestFrom(BMP390_ADDR, 6);
  if (Wire2.available() == 6) {
    uint32_t press = (Wire2.read() << 16) | (Wire2.read() << 8) | Wire2.read();
    uint32_t temp = (Wire2.read() << 16) | (Wire2.read() << 8) | Wire2.read();

    // Output data to Serial
    Serial.print("Channel ");
    Serial.print(channel);
    Serial.print(" - Pressure: ");
    Serial.print(press);
    Serial.print(" Pa, Temperature: ");
    Serial.print(temp);
    Serial.println(" °C");
  } else {
    Serial.print("Channel ");
    Serial.print(channel);
    Serial.println(" - No data available.");
  }
}

void tcaselect(uint8_t channel) {
  if (channel > 7) return;
  Wire2.beginTransmission(TCAADDR);
  Wire2.write(1 << channel);
  Wire2.endTransmission();
}

void setup() {
  Serial.begin(115200);
  Wire2.begin();
  delay(1000);

  Serial.println("Initializing BMP390 sensors on all channels...");
  for (uint8_t channel = 0; channel < 8; channel++) {
    tcaselect(channel);
    initBMP390();
    Serial.print("Initialized BMP390 on channel ");
    Serial.println(channel);
  }
  Serial.println("Initialization complete.");
}

void loop() {
  for (uint8_t channel = 0; channel < 8; channel++) {
    tcaselect(channel);
    readBMP390(channel);
    delay(1);  // Small delay to allow data collection
  }
  delay(10);  // Repeat every 10ms, approximately 1kHz collection
}
