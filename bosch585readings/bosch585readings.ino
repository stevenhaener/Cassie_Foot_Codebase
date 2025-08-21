#include <Wire.h>
#include "SparkFun_BMP581_Arduino_Library.h"

// I²C mux and sensor address
#define TCA9548A_ADDR 0x70
#define BMP_ADDR      0x47

// Number of channels on the TCA9548A
const uint8_t CH_PER_BANK = 8;

// Array of 8 BMP581 sensors on Wire + mux
BMP581 sensors[CH_PER_BANK];

// Forward declarations
bool tcaselect(uint8_t ch);
void resetBMP585(uint8_t addr);
void initSensors();

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  Serial.println("Initializing BMP581 tactile sensors...");

  initSensors();

  Serial.println("Setup complete.");
}

void loop() {
  String outputLine;
  outputLine.reserve(CH_PER_BANK * 8);

  // Read all 8 sensors
  for (uint8_t ch = 0; ch < CH_PER_BANK; ch++) {
    if (!tcaselect(ch)) {
      outputLine += "ERR";
    } else {
      delay(3);
      bmp5_sensor_data d;
      if (sensors[ch].getSensorData(&d) == BMP5_OK) {
        outputLine += String(d.pressure, 1);
      } else {
        outputLine += "ERR";
      }
    }
    if (ch < CH_PER_BANK - 1) outputLine += ",";
  }

  Serial.println(outputLine);
  delay(20);
}

//--------------------------------------------------------------------------------
// Select a channel on the TCA9548A
//--------------------------------------------------------------------------------
bool tcaselect(uint8_t ch) {
  if (ch > 7) return false;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
  delay(2);
  return true;
}

//--------------------------------------------------------------------------------
// Soft-reset BMP581
//--------------------------------------------------------------------------------
void resetBMP585(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x7E);
  Wire.write(0xB6);
  Wire.endTransmission();
  delay(2);
}

//--------------------------------------------------------------------------------
// Initialize 8 BMP581 sensors
//--------------------------------------------------------------------------------
void initSensors() {
  Wire.setClock(50000);
  for (uint8_t ch = 0; ch < CH_PER_BANK; ch++) {
    while (!tcaselect(ch)) delay(100);
    resetBMP585(BMP_ADDR);
    delay(50);
    while (sensors[ch].beginI2C(BMP_ADDR, Wire) != BMP5_OK) {
      delay(200);
    }
  }
  // deselect all channels
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
}
