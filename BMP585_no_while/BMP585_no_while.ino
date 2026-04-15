#include <Wire.h>
#include "SparkFun_BMP581_Arduino_Library.h"

// I²C mux and sensor address
#define TCA9548A_ADDR 0x70
#define BMP_ADDR      0x47

// Number of channels on the TCA9548A
const uint8_t CH_PER_BANK = 8;
const uint8_t MAX_SELECT_ATTEMPTS = 3; // how many times to try selecting the mux channel
const uint8_t MAX_INIT_ATTEMPTS = 3;   // how many times to try beginI2C before skipping

// Array of BMP581 sensors (one object per channel)
BMP581 sensors[CH_PER_BANK];

// Keep track of which sensors were successfully initialized
bool sensorOk[CH_PER_BANK] = { false };

// Forward declarations
bool tcaselect(uint8_t ch);
void resetBMP585(uint8_t addr);
void initSensors();

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  Serial.println("Initializing BMP581 tactile sensors...");

  // Uncomment to run a quick I2C scan for debugging before init
  // i2cScanner();

  initSensors();

  Serial.println("Setup complete.");
}

void loop() {
  String outputLine;
  outputLine.reserve(CH_PER_BANK * 8);

  // Read all channels and build a CSV-style line
  for (uint8_t ch = 0; ch < CH_PER_BANK; ch++) {
    if (!tcaselect(ch)) {
      outputLine += "ERR";
    } else {
      delay(5);
      if (!sensorOk[ch]) {
        // We previously failed to initialize this channel
        outputLine += "NO_DEV";
      } else {
        bmp5_sensor_data d;
        if (sensors[ch].getSensorData(&d) == BMP5_OK) {
          // Use one decimal place (you used this previously)
          outputLine += String(d.pressure, 1);
        } else {
          outputLine += "ERR";
        }
      }
    }

    if (ch < CH_PER_BANK - 1) outputLine += ",";
  }

  Serial.println(outputLine);
  delay(20);
}

//--------------------------------------------------------------------------------
// Select a channel on the TCA9548A
// Returns true on success, false on failure
//--------------------------------------------------------------------------------
bool tcaselect(uint8_t ch) {
  if (ch > 7) return false;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << ch);
  uint8_t res = Wire.endTransmission();
  if (res != 0) {
    // failed to talk to mux
    Serial.print("tcaselect: failed to select channel ");
    Serial.print(ch);
    Serial.print(" (Wire.endTransmission() = ");
    Serial.print(res);
    Serial.println(")");
    return false;
  }
  delay(5);
  return true;
}

//--------------------------------------------------------------------------------
// Soft-reset BMP585 (write 0xB6 to register 0x7E)
//--------------------------------------------------------------------------------
void resetBMP585(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x7E); // CMD register for BMP585 family
  Wire.write(0xB6); // soft reset command
  uint8_t res = Wire.endTransmission();
  if (res != 0) {
    Serial.print("resetBMP585: write returned ");
    Serial.println(res);
  } else {
    Serial.println("resetBMP585: soft reset command sent");
  }
  delay(5);
}

//--------------------------------------------------------------------------------
// Initialize CH_PER_BANK BMP581 sensors behind a TCA9548A
//--------------------------------------------------------------------------------
void initSensors() {
  Wire.setClock(200000); // keep your chosen I2C speed

  for (uint8_t ch = 0; ch < CH_PER_BANK; ch++) {
    // Try selecting the channel — limited attempts
    bool selected = false;
    for (uint8_t selAttempt = 0; selAttempt < MAX_SELECT_ATTEMPTS; selAttempt++) {
      if (tcaselect(ch)) {
        selected = true;
        break;
      }
      Serial.print("tcaselect attempt ");
      Serial.print(selAttempt + 1);
      Serial.print(" failed for channel ");
      Serial.println(ch);
      delay(100);
    }

    if (!selected) {
      Serial.print("Channel ");
      Serial.print(ch);
      Serial.println(": Failed to select channel after retries. Marking as not present.");
      sensorOk[ch] = false;
      continue; // move to next channel
    }

    // Quick presence check (ACK-based)
    Wire.beginTransmission(BMP_ADDR);
    uint8_t pres = Wire.endTransmission();
    if (pres != 0) {
      Serial.print("Channel ");
      Serial.print(ch);
      Serial.println(": No device found at BMP address. Marking as not present.");
      sensorOk[ch] = false;
      continue; // move to next channel
    }

    Serial.print("Channel ");
    Serial.print(ch);
    Serial.println(": Device detected. Sending soft reset...");

    // Soft reset and allow it to settle
    resetBMP585(BMP_ADDR);
    delay(50); // your chosen settle time

    // Try to initialize up to MAX_INIT_ATTEMPTS times, then give up and move on.
    int attempt = 0;
    int rst = -1;
    while (attempt < MAX_INIT_ATTEMPTS) {
      rst = sensors[ch].beginI2C(BMP_ADDR, Wire);
      if (rst == BMP5_OK) {
        Serial.print("Channel ");
        Serial.print(ch);
        Serial.println(": Initialization successful.");
        sensorOk[ch] = true;
        break;
      } else {
        Serial.print("Channel ");
        Serial.print(ch);
        Serial.print(": beginI2C attempt ");
        Serial.print(attempt + 1);
        Serial.print(" failed (err=");
        Serial.print(rst);
        Serial.println(").");
        attempt++;
        delay(200); // your chosen retry delay
      }
    }

    if (rst != BMP5_OK) {
      Serial.print("Channel ");
      Serial.print(ch);
      Serial.println(": Initialization failed after retries. Skipping sensor.");
      sensorOk[ch] = false;
    }

    delay(5);
  }

  // deselect all channels
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
}

