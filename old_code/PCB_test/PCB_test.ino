#include <Wire.h>
#include <Adafruit_BMP3XX.h>

#define TCA9548A_ADDR 0x70  // Default I2C address of the TCA9548A
#define BMP390_I2C_ADDRESS 0x77  // Default I2C address if SDO is high

Adafruit_BMP3XX bmp;  // Create an instance of the BMP3XX class

// Function to select the desired channel on the multiplexer
void tcaselect(uint8_t channel) {
  if (channel > 7) return;  // Ensure channel is valid (0-7)
  Wire2.beginTransmission(TCA9548A_ADDR);
  Wire2.write(1 << channel);  // Select the channel by writing a bitmask
  Wire2.endTransmission();
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Initialize I2C on Wire2 (pins 24=SDA2, 25=SCL2)
  Wire2.begin();

  // Initialize and configure each BMP390 sensor
  for (uint8_t i = 0; i < 8; i++) {
    tcaselect(i);  // Select channel on the multiplexer
    if (!bmp.begin_I2C(BMP390_I2C_ADDRESS, &Wire2)) {
      Serial.print("Failed to initialize BMP390 on channel ");
      Serial.println(i);
    } else {
      bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
      bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
      bmp.setOutputDataRate(BMP3_ODR_50_HZ);
      Serial.print("BMP390 initialized successfully on channel ");
      Serial.println(i);
    }
  }
}

void loop() {
  for (uint8_t i = 0; i < 8; i++) {
    tcaselect(i);  // Select channel on the multiplexer

    // Read sensor data
    if (bmp.performReading()) {
      Serial.print("Channel ");
      Serial.print(i);
      Serial.print(": Pressure = ");
      Serial.print(bmp.pressure / 100.0);  // Output pressure in hPa
      Serial.println(" hPa");
    } else {
      Serial.print("Channel ");
      Serial.print(i);
      Serial.println(": Failed to read pressure");
    }

    delay(200);  // Ensure each channel gets time to stabilize (~50Hz sampling)
  }
}
