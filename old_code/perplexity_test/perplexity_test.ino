#include <Wire.h>
#include <Adafruit_BMP3XX.h>

#define BMP390_I2C_ADDRESS 0x77  // Default I2C address if SDO is high

Adafruit_BMP3XX bmp;  // Create an instance of the BMP3XX class

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Initialize I2C on Wire2 (pins 24=SDA2, 25=SCL2)
  Wire2.begin();

  // Initialize the BMP390 sensor with Wire2
  if (!bmp.begin_I2C(BMP390_I2C_ADDRESS, &Wire2)) {
    Serial.println("Failed to initialize BMP390 sensor!");
    while (1);
  }

  // Configure BMP390 settings
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);  // Set to 50 Hz for faster sampling

  Serial.println("BMP390 initialized successfully in normal mode.");
}

void loop() {
  // Read sensor data
  if (bmp.performReading()) {
    Serial.print("Pressure: ");
    Serial.print(bmp.pressure / 100.0);  // Convert Pa to hPa
    Serial.println(" hPa");

    Serial.print("Temperature: ");
    Serial.print(bmp.temperature);
    Serial.println(" °C");

    delay(20);  // 20ms delay for ~50Hz update rate
  } else {
    Serial.println("Failed to read data from BMP390 sensor!");
  }
}
