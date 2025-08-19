#include <Wire.h>

#define BMP390_ADDR 0x77  // I²C address when SDO is tied to VDDIO
#define REG_PWR_CTRL 0x1B
#define REG_OSR 0x1C
#define REG_CONFIG 0x1F
#define REG_PRESS_MSB 0x04
#define REG_TEMP_MSB 0x07
#define REG_CALIB_DATA 0x31  // Start address of calibration data

struct BMP390_Calib {
  float par_t1, par_t2, par_t3;
  float par_p1, par_p2, par_p3, par_p4, par_p5, par_p6, par_p7, par_p8, par_p9, par_p10, par_p11;
} calib;

void readCalibrationData() {
  Wire2.beginTransmission(BMP390_ADDR);
  Wire2.write(REG_CALIB_DATA);
  Wire2.endTransmission();
  Wire2.requestFrom(BMP390_ADDR, 21);

  if (Wire2.available() == 21) {
    // Temperature coefficients
    uint16_t NVM_PAR_T1 = Wire2.read() | (Wire2.read() << 8);
    calib.par_t1 = NVM_PAR_T1 / 256.0;

    int16_t NVM_PAR_T2 = Wire2.read() | (Wire2.read() << 8);
    calib.par_t2 = NVM_PAR_T2 / 1073741824.0;

    int8_t NVM_PAR_T3 = Wire2.read();
    calib.par_t3 = NVM_PAR_T3 / 281474976710656.0;

    // Pressure coefficients
    int16_t NVM_PAR_P1 = Wire2.read() | (Wire2.read() << 8);
    calib.par_p1 = (NVM_PAR_P1 - 16384) / 1048576.0;

    int16_t NVM_PAR_P2 = Wire2.read() | (Wire2.read() << 8);
    calib.par_p2 = (NVM_PAR_P2 - 16384) / 536870912.0;

    int8_t NVM_PAR_P3 = Wire2.read();
    calib.par_p3 = NVM_PAR_P3 / 4294967296.0;

    int8_t NVM_PAR_P4 = Wire2.read();
    calib.par_p4 = NVM_PAR_P4 / 137438953472.0;

    uint8_t NVM_PAR_P5 = Wire2.read();
    calib.par_p5 = NVM_PAR_P5 / 8.0;

    uint8_t NVM_PAR_P6 = Wire2.read();
    calib.par_p6 = NVM_PAR_P6 / 64.0;

    int8_t NVM_PAR_P7 = Wire2.read();
    calib.par_p7 = NVM_PAR_P7 / 256.0;

    int8_t NVM_PAR_P8 = Wire2.read();
    calib.par_p8 = NVM_PAR_P8 / 32768.0;

    int16_t NVM_PAR_P9 = Wire2.read() | (Wire2.read() << 8);
    calib.par_p9 = NVM_PAR_P9 / 281474976710656.0;

    int8_t NVM_PAR_P10 = Wire2.read();
    calib.par_p10 = NVM_PAR_P10 / 281474976710656.0;

    int8_t NVM_PAR_P11 = Wire2.read();
    calib.par_p11 = NVM_PAR_P11 / 36893488147419103232.0;
  } else {
    Serial.println("Failed to read calibration data.");
  }
}

void initBMP390() {
  // Set to normal mode for continuous measurement
  Wire2.beginTransmission(BMP390_ADDR);
  Wire2.write(REG_PWR_CTRL);
  Wire2.write(0x30);  // Enable temperature and pressure, normal mode
  if (Wire2.endTransmission() != 0) {
    Serial.println("Failed to set power control to normal mode.");
  }

  // Configure oversampling and IIR filter
  Wire2.beginTransmission(BMP390_ADDR);
  Wire2.write(REG_OSR);
  Wire2.write(0x02);  // Moderate oversampling
  if (Wire2.endTransmission() != 0) {
    Serial.println("Failed to set oversampling.");
  }

  Wire2.beginTransmission(BMP390_ADDR);
  Wire2.write(REG_CONFIG);
  Wire2.write(0x02);  // IIR filter configuration
  if (Wire2.endTransmission() != 0) {
    Serial.println("Failed to set filter configuration.");
  }
}

void readBMP390() {
  // Step 1: Send the register address (start from REG_PRESS_MSB)
  Wire2.beginTransmission(BMP390_ADDR);
  Wire2.write(REG_PRESS_MSB);
  if (Wire2.endTransmission(false) != 0) {
    Serial.println("Failed to set register address for reading.");
  }

  // Step 2: Read 6 bytes (3 for pressure, 3 for temperature)
  Wire2.requestFrom(BMP390_ADDR, 6);
  if (Wire2.available() == 6) {
    int32_t rawPress = (Wire2.read() << 16) | (Wire2.read() << 8) | Wire2.read();
    int32_t rawTemp = (Wire2.read() << 16) | (Wire2.read() << 8) | Wire2.read();

    // Display raw data to check if values are dynamic
    Serial.print("Pressure (raw): ");
    Serial.print(rawPress);
    Serial.print(", Temperature (raw): ");
    Serial.println(rawTemp);
  } else {
    Serial.println("Error: Not enough data available for reading.");
  }
}

void setup() {
  Serial.begin(115200);
  Wire2.begin();  // Initialize I²C on Wire2 (SDA2/SCL2)
  delay(1000);

  initBMP390();  // Initialize BMP390 for continuous measurement
  readCalibrationData();  // Read and store calibration coefficients
}

void loop() {
  readBMP390();  // Read and display raw data
  delay(100);    // Delay for next reading
}
