// Consolidated I2C Scanner, Sensor Read, CSV Output
// Teensy 4.1 with BMP581 sensors behind TCA9548A muxes and dual actuators.

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "SparkFun_BMP581_Arduino_Library.h"
#include <Audio.h>

// I²C mux and sensor addresses
#define TCA9548A_ADDR_1  0x70
#define TCA9548A_ADDR_2  0x71
#define TCA9548A_ADDR_3  0x70
#define TCA9548A_ADDR_4  0x71
#define BMP_ADDR         0x47

// Create ADXL345 object
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Position Sensor settings
float init_voltage;  
const int positionSensorPin = A4; // Pin where linear position meter is connected, on board no 24
const float maxDistance = 5.0; // Maximum travel distance in mm (arbritary set on a scale of 0-100)
const float voltValue = 3.3; // Volts, conect yellow wire to GND and green wire to the voltage pin
float oldvoltage = 0.0;

// Four banks of eight BMP581 sensors each
BMP581 sensors1a[8];  // Wire1 @0x70
BMP581 sensors1b[8];  // Wire1 @0x71
BMP581 sensors2a[8];  // Wire2 @0x70
BMP581 sensors2b[8];  // Wire2 @0x71

// Arrays for generic looping
TwoWire* const buses[]     = { &Wire1, &Wire1, &Wire2, &Wire2 };
BMP581*    const banks[]    = { sensors1a,  sensors1b,  sensors2a,  sensors2b  };
uint8_t    const muxAddrs[] = { TCA9548A_ADDR_1, TCA9548A_ADDR_2,
                                TCA9548A_ADDR_3, TCA9548A_ADDR_4 };
const uint8_t NUM_BANKS    = 4;
const uint8_t CH_PER_BANK  = 8;

// Forward declarations
bool    tcaselect(TwoWire &bus, uint8_t muxAddr, uint8_t ch);
void    initSensors(TwoWire &bus, BMP581 sensors[], uint8_t muxAddr);
void    resetBMP585(TwoWire &bus, uint8_t addr);

// Auditory sensors
AudioInputI2S2      i2s2;
AudioAnalyzePeak    peakL;
AudioConnection     cordL(i2s2, 0, peakL, 0);
float emaLevel = 0.0f;
const float alpha = 0.2f;

void setup() {
    Serial.begin(115200);
    while (!Serial);

    // Initialize accelerometer
    if (!accel.begin()) {
    Serial.println("ADXL345 not detected. Check wiring.");
    while (1);
    }

    accel.setRange(ADXL345_RANGE_2_G); // Optional: 2G, 4G, 8G, or 16G
    Serial.println("ADXL345 ready. Reading acceleration...");

    // Initialize position sensor
    pinMode(positionSensorPin, INPUT);  // Set the sensor pin as input
    Serial.println("Positionmeter initialised");
    delay(10);

    // Calibration voltage
    int sensorValue = analogRead(positionSensorPin);
    // Convert the sensor value to voltage
    init_voltage = sensorValue* (voltValue/ 1023.0);

    // Initialize I2C busses
    Wire2.begin();

    // Initialize all four sensor banks
    initSensors(Wire1, sensors1a, TCA9548A_ADDR_1);
    initSensors(Wire1, sensors1b, TCA9548A_ADDR_2);
    initSensors(Wire2, sensors2a, TCA9548A_ADDR_3);
    initSensors(Wire2, sensors2b, TCA9548A_ADDR_4);

    // Auditory sensors
    AudioMemory(24);

    Serial.println("Setup complete.");
}

void loop() {
    // Auditory
    if (peakL.available()) {
        float inst = peakL.read(); // 0.0~1.0
        emaLevel = alpha * inst + (1 - alpha) * emaLevel; // EMA
    }

    // Totals for each bank
    float totals[NUM_BANKS] = { 0 };

    // Build one CSV line of 32 readings
    String outputLine;
    outputLine.reserve(NUM_BANKS * CH_PER_BANK * 8);

    // Read all banks & channels
    for (uint8_t bank = 0; bank < NUM_BANKS; bank++) {
        TwoWire &bus    = *buses[bank];
        BMP581* sensors = banks[bank];
        uint8_t muxAddr = muxAddrs[bank];

        for (uint8_t ch = 0; ch < CH_PER_BANK; ch++) {
            if (!tcaselect(bus, muxAddr, ch)) {
                outputLine += "ERR";
            } else {
                delay(3);
                bmp5_sensor_data d;
                if (sensors[ch].getSensorData(&d) == BMP5_OK) {
                    totals[bank] += d.pressure;
                    outputLine += String(d.pressure, 1);
                } else {
                    outputLine += "ERR";
                }
            }
            // comma except after the very last value
            if (!(bank == NUM_BANKS - 1 && ch == CH_PER_BANK - 1)) outputLine += ',';
        }
        // deselect this mux
        bus.beginTransmission(muxAddr);
        bus.write(0x00);
        bus.endTransmission();
    }

    //Accelerometer logic
    sensors_event_t event;
    accel.getEvent(&event);

    float acc_x = (event.acceleration.x); 
    float acc_y = (event.acceleration.y); 
    float acc_z = (event.acceleration.z);

    //Position sensor logic
    // Read the analog value from the sensor
    int sensorValue = analogRead(positionSensorPin);

    // Convert the sensor value to voltage
    float voltage = sensorValue* (voltValue/ 1023.0);

    // Map the voltage to distance (0V -> 0mm, voltValue -> maxDistance)
    //float distance = map(voltage, 3.23, voltValue, 0, maxDistance);
    float distance = (voltage - init_voltage) * (4.0 / (3.252-init_voltage));
    
    oldvoltage = voltage;

    outputLine += "," + String(acc_x, 3);
    outputLine += "," + String(acc_y, 3);
    outputLine += "," + String(acc_z, 3);
    outputLine += "," + String(distance, 2);
    outputLine += "," + String(emaLevel, 3);

    // Print CSV line of 32 values plus accel and pos
    Serial.println(outputLine);
    delay(5);
}


//--------------------------------------------------------------------------------
// Select a channel on the TCA9548A and verify
//--------------------------------------------------------------------------------
// bool tcaselect(TwoWire &bus, uint8_t muxAddr, uint8_t ch) {
//     if (ch > 7) return false;
//     bus.beginTransmission(muxAddr);
//     bus.write(1 << ch);
//     bus.endTransmission();
//     delay(2);
//     bus.requestFrom(muxAddr, (uint8_t)1);
//     return bus.available() && (bus.read() == (1 << ch));
// }

//--------------------------------------------------------------------------------
// Soft-reset BMP581
//--------------------------------------------------------------------------------
void resetBMP585(TwoWire &bus, uint8_t addr) {
    bus.beginTransmission(addr);
    bus.write(0x7E);
    bus.write(0xB6);
    bus.endTransmission();
    delay(2);
}

// //--------------------------------------------------------------------------------
// // Initialize a bank of 8 BMP581 sensors behind one mux
// //--------------------------------------------------------------------------------
// void initSensors(TwoWire &bus, BMP581 sensors[], uint8_t muxAddr) {
//     bus.setClock(50000);
//     for (uint8_t ch = 0; ch < CH_PER_BANK; ch++) {
//         while (!tcaselect(bus, muxAddr, ch)) delay(100);
//         resetBMP585(bus, BMP_ADDR);
//         delay(50);
//         while (sensors[ch].beginI2C(BMP_ADDR, bus) != BMP5_OK) {
//             delay(200);
//         }
//     }
//     // deselect all channels
//     bus.beginTransmission(muxAddr);
//     bus.write(0x00);
//     bus.endTransmission();
// }

//--------------------------------------------------------------------------------
// Select a channel on the TCA9548A
//--------------------------------------------------------------------------------
bool tcaselect(TwoWire &bus, uint8_t muxAddr, uint8_t ch) {
    if (ch > 7) return false;
    bus.beginTransmission(muxAddr);
    bus.write(1 << ch);
    return (bus.endTransmission() == 0); // 只判断是否成功写入
}

//--------------------------------------------------------------------------------
// Initialize a bank of 8 BMP581 sensors behind one mux
//--------------------------------------------------------------------------------
void initSensors(TwoWire &bus, BMP581 sensors[], uint8_t muxAddr) {
    bus.setClock(100000); // 可以先用100kHz，稳定性和速度折中

    for (uint8_t ch = 0; ch < CH_PER_BANK; ch++) {
        bool ok = false;

        // 尝试选择通道
        for (int attempt = 0; attempt < 3; attempt++) {
            if (tcaselect(bus, muxAddr, ch)) {
                ok = true;
                break;
            }
            delay(50);
        }
        if (!ok) {
            Serial.print("Mux select failed at addr 0x");
            Serial.print(muxAddr, HEX);
            Serial.print(" ch ");
            Serial.println(ch);
            continue; // 跳过这个通道
        }

        // 软复位
        resetBMP585(bus, BMP_ADDR);
        delay(50);

        // 尝试初始化传感器
        ok = false;
        for (int attempt = 0; attempt < 5; attempt++) {
            if (sensors[ch].beginI2C(BMP_ADDR, bus) == BMP5_OK) {
                ok = true;
                break;
            }
            delay(200);
        }
        if (!ok) {
            Serial.print("Sensor init failed at mux 0x");
            Serial.print(muxAddr, HEX);
            Serial.print(" ch ");
            Serial.println(ch);
        }
    }

    // deselect all channels
    bus.beginTransmission(muxAddr);
    bus.write(0x00);
    bus.endTransmission();
}

