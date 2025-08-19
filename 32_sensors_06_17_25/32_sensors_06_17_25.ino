// Consolidated I2C Scanner, Sensor Read, CSV Output, and Actuator Control
// Teensy 4.1 with BMP581 sensors behind TCA9548A muxes and dual actuators.

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "SparkFun_BMP581_Arduino_Library.h"

// Actuator and protocol definitions
#define start_byte1     0x55
#define start_byte2     0xAA
#define inst_type       0x32
#define ACTUATOR_1      Serial1
#define ACTUATOR_2      Serial1

// I²C mux and sensor addresses
#define TCA9548A_ADDR    0x70
#define TCA9548A_ADDR_2  0x71
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
TwoWire* const buses[]     = { &Wire1,     &Wire1,     &Wire2,     &Wire2     };
BMP581*    const banks[]    = { sensors1a,  sensors1b,  sensors2a,  sensors2b  };
uint8_t    const muxAddrs[] = { TCA9548A_ADDR, TCA9548A_ADDR_2,
                                TCA9548A_ADDR, TCA9548A_ADDR_2 };
const uint8_t NUM_BANKS    = 4;
const uint8_t CH_PER_BANK  = 8;

// Forward declarations
uint8_t calculateChecksum(uint8_t *data, int length);
bool    tcaselect(TwoWire &bus, uint8_t muxAddr, uint8_t ch);
void    initSensors(TwoWire &bus, BMP581 sensors[], uint8_t muxAddr);
void    sendCommandWithChecksum(uint8_t* command, uint8_t length);
void    sendForceMode();
void    sendRetractMode();
void    resetBMP585(TwoWire &bus, uint8_t addr);

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

    // Initialize I2C busses and actuators
    Wire1.begin();
    Wire2.begin();
    ACTUATOR_1.begin(921600);
    ACTUATOR_2.begin(921600);

    // Initialize all four sensor banks
    initSensors(Wire1, sensors1a, TCA9548A_ADDR);
    initSensors(Wire1, sensors1b, TCA9548A_ADDR_2);
    initSensors(Wire2, sensors2a, TCA9548A_ADDR);
    initSensors(Wire2, sensors2b, TCA9548A_ADDR_2);

    // Clear actuator faults
    uint8_t faultclr[] = { 0x55,0xAA,0x05,0xFF, inst_type,0x18,0x00,0x01, 0x00,0x4F };
    ACTUATOR_1.write(faultclr, sizeof(faultclr));
    ACTUATOR_2.write(faultclr, sizeof(faultclr));
    delay(100);

    Serial.println("Setup complete.");
}

void loop() {
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
    Serial.print(distance, 2);  // Print distance with 1 decimal place
    
    oldvoltage = voltage;

    outputLine += "," + String(acc_x, 3);
    outputLine += "," + String(acc_y, 3);
    outputLine += "," + String(acc_z, 3);
    outputLine += "," + String(distance, 2);

    // Print CSV line of 32 values plus accel and pos
    Serial.println(outputLine);

    // Compute averages
    float avgs[NUM_BANKS];
    for (uint8_t i = 0; i < NUM_BANKS; i++) {
        avgs[i] = totals[i] / CH_PER_BANK;
    }

    // Actuation decision based on thresholds
    bool overPressure = (avgs[0] > 102000.0) || (avgs[1] > 98000.0)
                      || (avgs[2] > 105000.0) || (avgs[3] >  122000.0);

    if (overPressure) sendForceMode();
    else             sendRetractMode();

    delay(5);
}

//--------------------------------------------------------------------------------
// Calculate simple 8-bit checksum (sum of bytes[2..end])
//--------------------------------------------------------------------------------
uint8_t calculateChecksum(uint8_t *data, int length) {
    uint16_t sum = 0;
    for (int i = 2; i < length; i++) sum += data[i];
    return (uint8_t)(sum & 0xFF);
}

//--------------------------------------------------------------------------------
// Send arbitrary command + checksum to both actuators
//--------------------------------------------------------------------------------
void sendCommandWithChecksum(uint8_t* command, uint8_t length) {
    uint8_t csum = calculateChecksum(command, length);
    ACTUATOR_1.write(command, length);
    ACTUATOR_1.write(csum);
    ACTUATOR_2.write(command, length);
    ACTUATOR_2.write(csum);
}

//--------------------------------------------------------------------------------
// Select a channel on the TCA9548A and verify
//--------------------------------------------------------------------------------
bool tcaselect(TwoWire &bus, uint8_t muxAddr, uint8_t ch) {
    if (ch > 7) return false;
    bus.beginTransmission(muxAddr);
    bus.write(1 << ch);
    bus.endTransmission();
    delay(2);
    bus.requestFrom(muxAddr, (uint8_t)1);
    return bus.available() && (bus.read() == (1 << ch));
}

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

//--------------------------------------------------------------------------------
// Initialize a bank of 8 BMP581 sensors behind one mux
//--------------------------------------------------------------------------------
void initSensors(TwoWire &bus, BMP581 sensors[], uint8_t muxAddr) {
    bus.setClock(50000);
    for (uint8_t ch = 0; ch < CH_PER_BANK; ch++) {
        while (!tcaselect(bus, muxAddr, ch)) delay(100);
        resetBMP585(bus, BMP_ADDR);
        delay(50);
        while (sensors[ch].beginI2C(BMP_ADDR, bus) != BMP5_OK) {
            delay(200);
        }
    }
    // deselect all channels
    bus.beginTransmission(muxAddr);
    bus.write(0x00);
    bus.endTransmission();
}

//--------------------------------------------------------------------------------
// Send force‐mode + force commands to both actuators
//--------------------------------------------------------------------------------
void sendForceMode() {
    static const uint8_t forcemode[] = { 0x55,0xAA,0x05,0xFF, inst_type,0x25,0x00,0x03, 0x00,0x5E };
    static const uint8_t force[]     = { 0x55,0xAA,0x05,0xFF, inst_type,0x27,0x00,0xD0, 0x07,0x34 };
    sendCommandWithChecksum((uint8_t*)forcemode, sizeof(forcemode));
    delay(5);
    sendCommandWithChecksum((uint8_t*)force,     sizeof(force));
}

//--------------------------------------------------------------------------------
// Send position‐mode + full‑retract commands to both actuators
//--------------------------------------------------------------------------------
void sendRetractMode() {
    static const uint8_t posmode[]     = { 0x55,0xAA,0x05,0xFF, inst_type,0x25,0x00,0x00, 0x00,0x5B };
    static const uint8_t fullretract[] = { 0x55,0xAA,0x05,0xFF, inst_type,0x29,0x00,0x00, 0x00,0x5F };
    sendCommandWithChecksum((uint8_t*)posmode,     sizeof(posmode));
    delay(5);
    sendCommandWithChecksum((uint8_t*)fullretract, sizeof(fullretract));
}
