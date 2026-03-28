// Consolidated I2C Scanner, Sensor Read, CSV Output, and Actuator Control
// Teensy 4.1 with BMP581 sensors behind TCA9548A muxes, IMU, audio, and dual actuators.
// Uses ONLY the back 16 tactile sensors (2 arrays x 8 sensors).
//
// Logic:
//   Start PUSHING when BOTH array averages exceed PUSH thresholds.
//   Stay PUSHING until EITHER array average drops below RETRACT threshold.
//   Otherwise stay in the current mode.
//
// This avoids getting stuck/chattering when force is released early.

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "SparkFun_BMP581_Arduino_Library.h"
#include <Audio.h>

// ========================= Actuator definitions =========================
#define start_byte1     0x55
#define start_byte2     0xAA
#define inst_type       0x32

// If actuators are on different UARTs, change ACTUATOR_2 to Serial2
#define ACTUATOR_1      Serial1
#define ACTUATOR_2      Serial1

// ========================= I2C mux and sensor addresses =========================
#define TCA9548A_ADDR_3  0x70
#define TCA9548A_ADDR_4  0x71
#define BMP_ADDR         0x47

// ========================= IMU =========================
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// ========================= Position sensor =========================
float init_voltage;
const int positionSensorPin = A4;
const float voltValue = 3.3;
float oldvoltage = 0.0;

// ========================= Tactile sensors =========================
BMP581 sensors2a[8];   // Wire2 @ 0x70
BMP581 sensors2b[8];   // Wire2 @ 0x71

TwoWire* const buses[]      = { &Wire2, &Wire2 };
BMP581*    const banks[]    = { sensors2a, sensors2b };
uint8_t    const muxAddrs[] = { TCA9548A_ADDR_3, TCA9548A_ADDR_4 };

const uint8_t NUM_BANKS   = 2;
const uint8_t CH_PER_BANK = 8;

// ========================= Audio sensor =========================
AudioInputI2S2      i2s2;
AudioAnalyzePeak    peakL;
AudioConnection     cordL(i2s2, 0, peakL, 0);

float emaLevel = 0.0f;
const float alpha = 0.2f;

// ========================= Tunable thresholds =========================
// PUSH starts only if BOTH averages are above these
const float PUSH_THRESHOLD_ARRAY1 = 99000.0;
const float PUSH_THRESHOLD_ARRAY2 = 103000.0;

// Once already pushing, retract only if EITHER average drops below these
// Set these a bit LOWER than the push thresholds
const float RETRACT_THRESHOLD_ARRAY1 = 98900.0;
const float RETRACT_THRESHOLD_ARRAY2 = 102800.0;

// ========================= Actuator mode =========================
bool pushingNow = false;
unsigned long lastCmdSend = 0;
const unsigned long CMD_REFRESH_MS = 100;

// ========================= Forward declarations =========================
bool    tcaselect(TwoWire &bus, uint8_t muxAddr, uint8_t ch);
void    initSensors(TwoWire &bus, BMP581 sensors[], uint8_t muxAddr);
void    resetBMP585(TwoWire &bus, uint8_t addr);

uint8_t calculateChecksum(uint8_t *data, int length);
void    sendCommandWithChecksum(uint8_t* command, uint8_t length);
void    clearActuatorFaults();
void    sendForceMode();
void    sendRetractMode();

void setup() {
    Serial.begin(115200);
    while (!Serial);

    if (!accel.begin()) {
        Serial.println("ADXL345 not detected. Check wiring.");
        while (1);
    }
    accel.setRange(ADXL345_RANGE_2_G);
    Serial.println("ADXL345 ready.");

    pinMode(positionSensorPin, INPUT);
    delay(10);
    int sensorValue = analogRead(positionSensorPin);
    init_voltage = sensorValue * (voltValue / 1023.0);

    Wire2.begin();
    initSensors(Wire2, sensors2a, TCA9548A_ADDR_3);
    initSensors(Wire2, sensors2b, TCA9548A_ADDR_4);

    AudioMemory(24);

    ACTUATOR_1.begin(921600);
    ACTUATOR_2.begin(921600);
    delay(100);

    clearActuatorFaults();
    delay(100);

    sendRetractMode();
    pushingNow = false;
    lastCmdSend = millis();

    Serial.println("Setup complete.");
}

void loop() {
    // ========================= Audio =========================
    if (peakL.available()) {
        float inst = peakL.read();
        emaLevel = alpha * inst + (1.0f - alpha) * emaLevel;
    }

    // ========================= Tactile read =========================
    float totals[NUM_BANKS] = { 0 };
    String outputLine;
    outputLine.reserve(NUM_BANKS * CH_PER_BANK * 8 + 180);

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

            if (!(bank == NUM_BANKS - 1 && ch == CH_PER_BANK - 1)) {
                outputLine += ',';
            }
        }

        bus.beginTransmission(muxAddr);
        bus.write(0x00);
        bus.endTransmission();
    }

    // ========================= IMU =========================
    sensors_event_t event;
    accel.getEvent(&event);
    float acc_x = event.acceleration.x;
    float acc_y = event.acceleration.y;
    float acc_z = event.acceleration.z;

    // ========================= Position sensor =========================
    int sensorValue = analogRead(positionSensorPin);
    float voltage = sensorValue * (voltValue / 1023.0);
    float distance = (voltage - init_voltage) * (4.0 / (3.252 - init_voltage));
    oldvoltage = voltage;

    // ========================= Compute array averages =========================
    float avgs[NUM_BANKS];
    for (uint8_t i = 0; i < NUM_BANKS; i++) {
        avgs[i] = totals[i] / CH_PER_BANK;
    }

    float array1Avg = avgs[0];
    float array2Avg = avgs[1];

    // ========================= Hysteresis control =========================
    bool startPush = (array1Avg > PUSH_THRESHOLD_ARRAY1) &&
                     (array2Avg > PUSH_THRESHOLD_ARRAY2);

    bool stopPush  = (array1Avg < RETRACT_THRESHOLD_ARRAY1) ||
                     (array2Avg < RETRACT_THRESHOLD_ARRAY2);

    if (!pushingNow) {
        if (startPush) {
            pushingNow = true;
            sendForceMode();
            lastCmdSend = millis();
        } else if (millis() - lastCmdSend >= CMD_REFRESH_MS) {
            sendRetractMode();
            lastCmdSend = millis();
        }
    } else {
        if (stopPush) {
            pushingNow = false;
            sendRetractMode();
            lastCmdSend = millis();
        } else if (millis() - lastCmdSend >= CMD_REFRESH_MS) {
            sendForceMode();
            lastCmdSend = millis();
        }
    }

    // ========================= CSV output =========================
    outputLine += "," + String(acc_x, 3);
    outputLine += "," + String(acc_y, 3);
    outputLine += "," + String(acc_z, 3);
    outputLine += "," + String(distance, 2);
    outputLine += "," + String(emaLevel, 3);
    outputLine += "," + String(array1Avg, 1);
    outputLine += "," + String(array2Avg, 1);
    outputLine += "," + String(PUSH_THRESHOLD_ARRAY1, 1);
    outputLine += "," + String(PUSH_THRESHOLD_ARRAY2, 1);
    outputLine += "," + String(RETRACT_THRESHOLD_ARRAY1, 1);
    outputLine += "," + String(RETRACT_THRESHOLD_ARRAY2, 1);
    outputLine += "," + String(pushingNow ? 1 : 0);

    Serial.println(outputLine);
    delay(5);
}

// --------------------------------------------------------------------------------
// Actuator checksum
// --------------------------------------------------------------------------------
uint8_t calculateChecksum(uint8_t *data, int length) {
    uint16_t sum = 0;
    for (int i = 2; i < length; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

// --------------------------------------------------------------------------------
// Send arbitrary actuator command + checksum
// --------------------------------------------------------------------------------
void sendCommandWithChecksum(uint8_t* command, uint8_t length) {
    uint8_t csum = calculateChecksum(command, length);

    ACTUATOR_1.write(command, length);
    ACTUATOR_1.write(csum);

    ACTUATOR_2.write(command, length);
    ACTUATOR_2.write(csum);
}

// --------------------------------------------------------------------------------
// Clear actuator faults
// --------------------------------------------------------------------------------
void clearActuatorFaults() {
    static uint8_t faultclr[] = {
        0x55, 0xAA, 0x05, 0xFF, inst_type, 0x18, 0x00, 0x01, 0x00, 0x4F
    };
    sendCommandWithChecksum(faultclr, sizeof(faultclr));
}

// --------------------------------------------------------------------------------
// Extend actuator using force mode
// --------------------------------------------------------------------------------
void sendForceMode() {
    static uint8_t forcemode[] = {
        0x55, 0xAA, 0x05, 0xFF, inst_type, 0x25, 0x00, 0x03, 0x00, 0x5E
    };

    static uint8_t force[] = {
        0x55, 0xAA, 0x05, 0xFF, inst_type, 0x27, 0x00, 0xD0, 0x07, 0x34
    };

    sendCommandWithChecksum(forcemode, sizeof(forcemode));
    delay(5);
    sendCommandWithChecksum(force, sizeof(force));
}

// --------------------------------------------------------------------------------
// Retract actuator using position mode
// --------------------------------------------------------------------------------
void sendRetractMode() {
    static uint8_t posmode[] = {
        0x55, 0xAA, 0x05, 0xFF, inst_type, 0x25, 0x00, 0x00, 0x00, 0x5B
    };

    static uint8_t fullretract[] = {
        0x55, 0xAA, 0x05, 0xFF, inst_type, 0x29, 0x00, 0x00, 0x00, 0x5F
    };

    sendCommandWithChecksum(posmode, sizeof(posmode));
    delay(5);
    sendCommandWithChecksum(fullretract, sizeof(fullretract));
}

// --------------------------------------------------------------------------------
// Soft-reset BMP581
// --------------------------------------------------------------------------------
void resetBMP585(TwoWire &bus, uint8_t addr) {
    bus.beginTransmission(addr);
    bus.write(0x7E);
    bus.write(0xB6);
    bus.endTransmission();
    delay(2);
}

// --------------------------------------------------------------------------------
// Select a channel on the TCA9548A
// --------------------------------------------------------------------------------
bool tcaselect(TwoWire &bus, uint8_t muxAddr, uint8_t ch) {
    if (ch > 7) return false;
    bus.beginTransmission(muxAddr);
    bus.write(1 << ch);
    return (bus.endTransmission() == 0);
}

// --------------------------------------------------------------------------------
// Initialize a bank of 8 BMP581 sensors behind one mux
// --------------------------------------------------------------------------------
void initSensors(TwoWire &bus, BMP581 sensors[], uint8_t muxAddr) {
    bus.setClock(100000);

    for (uint8_t ch = 0; ch < CH_PER_BANK; ch++) {
        bool ok = false;

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
            continue;
        }

        resetBMP585(bus, BMP_ADDR);
        delay(50);

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

    bus.beginTransmission(muxAddr);
    bus.write(0x00);
    bus.endTransmission();
}