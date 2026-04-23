// ==========================================================
// UPDATED VERSION:
// - IMU auto-calibration at startup
// - Future-ready IMU threshold logic
// - Future-ready audio threshold logic
// - Actuator changed from FORCE CONTROL to POSITION CONTROL
// - Raw actuator position range confirmed: 0 to 2000
// ==========================================================

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "SparkFun_BMP581_Arduino_Library.h"
#include <Audio.h>
#include <math.h>

// ========================= Actuator definitions =========================
#define start_byte1     0x55
#define start_byte2     0xAA
#define inst_type       0x32

#define ACTUATOR_1      Serial1
#define ACTUATOR_2      Serial1

// ========================= I2C mux and sensor addresses =========================
#define TCA9548A_ADDR_3  0x70
#define TCA9548A_ADDR_4  0x71
#define BMP_ADDR         0x47

// ========================= IMU =========================
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// ---- IMU calibration offsets ----
float imu_bias_x = 0.0;
float imu_bias_y = 0.0;
float imu_bias_z = 0.0;

// ========================= Position sensor =========================
float init_voltage;
const int positionSensorPin = A4;
const float voltValue = 3.3;

// ========================= Tactile sensors =========================
BMP581 sensors2a[8];
BMP581 sensors2b[8];

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

// ========================= Thresholds =========================
// ---- Tactile thresholds ----
const float THRESHOLD_ARRAY1 = 100000.0 - 1200;
const float THRESHOLD_ARRAY2 = 103000.0 - 1200;

// ---- IMU thresholds ----
const float IMU_THRESHOLD_X = 0.5;   // m/s^2
const float IMU_THRESHOLD_Y = 0.5;   // m/s^2
const float IMU_THRESHOLD_Z = 0.5;   // m/s^2

// Optional magnitude threshold
const float IMU_THRESHOLD_MAG = 1.0; // m/s^2

// ---- Audio threshold ----
const float AUDIO_THRESHOLD = 0.2;   // peak/EMA threshold

// ========================= Enable switches =========================
const bool USE_TACTILE_CONTROL = false;
const bool USE_IMU_CONTROL   = true;
const bool USE_AUDIO_CONTROL = false;

// Choose IMU logic style:
// true  -> use overall magnitude
// false -> use per-axis threshold check
const bool USE_IMU_MAGNITUDE = true;

// ========================= Position control settings =========================
// Raw range tested: 0 to 2000
const uint16_t ACTUATOR_MIN_RAW = 0;
const uint16_t ACTUATOR_MAX_RAW = 2000;

// Change these to the two positions you want
const uint16_t PUSH_POSITION_RAW    = 400;   // example push target
const uint16_t RETRACT_POSITION_RAW = 100;    // example retract target

enum ActuatorState {
    ACT_STATE_UNKNOWN = 0,
    ACT_STATE_PUSH,
    ACT_STATE_RETRACT
};

ActuatorState actuatorState = ACT_STATE_UNKNOWN;

// ========================= Function declarations =========================
bool tcaselect(TwoWire &bus, uint8_t muxAddr, uint8_t ch);
void initSensors(TwoWire &bus, BMP581 sensors[], uint8_t muxAddr);
void resetBMP585(TwoWire &bus, uint8_t addr);

uint8_t calculateChecksum(uint8_t *data, int length);
void sendCommandWithChecksum(uint8_t* command, uint8_t length);
void clearActuatorFaults();

void setPositionMode();
void sendPositionTargetRaw(uint16_t targetRaw);
void sendPushPosition();
void sendRetractPosition();

void calibrateIMU();

// ==========================================================
// SETUP
// ==========================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    if (!accel.begin()) {
        Serial.println("ADXL345 not detected.");
        while (1) {}
    }

    accel.setRange(ADXL345_RANGE_2_G);
    Serial.println("ADXL345 ready.");

    // -------- IMU Calibration --------
    calibrateIMU();

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

    // Put actuator in position mode once at startup
    setPositionMode();
    delay(100);

    // Start at retract position
    sendRetractPosition();
    actuatorState = ACT_STATE_RETRACT;

    Serial.println("Setup complete.");
}

// ==========================================================
// LOOP
// ==========================================================
void loop() {

    // ========================= Audio =========================
    if (peakL.available()) {
        float inst = peakL.read();
        emaLevel = alpha * inst + (1.0f - alpha) * emaLevel;
    }

    // ========================= Tactile =========================
    float totals[NUM_BANKS] = {0};
    String outputLine;
    outputLine.reserve(400);

    for (uint8_t bank = 0; bank < NUM_BANKS; bank++) {
        TwoWire &bus = *buses[bank];
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
                outputLine += ",";
            }
        }

        bus.beginTransmission(muxAddr);
        bus.write(0x00);
        bus.endTransmission();
    }

    // ========================= IMU =========================
    sensors_event_t event;
    accel.getEvent(&event);

    float acc_x = event.acceleration.x - imu_bias_x;
    float acc_y = event.acceleration.y - imu_bias_y;
    float acc_z = event.acceleration.z - imu_bias_z;

    float imu_mag = sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);

    // ========================= Position Sensor =========================
    int sensorValue = analogRead(positionSensorPin);
    float voltage = sensorValue * (voltValue / 1023.0);
    float distance = (voltage - init_voltage) * (4.0 / (3.252 - init_voltage));

    // ========================= Pressure averages =========================
    float array1Avg = totals[0] / CH_PER_BANK;
    float array2Avg = totals[1] / CH_PER_BANK;

    // ------------------------- Tactile condition -------------------------
    bool tactileCondition = true;
    if (USE_TACTILE_CONTROL) {
        tactileCondition =
            (array1Avg > THRESHOLD_ARRAY1) &&
            (array2Avg > THRESHOLD_ARRAY2);
    }

    // ------------------------- IMU condition -------------------------
    bool imuCondition = true;
    if (USE_IMU_CONTROL) {
        if (USE_IMU_MAGNITUDE) {
            imuCondition = (imu_mag < IMU_THRESHOLD_MAG);
        } else {
            imuCondition =
                (fabs(acc_x) > IMU_THRESHOLD_X) ||
                (fabs(acc_y) > IMU_THRESHOLD_Y) ||
                (fabs(acc_z) > IMU_THRESHOLD_Z);
        }
    }

    // ------------------------- Audio condition -------------------------
    bool audioCondition = true;
    if (USE_AUDIO_CONTROL) {
        audioCondition = (emaLevel > AUDIO_THRESHOLD);
    }

    // ========================= Final actuator logic =========================
    // All enabled conditions must be true to go to PUSH position
    bool pushCondition = tactileCondition && imuCondition && audioCondition;

    if (pushCondition) {
        if (actuatorState != ACT_STATE_PUSH) {
            sendPushPosition();
            actuatorState = ACT_STATE_PUSH;
        }
    } else {
        if (actuatorState != ACT_STATE_RETRACT) {
            sendRetractPosition();
            actuatorState = ACT_STATE_RETRACT;
        }
    }

    // ========================= CSV Output =========================
    outputLine += "," + String(acc_x, 3);
    outputLine += "," + String(acc_y, 3);
    outputLine += "," + String(acc_z, 3);
    outputLine += "," + String(distance, 2);
    outputLine += "," + String(emaLevel, 3);
    //outputLine += "," + String(pushCondition ? 1 : 0);
    //outputLine += "," + String(actuatorState == ACT_STATE_PUSH ? 1 : 0);

    Serial.println(outputLine);

    delay(5);
}

// ==========================================================
// IMU Calibration Function
// Keep sensor still during startup
// ==========================================================
void calibrateIMU() {
    Serial.println("Calibrating IMU... Keep still.");

    const int samples = 300;
    sensors_event_t event;

    float sumX = 0, sumY = 0, sumZ = 0;

    for (int i = 0; i < samples; i++) {
        accel.getEvent(&event);

        sumX += event.acceleration.x;
        sumY += event.acceleration.y;
        sumZ += event.acceleration.z;

        delay(5);
    }

    imu_bias_x = sumX / samples;
    imu_bias_y = sumY / samples;
    imu_bias_z = sumZ / samples;

    Serial.println("IMU calibration done.");
}

// ==========================================================
// Actuator checksum
// ==========================================================
uint8_t calculateChecksum(uint8_t *data, int length) {
    uint16_t sum = 0;
    for (int i = 2; i < length; i++) sum += data[i];
    return (uint8_t)(sum & 0xFF);
}

void sendCommandWithChecksum(uint8_t* command, uint8_t length) {
    uint8_t csum = calculateChecksum(command, length);

    ACTUATOR_1.write(command, length);
    ACTUATOR_1.write(csum);

    ACTUATOR_2.write(command, length);
    ACTUATOR_2.write(csum);
}

void clearActuatorFaults() {
    uint8_t cmd[] = {
        0x55,0xAA,0x05,0xFF,inst_type,0x18,0x00,0x01,0x00
    };
    sendCommandWithChecksum(cmd, sizeof(cmd));
}

void setPositionMode() {
    uint8_t mode[] = {
        0x55,0xAA,0x05,0xFF,inst_type,0x25,0x00,0x00,0x00
    };
    sendCommandWithChecksum(mode, sizeof(mode));
}

void sendPositionTargetRaw(uint16_t targetRaw) {
    if (targetRaw < ACTUATOR_MIN_RAW) targetRaw = ACTUATOR_MIN_RAW;
    if (targetRaw > ACTUATOR_MAX_RAW) targetRaw = ACTUATOR_MAX_RAW;

    uint8_t pos[] = {
        0x55,0xAA,0x05,0xFF,inst_type,0x29,0x00,
        (uint8_t)(targetRaw & 0xFF),
        (uint8_t)((targetRaw >> 8) & 0xFF)
    };

    sendCommandWithChecksum(pos, sizeof(pos));
}

void sendPushPosition() {
    sendPositionTargetRaw(PUSH_POSITION_RAW);
}

void sendRetractPosition() {
    sendPositionTargetRaw(RETRACT_POSITION_RAW);
}

// ==========================================================
// BMP Sensor Functions
// ==========================================================
void resetBMP585(TwoWire &bus, uint8_t addr) {
    bus.beginTransmission(addr);
    bus.write(0x7E);
    bus.write(0xB6);
    bus.endTransmission();
    delay(2);
}

bool tcaselect(TwoWire &bus, uint8_t muxAddr, uint8_t ch) {
    if (ch > 7) return false;
    bus.beginTransmission(muxAddr);
    bus.write(1 << ch);
    return (bus.endTransmission() == 0);
}

void initSensors(TwoWire &bus, BMP581 sensors[], uint8_t muxAddr) {
    bus.setClock(100000);

    for (uint8_t ch = 0; ch < CH_PER_BANK; ch++) {
        if (!tcaselect(bus, muxAddr, ch)) continue;

        resetBMP585(bus, BMP_ADDR);
        delay(50);

        sensors[ch].beginI2C(BMP_ADDR, bus);
        delay(100);
    }

    bus.beginTransmission(muxAddr);
    bus.write(0x00);
    bus.endTransmission();
}