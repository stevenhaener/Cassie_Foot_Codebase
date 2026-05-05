#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "SparkFun_BMP581_Arduino_Library.h"
#include <Audio.h>
#include <math.h>

// ==========================================================
// USB SERIAL FRAME FORMAT
// Header:
//   0xA5 0x5A TYPE LEN_L LEN_H PAYLOAD...
//
// TYPE:
//   0x01 = AUDIO binary frame, 256 bytes = 128 int16 samples
//   0x02 = SENSOR CSV text frame
// ==========================================================

const uint8_t FRAME_H1 = 0xA5;
const uint8_t FRAME_H2 = 0x5A;
const uint8_t TYPE_AUDIO  = 0x01;
const uint8_t TYPE_SENSOR = 0x02;

// ========================= LED =========================
const int LED_PIN = 13;

// ========================= Timing =========================
unsigned long lastSensorPacketTime = 0;

// ========================= Actuator definitions =========================
#define inst_type       0x32
#define ACTUATOR_1      Serial1
#define ACTUATOR_2      Serial1

// ========================= I2C mux and sensor addresses =========================
#define TCA9548A_ADDR_3  0x70
#define TCA9548A_ADDR_4  0x71
#define BMP_ADDR         0x47

// ========================= IMU =========================
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

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
// Teensy Audio Library: 44.1 kHz, 128 samples per block
AudioInputI2S2       i2s2;
AudioRecordQueue     audioQueue;
AudioConnection      cordL(i2s2, 0, audioQueue, 0);

const int AUDIO_BLOCKS_PER_SENSOR_PACKET = 4;
const int AUDIO_SAMPLES_PER_BLOCK = 128;
const int AUDIO_BYTES_PER_BLOCK = AUDIO_SAMPLES_PER_BLOCK * 2;
const int AUDIO_SAMPLES_PER_SENSOR_PACKET =
    AUDIO_BLOCKS_PER_SENSOR_PACKET * AUDIO_SAMPLES_PER_BLOCK;

int audioBlockCounter = 0;
float audioSumSquares = 0.0f;
float audioPeak = 0.0f;
float audioRMS = 0.0f;

// ========================= Thresholds =========================
const float THRESHOLD_ARRAY1 = 100000.0 - 1200;
const float THRESHOLD_ARRAY2 = 103000.0 - 1200;

const float IMU_THRESHOLD_X = 0.5;
const float IMU_THRESHOLD_Y = 0.5;
const float IMU_THRESHOLD_Z = 0.5;
const float IMU_THRESHOLD_MAG = 1.0;

const float AUDIO_THRESHOLD = 0.2;

// ========================= Enable switches =========================
const bool USE_TACTILE_CONTROL = false;
const bool USE_IMU_CONTROL     = true;
const bool USE_AUDIO_CONTROL   = false;
const bool USE_IMU_MAGNITUDE   = true;

// ========================= Position control settings =========================
const uint16_t ACTUATOR_MIN_RAW = 0;
const uint16_t ACTUATOR_MAX_RAW = 2000;

const uint16_t PUSH_POSITION_RAW    = 800;
const uint16_t RETRACT_POSITION_RAW = 200;

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

void sendFrame(uint8_t type, const uint8_t* payload, uint16_t length);
void sendAudioBlock(int16_t* buffer);
void updateAudioFeatures(int16_t* buffer);
void sendSensorPacket();

// ==========================================================
// SETUP
// ==========================================================
void setup() {
    Serial.begin(2000000);
    while (!Serial) {}

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    if (!accel.begin()) {
        while (1) {}
    }

    accel.setRange(ADXL345_RANGE_2_G);
    calibrateIMU();

    pinMode(positionSensorPin, INPUT);
    delay(10);

    int sensorValue = analogRead(positionSensorPin);
    init_voltage = sensorValue * (voltValue / 1023.0);

    Wire2.begin();
    initSensors(Wire2, sensors2a, TCA9548A_ADDR_3);
    initSensors(Wire2, sensors2b, TCA9548A_ADDR_4);

    AudioMemory(64);
    audioQueue.begin();

    ACTUATOR_1.begin(921600);
    ACTUATOR_2.begin(921600);

    delay(100);
    clearActuatorFaults();
    delay(100);

    setPositionMode();
    delay(100);

    sendRetractPosition();
    actuatorState = ACT_STATE_RETRACT;
    digitalWrite(LED_PIN, LOW);

    lastSensorPacketTime = micros();

    String header =
        "time_ms,"
        "BMP1_0,BMP1_1,BMP1_2,BMP1_3,BMP1_4,BMP1_5,BMP1_6,BMP1_7,"
        "BMP2_0,BMP2_1,BMP2_2,BMP2_3,BMP2_4,BMP2_5,BMP2_6,BMP2_7,"
        "acc_x,acc_y,acc_z,distance,audio_peak,audio_rms,"
        "sensor_packet_hz,estimated_audio_sample_rate";

    sendFrame(TYPE_SENSOR, (const uint8_t*)header.c_str(), header.length());
}

// ==========================================================
// LOOP
// ==========================================================
void loop() {
    while (audioQueue.available() > 0) {
        int16_t* buffer = audioQueue.readBuffer();

        // 1. Send raw audio to laptop at 44.1 kHz
        sendAudioBlock(buffer);

        // 2. Accumulate audio features
        updateAudioFeatures(buffer);

        audioQueue.freeBuffer();

        audioBlockCounter++;

        // 3. Every 4 audio blocks = 512 audio samples,
        // collect other sensors once and send sensor packet.
        if (audioBlockCounter >= AUDIO_BLOCKS_PER_SENSOR_PACKET) {
            audioRMS = sqrt(audioSumSquares / AUDIO_SAMPLES_PER_SENSOR_PACKET);

            sendSensorPacket();

            audioBlockCounter = 0;
            audioSumSquares = 0.0f;
            audioPeak = 0.0f;
        }
    }
}

// ==========================================================
// Frame sender
// ==========================================================
void sendFrame(uint8_t type, const uint8_t* payload, uint16_t length) {
    Serial.write(FRAME_H1);
    Serial.write(FRAME_H2);
    Serial.write(type);
    Serial.write((uint8_t)(length & 0xFF));
    Serial.write((uint8_t)((length >> 8) & 0xFF));
    Serial.write(payload, length);
}

// ==========================================================
// Send raw audio block
// 128 samples × 2 bytes = 256 bytes
// ==========================================================
void sendAudioBlock(int16_t* buffer) {
    sendFrame(TYPE_AUDIO, (const uint8_t*)buffer, AUDIO_BYTES_PER_BLOCK);
}

// ==========================================================
// Audio feature accumulation
// ==========================================================
void updateAudioFeatures(int16_t* buffer) {
    for (int i = 0; i < AUDIO_SAMPLES_PER_BLOCK; i++) {
        float x = buffer[i] / 32768.0f;
        float ax = fabs(x);

        if (ax > audioPeak) {
            audioPeak = ax;
        }

        audioSumSquares += x * x;
    }
}

// ==========================================================
// Sensor packet at ~86 Hz
// ==========================================================
void sendSensorPacket() {
    float totals[NUM_BANKS] = {0};
    String outputLine;
    outputLine.reserve(600);

    outputLine += String(millis());

    // ========================= Tactile =========================
    for (uint8_t bank = 0; bank < NUM_BANKS; bank++) {
        TwoWire &bus = *buses[bank];
        BMP581* sensors = banks[bank];
        uint8_t muxAddr = muxAddrs[bank];

        for (uint8_t ch = 0; ch < CH_PER_BANK; ch++) {
            outputLine += ",";

            if (!tcaselect(bus, muxAddr, ch)) {
                outputLine += "ERR";
            } else {
                bmp5_sensor_data d;

                if (sensors[ch].getSensorData(&d) == BMP5_OK) {
                    totals[bank] += d.pressure;
                    outputLine += String(d.pressure, 1);
                } else {
                    outputLine += "ERR";
                }
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

    // ========================= Control conditions =========================
    bool tactileCondition = true;
    if (USE_TACTILE_CONTROL) {
        tactileCondition =
            (array1Avg > THRESHOLD_ARRAY1) &&
            (array2Avg > THRESHOLD_ARRAY2);
    }

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

    bool audioCondition = true;
    if (USE_AUDIO_CONTROL) {
        audioCondition = (audioRMS > AUDIO_THRESHOLD);
    }

    // ========================= Actuator logic =========================
    bool pushCondition = tactileCondition && imuCondition && audioCondition;

    if (pushCondition) {
        if (actuatorState != ACT_STATE_PUSH) {
            sendPushPosition();
            actuatorState = ACT_STATE_PUSH;
        }
        digitalWrite(LED_PIN, HIGH);
    } else {
        if (actuatorState != ACT_STATE_RETRACT) {
            sendRetractPosition();
            actuatorState = ACT_STATE_RETRACT;
        }
        digitalWrite(LED_PIN, LOW);
    }

    // ========================= Frequency measurement =========================
    unsigned long now = micros();
    float dt = (now - lastSensorPacketTime) / 1000000.0;
    float sensorPacketHz = 1.0 / dt;
    lastSensorPacketTime = now;

    float estimatedAudioSampleRate =
        sensorPacketHz * AUDIO_SAMPLES_PER_SENSOR_PACKET;

    // ========================= Add sensor values =========================
    outputLine += "," + String(acc_x, 3);
    outputLine += "," + String(acc_y, 3);
    outputLine += "," + String(acc_z, 3);
    outputLine += "," + String(distance, 2);
    outputLine += "," + String(audioPeak, 6);
    outputLine += "," + String(audioRMS, 6);
    outputLine += "," + String(sensorPacketHz, 2);
    outputLine += "," + String(estimatedAudioSampleRate, 1);

    sendFrame(TYPE_SENSOR, (const uint8_t*)outputLine.c_str(), outputLine.length());
}

// ==========================================================
// IMU Calibration Function
// ==========================================================
void calibrateIMU() {
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
}

// ==========================================================
// Actuator checksum
// ==========================================================
uint8_t calculateChecksum(uint8_t *data, int length) {
    uint16_t sum = 0;
    for (int i = 2; i < length; i++) {
        sum += data[i];
    }
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
        0x55, 0xAA, 0x05, 0xFF,
        inst_type, 0x18, 0x00, 0x01, 0x00
    };
    sendCommandWithChecksum(cmd, sizeof(cmd));
}

void setPositionMode() {
    uint8_t mode[] = {
        0x55, 0xAA, 0x05, 0xFF,
        inst_type, 0x25, 0x00, 0x00, 0x00
    };
    sendCommandWithChecksum(mode, sizeof(mode));
}

void sendPositionTargetRaw(uint16_t targetRaw) {
    if (targetRaw < ACTUATOR_MIN_RAW) {
        targetRaw = ACTUATOR_MIN_RAW;
    }

    if (targetRaw > ACTUATOR_MAX_RAW) {
        targetRaw = ACTUATOR_MAX_RAW;
    }

    uint8_t pos[] = {
        0x55, 0xAA, 0x05, 0xFF,
        inst_type, 0x29, 0x00,
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
    bus.setClock(400000);

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