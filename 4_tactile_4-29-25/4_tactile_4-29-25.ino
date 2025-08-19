#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "SparkFun_BMP581_Arduino_Library.h"

// Actuator and protocol definitions
#define start_byte1     0x55
#define start_byte2     0xAA
#define response_byte1  0xAA
#define response_byte2  0x55
#define actuator1_ID    0x01
#define actuator2_ID    0x02
#define inst_type       0x32

#define ACTUATOR_1 Serial1
#define ACTUATOR_2 Serial1

// Create ADXL345 object
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Position Sensor settings
float init_voltage;  
const int positionSensorPin = A4; // Pin where linear position meter is connected, on board no 24
const float maxDistance = 5.0; // Maximum travel distance in mm (arbritary set on a scale of 0-100)
const float voltValue = 3.3; // Volts, conect yellow wire to GND and green wire to the voltage pin
float oldvoltage = 0.0;

// I²C addresses
#define TCA9548A_ADDR    0x70
#define TCA9548A_ADDR_2  0x71
#define BMP_ADDR         0x47  // BMP581 device address

// Sensor arrays for four banks (8 sensors each)
BMP581 sensors1a[8];  // Bank1a: Wire1 @0x70
BMP581 sensors1b[8];  // Bank1b: Wire1 @0x71
BMP581 sensors2a[8];  // Bank2a: Wire2 @0x70
BMP581 sensors2b[8];  // Bank2b: Wire2 @0x71

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function to calculate checksum
//////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t calculateChecksum(uint8_t *data, int length) {
    uint16_t sum = 0;
    for (int i = 2; i < length; i++) {
        sum += data[i];
    }
    return static_cast<uint8_t>(sum & 0xFF);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Send a command to actuator (with checksum)
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void sendCommandWithChecksum(uint8_t* command, uint8_t length) {
    uint8_t checksum = calculateChecksum(command, length);
    ACTUATOR_1.write(command, length);
    ACTUATOR_1.write(checksum);
    Serial.print("Actuator checksum: ");
    Serial.println(checksum);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Soft-reset BMP581
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void resetBMP585(TwoWire &bus, uint8_t addr) {
    bus.beginTransmission(addr);
    bus.write(0x7E);
    bus.write(0xB6);
    bus.endTransmission();
    delay(2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Select one channel on the given TCA9548A mux, verify switch
//////////////////////////////////////////////////////////////////////////////////////////////////////////
bool tcaselect(TwoWire &bus, uint8_t muxAddr, uint8_t ch) {
    if (ch > 7) return false;
    bus.beginTransmission(muxAddr);
    bus.write(1 << ch);
    bus.endTransmission();
    delay(2); // let the mux settle

    // read back register to verify
    bus.requestFrom(muxAddr, (uint8_t)1);
    if (bus.available()) {
      uint8_t mask = bus.read();
      return (mask == (1 << ch));
    }
    return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialize a bank of 8 BMP581 sensors behind a mux
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void initSensors(TwoWire &bus, BMP581 sensors[], uint8_t muxAddr, const char *label) {
    // Slow the bus clock in case of wiring/pull-up issues
    bus.setClock(50000);

    for (uint8_t ch = 0; ch < 8; ch++) {
        while (!tcaselect(bus, muxAddr, ch)) {
          Serial.print("Mux ");
          Serial.print(label);
          Serial.print(" failed to switch to ch ");
          Serial.println(ch);
          delay(100);
        }
        resetBMP585(bus, BMP_ADDR);
        delay(50);
        while (sensors[ch].beginI2C(BMP_ADDR, bus) != BMP5_OK) {
            Serial.print("Retrying ");
            Serial.print(label);
            Serial.print(" ch ");
            Serial.println(ch);
            delay(500);
            tcaselect(bus, muxAddr, ch);
        }
        Serial.print(label);
        Serial.print(" sensor ch ");
        Serial.println(ch);
    }

    // leave mux deselected
    bus.beginTransmission(muxAddr);
    bus.write(0x00);
    bus.endTransmission();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
    Serial.begin(9600);
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

    // Initialize only Wire1 and Wire2 (no default Wire)
    Wire1.begin();
    Wire2.begin();

    // Initialize actuator serial ports
    ACTUATOR_1.begin(921600);
    while (!ACTUATOR_1);
    ACTUATOR_2.begin(921600);
    while (!ACTUATOR_2);

    // Initialize four banks of BMP581 sensors
    initSensors(Wire1, sensors1a, TCA9548A_ADDR,   "Bank1a (Wire1 @0x70)");
    
    initSensors(Wire2, sensors2a, TCA9548A_ADDR,   "Bank2a (Wire2 @0x70)");
    initSensors(Wire2, sensors2b, TCA9548A_ADDR_2, "Bank2b (Wire2 @0x71)");
    initSensors(Wire1, sensors1b, TCA9548A_ADDR_2, "Bank1b (Wire1 @0x71)");
    

    // Clear actuator faults on both actuators
    uint8_t faultclr[] = {
        0x55, 0xAA, 0x05, 0xFF,
        inst_type, 0x18, 0x00, 0x01,
        0x00, 0x4F
    };
    Serial.println("Clearing actuator faults");
    ACTUATOR_1.write(faultclr, sizeof(faultclr));
    ACTUATOR_2.write(faultclr, sizeof(faultclr));
    delay(100);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Loop
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

    //Accelerometer logic
    sensors_event_t event;
    accel.getEvent(&event);

    Serial.print("X: "); Serial.print(event.acceleration.x); 
    Serial.print("Y: "); Serial.print(event.acceleration.y); 
    Serial.print("Z: "); Serial.println(event.acceleration.z);

    //Position sensor logic
    // Read the analog value from the sensor
    int sensorValue = analogRead(positionSensorPin);

    // Convert the sensor value to voltage
    float voltage = sensorValue* (voltValue/ 1023.0);

    // Map the voltage to distance (0V -> 0mm, voltValue -> maxDistance)
    //float distance = map(voltage, 3.23, voltValue, 0, maxDistance);
    float distance = (voltage - init_voltage) * (4.0 / (3.252-init_voltage));
    Serial.print(init_voltage,4);
    // Print the results
    Serial.print("Voltage: ");
    Serial.print(voltage, 3);  // Print voltage with 3 decimal places
    Serial.print(" V, Distance: ");
    Serial.print(distance, 2);  // Print distance with 1 decimal place
    Serial.println(" mm");
    oldvoltage = voltage;

    // Helper to read a bank and return its average pressure
    auto readAvg = [&](TwoWire &bus, BMP581 sensors[], uint8_t muxAddr) {
        float total = 0;
        for (uint8_t ch = 0; ch < 8; ch++) {
            if (!tcaselect(bus, muxAddr, ch)) {
              Serial.print("Failed to select ch ");
              Serial.println(ch);
              continue;
            }
            // let MUX and sensor settle
            delay(3);  // increased delay

            bmp5_sensor_data d;
            if (sensors[ch].getSensorData(&d) == BMP5_OK) {
                total += d.pressure;
                // Serial.print("Ch ");
                // Serial.print(ch);
                // Serial.print(": ");
                // Serial.println(d.pressure);
            } else {
                Serial.print("READ FAIL ch ");
                Serial.println(ch);
            }
        }
        // deselect all channels when done
        bus.beginTransmission(muxAddr);
        bus.write(0x00);
        bus.endTransmission();
        return total / 8.0;
    };

    // Read and average each bank
    float avg1a = readAvg(Wire1, sensors1a, TCA9548A_ADDR);
    float avg1b = readAvg(Wire1, sensors1b, TCA9548A_ADDR_2);
    float avg2a = readAvg(Wire2, sensors2a, TCA9548A_ADDR);
    float avg2b = readAvg(Wire2, sensors2b, TCA9548A_ADDR_2);

    // Debug prints
    Serial.print("Avg1a: "); Serial.println(avg1a);
    Serial.print("Avg1b: "); Serial.println(avg1b);
    Serial.print("Avg2a: "); Serial.println(avg2a);
    Serial.print("Avg2b: "); Serial.println(avg2b);

    // Actuator decision based on any bank exceeding pressure threshold
    if (avg1a > 102000.0 || avg1b > 98000.0 ||
        avg2a > 105000.0 || avg2b > 122000.0) {
        uint8_t forcemode[] = {0x55, 0xAA, 0x05, 0xFF, 
                        0x32, 0x25, 0x00, 0x03,
                        0x00, 0x5E};

        // sets the control mode as the positioning mode

        ACTUATOR_1.write(forcemode,10);
        ACTUATOR_2.write(forcemode,10);

        uint8_t force[] = {0x55, 0xAA, 0x05, 0xFF, 
                           0x32, 0x27, 0x00, 0xD0,
                           0x07, 0x34};


        //old checksum 0x34
        // uint8_t checksum_1 = calculateChecksum(force, sizeof(force));

        // Serial.println(checksum_1);

        delay(5);

        ACTUATOR_1.write(force,10);
        ACTUATOR_2.write(force,10);

    } else {
        uint8_t posmode[] = {0x55, 0xAA, 0x05, 0xFF,
                            0x32, 0x25, 0x00, 0x00,
                            0x00, 0x5B};
        ACTUATOR_1.write(posmode, 10);
        ACTUATOR_2.write(posmode, 10);
        
        delay(5);

        uint8_t fullretract[] = {0x55, 0xAA, 0x05, 0xFF,
                                0x32, 0x29, 0x00, 0x00,
                                0x00, 0x5F};
        ACTUATOR_1.write(fullretract, 10);
        ACTUATOR_2.write(fullretract, 10);
    }
    
    delay(5);
}

