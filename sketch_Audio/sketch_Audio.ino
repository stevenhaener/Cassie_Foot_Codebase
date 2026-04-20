#include <Audio.h>

// ==================================================
// Teensy 4.1 Audio Sensor Test
// Left channel only
// Outputs time + raw + smoothed values
// Use Serial Monitor or Serial Plotter
// ==================================================

// Primary I2S input
AudioInputI2S2       i2s2;

// Peak analyzer on LEFT channel only
AudioAnalyzePeak    peakL;
AudioConnection     cordL(i2s2, 0, peakL, 0);

// Smoothing
float emaLevel = 0.0f;
const float alpha = 0.2f;

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    AudioMemory(32);
    delay(500);

    // Header for CSV logging
    Serial.println("time_ms,raw_peak,smoothed_peak");
}

void loop() {
    if (peakL.available()) {
        float rawVal = peakL.read();

        // Exponential moving average
        emaLevel = alpha * rawVal + (1.0f - alpha) * emaLevel;

        // Print time, raw, smoothed
        Serial.print(millis());
        Serial.print(",");
        Serial.print(rawVal, 6);
        Serial.print(",");
        Serial.println(emaLevel, 6);
    }
}