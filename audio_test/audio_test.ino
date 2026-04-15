#include <Audio.h>

AudioInputI2S i2s1;
AudioRecordQueue queue1;
AudioConnection patchCord1(i2s1, 1, queue1, 0);

void setup() {
  Serial.begin(115200);
  AudioMemory(30);
  queue1.begin();
}

void loop() {
  if (queue1.available()) {
    int16_t *buffer = queue1.readBuffer();

    int max_val = 0;
    for (int i = 0; i < 128; i++) {
      int val = abs(buffer[i]);
      if (val > max_val) max_val = val;
    }

    Serial.println(max_val);

    queue1.freeBuffer();
  }
}