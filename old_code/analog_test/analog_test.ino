void setup() {
  Serial.begin(9600);  // Start the serial communication at 9600 baud
  while (!Serial) {
    // Wait for the Serial Monitor to connect (useful for some boards)
  }
  pinMode(14, INPUT);  // Set pin 14 (A0) as input
}

void loop() {
  int analogValue = analogRead(14);  // Read the analog value from pin 14
  Serial.println(analogValue);      // Print the value to the Serial Monitor
  delay(100);                       // Small delay for readability (100ms)
}
