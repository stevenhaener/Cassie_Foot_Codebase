// Pin setup
const int positionSensorPin = A11; // Pin where linear position meter is connected, on board no 24
const float maxDistance = 100.0; // Maximum travel distance in mm (arbritary set on a scale of 0-100)
const float voltValue = 5.0; // Volts, conect yellow wire to GND and green wire to the voltage pin
float oldvoltage = 0.0;

void setup() {
  Serial.begin(9600);
  pinMode(positionSensorPin, INPUT);  // Set the sensor pin as input
  Serial.println("Positionmeter initialised");
}

void loop() {
  // Read the analog value from the sensor
  int sensorValue = analogRead(positionSensorPin);

  // Convert the sensor value to voltage
  float voltage = sensorValue* (voltValue/ 1023.0);

  // Map the voltage to distance (0V -> 0mm, voltValue -> maxDistance)
  float distance = map(voltage, 0, voltValue, 0, maxDistance);

  // Print the results
  if (!(oldvoltage == voltage)){ // reduce activity in serial monitor with duplicate readings
    Serial.print("Voltage: ");
    Serial.print(voltage, 3);  // Print voltage with 3 decimal places
    Serial.print(" V, Distance: ");
    Serial.print(distance, 1);  // Print distance with 1 decimal place
    Serial.println(" mm");
  }
  oldvoltage = voltage;
  delay(500);  // Wait a bit before next reading
}