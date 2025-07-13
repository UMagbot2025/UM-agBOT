// Simple function to turn any LED on or off by pin number

void setLed(int pin, bool state) {
  digitalWrite(pin, state ? HIGH : LOW);
}

void setup() {
  // Example: using pins 13 and 8 for LEDs
  pinMode(13, OUTPUT);
  pinMode(8, OUTPUT);
}

void loop() {
  setLed(13, true);  // Turn ON LED at pin 13
  setLed(8, false);  // Turn OFF LED at pin 8
  delay(1000);

  setLed(13, false); // Turn OFF LED at pin 13
  setLed(8, true);   // Turn ON LED at pin 8
  delay(1000);
}
