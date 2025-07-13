// Simple function to turn any LED on or off by pin number

#define RED 13
#define GREEN 8

void setLed(int pin, bool state) {
  digitalWrite(pin, state ? HIGH : LOW);
}

void setup() {
  // Example: using pins 13 and 8 for LEDs
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
}

void goodEgg(){
  setLed(GREEN, true);
  delay(1000);
  setLed(GREEN, false);
}

void badEgg(){
  setLed(REN, true);
  delay(1000);
  setLed(RED, false);
}

void loop() {
  setLed(RED, true);  // Turn ON LED at pin 13
  setLed(GREEN, false);  // Turn OFF LED at pin 8
  delay(1000);

  setLed(RED, false); // Turn OFF LED at pin 13
  setLed(GREEN, true);   // Turn ON LED at pin 8
  delay(1000);
}
