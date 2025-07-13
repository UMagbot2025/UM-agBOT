#include <QTRSensors.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Initialize motor shield
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1); // Left front
Adafruit_DCMotor *motor2 = AFMS.getMotor(2); // Left back
Adafruit_DCMotor *motor3 = AFMS.getMotor(3); // Right front
Adafruit_DCMotor *motor4 = AFMS.getMotor(4); // Right back

// Initialize QTR sensor
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// Proportional control only
float Kp = 0.4;
float Kd = 0.1;

// Robot state
bool robotActive = true;
String inputString = "";
bool stringComplete = false;

void setup()
{
  Serial.begin(9600);
  inputString.reserve(50);
  
  AFMS.begin();

  // QTR Sensor setup: analog pins A8-A15
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A15, A14, A13, A12, A11, A10, A9, A8}, SensorCount);
  qtr.setEmitterPin(22);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Indicate calibration mode

  delay(500);

  // Calibrate sensors
  for (int i = 0; i < 200; i++)
  {
    qtr.calibrate();
    Serial.println(i);
    delay(20);
  }

  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Calibration complete.");
}

void setMotorSpeed(int leftSpeed, int rightSpeed)
{
  leftSpeed = constrain(leftSpeed, -225, 225);
  rightSpeed = constrain(rightSpeed, -225, 225);

  // Left motors
  if (leftSpeed >= 0)
  {
    motor1->setSpeed(leftSpeed);
    motor2->setSpeed(leftSpeed);
    motor1->run(FORWARD);
    motor2->run(FORWARD);
  }
  else
  {
    motor1->setSpeed(-leftSpeed);
    motor2->setSpeed(-leftSpeed);
    motor1->run(BACKWARD);
    motor2->run(BACKWARD);
  }

  // Right motors
  if (rightSpeed >= 0)
  {
    motor3->setSpeed(rightSpeed);
    motor4->setSpeed(rightSpeed);
    motor3->run(FORWARD);
    motor4->run(FORWARD);
  }
  else
  {
    motor3->setSpeed(-rightSpeed);
    motor4->setSpeed(-rightSpeed);
    motor3->run(BACKWARD);
    motor4->run(BACKWARD);
  }
}

void stopMotors() {
  motor1->setSpeed(0);
  motor2->setSpeed(0);
  motor3->setSpeed(0);
  motor4->setSpeed(0);
  
  motor1->run(RELEASE);
  motor2->run(RELEASE);
  motor3->run(RELEASE);
  motor4->run(RELEASE);
}

int lastError = 0;

void loop()
{
  // Process any incoming serial commands
  if (stringComplete) {
    if (inputString.indexOf("STOP") >= 0) {
      robotActive = false;
      stopMotors();
      Serial.println("Robot stopped");
    } 
    else if (inputString.indexOf("RESUME") >= 0) {
      robotActive = true;
      Serial.println("Robot resumed");
    }
    
    // Clear the string
    inputString = "";
    stringComplete = false;
  }
  
  // Only run line following if robot is active
  if (robotActive) {
    int position = qtr.readLineWhite(sensorValues);
    int error = position - 3500;
    int correction = Kp * error + Kd * (error - lastError);
    lastError = error;

    int baseSpeed = 75;
    int leftMotorSpeed = baseSpeed - correction;
    int rightMotorSpeed = baseSpeed + correction;

    setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
  }

  delay(10);
}

// Serial event function
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}