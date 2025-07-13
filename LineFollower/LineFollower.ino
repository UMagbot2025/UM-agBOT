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
float Kp = 0.9;
float Kd = 0.1; // Note that Kp < Kd
void setup()
{
  Serial.begin(9600);
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

int lastError = 0;
uint16_t lastSensorValues[SensorCount];
unsigned long lastChangeTime = 0;
const unsigned long STUCK_TIMEOUT = 5000; // 5 seconds
const int STUCK_TOLERANCE = 50;
bool isStuck = false;

void loop()
{
  qtr.read(sensorValues);

  // Check if all sensor values are within ±50 of previous readings
  bool allWithinRange = true;
  for (int i = 0; i < SensorCount; i++) {
    if (abs(sensorValues[i] - lastSensorValues[i]) > STUCK_TOLERANCE) {
      allWithinRange = false;
      break;
    }
  }

  // Update stuck detection logic
  if (!allWithinRange) {
    lastChangeTime = millis();
    isStuck = false;
  } else if (millis() - lastChangeTime > STUCK_TIMEOUT) {
    isStuck = true;
  }

  // Save current values for next loop
  for (int i = 0; i < SensorCount; i++) {
    lastSensorValues[i] = sensorValues[i];
  }

  int position = qtr.readLineWhite(sensorValues);
  int error = position - 3500;
  int correction = Kp * error + Kd * (error - lastError);
  lastError = error;

  int baseSpeed = isStuck ? 140 : 90;  // boost speed if stuck
  int leftMotorSpeed = baseSpeed - correction;
  int rightMotorSpeed = baseSpeed + correction;

  setMotorSpeed(leftMotorSpeed, rightMotorSpeed);

  if (isStuck) {
    Serial.println("Stuck (within ±50 range)! Giving it some umph...");
  }

  delay(10);
}

