#include <QTRSensors.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Initialize motor shield
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1); // left front
Adafruit_DCMotor *motor2 = AFMS.getMotor(2); // right back
Adafruit_DCMotor *motor3 = AFMS.getMotor(3); // right front
Adafruit_DCMotor *motor4 = AFMS.getMotor(4); // left back

// Initialize QTR sensor
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// PID control parameters
float Kp = 0.1;
float Ki = 0.0;
float Kd = 0.05;

int lastError = 0;
int integral = 0;

void setup()
{
  Serial.begin(9600);

  AFMS.begin();

  // QTR Sensor setup: analog pins A8-A15
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A8, A9, A10, A11, A12, A13, A14, A15}, SensorCount);
  qtr.setEmitterPin(22);

  delay(500);

  // Calibrate sensors (move robot over line during calibration)
  for (int i = 0; i < 400; i++)
  {
    qtr.calibrate();
    delay(20);
  }

  Serial.println("Calibration complete.");
}

void setMotorSpeed(int leftSpeed, int rightSpeed)
{
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Set left motors
  if (leftSpeed >= 0)
  {
    motor1->setSpeed(leftSpeed);
    motor4->setSpeed(leftSpeed);
    motor1->run(FORWARD);
    motor4->run(FORWARD);
  }
  else
  {
    motor1->setSpeed(-leftSpeed);
    motor4->setSpeed(-leftSpeed);
    motor1->run(BACKWARD);
    motor4->run(BACKWARD);
  }

  // Set right motors
  if (rightSpeed >= 0)
  {
    motor2->setSpeed(rightSpeed);
    motor3->setSpeed(rightSpeed);
    motor2->run(FORWARD);
    motor3->run(FORWARD);
  }
  else
  {
    motor2->setSpeed(-rightSpeed);
    motor3->setSpeed(-rightSpeed);
    motor2->run(BACKWARD);
    motor3->run(BACKWARD);
  }
}

void loop()
{
  int position = qtr.readLineWhite(sensorValues);
  int error = position - 3500;

  integral += error;
  int derivative = error - lastError;
  lastError = error;

  int correction = Kp * error + Ki * integral + Kd * derivative;

  int baseSpeed = 100;
  int leftMotorSpeed = baseSpeed - correction;
  int rightMotorSpeed = baseSpeed + correction;

  setMotorSpeed(leftMotorSpeed, rightMotorSpeed);

  delay(10);
}
