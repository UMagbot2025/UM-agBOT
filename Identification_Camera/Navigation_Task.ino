#include <QTRSensors.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

/* Turning logic:
if right sensor sees it but left doesn't: turn right
if both sensor see it: turn left first time
if left sensor sees it but right doesn't: turn left
if both sensor see it: turn right next time
.....
*/

const int LEFT_IR = 31; // need to change to the actual pin
const int RIGHT_IR = 32;
bool nextBothTurnLeft = true; // toggle variable, initially we need to turn left

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
float Kd = 0.1; // Note that Kp < Kd
int lastError = 0;

// --- Turn-until-line-found tuning ---
const uint16_t LINE_THRESHOLD = 500;     // calibrated value above which a sensor counts as "on the line" (0-1000 scale). Tune this.
const unsigned long BLIND_TIME_MS = 200; // ignore sensor readings for this long at the start of a turn, so we actually leave the current line
const unsigned long MAX_TURN_MS = 1200;  // safety cutoff in case the line is never found (tune based on max real turn time)

void setup()
{
    Serial.begin(115200);
    AFMS.begin();

    pinMode(LEFT_IR, INPUT);
    pinMode(RIGHT_IR, INPUT);

    // QTR Sensor setup: digital pins
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){23, 24, 25, 26, 27, 28, 29, 30}, SensorCount);
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
    leftSpeed = constrain(leftSpeed, -225, 225); // constrain to protect the motors
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

// Returns true if any sensor currently reads above the line threshold
bool lineDetected()
{
    qtr.readLineBlack(sensorValues); // also updates sensorValues as a side effect
    for (uint8_t i = 0; i < SensorCount; i++)
    {
        if (sensorValues[i] > LINE_THRESHOLD)
            return true;
    }
    return false;
}

void turnRight()
{
    setMotorSpeed(120, -120);

    unsigned long startTime = millis();

    // Blind window: don't check the sensor right away, otherwise it'll
    // immediately "see" the line it's currently pivoting off of.
    while (millis() - startTime < BLIND_TIME_MS)
    {
        // keep turning
    }

    // Keep turning until the line reappears under the sensor, or we time out
    while (!lineDetected() && (millis() - startTime < MAX_TURN_MS))
    {
        // keep turning
    }

    setMotorSpeed(0, 0);
    delay(100);
}

void turnLeft()
{
    setMotorSpeed(-120, 120);

    unsigned long startTime = millis();

    while (millis() - startTime < BLIND_TIME_MS)
    {
        // keep turning
    }

    while (!lineDetected() && (millis() - startTime < MAX_TURN_MS))
    {
        // keep turning
    }

    setMotorSpeed(0, 0);
    delay(100);
}

void loop()
{
    bool leftDetected = digitalRead(LEFT_IR);
    bool rightDetected = digitalRead(RIGHT_IR);

    // Right only
    if (rightDetected && !leftDetected)
    {
        turnRight();
        return;
    }

    // Left only
    if (leftDetected && !rightDetected)
    {
        turnLeft();
        return;
    }

    // Both detect line
    if (leftDetected && rightDetected)
    {
        if (nextBothTurnLeft)
            turnLeft();
        else
            turnRight();

        nextBothTurnLeft = !nextBothTurnLeft;

        return;
    }

    int position = qtr.readLineBlack(sensorValues);
    int error = position - 3500;
    int correction = Kp * error + Kd * (error - lastError);
    lastError = error;

    int baseSpeed = 75; // tweak the baseSpeed
    int leftMotorSpeed = baseSpeed - correction;
    int rightMotorSpeed = baseSpeed + correction;

    setMotorSpeed(leftMotorSpeed, rightMotorSpeed);

    delay(10);
}