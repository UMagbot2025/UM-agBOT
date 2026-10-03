#include <QTRSensors.h>

/*
  Turn behaviour:
    - IR sensor (left or right) fires  -> execute the next turn in turnSequence[]
    - A turn drives forward briefly to clear the intersection, then pivots
    - The pivot keeps going until the QTR array sees the line again (or MAX_TURN_MS expires)
    - As soon as the line is seen, motors stop and PID takes over
*/

// ---------------- Pins ----------------
const int LEFT_IR = 22;
const int RIGHT_IR = 23;

// DFRobot quad motor shield
// M1 = rear left, M2 = front right, M3 = front left, M4 = rear right
const int M1_PWM = 3, M1_DIR = 4;
const int M2_PWM = 11, M2_DIR = 12;
const int M3_PWM = 5, M3_DIR = 8;
const int M4_PWM = 6, M4_DIR = 7;

// ---------------- Bench test ----------------
const int TEST_SPEED = 100; // 0-255, keep low for bench testing
const int RUN_TIME_MS = 1000;
const int PAUSE_MS = 500;

// ---------------- Motion tuning ----------------
const int MAX_SPEED = 60;  // hard cap for driving/turning
const int BASE_SPEED = 50; // PID cruise speed
const int TURN_SPEED = 50; // pivot speed (<= MAX_SPEED or it gets clipped)
const int FORWARD_BEFORE_TURN_SPEED = 50;
const unsigned long FORWARD_BEFORE_TURN_MS = 600;

// ---------------- Line detection ----------------
// "Sees the line" = a sensor reads more than halfway between its own calibrated
// white and its own calibrated black. Calibration does the scaling, so this
// number is not a raw magic value you have to re-tune per surface.
const uint16_t ON_LINE_LEVEL = 800; // 0 = calibrated white, 1000 = calibrated black
const uint8_t MIN_SENSORS_ON_LINE = 2;

const unsigned long BLIND_TIME_MS = 1500;  // ignore the QTR at the start of a turn so we don't re-see the line we're leaving
const unsigned long MAX_TURN_MS = 2500;    // safety cutoff if the line is never found
const unsigned long LOG_INTERVAL_MS = 100; // throttle serial prints during a turn

// After a turn ends, ignore the IR sensors for a bit so we don't re-trigger on the same intersection
unsigned long irCooldownUntil = 0;
const unsigned long IR_COOLDOWN_MS = 2000;

// ---------------- Hardcoded turn sequence ----------------
enum TurnDirection
{
    TURN_LEFT,
    TURN_RIGHT
};
const TurnDirection turnSequence[] = {
    TURN_RIGHT, TURN_RIGHT, TURN_LEFT, TURN_LEFT,
    TURN_RIGHT, TURN_RIGHT, TURN_LEFT, TURN_LEFT,
    TURN_RIGHT, TURN_RIGHT};
const int NUM_TURNS = sizeof(turnSequence) / sizeof(turnSequence[0]);
int turnIndex = 0;

// ---------------- QTR ----------------
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// PD control
float Kp = 0.08;
float Kd = 0.03;
int lastError = 0;

void setup()
{
    Serial.begin(115200);

    pinMode(LEFT_IR, INPUT);
    pinMode(RIGHT_IR, INPUT);

    pinMode(M1_PWM, OUTPUT);
    pinMode(M1_DIR, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
    pinMode(M2_DIR, OUTPUT);
    pinMode(M3_PWM, OUTPUT);
    pinMode(M3_DIR, OUTPUT);
    pinMode(M4_PWM, OUTPUT);
    pinMode(M4_DIR, OUTPUT);

    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){25, 26, 27, 28, 29, 30, 31, 32}, SensorCount);
    qtr.setEmitterPin(24);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // calibration in progress
    delay(500);

    for (int i = 0; i < 200; i++)
    {
        qtr.calibrate();
        delay(20);
    }

    digitalWrite(LED_BUILTIN, LOW);
    Serial.println(F("Calibration complete."));
}

void loop()
{
    // --- Bench testing: uncomment these and comment out everything below ---
    // testMotor(M1_PWM, M1_DIR, "M1 (rear left)");
    // testMotor(M2_PWM, M2_DIR, "M2 (front right)");
    // testMotor(M3_PWM, M3_DIR, "M3 (front left)");
    // testMotor(M4_PWM, M4_DIR, "M4 (rear right)");
    // return;

    if (millis() >= irCooldownUntil)
    {
        if (digitalRead(LEFT_IR) || digitalRead(RIGHT_IR))
        {
            executeNextTurn();
            return;
        }
    }

    runPID();
    delay(10);
}

// ---------------- Motors ----------------
void driveMotor(int pwmPin, int dirPin, int speed)
{
    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

    if (speed >= 0)
    {
        digitalWrite(dirPin, HIGH); // forward
        analogWrite(pwmPin, speed);
    }
    else
    {
        digitalWrite(dirPin, LOW); // reverse
        analogWrite(pwmPin, -speed);
    }
}

void setMotorSpeed(int leftSpeed, int rightSpeed)
{
    driveMotor(M3_PWM, M3_DIR, leftSpeed);  // front left
    driveMotor(M1_PWM, M1_DIR, leftSpeed);  // rear left
    driveMotor(M2_PWM, M2_DIR, rightSpeed); // front right
    driveMotor(M4_PWM, M4_DIR, rightSpeed); // rear right
}

// ---------------- Bench test helpers ----------------
void stopMotor(int pwmPin, int dirPin)
{
    analogWrite(pwmPin, 0);
}

void moveForward(int pwmPin, int dirPin, int speed, const char *label)
{
    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin, speed);
}

void moveBackward(int pwmPin, int dirPin, int speed, const char *label)
{
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, speed);
}

// Spins one motor forward, stops, spins it backward, stops.
// NOTE: this writes analogWrite() directly, so it bypasses the MAX_SPEED cap.
void testMotor(int pwmPin, int dirPin, const char *label)
{
    Serial.print(label);
    Serial.println(F(" - FORWARD"));
    moveForward(pwmPin, dirPin, TEST_SPEED, label);
    delay(RUN_TIME_MS);

    stopMotor(pwmPin, dirPin);
    delay(PAUSE_MS);

    Serial.print(label);
    Serial.println(F(" - BACKWARD"));
    moveBackward(pwmPin, dirPin, TEST_SPEED, label);
    delay(RUN_TIME_MS);

    stopMotor(pwmPin, dirPin);
    delay(PAUSE_MS);
}

// ---------------- Line detection ----------------
// True once the QTR array sees the line again.
bool lineDetected()
{
    qtr.readCalibrated(sensorValues); // 0 = calibrated white, 1000 = calibrated black

    uint8_t count = 0;
    for (uint8_t i = 0; i < SensorCount; i++)
    {
        if (sensorValues[i] > ON_LINE_LEVEL)
            count++;
    }
    return count >= MIN_SENSORS_ON_LINE;
}

void printSensorValues()
{
    Serial.print(F("Sensors: "));
    for (uint8_t i = 0; i < SensorCount; i++)
    {
        Serial.print(sensorValues[i]);
        Serial.print(' ');
    }
    Serial.println();
}

// ---------------- Turning ----------------
void performTurn(TurnDirection dir)
{
    Serial.println(dir == TURN_LEFT ? F("Turning LEFT") : F("Turning RIGHT"));

    // 1. Clear the intersection
    setMotorSpeed(FORWARD_BEFORE_TURN_SPEED, FORWARD_BEFORE_TURN_SPEED);
    delay(FORWARD_BEFORE_TURN_MS);

    // 2. Start pivoting
    if (dir == TURN_LEFT)
        setMotorSpeed(-TURN_SPEED, TURN_SPEED);
    else
        setMotorSpeed(TURN_SPEED, -TURN_SPEED);

    // 3. Keep pivoting until the QTR sees the line (or we time out)
    unsigned long startTime = millis();
    unsigned long lastLog = 0;
    bool found = false;

    while (millis() - startTime < MAX_TURN_MS)
    {
        if (millis() - startTime >= BLIND_TIME_MS && lineDetected())
        {
            found = true;
            break;
        }

        if (millis() - lastLog >= LOG_INTERVAL_MS)
        {
            lastLog = millis();
            printSensorValues();
        }
    }

    // 4. Stop, hand control back to PID
    setMotorSpeed(0, 0);
    Serial.println(found ? F("Line found - resuming PID") : F("TURN TIMEOUT - no line found"));

    lastError = 0;                               // pre-turn error is meaningless now
    irCooldownUntil = millis() + IR_COOLDOWN_MS; // don't re-trigger on the intersection we just left
}

void executeNextTurn()
{
    if (turnIndex >= NUM_TURNS)
    {
        Serial.println(F("Turn sequence complete - no more turns."));
        return;
    }

    Serial.print(F("Executing turn #"));
    Serial.print(turnIndex);
    Serial.print(F(" of "));
    Serial.println(NUM_TURNS);

    performTurn(turnSequence[turnIndex]);
    turnIndex++;
}

// ---------------- PID ----------------
void runPID()
{
    int position = qtr.readLineBlack(sensorValues); // calibrated 0..7000
    int error = position - 3500;
    int correction = Kp * error + Kd * (error - lastError);
    lastError = error;

    setMotorSpeed(BASE_SPEED - correction, BASE_SPEED + correction);
}
