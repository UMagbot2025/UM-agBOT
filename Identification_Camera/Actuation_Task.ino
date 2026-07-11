#include <QTRSensors.h>
#include <SPI.h>
#include <Pixy2SPI_SS.h> // Pixy2 variant that supports a custom Slave-Select pin,
                         // which lets two Pixy2 cameras share one SPI bus.
                         // NOTE: confirm this class name/constructor matches the
                         // version of the Pixy2 Arduino library you have installed -
                         // some versions call init() with a cs pin, others take it
                         // in the constructor. Adjust setup() below if needed.

// ================= USER-CONFIGURABLE CONSTANTS =================

// --- Pixy2 chip-select pins (TODO: set to whatever CS pins you wired) ---
const int PIXY_LEFT_CS = 2;
const int PIXY_RIGHT_CS = 3;

// --- Arm motor driver pins (TODO: set to your actual M1/M2 control pins) ---
// Assumes a simple driver/relay where HIGH = motor on. If you're using a
// motor driver with direction pins or PWM, replace activateArms()/stopArms().
const int M1_PIN = 40;
const int M2_PIN = 41;

// --- Alignment calibration ---
// Steps needed to move the robot 1 mm. Calibrate by commanding e.g. 1000 steps
// and measuring how far the robot actually travelled.
const long STEPS_PER_MM = 5;

// Offset (in steps) between "Pixy2 confirms double plant" and "arm is centered
// on the plant". Positive = drive forward more after detection, negative = back up.
// Measure this physically on your robot: mark where the camera detects the plant
// vs. where the arm needs to be, convert that distance to steps.
long ARM_ALIGN_OFFSET_STEPS = 150;

// How long to run the arm motors once triggered (ms)
const unsigned int ARM_RUN_TIME_MS = 3000;

// --- PID gains for line following ---
float Kp = 0.045;
float Ki = 0.0;
float Kd = 0.02;

// Base step interval (microseconds between steps) at "cruise" speed.
// Lower = faster. Tune this together with Kp/Kd.
int baseSpeedDelay = 600;

// ================= STEPPER PINS (given) =================
const int stepPins[4] = {12, 6, 8, 10};
const int dirPins[4] = {13, 7, 9, 11};
bool invertDir[4] = {true, false, true, false};
// Index convention: 0 = FL, 1 = FR, 2 = RL, 3 = RR
// (Adjust if your existing code uses a different order - keep it consistent
//  with the rest of your codebase.)
const int LEFT_IDX[2] = {0, 2};  // FL, RL
const int RIGHT_IDX[2] = {1, 3}; // FR, RR

// ================= QTR SENSOR (given) =================
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// ================= PIXY2 =================
Pixy2SPI_SS pixyLeft;
Pixy2SPI_SS pixyRight;

// ================= STATE MACHINE =================
enum RobotState
{
    LINE_FOLLOWING,
    ARM_ALIGN,
    ARM_ACTIVE,
    DONE
};
RobotState state = LINE_FOLLOWING;

// ================= STEPPER TIMING (non-blocking) =================
unsigned long lastStepTime[4] = {0, 0, 0, 0};
unsigned long stepInterval[4] = {0, 0, 0, 0}; // 0 = wheel not moving

// PID working vars
int lastError = 0;
long integral = 0;

// ---------------------------------------------------------------
void setup()
{
    Serial.begin(115200);

    for (int i = 0; i < 4; i++)
    {
        pinMode(stepPins[i], OUTPUT);
        pinMode(dirPins[i], OUTPUT);
    }

    pinMode(M1_PIN, OUTPUT);
    pinMode(M2_PIN, OUTPUT);
    digitalWrite(M1_PIN, LOW);
    digitalWrite(M2_PIN, LOW);

    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){23, 24, 25, 26, 27, 28, 29, 30}, SensorCount);
    qtr.setEmitterPin(22);

    // Optional: run your existing calibration routine here (sweep the line
    // and call qtr.calibrate() repeatedly). Skipped here since you likely
    // already have this in your main sketch.

    // Pixy2 init - TODO: verify against your installed Pixy2 library version.
    pixyLeft.init(PIXY_LEFT_CS);
    pixyRight.init(PIXY_RIGHT_CS);

    Serial.println("agBOT harvest state machine ready.");
}

// ---------------------------------------------------------------
void loop()
{
    switch (state)
    {

    case LINE_FOLLOWING:
    {
        // Run PID sensor sampling every 5ms (QTR-RC read is slow, so we
        // decouple it from the step pulsing loop, same as your line-follow work).
        static unsigned long lastPID = 0;
        if (micros() - lastPID >= 5000UL)
        {
            lineFollowPID();
            lastPID = micros();
        }

        // Pulse steppers as fast as their individual intervals require.
        updateSteppers();

        // Check both cameras periodically (SPI reads aren't free, don't do it every loop).
        static unsigned long lastPixyCheck = 0;
        if (millis() - lastPixyCheck >= 50)
        {
            lastPixyCheck = millis();
            bool leftSees = seesDoublePlant(pixyLeft);
            bool rightSees = seesDoublePlant(pixyRight);

            if (leftSees && rightSees)
            {
                Serial.println("Double plant detected on BOTH sides. Stopping.");
                stopAllWheels();
                state = ARM_ALIGN;
            }
        }
        break;
    }

    case ARM_ALIGN:
        Serial.println("Aligning arms...");
        alignArms(ARM_ALIGN_OFFSET_STEPS);
        state = ARM_ACTIVE;
        break;

    case ARM_ACTIVE:
        Serial.println("Activating arm motors...");
        activateArms();
        state = DONE;
        break;

    case DONE:
        // Robot finished this cycle. Decide here whether to resume
        // line following for the next plant, or stop entirely.
        // e.g. state = LINE_FOLLOWING; if you want to keep going.
        break;
    }
}

// ================= LINE FOLLOWING =================

void lineFollowPID()
{
    uint16_t position = qtr.readLineBlack(sensorValues);
    int error = (int)position - 3500; // 3500 = center for 8 sensors (0-7000 range)

    integral += error;
    int derivative = error - lastError;
    lastError = error;

    float correction = Kp * error + Ki * integral + Kd * derivative;

    int leftDelay = baseSpeedDelay + (int)correction;
    int rightDelay = baseSpeedDelay - (int)correction;

    leftDelay = constrain(leftDelay, 200, 3000);
    rightDelay = constrain(rightDelay, 200, 3000);

    for (int i = 0; i < 2; i++)
        setWheelDir(LEFT_IDX[i], true);
    for (int i = 0; i < 2; i++)
        setWheelDir(RIGHT_IDX[i], true);

    for (int i = 0; i < 2; i++)
        stepInterval[LEFT_IDX[i]] = leftDelay;
    for (int i = 0; i < 2; i++)
        stepInterval[RIGHT_IDX[i]] = rightDelay;
}

// ================= STEPPER HELPERS =================

void setWheelDir(int idx, bool forward)
{
    bool dirState = forward ? !invertDir[idx] : invertDir[idx];
    digitalWrite(dirPins[idx], dirState ? HIGH : LOW);
}

void pulseStep(int idx)
{
    digitalWrite(stepPins[idx], HIGH);
    delayMicroseconds(3);
    digitalWrite(stepPins[idx], LOW);
}

void updateSteppers()
{
    unsigned long now = micros();
    for (int i = 0; i < 4; i++)
    {
        if (stepInterval[i] > 0 && (now - lastStepTime[i]) >= stepInterval[i])
        {
            pulseStep(i);
            lastStepTime[i] = now;
        }
    }
}

void stopAllWheels()
{
    for (int i = 0; i < 4; i++)
        stepInterval[i] = 0;
}

// ================= PIXY2 DETECTION =================

bool seesDoublePlant(Pixy2SPI_SS &pixy)
{
    pixy.ccc.getBlocks();
    bool sawSig1 = false;
    bool sawSig2 = false;

    for (int i = 0; i < pixy.ccc.numBlocks; i++)
    {
        if (pixy.ccc.blocks[i].m_signature == 1)
            sawSig1 = true;
        if (pixy.ccc.blocks[i].m_signature == 2)
            sawSig2 = true;
    }
    return sawSig1 && sawSig2;
}

// ================= ARM ALIGNMENT =================

// Moves the robot straight forward (steps > 0) or backward (steps < 0)
// by exactly `steps` stepper pulses on all four wheels, in lockstep,
// at a fixed slow speed suitable for a precise final approach.
// Call alignArmsMM(distanceInMM) instead if you'd rather work in mm.
void alignArms(long steps)
{
    bool forward = steps >= 0;
    long stepsToMove = abs(steps);

    for (int i = 0; i < 4; i++)
        setWheelDir(i, forward);

    const unsigned int moveDelayUs = 700; // slow + controlled for final approach; tune as needed

    for (long s = 0; s < stepsToMove; s++)
    {
        for (int i = 0; i < 4; i++)
            pulseStep(i);
        delayMicroseconds(moveDelayUs);
    }

    stopAllWheels();
}

// Convenience wrapper if you'd rather calibrate/verify in millimeters.
void alignArmsMM(float distanceMM)
{
    long steps = (long)(distanceMM * STEPS_PER_MM);
    alignArms(steps);
}

// ================= ARM MOTORS =================

void activateArms()
{
    digitalWrite(M1_PIN, HIGH);
    digitalWrite(M2_PIN, HIGH);
    delay(ARM_RUN_TIME_MS);
    digitalWrite(M1_PIN, LOW);
    digitalWrite(M2_PIN, LOW);
}
