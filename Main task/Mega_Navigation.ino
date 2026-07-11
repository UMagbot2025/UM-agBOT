/*
  agBOT - MEGA: Navigation controller
  ------------------------------------
  Runs line-following (QTR-8RC + PID) on the mecanum steppers.
  Talks to two Uno boards (left camera+arm, right camera+arm) over simple
  digital handshake lines - no serial protocol needed since we're only
  passing boolean events back and forth.

  Handshake pins (all digital, ALL THREE BOARDS MUST SHARE A COMMON GND):
    LEFT_DETECT_PIN  (input)  <- HIGH when the left Uno has latched a
                                  confirmed double-plant (sig 1 AND sig 2)
    RIGHT_DETECT_PIN (input)  <- same, from the right Uno
    ALIGN_GO_PIN     (output) -> HIGH once THIS Mega has finished the
                                  alignment move. Wire this single pin to
                                  BOTH Unos' "go" input so they fire their
                                  arms together, not the instant they spot
                                  the plant (that's the bug the original
                                  one-way version had - see chat).
    LEFT_DONE_PIN    (input)  <- HIGH once the left Uno's arm cycle is finished
    RIGHT_DONE_PIN   (input)  <- same, from the right Uno

  Sequence:
    1) LINE_FOLLOWING until LEFT_DETECT_PIN and RIGHT_DETECT_PIN are both HIGH
    2) Stop, run alignArms() to shift the fixed calibrated offset
    3) Raise ALIGN_GO_PIN - tells both Unos "go ahead, fire your arm now"
    4) Wait until LEFT_DONE_PIN and RIGHT_DONE_PIN are both HIGH
    5) Lower ALIGN_GO_PIN (resets the handshake), resume LINE_FOLLOWING
*/

#include <QTRSensors.h>

// ================= HANDSHAKE PINS (TODO: confirm these are free on your wiring) =================
const int LEFT_DETECT_PIN = 31;  // input  <- left Uno
const int RIGHT_DETECT_PIN = 32; // input  <- right Uno
const int ALIGN_GO_PIN = 33;     // output -> both Unos
const int LEFT_DONE_PIN = 34;    // input  <- left Uno
const int RIGHT_DONE_PIN = 35;   // input  <- right Uno

// ================= ALIGNMENT CALIBRATION =================
const long STEPS_PER_MM = 5;       // calibrate: steps needed to move the robot 1mm
long ARM_ALIGN_OFFSET_STEPS = 150; // measured physical offset between "cameras
                                   // confirm double plant" and "arm centered on it"

// ================= PID (line following) =================
float Kp = 0.045, Ki = 0.0, Kd = 0.02;
int baseSpeedDelay = 600;
int lastError = 0;
long integral = 0;

// ================= STEPPER PINS (given) =================
const int stepPins[4] = {12, 6, 8, 10};
const int dirPins[4] = {13, 7, 9, 11};
bool invertDir[4] = {true, false, true, false};
// Index convention: 0 = FL, 1 = FR, 2 = RL, 3 = RR
const int LEFT_IDX[2] = {0, 2};  // FL, RL
const int RIGHT_IDX[2] = {1, 3}; // FR, RR

// ================= QTR SENSOR (given) =================
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// ================= STATE MACHINE =================
enum RobotState
{
    LINE_FOLLOWING,
    ARM_ALIGN,
    WAIT_FOR_ARMS,
    RESET_HANDSHAKE
};
RobotState state = LINE_FOLLOWING;

unsigned long lastStepTime[4] = {0, 0, 0, 0};
unsigned long stepInterval[4] = {0, 0, 0, 0}; // 0 = wheel not moving

// ---------------------------------------------------------------
void setup()
{
    Serial.begin(115200);

    for (int i = 0; i < 4; i++)
    {
        pinMode(stepPins[i], OUTPUT);
        pinMode(dirPins[i], OUTPUT);
    }

    pinMode(LEFT_DETECT_PIN, INPUT);
    pinMode(RIGHT_DETECT_PIN, INPUT);
    pinMode(ALIGN_GO_PIN, OUTPUT);
    pinMode(LEFT_DONE_PIN, INPUT);
    pinMode(RIGHT_DONE_PIN, INPUT);
    digitalWrite(ALIGN_GO_PIN, LOW);

    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){23, 24, 25, 26, 27, 28, 29, 30}, SensorCount);
    qtr.setEmitterPin(22);

    // Optional: run your existing calibration routine here.

    Serial.println("Mega navigation ready.");
}

// ---------------------------------------------------------------
void loop()
{
    switch (state)
    {

    case LINE_FOLLOWING:
    {
        static unsigned long lastPID = 0;
        if (micros() - lastPID >= 5000UL)
        {
            lineFollowPID();
            lastPID = micros();
        }
        updateSteppers();

        if (digitalRead(LEFT_DETECT_PIN) == HIGH && digitalRead(RIGHT_DETECT_PIN) == HIGH)
        {
            Serial.println("Both sides confirmed double plant. Stopping.");
            stopAllWheels();
            state = ARM_ALIGN;
        }
        break;
    }

    case ARM_ALIGN:
        Serial.println("Aligning...");
        alignArms(ARM_ALIGN_OFFSET_STEPS);
        Serial.println("Aligned. Signaling both Unos to fire arms.");
        digitalWrite(ALIGN_GO_PIN, HIGH);
        state = WAIT_FOR_ARMS;
        break;

    case WAIT_FOR_ARMS:
        // Do NOT resume driving until both arms confirm they're finished,
        // or the robot would pull away mid-cycle.
        if (digitalRead(LEFT_DONE_PIN) == HIGH && digitalRead(RIGHT_DONE_PIN) == HIGH)
        {
            Serial.println("Both arms done.");
            state = RESET_HANDSHAKE;
        }
        break;

    case RESET_HANDSHAKE:
        digitalWrite(ALIGN_GO_PIN, LOW);
        delay(50); // give the Unos a moment to see GO drop and reset their own latches
        state = LINE_FOLLOWING;
        break;
    }
}

// ================= LINE FOLLOWING =================
void lineFollowPID()
{
    uint16_t position = qtr.readLineBlack(sensorValues);
    int error = (int)position - 3500;

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

// ================= ARM ALIGNMENT =================
void alignArms(long steps)
{
    bool forward = steps >= 0;
    long stepsToMove = abs(steps);
    for (int i = 0; i < 4; i++)
        setWheelDir(i, forward);

    const unsigned int moveDelayUs = 700; // slow + controlled for final approach
    for (long s = 0; s < stepsToMove; s++)
    {
        for (int i = 0; i < 4; i++)
            pulseStep(i);
        delayMicroseconds(moveDelayUs);
    }
    stopAllWheels();
}

void alignArmsMM(float distanceMM)
{
    alignArms((long)(distanceMM * STEPS_PER_MM));
}
