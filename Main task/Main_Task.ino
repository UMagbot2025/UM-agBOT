#include <QTRSensors.h>
#include <Pixy2.h>

Pixy2 pixy;

// ======================================================
// MOTOR PINS
// ======================================================
// Ben's commit

// M1 = Front Right
const uint8_t M1_PWM = 6;
const uint8_t M1_DIR = 7;

// M2 = Rear Left
const uint8_t M2_PWM = 5;
const uint8_t M2_DIR = 8;

// M3 = Rear Right
const uint8_t M3_PWM = 11;
const uint8_t M3_DIR = 12;

// M4 = Front Left
const uint8_t M4_PWM = 3;
const uint8_t M4_DIR = 4;

// ======================================================
// MOTOR DIRECTION SETTINGS
// ======================================================
// Change an entry to false if that motor spins backward
// when the robot is commanded to move forward.

const bool M1_FORWARD_HIGH = true;  // front right
const bool M2_FORWARD_HIGH = false; // rear left
const bool M3_FORWARD_HIGH = false; // rear right
const bool M4_FORWARD_HIGH = false; // front left

// ======================================================
// UNO HANDSHAKE
// ======================================================

/*
   Both lines are active LOW.

   The receiving side uses INPUT_PULLUP, so a board that has not
   finished booting (pins floating as inputs) is read as HIGH, which
   means "not asserted". Neither board can trigger the other by
   accident during power-up.
*/

const uint8_t ARM_GO_PIN = 40;   // OUTPUT to Uno D2
const uint8_t ARM_DONE_PIN = 41; // INPUT from Uno D9

// ======================================================
// QTR SENSOR
// ======================================================

QTRSensors qtr;

const uint8_t SensorCount = 6;

const uint8_t sensorPins[SensorCount] =
    {
        26, 27, 28, 29, 30, 31};

uint16_t sensorValues[SensorCount];

const uint8_t QTR_EMITTER_PIN = 24;

// For eight sensors, readLineBlack() returns 0-7000.
// The middle of the sensor is therefore approximately 3500.
const int LINE_CENTER = 2500;

// ======================================================
// PIXY2 VISION
// ======================================================

const uint8_t SIG_PLANT = 1;

/*
   Pixy2's colour-connected-components frame is 316 x 208.
   Block x values run 0 to 315, so the horizontal centre is 158.
*/
const int FRAME_CENTER_X = 155;

// How far off centre the plant may be and still count as centred.
// Smaller = more precise stop, but harder to trigger.
const int CENTER_TOLERANCE = 15; // pixels

// Reject small blocks: noise, distant plants, stray pixels.
const int MIN_BLOCK_AREA = 300; // width * height, in pixels

// Must be centred this many consecutive checks before stopping.
const uint8_t DETECT_THRESHOLD = 2;

uint8_t detectStreak = 0;

// Pixy2 runs at ~60 fps. Polling faster than that wastes loop time.
unsigned long lastPixyTime = 0;
const unsigned long PIXY_INTERVAL_MS = 20;

// ======================================================
// TURN TRIGGER SENSORS
// ======================================================

// --- IR (used on odd turns) ---
const uint8_t LEFT_IR = 42;
// const uint8_t RIGHT_IR = 40;

// After a turn ends, ignore the IR sensors for a bit so we do not
// re-trigger on the same intersection.
unsigned long irCooldownUntil = 0;
const unsigned long IR_COOLDOWN_MS = 2000;

// --- Ultrasonic (used on even turns) ---
const uint8_t US_TRIG = 44; // moved off 51 (SPI MOSI, used by Pixy2)
const uint8_t US_ECHO = 45; // moved off 50 (SPI MISO, used by Pixy2)

// Fire the turn once the object is this close or closer.
const float OBJECT_TRIGGER_IN = 7.75;

/*
   pulseIn() blocks until it hears an echo or the timeout expires, and a
   blocking read stalls the PID loop. The trigger distance is only ~8 in,
   so a short timeout is all that is needed: 6000 us is about 40 inches
   of round trip, which caps the worst-case stall at 6 ms.
*/
const unsigned long US_TIMEOUT_US = 6000UL;

// Poll the ultrasonic on an interval instead of every loop, and require
// a few consecutive in-range readings to reject noise. This replaces the
// 3-sample averaging from the nav sketch, which stalled the loop.
const unsigned long US_POLL_INTERVAL_MS = 40;
const uint8_t US_CONFIRM_COUNT = 2;

unsigned long lastUsPollTime = 0;
uint8_t usHitStreak = 0;

unsigned long usCooldownUntil = 0;
const unsigned long US_COOLDOWN_MS = 10000;

int lastOffset = 0;
bool haveLastOffset = false;

// ======================================================
// MOTION TUNING
// ======================================================

const int MAX_SPEED = 50;
const int BASE_SPEED = 25;

float Kp = 0.06;
float Kd = 0.025;

// --- Turn motion (from the nav sketch) ---
const int TURN_SPEED = 50; // pivot speed, <= MAX_SPEED
const int FORWARD_BEFORE_TURN_SPEED = 50;
const unsigned long FORWARD_BEFORE_TURN_MS = 600;

// Ignore the QTR at the start of a pivot so we do not immediately
// re-see the line we are leaving.
const unsigned long BLIND_TIME_MS = 1500;

// Safety cutoff if the line is never found.
const unsigned long MAX_TURN_MS = 2500;

/*
   "Sees the line" = a sensor reads more than this, where calibration
   scales 0 = white and 1000 = black. Calibration does the scaling, so
   this is not a raw value you have to re-tune per surface.
*/
const uint16_t ON_LINE_LEVEL = 800;
const uint8_t MIN_SENSORS_ON_LINE = 2;

/*
   After the arm finishes, drive forward for this long with vision
   ignored. Without it the robot would immediately re-detect the same
   plant it just serviced and trigger the arm again.
*/
const unsigned long CLEAR_PLANT_MS = 1500;

unsigned long clearStartTime = 0;

// ======================================================
// TURN SEQUENCE
// ======================================================

enum TurnDirection
{
    TURN_LEFT,
    TURN_RIGHT,
    SPIN_180
};

const TurnDirection turnSequence[] =
    {
        TURN_LEFT, TURN_LEFT, SPIN_180, TURN_LEFT, TURN_LEFT,
        SPIN_180};

const int NUM_TURNS = sizeof(turnSequence) / sizeof(turnSequence[0]);

int turnIndex = 0;

// ======================================================
// STATE MACHINE
// ======================================================

enum RunState
{
    STATE_DRIVING,     // PID line following, watching for plants and turns
    STATE_ARM_RUNNING, // stopped, Uno is working the arm
    STATE_CLEARING     // driving past the serviced plant, vision ignored
};

RunState runState = STATE_DRIVING;

// ======================================================
// PID VARIABLES
// ======================================================

int lastError = 0;

unsigned long lastDebugTime = 0;
const unsigned long DEBUG_INTERVAL_MS = 200;

// ======================================================
// SETUP
// ======================================================

void setup()
{
    Serial.begin(115200);

    setupMotorPins();
    stopMotors();

    // Idle HIGH means "do not run the arm".
    pinMode(ARM_GO_PIN, OUTPUT);
    digitalWrite(ARM_GO_PIN, HIGH);

    pinMode(ARM_DONE_PIN, INPUT_PULLUP);

    pinMode(LEFT_IR, INPUT);

    pinMode(US_TRIG, OUTPUT);
    pinMode(US_ECHO, INPUT);
    digitalWrite(US_TRIG, LOW);

    pixy.init();
    pixy.setLamp(1, 1);

    qtr.setTypeRC();
    qtr.setSensorPins(sensorPins, SensorCount);
    qtr.setEmitterPin(QTR_EMITTER_PIN);

    calibrateQTR();

    runState = STATE_DRIVING;
    lastError = 0;
    detectStreak = 0;
    usHitStreak = 0;
    turnIndex = 0;
}

// ======================================================
// MAIN LOOP
// ======================================================

void loop()
{
    switch (runState)
    {
    case STATE_DRIVING:
        handleDriving();
        break;

    case STATE_ARM_RUNNING:
        handleArmRunning();
        break;

    case STATE_CLEARING:
        handleClearing();
        break;
    }
}

// ======================================================
// STATE HANDLERS
// ======================================================

void handleDriving()
{
    // Plants take priority: stop and service before anything else.
    if (plantIsCentered())
    {
        Serial.println("Plant is centered");
        // Tell the Uno to run the arm.
        digitalWrite(ARM_GO_PIN, LOW);

        runState = STATE_ARM_RUNNING;
        detectStreak = 0;
        stopMotors();
        // ln(F("Plant centred. Stopping. Arm requested."));
        return;
    }

    // checkTurnTriggers() blocks through the whole pivot if it fires,
    // and leaves the motors stopped with PID reset when it returns.
    if (checkTurnTriggers())
    {
        return;
    }

    followLinePID();
}

void handleArmRunning()
{
    // Hold still while the Uno works.

    if (digitalRead(ARM_DONE_PIN) == LOW)
    {
        // Release the request. The Uno will clear its done line in response.
        digitalWrite(ARM_GO_PIN, HIGH);

        clearStartTime = millis();
        lastError = 0;

        runState = STATE_CLEARING;
    }
}

void handleClearing()
{
    /*
       Keep line following, but ignore the camera. This drives the
       serviced plant out of frame so it cannot re-trigger the arm.

       Turn triggers are also ignored here. If a plant can sit close
       enough to an intersection that the robot is still clearing it
       when the turn marker arrives, move checkTurnTriggers() into
       this handler as well.
    */

    if (millis() - clearStartTime >= CLEAR_PLANT_MS)
    {
        runState = STATE_DRIVING;
        detectStreak = 0;

        // ln(F("Plant cleared. Watching again."));
        return;
    }

    followLinePID();
}

// ======================================================
// TURN TRIGGERS
// ======================================================

// Even turnIndex (0, 2, 4...) -> ultrasonic triggers the turn
// Odd  turnIndex (1, 3, 5...) -> IR triggers the turn
bool nextTurnUsesDistance()
{
    if (turnIndex >= NUM_TURNS)
        return false;
    return turnSequence[turnIndex] == SPIN_180;
}

// Returns true if a turn was executed this call.
bool checkTurnTriggers()
{
    if (turnIndex >= NUM_TURNS)
    {
        return false;
    }

    if (nextTurnUsesDistance())
    {
        if (millis() < usCooldownUntil)
        {
            return false;
        }

        if (objectInRange())
        {
            // ln(F("Object in range - triggering turn"));
            executeNextTurn();
            return true;
        }
    }
    else
    {
        if (millis() < irCooldownUntil)
        {
            return false;
        }

        if (digitalRead(LEFT_IR))
        {
            // ln(F("IR marker - triggering turn"));
            executeNextTurn();
            return true;
        }
    }

    return false;
}

/*
   Polled, single-ping ultrasonic check.

   The nav sketch used a +/- window around 7 in. A window can be driven
   straight through between polls, so this fires on "at or closer than
   the trigger distance" instead, confirmed over a few consecutive reads.
*/
bool objectInRange()
{
    if (millis() - lastUsPollTime < US_POLL_INTERVAL_MS)
    {
        return false;
    }

    lastUsPollTime = millis();

    float distance = readDistanceInches();

    if (distance > 0 && distance <= OBJECT_TRIGGER_IN)
    {
        usHitStreak++;
    }
    else
    {
        usHitStreak = 0;
    }

    if (usHitStreak >= US_CONFIRM_COUNT)
    {
        //(F("Object at "));
        //(distance);
        // ln(F(" in"));
        return true;
    }

    return false;
}

// Returns distance in inches, or -1 if no echo (out of range / bad reading).
float readDistanceInches()
{
    digitalWrite(US_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(US_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_TRIG, LOW);

    unsigned long duration = pulseIn(US_ECHO, HIGH, US_TIMEOUT_US);

    if (duration == 0)
    {
        return -1;
    }

    return duration / 74.0 / 2.0; // microseconds -> inches
}

// ======================================================
// TURNING
// ======================================================

void executeNextTurn()
{
    if (turnIndex >= NUM_TURNS)
    {
        return;
    }

    if (turnSequence[turnIndex] == SPIN_180)
    {
        spin180();
    }
    else
    {
        performTurn(turnSequence[turnIndex]);
    }

    turnIndex++;
}

void performTurn(TurnDirection dir)
{
    // ln(dir == TURN_LEFT ? F("Turning LEFT") : F("Turning RIGHT"));

    // 1. Clear the intersection.
    setLeftMotors(FORWARD_BEFORE_TURN_SPEED);
    setRightMotors(FORWARD_BEFORE_TURN_SPEED);
    delay(FORWARD_BEFORE_TURN_MS);

    // 2. Start pivoting in place.
    if (dir == TURN_LEFT)
    {
        setLeftMotors(-TURN_SPEED);
        setRightMotors(TURN_SPEED);
    }
    else
    {
        setLeftMotors(TURN_SPEED);
        setRightMotors(-TURN_SPEED);
    }

    // 3. Keep pivoting until the QTR sees the line (or we time out).
    unsigned long startTime = millis();
    bool found = false;

    while (millis() - startTime < MAX_TURN_MS)
    {
        if (millis() - startTime >= BLIND_TIME_MS && lineDetected())
        {
            found = true;
            break;
        }
    }

    // 4. Stop and hand control back to PID.
    stopMotors();
    // ln(found ? F("Line found - resuming PID")
    //: F("TURN TIMEOUT - no line found"));

    lastError = 0;
    detectStreak = 0;
    usHitStreak = 0;

    irCooldownUntil = millis() + IR_COOLDOWN_MS;
    usCooldownUntil = millis() + US_COOLDOWN_MS;
}

// True once the QTR array sees the line again.
bool lineDetected()
{
    qtr.readCalibrated(sensorValues); // 0 = calibrated white, 1000 = calibrated black

    uint8_t count = 0;

    for (uint8_t i = 0; i < SensorCount; i++)
    {
        if (sensorValues[i] > ON_LINE_LEVEL)
        {
            count++;
        }
    }

    return count >= MIN_SENSORS_ON_LINE;
}

// ======================================================
// PIXY2 DETECTION
// ======================================================

/*
   Returns true once the largest signature block has been within
   CENTER_TOLERANCE of the frame centre for DETECT_THRESHOLD
   consecutive checks.
*/
bool plantIsCentered()
{
    if (millis() - lastPixyTime < PIXY_INTERVAL_MS)
        return false;
    lastPixyTime = millis();

    int8_t res = pixy.ccc.getBlocks(false);

    // No new frame ready yet -> no new information. Don't stomp the state.
    if (res == PIXY_RESULT_BUSY || res < 0)
        return false;

    long bestArea = 0;
    int bestX = -1;

    for (int i = 0; i < pixy.ccc.numBlocks; i++)
    {
        if (pixy.ccc.blocks[i].m_signature != SIG_PLANT)
            continue;

        long area = (long)pixy.ccc.blocks[i].m_width *
                    (long)pixy.ccc.blocks[i].m_height;
        if (area < MIN_BLOCK_AREA)
            continue;

        if (area > bestArea)
        {
            bestArea = area;
            bestX = pixy.ccc.blocks[i].m_x;
        }
    }

    if (bestX < 0) // genuinely nothing in view
    {
        haveLastOffset = false;
        return false;
    }

    int offset = bestX - FRAME_CENTER_X;

    bool crossed = haveLastOffset &&
                   ((lastOffset < 0 && offset >= 0) ||
                    (lastOffset > 0 && offset <= 0));

    lastOffset = offset;
    haveLastOffset = true;

    Serial.print(F("x="));
    Serial.print(bestX);
    Serial.print(F(" off="));
    Serial.print(offset);
    Serial.print(F(" area="));
    Serial.println(bestArea);

    if (crossed || abs(offset) <= CENTER_TOLERANCE)
    {
        haveLastOffset = false; // reseed after firing
        return true;
    }
    return false;
}

// ======================================================
// PID LINE FOLLOWING
// ======================================================

void followLinePID()
{
    // For a black line on a light surface.
    uint16_t position = qtr.readLineBlack(sensorValues);

    int error = (int)position - LINE_CENTER;

    int derivative = error - lastError;

    float correctionFloat =
        (Kp * error) +
        (Kd * derivative);

    int correction = (int)correctionFloat;

    // Prevent an excessive turn command.
    correction = constrain(
        correction,
        -MAX_SPEED,
        MAX_SPEED);

    /*
       If the line is to the right:
         error is positive
         left motors speed up
         right motors slow down

       This turns the robot toward the right.
    */

    int leftSpeed = BASE_SPEED - correction;
    int rightSpeed = BASE_SPEED + correction;

    leftSpeed = constrain(
        leftSpeed,
        -MAX_SPEED,
        MAX_SPEED);

    rightSpeed = constrain(
        rightSpeed,
        -MAX_SPEED,
        MAX_SPEED);

    setLeftMotors(leftSpeed);
    setRightMotors(rightSpeed);

    lastError = error;

    printPIDDebug(
        position,
        error,
        correction,
        leftSpeed,
        rightSpeed);
}

// ======================================================
// MOTOR CONTROL
// ======================================================

void setupMotorPins()
{
    pinMode(M1_PWM, OUTPUT);
    pinMode(M1_DIR, OUTPUT);

    pinMode(M2_PWM, OUTPUT);
    pinMode(M2_DIR, OUTPUT);

    pinMode(M3_PWM, OUTPUT);
    pinMode(M3_DIR, OUTPUT);

    pinMode(M4_PWM, OUTPUT);
    pinMode(M4_DIR, OUTPUT);
}

void setLeftMotors(int speed)
{
    setMotor(M1_PWM, M1_DIR, M1_FORWARD_HIGH, speed);
    setMotor(M3_PWM, M3_DIR, M3_FORWARD_HIGH, speed);
}

void setRightMotors(int speed)
{
    setMotor(M2_PWM, M2_DIR, M2_FORWARD_HIGH, speed);
    setMotor(M4_PWM, M4_DIR, M4_FORWARD_HIGH, speed);
}

void setMotor(
    uint8_t pwmPin,
    uint8_t dirPin,
    bool forwardIsHigh,
    int speed)
{
    speed = constrain(speed, -255, 255);

    if (speed == 0)
    {
        analogWrite(pwmPin, 0);
        return;
    }

    bool moveForward = speed > 0;

    bool directionLevel;

    if (moveForward)
    {
        directionLevel = forwardIsHigh;
    }
    else
    {
        directionLevel = !forwardIsHigh;
    }

    digitalWrite(
        dirPin,
        directionLevel ? HIGH : LOW);

    analogWrite(
        pwmPin,
        abs(speed));
}

void stopMotors()
{
    analogWrite(M1_PWM, 0);
    analogWrite(M2_PWM, 0);
    analogWrite(M3_PWM, 0);
    analogWrite(M4_PWM, 0);
}

// ======================================================
// QTR CALIBRATION
// ======================================================

void calibrateQTR()
{
    /*
       The sensor must see both the black line and the white surface
       during calibration.

       The motor sweep is currently commented out, so the robot must
       be moved across the line by hand while this runs.
    */

    // const int CALIBRATION_SPEED = 25;
    const int CALIBRATION_SAMPLES = 160;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        //   if (i < 40)
        //   {
        //     // Turn left.
        //     setLeftMotors(-CALIBRATION_SPEED);
        //     setRightMotors(CALIBRATION_SPEED);
        //   }
        //   else if (i < 120)
        //   {
        //     // Turn right.
        //     setLeftMotors(CALIBRATION_SPEED);
        //     setRightMotors(-CALIBRATION_SPEED);
        //   }
        //   else
        //   {
        //     // Return left toward the original position.
        //     setLeftMotors(-CALIBRATION_SPEED);
        //     setRightMotors(CALIBRATION_SPEED);
        //   }

        qtr.calibrate();
        delay(20);
    }

    stopMotors();
    delay(500);

    printCalibrationValues();
}

void printCalibrationValues()
{
    // ln(F("Calibration minimums:"));

    for (uint8_t i = 0; i < SensorCount; i++)
    {
        //(qtr.calibrationOn.minimum[i]);
        //('\t');
    }

    // ln();
    // ln(F("Calibration maximums:"));

    for (uint8_t i = 0; i < SensorCount; i++)
    {
        //(qtr.calibrationOn.maximum[i]);
        //('\t');
    }

    // ln();
}

// ======================================================
// DEBUG OUTPUT
// ======================================================

void printPIDDebug(
    uint16_t position,
    int error,
    int correction,
    int leftSpeed,
    int rightSpeed)
{
    if (millis() - lastDebugTime < DEBUG_INTERVAL_MS)
    {
        return;
    }

    lastDebugTime = millis();

    //(F("Position: "));
    //(position);

    //(F(" Error: "));
    //(error);

    //(F(" Correction: "));
    //(correction);

    //(F(" Left: "));
    //(leftSpeed);

    //(F(" Right: "));
    // ln(rightSpeed);
}

// ======================================================
// 180 SPIN
// ======================================================

// Time to sweep 180 degrees at TURN_SPEED. Tune on the field:
// run it, see how far past (or short of) reversed it lands, adjust.
const unsigned long SPIN_180_MS = 4766;

// Which way the robot rotates during a 180. Pick whichever
// clears your row end without clipping the plants.
const bool SPIN_180_GOES_LEFT = true;

void spin180()
{
    // ln(SPIN_180_GOES_LEFT ? F("Spinning 180 LEFT")
    //: F("Spinning 180 RIGHT"));

    if (SPIN_180_GOES_LEFT)
    {
        setLeftMotors(-TURN_SPEED - 13);
        setRightMotors(TURN_SPEED);
    }
    else
    {
        setLeftMotors(TURN_SPEED);
        setRightMotors(-TURN_SPEED - 13);
    }

    delay(SPIN_180_MS);
    stopMotors();

    lastError = 0;
    detectStreak = 0;
    usHitStreak = 0;

    irCooldownUntil = millis() + IR_COOLDOWN_MS;
    usCooldownUntil = millis() + US_COOLDOWN_MS;

    // ln(F("Spin complete."));
}