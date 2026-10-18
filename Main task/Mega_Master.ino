#include <QTRSensors.h>
#include <Pixy2.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

Pixy2 pixy;

// ======================================================
// TEST MODE
// ======================================================

/*
   ARM TEST MODE: set ENABLE_DRIVING to false to test only the camera
   and the arms. With it false:
     - the QTR is not set up or calibrated
     - the line-following PID never runs
     - the drive motors are never touched
   The robot just sits still, watches for S1 in the arm windows, and
   sends the GO command to the Uno.

   Set it back to true to restore normal driving.
*/
const bool ENABLE_DRIVING = true;

// ======================================================
// MOTOR PINS
// ======================================================

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
   All lines are active LOW.

   The receiving side uses INPUT_PULLUP, so a board that has not
   finished booting (pins floating as inputs) is read as HIGH, which
   means "not asserted". Neither board can trigger the other by
   accident during power-up.

   There is one GO line per arm and one shared DONE line:
     ARM1_GO_PIN  (40) -> Uno D2   : run arm 1 (Uno M1)
     ARM2_GO_PIN  (42) -> Uno D10  : run arm 2 (Uno M2)
     ARM_DONE_PIN (41) <- Uno D9   : arm finished
*/

const uint8_t ARM1_GO_PIN = 40;  // OUTPUT to Uno D2
const uint8_t ARM2_GO_PIN = 42;  // OUTPUT to Uno D10
const uint8_t ARM_DONE_PIN = 41; // INPUT from Uno D9

enum ArmSelect
{
    ARM_NONE,
    ARM_1,
    ARM_2
};

// Which arm the current/last command was sent to.
ArmSelect activeArm = ARM_NONE;

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

// With six sensors, the line position runs 0-5000, so the middle
// of the array is 2500.
const int LINE_CENTER = 2500;

// ======================================================
// PIXY2 VISION
// ======================================================

const uint8_t SIG_PLANT = 1;

/*
   Pixy2's colour-connected-components frame is 316 x 208.
   Block x values run 0 to 315.

   Arm selection by where S1 sits in the frame:
     x in ARM1 window -> run arm 1 (M1 on the Uno)
     x in ARM2 window -> run arm 2 (M2 on the Uno)
   Bounds are inclusive.
*/
const int ARM1_X_MIN = 80;
const int ARM1_X_MAX = 90;

const int ARM2_X_MIN = 200;
const int ARM2_X_MAX = 280;

// Reject small blocks: noise, distant plants, stray pixels.
const int MIN_BLOCK_AREA = 300; // width * height, in pixels

// Pixy2 runs at ~60 fps. Polling faster than that wastes loop time.
unsigned long lastPixyTime = 0;
const unsigned long PIXY_INTERVAL_MS = 20;

// Log the S1 (plant) x coordinate while it is in view. Throttled so the
// serial output does not slow the line-following loop.
const bool LOG_S1_X = true;
const unsigned long S1_LOG_INTERVAL_MS = 100;
unsigned long lastS1LogTime = 0;
bool s1WasVisible = false;

// What caused the most recent arm command (used for logging only).
int triggerX = 0;
long triggerArea = 0;

// ======================================================
// MOTION TUNING
// ======================================================

const int MAX_SPEED = 60;
const int BASE_SPEED = 45;

float Kp = 0.06;
float Kd = 0.025;

// --- Turn motion ---
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
   CAMERA_BLIND_TIME (milliseconds): after an arm finishes, the camera
   is completely ignored (the Pixy is not even polled) for this long.
   Set it long enough for the serviced plant to leave the arm windows.
   When driving, the robot keeps line following during this time. In
   arm test mode the robot does not move, so a plant that stays in a
   window will trigger again once this time is up.

   Edit this value to tune it.
*/
const unsigned long CAMERA_BLIND_TIME = 1000;

unsigned long blindStartTime = 0;

// When the GO command was sent (used to log how long the arm took).
unsigned long armCommandTime = 0;
uint16_t armCommandCount = 0;

// ======================================================
// TURN SEQUENCE
// ====================================================

// ======================================================
// STATE MACHINE
// ======================================================

enum RunState
{
    STATE_DRIVING,     // PID line following, watching for plants
    STATE_ARM_RUNNING, // stopped, Uno is working the arm
    STATE_CAMERA_BLIND // arm done, camera ignored for CAMERA_BLIND_TIME
};

RunState runState = STATE_DRIVING;

// ======================================================
// PID VARIABLES
// ======================================================

int lastError = 0;

unsigned long lastDebugTime = 0;
const unsigned long DEBUG_INTERVAL_MS = 200;

// ---------- Counting settings (edit these) ----------

const uint8_t SIG_YELLOW = 1;
const uint8_t SIG_GREEN = 2;

const int TOTAL_HOLDERS = 10; // 5 per row, 2 rows

// Ignore blocks smaller than this (width * height, pixels).
const int COUNT_MIN_AREA = 300;

// A block must have been tracked by the Pixy for this many frames before
// it counts (rejects one-frame flickers). ~60 frames per second.
const uint8_t COUNT_MIN_AGE = 40;

// After a counted block has been missing for this many camera polls, its
// tracking index may be reused, so it can be counted again.
const uint8_t COUNT_FORGET_POLLS = 10;

// ---------- IR end-of-run settings (edit these) ----------
const uint8_t IR_END_PIN = 23;

// Level the IR sensor outputs while it is over the white line.
// Your old IR code treated HIGH as "marker seen". Change to LOW if yours
// is the other way round.
const uint8_t IR_LINE_LEVEL = LOW;

// The IR must see the line continuously for this long to count (ms).
const unsigned long IR_CONFIRM_MS = 20;

// ---------- State ----------

uint16_t yellowCount = 0;
uint16_t greenCount = 0;

bool countedIndex[256];   // this Pixy tracking index has already been counted
uint8_t missedPolls[256]; // polls in a row this counted index was not seen

uint8_t irLineCount = 0; // 0 = none yet, 1 = spun around, 2 = finished
bool irArmed = false;    // IR has been off the line long enough to count
bool irSeeing = false;
bool irClear = false;
unsigned long irSeenSince = 0;
unsigned long irClearSince = 0;
unsigned long irIgnoreUntil = 0;

bool runFinished = false;

// ---------- Counting ----------

/*
   Call right after a successful pixy.ccc.getBlocks(). Each tracked object
   (Pixy tracking index) is counted once, however many frames it stays in
   view.
*/
void countPlantsInFrame()
{
    bool seen[256] = {false};

    for (int i = 0; i < pixy.ccc.numBlocks; i++)
    {
        uint8_t idx = pixy.ccc.blocks[i].m_index;
        seen[idx] = true;

        uint16_t sig = pixy.ccc.blocks[i].m_signature;
        if (sig != SIG_YELLOW && sig != SIG_GREEN)
            continue;

        long area = (long)pixy.ccc.blocks[i].m_width *
                    (long)pixy.ccc.blocks[i].m_height;
        if (area < COUNT_MIN_AREA)
            continue;

        if (pixy.ccc.blocks[i].m_age < COUNT_MIN_AGE)
            continue;

        if (countedIndex[idx])
            continue;

        countedIndex[idx] = true;
        missedPolls[idx] = 0;

        if (sig == SIG_YELLOW) {
            yellowCount++;
            greenCount--;
        } else {
            greenCount++;
        }

        Serial.print('[');
        Serial.print(millis());
        Serial.print(F(" ms] COUNTED "));
        Serial.print(sig == SIG_YELLOW ? F("YELLOW") : F("GREEN"));
        Serial.print(F(" (x="));
        Serial.print(pixy.ccc.blocks[i].m_x);
        Serial.print(F(", index="));
        Serial.print(idx);
        Serial.print(F("). Yellow="));
        Serial.print(yellowCount);
        Serial.print(F(" Green="));
        Serial.println(greenCount);
    }

    // Forget counted objects that have left the view, so a reused
    // tracking index can be counted again.
    for (int idx = 0; idx < 256; idx++)
    {
        if (!countedIndex[idx])
            continue;

        if (seen[idx])
        {
            missedPolls[idx] = 0;
        }
        else if (++missedPolls[idx] >= COUNT_FORGET_POLLS)
        {
            countedIndex[idx] = false;
        }
    }
}

/*
   Polls the Pixy and counts. Use this only where plantArmToRun() is NOT
   polling the camera (the camera blind time), so the Pixy is never
   polled twice per loop.
*/
void pollPixyForCounting()
{
    if (millis() - lastPixyTime < PIXY_INTERVAL_MS)
        return;
    lastPixyTime = millis();

    int8_t res = pixy.ccc.getBlocks(false);

    if (res == PIXY_RESULT_BUSY || res < 0)
        return;

    countPlantsInFrame();
}

// ---------- End of run (IR) ----------

// Call once in setup().
void initEndSensor()
{
    pinMode(IR_END_PIN, INPUT);
}

/*
   Watches the IR sensor for the white line.
     1st confirmed crossing -> spin180() in place, then keep driving.
     2nd confirmed crossing -> finishRun().

   Returns true if it took over the robot this call (it spun or finished),
   so loop() should skip the rest of this iteration.

   Call it while driving or while the camera is blind; do not call it
   while the arm is running.
*/
bool handleIrEndLogic()
{
    if (runFinished)
        return true;

    if (digitalRead(IR_END_PIN) != IR_LINE_LEVEL)
        return false;

    irLineCount++;

    Serial.print('[');
    Serial.print(millis());
    Serial.print(F(" ms] IR saw the white line (crossing #"));
    Serial.print(irLineCount);
    Serial.println(')');

    if (irLineCount == 1)
    {
        Serial.println(F("  -> spinning 180 in place"));
        stopMotors();
        spin180();
        return true;
    }

    finishRun();
    return true;
}

/*
   Stops the robot, shows the counts on the LCD and lights every LED.
   Safe to call more than once.
*/
void finishRun()
{
    if (runFinished)
        return;
    runFinished = true;

    stopMotors();

    int doubles = yellowCount;
    int singles = (int)greenCount - (int)yellowCount;
    int empties = TOTAL_HOLDERS - (int)greenCount;

    showStatus(doubles, singles, empties);
    setAllLeds(true);

    Serial.print('[');
    Serial.print(millis());
    Serial.print(F(" ms] END OF RUN. Yellow="));
    Serial.print(yellowCount);
    Serial.print(F(" Green="));
    Serial.print(greenCount);
    Serial.print(F(" -> D="));
    Serial.print(doubles);
    Serial.print(F(" S="));
    Serial.print(singles);
    Serial.print(F(" E="));
    Serial.println(empties);
}

// ======================================================
// SETUP
// ======================================================

void setup()
{
    initStatusIndicators();
    Serial.begin(115200);

    if (ENABLE_DRIVING)
    {
        setupMotorPins();
        stopMotors();
    }

    // Idle HIGH means "do not run the arm". Write HIGH before switching
    // to OUTPUT so the line never glitches LOW.
    digitalWrite(ARM1_GO_PIN, HIGH);
    pinMode(ARM1_GO_PIN, OUTPUT);
    digitalWrite(ARM2_GO_PIN, HIGH);
    pinMode(ARM2_GO_PIN, OUTPUT);

    pinMode(ARM_DONE_PIN, INPUT_PULLUP);

    pixy.init();
    initEndSensor();
    // pixy.setLamp(1, 1);

    if (ENABLE_DRIVING)
    {
        qtr.setTypeRC();
        qtr.setSensorPins(sensorPins, SensorCount);
        qtr.setEmitterPin(QTR_EMITTER_PIN);

        calibrateQTR();
    }

    Serial.println(F("=== Mega master ready ==="));
    if (!ENABLE_DRIVING)
    {
        Serial.println(F("ARM TEST MODE: QTR and drive motors disabled."));
    }
    Serial.print(F("Arm 1 window: x "));
    Serial.print(ARM1_X_MIN);
    Serial.print('-');
    Serial.print(ARM1_X_MAX);
    Serial.println(F(" -> GO pin 40 (Uno D2)"));
    Serial.print(F("Arm 2 window: x "));
    Serial.print(ARM2_X_MIN);
    Serial.print('-');
    Serial.print(ARM2_X_MAX);
    Serial.println(F(" -> GO pin 42 (Uno D10)"));
    Serial.print(F("Camera blind time after each arm: "));
    Serial.print(CAMERA_BLIND_TIME);
    Serial.println(F(" ms"));

    runState = STATE_DRIVING;
    lastError = 0;
}

// ======================================================
// MAIN LOOP
// ======================================================

void loop()
{
    if (runFinished)
        return;                                             
    if (runState != STATE_ARM_RUNNING && handleIrEndLogic())
        return;  
    switch (runState)
    {
    case STATE_DRIVING:
        handleDriving();
        break;

    case STATE_ARM_RUNNING:
        handleArmRunning();
        break;

    case STATE_CAMERA_BLIND:
        pollPixyForCounting();
        handleCameraBlind();
        break;
    }
}

// ======================================================
// ARM HELPERS
// ======================================================

uint8_t goPinFor(ArmSelect arm)
{
    return (arm == ARM_1) ? ARM1_GO_PIN : ARM2_GO_PIN;
}

// ======================================================
// STATE HANDLERS
// ======================================================

void handleDriving()
{
    // Plants take priority: stop and service before anything else.
    ArmSelect arm = plantArmToRun();

    if (arm != ARM_NONE)
    {
        stopMotors();

        activeArm = arm;
        uint8_t goPin = goPinFor(arm);

        // Tell the Uno which arm to run.
        digitalWrite(goPin, LOW);

        armCommandTime = millis();
        armCommandCount++;

        Serial.print('[');
        Serial.print(armCommandTime);
        Serial.print(F(" ms] ARM COMMAND #"));
        Serial.print(armCommandCount);
        Serial.print(F(": S1 x="));
        Serial.print(triggerX);
        Serial.print(F(" (area="));
        Serial.print(triggerArea);
        Serial.print(F(") is in the arm "));
        Serial.print(arm == ARM_1 ? 1 : 2);
        Serial.print(F(" window ("));
        Serial.print(arm == ARM_1 ? ARM1_X_MIN : ARM2_X_MIN);
        Serial.print('-');
        Serial.print(arm == ARM_1 ? ARM1_X_MAX : ARM2_X_MAX);
        Serial.print(F("). Sending GO on pin "));
        Serial.print(goPin);
        Serial.print(F(" -> LOW (Uno runs arm "));
        Serial.print(arm == ARM_1 ? F("1 / M1") : F("2 / M2"));
        Serial.println(F("). Motors stopped, waiting for DONE."));

        runState = STATE_ARM_RUNNING;
        return;
    }

    if (ENABLE_DRIVING)
    {
        followLinePID();
    }
}

void handleArmRunning()
{
    // Hold still while the Uno works.

    if (digitalRead(ARM_DONE_PIN) == LOW)
    {
        // Release the request. The Uno will clear its done line in response.
        digitalWrite(goPinFor(activeArm), HIGH);

        unsigned long now = millis();

        Serial.print('[');
        Serial.print(now);
        Serial.print(F(" ms] ARM COMMAND #"));
        Serial.print(armCommandCount);
        Serial.print(F(": DONE (pin 41) received after "));
        Serial.print(now - armCommandTime);
        Serial.print(F(" ms. GO on pin "));
        Serial.print(goPinFor(activeArm));
        Serial.println(F(" released (HIGH)."));

        Serial.print('[');
        Serial.print(now);
        Serial.print(F(" ms] Camera blind for "));
        Serial.print(CAMERA_BLIND_TIME);
        Serial.println(F(" ms (not looking for S1)."));

        blindStartTime = now;
        lastError = 0;

        runState = STATE_CAMERA_BLIND;
    }
}

void handleCameraBlind()
{
    /*
       Keep line following (when driving is enabled), but do not look at
       the camera at all. This gives the serviced plant time to leave the
       arm windows so it cannot re-trigger an arm.
    */

    if (millis() - blindStartTime >= CAMERA_BLIND_TIME)
    {
        Serial.print('[');
        Serial.print(millis());
        Serial.println(F(" ms] Camera blind time over. Looking for S1 again."));

        s1WasVisible = false; // so a stale "S1 lost" is not printed
        runState = STATE_DRIVING;
        return;
    }

    if (ENABLE_DRIVING)
    {
        followLinePID();
    }
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

// Which arm's window (if any) an x value falls in.
ArmSelect armForX(int x)
{
    //if (x >= ARM1_X_MIN && x <= ARM1_X_MAX)
    //    return ARM_1;
    if (x >= ARM2_X_MIN && x <= ARM2_X_MAX)
        return ARM_2;
    return ARM_NONE;
}

/*
   Looks at every S1 block big enough to count. If any block's x is in
   the arm 1 or arm 2 window, returns that arm. If several qualify,
   the largest block wins.

   Also logs the x of the largest S1 block in view (throttled).
*/
ArmSelect plantArmToRun()
{
    if (millis() - lastPixyTime < PIXY_INTERVAL_MS)
        return ARM_NONE;
    lastPixyTime = millis();

    int8_t res = pixy.ccc.getBlocks(false);

    // No new frame ready yet -> no new information. Don't stomp the state.
    if (res == PIXY_RESULT_BUSY || res < 0)
        return ARM_NONE;
    
    countPlantsInFrame();
    long bestArea = 0; // largest S1 block overall (for logging)
    int bestX = -1;

    long hitArea = 0; // largest S1 block inside an arm window
    int hitX = -1;
    ArmSelect hitArm = ARM_NONE;

    for (int i = 0; i < pixy.ccc.numBlocks; i++)
    {
        if (pixy.ccc.blocks[i].m_signature != SIG_PLANT)
            continue;

        long area = (long)pixy.ccc.blocks[i].m_width *
                    (long)pixy.ccc.blocks[i].m_height;
        if (area < MIN_BLOCK_AREA)
            continue;

        int x = pixy.ccc.blocks[i].m_x;

        if (area > bestArea)
        {
            bestArea = area;
            bestX = x;
        }

        ArmSelect arm = armForX(x);
        if (arm != ARM_NONE && area > hitArea)
        {
            hitArea = area;
            hitX = x;
            hitArm = arm;
        }
    }

    if (bestX < 0) // genuinely nothing in view
    {
        if (LOG_S1_X && s1WasVisible)
        {
            Serial.print('[');
            Serial.print(millis());
            Serial.println(F(" ms] S1 lost (no block in view)"));
        }
        s1WasVisible = false;
        return ARM_NONE;
    }

    if (LOG_S1_X)
    {
        // Always log the first frame S1 appears, then at most every
        // S1_LOG_INTERVAL_MS after that.
        if (!s1WasVisible || millis() - lastS1LogTime >= S1_LOG_INTERVAL_MS)
        {
            lastS1LogTime = millis();

            Serial.print('[');
            Serial.print(lastS1LogTime);
            Serial.print(F(" ms] S1 x="));
            Serial.print(bestX);
            Serial.print(F(" area="));
            Serial.println(bestArea);
        }
    }
    s1WasVisible = true;

    if (hitArm != ARM_NONE)
    {
        // Remember what fired, so the command log can report it.
        triggerX = hitX;
        triggerArea = hitArea;
        return hitArm;
    }

    return ARM_NONE;
}

// ======================================================
// PID LINE FOLLOWING
// ======================================================

void followLinePID()
{
    uint16_t position = qtr.readLineWhite(sensorValues);

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
    if (!ENABLE_DRIVING)
        return;

    setMotor(M1_PWM, M1_DIR, M1_FORWARD_HIGH, speed);
    setMotor(M3_PWM, M3_DIR, M3_FORWARD_HIGH, speed);
}

void setRightMotors(int speed)
{
    if (!ENABLE_DRIVING)
        return;

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
    if (!ENABLE_DRIVING)
        return;

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

    const int CALIBRATION_SAMPLES = 160;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        qtr.calibrate();
        delay(20);
    }

    stopMotors();
    delay(500);
}

// ======================================================
// 180 SPIN (currently not triggered by anything)
// ======================================================

// Which way the robot rotates during a 180. Pick whichever
// clears your row end without clipping the plants.
const bool SPIN_180_GOES_LEFT = false;

// Speed of the 180 spin (0-255). Independent of TURN_SPEED, BASE_SPEED and MAX_SPEED.
const int SPIN_180_SPEED = 80;

// The QTR is ignored for this long at the start of the spin, so it does not
// see the line the robot is leaving (or a crossing line) and stop early.
// Your old timed spin took about 2900 ms at speed 80, so 1500 ms keeps the
// QTR blind for roughly the first half of the turn. Tune it from the log
// line the spin prints.
const unsigned long SPIN_QTR_BLIND_MS = 1500;

// A sensor "sees the white line" when its calibrated value is below this
// (calibration scales 0 = white, 1000 = black).
const uint16_t SPIN_LINE_SEEN_BELOW = 500;

// How many sensors must see the line to end the spin.
const uint8_t SPIN_MIN_SENSORS = 2;

// Safety only: stop spinning after this long even if the line was never
// found, so the robot cannot spin forever. Make it large (but not huge).
const unsigned long SPIN_MAX_MS = 8000;

// True when enough QTR sensors are over the WHITE line.
// (lineDetected() above looks for dark, which is the wrong way round for a
// white line, so the spin uses this instead.)
bool whiteLineSeen()
{
    qtr.readCalibrated(sensorValues);

    uint8_t count = 0;

    for (uint8_t i = 0; i < SensorCount; i++)
    {
        if (sensorValues[i] < SPIN_LINE_SEEN_BELOW)
        {
            count++;
        }
    }

    return count >= SPIN_MIN_SENSORS;
}

void spin180()
{
    if (!ENABLE_DRIVING)
    {
        Serial.println(F("spin180 skipped (driving disabled)"));
        return;
    }

    if (SPIN_180_GOES_LEFT)
    {
        setLeftMotors(-SPIN_180_SPEED);
        setRightMotors(SPIN_180_SPEED);
    }
    else
    {
        setLeftMotors(SPIN_180_SPEED);
        setRightMotors(-SPIN_180_SPEED);
    }

    unsigned long startTime = millis();
    bool found = false;

    while (millis() - startTime < SPIN_MAX_MS)
    {
        if (millis() - startTime >= SPIN_QTR_BLIND_MS && whiteLineSeen())
        {
            found = true;
            break;
        }
    }

    stopMotors();

    Serial.print('[');
    Serial.print(millis());
    if (found)
    {
        Serial.print(F(" ms] Spin done: line found after "));
        Serial.print(millis() - startTime);
        Serial.println(F(" ms"));
    }
    else
    {
        Serial.print(F(" ms] SPIN TIMEOUT: line never found after "));
        Serial.print(SPIN_MAX_MS);
        Serial.println(F(" ms"));
    }

    lastError = 0;
}

// ======================= COPY FROM HERE =======================

const uint8_t LCD_ADDRESS = 0x27; // some modules use 0x3F
const uint8_t LCD_COLS = 16;
const uint8_t LCD_ROWS = 2;

// Do NOT use pins 0 or 1: they are the USB serial lines on the Mega.
const uint8_t BLUE_LED_PIN = 32;
const uint8_t YELLOW_LED_PIN = 33;
const uint8_t GREEN_LED_PIN = 2;

LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);

// Call once in setup().
void initStatusIndicators()
{
    pinMode(BLUE_LED_PIN, OUTPUT);
    pinMode(YELLOW_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    setAllLeds(false);

    lcd.init();
    lcd.backlight();
    lcd.clear();
}

void setAllLeds(bool on)
{
    digitalWrite(BLUE_LED_PIN, on ? HIGH : LOW);
    digitalWrite(YELLOW_LED_PIN, on ? HIGH : LOW);
    digitalWrite(GREEN_LED_PIN, on ? HIGH : LOW);
}

// Shows "D : <d>, S : <s>, E : <e>".
// That text is 19 characters, so a 16-column display breaks it after
// the second value:   D : 0, S : 0,
//                     E : 0
// A 20-column display shows it on one row.
void showStatus(int d, int s, int e)
{
    char whole[32];
    snprintf(whole, sizeof(whole), "D : %d, S : %d, E : %d", d, s, e);

    lcd.clear();

    if (strlen(whole) <= LCD_COLS)
    {
        lcd.setCursor(0, 0);
        lcd.print(whole);
    }
    else
    {
        char first[32];
        char second[16];
        snprintf(first, sizeof(first), "D : %d, S : %d,", d, s);
        snprintf(second, sizeof(second), "E : %d", e);

        lcd.setCursor(0, 0);
        lcd.print(first);
        lcd.setCursor(0, 1);
        lcd.print(second);
    }
}

// Lights all the LEDs and shows D : 0, S : 0, E : 0.
void indicatorTest()
{
    setAllLeds(true);
    showStatus(0, 0, 0);
}