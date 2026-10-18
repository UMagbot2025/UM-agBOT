// ======================================================
// UNO ARM SLAVE - separate command line per arm (with serial logging)
// DFRobot quad motor shield: arm 1 on M1, arm 2 on M2
// ======================================================
//
// Open the Serial Monitor (at SERIAL_BAUD, below) to watch what the Uno is doing.
// Serial uses D0/D1, so keep those pins free. Set DEBUG_LOG to false for
// a silent build.
//
// Handshake with the Mega (all lines active LOW):
//   Mega pin 40 -> Uno D2   (GO_ARM1)  LOW = extend/retract arm 1 (M1)
//   Mega pin 42 -> Uno D10  (GO_ARM2)  LOW = extend/retract arm 2 (M2)
//   Uno D9      -> Mega 41  (DONE)     Uno pulls LOW when the arm is back
//
// Sequence:
//   1. Mega pulls the GO line for ONE arm LOW.
//   2. Uno extends then retracts that arm only.
//   3. Uno pulls DONE LOW.
//   4. Mega sees DONE, releases its GO line to HIGH.
//   5. Uno sees both GO lines HIGH, releases DONE, goes back to idle.
//
// Also needed: Mega GND tied to Uno GND.

// ======================================================
// LOGGING
// ======================================================

const bool DEBUG_LOG = true;

// Serial Monitor must be set to this same baud rate.
const unsigned long SERIAL_BAUD = 9600;

// Print a status line this often so you can tell the Uno is alive.
const unsigned long HEARTBEAT_MS = 2000;

// ======================================================
// HANDSHAKE PINS
// ======================================================

const uint8_t ARM1_GO_PIN = 2;  // INPUT_PULLUP from Mega 40
const uint8_t ARM2_GO_PIN = 10; // INPUT_PULLUP from Mega 42
const uint8_t ARM_DONE_PIN = 9; // OUTPUT to Mega 41

// ======================================================
// MOTOR PINS (DFRobot quad motor shield)
// ======================================================

/*
   Per DFRobot's DRI0039 documentation the shield terminals map to pins:
     terminal M1: PWM 3,  DIR 4
     terminal M2: PWM 11, DIR 12
     terminal M3: PWM 5,  DIR 8
     terminal M4: PWM 6,  DIR 7

   Arm 1 is the motor on the terminal printed "M1", and arm 2 is the
   motor on the terminal printed "M2". If your arms are actually on the
   M4 and M3 terminals instead, use PWM 6 / DIR 7 for arm 1 and
   PWM 5 / DIR 8 for arm 2.
*/

// Arm 1 = terminal M1
const uint8_t M1_PWM = 3;
const uint8_t M1_DIR = 4;

// Arm 2 = terminal M2
const uint8_t M2_PWM = 11;
const uint8_t M2_DIR = 12;

// Set to false if an arm moves the wrong way when told to extend.
const bool ARM1_EXTEND_HIGH = false;
const bool ARM2_EXTEND_HIGH = false;

// ======================================================
// ARM TUNING
// ======================================================

const uint8_t ARM_SPEED = 120; // 0-255

const unsigned long EXTEND_MS = 1000;

// No limit switches, so retract slightly longer than extend so the
// arm always finishes fully home. Tune on the robot.
const unsigned long RETRACT_MS = 1000;

// Short pause at full extension so the motor is not slammed straight
// from forward to reverse.
const unsigned long PAUSE_MS = 100;

// A GO line must stay LOW this long before it is accepted (noise rejection).
const unsigned long GO_DEBOUNCE_MS = 5;

// ======================================================
// STATE
// ======================================================

enum UnoState
{
    WAIT_GO_RELEASED, // boot: make sure both GO lines are HIGH before accepting a request
    IDLE,             // waiting for a GO line to go LOW
    WAIT_GO_HIGH      // arm finished, DONE asserted, waiting for Mega to release GO
};

UnoState state = WAIT_GO_RELEASED;

unsigned long lastHeartbeat = 0;
int lastGo1Level = -1; // -1 = not read yet
int lastGo2Level = -1;
uint16_t runCount = 0;

// ======================================================
// LOG HELPERS
// ======================================================

void logStart()
{
    if (!DEBUG_LOG)
        return;
    Serial.print('[');
    Serial.print(millis());
    Serial.print(F(" ms] "));
}

void logLine(const __FlashStringHelper *msg)
{
    if (!DEBUG_LOG)
        return;
    logStart();
    Serial.println(msg);
}

const __FlashStringHelper *stateName(UnoState s)
{
    switch (s)
    {
    case WAIT_GO_RELEASED:
        return F("WAIT_GO_RELEASED");
    case IDLE:
        return F("IDLE");
    case WAIT_GO_HIGH:
        return F("WAIT_GO_HIGH");
    }
    return F("?");
}

void setState(UnoState next)
{
    if (DEBUG_LOG)
    {
        logStart();
        Serial.print(F("STATE: "));
        Serial.print(stateName(state));
        Serial.print(F(" -> "));
        Serial.println(stateName(next));
    }
    state = next;
}

// ======================================================
// SETUP
// ======================================================

void setup()
{
    if (DEBUG_LOG)
    {
        Serial.begin(SERIAL_BAUD);
    }

    pinMode(M1_PWM, OUTPUT);
    pinMode(M1_DIR, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
    pinMode(M2_DIR, OUTPUT);
    stopArms();

    // Write HIGH first so the pin never glitches LOW when it becomes
    // an output. HIGH means "not done".
    digitalWrite(ARM_DONE_PIN, HIGH);
    pinMode(ARM_DONE_PIN, OUTPUT);

    pinMode(ARM1_GO_PIN, INPUT_PULLUP);
    pinMode(ARM2_GO_PIN, INPUT_PULLUP);

    if (DEBUG_LOG)
    {
        Serial.println();
        Serial.println(F("=== Uno arm slave booted ==="));
        Serial.println(F("Handshake: GO_ARM1=D2, GO_ARM2=D10 (in, active LOW), DONE=D9 (out, active LOW)"));
        Serial.print(F("Arms: arm 1 = PWM"));
        Serial.print(M1_PWM);
        Serial.print(F("/DIR"));
        Serial.print(M1_DIR);
        Serial.print(F(", arm 2 = PWM"));
        Serial.print(M2_PWM);
        Serial.print(F("/DIR"));
        Serial.println(M2_DIR);
        Serial.print(F("Speed="));
        Serial.print(ARM_SPEED);
        Serial.print(F(" extend="));
        Serial.print(EXTEND_MS);
        Serial.print(F("ms pause="));
        Serial.print(PAUSE_MS);
        Serial.print(F("ms retract="));
        Serial.print(RETRACT_MS);
        Serial.println(F("ms"));
        Serial.println(F("Boot: DONE released (HIGH). Waiting for both GO lines to be HIGH."));
    }

    state = WAIT_GO_RELEASED;
}

// ======================================================
// MAIN LOOP
// ======================================================

void loop()
{
    logGoEdges();
    logHeartbeat();

    switch (state)
    {
    case WAIT_GO_RELEASED:
        if (digitalRead(ARM1_GO_PIN) == HIGH && digitalRead(ARM2_GO_PIN) == HIGH)
        {
            logLine(F("Both GO lines HIGH (idle). Ready for requests."));
            setState(IDLE);
        }
        break;

    case IDLE:
    {
        bool runArm1 = false;
        bool runArm2 = false;

        if (goRequested(runArm1, runArm2))
        {
            runCount++;
            if (DEBUG_LOG)
            {
                logStart();
                Serial.print(F("COMMAND RECEIVED (run #"));
                Serial.print(runCount);
                Serial.print(F("): "));
                if (runArm1 && runArm2)
                {
                    Serial.println(F("BOTH GO lines LOW -> running arm 1 and arm 2"));
                }
                else if (runArm1)
                {
                    Serial.println(F("GO_ARM1 (D2) LOW -> running arm 1 (M1)"));
                }
                else
                {
                    Serial.println(F("GO_ARM2 (D10) LOW -> running arm 2 (M2)"));
                }
            }

            runArms(runArm1, runArm2);

            // Tell the Mega we are finished.
            digitalWrite(ARM_DONE_PIN, LOW);
            logLine(F("DONE asserted (LOW). Waiting for Mega to release GO."));
            setState(WAIT_GO_HIGH);
        }
        break;
    }

    case WAIT_GO_HIGH:
        if (digitalRead(ARM1_GO_PIN) == HIGH && digitalRead(ARM2_GO_PIN) == HIGH)
        {
            // Mega acknowledged. Release DONE.
            digitalWrite(ARM_DONE_PIN, HIGH);
            logLine(F("GO released by Mega. DONE released (HIGH)."));
            setState(IDLE);
        }
        break;
    }
}

// ======================================================
// LOGGING IN THE LOOP
// ======================================================

// Print whenever a GO line changes level, so you can see exactly
// what the Mega is sending (and spot noise or a bad connection).
void logGoEdges()
{
    if (!DEBUG_LOG)
        return;

    int go1 = digitalRead(ARM1_GO_PIN);
    if (go1 != lastGo1Level)
    {
        logStart();
        Serial.print(F("GO_ARM1 line (D2): "));
        Serial.println(go1 == LOW ? F("LOW (asserted)") : F("HIGH (idle)"));
        lastGo1Level = go1;
    }

    int go2 = digitalRead(ARM2_GO_PIN);
    if (go2 != lastGo2Level)
    {
        logStart();
        Serial.print(F("GO_ARM2 line (D10): "));
        Serial.println(go2 == LOW ? F("LOW (asserted)") : F("HIGH (idle)"));
        lastGo2Level = go2;
    }
}

void logHeartbeat()
{
    if (!DEBUG_LOG)
        return;

    if (millis() - lastHeartbeat < HEARTBEAT_MS)
        return;
    lastHeartbeat = millis();

    logStart();
    Serial.print(F("alive | state="));
    Serial.print(stateName(state));
    Serial.print(F(" GO1="));
    Serial.print(digitalRead(ARM1_GO_PIN) == LOW ? F("LOW") : F("HIGH"));
    Serial.print(F(" GO2="));
    Serial.print(digitalRead(ARM2_GO_PIN) == LOW ? F("LOW") : F("HIGH"));
    Serial.print(F(" DONE="));
    Serial.print(digitalRead(ARM_DONE_PIN) == LOW ? F("LOW") : F("HIGH"));
    Serial.print(F(" runs="));
    Serial.println(runCount);
}

// ======================================================
// HANDSHAKE
// ======================================================

// Returns true if at least one GO line is LOW (debounced), and says
// which arm(s) were requested.
bool goRequested(bool &arm1, bool &arm2)
{
    if (digitalRead(ARM1_GO_PIN) == HIGH && digitalRead(ARM2_GO_PIN) == HIGH)
    {
        return false;
    }

    delay(GO_DEBOUNCE_MS);

    arm1 = (digitalRead(ARM1_GO_PIN) == LOW);
    arm2 = (digitalRead(ARM2_GO_PIN) == LOW);

    if (arm1 || arm2)
    {
        return true;
    }

    logLine(F("GO glitch rejected (went back HIGH during debounce)."));
    return false;
}

// ======================================================
// ARM ROUTINE
// ======================================================

// Once a request is accepted, the whole extend / pause / retract routine
// runs to completion. It does not look at the GO lines (or anything else)
// until it has finished, so losing the plant mid-run cannot cut it short.
void runArms(bool arm1, bool arm2)
{
    // Extend the requested arm(s).
    if (DEBUG_LOG)
    {
        logStart();
        Serial.print(F("ARMS: extending "));
        if (arm1)
            Serial.print(F("arm 1 (M1) "));
        if (arm2)
            Serial.print(F("arm 2 (M2) "));
        Serial.print(F("for "));
        Serial.print(EXTEND_MS);
        Serial.println(F(" ms"));
    }

    // if (arm1)
        // setArm(M1_PWM, M1_DIR, ARM1_EXTEND_HIGH, true);
    if (arm2)
        setArm(M2_PWM, M2_DIR, ARM2_EXTEND_HIGH, true);
    delay(EXTEND_MS);

    stopArms();
    if (DEBUG_LOG)
    {
        logStart();
        Serial.print(F("ARMS: stopped, pausing "));
        Serial.print(PAUSE_MS);
        Serial.println(F(" ms"));
    }
    delay(PAUSE_MS);

    // Retract the same arm(s).
    if (DEBUG_LOG)
    {
        logStart();
        Serial.print(F("ARMS: retracting "));
        // if (arm1)
            // Serial.print(F("arm 1 (M1) "));
        if (arm2)
            Serial.print(F("arm 2 (M2) "));
        Serial.print(F("for "));
        Serial.print(RETRACT_MS);
        Serial.println(F(" ms"));
    }

    // if (arm1)
        // setArm(M1_PWM, M1_DIR, ARM1_EXTEND_HIGH, false);
    if (arm2)
        setArm(M2_PWM, M2_DIR, ARM2_EXTEND_HIGH, false);
    delay(RETRACT_MS);

    stopArms();
    logLine(F("ARMS: stopped. Routine complete."));
}

// ======================================================
// MOTOR CONTROL
// ======================================================

void setArm(uint8_t pwmPin, uint8_t dirPin, bool extendIsHigh, bool extend)
{
    bool level = extend ? extendIsHigh : !extendIsHigh;
    digitalWrite(dirPin, level ? HIGH : LOW);
    analogWrite(pwmPin, ARM_SPEED);
}

void stopArms()
{
    analogWrite(M1_PWM, 0);
    analogWrite(M2_PWM, 0);
}
