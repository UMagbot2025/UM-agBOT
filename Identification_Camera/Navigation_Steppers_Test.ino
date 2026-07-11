// ---------------- MOTOR PINS ----------------

const int stepPins[4] = {12, 6, 8, 10};
const int dirPins[4] = {13, 7, 9, 11};

// Change these if a wheel spins opposite
bool invertDir[4] = {true, false, true, false};

// ---------------- SETTINGS ----------------

const int STEP_DELAY = 300; // microseconds

// ---------------- SETUP ----------------

void setup()
{
    Serial.begin(115200);

    for (int i = 0; i < 4; i++)
    {
        pinMode(stepPins[i], OUTPUT);
        pinMode(dirPins[i], OUTPUT);

        digitalWrite(stepPins[i], LOW);
        digitalWrite(dirPins[i], LOW);
    }

    Serial.println("Mecanum stepper test starting");
    delay(1000);
}

// ---------------- LOOP TEST ----------------

void loop()
{
    Serial.println("FORWARD");
    moveForward(500);
    delay(1000);

    Serial.println("BACKWARD");
    moveBackward(500);
    delay(1000);

    Serial.println("STRAFE LEFT");
    strafeLeft(1500);
    delay(1000);

    Serial.println("STRAFE RIGHT");
    strafeRight(1500);
    delay(3000);

    Serial.println("TURN 180");
    turn180(5750);
    delay(3000);
}

// ---------------- DIRECTION CONTROL ----------------

void setDirections(int dir[4])
{
    for (int i = 0; i < 4; i++)
    {
        if (dir[i] == 0)
            continue;

        bool forward = dir[i] > 0;

        if (invertDir[i])
            forward = !forward;

        digitalWrite(dirPins[i], forward ? HIGH : LOW);
    }
}

// ---------------- STEP PULSE ----------------

void stepMotors(int dir[4])
{
    for (int i = 0; i < 4; i++)
    {
        if (dir[i] != 0)
            digitalWrite(stepPins[i], HIGH);
    }

    delayMicroseconds(STEP_DELAY);

    for (int i = 0; i < 4; i++)
    {
        if (dir[i] != 0)
            digitalWrite(stepPins[i], LOW);
    }

    delayMicroseconds(STEP_DELAY);
}

// ---------------- RAW DRIVE ----------------

void driveRaw(int dir[4])
{
    setDirections(dir);
    stepMotors(dir);
}

// ---------------- MOVEMENT FUNCTIONS ----------------

void moveForward(long steps)
{
    int dir[4] = {1, 1, 1, 1};

    for (long i = 0; i < steps; i++)
    {
        driveRaw(dir);
    }

    stopMotors();
}

void moveBackward(long steps)
{
    int dir[4] = {-1, -1, -1, -1};

    for (long i = 0; i < steps; i++)
    {
        driveRaw(dir);
    }

    stopMotors();
}

void strafeLeft(long steps)
{
    int dir[4] = {-1, 1, 1, -1};

    for (long i = 0; i < steps; i++)
    {
        driveRaw(dir);
    }

    stopMotors();
}

void strafeRight(long steps)
{
    int dir[4] = {1, -1, -1, 1};

    for (long i = 0; i < steps; i++)
    {
        driveRaw(dir);
    }

    stopMotors();
}

void turn180(long steps)
{
    // Left side forward, right side backward
    // Spins robot in place

    int dir[4] = {1, -1, 1, -1};

    for (long i = 0; i < steps; i++)
    {
        driveRaw(dir);
    }

    stopMotors();
}

// ---------------- STOP ----------------

void stopMotors()
{
    for (int i = 0; i < 4; i++)
    {
        digitalWrite(stepPins[i], LOW);
    }
}