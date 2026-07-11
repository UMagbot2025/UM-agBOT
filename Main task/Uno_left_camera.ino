/*
  agBOT - LEFT UNO: camera + arm controller
  ------------------------------------------
  Watches its own Pixy2 (SPI/ICSP - the ONLY SPI device on this board, so
  none of the multi-camera bus-sharing concerns from the single-Mega version
  apply here) for signature 1 AND signature 2 at once ("double plant").

  Latches a DETECT flag high for the Mega once that's been stable for a
  short hold time (avoids one flickery camera frame causing a false trigger).
  The flag LATCHES (stays high) rather than tracking live camera state,
  because by the time the Mega finishes its alignment move the plant may
  have scrolled out of frame - we don't want the flag to drop just because
  the camera can no longer see it.

  Waits for the Mega's GO signal (meaning "I've finished aligning, fire
  now") before running its local arm motor - it does NOT fire the instant
  it sees the plant, since the robot hasn't repositioned yet at that point.
  Reports DONE once the arm cycle finishes, and waits for GO to drop before
  resetting its own latches, so it doesn't reset mid-handshake.

  Handshake pins (must share common GND with Mega + right Uno):
    DETECT_OUT_PIN (output) -> Mega LEFT_DETECT_PIN
    GO_IN_PIN       (input) <- Mega ALIGN_GO_PIN
    DONE_OUT_PIN   (output) -> Mega LEFT_DONE_PIN
*/

#include <Pixy2.h>

const int DETECT_OUT_PIN = 2;
const int GO_IN_PIN = 3;
const int DONE_OUT_PIN = 4;

const int ARM_PIN = 7; // M1 - TODO: confirm your actual driver pin
const unsigned int ARM_RUN_TIME_MS = 3000;

const unsigned long DETECT_HOLD_MS = 150; // double-plant read must be continuously
                                          // true for this long before we latch it

Pixy2 pixy;

enum LocalState
{
    WATCHING,
    LATCHED_WAITING_GO,
    RUNNING_ARM,
    WAITING_RESET
};
LocalState state = WATCHING;

unsigned long detectStableSince = 0;
bool wasSeeingDouble = false;

void setup()
{
    Serial.begin(115200);
    pinMode(DETECT_OUT_PIN, OUTPUT);
    pinMode(GO_IN_PIN, INPUT);
    pinMode(DONE_OUT_PIN, OUTPUT);
    pinMode(ARM_PIN, OUTPUT);

    digitalWrite(DETECT_OUT_PIN, LOW);
    digitalWrite(DONE_OUT_PIN, LOW);
    digitalWrite(ARM_PIN, LOW);

    pixy.init();
    Serial.println("Left Uno ready.");
}

void loop()
{
    switch (state)
    {

    case WATCHING:
    {
        bool seeingDouble = seesDoublePlant();

        if (seeingDouble && !wasSeeingDouble)
        {
            detectStableSince = millis();
        }
        wasSeeingDouble = seeingDouble;

        if (seeingDouble && (millis() - detectStableSince >= DETECT_HOLD_MS))
        {
            Serial.println("Double plant confirmed (left). Latching detect flag.");
            digitalWrite(DETECT_OUT_PIN, HIGH);
            state = LATCHED_WAITING_GO;
        }
        break;
    }

    case LATCHED_WAITING_GO:
        // Keep DETECT_OUT_PIN high; just wait for the Mega to finish aligning.
        if (digitalRead(GO_IN_PIN) == HIGH)
        {
            Serial.println("GO received. Firing arm.");
            state = RUNNING_ARM;
        }
        break;

    case RUNNING_ARM:
        digitalWrite(ARM_PIN, HIGH);
        delay(ARM_RUN_TIME_MS);
        digitalWrite(ARM_PIN, LOW);
        digitalWrite(DONE_OUT_PIN, HIGH);
        state = WAITING_RESET;
        break;

    case WAITING_RESET:
        // The Mega drops ALIGN_GO_PIN once BOTH sides report done. Wait for
        // that before clearing our own flags, so we don't reset early.
        if (digitalRead(GO_IN_PIN) == LOW)
        {
            digitalWrite(DETECT_OUT_PIN, LOW);
            digitalWrite(DONE_OUT_PIN, LOW);
            wasSeeingDouble = false;
            state = WATCHING;
        }
        break;
    }
}

bool seesDoublePlant()
{
    pixy.ccc.getBlocks();
    bool sawSig1 = false, sawSig2 = false;
    for (int i = 0; i < pixy.ccc.numBlocks; i++)
    {
        if (pixy.ccc.blocks[i].m_signature == 1)
            sawSig1 = true;
        if (pixy.ccc.blocks[i].m_signature == 2)
            sawSig2 = true;
    }
    return sawSig1 && sawSig2;
}
