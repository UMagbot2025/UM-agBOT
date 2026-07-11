/*
  agBOT - RIGHT UNO: camera + arm controller
  -------------------------------------------
  Mirror of the left Uno sketch - see that file's header comment for the
  full explanation of the latch-and-wait-for-go logic. This one drives M2.

  Handshake pins (must share common GND with Mega + left Uno):
    DETECT_OUT_PIN (output) -> Mega RIGHT_DETECT_PIN
    GO_IN_PIN       (input) <- Mega ALIGN_GO_PIN   (same shared line as left Uno)
    DONE_OUT_PIN   (output) -> Mega RIGHT_DONE_PIN
*/

#include <Pixy2.h>

const int DETECT_OUT_PIN = 2;
const int GO_IN_PIN = 3;
const int DONE_OUT_PIN = 4;

const int ARM_PIN = 7; // M2 - TODO: confirm your actual driver pin
const unsigned int ARM_RUN_TIME_MS = 3000;

const unsigned long DETECT_HOLD_MS = 150;

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
    Serial.println("Right Uno ready.");
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
            Serial.println("Double plant confirmed (right). Latching detect flag.");
            digitalWrite(DETECT_OUT_PIN, HIGH);
            state = LATCHED_WAITING_GO;
        }
        break;
    }

    case LATCHED_WAITING_GO:
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
