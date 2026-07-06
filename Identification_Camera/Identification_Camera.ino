#include <Pixy2.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

Pixy2 pixy;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// LED Pins
const int GREEN_LED = 10;
const int RED_LED = 11;
const int BLUE_LED = 12;

// Stand Counters
int emptyCount = 0;
int singleCount = 0;
int doubleCount = 0;

void updateDisplay()
{
    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("E:");
    lcd.print(emptyCount);

    lcd.print(" S:");
    lcd.print(singleCount);

    lcd.print(" D:");
    lcd.print(doubleCount);

    lcd.setCursor(0, 1);
    lcd.print("Total:");
    lcd.print(emptyCount + singleCount + doubleCount);
}

void turnOffLEDs()
{
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
}

void setup()
{
    Serial.begin(115200);

    pixy.init();

    pinMode(GREEN_LED, OUTPUT);
    pinMode(RED_LED, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);

    lcd.init();
    lcd.backlight();

    lcd.clear();
    lcd.print("Stand Counter");
    delay(1500);

    updateDisplay();

    Serial.println("--------------------------------");
    Serial.println("Press ENTER to scan");
}

void loop()
{
    if (Serial.available() > 0)
    {
        char input = Serial.read();

        // Only respond to Enter key
        if (input == '\n')
        {

            bool sig1Found = false;
            bool sig2Found = false;

            // Read Pixy blocks
            pixy.ccc.getBlocks();

            // Check all detected objects
            for (int i = 0; i < pixy.ccc.numBlocks; i++)
            {
                uint8_t sig = pixy.ccc.blocks[i].m_signature;

                if (sig == 1)
                    sig1Found = true;

                if (sig == 2)
                    sig2Found = true;
            }

            turnOffLEDs();

            // Both signatures found
            if (sig1Found && sig2Found)
            {
                digitalWrite(GREEN_LED, HIGH);
                doubleCount++;

                Serial.println("Double plant detected");
            }
            // Signature 1 only
            else if (sig1Found && !sig2Found)
            {
                digitalWrite(BLUE_LED, HIGH);
                singleCount++;

                Serial.println("Single plant detected");
            }
            // No signatures OR Signature 2 only
            else
            {
                digitalWrite(RED_LED, HIGH);
                emptyCount++;

                if (!sig1Found && !sig2Found)
                    Serial.println("Empty stand");
                else
                    Serial.println("Signature 2 only");
            }

            updateDisplay();

            Serial.print("Empty: ");
            Serial.print(emptyCount);

            Serial.print("  Single: ");
            Serial.print(singleCount);

            Serial.print("  Double: ");
            Serial.print(doubleCount);

            Serial.print("  Total: ");
            Serial.println(emptyCount + singleCount + doubleCount);

            Serial.println("Press ENTER to scan again");
        }
    }