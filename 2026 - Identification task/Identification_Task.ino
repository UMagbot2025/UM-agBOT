#include <Pixy2.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

Pixy2 pixy;
/*
    The I2C LCD has 4 wires coming out of it in the below order:
    GND -> Connect to GND
    VCC -> Connect to 5V
    SDA -> Connect to the arduino SDA
    SCL -> Connect to the arduino SCL
*/
LiquidCrystal_I2C lcd(0x27, 16, 2);

// LED Pins
const int GREEN_LED = 10; // Connect the led's to their respective pins
const int RED_LED = 11;
const int BLUE_LED = 12;

// Start button (switch between pin 18 and GND)
const int START_BUTTON = 18; // Connect the button to pin 18

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

// Blocks until the button is pressed, with debounce.
// One press = one scan (waits for release too).
void waitForButton()
{
    // Wait for press (pin goes LOW)
    while (digitalRead(START_BUTTON) == HIGH)
    { /* idle */
    }
    delay(30); // debounce
    if (digitalRead(START_BUTTON) == HIGH)
        return; // noise, ignore

    // Wait for release so a held button doesn't retrigger
    while (digitalRead(START_BUTTON) == LOW)
    { /* held */
    }
    delay(30); // debounce release
}

void setup()
{
    Serial.begin(115200);

    pixy.init();

    pinMode(GREEN_LED, OUTPUT);
    pinMode(RED_LED, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);

    pinMode(START_BUTTON, INPUT_PULLUP); // HIGH = open, LOW = pressed

    lcd.init();
    lcd.backlight();

    lcd.clear();
    lcd.print("Stand Counter");
    delay(1500);

    updateDisplay();

    Serial.println("--------------------------------");
    Serial.println("Press button to scan");
}

void loop()
{
    // Wait here until the button is pressed
    waitForButton();

    Serial.println("Scanning...");

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

    Serial.println("Press button to scan again");
}