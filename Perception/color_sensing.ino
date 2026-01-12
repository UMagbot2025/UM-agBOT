#include "Wire.h"
#include "Adafruit_TCS34725.h"


//---------------- DEPENDENCY -------------------//
// Adafruit_TCS34725 library


//---------- SENSOR CONNECTION GUIDE ------------//
// https://www.makerguides.com/tcs34725-rgb-color-sensor-with-arduino/


//---------------- CONFIGURATION ----------------//

#define MOE 30  // Margin of error for color range

#define RED_PIN   3
#define GREEN_PIN 5
#define BLUE_PIN  6

#define COMMON_ANODE true  // Set false for common cathode RGB LEDs


// Struct to store target RGB values representing a color range
struct ColorRange {
  int R;
  int G;
  int B;
};


// Color calibration targets
ColorRange ripeColor    = {120, 95, 45};
ColorRange unripeColor  = {84, 110, 63};


// Lookup table for gamma-corrected brightness
byte gammaTable[256];


// TCS34725 color sensor instance
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);


//---------------- HELPER FUNCTIONS ----------------//

/**
 * @brief Generates a 256-entry gamma correction table for LED output.
 *
 * Adjusts intensity for human perception of brightness.
 * The output is inverted if using a common-anode configuration.
 * Loopup table generator for testing: https://victornpb.github.io/gamma-table-generator/
 */
void generateGammaTable() {
  for (int i = 0; i < 256; i++) {
    float x = pow(i / 255.0, 2.5) * 255.0;
    gammaTable[i] = COMMON_ANODE ? 255 - x : x;
  }
}


/**
 * @brief Applies RGB color values to the LED with gamma correction.
 *
 * Uses PWM to adjust LED brightness for each color channel.
 *
 * @param R Red component (0–255).
 * @param G Green component (0–255).
 * @param B Blue component (0–255).
 */
void applyColorToLED(float R, float G, float B) {
  analogWrite(RED_PIN,   gammaTable[(int)R]);
  analogWrite(GREEN_PIN, gammaTable[(int)G]);
  analogWrite(BLUE_PIN,  gammaTable[(int)B]);
}

void printRGB(float R, float G, float B) {
  Serial.print("R: "); Serial.print(int(R));
  Serial.print("\tG: "); Serial.print(int(G));
  Serial.print("\tB: "); Serial.println(int(B));
}


/**
 * @brief Checks if given RGB values are within a color range tolerance.
 *
 * Compares R, G, and B against target values in range, using mask bits to
 * select which channels to test:
 *   0b100: Red, 0b010: Green, 0b001: Blue.
 * Each checked channel must differ by less than MOE for the function to return true.
 *
 * @param R     Input red component.
 * @param G     Input green component.
 * @param B     Input blue component.
 * @param range Target color values.
 * @param mask  Bitmask specifying which channels to compare.
 * @return true if all selected channels are within the tolerance.
 */
bool isWithinRange(float R, float G, float B, const ColorRange &range, byte mask) {
  bool checkR = mask & 0b100;
  bool checkG = mask & 0b010;
  bool checkB = mask & 0b001;

  bool result = true;

  if (checkR) result &= (fabs(R - range.R) < MOE);
  if (checkG) result &= (fabs(G - range.G) < MOE);
  if (checkB) result &= (fabs(B - range.B) < MOE);

  return result;
}


//---------------- SETUP & LOOP ----------------//

/**
 * @brief Initializes serial communication, sensor, and LED outputs.
 *
 * Sets up gamma correction and verifies sensor connection.
 * The program halts if no TCS34725 sensor is found.
 */
void setup() {
  Serial.begin(9600);

  if (!tcs.begin()) {
    Serial.println("No TCS34725 found ... check connections");
    while (1);
  }

  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  generateGammaTable();
}


/**
 * @brief Continuously reads sensor values and categorizes detected color.
 *
 * Determines whether the measured color matches "ripe" or "unripe" targets
 * and updates both serial output and LED display accordingly.
 */
void loop() {
  float R, G, B;

  tcs.setInterrupt(false);
  delay(2000);  // Allow sensor capture
  tcs.getRGB(&R, &G, &B);
  tcs.setInterrupt(true);

  if (isWithinRange(R, G, B, unripeColor, 0b111)) {
    Serial.println("Unripe coorn");
  } 
  else if (isWithinRange(R, G, B, ripeColor, 0b111)) {
    Serial.println("Ripe cccoorn");
  } 
  else {
    Serial.println("NOOOO Corn");
    printRGB(R, G, B);
  }

  applyColorToLED(R, G, B);
}
