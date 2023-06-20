/*!
 * @file mprls_simpletest.ino
 *
 * A basic test of the sensor with default settings
 * 
 * Designed specifically to work with the MPRLS sensor from Adafruit
 * ----> https://www.adafruit.com/products/3965
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.  
 *
 * MIT license, all text here must be included in any redistribution.
 * 
 * Code is based on the above, with added functionality for an override button,
 * and an LED indicator
 * 
 */
 
#include <Wire.h>
#include "Adafruit_MPRLS.h"
#include <LiquidCrystal.h>

// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

// Pin Settings
const int ledPin =  22;
const int buttonPin = 13;
const int relayPin = 26;
const int contrastPin = 6;

// SET THE PRESSURE HERE (In hPa or PSI, comment the one you didn't use out)

// High point 14.05

// Low point 13.31
// hPa
//const float triggerPressure = 900;
// PSI
const float triggerPressure = 14;
const float lowPoint = 13.32;

int buttonState = 0;
String state;

// Settings for the LCD:
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
int Contrast=75;


void setup() {
  Serial.begin(115200);
  analogWrite(contrastPin,Contrast);
  Serial.println("MPRLS Simple Test");
  lcd.begin(16, 2);
  if (! mpr.begin()) {
    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
    while (1) {
      delay(10);
    }
  }
  Serial.println("Found MPRLS sensor");
  lcd.print("Cur PSI:");
  lcd.setCursor(0, 1);
  lcd.print("Cur hPa:");
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(buttonPin, INPUT);
}


void loop() {
   /* we're going to need to read the pressure, and then decide if we need to trigger the relay for the vacuum
   *  once we trigger the vacuum, we want to check the pressure every second until it hits the expected pressure,
   *  once it goes above the preferred pressure, it will close the relay again.
   */
  
  buttonState = digitalRead(buttonPin);
  float pressure_hPa = mpr.readPressure();
  Serial.print("Pressure (hPa): "); Serial.println(pressure_hPa);
  
  float pressurePsi = pressure_hPa / 68.947572932;
  Serial.print("Pressure (PSI): "); Serial.println(String(pressurePsi));
  Serial.println(state);
  lcd.setCursor(9, 0);
  lcd.print(String(pressurePsi));
  lcd.setCursor(9, 1);
  lcd.print(String(pressure_hPa));
  if (buttonState == HIGH) {
    if (pressurePsi >= triggerPressure) {
      // Turn on the LED to indicate we're going to flip on the vacuum
      digitalWrite(ledPin, HIGH);
      digitalWrite(relayPin, HIGH);
      state = "on";
    } else if (pressurePsi <= lowPoint) {
      digitalWrite(ledPin, LOW);
      digitalWrite(relayPin, LOW);
      state = "off";
    } /*else {
      digitalWrite(ledPin, LOW);
      digitalWrite(relayPin, LOW);
      state = "off";
    }*/
  } else if (buttonState == HIGH) {
    digitalWrite(ledPin, LOW);
    digitalWrite(relayPin, LOW);
  } else {
    digitalWrite(ledPin, HIGH);
    digitalWrite(relayPin, HIGH);
  }
  delay(1000);

}
