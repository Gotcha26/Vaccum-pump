#include <Arduino.h>


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


 /*
  * 1 Bar = 14.61 PSI
  * -0.25 Bar = (1-0.25) = 0.75 Bar = 10.88 PSI
  * Sources : https://www.gflow.fr/upload/files/psi-bar.pdf and http://dominique.melotti.pagesperso-orange.fr/bar.htm
  * 1 Bar = 1000 Hectopascals
  * 0.75 Bar = 750 hPa
  * Source : https://convertlive.com/fr/u/convert/barres/a/hectopascals#0.75
  * -0.25 Bar = 850 hPa = +0.75 Bar
  */
 
#include <Wire.h>
#include <Adafruit_MPRLS.h>
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
const int potentiometerPin = A15;
const int buttonFPin = 7;


// SET THE PRESSURE HERE (In absolute Bar)
float triggerPressure = 1.01;  // In ABS - Default is 0.825 (-0.175 bar)
float lowPoint = 0.85;         // In ABS - Default is 0.750 (-0.250 bar)


// BUTTOM
// The button take effect between the lowPoint and the triggerPressur for restart the vacuum until the the lowPoint.
int buttonState = 0;
int buttonFState = 0;
String state;
float potentiometerValue;


// Settings for the LCD:
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
int Contrast=75;


// Settings for initialisation
int EOI = 0;
int Stop;



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


  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(buttonFPin, INPUT_PULLUP); // Bouton est passant en PULLUP


  // Forced work (only at the startup or after RESET)
  buttonFState = digitalRead(buttonFPin);
  Serial.print("Etat du bouton de marche forc�e : "); Serial.println(buttonFState);
  if (buttonFState == 0) { // Infinite loop
    Serial.println();
    Serial.println("*****************");
    Serial.println("**** WARNING ****");
    Serial.println("*****************");
    Serial.println();
    while (1) {
      float pressure_hPa = mpr.readPressure();
      digitalWrite(ledPin, HIGH);
      digitalWrite(relayPin, HIGH);
      state = "on";
      Serial.print("Pressure (hPa): "); Serial.println(String(pressure_hPa));
      Serial.println("FORCED WORK ! BE CAREFULL !!!");
      lcd.clear();
      lcd.print(" NO LIMITS !!!! ");
      lcd.setCursor(0, 1);
      lcd.print("RESET TO STOP IT");
      delay (1000);
    }
  }



  // INITIALISATION
  lcd.clear();
  lcd.print(" INITIALISATION");
  lcd.setCursor(0, 1);
  lcd.print("****************");
  Serial.print("buttonState : "); Serial.println(String(buttonState));
  Serial.print("potentiometerValue : "); Serial.println(analogRead(A15));
  Serial.println();
  delay(2000);


}



void loop() {



// Adjust the trigger and low points
while (EOI != 2) { // Condition � v�rifier pour rester dans la boucle jusqu'� ce que...
  if (EOI == 0) {
    // Setting the Low point
    buttonState = digitalRead(buttonPin);
    lcd.clear();
    lcd.print("SET low point :");
    lcd.setCursor(0, 1);
    potentiometerValue = (analogRead(potentiometerPin) * (-0.33/1024));
    lcd.print(potentiometerValue); lcd.print(" bar");


    // Condition de sortie du WHILE
    if (buttonState == LOW) {
      EOI = 1;
      lowPoint = 1 + potentiometerValue;
    }
    Serial.print("buttonState : "); Serial.println(String(buttonState)); 
    Serial.print("EOI : "); Serial.println(String(EOI));
    Serial.print("lowPoint : "); Serial.print(String(lowPoint)); Serial.println(" bar (Absolut) by DEFAULT");
    Serial.println();
    delay(400); 
  } else {
    if (EOI == 1) {
      // Settings the Trigger point
      buttonState = digitalRead(buttonPin);
      lcd.clear();
      lcd.print("SET trigger pt :");
      lcd.setCursor(0, 1);
      potentiometerValue = (analogRead(potentiometerPin) * (-0.23/1024));
      lcd.print(potentiometerValue); lcd.print(" bar");
      
      // Condition de sortie du WHILE
      if (buttonState == LOW) {
        EOI = 2;
        triggerPressure = 1 + potentiometerValue;
      }
      Serial.print("buttonState : "); Serial.println(String(buttonState)); 
      Serial.print("EOI : "); Serial.println(String(EOI));
      Serial.print("lowPoint : "); Serial.print(String(lowPoint)); Serial.println(" bar (Absolut) REGISTRED");
      Serial.print("triggerPressure : "); Serial.print(String(triggerPressure)); Serial.println(" bar (Absolut) by DEFAULT");
      Serial.println();
      delay(400);
    }
  }
}



/*
*  We're going to need to read the pressure, and then decide if we need to trigger the relay for the vacuum
*  once we trigger the vacuum, we want to check the pressure every second until it hits the expected pressure,
*  once it goes above the preferred pressure, it will close the relay again.
*
*  Beetwen the lowPoint and the triggerPressure, the vaccum system can be restarted if the "OK - RESTART" button is pressed. 
*/


lcd.clear();
lcd.print("LET'S STARTED !!");
delay(2500);
lcd.clear();


while (1) { // Infinite loop
  lcd.print("Cur Bar:");
  lcd.setCursor(0, 1);
  lcd.print("Cur hPa:"); 


  buttonState = digitalRead(buttonPin);
  float pressure_hPa = mpr.readPressure();
  Serial.print("Pressure (hPa): "); Serial.println(String(pressure_hPa));
  float pressureBar = pressure_hPa / 1000;
  Serial.print("Pressure (Bar): "); Serial.println(String(pressureBar));
  Serial.print("Light is : "); Serial.println(state);
  Serial.print("lowPoint : "); Serial.print(String(lowPoint)); Serial.println(" bar (Absolut) REGISTRED");
  Serial.print("triggerPressure : "); Serial.print(String(triggerPressure)); Serial.println(" bar (Absolut) REGISTRED");
  Serial.println();
  lcd.setCursor(9, 0);
  lcd.print(String(pressureBar));
  lcd.setCursor(9, 1);
  lcd.print(String(pressure_hPa));
  if (buttonState == HIGH) {    // Normally must be true everytime
    if (pressureBar >= triggerPressure) {
      // Turn on the LED to indicate we're going to flip on the vacuum
      digitalWrite(ledPin, HIGH);
      digitalWrite(relayPin, HIGH);
      state = "on";
    } else if (pressureBar <= lowPoint) {
      // Turn off the LED and relay
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
    state = "off";
  } else {
    digitalWrite(ledPin, HIGH);
    digitalWrite(relayPin, HIGH);
    state = "on";
  }
  delay(1000);
  }


}