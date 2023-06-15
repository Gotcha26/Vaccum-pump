#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPRLS.h>
#include <LiquidCrystal.h>


// Vous n'avez PAS besoin de pin RESET ni EOC dans la plus part des cas. Donc ces fonctions sont définies à -1 et ne sont pas utilisées.
#define RESET_PIN  -1  // Pour définir n'importe quelle broche # du GPIO comme un hard-reset au démarrage begin()
#define EOC_PIN    -1  // Pour définir n'importe quelle broche du GPIO comme une attente delecture pour end-of-conversion
// Adresse I2C du capteur MPRLS begin(0x18)
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);


// Attribution des broches
const int ledPin =  22;
const int buttonPin = 13;
const int relayPin = 26;
const int contrastPin = 6;
const int potentiometerPin = A15;
const int buttonFPin = 7;


// Définition des pression (En ABSOLU - Bar)
// Pour triggerPressure, par défaut, en relatif est de -0.175 bar soit 0.825 bar ABS
// Pour lowPoint, par défaut, en relatif est de -0.250 bar soit 0.750 bar ABS
// Pour un potentiomère unique ET commun, les valeurs par défaut seront de :
float lowPoint = 0.85;         // 0.85
float triggerPressure = 1.01;  // 1.01



// Boutons
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
  Serial.println("Test du capteur MPRLS");
  lcd.begin(16, 2);
  if (! mpr.begin(0x18)) {
    Serial.println("Communication impossible. Vérifiez le branchement et/ou l'adresse.");
    while (1) {
      delay(10);
    }
  }
  Serial.println("Capteur MPRLS fonctionnel.");


  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(buttonFPin, INPUT_PULLUP); // Bouton est passant en PULLUP


  // Forced work (only at the startup or after RESET)
  buttonFState = digitalRead(buttonFPin);
  Serial.print("Etat du bouton de marche forcée : "); Serial.println(buttonFState);
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
while (EOI != 2) { // Condition à vérifier pour rester dans la boucle jusqu'à ce que...
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