#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPRLS.h>
#include <LiquidCrystal.h>


// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);


// Definition des Pin;
const int LedAction =  22;
const int RelayMoteur = 24;
const int ButtonValidation = 26;
const int ButtonForcer = 28;
const int PinPotentio = A15;
const int PinPotentio2 = A14;

//configuration des Seuils Mini / Maxi;

float highPoint = -0.175;  // In ABS - Default is 0.825 (-0.175 bar)
float lowPoint = -0.250;         // In ABS - Default is 0.750 (-0.250 bar)
float Pression = mpr.readPressure()/1000;

//Initialisation de l'etats des Bouttons;

int ButtonValidation = 0;
float Valeurpotentio;
float Valeurpotentio2;

// Parametre LCD ;
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
const int contrastPin = 6;
int Contrast=75;    // Contraste de 0 à 100


// Settings for initialisation
int StateConfig = 0;
int Stop;


void Forcer()
  {
    Serial.print("Etat du bouton de marche forcer: ");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" Marche Forcer ");

      do
        {
          digitalWrite(LedAction, HIGH);
          digitalWrite(RelayMoteur, HIGH);
          lcd.setCursor(0, 1);
          lcd.print(Pression);lcd.println(" Bar");
        }
      while (digitalRead(ButtonForcer==LOW));
  }


void setup()
  {
    Serial.begin(115200);
    analogWrite(contrastPin,Contrast);
    Serial.println("MPRLS Simple Test");
    lcd.begin(16, 2);
    lcd.print("Hello");
    if (! mpr.begin(0x18)) 

      {

        Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
        lcd.print("Error Com MPRLS");
        while (1) 

      {
      
      delay(10);
    }
  }
  Serial.println("Found MPRLS sensor");
  lcd.print("Setup OK");
  delay(1000);


  // INITIALISATION
  lcd.clear();
  lcd.print("INITIALISATION");
  lcd.setCursor(0, 1);
  lcd.print("****************");
  //Serial.print("buttonState : "); Serial.println(String(buttonState));
  //Serial.print("potentiometerValue : "); Serial.println(analogRead(A15));
  //Serial.println();
  delay(2000);


  // initialize the LED pin as an output:
  pinMode(LedAction, OUTPUT);
  pinMode(RelayMoteur, OUTPUT);
  pinMode(ButtonValidation, INPUT_PULLUP);
  pinMode(ButtonForcer, INPUT_PULLUP); // Bouton est passant en PULLUP
  attachInterrupt(digitalPinToInterrupt(ButtonForcer), Forcer, FALLING);

  }


void loop() {
  if (StateConfig == 0){Mini();}
  Auto();
  }




void Mini()
  {
    do
      {
        lcd.clear();
        lcd.print("SET low point :");
        lcd.setCursor(0, 1);
        Valeurpotentio = (analogRead(PinPotentio) * (-0.33/1024));
        lcd.print(Valeurpotentio); lcd.print(" bar");
      }
    while(ButtonValidation == LOW);  
    StateConfig=1;
    lowPoint = 1 + Valeurpotentio;
    Maxi();

  }


void Maxi()
  {
    do
      {
        lcd.clear();
        lcd.print("SET High point :");
        lcd.setCursor(0, 1);
        Valeurpotentio2 = (analogRead(PinPotentio2) * (-0.33/1024));
        lcd.print(Valeurpotentio2); lcd.print(" bar");
      }
    while((ButtonValidation == LOW));  
    StateConfig=2;
    highPoint = 1 + Valeurpotentio2;

  }


void Auto() 
  {
    lcd.clear();
    lcd.print("LET'S STARTED !!");
    delay(2500);
    lcd.clear();


      while (1) 
        { // Infinite loop
          lcd.print("Cur Bar:");
          lcd.setCursor(0, 1);
          lcd.setCursor(9, 0);
          lcd.print(String(Pression));

                  //Serial.print("Pressure (hPa): "); Serial.println(String(pressure_hPa));
                  //float pressureBar = pressure_hPa / 1000;
                  //Serial.print("Pressure (Bar): "); Serial.println(String(pressureBar));
                  //Serial.print("Light is : "); Serial.println(state);
                  //Serial.print("lowPoint : "); Serial.print(String(lowPoint)); Serial.println(" bar (Absolut) REGISTRED");
                  //Serial.print("triggerPressure : "); Serial.print(String(triggerPressure)); Serial.println(" bar (Absolut) REGISTRED");
                  //Serial.println();
                  
          if (digitalRead(ButtonForcer==LOW)) {Forcer();}

          if (Pression <= highPoint)   // Turn on the LED to indicate we're going to flip on the vacuum
            {
              digitalWrite(LedAction, HIGH);
              digitalWrite(RelayMoteur, HIGH);
            }

          else if (Pression >= lowPoint) // Turn off the LED and relay
            {
              digitalWrite(LedAction, LOW);
              digitalWrite(RelayMoteur, LOW);
            }
        }
  }