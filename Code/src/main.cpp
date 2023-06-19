#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPRLS.h>
#include <LiquidCrystal.h>

/*
*********************************************************************************************************************************************************
|                                                   Différence entre pression ABSOLUE et MANOMÉTRIQUE                                                   |
*********************************************************************************************************************************************************

Par convention internationnale, la mesure de la pression doit se faire de manière ABSOLUE et est exprimée en Hecto Pascal (hPa).
Donc ce chiffre sera compris entre 0 (zéro) et +∞ (infini).
Plus d'informations : https://www.thermal-engineering.org/fr/quest-ce-que-la-pression-absolue-definition/

La pression MANOMÉTRIQUE (appelée aussi "pression de jauge") n'est autre d'une mesure __relative__ en fonction de l'environement ouvert où se trouve
l'appreil (jauge) de mesure.
Cette pression relative sera donc souvent exprimée en barre (bar) où la différence est faite en fonction de la pression atmosphérique.
Cette pression relative sera donc positive ou négative. Au niveau de la mer (0m d'altitude) cette pression vaut 1,01325 bar (soit 1013.25 hpa).

*** CALCULS ***
Aussi, une pression négative (communément dénommée "dépression"), sera l'expression de cette différence de pression entre 2 points.
Par calcul, pour une valeure de dépression de -0.250 bar (manomètrique) la pression ABSOLUE correspondante vaudra :
1.01325 - 0.250 = 0.76325 bar ou encore 763.25 hpa.
Par calcul, pour une valeure de dépression de -0.175 bar (manomètrique) la pression ABSOLUE correspondante vaudra :
1.01325 - 0.175 = 0.83825 bar ou encore 838.25 hpa.
*** ***


*********************************************************************************************************************************************************
|                                                               Choix de l'unité de mesure                                                              |
*********************************************************************************************************************************************************

Pour l'affichage/utilisation du dispositif, vous avez le choix entre des mesures en :
- Pression ABSOLUE (hPa). Plage allant 0 (zéro) et +∞ (infini). [chiffres positifs]
- Pression MANOMETRIQUE (bar) Plage allant de -1 (moins un) à 0 (zéro) [chiffres inférieurs ou égaux à zéro]
*/

String Unite = "bar";                  // Par défaut, pour ne pas avoir à affichier de nombre négatif, je préfère affichier des hPa.
// hPa !!! et non pas hpa...

/*
*********************************************************************************************************************************************************
*/


// Valeurs par défaut. NE PAS MODIFIER pour rester dans une plage raisonnable !
unsigned int PressureL_hpa_max = 500;  // Valeure seuil la plus petite de pression absolue. Tend vers 0. Par défaut 700 hPa [Demo : 500 hPa] {Réel : 750 hPa}
unsigned int PressureH_hpa_max = 900;  // Valeure seuil la plus grande de pression absolue. Tend vers 1. Par défaut 800 hPa [Demo : 900 hPa] {Réel : 825 hpa}

float PressureL_bar_max = -0.5;        // Valeure seuil la plus petite de pression manométrique. Tend vers 1. Par défaut -0.3 bar [Demo -0.5 bar] {Réel : -0.250 bar}
float PressureH_bar_max = -0.1;        // Valeure seuil la plus grande de pression manométrique. Tend vers 0. Par défaut -0.2 bar [Demo -0.1 bar] {Réel : -0.175 bar}

byte debugMode = 0;                    // Mode de débuggage
String myVersion = "v04.22.00";        // Version


// Settings for initialisation
unsigned int time_break = 10000;       // Temps minimum pour empècher un redémarrage à chaux du moteur. Protection antidémarrage/anti-dribble. Par défaut : 10000 ms
int atmPressure_hpa = 1008;            // Pression atmosphérique classic. 1013 hpa. Il n'est pas vraiment nécessaire de modifier cette valeure.
float atmPressure_bar = 1.01;          // Pression atmosphérique classic. 1.01 bar. Il n'est pas vraiment nécessaire de modifier cette valeure.
byte uPas = 10;                        // Pas (précision) des potentiomètres. Par défaut : 10
unsigned int frameRate = 500;          // Taux de rafraichissement (en milli-secondes) pour l'exécution du programme. Par défaut : 500 ms

float lowPoint;
float maxPoint;
float Pressure;
float atmPressure;
float PressureL_max;
float PressureH_max;
float ValuePotentioH;
float ValuePotentioL;
char outstr[15];
double puissanceLED = 0.25;            // Facteur de puissance pour la led d'état du système en PWM. De 0 à 1. Par défaut : 0.25

// Attribution des Pins;
#define LedAction 52
#define RelayMoteur 23
#define ButtonValidation 26
#define ButtonForcer 18
#define PinPotentioH A15
#define PinPotentioL A14
#define LedRGB_R 10
#define LedRGB_G 9
#define LedRGB_B 8


// Paramètres LCD ;
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
#define contrastPin 6
unsigned char Contrast = 75;           // Contraste de 0 à 100. Par défaut : 75


// Couleur rouge (255, 0, 0)
// Couleur vert (0, 255, 0)
// Couleur bleu (0, 0, 255)
// Couleur jaune (255, 255, 0)
// Couleur orange (255, 128, 0)
void setLedRGB (int R, int G, int B) {

  analogWrite(LedRGB_R, R * puissanceLED);
  analogWrite(LedRGB_G, G * puissanceLED);
  analogWrite(LedRGB_B, B * puissanceLED);

}

// Bargraph vue-mètre
unsigned long time_now = millis();
unsigned long time_previous = 0;

// https://www.instructables.com/Simple-Progress-Bar-for-Arduino-and-LCD/
byte zero[]  = {B00000, B00000, B00000, B00000, B00000, B00000, B00000, B00000};
byte one[]   = {B10000, B10000, B10000, B10000, B10000, B10000, B10000, B10000};
byte two[]   = {B11000, B11000, B11000, B11000, B11000, B11000, B11000, B11000};
byte three[] = {B11100, B11100, B11100, B11100, B11100, B11100, B11100, B11100};
byte four[]  = {B11110, B11110, B11110, B11110, B11110, B11110, B11110, B11110};
byte five[]  = {B11111, B11111, B11111, B11111, B11111, B11111, B11111, B11111};


// Marche forcée.
// Fonction d'interruption.
void Forced() {

  if (Unite == "hPa") {maxPoint = lowPoint = 0;} else {maxPoint = lowPoint = -1;}

}


// Fonction de boucle infinie en cas de soucis.
void functionExit () {

  while (1) {

    setLedRGB(255, 0, 0);
    digitalWrite(LedAction, LOW);
    digitalWrite(RelayMoteur, LOW);
    delay(100);

  }

}


// Fonction pour le contrôle de présence du capteur. En cas de problème : Exit
// Vous n'avez pas *besoin* des pin reset ni EOC pour la plupart des usages, donc nous les déclarons à -1 et omettons de les connecter.
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

void functionTestSensor () {

  if (debugMode >= 90) {Serial.println("MPRLS Simple Test");}
  if (! mpr.begin(0x18)) {    // Adresse par défaut 0x18 pour ce capteur
  
  Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Error Com MPRLS");
  functionExit();

  }

  if (debugMode >= 90) {Serial.println("Found MPRLS sensor");}

}


// Fonction pour la petite barre de progression (compte à rebours) pour l'anti-drible
void updateProgressBar(unsigned long a, unsigned long b, int lineToPrintOn) {
    double factor = time_break / (1.0*(5*5));        // Répartition du plein moment sur le nombre de colonnes disponnibles (5 caractères * 5 colonnes = 25 colonnes au total à disposition.)
    unsigned long delta = (a - b);                   // Quantité de temps à répartir.
    int rest;                                        // Temps (répartie)
    if (delta >= time_break) {

      rest = time_break;

    } else {

      rest = time_break - (a - b);

    }
    int percent = (rest+1) / factor;
    byte number = percent/5;                         //Nombre de caractères (blocs) entiers
    int remainder = percent%5;                       //Restant (pouillèmes) de la division par 5 sur la variable "percent". https://www.lalanguefrancaise.com/dictionnaire/definition/pouilleme
    if (debugMode >= 10) {

      Serial.print("Valeure prise par delta est de :     ");
      Serial.println(delta);
      Serial.print("Valeure prise par rest est de :      ");
      Serial.println(rest);
      Serial.print("Valeure prise par factor est de :    ");
      Serial.println(factor);
      Serial.print("Valeure prise par percent est de :   ");
      Serial.println(percent);
      Serial.print("Valeure prise par number est de :    ");
      Serial.println(number);
      Serial.print("Valeure prise par remainder est de : ");
      Serial.println(remainder);
      Serial.println("-------");
      //delay(1000);

    }

    if (number > 0) {

      for (int j = 0; j < number; j++) {

        lcd.setCursor(j, lineToPrintOn);
        lcd.write(5);

      }

    }
       lcd.setCursor(number, lineToPrintOn);
       lcd.write(remainder); 
    if (number < 5)	{                         //If using a 20 character display, this should be 20!

      for (int j = number+1; j <= 5; j++)  {  //If using a 20 character display, this should be 20!

      lcd.setCursor(j, lineToPrintOn);
      lcd.write((byte)0);

      }

    } 

 }


// Fonction de normalisation pour les potentiomètres avec un mini/maxi ainsi qu'une précision (pas).
int updatePotentio_hpa(uint8_t PinPotentio, int Potentio_max, int atmPressure) {

  int valueUpdated = (map(analogRead(PinPotentio), 0, 1023, atmPressure, Potentio_max)/uPas)*uPas;
  return valueUpdated;

}
float updatePotentio_bar(uint8_t PinPotentio, float Potentio_max, float atmPressure) {

    float valueUpdated = map(analogRead(PinPotentio), 0, 1023, (atmPressure), (Potentio_max * 1000)/(uPas/2))*(uPas/2);
    return valueUpdated / 1000;

}




void setup() {

  // Interruption
  attachInterrupt(digitalPinToInterrupt(ButtonForcer), Forced, FALLING); //Déclenchement lors d'une chute vers l'état BAS.

  // Affectations
  Serial.begin(115200);
  analogWrite(contrastPin, Contrast);

  lcd.begin(16, 2);
  lcd.createChar(0, zero);
  lcd.createChar(1, one);
  lcd.createChar(2, two);
  lcd.createChar(3, three);
  lcd.createChar(4, four);
  lcd.createChar(5, five);

  pinMode(LedRGB_R, OUTPUT);
  pinMode(LedRGB_G, OUTPUT);
  pinMode(LedRGB_B, OUTPUT);

  setLedRGB(255, 0, 0); // Rouge

  pinMode(LedAction, OUTPUT);
  pinMode(RelayMoteur, OUTPUT);
  pinMode(ButtonValidation, INPUT_PULLUP);
  pinMode(ButtonForcer, INPUT_PULLUP); //Bouton est passant en PULLUP


  functionTestSensor ();    //Test initial sur le capteur MPRLS + écran de démarrage.
  lcd.begin(16, 2);
  lcd.print("  GOTCHA !");
  lcd.setCursor(7, 1);
  lcd.print(myVersion);
  unsigned int i = frameRate * 6;
  if (debugMode > 0) {delay(frameRate);} else {delay(i);} //En cas de débuggage : accélaration.


  // Correspondance hPa <=> bar
  if (Unite == "hPa") {

      Pressure = mpr.readPressure();

  } else if (Unite == "bar") {

    Pressure = mpr.readPressure() / 1000;

  } else if (Unite != "hPa" || "bar") {

    while (1) {

      setLedRGB (255, 0, 0); //Rouge
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("  ! WARNING !");
      lcd.setCursor(0, 1);
      lcd.print("UNDEFINED UNIT !");
      delay(1000);

    }

  }

  
  // INITIALISATION
  setLedRGB (255, 128, 0); // Orange
  lcd.clear();
  lcd.print(" INITIALISATION");
  lcd.setCursor(0, 1);
  lcd.print("****************");
  i = millis();
  unsigned int duration;
  if (debugMode > 0) {duration = frameRate;} else {duration = frameRate * 3;}
  while (millis() < (i + duration)) {

    if ( digitalRead(ButtonValidation) == LOW ) {

      setLedRGB (0, 255, 0); // Vert
      delay(250);
      if ( Unite == "bar" ) {

        Unite = "hPa";

      } else {

          Unite = "bar";
      }

    }

  }

  do {

    setLedRGB (0, 0, 255); // Bleu
    lcd.clear();
    lcd.print("SET low. point :");
    lcd.setCursor(0, 1);
    if (Unite == "hPa") {lowPoint = updatePotentio_hpa(PinPotentioL, PressureL_hpa_max, atmPressure_hpa);} else {lowPoint = updatePotentio_bar(PinPotentioL, PressureL_bar_max, atmPressure_bar);}
    if (Unite == "hPa") {lcd.print(dtostrf(lowPoint, 4, 0, outstr));} else {lcd.print(dtostrf(lowPoint, 5, 3, outstr));}
    lcd.print(" ");
    lcd.print(Unite);
    delay(100);

  } while (digitalRead(ButtonValidation) == HIGH);

  delay(350); //Evite de passer la séquence si le bouton n'est pas relaché assez vite.
  do {

    lcd.clear();
    lcd.print("SET max. point :");
    lcd.setCursor(0, 1);
    if (Unite == "hPa") {maxPoint = updatePotentio_hpa(PinPotentioH, lowPoint, atmPressure_hpa);} else {maxPoint = updatePotentio_bar(PinPotentioH, lowPoint, atmPressure_bar);}
    if (Unite == "hPa") {lcd.print(dtostrf(maxPoint, 4, 0, outstr));} else {lcd.print(dtostrf(maxPoint, 5, 3, outstr));}
    lcd.print(" ");
    lcd.print(Unite);
    delay(100);

  } while (digitalRead(ButtonValidation) == HIGH);

}


void loop() {

  bool Starting1 = 1;
  bool Impulse;

  lcd.clear();
  lcd.print("LET'S STARTED !!");
  delay(2000);
  lcd.clear();
  setLedRGB (0, 255, 0); // Vert

  while (1) { // Boucle infinie
    
    functionTestSensor (); // Test sur le capteur MPRLS
    if (Unite == "hPa") {Pressure = mpr.readPressure();} else {Pressure = (mpr.readPressure() / 1000) - atmPressure_bar;}
    lcd.setCursor(0, 0);
    lcd.print(Unite);
    lcd.print(" Actual:");
    if (Unite == "hPa") {lcd.print(" "); lcd.print(Pressure);} else {lcd.print(dtostrf(Pressure, 5, 2, outstr));}
    //lcd.print(" "); // Efface le reste de la ligne
    if (Pressure >= maxPoint) {                                           // En fonction (700hpa)
      
      lcd.setCursor(6, 1);
      lcd.print("Next:");
      if (Unite == "hPa") {lcd.print(" "); lcd.print(lowPoint);} else {lcd.print(dtostrf(lowPoint, 5, 3, outstr));}
      //lcd.print("   "); // Efface le reste de la ligne
      time_now = millis();
      if (Starting1 == 1) {time_previous = time_now + time_break;}                // Verification si le moteur peut se lancer directement ou s'il doit attendre (anti-drible).
      if (time_now - time_previous >= time_break) {

        updateProgressBar(time_now, time_previous, 2);
        digitalWrite(LedAction, HIGH);
        digitalWrite(RelayMoteur, HIGH);
        Impulse = 1;

      } else {

        updateProgressBar(time_now, time_previous, 2);
        digitalWrite(LedAction, HIGH);                                          // Clignotement car le moteur est en phase de repos (anti-drible).
        delay(50);
        digitalWrite(LedAction, LOW);
        delay(25);
        Impulse = 1;

      }

    } else if (Pressure <= lowPoint) {                                    // En attente (800hpa)  
        
        time_previous = time_now;
        Starting1 = 0;
        Impulse = 0;
        digitalWrite(LedAction, LOW);
        digitalWrite(RelayMoteur, LOW);
        lcd.setCursor(6, 1);
        lcd.print("Next:");
        if (Unite == "hPa") {lcd.print(" "); lcd.print(maxPoint);} else {lcd.print(dtostrf(maxPoint, 5, 3, outstr));}
        //lcd.print("   ");
        
    }

    if (Impulse == 0) {                                                       // Entre les deux valeurs !
      
      updateProgressBar(millis(), time_now, 2);

    } else {

      if (debugMode >= 100) {Serial.println("Zone entre deux.");}
      time_now = millis();

    }

    delay(frameRate);

  }

}