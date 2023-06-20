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
Cette pression relative sera donc positive ou négative. Au niveau de la mer (0m d'altitude) cette pression vaut 1013.25 hpa (soit 1,01325 bar).

Aussi, une pression négative (communément dénommée "dépression"), sera l'expression de cette différence de pression entre 2 points.
Par calcul, pour une valeure de dépression de -0.250 bar (manomètrique) la pression ABSOLUE correspondante vaudra :
1.01325 - 0.250 = 0.76325 bar ou encore 763.25 hpa.
Par calcul, pour une valeure de dépression de -0.175 bar (manomètrique) la pression ABSOLUE correspondante vaudra :
1.01325 - 0.175 = 0.83825 bar ou encore 838.25 hpa.   

*********************************************************************************************************************************************************
*/


// Settings for initialisation
unsigned int time_break = 10000;    // Temps minimum pour empècher un redémarrage à chaux du relais. Protection anti-drible. Par défaut : 10000 ms
int atmPressure_hpa = 1008;         // Pression atmosphérique classic. 1013 hpa. Il n'est pas vraiment nécessaire de modifier cette valeure.
int PressureL_hpa_max = 700;        // Valeure seuil la plus petite de pression absolue. Par défaut 750 hpa
int PressureH_hpa_max = 800;        // Valeure seuil la plus grande de pression absolue. Par défaut 830 hpa
byte uPas = 10;                     // Pas (précision) des potentiomètres. Par défaut : 10
int frameRate = 500;                // Taux de rafraichissement (en milli-secondes) pour l'exécution du programme. Par défaut : 500 ms

// Attribution des Pin;
static const byte LedAction =  52;
static const byte RelayMoteur = 23;
static const byte ButtonValidation = 26;
static const byte ButtonForcer = 18;
static const uint8_t PinPotentioH = A15;
static const uint8_t PinPotentioL = A14;


volatile int debugMode = 0;
String myVersion = "       v04.00.00";


volatile int lowPoint;
volatile int maxPoint;

String pressure_hpa;
float ValuePotentioH;
float ValuePotentioL;


// Paramètres LCD ;
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
const int contrastPin = 6;
int Contrast=75;    // Contraste de 0 à 100


// LED RGB pour l'état du dispositif.
#define LedRGB_R A0
#define LedRGB_G A1
#define LedRGB_B A2


// Couleur jaune
// setLedRGB(255, 255, 0);
// Couleur orange
// setLedRGB(255, 128, 0);
// Couleur rouge
// setLedRGB(255, 0, 0);
void setLedRGB (int R, int G, int B) {

  analogWrite(LedRGB_R, R);
  analogWrite(LedRGB_G, G);
  analogWrite(LedRGB_B, B);

}

// Bargraph vue-mètre
unsigned long time_now = millis();
unsigned long time_previous = 0;

// https://www.instructables.com/Simple-Progress-Bar-for-Arduino-and-LCD/
byte zero[]   = {B00000, B00000, B00000, B00000, B00000, B00000, B00000, B00000};
byte one[]     = {B10000, B10000, B10000, B10000, B10000, B10000, B10000, B10000};
byte two[]   = {B11000, B11000, B11000, B11000, B11000, B11000, B11000, B11000};
byte three[]  = {B11100, B11100, B11100, B11100, B11100, B11100, B11100, B11100};
byte four[] = {B11110, B11110, B11110, B11110, B11110, B11110, B11110, B11110};
byte five[]   = {B11111, B11111, B11111, B11111, B11111, B11111, B11111, B11111};


// Marche forcée.
// Fonction d'interruption.
void Forced() {

  maxPoint = 0;
  lowPoint = 0;
  Serial.println("Je suis là !");

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
  if (! mpr.begin(0x18)) { // Adresse par défaut 0x18 pour ce capteur
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
    double factor = time_break / (1.0*(5*5));        // Répartition du plein moment sur le nombre de colonnes disponnibles (5*5 = 25 colonnes)
    unsigned long delta = (a - b);                   // Quantité de temps à répartir.
    int rest;                                        // Temps (normalisé)
    if (delta >= time_break) {
      rest = time_break;
    } else {
      rest = time_break - (a - b);
    }
    int percent = (rest+1) / factor;
    byte number = percent/5;                         //Nombre de caractères (blocs) entiers
    int remainder = percent%5;                       //Restant (pouillèmes) de la division par 5 sur la variable "percent". https://www.lalanguefrancaise.com/dictionnaire/definition/pouilleme
    if (debugMode >= 10) {
      Serial.print("Valeure prise par time_now  :        ");
      Serial.println(a);
      Serial.print("Valeure prise par time_previous :    ");
      Serial.println(b);
      Serial.print("Valeure prise par time_break :       ");
      Serial.println(time_break);
      Serial.println("-------");
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
    if(number > 0)
    {
      for(int j = 0; j < number; j++)
      {
        lcd.setCursor(j,lineToPrintOn);
       lcd.write(5);
      }
    }
       lcd.setCursor(number,lineToPrintOn);
       lcd.write(remainder); 
     if(number < 5)	//If using a 20 character display, this should be 20!
    {
      for(int j = number+1; j <= 5; j++)  //If using a 20 character display, this should be 20!
      {
        lcd.setCursor(j,lineToPrintOn);
       lcd.write((byte)0);
      }
    }  
 }


// Fonction de normalisation pour les potentiomètres avec un mini/maxi ainsi qu'une précision (pas).
// Retranche 10 pour compenser le manque de fiabilité des potentiomètres...
int updatePotentio(uint8_t PinPotentio, int Potentio_max, int Potentio_min) {
  int valueUpdated = (map(analogRead(PinPotentio), 0, 1023, Potentio_min, Potentio_max)/uPas)*uPas;
  return valueUpdated;
}




void setup() {

  // Interruption
  attachInterrupt(digitalPinToInterrupt(ButtonForcer), Forced, FALLING); //Déclenchement quand une chute vers l'état BAS.

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

  setLedRGB(0, 0, 255);

  pinMode(LedAction, OUTPUT);
  pinMode(RelayMoteur, OUTPUT);
  pinMode(ButtonValidation, INPUT_PULLUP);
  pinMode(ButtonForcer, INPUT_PULLUP); //Bouton est passant en PULLUP


  functionTestSensor ();    //Test initial sur le capteur MPRLS + écran de démarrage.
  lcd.begin(16, 2);
  lcd.print("  GOTCHA !");
    lcd.setCursor(0, 1);
    lcd.print(myVersion);
  int i = frameRate * 6;
  if (debugMode > 0) {delay(frameRate);} else {delay(i);} //En cas de débuggage : accélaration.


  // Lecture de la pression depuis le capteur
  pressure_hpa = mpr.readPressure();

  
  // INITIALISATION
  lcd.clear();
  lcd.print(" INITIALISATION");
   lcd.setCursor(0, 1);
   lcd.print("****************");
  if (debugMode > 0) {delay(500);} else {delay(1200);} //En cas de débuggage : accélaration.

  do {
    setLedRGB (255, 255, 0);
    lcd.clear();
    lcd.print("SET low. point :");
     lcd.setCursor(0, 1);
     lowPoint = updatePotentio(PinPotentioL, PressureL_hpa_max, atmPressure_hpa);
     lcd.print(lowPoint);
     lcd.print(" hPa");
    delay(100);
    if (debugMode >= 100) {
      Serial.print("Valeure prise par lowPoint : ");
      Serial.println(lowPoint);
       Serial.print("Etat du bouton de validation : ");
       Serial.println(String(digitalRead(ButtonValidation)));
    }
  }
  while (digitalRead(ButtonValidation) == HIGH);
  delay(350); //Evite de passer la séquence si le bouton n'est pas relaché assez vite.
  
  do {
    lcd.clear();
    lcd.print("SET max. point :");
     lcd.setCursor(0, 1);
     maxPoint = updatePotentio(PinPotentioH, lowPoint, atmPressure_hpa);
     lcd.print(maxPoint);
     lcd.print(" hPa");
    delay(100);
    if (debugMode >= 100) {
      Serial.print("Valeure prise par maxPoint : ");
      Serial.println(lowPoint);
       Serial.print("Etat du bouton de validation : ");
       Serial.println(String(digitalRead(ButtonValidation)));
    }
  }
  while (digitalRead(ButtonValidation) == HIGH);

}


void loop() {

  bool Starting1 = 1;
  bool Impulse;

  // Affichage de la datas
  if (debugMode >= 100) {
  Serial.print("Pression actuelle : ");
  Serial.print(pressure_hpa);
  Serial.println(" hPa");
  
  ValuePotentioH = (analogRead(PinPotentioH));
  Serial.print("Valeure ABS prise par PotentioH :");
  Serial.println(ValuePotentioH);
  
  ValuePotentioL = (analogRead(PinPotentioL));
  Serial.print("Valeure ABS prise par PotentioL :");
  Serial.println(ValuePotentioL);

  Serial.print("Etat du bouton de validation : ");
  Serial.println(String(digitalRead(ButtonValidation)));
  }

  lcd.clear();
   lcd.print("LET'S STARTED !!");
  delay(2000);
  lcd.clear();
  setLedRGB (0, 255, 0);

  while (1) { // Boucle infinie
    functionTestSensor (); // Test sur le capteur MPRLS
    int pressure_hpa = mpr.readPressure();
    lcd.setCursor(0, 0);
    lcd.print("hPa Actual: ");
    lcd.print(pressure_hpa);
    lcd.print(" "); // Efface le reste de la ligne
    if (pressure_hpa >= maxPoint) {                                           // En fonction (700hpa)
      lcd.setCursor(6, 1);
      lcd.print("Next: ");
      lcd.print(lowPoint);
      lcd.print("   ");
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
    } else if (pressure_hpa <= lowPoint) {                                    // En attente (800hpa)  
        time_previous = time_now;
        Starting1 = 0;
        Impulse = 0;
        digitalWrite(LedAction, LOW);
        digitalWrite(RelayMoteur, LOW);
        lcd.setCursor(6, 1);
        lcd.print("Next: ");
        lcd.print(maxPoint);
        lcd.print("   ");
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