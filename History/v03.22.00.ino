#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPRLS.h>
#include <LiquidCrystal.h>


// Settings for initialisation
volatile int debugMode = 1;
String myVersion = "       v03.22.00";


// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);


// Attribution des Pin;
static const byte LedAction =  22;
static const byte RelayMoteur = 24;
static const byte ButtonValidation = 26;
static const byte ButtonForcer = 18;
static const uint8_t PinPotentioH = A15;
static const uint8_t PinPotentioL = A14;


// Taux de rafraichissement (en milli-secondes) .
int frameRate = 500;  


// Configuration des seuils Bas / Haut;
// Les pressions sont données en BAR (bar) donc une pression relative par rapport à la pression ambiante.
// Cette pression ambiante est exprimée en Hecto Pascal (hPa) est vaut 1010 hPa environ.
// La convertion est donc environ de 1000 et donne 1 bar à une pression ambiante.
// Une dépression (exprimée en bar) est évaluée par rapport à la pression ambiante et est donc négative.
// Une dépression (exprimée en hPa) est évaluée par rapport à la pression ambiante et est donc toujours positive.
volatile int lowPoint;  // Point le plus  bas (low) sur l'echelle de la dépression. -0.3 bar = 700 hPa (1 - 0.3 = 0.7 * 1000 = 700)
volatile int maxPoint;  // Point le plus haut (max) sur l'echelle de la dépression. -0.2 bar = 800 hPa (1 - 0.2 = 0.8 * 1000 = 800)

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


// Fonction d'interruption
void Forced() {

  maxPoint = 0;
  lowPoint = 0;
  Serial.println("Je suis là !");

}


// Fonction de boucle infinie en cas de soucis
void functionExit () {

  while (1) {
    setLedRGB(255, 0, 0);
    digitalWrite(LedAction, LOW);
    digitalWrite(RelayMoteur, LOW);
    delay(100);
    }

}


// Fonction pour le contrôle de présence du capteur. En cas de problème : Exit
void functionTestSensor () {

  Serial.println("MPRLS Simple Test");
  if (! mpr.begin(0x18)) { // Adresse par défaut 0x18 pour ce capteur
  Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
  lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Error Com MPRLS");
  functionExit();
  }
  Serial.println("Found MPRLS sensor");

}


void setup() {

  // Interruption
  attachInterrupt(digitalPinToInterrupt(ButtonForcer), Forced, FALLING); // Déclenchement quand une chute vers l'état BAS.

  // Affectations
  Serial.begin(115200);
  analogWrite(contrastPin, Contrast);

  pinMode(LedRGB_R, OUTPUT);
  pinMode(LedRGB_G, OUTPUT);
  pinMode(LedRGB_B, OUTPUT);

  setLedRGB(0, 0, 255);

  pinMode(LedAction, OUTPUT);
  pinMode(RelayMoteur, OUTPUT);
  pinMode(ButtonValidation, INPUT_PULLUP);
  pinMode(ButtonForcer, INPUT_PULLUP); // Bouton est passant en PULLUP

  // Test sur le capteur MPRLS
  functionTestSensor ();
  if (debugMode >= 1) {
    lcd.begin(16, 2);
    lcd.print("  G O T C H A");
     lcd.setCursor(0, 1);
     lcd.print(myVersion);
    int i = frameRate * 8;
    delay(i);
  }

  // Lecture de la pression depuis le capteur
  pressure_hpa = mpr.readPressure();
  
  // INITIALISATION
  setLedRGB (255, 255, 0);
  lcd.clear();
  lcd.begin(16, 2);
  lcd.print(" INITIALISATION");
   lcd.setCursor(0, 1);
   lcd.print("****************");
  delay(2000);

  do {
    lcd.clear();
    lcd.print("SET low. point :");
     lcd.setCursor(0, 1);
     lowPoint = (int(analogRead(PinPotentioL))/10)*10;
     lcd.print(lowPoint);
     lcd.print(" hPa");
    delay(100);
    if (debugMode >= 1) {
      Serial.print("Valeure prise par lowPoint : ");
      Serial.println(lowPoint);
       Serial.print("Etat du bouton de validation : ");
       Serial.println(String(digitalRead(ButtonValidation)));
    }
  }
  while (digitalRead(ButtonValidation) == HIGH);
  delay(350); // Evite de passer la séquence si le bouton n'est pas relaché assez vite.
  
  do {
    lcd.clear();
    lcd.print("SET max. point :");
     lcd.setCursor(0, 1);
     maxPoint = (int(analogRead(PinPotentioH))/10)*10;
     lcd.print(maxPoint);
     lcd.print(" hPa");
    delay(100);
    if (debugMode >= 1) {
      Serial.print("Valeure prise par maxPoint : ");
      Serial.println(lowPoint);
       Serial.print("Etat du bouton de validation : ");
       Serial.println(String(digitalRead(ButtonValidation)));
    }
  }
  while (digitalRead(ButtonValidation) == HIGH);


}


void loop() {

  setLedRGB (0, 255, 0);

  // Affichage de la datas
  if (debugMode >= 1) {
  Serial.print("Pression actuelle : ");
  Serial.print(pressure_hpa);
  Serial.println(" hPa");
  
  ValuePotentioH = (analogRead(PinPotentioH));
  Serial.print("Valeure prise par PotentioH :");
  Serial.println(ValuePotentioH);
  
  ValuePotentioL = (analogRead(PinPotentioL));
  Serial.print("Valeure prise par PotentioL :");
  Serial.println(ValuePotentioL);

  Serial.print("Etat du bouton de validation : ");
  Serial.println(String(digitalRead(ButtonValidation)));
  }

  lcd.clear();
   lcd.print("LET'S STARTED !!");
  delay(2000);
  lcd.clear();

  while (1) { // Boucle infinie
    functionTestSensor (); // Test sur le capteur MPRLS
    int pressure_hpa = mpr.readPressure();
    lcd.setCursor(0, 0);
    lcd.print("hPa Actual: ");
    lcd.print(pressure_hpa);
    lcd.print(" "); // Efface le reste de la ligne
    if (pressure_hpa >= maxPoint) { // En fonction (800hpa)
      digitalWrite(LedAction, HIGH);
      digitalWrite(RelayMoteur, HIGH);
      lcd.setCursor(5, 1);
      lcd.print("Next : ");
      lcd.print(lowPoint);
      lcd.print("   ");
    }
    else if (pressure_hpa <= lowPoint) { // En attente (700hpa)
      digitalWrite(LedAction, LOW);
      digitalWrite(RelayMoteur, LOW);
      lcd.setCursor(5, 1);
      lcd.print("Next : ");
      lcd.print(maxPoint);
      lcd.print("   ");
    }
    delay(frameRate);
  }
}