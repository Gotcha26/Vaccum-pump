// ToDo list
// Led RGB pour les états du dispositif
// Boucler le test du capteur (COM ok vs NOK)
// Précision sur la dixaine uniquement pour les potentiomètres
// Inverser le sens d'encodage des potentiomètres
// Ajouter un symbale en tête de ligne #2 dans loop pour indiquer ce que doit faire la pression (up/down)

// Inspiré de : https://gitlab.com/pleasurepi/vacbedcontrol @halon1301 sur Twitter


#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPRLS.h>
#include <LiquidCrystal.h>


// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);


// Definition des Pin;
const byte LedAction =  22;
const byte RelayMoteur = 24;
const byte ButtonValidation = 26;
const byte ButtonForcer = 18;
const int PinPotentioH = A15;
const int PinPotentioL = A14;
const byte LedTest = 38;

// Configuration des seuils Bas / Haut;
// Les pression sont données en BAR (bar) donc une pression relative par rapport à la pression ambiante.
// Cette pression ambiante est exprimée en Hecto Pascal (hPa) est vaut 1010 hPa environ.
// La convertion est donc environ de 1000 et donne 1 bar à une pression ambiante.
// Une dépression (exprimée en bar) est évaluée par rapport à la pression ambiante et est donc négative.
// Une dépression (exprimée en hPa) est évaluée par rapport à la pression ambiante et est donc toujours positive.
volatile int lowPoint;  // Point le plus  bas (low) sur l'echelle de la dépression. -0.3 bar = 700 hPa
volatile int maxPoint;  // Point le plus haut (max) sur l'echelle de la dépression. -0.2 bar = 800 hPa

String pressure_hpa;
float ValuePotentioH;
float ValuePotentioL;

// Parametre LCD ;
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
const int contrastPin = 6;
int Contrast=75;    // Contraste de 0 à 100


// Settings for initialisation
int debugMode = 1;

// Fonction d'interruption
void Forced() {
  maxPoint = 0;
  lowPoint = 0;
  Serial.println("Je suis là !");
}

// Fonction de boucle infinie en cas de soucis
void functionExit () {
  while (1) {
    digitalWrite(LedTest, LOW);
    delay(100);
    }
}


void setup() {

  // Affectations
  digitalWrite(LedTest, HIGH);
  Serial.begin(115200);
  analogWrite(contrastPin,Contrast);
  pinMode(LedTest, OUTPUT);
  pinMode(LedAction, OUTPUT);
  pinMode(RelayMoteur, OUTPUT);
  pinMode(ButtonValidation, INPUT_PULLUP);
  pinMode(ButtonForcer, INPUT_PULLUP); // Bouton est passant en PULLUP

  // Interruption
  attachInterrupt(digitalPinToInterrupt(ButtonForcer), Forced, FALLING); // Déclenchement quand une chute vers l'état BAS.

  // Check the MPRLS sensor
  Serial.println("MPRLS Simple Test");
  lcd.begin(16, 2);
  lcd.print("Hello");

  if (! mpr.begin(0x18)) { // Adresse par défaut 0x18 pour ce capteur
      Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
      lcd.clear();
       lcd.setCursor(0, 1);
       lcd.print("Error Com MPRLS");
      functionExit();
  }
  
  Serial.println("Found MPRLS sensor");
  lcd.setCursor(0, 1);
  lcd.print("Setup OK");
  delay(500);

  // Lecture de la pression depus le capteur
  pressure_hpa = mpr.readPressure();
  
  // INITIALISATION
  lcd.clear();
  lcd.print(" INITIALISATION");
   lcd.setCursor(0, 1);
   lcd.print("****************");
  delay(2000);

  do {
    lcd.clear();
    lcd.print("SET low. point :");
     lcd.setCursor(0, 1);
     lowPoint = ((int)analogRead(PinPotentioL));
     lcd.print(lowPoint);
     lcd.print(" hPa");
    delay(100);
    if (debugMode > 0) {
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
     maxPoint = ((int)analogRead(PinPotentioH));
     lcd.print(maxPoint);
     lcd.print(" hPa");
    delay(100);
    if (debugMode > 0) {
      Serial.print("Valeure prise par maxPoint : ");
      Serial.println(lowPoint);
       Serial.print("Etat du bouton de validation : ");
       Serial.println(String(digitalRead(ButtonValidation)));
    }
  }
  while(digitalRead(ButtonValidation) == HIGH);

}

void loop() {

  // Affichage de la datas
  if (debugMode > 0) {
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
    int pressure_hpa = mpr.readPressure();
    lcd.setCursor(0, 0);
    lcd.print("hPa Actual: ");
    lcd.print(pressure_hpa);
    lcd.print(" "); // Efface le reste de la ligne

    if (pressure_hpa >= maxPoint) { // En fonction
      digitalWrite(LedAction, HIGH);
      digitalWrite(RelayMoteur, HIGH);
      lcd.setCursor(5, 1);
      lcd.print("Next : ");
      lcd.print(lowPoint);
      lcd.print("   ");
      delay(300);
    }
    else if (pressure_hpa <= lowPoint) { // En attente
      digitalWrite(LedAction, LOW);
      digitalWrite(RelayMoteur, LOW);
      lcd.setCursor(5, 1);
      lcd.print("Next : ");
      lcd.print(maxPoint);
      lcd.print("   ");
      delay(300);
    }
  }
}