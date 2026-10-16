#include <AccelStepper.h>
#include <Encoder.h>

const int PIN_STEP = 2;
const int PIN_DIR  = 3;
const int PIN_EN   = 4;

const int PIN_ENC_A = 5;
const int PIN_ENC_B = 6;

volatile long positionEncodeur = 0;

// Limites mécaniques de la pince
const long POSITION_FERMEE = 0;
const long POSITION_OUVERTE = 4000;

// Vitesse moteur (microsecondes entre deux pas)
const int DELAI_STEP_US = 500;

void setup() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), lireEncodeurA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), lireEncodeurB, CHANGE);

  digitalWrite(PIN_EN, LOW); // active le driver (souvent LOW = actif)

  Serial.begin(115200);

  Serial.println("Systeme pince initialise");
}

void loop() {
  // Exemple de test
  OuverturePince(100);
  delay(2000);

  OuverturePince(30);
  delay(2000);

  OuverturePince(0);
  delay(2000);
}

// --------------------------------------------------
// FONCTION PRINCIPALE
// --------------------------------------------------
void OuverturePince(int pourcentageOuverture) {
  pourcentageOuverture = constrain(pourcentageOuverture, 0, 100);

  long positionCible = map(pourcentageOuverture, 0, 100,
                           POSITION_FERMEE, POSITION_OUVERTE);

  AllerPosition(positionCible);

  Serial.print("Pince ouverte a ");
  Serial.print(pourcentageOuverture);
  Serial.println("%");
}

void FermeturePince(int pourcentageOuverture) {
  pourcentageOuverture = constrain(pourcentageOuverture, 0, 100);

  long positionCible = map(pourcentageOuverture, 0, 100,
                           POSITION_OUVERTE, POSITION_FERMEE);

  AllerPosition(positionCible);

  Serial.print("Pince fermée a ");
  Serial.print(pourcentageOuverture);
  Serial.println("%");
}

// --------------------------------------------------
// DEPLACEMENT VERS UNE POSITION
// --------------------------------------------------
void AllerPosition(long positionCible) {
  long erreur = positionCible - positionEncodeur;

  if (erreur == 0) return;

  if (erreur > 0) {
    digitalWrite(PIN_DIR, HIGH); // sens ouverture
  } else {
    digitalWrite(PIN_DIR, LOW);  // sens fermeture
  }

  while (abs(positionCible - positionEncodeur) > 5) {
    faireUnPas();
  }
}

// --------------------------------------------------
// IMPULSION STEP
// --------------------------------------------------
void faireUnPas() {
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(DELAI_STEP_US);
  digitalWrite(PIN_STEP, LOW);
  delayMicroseconds(DELAI_STEP_US);
}

// --------------------------------------------------
// LECTURE ENCODEUR QUADRATURE
// --------------------------------------------------
void lireEncodeurA() {
  bool A = digitalRead(PIN_ENC_A);
  bool B = digitalRead(PIN_ENC_B);

  if (A == B) {
    positionEncodeur++;
  } else {
    positionEncodeur--;
  }
}

void lireEncodeurB() {
  bool A = digitalRead(PIN_ENC_A);
  bool B = digitalRead(PIN_ENC_B);

  if (A != B) {
    positionEncodeur++;
  } else {
    positionEncodeur--;
  }
}