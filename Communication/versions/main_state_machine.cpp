#include <Arduino.h>
#include <errno.h>
#include <limits.h>
#include <stdlib.h>

// Variante de src/main.cpp : copier ce fichier a sa place pour l'utiliser.
// Chaque commande et chaque point doivent se terminer par '\n'.
// Exemple : START_MATRIX\n1 2 3 4 5\n6 7 8 9 10\nEND_MATRIX\n
constexpr unsigned long BAUD_RATE = 115200;
constexpr size_t NB_AXES = 5;
constexpr size_t MAX_POINTS = 9999;
constexpr size_t TAILLE_LIGNE = 128;
constexpr size_t NB_MOTEURS = 4;
constexpr unsigned long PERIODE_POSITIONS_MS = 100;
// Base provisoire : angle1 angle2 angle3 angle4 duree_ms (entiers).
// La duree est une attente apres l'envoi, pas une vitesse garantie.
// Mettre a true uniquement apres implementation ET validation des fonctions drivers.
constexpr bool DRIVERS_IMPLEMENTES = false;

enum class Etat { ATTENTE, RECEPTION_MATRICE, FREE_DRIVE, CONTROLE_ANGLE };
Etat etat = Etat::ATTENTE;

int matrice[NB_AXES][MAX_POINTS] = {0};
size_t nombrePoints = 0;
bool matriceDisponible = false;
char ligne[TAILLE_LIGNE];
size_t longueurLigne = 0;
bool ligneTropLongue = false;
size_t pointCourant = 0;
bool pointEnvoye = false;
unsigned long debutPoint = 0;
unsigned long dernierRetourPositions = 0;

// Interface materielle a adapter au protocole des drivers Serial2 a Serial5.
void commanderAngle(size_t moteur, int angle)
{
    (void)moteur;
    (void)angle;
}

void commanderCoupleNul(size_t moteur) { (void)moteur; }
void arreterMoteurs() { /* Implementer l'arret physique selon les drivers. */ }
bool lirePositions(int positions[NB_MOTEURS])
{
    (void)positions;
    return false; // Ne pas presenter des positions fictives comme des mesures.
}

void repondre(const char* message)
{
    Serial.println(message);
    Serial1.println(message);
}

// Valider les cinq entiers avant de modifier la matrice.
bool ajouterPoint(char* texte)
{
    int valeurs[NB_AXES];
    char* curseur = texte;
    for (size_t axe = 0; axe < NB_AXES; ++axe) {
        while (*curseur == ' ' || *curseur == '\t') ++curseur;
        char* fin;
        errno = 0;
        long valeur = strtol(curseur, &fin, 10);
        if (fin == curseur || errno == ERANGE ||
            valeur < INT_MIN || valeur > INT_MAX) return false;
        if (*fin != '\0' && *fin != ' ' && *fin != '\t') return false;
        valeurs[axe] = static_cast<int>(valeur);
        curseur = fin;
    }
    while (*curseur == ' ' || *curseur == '\t') ++curseur;
    if (*curseur != '\0') return false;
    if (valeurs[4] <= 0) return false;
    for (size_t axe = 0; axe < NB_AXES; ++axe) {
        matrice[axe][nombrePoints] = valeurs[axe];
    }
    ++nombrePoints;
    return true;
}

void traiterLigne(char* texte)
{
    String commande(texte);
    commande.trim();
    if (commande.length() == 0) return;

    // STOP quitte le mode courant. En reception, la matrice partielle est invalidee.
    // L'arret physique des moteurs devra etre ajoute selon leur protocole.
    if (commande == "STOP") {
        arreterMoteurs();
        if (etat == Etat::RECEPTION_MATRICE) {
            nombrePoints = 0;
            matriceDisponible = false;
        }
        etat = Etat::ATTENTE;
        repondre("STOPPED");
        return;
    }

    switch (etat) {
    case Etat::ATTENTE:
        if (commande == "START_MATRIX") {
            nombrePoints = 0;
            matriceDisponible = false;
            etat = Etat::RECEPTION_MATRICE;
        } else if (commande == "FREE_DRIVE") {
            if (!DRIVERS_IMPLEMENTES) {
                repondre("ERROR_DRIVERS_NOT_IMPLEMENTED");
                break;
            }
            for (size_t moteur = 0; moteur < NB_MOTEURS; ++moteur)
                commanderCoupleNul(moteur);
            dernierRetourPositions = millis();
            etat = Etat::FREE_DRIVE;
            repondre("FREE_DRIVE_STARTED");
        } else if (commande == "CONTROLE_ANGLE") {
            if (!matriceDisponible) {
                repondre("ERROR_NO_MATRIX");
            } else if (!DRIVERS_IMPLEMENTES) {
                repondre("ERROR_DRIVERS_NOT_IMPLEMENTED");
            } else {
                pointCourant = 0;
                pointEnvoye = false;
                etat = Etat::CONTROLE_ANGLE;
                repondre("ANGLE_STARTED");
            }
        } else {
            repondre("ERROR_UNKNOWN_COMMAND");
        }
        break;

    case Etat::RECEPTION_MATRICE:
        if (commande == "END_MATRIX") {
            if (nombrePoints == 0) {
                repondre("ERROR_EMPTY_MATRIX");
                break;
            }
            matriceDisponible = true;
            etat = Etat::ATTENTE;
            repondre("MATRIX_DONE");
        } else if (nombrePoints >= MAX_POINTS) {
            repondre("ERROR_MATRIX_FULL");
        } else if (!ajouterPoint(texte)) {
            repondre("ERROR_EXPECTED_4_ANGLES_AND_POSITIVE_DURATION_MS");
        }
        break;

    case Etat::FREE_DRIVE:
    case Etat::CONTROLE_ANGLE:
        repondre("ERROR_BUSY_USE_STOP");
        break;
    }
}

void lireRaspberry()
{
    // Budget par passage pour laisser aussi du temps aux drivers.
    for (size_t lus = 0; lus < 128 && Serial1.available() > 0; ++lus) {
        char caractere = static_cast<char>(Serial1.read());
        if (caractere == '\n') {
            if (ligneTropLongue) {
                repondre("ERROR_LINE_TOO_LONG");
            } else {
                ligne[longueurLigne] = '\0';
                traiterLigne(ligne);
            }
            longueurLigne = 0;
            ligneTropLongue = false;
        } else if (caractere != '\r' && !ligneTropLongue) {
            if (longueurLigne < TAILLE_LIGNE - 1) {
                ligne[longueurLigne++] = caractere;
            } else {
                ligneTropLongue = true;
            }
        }
    }
}

void free_drive()
{
    unsigned long maintenant = millis();
    if (maintenant - dernierRetourPositions < PERIODE_POSITIONS_MS) return;
    dernierRetourPositions = maintenant;
    int positions[NB_MOTEURS];
    if (!lirePositions(positions)) return;
    Serial1.print("POSITIONS");
    for (size_t moteur = 0; moteur < NB_MOTEURS; ++moteur) {
        Serial1.print(' ');
        Serial1.print(positions[moteur]);
    }
    Serial1.println();
}

void controleAngle()
{
    if (!pointEnvoye) {
        for (size_t moteur = 0; moteur < NB_MOTEURS; ++moteur)
            commanderAngle(moteur, matrice[moteur][pointCourant]);
        debutPoint = millis();
        pointEnvoye = true;
    }
    if (millis() - debutPoint < static_cast<unsigned long>(matrice[4][pointCourant]))
        return;
    ++pointCourant;
    pointEnvoye = false;
    if (pointCourant >= nombrePoints) {
        // La derniere consigne reste active : aucune desactivation implicite.
        etat = Etat::ATTENTE;
        repondre("TRAJECTORY_DONE");
    }
}

void setup()
{
    Serial.begin(BAUD_RATE);
    Serial1.begin(BAUD_RATE);
    Serial2.begin(BAUD_RATE);
    Serial3.begin(BAUD_RATE);
    Serial4.begin(BAUD_RATE);
    Serial5.begin(BAUD_RATE);
    Serial.println("UART du Teensy 4.1 initialise - machine a etats.");
}

void loop()
{
    lireRaspberry();
    if (etat == Etat::FREE_DRIVE) free_drive();
    else if (etat == Etat::CONTROLE_ANGLE) controleAngle();
}
