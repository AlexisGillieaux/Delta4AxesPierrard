/*
 * test_moteur_rs485.cpp
 *
 * Test d'UN variateur A6-RS depuis le Teensy 4.1, en Modbus RTU sur RS485
 * (bibliothèque lib/A6RS). Les commandes se tapent dans le moniteur série USB.
 *
 * Câblage (Serial2 du Teensy) :
 *   broche 7 (RX2)        <- RO du MAX3485
 *   broche 8 (TX2)        -> DI du MAX3485
 *   broche 2              -> DE et /RE du MAX3485, reliées ensemble (PIN_RS485_DE)
 *   MAX3485 alimenté en 3,3 V (les broches du Teensy 4.1 ne supportent pas le 5 V),
 *   A/B vers le connecteur RS485 du variateur, GND commun avec le variateur,
 *   résistance de 120 ohms entre A et B aux deux bouts du bus.
 *
 * Lancement : PlatformIO > env:test_moteur > Upload and Monitor
 *   ou :      pio run -e test_moteur -t upload   puis   pio device monitor
 *
 * Commandes (tapez "aide") :
 *   etat                       état, position, angle du codeur, temps d'une trame
 *   on / off                   configuration PP + mise sous couple / coupure
 *   deg <angle> [tr/min]       déplacement relatif en degrés, attend la fin
 *   pas <impulsions> [tr/min]  déplacement relatif en impulsions
 *   sinus <ampl> <période s> [cycles]
 *                              aller-retour 0 -> ampl -> 0 envoyé toutes les 20 ms,
 *                              comme une trajectoire (teste la cadence réelle)
 *   lire <registre> [32]       lit un paramètre, ex. "lire 0x202A" ou "lire 0x4016 32"
 * Entrée pendant un mouvement l'arrête (le moniteur n'envoie le texte qu'à Entrée).
 */

#include <Arduino.h>
#include <A6RS.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ============================================================================
 *                         RÉGLAGES À ADAPTER
 * ========================================================================== */

constexpr int PIN_RS485_DE = 2;            // DE + /RE du MAX3485, reliées ensemble à la broche 2
constexpr uint8_t ADRESSE_VARIATEUR = 1;   // C0A.00 du variateur
constexpr int SENS_MOTEUR = 1;             // -1 si le moteur tourne dans le mauvais sens

constexpr uint16_t VITESSE_DEFAUT_RPM = 5;
constexpr uint16_t VITESSE_MAX_RPM = 60;
constexpr double DEPLACEMENT_MAX_DEG = 180.0;   // par commande "deg" ou "pas"
constexpr double AMPLITUDE_MAX_DEG = 90.0;      // commande "sinus"

constexpr uint32_t PERIODE_US = 20000;     // pas de temps des trajectoires : 20 ms
constexpr double MARGE_VITESSE = 1.15;     // vitesse envoyée = vitesse nécessaire x marge
constexpr uint32_t TEMPS_ACCEL_MS = 5;     // C11.0A / C11.0C

/* ============================================================================ */

BusModbus bus(Serial2, PIN_RS485_DE);      // RX2 = broche 7, TX2 = broche 8
A6RS moteur(bus, ADRESSE_VARIATEUR);
bool connecte = false;
bool sousCouple = false;

static void viderEntree()
{
    delay(5);
    while (Serial.available() > 0) {
        Serial.read();
    }
}

static const char *nomEtat(uint16_t etat)
{
    switch (etat) {
    case SERVO_PAS_PRET: return "pas prêt (puissance coupée ?)";
    case SERVO_PRET: return "prêt (S-OFF)";
    case SERVO_EN_MARCHE: return "en marche (S-ON)";
    case SERVO_DEFAUT: return "DÉFAUT (voir l'afficheur du variateur)";
    default: return "inconnu";
    }
}

static bool connecter()
{
    Serial.printf("Recherche du variateur %u sur Serial2 (RX 7, TX 8, DE %d)...\n",
                  ADRESSE_VARIATEUR, PIN_RS485_DE);
    uint32_t debit = moteur.detecterDebit();
    if (debit == 0) {
        Serial.println("Aucune réponse, à aucun débit. Vérifiez :\n"
                       "  - DE et /RE du MAX3485 reliés à PIN_RS485_DE (ou module à sens automatique et -1) ;\n"
                       "  - RO -> broche 7, DI -> broche 8 (et pas l'inverse) ;\n"
                       "  - A et B pas inversés, GND commun, variateur alimenté ;\n"
                       "  - adresse du variateur C0A.00 = ADRESSE_VARIATEUR.\n"
                       "Tapez \"connecter\" pour réessayer.");
        return false;
    }
    if (!moteur.connecter()) {
        Serial.println("Le variateur répond mais la lecture de C00.02 a échoué.");
        return false;
    }
    Serial.printf("Variateur trouvé à %lu bauds. C00.02 = %lu impulsions/tour (%.7f° par impulsion).\n",
                  (unsigned long)debit, (unsigned long)moteur.impulsionsParTour(),
                  moteur.impulsionsParTour() ? 360.0 / moteur.impulsionsParTour() : 0.0);
    if (moteur.impulsionsParTour() == 0) {
        Serial.println("C00.02 = 0 : le variateur utilise le réducteur électronique, non géré ici. "
                       "Lancez --config-variateur depuis le PC.");
        return false;
    }
    if (debit != 115200) {
        Serial.println("(Pour une trajectoire à 20 ms, passez le variateur à 115200 bauds : "
                       "--config-variateur depuis le PC.)");
    }
    return true;
}

static void commandeEtat()
{
    uint16_t etat = 0;
    int32_t position = 0;
    float angle = 0.0f;
    if (!moteur.lireEtat(etat)) {
        return;
    }
    uint32_t trameUs = bus.dureeDerniereTrameUs();
    if (!moteur.lirePosition(position) || !moteur.lireAngleMecanique(angle)) {
        return;
    }
    Serial.printf("  État du servo (U41.0A)    : %u = %s\n", etat, nomEtat(etat));
    Serial.printf("  Position (U40.16)         : %ld impulsions (%.3f°)\n",
                  (long)position, moteur.impulsionsVersDegres(position));
    Serial.printf("  Angle du codeur (U40.09)  : %.2f°\n", angle);
    Serial.printf("  Liaison                   : %lu bauds, une lecture = %lu µs\n",
                  (unsigned long)bus.debit(), (unsigned long)trameUs);
}

static void commandeOn()
{
    Serial.println("Configuration du variateur (mode PP, déplacements relatifs, pas d'EEPROM)...");
    if (!moteur.configurerModePP(TEMPS_ACCEL_MS)) {
        Serial.println("Échec de la configuration.");
        return;
    }
    Serial.println("Mise sous couple (S-ON)...");
    sousCouple = moteur.mettreSousCouple();
    uint16_t etat = 0;
    if (moteur.lireEtat(etat)) {
        Serial.printf("État du servo : %u = %s\n", etat, nomEtat(etat));
    }
    if (!sousCouple) {
        Serial.println("Le moteur n'est pas sous couple : il ne bougera pas. Coupure par sécurité.");
        moteur.couper();
    }
}

static void commandeOff()
{
    bool ok = moteur.couper();
    sousCouple = false;
    Serial.println(ok ? "Moteur coupé (S-OFF)." : "ERREUR : coupure non confirmée, utilisez l'arrêt d'urgence !");
}

/* Déplacement relatif, puis attente de la fin (ou d'une touche) */
static void deplacer(int32_t impulsions, uint16_t vitesse)
{
    double degres = moteur.impulsionsVersDegres(impulsions);
    if (fabs(degres) > DEPLACEMENT_MAX_DEG || vitesse < 1 || vitesse > VITESSE_MAX_RPM) {
        Serial.printf("Refusé : %.2f° (max %.0f°) à %u tr/min (1 à %u).\n",
                      degres, DEPLACEMENT_MAX_DEG, vitesse, VITESSE_MAX_RPM);
        return;
    }
    int32_t depart = 0;
    if (!moteur.lirePosition(depart) || !moteur.deplacer(SENS_MOTEUR * impulsions, vitesse)) {
        return;
    }
    Serial.printf("Déplacement de %+.3f° (%+ld impulsions) à %u tr/min...\n",
                  degres, (long)impulsions, vitesse);

    int32_t cible = depart + SENS_MOTEUR * impulsions;
    int32_t tolerance = moteur.degresVersImpulsions(0.05);
    uint32_t delaiMs = (uint32_t)(fabs(degres) / (vitesse * 6.0) * 2000.0) + 2000;
    uint32_t debut = millis();
    int32_t position = depart;
    while (millis() - debut < delaiMs) {
        delay(20);
        if (Serial.available() > 0) {
            viderEntree();
            Serial.println("Interrompu : le déplacement en cours se termine, rien d'autre n'est envoyé.");
            return;
        }
        if (moteur.lirePosition(position) && abs(position - cible) <= tolerance) {
            Serial.printf("Arrivé en %lu ms, écart %+ld impulsions.\n",
                          (unsigned long)(millis() - debut), (long)(position - cible));
            return;
        }
    }
    Serial.printf("Pas arrivé après %lu ms : position %ld, cible %ld.\n",
                  (unsigned long)delaiMs, (long)position, (long)cible);
}

/* Aller-retour 0 -> ampl -> 0 en (1 - cos)/2, un point toutes les PERIODE_US, comme
   une trajectoire calculée : le déplacement vers le point i part à l'instant du point i-1. */
static void commandeSinus(double amplitude, double periodeS, int cycles)
{
    const double dt = PERIODE_US / 1e6;
    double vitesseMaxRpm = fabs(amplitude) * M_PI / periodeS / 6.0 * MARGE_VITESSE;
    if (fabs(amplitude) > AMPLITUDE_MAX_DEG || periodeS < 0.2 || cycles < 1 || cycles > 50 ||
        vitesseMaxRpm > VITESSE_MAX_RPM) {
        Serial.printf("Refusé : amplitude max %.0f°, période >= 0,2 s, 1 à 50 cycles, "
                      "vitesse max %u tr/min (demandé : %.1f tr/min).\n",
                      AMPLITUDE_MAX_DEG, VITESSE_MAX_RPM, vitesseMaxRpm);
        return;
    }
    const uint32_t n = (uint32_t)lround(cycles * periodeS / dt);
    const double impulsionsParTour = moteur.impulsionsParTour();
    int32_t depart = 0;
    if (!moteur.lirePosition(depart)) {
        return;
    }
    Serial.printf("Aller-retour de %.2f°, %d cycle(s) de %.2f s : %lu points toutes les %lu ms. "
                  "Entrée l'arrête.\n", amplitude, cycles, periodeS, (unsigned long)n,
                  (unsigned long)(PERIODE_US / 1000));

    int32_t precedent = 0;
    uint32_t commMaxUs = 0, retardMaxUs = 0, nbTropLents = 0;
    bool ok = true;
    uint32_t t0 = micros();
    for (uint32_t i = 1; i <= n; i++) {
        uint32_t echeance = (i - 1) * PERIODE_US;
        while ((uint32_t)(micros() - t0) < echeance) {
        }
        uint32_t debut = micros();
        retardMaxUs = max(retardMaxUs, (uint32_t)(debut - t0) - echeance);
        if (Serial.available() > 0) {
            viderEntree();
            Serial.println("Interrompu.");
            ok = false;
            break;
        }

        double t = i * dt;
        double angle = (i == n) ? 0.0 : amplitude * (1.0 - cos(2.0 * M_PI * t / periodeS)) / 2.0;
        int32_t cible = (int32_t)lround(SENS_MOTEUR * angle / 360.0 * impulsionsParTour);
        int32_t pas = cible - precedent;
        precedent = cible;
        if (pas == 0) {
            continue;
        }
        double rpm = fabs((double)pas) / impulsionsParTour / dt * 60.0;
        uint16_t vitesse = (uint16_t)max(1.0, ceil(rpm * MARGE_VITESSE));
        if (!moteur.deplacer(pas, vitesse)) {
            Serial.printf("Échec Modbus au point %lu : arrêt de l'envoi.\n", (unsigned long)i);
            ok = false;
            break;
        }
        uint32_t comm = micros() - debut;
        commMaxUs = max(commMaxUs, comm);
        if (comm > PERIODE_US) {
            nbTropLents++;
        }
    }

    delay(PERIODE_US / 1000 + 300);                  // laisse finir le dernier déplacement
    Serial.printf("Modbus : %lu µs au plus par point (période %lu µs), retard max %lu µs, "
                  "%lu point(s) plus lent(s) que la période.\n",
                  (unsigned long)commMaxUs, (unsigned long)PERIODE_US,
                  (unsigned long)retardMaxUs, (unsigned long)nbTropLents);
    int32_t fin = 0;
    if (ok && moteur.lirePosition(fin)) {
        int32_t attendu = depart + precedent;
        Serial.printf("Retour au départ : écart %+ld impulsions = %+.4f°\n",
                      (long)(fin - attendu), moteur.impulsionsVersDegres(fin - attendu));
    }
}

static void commandeLire(const char *texteReg, bool bits32)
{
    char *fin = nullptr;
    unsigned long reg = strtoul(texteReg, &fin, 0);
    if (fin == texteReg || *fin != '\0' || reg > 0xFFFF) {
        Serial.println("Registre invalide (ex. 0x4016 pour U40.16).");
        return;
    }
    if (bits32) {
        uint32_t valeur = 0;
        if (moteur.lire32((uint16_t)reg, valeur)) {
            Serial.printf("  0x%04lX = %ld (0x%08lX)\n", reg, (long)(int32_t)valeur, (unsigned long)valeur);
        }
    } else {
        uint16_t valeur = 0;
        if (moteur.lire16((uint16_t)reg, valeur)) {
            Serial.printf("  0x%04lX = %u (signé : %d)\n", reg, valeur, (int16_t)valeur);
        }
    }
}

static void afficherAide()
{
    Serial.println("Commandes :\n"
                   "  etat                         état, position, angle du codeur, temps d'une trame\n"
                   "  on | off                     configuration + mise sous couple | coupure\n"
                   "  deg <angle> [tr/min]         déplacement relatif en degrés (ex. deg 10 5)\n"
                   "  pas <impulsions> [tr/min]    déplacement relatif en impulsions\n"
                   "  sinus <ampl> <T s> [cycles]  aller-retour toutes les 20 ms (ex. sinus 30 4 2)\n"
                   "  lire <registre> [32]         lit un paramètre (ex. lire 0x202A, lire 0x4016 32)\n"
                   "  connecter                    recherche à nouveau le variateur\n"
                   "Entrée pendant un mouvement l'arrête.");
}

static void executer(char *ligne)
{
    char *commande = strtok(ligne, " \t");
    char *arg1 = strtok(nullptr, " \t");
    char *arg2 = strtok(nullptr, " \t");
    char *arg3 = strtok(nullptr, " \t");
    if (commande == nullptr) {
        return;
    }
    if (strcasecmp(commande, "aide") == 0 || strcmp(commande, "?") == 0) {
        afficherAide();
        return;
    }
    if (strcasecmp(commande, "connecter") == 0) {
        connecte = connecter();
        return;
    }
    if (!connecte) {
        Serial.println("Variateur non connecté : tapez \"connecter\".");
        return;
    }

    if (strcasecmp(commande, "etat") == 0) {
        commandeEtat();
    } else if (strcasecmp(commande, "on") == 0) {
        commandeOn();
    } else if (strcasecmp(commande, "off") == 0) {
        commandeOff();
    } else if (strcasecmp(commande, "lire") == 0 && arg1 != nullptr) {
        commandeLire(arg1, arg2 != nullptr && strcmp(arg2, "32") == 0);
    } else if (strcasecmp(commande, "deg") == 0 || strcasecmp(commande, "pas") == 0 ||
               strcasecmp(commande, "sinus") == 0) {
        if (!sousCouple) {
            Serial.println("Moteur coupé : tapez d'abord \"on\".");
        } else if (strcasecmp(commande, "sinus") == 0 && arg1 != nullptr && arg2 != nullptr) {
            commandeSinus(atof(arg1), atof(arg2), arg3 ? atoi(arg3) : 1);
        } else if (arg1 != nullptr) {
            uint16_t vitesse = arg2 ? (uint16_t)constrain(atoi(arg2), 0, 65535) : VITESSE_DEFAUT_RPM;
            int32_t impulsions = (strcasecmp(commande, "deg") == 0)
                                     ? moteur.degresVersImpulsions(atof(arg1))
                                     : (int32_t)atol(arg1);
            deplacer(impulsions, vitesse);
        } else {
            afficherAide();
        }
    } else {
        afficherAide();
    }
}

void setup()
{
    Serial.begin(115200);                 // USB : la vitesse n'a pas d'importance
    while (!Serial && millis() < 3000) {
    }
    Serial.println("\n=== Test variateur A6-RS en RS485 (Teensy 4.1) ===");
    connecte = connecter();
    afficherAide();
}

void loop()
{
    static char ligne[80];
    static size_t longueur = 0;
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\r' || c == '\n') {
            if (longueur > 0) {
                ligne[longueur] = '\0';
                Serial.printf("> %s\n", ligne);
                executer(ligne);
                longueur = 0;
            }
        } else if (longueur < sizeof(ligne) - 1) {
            ligne[longueur++] = c;
        }
    }
}
