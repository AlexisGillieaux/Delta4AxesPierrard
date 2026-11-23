/*
 * test_xyz.cpp : PHASE 4, aller à une position X Y Z (côté Teensy)
 *
 * Répartition du robot final : le Raspberry Pi calcule (cinématique inverse,
 * trajectoires4Axes.py), le Teensy fait bouger les moteurs. Pour ce test, le PC joue le
 * rôle du Raspberry Pi avec Communication/test_xyz.py : il calcule les angles des
 * moteurs 1 et 2 à partir de X Y Z et les envoie à ce programme par le câble USB.
 *
 * Commandes (une ligne de texte chacune, angles en degrés). Chaque commande se termine
 * par une ligne "OK θ1 θ2" (angles atteints) ou "ERREUR ..." ; les autres lignes sont
 * des informations. Elles peuvent aussi être tapées à la main dans le moniteur série.
 *   Z θ1 θ2              les moteurs sont actuellement aux angles θ1, θ2 (définit le
 *                        zéro ; avant cette commande, 0° = position 0 du codeur)
 *   P                    angles actuels
 *   J T a θ1 θ2          mouvement articulaire trapézoïdal vers θ1, θ2, de durée T (s),
 *                        accélération a (°/s²) : profils de jointmotangles_trapezoidal,
 *                        calculés par chaque variateur (comme test_articulaire)
 *   L dt θ1 θ2 θ1 θ2 ... ligne droite : NB_SEGMENTS segments de dt ms, chacun donné par
 *                        les angles de sa fin (comme test_lineaire)
 *   OFF                  coupe les moteurs (S-OFF)
 * Les moteurs restent sous couple entre les commandes.
 *
 * Changement de mode : les mouvements J et L n'utilisent pas le même mode du variateur
 * (C11.00 = 3 ou 4), et ce paramètre ne se change que moteur coupé. Passer de J à L (ou
 * l'inverse) coupe donc le couple un court instant (le programme l'affiche). À résoudre
 * avant de monter les bras : ils retomberaient pendant cet instant.
 *
 * Pour le robot final, la même chose se fera par Serial1 (Raspberry Pi) au lieu de
 * Serial (USB).
 *
 * Réglages au panneau de chaque variateur (le reste est écrit par le programme) :
 *   C0A.00 = son adresse (1, 2), C0A.01 = 7 (115200), C0A.02 = 0, C0A.05 = 0, C0A.06 = 0,
 *   C00.07 = 3, puis éteindre et rallumer le variateur.
 * Câblage : le même que test_moteur_rs485.cpp (module MAX485, DE et RE sur la broche 2).
 *
 * Lancement : PlatformIO > PROJECT TASKS > test_xyz > Upload (le port USB doit rester
 * libre pour test_xyz.py), puis : python Communication/test_xyz.py
 */

#include <Arduino.h>
#include <ctype.h>

/* ============================ RÉGLAGES ============================ */

const uint8_t ADRESSES[] = {1, 2};            // adresses des variateurs du bus (C0A.00)
constexpr uint32_t DEBIT = 115200;            // C0A.01 des variateurs (7 = 115200)
constexpr uint8_t PIN_DE_RE = 2;              // DE et RE du MAX485, reliées ensemble
constexpr uint32_t ATTENTE_REPONSE_MS = 100;  // temps laissé au variateur pour répondre

constexpr size_t NB_SEGMENTS = 16;            // segments d'une ligne droite (16 au maximum)
constexpr uint32_t RAMPE_SEGMENT_MS = 20;     // rampes d'un segment à l'autre (C11.0A, C11.0C)

constexpr uint16_t VITESSE_LIMITE_RPM = 60;   // sécurité : aucun moteur plus vite
constexpr bool RAMPE_JUSQUA_1000_RPM = false; // signification de C11.0A (voir test_synchro.cpp)
constexpr int32_t TOLERANCE_PAS = 10;         // moteur "arrivé" à moins de 10 pas (0,36°) de sa cible

/* Angles des bras, convention de DeltaCoord : θ = 0 bras horizontal, θ > 0 bras vers le bas */
const int8_t SENS[] = {1, 1};                 // +1 : θ augmente quand le moteur tourne en sens +,
                                              // -1 sinon (à vérifier moteur par moteur, bras montés)
constexpr double ANGLE_MIN_DEG = -90.0;       // butées logicielles des bras (mêmes valeurs que
constexpr double ANGLE_MAX_DEG = 90.0;        // DeltaCoord) : à adapter aux butées réelles

HardwareSerialIMXRT &rs485 = Serial2;         // RX2 = broche 7, TX2 = broche 8

/* ================================================================== */

constexpr size_t NB_MOTEURS = sizeof(ADRESSES);
static_assert(sizeof(SENS) == NB_MOTEURS, "un SENS par moteur");
static_assert(NB_SEGMENTS >= 1 && NB_SEGMENTS <= 16, "le variateur a 16 groupes de segments");

enum Mode { AUCUN, MODE_PP, MODE_SEGMENTS };  // configuration actuelle des variateurs

bool sousCouple = false;                      // true : moteurs sous couple, ils tiennent leur position
Mode modeActuel = AUCUN;
int32_t pasParTour[NB_MOTEURS] = {};          // C00.02 de chaque variateur
int32_t zeroPas[NB_MOTEURS] = {};             // position (pas) de l'angle 0°, fixée par Z

/* ---------------------- Modbus RTU (inchangé) ---------------------- */

/* CRC16 Modbus (polynôme 0xA001) */
uint16_t crc16(const uint8_t *octets, size_t n)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= octets[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

uint8_t haut(uint16_t mot) { return mot >> 8; }
uint8_t bas(uint16_t mot) { return mot & 0xFF; }

/* Envoie la requête (req[0] = adresse du variateur, les 2 derniers octets reçoivent le
   CRC) et lit la réponse de lenRep octets. Retourne true si elle est complète, vient du
   bon variateur, a un CRC valide, et si le variateur n'a pas refusé la requête. */
bool transaction(uint8_t *req, size_t lenReq, uint8_t *rep, size_t lenRep)
{
    uint16_t crc = crc16(req, lenReq - 2);
    req[lenReq - 2] = bas(crc);               // CRC : octet bas d'abord
    req[lenReq - 1] = haut(crc);

    delay(2);                                 // silence entre deux trames (> 1,75 ms)
    rs485.clear();
    digitalWrite(PIN_DE_RE, HIGH);            // le Teensy prend la ligne
    rs485.write(req, lenReq);
    rs485.flush();                            // attend que le dernier octet soit parti
    digitalWrite(PIN_DE_RE, LOW);             // libère la ligne pour la réponse

    size_t n = 0;
    uint32_t debut = millis();
    while (n < lenRep && millis() - debut < ATTENTE_REPONSE_MS) {
        if (rs485.available() > 0) {
            rep[n++] = (uint8_t)rs485.read();
            if (n == 2 && rep[1] == (req[1] | 0x80)) {
                lenRep = 5;                   // réponse d'erreur : adresse, code, erreur, CRC
            }
        }
    }

    if (n < lenRep) {
        Serial.printf("   Variateur %u : pas de réponse (registre 0x%02X%02X)\n",
                      req[0], req[2], req[3]);
        return false;
    }
    if (rep[0] != req[0]
        || crc16(rep, lenRep - 2) != (uint16_t)(rep[lenRep - 2] | (rep[lenRep - 1] << 8))) {
        Serial.printf("   Variateur %u : réponse abîmée (registre 0x%02X%02X)\n",
                      req[0], req[2], req[3]);
        return false;
    }
    if (rep[1] & 0x80) {
        Serial.printf("   Variateur %u : refusé, code %u (registre 0x%02X%02X)\n",
                      req[0], rep[2], req[2], req[3]);
        return false;
    }
    return true;
}

/* Fonction 0x06 : écrit un paramètre 16 bits */
bool ecrire16(uint8_t adresse, uint16_t reg, uint16_t valeur)
{
    uint8_t req[8] = {adresse, 0x06, haut(reg), bas(reg), haut(valeur), bas(valeur), 0, 0};
    uint8_t rep[8];
    return transaction(req, sizeof(req), rep, sizeof(rep));
}

/* Fonction 0x10 : écrit un paramètre 32 bits (2 registres, mot bas d'abord : C0A.06 = 0) */
bool ecrire32(uint8_t adresse, uint16_t reg, int32_t valeur)
{
    uint16_t motBas = (uint32_t)valeur & 0xFFFF;
    uint16_t motHaut = (uint32_t)valeur >> 16;
    uint8_t req[13] = {adresse, 0x10, haut(reg), bas(reg), 0x00, 0x02, 0x04,
                       haut(motBas), bas(motBas), haut(motHaut), bas(motHaut), 0, 0};
    uint8_t rep[8];
    return transaction(req, sizeof(req), rep, sizeof(rep));
}

/* Fonction 0x03 : lit un paramètre 16 bits */
bool lire16(uint8_t adresse, uint16_t reg, uint16_t &valeur)
{
    uint8_t req[8] = {adresse, 0x03, haut(reg), bas(reg), 0x00, 0x01, 0, 0};
    uint8_t rep[7];
    if (!transaction(req, sizeof(req), rep, sizeof(rep))) {
        return false;
    }
    valeur = (uint16_t)((rep[3] << 8) | rep[4]);
    return true;
}

/* Fonction 0x03 : lit un paramètre 32 bits (mot bas d'abord : C0A.06 = 0) */
bool lire32(uint8_t adresse, uint16_t reg, int32_t &valeur)
{
    uint8_t req[8] = {adresse, 0x03, haut(reg), bas(reg), 0x00, 0x02, 0, 0};
    uint8_t rep[9];
    if (!transaction(req, sizeof(req), rep, sizeof(rep))) {
        return false;
    }
    uint32_t motBas = (uint32_t)((rep[3] << 8) | rep[4]);
    uint32_t motHaut = (uint32_t)((rep[5] << 8) | rep[6]);
    valeur = (int32_t)((motHaut << 16) | motBas);
    return true;
}

/* ------------------ Mise sous couple et coupure ------------------ */

/* Lit l'état de chaque servo (U41.0A : 0 pas prêt, 1 prêt, 2 en marche, 3 défaut).
   Retourne true si tous sont en marche. */
bool tousEnMarche(uint16_t etat[NB_MOTEURS])
{
    bool tous = true;
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        etat[i] = 0;
        lire16(ADRESSES[i], 0x410A, etat[i]);
        if (etat[i] != 2) {
            tous = false;
        }
    }
    return tous;
}

void afficherEtats(const uint16_t etat[NB_MOTEURS])
{
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        if (etat[i] != 2) {
            Serial.printf("   Moteur %u pas en marche : U41.0A = %u (3 = défaut, voir l'afficheur)\n",
                          ADRESSES[i], etat[i]);
        }
    }
}

/* Coupe tous les moteurs (S-OFF) : ils peuvent alors être tournés à la main */
bool couper()
{
    Serial.println("   Coupure (S-OFF) des moteurs");
    bool ok = true;
    for (uint8_t adresse : ADRESSES) {
        if (!ecrire16(adresse, 0x0411, 0)) {  // C04.11 = 0 : DI5 (S-ON) inactive
            Serial.printf("   COUPURE NON CONFIRMÉE : coupez l'alimentation du variateur %u !\n", adresse);
            ok = false;
        }
    }
    sousCouple = false;
    return ok;
}

/* Lit C00.02 (pas par tour moteur) du variateur i */
bool lirePasParTour(size_t i)
{
    if (!lire32(ADRESSES[i], 0x0002, pasParTour[i])) {
        return false;
    }
    if (pasParTour[i] <= 0) {
        Serial.printf("   Variateur %u : C00.02 = 0, il utilise le réducteur électronique, non géré ici.\n",
                      ADRESSES[i]);
        return false;
    }
    return true;
}

/* C00.07 = 3 (absolu mono-tour) : la position du codeur repasse à 0 après pasParTour - 1.
   On ne sait pas encore ce que fait le variateur quand un mouvement traverse ce point (il
   pourrait faire presque un tour dans l'autre sens) : ces mouvements sont refusés.
   Sur le robot, ce point devra être placé là où le bras ne va jamais (C10.1E).
   Retourne true, avec un message, si pos n'est pas dans le même tour que depart. */
bool passeParZero(size_t i, int32_t depart, int32_t pos)
{
    const double n = pasParTour[i];
    if (floor(depart / n) == floor(pos / n)) {
        return false;
    }
    Serial.printf("   Moteur %u : ce mouvement passerait par le 0 du codeur (%ld -> %ld pas, %ld pas par "
                  "tour) : refusé.\n   Tapez off, tournez ce moteur à la main d'environ un demi-tour, "
                  "puis z.\n",
                  ADRESSES[i], (long)depart, (long)pos, (long)pasParTour[i]);
    return true;
}

/* Registre du déplacement du segment k (0 à 15) : C11.06 pour le 1er, puis 10 registres
   par segment (déplacement +0, vitesse +2, rampe d'accélération +4, de décélération +6) */
uint16_t registreSegment(size_t k)
{
    return (uint16_t)(0x1106 + 0x0A * k);
}

/* Vérifie les réglages faits au panneau (le variateur refuse qu'on modifie C0A.05 par
   Modbus), coupe le moteur, puis le configure pour le mode demandé :
   MODE_PP : un trapèze vers une position absolue (C11.00 = 3, C11.01 = 0) ;
   MODE_SEGMENTS : NB_SEGMENTS segments relatifs enchaînés (C11.00 = 4, C11.01 = 1).
   Plusieurs paramètres ne s'écrivent que moteur coupé ("At stop" dans le manuel). */
bool configurer(size_t i, Mode mode)
{
    const uint8_t adresse = ADRESSES[i];
    uint16_t stockage = 1;
    uint16_t ordreMots = 1;
    if (!(lire16(adresse, 0x0A05, stockage)   // C0A.05 : sauvegarde des écritures Modbus en EEPROM
          && lire16(adresse, 0x0A06, ordreMots))) {  // C0A.06 : ordre des mots des valeurs 32 bits
        return false;
    }
    if (stockage != 0 || ordreMots != 0) {
        Serial.printf("   Réglez au panneau du variateur %u C0A.05 = 0 (lu : %u) et C0A.06 = 0 (lu : %u).\n",
                      adresse, stockage, ordreMots);
        return false;
    }
    if (!(ecrire16(adresse, 0x0411, 0)        // C04.11 = 0 : DI5 (S-ON) inactive -> moteur coupé
          && ecrire16(adresse, 0x0415, 0))) { // C04.15 = 0 : DI6 (départ) au repos
        return false;
    }
    delay(100);                               // laisse le servo s'arrêter
    if (!(ecrire16(adresse, 0x0000, 0)        // C00.00 = 0 : mode position
          && ecrire16(adresse, 0x0300, 1)     // C03.00 = 1 : consigne de position interne
          && ecrire16(adresse, 0x0414, 19)    // C04.14 = 19 : DI6 = départ du mouvement
          && lirePasParTour(i))) {
        return false;
    }
    if (mode == MODE_PP) {
        return ecrire16(adresse, 0x1100, 3)   // C11.00 = 3 : mode PP
               && ecrire16(adresse, 0x1101, 0);  // C11.01 = 0 : consigne ABSOLUE
    }
    if (!(ecrire16(adresse, 0x1100, 4)        // C11.00 = 4 : segments enchaînés sans arrêt, une fois
          && ecrire16(adresse, 0x1101, 1)     // C11.01 = 1 : déplacements RELATIFS
          && ecrire16(adresse, 0x1103, 1)     // C11.03 = 1 : premier segment
          && ecrire16(adresse, 0x1104, NB_SEGMENTS)  // C11.04 : dernier segment
          && ecrire16(adresse, 0x1105, 1))) { // C11.05 = 1 : chaque départ reprend au 1er segment
        return false;
    }
    for (size_t k = 0; k < NB_SEGMENTS; k++) {
        if (!(ecrire32(adresse, registreSegment(k) + 4, RAMPE_SEGMENT_MS)      // rampe d'accélération
              && ecrire32(adresse, registreSegment(k) + 6, RAMPE_SEGMENT_MS))) { // rampe de décélération
            return false;
        }
    }
    return true;
}

/* Moteurs sous couple, en marche et configurés pour le mode demandé. Si les variateurs
   sont dans l'autre mode, ils sont reconfigurés : le couple est coupé un instant.
   En cas d'échec après une mise sous couple, tous les moteurs sont coupés. */
bool assurerMode(Mode mode)
{
    uint16_t etat[NB_MOTEURS] = {};
    if (sousCouple && modeActuel == mode) {
        if (tousEnMarche(etat)) {
            return true;
        }
        afficherEtats(etat);
        couper();
        return false;
    }
    Serial.printf("   Configuration des variateurs pour %s%s\n",
                  mode == MODE_PP ? "un mouvement articulaire" : "une ligne droite",
                  sousCouple ? " : COUPLE COUPÉ UN INSTANT" : "");
    modeActuel = AUCUN;                       // tant que la configuration n'est pas finie
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        if (!configurer(i, mode)) {
            couper();
            return false;
        }
    }
    sousCouple = false;                       // configurer() a coupé chaque moteur
    modeActuel = mode;
    bool ok = true;
    for (size_t i = 0; i < NB_MOTEURS && ok; i++) {
        ok = ecrire16(ADRESSES[i], 0x0411, 1);  // C04.11 = 1 : DI5 non câblée devient active
    }
    bool enMarche = false;
    uint32_t debut = millis();
    while (ok && !enMarche && millis() - debut < 1000) {
        delay(50);
        enMarche = tousEnMarche(etat);
    }
    if (!enMarche) {
        if (ok) {
            afficherEtats(etat);
        }
        couper();
        return false;
    }
    sousCouple = true;
    return true;
}

/* ------------------------- Angles <-> pas ------------------------- */

double angleDe(size_t i, int32_t pas)
{
    return SENS[i] * (pas - zeroPas[i]) * 360.0 / pasParTour[i];
}

int32_t pasDe(size_t i, double angle)
{
    return zeroPas[i] + SENS[i] * (int32_t)lround(angle * pasParTour[i] / 360.0);
}

/* true, avec un message, si l'angle sort des butées logicielles du bras */
bool horsLimites(size_t i, double angle)
{
    if (angle >= ANGLE_MIN_DEG && angle <= ANGLE_MAX_DEG) {
        return false;
    }
    Serial.printf("   Moteur %u : %.2f° hors des butées [%.0f°, %.0f°] : refusé.\n",
                  ADRESSES[i], angle, ANGLE_MIN_DEG, ANGLE_MAX_DEG);
    return true;
}

/* Lit la position (pas) de chaque moteur et C00.02 */
bool lirePositions(int32_t pos[NB_MOTEURS])
{
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        if (!(lirePasParTour(i) && lire32(ADRESSES[i], 0x4016, pos[i]))) {   // U40.16 : position
            return false;
        }
    }
    return true;
}

/* ------------- Profils (mêmes formules que test_articulaire.cpp) ------------- */

/* Vitesse de croisière (°/s) du trapèze qui parcourt d degrés en exactement T s avec
   l'accélération a, comme _profil_trapeze de trajectoires4Axes.py */
double vitesseCroisiere(double d, double T, double a)
{
    double discriminant = (a * T) * (a * T) - 4.0 * a * d;
    if (discriminant < 0.0) {
        discriminant = 0.0;
    }
    return (a * T - sqrt(discriminant)) / 2.0;
}

/* Durée (ms) d'un mouvement de d pas (d > 0) fait par le variateur en mode PP, à la
   vitesse v (tr/min) avec des rampes C11.0A = C11.0C = rampe (ms) */
double dureeProfil(int32_t d, uint16_t v, uint32_t rampe, int32_t pas)
{
    const double vitesse = v * (double)pas / 60000.0;                            // pas/ms
    const double tr = RAMPE_JUSQUA_1000_RPM ? rampe * v / 1000.0 : (double)rampe;  // durée d'une rampe (ms)
    if (d >= vitesse * tr) {
        return d / vitesse + tr;              // trapèze : rampe, vitesse constante, rampe
    }
    return 2.0 * sqrt(d * tr / vitesse);      // triangle : la vitesse v n'est jamais atteinte
}

/* Vitesse v (tr/min entiers) proche de vSouhaitee et rampe (ms) pour que le mouvement
   de d pas (d > 0) dure T ms. Retourne false si v dépasse VITESSE_LIMITE_RPM. */
bool planifier(int32_t d, double T, double vSouhaitee, int32_t pas, uint16_t &v, uint32_t &rampe)
{
    long vChoisie = lround(vSouhaitee);
    const long vMin = (long)ceil(d * 60000.0 / (pas * T));   // plus lent : n'arriverait pas à temps
    if (vChoisie < vMin) {
        vChoisie = vMin;
    }
    if (vChoisie < 1) {
        vChoisie = 1;
    }
    if (vChoisie > VITESSE_LIMITE_RPM) {
        Serial.printf("   Vitesse nécessaire %ld tr/min > VITESSE_LIMITE_RPM (%u)\n",
                      vChoisie, VITESSE_LIMITE_RPM);
        return false;
    }
    v = (uint16_t)vChoisie;
    uint32_t rMin = 0;
    uint32_t rMax = 65535;
    if (dureeProfil(d, v, rMax, pas) <= T) {
        rampe = rMax;
        Serial.println("   Attention : rampe maximale atteinte, ce moteur arrivera en avance.");
        return true;
    }
    while (rMax - rMin > 1) {                 // la durée augmente avec la rampe : dichotomie
        const uint32_t milieu = (rMin + rMax) / 2;
        if (dureeProfil(d, v, milieu, pas) < T) {
            rMin = milieu;
        } else {
            rMax = milieu;
        }
    }
    rampe = (T - dureeProfil(d, v, rMin, pas) <= dureeProfil(d, v, rMax, pas) - T) ? rMin : rMax;
    return true;
}

/* Segments d'un moteur qui doit passer par pos[0]... pos[NB_SEGMENTS] (pas) toutes les
   dureeSeg ms (voir test_lineaire.cpp) : déplacement dep[k], vitesse entière vit[k].
   debutDernier : début prévu (ms) du dernier segment non nul.
   Retourne l'écart maximal (ms) entre la fin réelle et la fin prévue d'un segment,
   ou -1 si un segment dépasse VITESSE_LIMITE_RPM. */
double planifierSegments(const int32_t pos[NB_SEGMENTS + 1], int32_t pas, uint32_t dureeSeg,
                         int32_t dep[NB_SEGMENTS], uint16_t vit[NB_SEGMENTS], double &debutDernier)
{
    double t = 0.0;                                    // fin réelle du segment précédent (ms)
    double ecartMax = 0.0;
    debutDernier = 0.0;
    for (size_t k = 0; k < NB_SEGMENTS; k++) {
        dep[k] = pos[k + 1] - pos[k];
        const double d = abs(dep[k]);
        const double finPrevue = (k + 1) * (double)dureeSeg;
        if (dep[k] != 0) {                             // segment nul : supposé instantané
            debutDernier = t;
            const double budget = finPrevue - t;
            const double vIdeale = (budget > 0.0) ? d * 60000.0 / (pas * budget) : VITESSE_LIMITE_RPM;
            const long vBas = (vIdeale < 1.0) ? 1 : (long)floor(vIdeale);
            const long vHaut = (vIdeale < 1.0) ? 1 : (long)ceil(vIdeale);
            const double finBas = t + d * 60000.0 / (pas * vBas);
            const double finHaut = t + d * 60000.0 / (pas * vHaut);
            const bool garderBas = fabs(finBas - finPrevue) <= fabs(finHaut - finPrevue);
            const long v = garderBas ? vBas : vHaut;
            if (v > VITESSE_LIMITE_RPM) {
                Serial.printf("   Segment %u : %ld tr/min > VITESSE_LIMITE_RPM (%u)\n",
                              (unsigned)(k + 1), v, VITESSE_LIMITE_RPM);
                return -1.0;
            }
            vit[k] = (uint16_t)v;
            t = garderBas ? finBas : finHaut;
        } else {
            vit[k] = 1;
        }
        ecartMax = max(ecartMax, fabs(t - finPrevue));
    }
    return ecartMax;
}

/* Lit les positions en boucle jusqu'à ce que chaque moteur soit à moins de TOLERANCE_PAS
   de cible[i], ou limiteMs après t0. Un moteur n'est pas compté arrivé avant pasAvantMs[i]
   (ms depuis son départ) : en ligne droite, il peut passer près de sa position finale
   avant la fin. arrivee[i] : ms depuis le départ du moteur, -1 si pas arrivé.
   Retourne true si tous sont arrivés. */
bool attendreArrivees(const int32_t cible[NB_MOTEURS], const uint32_t depart_us[NB_MOTEURS],
                      const double pasAvantMs[NB_MOTEURS], uint32_t t0, uint32_t limiteMs,
                      double arrivee[NB_MOTEURS])
{
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        arrivee[i] = -1.0;
    }
    bool tousArrives = false;
    while (!tousArrives && micros() - t0 < limiteMs * 1000) {
        tousArrives = true;
        for (size_t i = 0; i < NB_MOTEURS; i++) {
            if (arrivee[i] >= 0.0) {
                continue;
            }
            int32_t position = 0;
            const bool assezTard = (micros() - depart_us[i]) / 1000.0 >= pasAvantMs[i];
            if (assezTard && lire32(ADRESSES[i], 0x4016, position)
                && abs(position - cible[i]) <= TOLERANCE_PAS) {
                arrivee[i] = (micros() - depart_us[i]) / 1000.0;
            } else {
                tousArrives = false;
            }
        }
    }
    return tousArrives;
}

/* Affiche, pour chaque moteur, la durée mesurée et l'écart final à la cible */
void afficherArrivees(const int32_t cible[NB_MOTEURS], const double arrivee[NB_MOTEURS], double prevuMs)
{
    delay(200);                               // laisse les moteurs se stabiliser
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        int32_t finale = 0;
        lire32(ADRESSES[i], 0x4016, finale);
        if (arrivee[i] < 0.0) {
            Serial.printf("   Moteur %u PAS ARRIVÉ : %.2f° au lieu de %.2f°\n",
                          ADRESSES[i], angleDe(i, finale), angleDe(i, cible[i]));
        } else {
            Serial.printf("   Moteur %u arrivé après %.0f ms (prévu %.0f ms), écart final %+ld pas\n",
                          ADRESSES[i], arrivee[i], prevuMs, (long)(finale - cible[i]));
        }
    }
}

/* ------------------------- Commandes ------------------------- */

/* Z θ1 θ2 : les moteurs sont actuellement aux angles donnés */
bool commandeZ(const double angle[NB_MOTEURS])
{
    int32_t pos[NB_MOTEURS];
    if (!lirePositions(pos)) {
        return false;
    }
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        zeroPas[i] = pos[i] - SENS[i] * (int32_t)lround(angle[i] * pasParTour[i] / 360.0);
        Serial.printf("   Moteur %u : position %ld pas = %.2f°\n", ADRESSES[i], (long)pos[i], angle[i]);
    }
    return true;
}

/* J T a θ1 θ2 : trapèzes synchronisés de durée T (s), accélération a (°/s²) */
bool commandeJ(double T, double a, const double angle[NB_MOTEURS])
{
    if (!(T > 0.0 && a > 0.0)) {
        Serial.println("   T et a doivent être > 0");
        return false;
    }
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        if (horsLimites(i, angle[i])) {
            return false;
        }
    }
    if (!assurerMode(MODE_PP)) {
        return false;
    }
    int32_t depart[NB_MOTEURS];
    if (!lirePositions(depart)) {
        return false;
    }
    int32_t cible[NB_MOTEURS];
    uint16_t vitesse[NB_MOTEURS] = {};
    uint32_t rampe[NB_MOTEURS] = {};
    bool bouge[NB_MOTEURS] = {};
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        cible[i] = pasDe(i, angle[i]);
        if (passeParZero(i, depart[i], cible[i])) {
            return false;
        }
        const int32_t d = abs(cible[i] - depart[i]);
        bouge[i] = (d != 0);
        if (!bouge[i]) {
            continue;
        }
        const double dDeg = fabs(angle[i] - angleDe(i, depart[i]));
        const double vCroisiere = vitesseCroisiere(dDeg, T, a);              // °/s
        if (!planifier(d, T * 1000.0, vCroisiere / 6.0, pasParTour[i], vitesse[i], rampe[i])) {
            return false;
        }
        Serial.printf("   Moteur %u : %.2f° -> %.2f°, croisière %.2f °/s -> %u tr/min, rampes %lu ms\n",
                      ADRESSES[i], angleDe(i, depart[i]), angle[i], vCroisiere, vitesse[i],
                      (unsigned long)rampe[i]);
    }
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        if (bouge[i] && !(ecrire32(ADRESSES[i], 0x1106, cible[i])       // C11.06 : position visée
                          && ecrire16(ADRESSES[i], 0x1108, vitesse[i])  // C11.08 : vitesse (tr/min)
                          && ecrire32(ADRESSES[i], 0x110A, rampe[i])    // C11.0A : rampe d'accélération
                          && ecrire32(ADRESSES[i], 0x110C, rampe[i]))) { // C11.0C : rampe de décélération
            return false;
        }
    }
    const uint32_t t0 = micros();
    uint32_t depart_us[NB_MOTEURS];
    bool ok = true;
    for (size_t i = 0; i < NB_MOTEURS && ok; i++) {
        depart_us[i] = micros();
        if (bouge[i]) {
            ok = ecrire16(ADRESSES[i], 0x0415, 1);   // C04.15 = 1 : DI6 devient active -> départ
        }
    }
    delay(10);
    for (uint8_t adresse : ADRESSES) {
        ecrire16(adresse, 0x0415, 0);                // DI6 au repos (le mode PP part sur le front)
    }
    if (!ok) {
        return false;
    }
    double arrivee[NB_MOTEURS];
    const double sansAttente[NB_MOTEURS] = {};       // un trapèze ne repasse jamais par sa cible
    const bool arrives = attendreArrivees(cible, depart_us, sansAttente, t0,
                                          (uint32_t)(T * 1000.0) + 3000, arrivee);
    afficherArrivees(cible, arrivee, T * 1000.0);
    return arrives;
}

/* L dt θ1 θ2 ... : ligne droite en NB_SEGMENTS segments de dt ms ; angle[k][i] = angle
   du moteur i à la fin du segment k */
bool commandeL(double dt, const double angle[NB_SEGMENTS][NB_MOTEURS])
{
    if (!(dt >= 1.0 && dt <= 60000.0)) {
        Serial.println("   dt doit être entre 1 et 60000 ms");
        return false;
    }
    const uint32_t dureeSeg = (uint32_t)lround(dt);
    for (size_t k = 0; k < NB_SEGMENTS; k++) {
        for (size_t i = 0; i < NB_MOTEURS; i++) {
            if (horsLimites(i, angle[k][i])) {
                return false;
            }
        }
    }
    if (!assurerMode(MODE_SEGMENTS)) {
        return false;
    }
    int32_t depart[NB_MOTEURS];
    if (!lirePositions(depart)) {
        return false;
    }
    int32_t pos[NB_MOTEURS][NB_SEGMENTS + 1];
    int32_t dep[NB_MOTEURS][NB_SEGMENTS];
    uint16_t vit[NB_MOTEURS][NB_SEGMENTS];
    int32_t cible[NB_MOTEURS];
    double debutDernier[NB_MOTEURS];
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        pos[i][0] = depart[i];
        for (size_t k = 0; k < NB_SEGMENTS; k++) {
            pos[i][k + 1] = pasDe(i, angle[k][i]);
            if (passeParZero(i, depart[i], pos[i][k + 1])) {
                return false;
            }
        }
        cible[i] = pos[i][NB_SEGMENTS];
        const double ecart = planifierSegments(pos[i], pasParTour[i], dureeSeg, dep[i], vit[i],
                                               debutDernier[i]);
        if (ecart < 0.0) {
            return false;
        }
        Serial.printf("   Moteur %u : %.2f° -> %.2f° en %u segments, écart de temps max dû aux "
                      "tr/min entiers %.0f ms\n",
                      ADRESSES[i], angleDe(i, depart[i]), angle[NB_SEGMENTS - 1][i],
                      (unsigned)NB_SEGMENTS, ecart);
    }
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        for (size_t k = 0; k < NB_SEGMENTS; k++) {
            if (!(ecrire32(ADRESSES[i], registreSegment(k), dep[i][k])          // déplacement (pas)
                  && ecrire16(ADRESSES[i], registreSegment(k) + 2, vit[i][k]))) { // vitesse (tr/min)
                return false;
            }
        }
    }
    /* DI6 reste active pendant tout le mouvement : en mode 4, le variateur abandonne les
       segments restants si elle retombe */
    const uint32_t t0 = micros();
    uint32_t depart_us[NB_MOTEURS];
    bool ok = true;
    for (size_t i = 0; i < NB_MOTEURS && ok; i++) {
        depart_us[i] = micros();
        ok = ecrire16(ADRESSES[i], 0x0415, 1);       // C04.15 = 1 : DI6 devient active -> départ
    }
    double arrivee[NB_MOTEURS] = {};
    bool arrives = false;
    if (ok) {
        arrives = attendreArrivees(cible, depart_us, debutDernier, t0, NB_SEGMENTS * dureeSeg + 3000,
                                   arrivee);
    }
    for (uint8_t adresse : ADRESSES) {
        ecrire16(adresse, 0x0415, 0);                // DI6 au repos, prête pour le départ suivant
    }
    if (!ok) {
        return false;
    }
    afficherArrivees(cible, arrivee, (double)NB_SEGMENTS * dureeSeg);
    return arrives;
}

/* ------------------------- Liaison série ------------------------- */

/* Lit n nombres à partir de p ; p avance après le dernier. false s'il en manque. */
bool lireNombres(char *&p, double valeurs[], size_t n)
{
    for (size_t k = 0; k < n; k++) {
        char *fin = nullptr;
        valeurs[k] = strtod(p, &fin);
        if (fin == p) {
            return false;
        }
        p = fin;
    }
    return true;
}

/* Exécute une ligne de commande ; retourne false en cas d'erreur */
bool executer(char *ligne)
{
    /* Mot de commande, en majuscules */
    char *p = ligne;
    while (*p == ' ') {
        p++;
    }
    char *mot = p;
    while (*p != '\0' && *p != ' ') {
        *p = (char)toupper((unsigned char)*p);
        p++;
    }
    if (*p == ' ') {
        *p++ = '\0';
    }

    bool lu = false;                          // nombres attendus bien lus
    bool ok = false;
    if (strcmp(mot, "P") == 0) {
        lu = true;
        ok = true;
    } else if (strcmp(mot, "OFF") == 0) {
        lu = true;
        ok = couper();
    } else if (strcmp(mot, "Z") == 0) {
        double angle[NB_MOTEURS];
        lu = lireNombres(p, angle, NB_MOTEURS);
        ok = lu && commandeZ(angle);
    } else if (strcmp(mot, "J") == 0) {
        double v[2 + NB_MOTEURS];
        lu = lireNombres(p, v, 2 + NB_MOTEURS);
        ok = lu && commandeJ(v[0], v[1], v + 2);
    } else if (strcmp(mot, "L") == 0) {
        double dt = 0.0;
        double angle[NB_SEGMENTS][NB_MOTEURS];
        lu = lireNombres(p, &dt, 1) && lireNombres(p, &angle[0][0], NB_SEGMENTS * NB_MOTEURS);
        ok = lu && commandeL(dt, angle);
    } else {
        Serial.println("ERREUR commande inconnue (Z, P, J, L ou OFF)");
        return false;
    }
    while (*p == ' ') {
        p++;
    }
    if (!lu || *p != '\0') {
        Serial.printf("ERREUR nombre de valeurs incorrect pour %s\n", mot);
        return false;
    }
    if (!ok) {
        Serial.println("ERREUR (voir les lignes ci-dessus)");
        return false;
    }
    /* Réponse : angles actuels */
    int32_t pos[NB_MOTEURS];
    if (!lirePositions(pos)) {
        Serial.println("ERREUR lecture des positions");
        return false;
    }
    Serial.print("OK");
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        Serial.printf(" %.3f", angleDe(i, pos[i]));
    }
    Serial.println();
    return true;
}

void setup()
{
    pinMode(PIN_DE_RE, OUTPUT);
    digitalWrite(PIN_DE_RE, LOW);             // réception : la ligne reste libre

    Serial.begin(115200);                     // USB vers le PC (plus tard : Serial1, Raspberry Pi)
    while (!Serial) {                         // attend l'ouverture du port série
    }
    delay(500);
    rs485.begin(DEBIT, SERIAL_8N1);           // 8 bits, sans parité, 1 bit de stop (C0A.02 = 0)
    Serial.println("=== Phase 4 : commandes Z, P, J, L, OFF (voir test_xyz.cpp) ===");
    couper();                                 // état connu au démarrage : moteurs hors couple
}

void loop()
{
    static char ligne[600];                   // une commande L : 1 + 2 x 16 nombres
    static size_t n = 0;
    static bool tropLongue = false;
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\r') {
            continue;
        }
        if (c != '\n') {
            if (n < sizeof(ligne) - 1) {
                ligne[n++] = c;
            } else {
                tropLongue = true;
            }
            continue;
        }
        ligne[n] = '\0';
        n = 0;
        if (tropLongue) {
            tropLongue = false;
            Serial.println("ERREUR ligne trop longue");
        } else if (ligne[0] != '\0') {
            executer(ligne);
        }
    }
}
