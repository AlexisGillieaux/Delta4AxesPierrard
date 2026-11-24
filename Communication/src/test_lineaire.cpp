/*
 * test_lineaire.cpp : PHASE 3, déplacement linéaire (ligne droite en XYZ)
 *
 * En ligne droite, la vitesse de chaque moteur change tout au long du mouvement (elle
 * peut même s'inverser). Un seul trapèze par moteur ne suffit plus : la trajectoire
 * calculée par cartesian_trapezoidal_trajectory (Coords/trajectoires4Axes.py) est
 * découpée en NB_SEGMENTS segments de même durée. Chaque variateur enchaîne ses
 * segments sans s'arrêter (C11.00 = 4, "single continuous running" du manuel), chacun
 * avec sa propre vitesse : c'est la liste d'angles, réduite à 17 points.
 * Dans le moniteur série :
 *   a + Entrée   -> aller : les moteurs suivent TRAJET depuis leur position actuelle
 *   r + Entrée   -> retour : la même trajectoire à l'envers
 *   off + Entrée -> coupe les moteurs (S-OFF) : ils se tournent alors à la main
 *   Entrée seule -> affiche la position de chaque moteur
 * Les moteurs restent sous couple entre les mouvements.
 *
 * Trajectoire TRAJET : moteurs 1 et 2 d'une ligne droite de [0, -150, -300] à
 * [0, 150, -300] mm, calculée par :
 *   cartesian_trapezoidal_trajectory([0, -150, -300, 0], [0, 150, -300, 0],
 *       v_max_mm_s=200, a_max_mm_s2=600, dt=0.001, v_max_deg_s=60, a_max_deg_s2=120)
 * (durée 2277 ms) puis échantillonnée en 17 points espacés de 2277 / 16 = 142 ms.
 * C'est exactement ce qu'envoie test_xyz.py pour "0 150 -300 l" depuis [0, -150, -300] :
 * les phases 3 et 4 doivent donner le même mouvement.
 * Le test exécute les déplacements de cette trajectoire à partir de la position
 * actuelle des moteurs (pas besoin d'être à la position de départ).
 * Géométrie (calculée avec DeltaForward, θ3 = θ2) : avec des vitesses parfaites, 16
 * segments écartent la plateforme de 0,2 mm de la vraie ligne droite. Avec les tr/min
 * entiers du plan affiché, l'écart prévu monte à 1,8 mm à l'aller et 6,4 mm au retour.
 *
 * Limites connues, que ce test doit mesurer :
 *   - vitesse en tr/min ENTIERS : un segment ne dure pas exactement DUREE_SEGMENT_MS.
 *     L'avance ou le retard est rattrapé au segment suivant (le programme choisit entre
 *     l'arrondi inférieur et supérieur). Sous 1 tr/min (6 °/s), impossible d'aller plus
 *     lentement : le moteur prend de l'avance, ce qui arrive en début et en fin de
 *     trajectoire ;
 *   - 16 segments au maximum par mouvement (16 groupes C11.06... dans le variateur) ;
 *   - la façon exacte dont le variateur passe d'une vitesse à la suivante n'est pas
 *     décrite dans le manuel (rampes RAMPE_SEGMENT_MS).
 *
 * Réglages au panneau de chaque variateur (le reste est écrit par le programme) :
 *   C0A.00 = son adresse (1, 2), C0A.01 = 7 (115200), C0A.02 = 0, C0A.05 = 0, C0A.06 = 0,
 *   C00.07 = 3, puis éteindre et rallumer le variateur.
 * Câblage : le même que test_moteur_rs485.cpp (module MAX485, DE et RE sur la broche 2).
 *
 * Lancement : PlatformIO > PROJECT TASKS > test_lineaire > Upload and Monitor.
 */

#include <Arduino.h>

/* ============================ RÉGLAGES ============================ */

const uint8_t ADRESSES[] = {1, 2};            // adresses des variateurs du bus (C0A.00)
constexpr uint32_t DEBIT = 115200;            // C0A.01 des variateurs (7 = 115200)
constexpr uint8_t PIN_DE_RE = 2;              // DE et RE du MAX485, reliées ensemble
constexpr uint32_t ATTENTE_REPONSE_MS = 100;  // temps laissé au variateur pour répondre

constexpr size_t NB_SEGMENTS = 16;            // 16 au maximum (groupes du variateur)
constexpr uint32_t DUREE_SEGMENT_MS = 142;    // durée de chaque segment de TRAJET
constexpr uint32_t RAMPE_SEGMENT_MS = 20;     // rampes d'un segment à l'autre (C11.0A, C11.0C)

/* Angles des moteurs 1 et 2 (degrés) aux instants 0, 142, 285... ms (voir en tête) */
const double TRAJET[NB_SEGMENTS + 1][2] = {
    {-38.63,  15.57},   // t =    0 ms
    {-38.37,  14.62},   // t =  142 ms
    {-37.45,  11.81},   // t =  285 ms
    {-35.51,   7.30},   // t =  427 ms
    {-32.56,   2.34},   // t =  569 ms
    {-28.84,  -2.26},   // t =  711 ms
    {-24.38,  -6.46},   // t =  854 ms
    {-19.23, -10.21},   // t =  996 ms
    {-13.47, -13.47},   // t = 1138 ms
    { -7.24, -16.21},   // t = 1281 ms
    { -0.65, -18.40},   // t = 1423 ms
    {  6.16, -20.01},   // t = 1565 ms
    { 13.10, -21.05},   // t = 1708 ms
    { 20.07, -21.53},   // t = 1850 ms
    { 26.02, -21.50},   // t = 1992 ms
    { 29.58, -21.30},   // t = 2134 ms
    { 30.76, -21.20},   // t = 2277 ms
};

constexpr uint16_t VITESSE_LIMITE_RPM = 60;   // sécurité : aucun segment plus vite
constexpr int32_t TOLERANCE_PAS = 10;         // moteur "arrivé" à moins de 10 pas (0,36°) de sa cible
constexpr uint32_t PERIODE_TRACE_MS = 100;    // une ligne du tableau de comparaison toutes les 100 ms

HardwareSerialIMXRT &rs485 = Serial2;         // RX2 = broche 7, TX2 = broche 8

/* ================================================================== */

constexpr size_t NB_MOTEURS = sizeof(ADRESSES);
static_assert(NB_MOTEURS == 2, "TRAJET contient les angles de 2 moteurs");
static_assert(NB_SEGMENTS >= 1 && NB_SEGMENTS <= 16, "le variateur a 16 groupes de segments");

bool sousCouple = false;                      // true : moteurs sous couple, ils tiennent leur position
int32_t pasParTour[NB_MOTEURS] = {};          // C00.02 de chaque variateur

/* Positions lues pendant le mouvement (environ une lecture de chaque moteur toutes les 10 ms) */
struct Echantillon {
    uint32_t t_us[NB_MOTEURS];                // instant de la lecture (micros())
    int32_t pos[NB_MOTEURS];                  // position lue (pas)
};
constexpr size_t MAX_ECHANTILLONS = 4000;     // 40 s de mouvement
Echantillon trace[MAX_ECHANTILLONS];
size_t nbEchantillons = 0;

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
        Serial.printf("   ERREUR variateur %u : pas de réponse (registre 0x%02X%02X)\n",
                      req[0], req[2], req[3]);
        return false;
    }
    if (rep[0] != req[0]
        || crc16(rep, lenRep - 2) != (uint16_t)(rep[lenRep - 2] | (rep[lenRep - 1] << 8))) {
        Serial.printf("   ERREUR variateur %u : réponse abîmée (registre 0x%02X%02X)\n",
                      req[0], req[2], req[3]);
        return false;
    }
    if (rep[1] & 0x80) {
        Serial.printf("   ERREUR variateur %u : refusé, code %u (registre 0x%02X%02X)\n",
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
void couper()
{
    Serial.println("Coupure (S-OFF) des moteurs");
    for (uint8_t adresse : ADRESSES) {
        if (!ecrire16(adresse, 0x0411, 0)) {  // C04.11 = 0 : DI5 (S-ON) inactive
            Serial.printf("   COUPURE NON CONFIRMÉE : coupez l'alimentation du variateur %u !\n", adresse);
        }
    }
    sousCouple = false;
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
   On ne sait pas encore ce que fait le variateur quand un mouvement traverse ce point (la
   position lue saute, l'arrivée ne serait jamais détectée) : ces mouvements sont refusés.
   Retourne true, avec un message, si pos n'est pas dans le même tour que depart. */
bool passeParZero(size_t i, int32_t depart, int32_t pos)
{
    const double n = pasParTour[i];
    if (floor(depart / n) == floor(pos / n)) {
        return false;
    }
    Serial.printf("   Moteur %u : ce mouvement passerait par le 0 du codeur (%ld -> %ld pas, %ld pas par "
                  "tour) : refusé.\n   Faites d'abord l'autre sens (a ou r), ou off puis tournez ce "
                  "moteur à la main.\n",
                  ADRESSES[i], (long)depart, (long)pos, (long)pasParTour[i]);
    return true;
}

/* Registre du déplacement du segment k (0 à 15), tableau 8.13 du manuel : C11.06 pour le
   1er, puis C11.10, C11.20... jusqu'à C11.F0 pour le 16e. À partir de ce registre :
   vitesse +2, rampe d'accélération +4, rampe de décélération +6. */
uint16_t registreSegment(size_t k)
{
    return (k == 0) ? 0x1106 : (uint16_t)(0x1100 + 0x10 * k);
}

/* Vérifie les réglages faits au panneau (le variateur refuse qu'on modifie C0A.05 par
   Modbus), coupe le moteur, puis le configure pour enchaîner NB_SEGMENTS segments
   relatifs. Plusieurs paramètres ne s'écrivent que moteur coupé ("At stop"). */
bool configurer(size_t i)
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
          && ecrire16(adresse, 0x1100, 4)     // C11.00 = 4 : segments enchaînés sans arrêt, une fois
          && ecrire16(adresse, 0x1101, 1)     // C11.01 = 1 : déplacements RELATIFS
          && ecrire16(adresse, 0x1103, 1)     // C11.03 = 1 : premier segment
          && ecrire16(adresse, 0x1104, NB_SEGMENTS)  // C11.04 : dernier segment
          && ecrire16(adresse, 0x1105, 1)     // C11.05 = 1 : chaque départ reprend au 1er segment
          && ecrire16(adresse, 0x0414, 19)    // C04.14 = 19 : DI6 = départ du mouvement
          && lirePasParTour(i))) {
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

/* Moteurs sous couple et en marche : la première fois, configuration puis S-ON ;
   ensuite, vérification qu'aucun servo n'est passé en défaut. En cas d'échec après
   une mise sous couple, tous les moteurs sont coupés. */
bool assurerSousCouple()
{
    uint16_t etat[NB_MOTEURS] = {};
    if (sousCouple) {
        if (tousEnMarche(etat)) {
            return true;
        }
        afficherEtats(etat);
        couper();
        return false;
    }
    Serial.println("Configuration des variateurs");
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        if (!configurer(i)) {
            return false;                     // aucun moteur n'est encore sous couple
        }
    }
    Serial.println("Mise sous couple (S-ON) des moteurs");
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

/* Affiche la position actuelle de chaque moteur (U40.16), en pas et en degrés */
void afficherPositions()
{
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        int32_t position = 0;
        if (lire32(ADRESSES[i], 0x4016, position) && lirePasParTour(i)) {
            Serial.printf("Moteur %u : position %ld pas (%.2f°)\n", ADRESSES[i], (long)position,
                          position * 360.0 / pasParTour[i]);
        }
    }
}

/* ------------------------- Segments ------------------------- */

/* Calcule les segments d'un moteur qui doit passer par pos[0], pos[1]... pos[NB_SEGMENTS]
   (pas) à DUREE_SEGMENT_MS d'intervalle : déplacement dep[k] (pas) et vitesse vit[k]
   (tr/min entiers). Avec des tr/min entiers, un segment dure un peu plus ou un peu moins
   que prévu ; à chaque segment, on garde entre l'arrondi inférieur et supérieur la
   vitesse qui fait finir le segment au plus près de l'instant prévu : l'écart ne
   s'accumule pas. fin[k] : instant réel de fin du segment k (ms).
   Retourne false si un segment dépasse VITESSE_LIMITE_RPM. */
bool planifierSegments(const int32_t pos[NB_SEGMENTS + 1], int32_t pas,
                       int32_t dep[NB_SEGMENTS], uint16_t vit[NB_SEGMENTS], double fin[NB_SEGMENTS])
{
    double t = 0.0;                                    // fin réelle du segment précédent (ms)
    for (size_t k = 0; k < NB_SEGMENTS; k++) {
        dep[k] = pos[k + 1] - pos[k];
        const double d = abs(dep[k]);
        const double finPrevue = (k + 1) * (double)DUREE_SEGMENT_MS;
        if (dep[k] == 0) {                             // segment nul : supposé instantané
            vit[k] = 1;
            fin[k] = t;
            continue;
        }
        const double budget = finPrevue - t;           // temps restant pour ce segment (ms)
        const double vIdeale = (budget > 0.0) ? d * 60000.0 / (pas * budget) : VITESSE_LIMITE_RPM;
        const long vBas = (vIdeale < 1.0) ? 1 : (long)floor(vIdeale);
        const long vHaut = (vIdeale < 1.0) ? 1 : (long)ceil(vIdeale);
        const double finBas = t + d * 60000.0 / (pas * vBas);
        const double finHaut = t + d * 60000.0 / (pas * vHaut);
        const bool garderBas = fabs(finBas - finPrevue) <= fabs(finHaut - finPrevue);
        const long v = garderBas ? vBas : vHaut;
        if (v > VITESSE_LIMITE_RPM) {
            Serial.printf("   Segment %u : %ld tr/min > VITESSE_LIMITE_RPM (%u) : mouvement refusé.\n",
                          (unsigned)(k + 1), v, VITESSE_LIMITE_RPM);
            return false;
        }
        vit[k] = (uint16_t)v;
        t = garderBas ? finBas : finHaut;
        fin[k] = t;
    }
    return true;
}

/* Position prévue (pas) à t ms du départ : interpolation linéaire entre les points */
double positionPrevue(const int32_t pos[NB_SEGMENTS + 1], double t)
{
    if (t <= 0.0) {
        return pos[0];
    }
    const double k = t / DUREE_SEGMENT_MS;
    if (k >= NB_SEGMENTS) {
        return pos[NB_SEGMENTS];
    }
    const size_t k0 = (size_t)k;
    return pos[k0] + (k - k0) * (pos[k0 + 1] - pos[k0]);
}

/* ------------------------- Mouvement ------------------------- */

/* Fait suivre TRAJET aux moteurs à partir de leur position actuelle (à l'envers si retour) */
void bougerLineaire(bool retour)
{
    if (!assurerSousCouple()) {
        return;
    }

    /* 1. Points de passage de chaque moteur (pas), à partir de sa position actuelle */
    int32_t pos[NB_MOTEURS][NB_SEGMENTS + 1];
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        int32_t depart = 0;
        if (!lire32(ADRESSES[i], 0x4016, depart)) {    // U40.16 : position actuelle (pas)
            return;
        }
        const double angle0 = TRAJET[retour ? NB_SEGMENTS : 0][i];
        for (size_t k = 0; k <= NB_SEGMENTS; k++) {
            const double angle = TRAJET[retour ? NB_SEGMENTS - k : k][i];
            pos[i][k] = depart + (int32_t)lround((angle - angle0) * pasParTour[i] / 360.0);
            if (passeParZero(i, depart, pos[i][k])) {
                return;
            }
        }
    }

    /* 2. Vitesse entière de chaque segment */
    int32_t dep[NB_MOTEURS][NB_SEGMENTS];
    uint16_t vit[NB_MOTEURS][NB_SEGMENTS];
    double fin[NB_MOTEURS][NB_SEGMENTS];
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        if (!planifierSegments(pos[i], pasParTour[i], dep[i], vit[i], fin[i])) {
            return;
        }
    }

    /* Début prévu du dernier segment non nul de chaque moteur. Avant cet instant, un moteur
       peut passer tout près de sa position finale sans avoir fini (le moteur 2 à l'aller,
       vers 1850 ms, puis il revient un peu) : il n'est pas encore compté arrivé. */
    double debutDernier[NB_MOTEURS] = {};
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        for (size_t k = 1; k < NB_SEGMENTS; k++) {
            if (dep[i][k] != 0) {
                debutDernier[i] = fin[i][k - 1];
            }
        }
    }
    Serial.printf("Plan (%s, %u segments de %lu ms ; fin réelle = fin du segment avec la vitesse entière) :\n",
                  retour ? "retour" : "aller", (unsigned)NB_SEGMENTS, (unsigned long)DUREE_SEGMENT_MS);
    Serial.print("  seg  fin prévue");
    for (uint8_t adresse : ADRESSES) {
        Serial.printf(" | M%u    pas tr/min fin réelle", adresse);
    }
    Serial.println();
    for (size_t k = 0; k < NB_SEGMENTS; k++) {
        Serial.printf("  %3u  %7lu ms", (unsigned)(k + 1), (unsigned long)((k + 1) * DUREE_SEGMENT_MS));
        for (size_t i = 0; i < NB_MOTEURS; i++) {
            Serial.printf(" |   %+6ld %6u %7.0f ms", (long)dep[i][k], vit[i][k], fin[i][k]);
        }
        Serial.println();
    }

    /* 3. Segments envoyés à chaque variateur (déplacement relatif et vitesse) */
    Serial.println("Envoi des segments");
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        for (size_t k = 0; k < NB_SEGMENTS; k++) {
            if (!(ecrire32(ADRESSES[i], registreSegment(k), dep[i][k])          // déplacement (pas)
                  && ecrire16(ADRESSES[i], registreSegment(k) + 2, vit[i][k]))) { // vitesse (tr/min)
                return;
            }
        }
    }

    /* 4. Départs l'un juste après l'autre. DI6 doit RESTER active pendant tout le
          mouvement : en mode 4, le variateur abandonne les segments restants si elle
          retombe (manuel, tableau 4-9). */
    Serial.println("Déplacement");
    const uint32_t t0 = micros();
    uint32_t depart_us[NB_MOTEURS];
    bool ok = true;
    for (size_t i = 0; i < NB_MOTEURS && ok; i++) {
        depart_us[i] = micros();
        ok = ecrire16(ADRESSES[i], 0x0415, 1);         // C04.15 = 1 : DI6 devient active -> départ
    }

    /* 5. Lecture des positions en boucle jusqu'à l'arrivée de tous les moteurs */
    double arrivee[NB_MOTEURS];               // durée mesurée (ms depuis le départ), -1 : pas arrivé
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        arrivee[i] = -1.0;
    }
    nbEchantillons = 0;
    bool tousArrives = false;
    const uint32_t limite_us = (NB_SEGMENTS * DUREE_SEGMENT_MS + 3000) * 1000;
    while (ok && !tousArrives && micros() - t0 < limite_us && nbEchantillons < MAX_ECHANTILLONS) {
        Echantillon &e = trace[nbEchantillons];
        bool lu = true;
        tousArrives = true;
        for (size_t i = 0; i < NB_MOTEURS && lu; i++) {
            e.t_us[i] = micros();
            lu = lire32(ADRESSES[i], 0x4016, e.pos[i]);
            const double tMoteur = (e.t_us[i] - depart_us[i]) / 1000.0;   // ms depuis son départ
            if (lu && tMoteur >= debutDernier[i] && abs(e.pos[i] - pos[i][NB_SEGMENTS]) <= TOLERANCE_PAS) {
                if (arrivee[i] < 0.0) {
                    arrivee[i] = (e.t_us[i] - depart_us[i]) / 1000.0;
                }
            } else {
                tousArrives = false;
            }
        }
        if (lu) {
            nbEchantillons++;
        }
    }
    /* "Arrivé" = à moins de TOLERANCE_PAS : il reste quelques pas à faire. DI6 reste active
       pendant ce temps, sinon le variateur les abandonnerait. */
    delay(300);
    for (uint8_t adresse : ADRESSES) {
        ecrire16(adresse, 0x0415, 0);                  // DI6 au repos, prête pour le départ suivant
    }
    if (!ok) {
        Serial.println("   Départ non confirmé : mouvement abandonné (les moteurs restent sous couple).");
        return;
    }

    /* 6. Tableau : trajectoire prévue (TRAJET) et mesure, en degrés depuis le départ */
    Serial.println("Comparaison (degrés depuis le départ ; écart = mesuré - prévu) :");
    Serial.print("  t (ms)");
    for (uint8_t adresse : ADRESSES) {
        Serial.printf(" | M%u    prévu  mesuré   écart", adresse);
    }
    Serial.println();
    double ecartMax[NB_MOTEURS] = {};
    uint32_t prochaineLigne = 0;
    for (size_t k = 0; k < nbEchantillons; k++) {
        const Echantillon &e = trace[k];
        const uint32_t tLigne = (e.t_us[0] - t0) / 1000;
        const bool afficher = (tLigne >= prochaineLigne) || (k + 1 == nbEchantillons);
        if (afficher) {
            Serial.printf("  %6lu", (unsigned long)tLigne);
            prochaineLigne = tLigne - tLigne % PERIODE_TRACE_MS + PERIODE_TRACE_MS;
        }
        for (size_t i = 0; i < NB_MOTEURS; i++) {
            const double t = (e.t_us[i] - depart_us[i]) / 1000.0;      // ms depuis le départ du moteur
            const double prevu = (positionPrevue(pos[i], t) - pos[i][0]) * 360.0 / pasParTour[i];
            const double mesure = (e.pos[i] - pos[i][0]) * 360.0 / pasParTour[i];
            ecartMax[i] = max(ecartMax[i], fabs(mesure - prevu));
            if (afficher) {
                Serial.printf(" | %8.2f %7.2f %7.2f", prevu, mesure, mesure - prevu);
            }
        }
        if (afficher) {
            Serial.println();
        }
    }

    /* 7. Résumé */
    delay(300);                               // laisse les moteurs se stabiliser
    Serial.println("Résultat :");
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        int32_t finale = 0;
        lire32(ADRESSES[i], 0x4016, finale);
        if (arrivee[i] < 0.0) {
            Serial.printf("   Moteur %u : PAS ARRIVÉ (position %ld, cible %ld)\n",
                          ADRESSES[i], (long)finale, (long)pos[i][NB_SEGMENTS]);
            continue;
        }
        Serial.printf("   Moteur %u : arrivé après %.0f ms (prévu %lu ms), écart maximal à la "
                      "trajectoire %.2f°, écart final à la cible %+ld pas\n",
                      ADRESSES[i], arrivee[i], (unsigned long)(NB_SEGMENTS * DUREE_SEGMENT_MS),
                      ecartMax[i], (long)(finale - pos[i][NB_SEGMENTS]));
    }
}

/* ------------------------- Moniteur série ------------------------- */

void afficherInvite()
{
    Serial.printf("\nMoteurs %s. Tapez a + Entrée (aller), r + Entrée (retour),\n"
                  "off + Entrée pour couper les moteurs, Entrée seule pour lire les positions.\n",
                  sousCouple ? "SOUS COUPLE" : "hors couple");
}

void setup()
{
    pinMode(PIN_DE_RE, OUTPUT);
    digitalWrite(PIN_DE_RE, LOW);             // réception : la ligne reste libre

    Serial.begin(115200);                     // USB vers le PC
    while (!Serial) {                         // attend l'ouverture du moniteur série
    }
    delay(500);
    rs485.begin(DEBIT, SERIAL_8N1);           // 8 bits, sans parité, 1 bit de stop (C0A.02 = 0)
    Serial.printf("\n=== Phase 3 : ligne droite en %u segments de %lu ms ===\n",
                  (unsigned)NB_SEGMENTS, (unsigned long)DUREE_SEGMENT_MS);
    couper();                                 // état connu au démarrage : moteurs hors couple
    afficherPositions();
    afficherInvite();
}

void loop()
{
    static char ligne[16];
    static size_t n = 0;
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\r') {                      // le moniteur envoie "\r\n" : on traite sur '\n'
            continue;
        }
        if (c != '\n') {
            if (n < sizeof(ligne) - 1) {
                ligne[n++] = c;
            }
            continue;
        }
        ligne[n] = '\0';
        n = 0;
        if (ligne[0] == '\0') {
            afficherPositions();
        } else if (strcmp(ligne, "off") == 0) {
            couper();
        } else if (strcmp(ligne, "a") == 0) {
            bougerLineaire(false);
        } else if (strcmp(ligne, "r") == 0) {
            bougerLineaire(true);
        } else {
            Serial.println("Commandes : a, r, off, ou Entrée seule.");
        }
        afficherInvite();
    }
}
