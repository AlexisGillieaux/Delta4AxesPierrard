/*
 * test_articulaire.cpp : PHASE 2, mouvement articulaire trapézoïdal
 *
 * Même mouvement que jointmotangles_trapezoidal (Coords/trajectoires4Axes.py) : chaque
 * moteur suit un trapèze (accélération A_MAX_DEG_S2, vitesse constante, décélération) ;
 * le moteur le plus lent fixe la durée T et les autres réduisent leur vitesse pour finir
 * au même instant. Dans le moniteur série :
 *   55 -15 + Entrée -> le moteur 1 tourne de +55°, le moteur 2 de -15° (déplacements
 *                      relatifs à la position actuelle, en degrés)
 *   off + Entrée    -> coupe les moteurs (S-OFF) : ils se tournent alors à la main
 *   Entrée seule    -> affiche la position de chaque moteur
 * Les moteurs restent sous couple entre les mouvements.
 *
 * Faut-il envoyer la liste d'angles ou le variateur peut-il calculer le diagramme temps ?
 *   Pour ce mouvement, le diagramme temps de chaque moteur est un trapèze, entièrement
 *   défini par 3 nombres : la position d'arrivée, la vitesse de croisière et la durée
 *   des rampes. Le variateur sait calculer ce trapèze lui-même (mode PP) : le Teensy lui
 *   envoie ces 3 nombres, jamais la liste d'angles. Les formules sont celles de
 *   _temps_min_trapeze et _profil_trapeze.
 *   Seule différence : le variateur n'accepte que des tr/min ENTIERS. La vitesse est
 *   arrondie et la rampe ajustée (à la ms près) pour garder la même durée T : le
 *   trapèze réel est légèrement différent du théorique, l'arrivée a lieu au même instant.
 *
 * Ce que le test affiche : pendant le mouvement, la position lue en continu est
 * comparée au profil théorique de trajectoires4Axes.py, toutes les PERIODE_TRACE_MS.
 * L'écart pendant le mouvement vient surtout du retard du servo sur sa consigne
 * (quelques ms × la vitesse) ; il doit revenir près de 0 à l'arrivée.
 *
 * Réglages au panneau de chaque variateur (le reste est écrit par le programme) :
 *   C0A.00 = son adresse (1, 2), C0A.01 = 7 (115200), C0A.02 = 0, C0A.05 = 0, C0A.06 = 0,
 *   C00.07 = 3, puis éteindre et rallumer le variateur.
 * Câblage : le même que test_moteur_rs485.cpp (module MAX485, DE et RE sur la broche 2).
 *
 * Lancement : PlatformIO > PROJECT TASKS > test_articulaire > Upload and Monitor.
 */

#include <Arduino.h>

/* ============================ RÉGLAGES ============================ */

const uint8_t ADRESSES[] = {1, 2};            // adresses des variateurs du bus (C0A.00)
constexpr uint32_t DEBIT = 115200;            // C0A.01 des variateurs (7 = 115200)
constexpr uint8_t PIN_DE_RE = 2;              // DE et RE du MAX485, reliées ensemble
constexpr uint32_t ATTENTE_REPONSE_MS = 100;  // temps laissé au variateur pour répondre

/* Limites des moteurs, comme v_max_deg_s et a_max_deg_s2 de jointmotangles_trapezoidal.
   60 °/s = 10 tr/min. À plus basse vitesse, l'arrondi aux tr/min entiers pèse plus lourd. */
constexpr double V_MAX_DEG_S = 60.0;
constexpr double A_MAX_DEG_S2 = 120.0;

constexpr uint16_t VITESSE_LIMITE_RPM = 60;   // sécurité : aucun moteur plus vite
constexpr bool RAMPE_JUSQUA_1000_RPM = false; // signification de C11.0A (voir test_synchro.cpp)
constexpr int32_t TOLERANCE_PAS = 10;         // moteur "arrivé" à moins de 10 pas (0,36°) de sa cible
constexpr uint32_t PERIODE_TRACE_MS = 100;    // une ligne du tableau de comparaison toutes les 100 ms

HardwareSerialIMXRT &rs485 = Serial2;         // RX2 = broche 7, TX2 = broche 8

/* ================================================================== */

constexpr size_t NB_MOTEURS = sizeof(ADRESSES);

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
   On ne sait pas encore ce que fait le variateur quand un mouvement traverse ce point (il
   pourrait faire presque un tour dans l'autre sens) : ces mouvements sont refusés.
   Retourne true, avec un message, si pos n'est pas dans le même tour que depart. */
bool passeParZero(size_t i, int32_t depart, int32_t pos)
{
    const double n = pasParTour[i];
    if (floor(depart / n) == floor(pos / n)) {
        return false;
    }
    Serial.printf("   Moteur %u : ce mouvement passerait par le 0 du codeur (%ld -> %ld pas, %ld pas par "
                  "tour) : refusé.\n   Faites le mouvement inverse (ex. -55 15), ou off puis tournez ce "
                  "moteur à la main.\n",
                  ADRESSES[i], (long)depart, (long)pos, (long)pasParTour[i]);
    return true;
}

/* Vérifie les réglages faits au panneau (le variateur refuse qu'on modifie C0A.05 par
   Modbus), coupe le moteur, puis le configure en mode PP absolu. Plusieurs paramètres
   ne s'écrivent que moteur coupé ("At stop" dans le manuel). */
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
    return ecrire16(adresse, 0x0000, 0)       // C00.00 = 0 : mode position
           && ecrire16(adresse, 0x0300, 1)    // C03.00 = 1 : consigne de position interne
           && ecrire16(adresse, 0x1100, 3)    // C11.00 = 3 : mode PP
           && ecrire16(adresse, 0x1101, 0)    // C11.01 = 0 : consigne ABSOLUE
           && ecrire16(adresse, 0x0414, 19)   // C04.14 = 19 : DI6 = départ du mouvement
           && lirePasParTour(i);
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

/* ---------- Profil théorique : mêmes formules que trajectoires4Axes.py ---------- */

/* _temps_min_trapeze : temps minimal (s) pour parcourir d degrés (d >= 0) */
double tempsMinTrapeze(double d, double vMax, double aMax)
{
    if (d <= 0.0) {
        return 0.0;
    }
    if (vMax * vMax / aMax > d) {
        return 2.0 * sqrt(d / aMax);          // triangle : vMax n'est pas atteinte
    }
    return d / vMax + vMax / aMax;            // accélération, vitesse constante, décélération
}

/* Vitesse de croisière (°/s) du trapèze qui parcourt d degrés en exactement T s avec
   l'accélération a : racine de v² - a*T*v + a*d = 0 (voir _profil_trapeze) */
double vitesseCroisiere(double d, double T, double a)
{
    double discriminant = (a * T) * (a * T) - 4.0 * a * d;
    if (discriminant < 0.0) {
        discriminant = 0.0;
    }
    return (a * T - sqrt(discriminant)) / 2.0;
}

/* _profil_trapeze : position (°) à l'instant t (s) du trapèze qui parcourt d en T */
double profilTrapeze(double t, double d, double T, double a)
{
    if (d <= 0.0 || T <= 0.0) {
        return 0.0;
    }
    t = constrain(t, 0.0, T);
    const double v = vitesseCroisiere(d, T, a);
    const double tAcc = v / a;
    if (t < tAcc) {
        return 0.5 * a * t * t;                               // accélération
    }
    if (t <= T - tAcc) {
        return 0.5 * a * tAcc * tAcc + v * (t - tAcc);        // vitesse constante
    }
    return d - 0.5 * a * (T - t) * (T - t);                   // décélération
}

/* ------------------ Profil calculé par le variateur ------------------ */

/* Durée (ms) d'un mouvement de d pas (d > 0) fait par le variateur en mode PP, à la
   vitesse v (tr/min) avec des rampes C11.0A = C11.0C = rampe (ms).
   jusqua1000 : signification de la rampe (voir RAMPE_JUSQUA_1000_RPM). */
double dureeProfil(int32_t d, uint16_t v, uint32_t rampe, int32_t pas, bool jusqua1000)
{
    const double vitesse = v * (double)pas / 60000.0;                  // pas/ms
    const double tr = jusqua1000 ? rampe * v / 1000.0 : (double)rampe;   // durée réelle d'une rampe (ms)
    if (d >= vitesse * tr) {
        return d / vitesse + tr;              // trapèze : rampe, vitesse constante, rampe
    }
    return 2.0 * sqrt(d * tr / vitesse);      // triangle : la vitesse v n'est jamais atteinte
}

/* Choisit la vitesse v (tr/min entiers) la plus proche de vSouhaitee qui permet encore
   d'arriver à temps, puis la rampe (ms) pour que le mouvement de d pas (d > 0) dure
   T ms. Retourne false si v dépasse VITESSE_LIMITE_RPM. Si même la rampe maximale ne
   ralentit pas assez le moteur, il arrivera en avance (message affiché). */
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
        Serial.printf("   Vitesse nécessaire %ld tr/min > VITESSE_LIMITE_RPM (%u) : mouvement refusé.\n",
                      vChoisie, VITESSE_LIMITE_RPM);
        return false;
    }
    v = (uint16_t)vChoisie;

    /* La durée augmente avec la rampe : recherche de la rampe par dichotomie */
    uint32_t rMin = 0;
    uint32_t rMax = 65535;
    if (dureeProfil(d, v, rMax, pas, RAMPE_JUSQUA_1000_RPM) <= T) {
        rampe = rMax;
        Serial.println("   Attention : rampe maximale atteinte, ce moteur arrivera en avance.");
        return true;
    }
    while (rMax - rMin > 1) {
        const uint32_t milieu = (rMin + rMax) / 2;
        if (dureeProfil(d, v, milieu, pas, RAMPE_JUSQUA_1000_RPM) < T) {
            rMin = milieu;
        } else {
            rMax = milieu;
        }
    }
    const double ecartMin = T - dureeProfil(d, v, rMin, pas, RAMPE_JUSQUA_1000_RPM);
    const double ecartMax = dureeProfil(d, v, rMax, pas, RAMPE_JUSQUA_1000_RPM) - T;
    rampe = (ecartMin <= ecartMax) ? rMin : rMax;
    return true;
}

/* ------------------------- Mouvement ------------------------- */

/* Fait tourner le moteur i de delta[i] degrés, en trapèzes synchronisés comme
   jointmotangles_trapezoidal, puis compare le mouvement mesuré au profil théorique */
void bougerArticulaire(const double delta[NB_MOTEURS])
{
    if (!assurerSousCouple()) {
        return;
    }

    /* 1. Positions de départ et cibles */
    int32_t depart[NB_MOTEURS];
    int32_t cible[NB_MOTEURS];
    double distanceDeg[NB_MOTEURS];           // trajet en degrés, toujours >= 0
    double T = 0.0;                           // durée du mouvement (s)
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        if (!lire32(ADRESSES[i], 0x4016, depart[i])) {   // U40.16 : position actuelle (pas)
            return;
        }
        cible[i] = depart[i] + (int32_t)lround(delta[i] * pasParTour[i] / 360.0);
        if (passeParZero(i, depart[i], cible[i])) {
            return;
        }
        distanceDeg[i] = (cible[i] == depart[i]) ? 0.0 : fabs(delta[i]);
        T = max(T, tempsMinTrapeze(distanceDeg[i], V_MAX_DEG_S, A_MAX_DEG_S2));
    }
    if (T <= 0.0) {
        Serial.println("   Aucun moteur n'a de trajet à faire.");
        return;
    }

    /* 2. Profil théorique de chaque moteur, puis vitesse et rampe pour le variateur */
    Serial.printf("Profil (comme jointmotangles_trapezoidal, %.0f °/s, %.0f °/s²) : durée %.0f ms\n",
                  V_MAX_DEG_S, A_MAX_DEG_S2, T * 1000.0);
    uint16_t vitesse[NB_MOTEURS] = {};
    uint32_t rampe[NB_MOTEURS] = {};
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        if (distanceDeg[i] == 0.0) {
            Serial.printf("   Moteur %u : ne bouge pas\n", ADRESSES[i]);
            continue;
        }
        const double vCroisiere = vitesseCroisiere(distanceDeg[i], T, A_MAX_DEG_S2);   // °/s
        const int32_t d = abs(cible[i] - depart[i]);
        if (!planifier(d, T * 1000.0, vCroisiere / 6.0, pasParTour[i], vitesse[i], rampe[i])) {
            return;
        }
        Serial.printf("   Moteur %u : %+.2f°. Théorique : croisière %.2f °/s (%.2f tr/min), "
                      "accélération %.0f ms\n",
                      ADRESSES[i], delta[i], vCroisiere, vCroisiere / 6.0,
                      vCroisiere / A_MAX_DEG_S2 * 1000.0);
        Serial.printf("              Variateur : %u tr/min, rampes %lu ms, durée prévue %.0f ms\n",
                      vitesse[i], (unsigned long)rampe[i],
                      dureeProfil(d, vitesse[i], rampe[i], pasParTour[i], RAMPE_JUSQUA_1000_RPM));
    }

    /* 3. Cible, vitesse et rampes envoyées à chaque variateur qui bouge */
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        if (distanceDeg[i] == 0.0) {
            continue;
        }
        const uint8_t a = ADRESSES[i];
        if (!(ecrire32(a, 0x1106, cible[i])          // C11.06 : position à atteindre (pas)
              && ecrire16(a, 0x1108, vitesse[i])     // C11.08 : vitesse (tr/min)
              && ecrire32(a, 0x110A, rampe[i])       // C11.0A : rampe d'accélération (ms)
              && ecrire32(a, 0x110C, rampe[i]))) {   // C11.0C : rampe de décélération (ms)
            return;
        }
    }

    /* 4. Départs l'un juste après l'autre (t0 : juste avant le premier) */
    Serial.println("Déplacement");
    const uint32_t t0 = micros();
    uint32_t depart_us[NB_MOTEURS];
    bool ok = true;
    for (size_t i = 0; i < NB_MOTEURS && ok; i++) {
        depart_us[i] = micros();
        if (distanceDeg[i] != 0.0) {
            ok = ecrire16(ADRESSES[i], 0x0415, 1);   // C04.15 = 1 : DI6 devient active -> départ
        }
    }
    delay(10);
    for (uint8_t adresse : ADRESSES) {
        ecrire16(adresse, 0x0415, 0);                // DI6 au repos, prête pour le départ suivant
    }
    if (!ok) {
        Serial.println("   Départ non confirmé : mesure abandonnée (les moteurs restent sous couple).");
        return;
    }

    /* 5. Lecture des positions en boucle jusqu'à l'arrivée de tous les moteurs */
    double arrivee[NB_MOTEURS];               // durée mesurée (ms depuis le départ), -1 : pas arrivé
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        arrivee[i] = -1.0;
    }
    nbEchantillons = 0;
    bool tousArrives = false;
    while (!tousArrives && micros() - t0 < (uint32_t)(T * 1000.0 + 3000.0) * 1000
           && nbEchantillons < MAX_ECHANTILLONS) {
        Echantillon &e = trace[nbEchantillons];
        bool lu = true;
        tousArrives = true;
        for (size_t i = 0; i < NB_MOTEURS && lu; i++) {
            e.t_us[i] = micros();
            lu = lire32(ADRESSES[i], 0x4016, e.pos[i]);
            if (lu && abs(e.pos[i] - cible[i]) <= TOLERANCE_PAS) {
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

    /* 6. Tableau : profil théorique (trajectoires4Axes.py) et mesure, en degrés depuis le départ */
    Serial.println("Comparaison (degrés depuis le départ ; écart = mesuré - théorique) :");
    Serial.print("  t (ms)");
    for (uint8_t adresse : ADRESSES) {
        Serial.printf(" | M%u théorique  mesuré   écart", adresse);
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
            const double t = (e.t_us[i] - depart_us[i]) / 1e6;          // s depuis le départ du moteur
            const double sens = (delta[i] < 0.0) ? -1.0 : 1.0;
            const double theorique = sens * profilTrapeze(t, distanceDeg[i], T, A_MAX_DEG_S2);
            const double mesure = (e.pos[i] - depart[i]) * 360.0 / pasParTour[i];
            ecartMax[i] = max(ecartMax[i], fabs(mesure - theorique));
            if (afficher) {
                Serial.printf(" |   %9.2f %7.2f %7.2f", theorique, mesure, mesure - theorique);
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
                          ADRESSES[i], (long)finale, (long)cible[i]);
            continue;
        }
        Serial.printf("   Moteur %u : arrivé après %.0f ms (prévu %.0f ms), écart maximal au profil "
                      "%.2f°, écart final à la cible %+ld pas\n",
                      ADRESSES[i], arrivee[i], T * 1000.0, ecartMax[i], (long)(finale - cible[i]));
    }
}

/* ------------------------- Moniteur série ------------------------- */

void afficherInvite()
{
    Serial.printf("\nMoteurs %s. Tapez le déplacement de chaque moteur en degrés + Entrée (ex. 55 -15),\n"
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
    Serial.printf("\n=== Phase 2 : mouvement articulaire trapézoïdal (%.0f °/s, %.0f °/s²) ===\n",
                  V_MAX_DEG_S, A_MAX_DEG_S2);
    couper();                                 // état connu au démarrage : moteurs hors couple
    afficherPositions();
    afficherInvite();
}

void loop()
{
    static char ligne[64];
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
            afficherInvite();
            continue;
        }
        if (strcmp(ligne, "off") == 0) {
            couper();
            afficherInvite();
            continue;
        }
        /* "55 -15" : un déplacement en degrés par moteur, dans l'ordre de ADRESSES */
        double delta[NB_MOTEURS];
        char *p = ligne;
        bool valide = true;
        for (size_t i = 0; i < NB_MOTEURS && valide; i++) {
            char *fin = nullptr;
            delta[i] = strtod(p, &fin);
            valide = (fin != p);              // au moins un chiffre lu
            p = fin;
        }
        while (*p == ' ') {
            p++;
        }
        if (!valide || *p != '\0') {
            Serial.printf("Tapez %u nombres (un déplacement en degrés par moteur, ex. 55 -15) ou off.\n",
                          (unsigned)NB_MOTEURS);
        } else {
            bougerArticulaire(delta);
            afficherInvite();
        }
    }
}
