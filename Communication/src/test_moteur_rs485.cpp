/*
 * test_moteur_rs485.cpp
 *
 * Déplacement SIMULTANÉ des moteurs du bus vers des positions ABSOLUES, depuis le
 * Teensy, en Modbus RTU sur RS485. Les moteurs restent sous couple (S-ON) entre les
 * mouvements : ils tiennent leur position jusqu'à la commande "off". Dans le moniteur :
 *   5108 3000 + Entrée -> moteur 1 au pas 5108 et moteur 2 au pas 3000, en même temps ;
 *                         les moteurs restent ensuite sous couple
 *   off + Entrée       -> coupe les moteurs (S-OFF) : ils se tournent alors à la main
 *   Entrée seule       -> affiche la position de chaque moteur
 * Pour ne bouger qu'un moteur, tapez pour l'autre sa position actuelle.
 * Hors couple, un moteur se tourne à la main ; son variateur continue de compter les pas.
 *
 * Sécurité : au démarrage du Teensy (téléversement, bouton reset), les moteurs sont
 * coupés. Si le Teensy est débranché pendant que les moteurs sont sous couple, ils
 * RESTENT sous couple : pour les couper, rebranchez le Teensy ou coupez l'alimentation
 * des variateurs.
 *
 * Unité : le "pas" est l'unité de position du variateur, C00.02 pas par tour moteur
 * (10000 par défaut : 5108 pas = 183,9°). C00.07 = 3 (absolu monotour, réglé au
 * panneau) : à la mise sous tension, le variateur reprend la position du codeur dans
 * le tour. En consigne absolue, le sens du mouvement est donné par le nombre visé
 * (depuis 0 : 2778 -> +100°, -7222 -> -260°).
 *
 * Principe (rien n'est câblé sur les entrées des variateurs) :
 *   - mode position avec consigne interne en mode PP (C11.00 = 3) et consigne absolue
 *     (C11.01 = 0) : chaque front sur l'entrée DI6 envoie le moteur à la position C11.06,
 *     à la vitesse C11.08 ;
 *   - DI5 (mise sous couple, S-ON) et DI6 (départ du mouvement) ne sont pas câblées :
 *     on les active par Modbus en inversant leur logique (0 = NO, 1 = NC) avec C04.11
 *     et C04.15. Une entrée non câblée en logique NC est vue comme active ;
 *   - au premier mouvement (moteurs hors couple), les variateurs sont configurés puis
 *     mis sous couple : plusieurs paramètres ne s'écrivent que moteur arrêté ("At stop"
 *     dans le manuel). Aux mouvements suivants, seules la cible C11.06 et le départ DI6
 *     sont envoyés (modifiables pendant le fonctionnement) ;
 *   - mouvement simultané : tous les variateurs reçoivent leur cible, PUIS les départs
 *     (DI6) sont envoyés l'un juste après l'autre. L'écart entre deux départs est de
 *     quelques millisecondes (affiché). Les moteurs ont la même vitesse : celui qui a
 *     le moins de chemin arrive le premier ;
 *   - tous ces réglages sont écrits par le programme : les variateurs peuvent garder
 *     leurs valeurs d'usine (C11, C03, C04...).
 *   Adresse Modbus d'un paramètre CGG.II (ou UGG.II) = 0xGGII, ex. C11.08 -> 0x1108.
 *
 * Réglages au panneau de chaque variateur (le variateur refuse qu'on les modifie par
 * Modbus ; le programme vérifie C0A.05 et C0A.06) :
 *   C0A.00 = son adresse (1, 2), C0A.01 = 7 (115200), C0A.02 = 0
 *   C0A.05 = 0 : rien de ce que le programme écrit n'est sauvegardé, le variateur
 *            retrouve ses réglages à la mise sous tension (valeur d'usine : 1)
 *   C0A.06 = 0 : valeurs 32 bits mot bas d'abord (valeur d'usine)
 *   C00.07 = 3, puis éteindre et rallumer le variateur.
 *
 * Câblage : le même que test_rs485_simple.cpp (module MAX485, DE et RE sur la broche 2,
 * variateur 1 CN4 -> variateur 2 CN3 par un câble Ethernet droit).
 *
 * Lancement : PlatformIO > PROJECT TASKS > test_moteur > Upload and Monitor.
 */

#include <Arduino.h>

/* ============================ RÉGLAGES ============================ */

const uint8_t ADRESSES[] = {1, 2};            // adresses des variateurs du bus (C0A.00)
constexpr uint32_t DEBIT = 115200;            // C0A.01 des variateurs (7 = 115200)
constexpr uint8_t PIN_DE_RE = 2;              // DE et RE du MAX485, reliées ensemble
constexpr uint32_t ATTENTE_REPONSE_MS = 100;  // temps laissé au variateur pour répondre

constexpr uint16_t VITESSE_RPM = 5;           // vitesse des moteurs (1 tr/min = 6°/s)

HardwareSerialIMXRT &rs485 = Serial2;         // RX2 = broche 7, TX2 = broche 8

/* ================================================================== */

constexpr size_t NB_MOTEURS = sizeof(ADRESSES);

bool sousCouple = false;                      // true : moteurs sous couple, ils tiennent leur position

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

/* Affiche la position actuelle de chaque moteur (U40.16, en pas) */
void afficherPositions()
{
    for (uint8_t adresse : ADRESSES) {
        int32_t position = 0;
        if (lire32(adresse, 0x4016, position)) {
            Serial.printf("Moteur %u : position %ld pas\n", adresse, (long)position);
        }
    }
}

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

/* Vérifie les réglages de communication du variateur (faits au panneau : il refuse
   qu'on modifie C0A.05 par Modbus), coupe le moteur, puis le configure en mode PP
   absolu. Plusieurs paramètres ne s'écrivent que moteur arrêté ("At stop"). */
bool configurer(uint8_t adresse)
{
    uint16_t stockage = 1;
    uint16_t ordreMots = 1;
    if (!(lire16(adresse, 0x0A05, stockage)   // C0A.05 : sauvegarde des écritures Modbus en EEPROM
          && lire16(adresse, 0x0A06, ordreMots))) {  // C0A.06 : ordre des mots des valeurs 32 bits
        return false;
    }
    if (stockage != 0 || ordreMots != 0) {
        Serial.printf("   Réglez au panneau du variateur %u C0A.05 = 0 (lu : %u) et C0A.06 = 0 (lu : %u).\n"
                      "   Avec C0A.05 = 1, tout ce que ce programme écrit serait sauvegardé et\n"
                      "   resterait après un redémarrage du variateur.\n",
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
           && ecrire16(adresse, 0x1101, 0)    // C11.01 = 0 : consigne ABSOLUE (1 = relative)
           && ecrire16(adresse, 0x1108, VITESSE_RPM)  // C11.08 : vitesse (tr/min)
           && ecrire16(adresse, 0x0414, 19);  // C04.14 = 19 : DI6 = départ du mouvement
}

/* Configure tous les variateurs, puis met tous les moteurs sous couple et attend
   qu'ils soient en marche. En cas d'échec, tous les moteurs sont coupés. */
bool mettreSousCouple()
{
    Serial.println("Configuration des variateurs");
    for (uint8_t adresse : ADRESSES) {
        if (!configurer(adresse)) {
            return false;                     // aucun moteur n'est encore sous couple
        }
    }
    Serial.println("Mise sous couple (S-ON) des moteurs");
    bool ok = true;
    for (size_t i = 0; i < NB_MOTEURS && ok; i++) {
        ok = ecrire16(ADRESSES[i], 0x0411, 1);  // C04.11 = 1 : DI5 non câblée devient active
    }
    uint16_t etat[NB_MOTEURS] = {};
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

/* Envoie en même temps le moteur i au pas cibles[i] (positions absolues).
   Les moteurs restent sous couple à la fin. */
void bougerEnsemble(const int32_t cibles[NB_MOTEURS])
{
    /* 1. Moteurs sous couple : la première fois, configuration puis S-ON ; ensuite,
          vérification qu'aucun servo n'est passé en défaut depuis le dernier mouvement */
    if (!sousCouple) {
        if (!mettreSousCouple()) {
            return;
        }
    } else {
        uint16_t etat[NB_MOTEURS];
        if (!tousEnMarche(etat)) {
            afficherEtats(etat);
            couper();
            return;
        }
    }

    /* 2. Positions de départ, distances et durée du plus long mouvement */
    int32_t pasParTour[NB_MOTEURS];
    int32_t depart[NB_MOTEURS];
    uint32_t dureeMax = 0;
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        if (!(lire32(ADRESSES[i], 0x0002, pasParTour[i])   // C00.02 : pas par tour moteur
              && lire32(ADRESSES[i], 0x4016, depart[i]))) { // U40.16 : position actuelle (pas)
            return;
        }
        if (pasParTour[i] <= 0) {
            Serial.printf("   Variateur %u : C00.02 = 0, il utilise le réducteur électronique, non géré ici.\n",
                          ADRESSES[i]);
            return;
        }
        const int32_t distance = cibles[i] - depart[i];
        Serial.printf("   Moteur %u : position %ld pas, cible %ld pas, soit %+ld pas (%+.1f°)\n",
                      ADRESSES[i], (long)depart[i], (long)cibles[i], (long)distance,
                      distance * 360.0 / pasParTour[i]);
        /* durée du mouvement : tours / (tr/min), en ms */
        const uint32_t duree = (uint32_t)(abs(distance) * 60000.0 / pasParTour[i] / VITESSE_RPM);
        if (duree > dureeMax) {
            dureeMax = duree;
        }
    }
    if (dureeMax == 0) {
        Serial.println("   Tous les moteurs sont déjà à leur cible.");
        return;
    }

    /* 3. Cibles envoyées à tous les variateurs, puis départs l'un juste après l'autre */
    bool ok = true;
    for (size_t i = 0; i < NB_MOTEURS && ok; i++) {
        ok = ecrire32(ADRESSES[i], 0x1106, cibles[i]);  // C11.06 : position à atteindre (pas)
    }
    if (!ok) {
        return;
    }
    Serial.printf("Déplacement simultané à %u tr/min\n", VITESSE_RPM);
    uint32_t premierDepart = 0;
    uint32_t dernierDepart = 0;
    for (size_t i = 0; i < NB_MOTEURS && ok; i++) {
        dernierDepart = micros();
        if (i == 0) {
            premierDepart = dernierDepart;
        }
        ok = ecrire16(ADRESSES[i], 0x0415, 1);  // C04.15 = 1 : DI6 devient active -> départ
    }
    delay(10);
    for (uint8_t adresse : ADRESSES) {
        ecrire16(adresse, 0x0415, 0);         // DI6 au repos, prête pour le départ suivant
    }
    if (!ok) {
        return;
    }
    Serial.printf("   Écart entre le premier et le dernier départ : %.1f ms\n",
                  (dernierDepart - premierDepart) / 1000.0);
    delay(dureeMax + 500);                    // durée du plus long mouvement + marge
    for (size_t i = 0; i < NB_MOTEURS; i++) {
        int32_t arrivee = 0;
        if (lire32(ADRESSES[i], 0x4016, arrivee)) {
            Serial.printf("   Moteur %u arrivé : position %ld pas (écart à la cible : %+ld)\n",
                          ADRESSES[i], (long)arrivee, (long)(arrivee - cibles[i]));
        }
    }
}

void afficherInvite()
{
    Serial.printf("\nMoteurs %s. Tapez le pas visé de chaque moteur + Entrée (ex. 5108 3000),\n"
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
    Serial.printf("\n=== Déplacement absolu simultané des moteurs (%lu bauds, %u tr/min) ===\n",
                  (unsigned long)DEBIT, VITESSE_RPM);
    couper();                                 // état connu au démarrage : moteurs hors couple
    afficherPositions();
    afficherInvite();
}

void loop()
{
    static char ligne[32];
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
        /* "5108 3000" : un pas visé par moteur, dans l'ordre de ADRESSES */
        int32_t cibles[NB_MOTEURS];
        char *p = ligne;
        bool valide = true;
        for (size_t i = 0; i < NB_MOTEURS && valide; i++) {
            char *fin = nullptr;
            cibles[i] = (int32_t)strtol(p, &fin, 10);
            valide = (fin != p);              // au moins un chiffre lu
            p = fin;
        }
        while (*p == ' ') {
            p++;
        }
        if (!valide || *p != '\0') {
            Serial.printf("Tapez %u nombres entiers (un pas par moteur, ex. 5108 3000) ou off.\n",
                          (unsigned)NB_MOTEURS);
        } else {
            bougerEnsemble(cibles);
            afficherInvite();
        }
    }
}
