/*
 * test_moteur_rs485.cpp
 *
 * Mouvement simple d'un moteur depuis le Teensy, en Modbus RTU sur RS485 : après un
 * appui sur Entrée dans le moniteur série, le moteur tourne de ANGLE_DEG, revient à
 * son point de départ, puis est remis hors couple.
 *
 * Principe (rien n'est câblé sur les entrées du variateur) :
 *   - mode position avec consigne interne en mode PP (C11.00 = 3) : chaque front sur
 *     l'entrée DI6 fait parcourir au moteur la distance C11.06 à la vitesse C11.08 ;
 *   - DI5 (mise sous couple, S-ON) et DI6 (départ du mouvement) ne sont pas câblées :
 *     on les active par Modbus en inversant leur logique (0 = NO, 1 = NC) avec C04.11
 *     et C04.15. Une entrée non câblée en logique NC est vue comme active ;
 *   - C0A.05 = 0 : rien de ce qui est écrit n'est sauvegardé, le variateur retrouve
 *     ses réglages à la prochaine mise sous tension. Le variateur refuse qu'on modifie
 *     C0A.05 par Modbus (code d'erreur 4) : il se règle une fois au panneau, avec
 *     C0A.06 = 0 (valeurs 32 bits mot bas d'abord), et le programme le vérifie.
 *   Adresse Modbus d'un paramètre CGG.II (ou UGG.II) = 0xGGII, ex. C11.08 -> 0x1108.
 *
 * Câblage : le même que test_rs485_simple.cpp (module MAX485, DE et RE sur la broche 2).
 *
 * Lancement : PlatformIO > PROJECT TASKS > test_moteur > Upload and Monitor, puis
 * Entrée dans le moniteur. Entrée à nouveau pour recommencer.
 */

#include <Arduino.h>

/* ============================ RÉGLAGES ============================ */

constexpr uint32_t DEBIT = 115200;            // C0A.01 du variateur (7 = 115200)
constexpr uint8_t ADRESSE = 1;                // C0A.00 du variateur
constexpr uint8_t PIN_DE_RE = 2;              // DE et RE du MAX485, reliées ensemble
constexpr uint32_t ATTENTE_REPONSE_MS = 100;  // temps laissé au variateur pour répondre

constexpr double ANGLE_DEG = 45.0;            // amplitude du mouvement (négatif : autre sens)
constexpr uint16_t VITESSE_RPM = 5;           // vitesse du moteur (1 tr/min = 6°/s)

HardwareSerialIMXRT &rs485 = Serial2;         // RX2 = broche 7, TX2 = broche 8

/* ================================================================== */

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

/* Envoie la requête (les 2 derniers octets reçoivent le CRC) et lit la réponse de
   lenRep octets. Retourne true si elle est complète, avec un CRC valide, et si le
   variateur n'a pas refusé la requête. */
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
        Serial.printf("   ERREUR : pas de réponse (registre 0x%02X%02X)\n", req[2], req[3]);
        return false;
    }
    if (crc16(rep, lenRep - 2) != (uint16_t)(rep[lenRep - 2] | (rep[lenRep - 1] << 8))) {
        Serial.printf("   ERREUR : réponse abîmée, CRC faux (registre 0x%02X%02X)\n", req[2], req[3]);
        return false;
    }
    if (rep[1] & 0x80) {
        Serial.printf("   ERREUR : refusé par le variateur, code %u (registre 0x%02X%02X)\n",
                      rep[2], req[2], req[3]);
        return false;
    }
    return true;
}

/* Fonction 0x06 : écrit un paramètre 16 bits */
bool ecrire16(uint16_t reg, uint16_t valeur)
{
    uint8_t req[8] = {ADRESSE, 0x06, haut(reg), bas(reg), haut(valeur), bas(valeur), 0, 0};
    uint8_t rep[8];
    return transaction(req, sizeof(req), rep, sizeof(rep));
}

/* Fonction 0x10 : écrit un paramètre 32 bits (2 registres, mot bas d'abord : C0A.06 = 0) */
bool ecrire32(uint16_t reg, int32_t valeur)
{
    uint16_t motBas = (uint32_t)valeur & 0xFFFF;
    uint16_t motHaut = (uint32_t)valeur >> 16;
    uint8_t req[13] = {ADRESSE, 0x10, haut(reg), bas(reg), 0x00, 0x02, 0x04,
                       haut(motBas), bas(motBas), haut(motHaut), bas(motHaut), 0, 0};
    uint8_t rep[8];
    return transaction(req, sizeof(req), rep, sizeof(rep));
}

/* Fonction 0x03 : lit un paramètre 16 bits */
bool lire16(uint16_t reg, uint16_t &valeur)
{
    uint8_t req[8] = {ADRESSE, 0x03, haut(reg), bas(reg), 0x00, 0x01, 0, 0};
    uint8_t rep[7];
    if (!transaction(req, sizeof(req), rep, sizeof(rep))) {
        return false;
    }
    valeur = (uint16_t)((rep[3] << 8) | rep[4]);
    return true;
}

/* Fonction 0x03 : lit un paramètre 32 bits (mot bas d'abord : C0A.06 = 0) */
bool lire32(uint16_t reg, int32_t &valeur)
{
    uint8_t req[8] = {ADRESSE, 0x03, haut(reg), bas(reg), 0x00, 0x02, 0, 0};
    uint8_t rep[9];
    if (!transaction(req, sizeof(req), rep, sizeof(rep))) {
        return false;
    }
    uint32_t motBas = (uint32_t)((rep[3] << 8) | rep[4]);
    uint32_t motHaut = (uint32_t)((rep[5] << 8) | rep[6]);
    valeur = (int32_t)((motHaut << 16) | motBas);
    return true;
}

/* Déplacement relatif de "impulsions", puis attente de la fin du mouvement */
bool deplacer(int32_t impulsions)
{
    int32_t avant = 0;
    int32_t apres = 0;
    if (!(lire32(0x4016, avant)               // U40.16 : position actuelle du moteur
          && ecrire32(0x1106, impulsions)     // C11.06 : distance à parcourir
          && ecrire16(0x0415, 1))) {          // C04.15 = 1 : DI6 devient active -> départ
        return false;
    }
    delay(10);
    if (!ecrire16(0x0415, 0)) {               // DI6 au repos, prête pour le départ suivant
        return false;
    }
    delay((uint32_t)(fabs(ANGLE_DEG) / (VITESSE_RPM * 6.0) * 1000.0) + 500);  // durée + marge
    if (!lire32(0x4016, apres)) {
        return false;
    }
    Serial.printf("   position : %ld -> %ld impulsions (écart à la cible : %+ld)\n",
                  (long)avant, (long)apres, (long)(apres - avant - impulsions));
    return true;
}

void faireMouvement()
{
    /* 1. Vérification des réglages de communication (faits au panneau : le variateur
          refuse qu'on modifie C0A.05 par Modbus), moteur hors couple, puis configuration.
          Plusieurs paramètres ne s'écrivent que moteur arrêté ("At stop" dans le manuel). */
    Serial.println("1. Configuration du variateur");
    uint16_t stockage = 1;
    uint16_t ordreMots = 1;
    if (!(lire16(0x0A05, stockage)            // C0A.05 : sauvegarde des écritures Modbus en EEPROM
          && lire16(0x0A06, ordreMots))) {    // C0A.06 : ordre des mots des valeurs 32 bits
        return;
    }
    if (stockage != 0 || ordreMots != 0) {
        Serial.printf("   Réglez au panneau du variateur C0A.05 = 0 (lu : %u) et C0A.06 = 0 (lu : %u).\n"
                      "   Avec C0A.05 = 1, tout ce que ce programme écrit serait sauvegardé et\n"
                      "   resterait après un redémarrage du variateur.\n",
                      stockage, ordreMots);
        return;
    }
    if (!(ecrire16(0x0411, 0)                 // C04.11 = 0 : DI5 (S-ON) inactive -> moteur coupé
          && ecrire16(0x0415, 0))) {          // C04.15 = 0 : DI6 (départ) au repos
        return;
    }
    delay(100);                               // laisse le servo s'arrêter
    int32_t impulsionsParTour = 0;
    if (!(ecrire16(0x0000, 0)                 // C00.00 = 0 : mode position
          && ecrire16(0x0300, 1)              // C03.00 = 1 : consigne de position interne
          && ecrire16(0x1100, 3)              // C11.00 = 3 : mode PP
          && ecrire16(0x1101, 1)              // C11.01 = 1 : déplacements relatifs
          && ecrire16(0x1108, VITESSE_RPM)    // C11.08 : vitesse (tr/min)
          && ecrire16(0x0414, 19)             // C04.14 = 19 : DI6 = départ du mouvement
          && lire32(0x0002, impulsionsParTour))) {  // C00.02 : impulsions par tour moteur
        return;
    }
    if (impulsionsParTour <= 0) {
        Serial.println("   C00.02 = 0 : le variateur utilise le réducteur électronique, non géré ici.");
        return;
    }
    const int32_t impulsions = (int32_t)lround(ANGLE_DEG / 360.0 * impulsionsParTour);
    Serial.printf("   %ld impulsions par tour : %.1f° = %ld impulsions\n",
                  (long)impulsionsParTour, ANGLE_DEG, (long)impulsions);

    /* 2. Mise sous couple, puis attente de l'état "en marche" (U41.0A = 2) */
    Serial.println("2. Mise sous couple (S-ON)");
    uint16_t etat = 0;
    if (ecrire16(0x0411, 1)) {                // C04.11 = 1 : DI5 non câblée devient active
        uint32_t debut = millis();
        while (etat != 2 && millis() - debut < 1000) {
            delay(50);
            lire16(0x410A, etat);             // U41.0A : 0 pas prêt, 1 prêt, 2 en marche, 3 défaut
        }
    }

    /* 3 et 4. Aller-retour, seulement si le servo est en marche */
    if (etat == 2) {
        Serial.printf("3. Aller : %+.1f° à %u tr/min\n", ANGLE_DEG, VITESSE_RPM);
        if (deplacer(impulsions)) {
            Serial.printf("4. Retour : %+.1f°\n", -ANGLE_DEG);
            deplacer(-impulsions);
        }
    } else {
        Serial.printf("   Le servo n'est pas en marche (U41.0A = %u ; 3 = défaut, voir l'afficheur).\n",
                      etat);
    }

    /* 5. Toujours : moteur hors couple */
    Serial.println("5. Coupure (S-OFF)");
    if (!ecrire16(0x0411, 0)) {
        Serial.println("   COUPURE NON CONFIRMÉE : coupez l'alimentation du variateur !");
    }
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
    Serial.printf("\n=== Mouvement simple du moteur (variateur %u, %lu bauds) ===\n"
                  "Le moteur va tourner de %+.1f° à %u tr/min, revenir, puis être coupé.\n"
                  "Entrée pour lancer.\n",
                  ADRESSE, (unsigned long)DEBIT, ANGLE_DEG, VITESSE_RPM);
}

void loop()
{
    if (Serial.available() > 0) {
        delay(10);
        while (Serial.available() > 0) {      // vide la ligne tapée
            Serial.read();
        }
        faireMouvement();
        Serial.println("\nEntrée pour recommencer.");
    }
}
