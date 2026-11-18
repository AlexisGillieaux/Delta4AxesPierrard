/*
 * test_rs485_simple.cpp
 *
 * Test le plus simple possible du bus RS485 : le Teensy lit la température de
 * chaque variateur A6-RS du bus (adresses listées dans ADRESSES) et l'affiche,
 * une fois par seconde. Aucune écriture dans les variateurs.
 *
 *   Trame envoyée au variateur 1 : 01 03 40 30 00 02 D1 C4
 *   Trame envoyée au variateur 2 : 02 03 40 30 00 02 D1 F7
 *     01/02 = adresse du variateur (C0A.00), seul ce variateur répond
 *     03    = fonction Modbus "lecture"
 *     40 30 = premier registre lu : U40.30, température du module de puissance (IGBT)
 *     00 02 = 2 registres : U40.30 puis U40.31 (température du moteur)
 *     xx xx = CRC (calculé par le programme, il dépend de l'adresse)
 *
 *   Réponse attendue (9 octets) : [adresse] 03 04 [IGBT : 2 octets] [moteur : 2 octets] CRC CRC
 *     Chaque température est un entier signé en 0,1 °C (ex. 01 2C = 300 = 30,0 °C).
 *
 * Module : "TTL to RS485" bleu à MAX485, broches RO, RE, DE, DI / VCC, B, A, GND.
 * Le sens de la ligne est commandé par le Teensy (broche 2 reliée à DE et RE) :
 *   broche 2 = 1 : DE = 1, émetteur actif ; RE = 1, récepteur coupé  -> le Teensy parle
 *   broche 2 = 0 : DE = 0, émetteur coupé ; RE = 0, récepteur actif  -> le variateur parle
 *
 * Câblage :
 *   Teensy broche 8 (TX2) -> DI
 *   Teensy broche 7 (RX2) <- RO
 *   Teensy broche 2       -> DE et RE (reliées ensemble)
 *   Teensy 3,3 V          -> VCC,   Teensy GND -> GND
 *   A -> RJ45 broche 4 (bleu)   B -> broche 5 (blanc-bleu)   GND -> broche 8 (marron)
 *      branché dans CN3 du variateur 1
 *   Variateur 1 CN4 -> variateur 2 CN3 : câble Ethernet droit (CN3 et CN4 sont reliés
 *      en parallèle dans le variateur, les deux variateurs sont donc sur les mêmes fils)
 *   Pas de résistance de 120 ohms en plus : le module en a déjà une (R7).
 *
 * Réglages au panneau de chaque variateur (puis éteindre et rallumer le variateur) :
 *   C0A.00 = son adresse (1, 2, ...), différente pour chaque variateur
 *   C0A.01 = 7 (115200 bauds)   C0A.02 = 0
 *
 * Lancement : PlatformIO > PROJECT TASKS > test_rs485 > Upload and Monitor.
 */

#include <Arduino.h>

/* ============================ RÉGLAGES ============================ */

const uint8_t ADRESSES[] = {1, 2};            // adresses des variateurs à interroger (C0A.00)
constexpr uint32_t DEBIT = 115200;            // doit valoir C0A.01 des variateurs (7 = 115200, 3 = 9600)
constexpr uint32_t ATTENTE_REPONSE_MS = 100;  // temps laissé au variateur pour répondre
constexpr uint32_t PERIODE_MS = 1000;         // pause entre deux tours de lecture
constexpr uint8_t PIN_DE_RE = 2;              // DE et RE du MAX485, reliées ensemble

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

void lireTemperature(uint8_t adresse)
{
    Serial.printf("--- Variateur %u ---\n", (unsigned)adresse);

    /* 1. Envoi de la trame de lecture, avec l'adresse de ce variateur */
    uint8_t trame[8] = {adresse, 0x03, 0x40, 0x30, 0x00, 0x02, 0, 0};
    uint16_t crc = crc16(trame, 6);
    trame[6] = (uint8_t)(crc & 0xFF);         // CRC : octet de poids faible en premier
    trame[7] = (uint8_t)(crc >> 8);

    rs485.clear();                            // oublie ce qui traînait dans le tampon
    digitalWrite(PIN_DE_RE, HIGH);            // le Teensy prend la ligne
    rs485.write(trame, sizeof(trame));
    rs485.flush();                            // attend que le bit de stop du dernier octet soit parti
    digitalWrite(PIN_DE_RE, LOW);             // libère la ligne pour la réponse du variateur
    Serial.print("Envoyé :");
    for (uint8_t octet : trame) {
        Serial.printf(" %02X", octet);
    }
    Serial.println();

    /* 2. Réception de tout ce qui revient pendant ATTENTE_REPONSE_MS */
    uint8_t recu[256];
    size_t n = 0;
    size_t total = 0;
    uint32_t debut = millis();
    while (millis() - debut < ATTENTE_REPONSE_MS) {
        if (rs485.available() > 0) {
            uint8_t octet = (uint8_t)rs485.read();
            if (n < sizeof(recu)) {
                recu[n++] = octet;
            }
            total++;
        }
    }
    Serial.printf("Reçu   : %u octet(s)", (unsigned)total);
    for (size_t i = 0; i < n && i < 32; i++) {
        Serial.printf(" %02X", recu[i]);
    }
    Serial.println(n > 32 ? " ..." : "");

    /* 3. Recherche de la réponse attendue ([adresse] 03 04 + 4 octets + CRC valide),
          éventuellement précédée de parasites */
    for (size_t i = 0; i + 9 <= n; i++) {
        const uint8_t *r = recu + i;
        uint16_t crcRecu = (uint16_t)(r[7] | (r[8] << 8));
        if (r[0] == adresse && r[1] == 0x03 && r[2] == 0x04 && crc16(r, 7) == crcRecu) {
            int16_t igbt = (int16_t)((r[3] << 8) | r[4]);
            int16_t moteur = (int16_t)((r[5] << 8) | r[6]);
            if (i > 0) {
                Serial.printf("(réponse trouvée après %u octet(s) parasite(s))\n", (unsigned)i);
            }
            Serial.printf("  Module de puissance (U40.30) : %.1f °C\n", igbt / 10.0);
            Serial.printf("  Moteur (U40.31)              : %.1f °C\n", moteur / 10.0);
            return;
        }
    }

    Serial.println("Pas de réponse valide.");
    if (total == 0) {
        Serial.printf("  Rien n'est revenu : au panneau de ce variateur, vérifiez C0A.00 = %u,\n"
                      "  C0A.01 = 7 (115200) et C0A.02 = 0, puis éteint/rallumé. Si c'est le\n"
                      "  variateur 2 : vérifiez le câble entre CN4 du variateur 1 et CN3 du variateur 2.\n",
                      (unsigned)adresse);
    } else {
        Serial.println("  Seulement des octets parasites : deux variateurs avec la même adresse\n"
                       "  (C0A.00) qui répondent en même temps, A et B inversés, masse non reliée,\n"
                       "  ou débit différent (C0A.01).");
    }
}

void setup()
{
    /* En premier : sans ça, la résistance de tirage du module met DE à 1 et
       l'émetteur du MAX485 occupe la ligne tant que le moniteur n'est pas ouvert. */
    pinMode(PIN_DE_RE, OUTPUT);
    digitalWrite(PIN_DE_RE, LOW);             // réception

    Serial.begin(115200);                     // USB vers le PC
    while (!Serial) {                         // attend l'ouverture du moniteur série
    }
    delay(500);
    rs485.begin(DEBIT, SERIAL_8N1);           // 8 bits, sans parité, 1 bit de stop (C0A.02 = 0)
    Serial.printf("\n=== Lecture des températures des variateurs (Serial2, %lu bauds, DE/RE broche %u) ===\n",
                  (unsigned long)DEBIT, (unsigned)PIN_DE_RE);
}

void loop()
{
    for (uint8_t adresse : ADRESSES) {
        lireTemperature(adresse);
    }
    Serial.println();
    delay(PERIODE_MS);
}
