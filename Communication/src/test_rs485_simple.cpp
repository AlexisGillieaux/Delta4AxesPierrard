/*
 * test_rs485_simple.cpp
 *
 * Test le plus simple possible de la liaison RS485 : le Teensy lit la température
 * du variateur A6-RS (trame de la FAQ StepperOnline) et l'affiche, une fois par
 * seconde. Aucune écriture dans le variateur.
 *
 *   Trame envoyée : 01 03 40 30 00 02 D1 C4
 *     01    = adresse du variateur (C0A.00)
 *     03    = fonction Modbus "lecture"
 *     40 30 = premier registre lu : U40.30, température du module de puissance (IGBT)
 *     00 02 = 2 registres : U40.30 puis U40.31 (température du moteur)
 *     D1 C4 = CRC
 *
 *   Réponse attendue (9 octets) : 01 03 04 [IGBT : 2 octets] [moteur : 2 octets] CRC CRC
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
 *   Pas de résistance de 120 ohms en plus : le module en a déjà une (R7).
 *
 * Lancement : PlatformIO > PROJECT TASKS > test_rs485 > Upload and Monitor.
 */

#include <Arduino.h>

/* ============================ RÉGLAGES ============================ */

constexpr uint32_t DEBIT = 115200;            // doit valoir C0A.01 du variateur (7 = 115200, 3 = 9600)
constexpr uint32_t ATTENTE_REPONSE_MS = 100;  // temps laissé au variateur pour répondre
constexpr uint32_t PERIODE_MS = 1000;         // pause entre deux lectures
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

void lireTemperature()
{
    /* 1. Envoi de la trame de la FAQ */
    const uint8_t trame[8] = {0x01, 0x03, 0x40, 0x30, 0x00, 0x02, 0xD1, 0xC4};
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

    /* 3. Recherche de la réponse attendue (01 03 04 + 4 octets + CRC valide),
          éventuellement précédée de parasites */
    for (size_t i = 0; i + 9 <= n; i++) {
        const uint8_t *r = recu + i;
        uint16_t crcRecu = (uint16_t)(r[7] | (r[8] << 8));
        if (r[0] == 0x01 && r[1] == 0x03 && r[2] == 0x04 && crc16(r, 7) == crcRecu) {
            int16_t igbt = (int16_t)((r[3] << 8) | r[4]);
            int16_t moteur = (int16_t)((r[5] << 8) | r[6]);
            Serial.printf("RÉPONSE CORRECTE (après %u octet(s) parasite(s))\n", (unsigned)i);
            Serial.printf("  Température du module de puissance (U40.30) : %.1f °C\n", igbt / 10.0);
            Serial.printf("  Température du moteur (U40.31)              : %.1f °C\n\n", moteur / 10.0);
            return;
        }
    }

    Serial.println("Pas de réponse valide.");
    if (total == 0) {
        Serial.println("  Rien n'est revenu : vérifiez DE et RE reliés à la broche 2, A/B, la masse\n"
                       "  commune (broche 8 du RJ45), puis au panneau du variateur : C0A.00 = 1,\n"
                       "  C0A.01 = 7 si DEBIT vaut 115200 (3 pour 9600), C0A.02 = 0.\n");
    } else {
        Serial.println("  Seulement des octets parasites : A et B inversés, masse non reliée, ou débit\n"
                       "  différent de celui du variateur (C0A.01).\n");
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
    Serial.printf("\n=== Lecture de la température du variateur (Serial2, %lu bauds, DE/RE broche %u) ===\n",
                  (unsigned long)DEBIT, (unsigned)PIN_DE_RE);
}

void loop()
{
    lireTemperature();
    delay(PERIODE_MS);
}
