#include <A6RS.h>

namespace {
constexpr uint16_t MODE = 0x0000;
constexpr uint16_t SOURCE_POSITION = 0x0300;
constexpr uint16_t SOURCE_COUPLE = 0x0340;
constexpr uint16_t CONSIGNE_COUPLE = 0x0341;
constexpr uint16_t FONCTION_DI5 = 0x0410;
constexpr uint16_t SERVO_ON = 0x0411;
constexpr uint16_t FONCTION_DI6 = 0x0414;
constexpr uint16_t DECLENCHEMENT = 0x0415;
constexpr uint16_t ETAT_SERVO = 0x410A;
constexpr uint16_t POSITION = 0x1106;
constexpr uint16_t VITESSE = 0x1108;
constexpr uint16_t ACCELERATION = 0x110A;
constexpr uint16_t DECELERATION = 0x110C;

uint8_t haut(uint16_t mot) { return mot >> 8; }
uint8_t bas(uint16_t mot) { return mot & 0xFF; }

uint16_t crc16(const uint8_t *octets, size_t taille)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < taille; ++i) {
        crc ^= octets[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
        }
    }
    return crc;
}
}

A6RS::A6RS(HardwareSerial &port, uint8_t brocheDirection)
    : port(port), brocheDirection(brocheDirection)
{
}

void A6RS::begin(uint32_t debit)
{
    pinMode(brocheDirection, OUTPUT);
    digitalWrite(brocheDirection, LOW);
    initialise = debit != 0;
    if (initialise) {
        silenceUs = debit > 19200 ? 1750 : (38500000 / debit + 1);
        port.begin(debit, SERIAL_8N1);
    }
}

bool A6RS::transaction(uint8_t *requete, size_t taille, uint8_t *reponse, size_t attendus)
{
    if (!initialise || requete[0] == 0 || requete[0] > 247) {
        return false;
    }

    const uint16_t crc = crc16(requete, taille - 2);
    requete[taille - 2] = bas(crc);
    requete[taille - 1] = haut(crc);

    delayMicroseconds(silenceUs);
    while (port.available() > 0) {
        port.read();
    }
    digitalWrite(brocheDirection, HIGH);
    port.write(requete, taille);
    port.flush();
    digitalWrite(brocheDirection, LOW);

    size_t recus = 0;
    const uint32_t debut = millis();
    while (recus < attendus && millis() - debut < 100) {
        const int octet = port.read();
        if (octet >= 0) {
            reponse[recus++] = static_cast<uint8_t>(octet);
            if (recus == 2 && reponse[1] == (requete[1] | 0x80)) {
                attendus = 5;
            }
        }
    }
    return recus == attendus && reponse[0] == requete[0] && reponse[1] == requete[1]
        && crc16(reponse, recus - 2) == (reponse[recus - 2] | reponse[recus - 1] << 8);
}

bool A6RS::lire16(uint8_t adresse, uint16_t registre, uint16_t &valeur)
{
    uint8_t requete[] = {adresse, 0x03, haut(registre), bas(registre), 0, 1, 0, 0};
    uint8_t reponse[7];
    if (!transaction(requete, sizeof(requete), reponse, sizeof(reponse)) || reponse[2] != 2) {
        return false;
    }
    valeur = (reponse[3] << 8) | reponse[4];
    return true;
}

bool A6RS::ecrire16(uint8_t adresse, uint16_t registre, uint16_t valeur)
{
    uint8_t requete[] = {adresse, 0x06, haut(registre), bas(registre), haut(valeur), bas(valeur), 0, 0};
    uint8_t reponse[8];
    return transaction(requete, sizeof(requete), reponse, sizeof(reponse))
        && memcmp(requete + 2, reponse + 2, 4) == 0;
}

bool A6RS::ecrire32(uint8_t adresse, uint16_t registre, uint32_t valeur, bool motBasPremier)
{
    const uint16_t motBas = valeur & 0xFFFF;
    const uint16_t motHaut = valeur >> 16;
    const uint16_t premier = motBasPremier ? motBas : motHaut;
    const uint16_t second = motBasPremier ? motHaut : motBas;
    uint8_t requete[] = {adresse, 0x10, haut(registre), bas(registre), 0, 2, 4,
                         haut(premier), bas(premier), haut(second), bas(second), 0, 0};
    uint8_t reponse[8];
    return transaction(requete, sizeof(requete), reponse, sizeof(reponse))
        && memcmp(requete + 2, reponse + 2, 4) == 0;
}

bool A6RS::verifierVariateur(uint8_t adresse, bool &motBasPremier)
{
    uint16_t stockage, ordre, fonction;
    if (!lire16(adresse, 0x0A05, stockage) || stockage != 0
        || !lire16(adresse, 0x0A06, ordre) || ordre > 1
        || !lire16(adresse, FONCTION_DI5, fonction) || fonction != 1) {
        return false;
    }
    motBasPremier = ordre == 0;
    return true;
}

bool A6RS::attendreEtat(uint8_t adresse, bool enMarche)
{
    const uint32_t debut = millis();
    do {
        uint16_t etat;
        if (!lire16(adresse, ETAT_SERVO, etat) || etat > 2) {
            return false;
        }
        if (enMarche ? etat == 2 : etat == 1) {
            return true;
        }
        delay(10);
    } while (millis() - debut < 1000);
    return false;
}

bool A6RS::deplacerAbsolu(uint8_t adresse, int32_t position, uint16_t vitesse,
                           uint32_t accelerationMs, uint32_t decelerationMs)
{
    bool motBasPremier;
    if (vitesse == 0 || vitesse > 8000 || accelerationMs > 65535 || decelerationMs > 65535
        || !verifierVariateur(adresse, motBasPremier)) {
        return false;
    }

    const uint16_t configuration[][2] = {
        {MODE, 0}, {SOURCE_POSITION, 1}, {0x1100, 3}, {0x1101, 0},
        {0x1102, 0}, {FONCTION_DI6, 19}
    };
    bool aConfigurer = false;
    for (const auto &parametre : configuration) {
        uint16_t valeur;
        if (!lire16(adresse, parametre[0], valeur)) {
            return false;
        }
        aConfigurer |= valeur != parametre[1];
    }
    if (aConfigurer) {
        if (!desactiver(adresse) || !attendreEtat(adresse, false)) {
            return false;
        }
        for (const auto &parametre : configuration) {
            if (!ecrire16(adresse, parametre[0], parametre[1])) {
                return false;
            }
        }
    }

    if (!ecrire16(adresse, DECLENCHEMENT, 0)
        || !ecrire32(adresse, POSITION, static_cast<uint32_t>(position), motBasPremier)
        || !ecrire16(adresse, VITESSE, vitesse)
        || !ecrire32(adresse, ACCELERATION, accelerationMs, motBasPremier)
        || !ecrire32(adresse, DECELERATION, decelerationMs, motBasPremier)) {
        return false;
    }

    if (ecrire16(adresse, SERVO_ON, 1) && attendreEtat(adresse, true)
        && ecrire16(adresse, DECLENCHEMENT, 1)) {
        delay(10);
        if (ecrire16(adresse, DECLENCHEMENT, 0)) {
            return true;
        }
    }
    desactiver(adresse);
    return false;
}

bool A6RS::modeCouple(uint8_t adresse)
{
    bool motBasPremier;
    return verifierVariateur(adresse, motBasPremier)
        && desactiver(adresse) && attendreEtat(adresse, false)
        && ecrire16(adresse, SOURCE_COUPLE, 0)
        && ecrire16(adresse, CONSIGNE_COUPLE, 0)
        && ecrire16(adresse, MODE, 2);
}

bool A6RS::desactiver(uint8_t adresse)
{
    return ecrire16(adresse, SERVO_ON, 0);
}
