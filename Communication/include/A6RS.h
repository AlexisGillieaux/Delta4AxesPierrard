#pragma once

#include <Arduino.h>

class A6RS {
public:
    A6RS(HardwareSerial &port, uint8_t brocheDirection);
    void begin(uint32_t debit = 115200);

    // Position absolue en pas du variateur, vitesse en tr/min, rampes en ms.
    // true confirme le déclenchement, dont le mouvement se poursuit sur le variateur.
    bool deplacerAbsolu(uint8_t adresse, int32_t position, uint16_t vitesse,
                        uint32_t accelerationMs, uint32_t decelerationMs);
    // Sélectionne le couple numérique à 0 %, avec S-ON désactivé.
    bool modeCouple(uint8_t adresse);
    bool desactiver(uint8_t adresse);

private:
    HardwareSerial &port;
    uint8_t brocheDirection;
    uint32_t silenceUs = 1750;
    bool initialise = false;

    bool verifierVariateur(uint8_t adresse, bool &motBasPremier);
    bool attendreEtat(uint8_t adresse, bool enMarche);
    bool lire16(uint8_t adresse, uint16_t registre, uint16_t &valeur);
    bool ecrire16(uint8_t adresse, uint16_t registre, uint16_t valeur);
    bool ecrire32(uint8_t adresse, uint16_t registre, uint32_t valeur, bool motBasPremier);
    bool transaction(uint8_t *requete, size_t taille, uint8_t *reponse, size_t attendus);
};
