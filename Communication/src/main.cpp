#include <Arduino.h>

constexpr unsigned long BAUD_RATE = 115200;

void setup() {
    Serial.begin(BAUD_RATE);   // Moniteur USB pour le débogage
    Serial1.begin(BAUD_RATE);  // UART matériel : RX1 broche 0, TX1 broche 1
    Serial.println("UART du Teensy 4.1 initialise.");
}

void loop() {
    while (Serial1.available() > 0) {
        const int receivedByte = Serial1.read();
        Serial.write(receivedByte);   // Affiche les octets reçus sur le moniteur USB
        int testing = receivedByte+1;  // Stocke l'octet reçu pour un traitement ultérieur si nécessaire
        Serial1.write(testing);  // Renvoie les octets au Raspberry Pi (écho de test)
    }
}