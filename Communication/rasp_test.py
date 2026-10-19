import serial
import time
import random

# Configuration du port série du Raspberry Pi
# /dev/serial0 pointe automatiquement vers le port UART matériel principal
ser = serial.Serial('/dev/serial0', baudrate=115200, timeout=1)

time.sleep(2) # Attendre que la connexion s'initialise

# #ser.write(f"Ecrivez un nombre à envoyer au Teensy : ".encode('utf-8'))
# votre_nombre=input()  # Lire le nombre depuis l'entrée standard
# try:
#     print("Envoi du nombre converti en texte avec un saut de ligne (\\n)")
#     ser.write(f"{votre_nombre}\n".encode('utf-8'))
#     print(f"Nombre {votre_nombre} envoyé avec succès !")
# except serial.SerialException as e:
#     print(f"Erreur série : {e}")
# finally:
#     ser.close()

def matrice_creation():
    # Création d'une matrice de points (X, Y, Z, T)
    matrice = [[0 for _ in range(255)] for _ in range(255)]
    for i in range(255):
      for j in range(255):
        nb_random = random.randint(0, 255)
        matrice[i][j] = nb_random
        if j == 254:
            j = 0
            if i == 254:
             return matrice
      

    
 
#def test_RXTX():
    # test d'un bit de donné
#include <Arduino.h>

# constexpr unsigned long BAUD_RATE = 115200;
# int flag = 0;
# int i = 0, j = 0;
# int matrice[5][3] = {0};  // Déclaration de la matrice globale
# void setup() {
#     Serial.begin(BAUD_RATE);   // Moniteur USB pour le débogage
#     Serial1.begin(BAUD_RATE);  // UART matériel : RX1 broche 0, TX1 broche 1
#     Serial.println("UART du Teensy 4.1 initialise.");
    
# }

# void loop() {
#     if (Serial1.available() > 0) {
#         String message = Serial1.readStringUntil('\n');
#         int message2;
#         message.trim();  // Retire notamment le '\r' éventuel
#     for (int i = 0; i < 5; i++) {
#         int pos = message.indexOf(' ');
#         if (pos == -1) {
#             matrice[0][i] = message.toInt();
#             break;
#         }

#       matrice[0][i] = message.substring(0, pos).toInt();
#       message = message.substring(pos + 1);
#       message.trim();
#     }

#     Serial.println("Matrice recue :");
#     for (int i = 0; i < 5; i++) {
#       Serial.print(matrice[0][i]);
#       Serial.print(" ");
#     }
#     Serial.println();

#     //     message2 = message.toInt();  // Stocke le message dans la variable globale
#     //     Serial.print("Message reçu : ");
#     //     Serial.println(message);
#     //     Serial.print("Message converti en entier : ");
#     //     Serial.println(message2+1);  // Affiche le message converti en entier
        
#     }
# }

# void menu()
# {
#     //menu
#     // Si je recois COORD
#     //SI je fais

# }