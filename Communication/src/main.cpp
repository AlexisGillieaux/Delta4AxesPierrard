#include <Arduino.h>
#include <A6RS.h>
#include <ctype.h>
#include <errno.h>
#include <stdlib.h>

constexpr unsigned long BAUD_RATE = 115200;
A6RS busMoteurs(Serial2, 2);  // DE/RE du MAX485 sur la broche 2
int flag = 0;
int i = 0, j = 0;
int matrice[5][9999] = {0};  // Déclaration de la matrice globale
int controle_driver=999;
int ecrire_raspberry_flag=0;
int free_drive_flag=0;
String message_raspberry;
String message_moteur1;
String message_moteur2;
String message_moteur3;
String message_moteurRotation;
String message_moteurPince;
void menu();
void ecrire_matrice();
void driver();
void ecrireToRaspberry();
void free_drive();
String message;
void setup() 
{
    Serial.begin(BAUD_RATE);   // Moniteur USB pour le débogage
    Serial1.begin(BAUD_RATE);  // UART matériel : RX1 broche 0, TX1 broche 1
    busMoteurs.begin(BAUD_RATE);
    Serial3.begin(BAUD_RATE);  // UART matériel : RX3 broche 15, TX3 broche 14
    Serial4.begin(BAUD_RATE);  // UART matériel : RX4 broche 16, TX4 broche 17
    Serial5.begin(BAUD_RATE);  // UART matériel : RX5 broche 21, TX5 broche 20
    Serial.println("UART du Teensy 4.1 initialise.");
    
}

bool lireConsigneMoteur(const char *ligne, long (&valeurs)[5])
{
    for (long &valeur : valeurs)
    {
        char *fin;
        errno = 0;
        valeur = strtol(ligne, &fin, 10);
        if (fin == ligne || errno == ERANGE
            || (*fin && !isspace(static_cast<unsigned char>(*fin))))
        {
            return false;
        }
        ligne = fin;
    }
    while (isspace(static_cast<unsigned char>(*ligne)))
    {
        ++ligne;
    }
    return *ligne == '\0' && valeurs[0] >= 1 && valeurs[0] <= 247
        && valeurs[1] >= INT32_MIN && valeurs[1] <= INT32_MAX
        && valeurs[2] >= 1 && valeurs[2] <= 8000
        && valeurs[3] >= 0 && valeurs[3] <= 65535
        && valeurs[4] >= 0 && valeurs[4] <= 65535;
}

void loop()
{
    menu();
    driver();
}

void menu()
{
    static constexpr char commandeMoteur[] = "DEPLACER_ABSOLU ";
    static char ligne[80];
    static size_t longueur = 0;
    static bool ligneInvalide = false;

    while (Serial1.available() > 0)
    {
        const int octet = Serial1.read();
        if (octet != '\n')
        {
            if (octet > 0 && longueur < sizeof(ligne) - 1)
            {
                ligne[longueur++] = static_cast<char>(octet);
            }
            else
            {
                ligneInvalide = true;
            }
            continue;
        }

        if (longueur > 0 && ligne[longueur - 1] == '\r')
        {
            --longueur;
        }
        ligne[longueur] = '\0';
        const bool invalide = ligneInvalide;
        longueur = 0;
        ligneInvalide = false;

        if (invalide)
        {
            Serial1.println("ERR TRAME");
        }
        else if (strcmp(ligne, "START_MATRIX") == 0)
        {
            ecrire_matrice();
        }
        else if (strcmp(ligne, "FREE_DRIVE") == 0)
        {
            free_drive_flag = 1;
        }
        else if (strncmp(ligne, commandeMoteur, sizeof(commandeMoteur) - 1) == 0)
        {
            long valeurs[5];
            if (!lireConsigneMoteur(ligne + sizeof(commandeMoteur) - 1, valeurs))
            {
                Serial1.println("ERR PARAMETRES");
            }
            else
            {
                const bool declenche = busMoteurs.deplacerAbsolu(
                    static_cast<uint8_t>(valeurs[0]), static_cast<int32_t>(valeurs[1]),
                    static_cast<uint16_t>(valeurs[2]), static_cast<uint32_t>(valeurs[3]),
                    static_cast<uint32_t>(valeurs[4]));
                Serial1.println(declenche ? "OK DECLENCHE" : "ERR VARIATEUR");
            }
        }
        else if (*ligne)
        {
            Serial.println(ligne);
        }
        break;
    }
    if (ecrire_raspberry_flag == 1)
    {
        ecrireToRaspberry();
    }
}

void driver()
{
    if(free_drive_flag==1)
    {
        free_drive();
    }
}

void ecrire_matrice()
{
    //Serial.println("Veuillez entrer les valeurs de la matrice (5 valeurs séparées par des espaces) :");
    for (int chill=0;message =='END_MATRIX';chill++)
    {
        String message = Serial1.readStringUntil('\n');
        message.trim();
        for (int i = 0; message == 'END_MATRIX'; i++) 
        {
            int pos = message.indexOf(' ');
            if (pos == -1) 
            {
                matrice[j][i] = message.toInt();
                break;
            }
            j++;

            matrice[j][i] = message.substring(0, pos).toInt();
            message = message.substring(pos + 1);
            message.trim();
        }      
    }
    Serial.println("Matrice recue :");
    for (int ini=0; i ==0; i--) 
    {
      Serial.print(matrice[j][i]);
      Serial.print(" ");
      flag=1;
    }
    // Serial1.write("MATRIX_DONE\n");
}
void ecrireToRaspberry()
{
    Serial1.println(message_raspberry);
}














// if (Serial1.available() > 0) {
    //     String message = Serial1.readStringUntil('\n');
    //     int message2;
    //     message.trim();  // Retire notamment le '\r' éventuel
    // for (int i = 0; i < 5; i++) {
    //     int pos = message.indexOf(' ');
    //     if (pos == -1) {
    //         matrice[0][i] = message.toInt();
    //         break;
    //     }

    //   matrice[0][i] = message.substring(0, pos).toInt();
    //   message = message.substring(pos + 1);
    //   message.trim();
    // }

    // Serial.println("Matrice recue :");
    // for (int i = 0; i < 5; i++) {
    //   Serial.print(matrice[0][i]);
    //   Serial.print(" ");
    //   flag=1;
    // }
    // Serial.println();

    //     message2 = message.toInt();  // Stocke le message dans la variable globale
    //     Serial.print("Message reçu : ");
    //     Serial.println(message);
    //     Serial.print("Message converti en entier : ");
    //     Serial.println(message2+1);  // Affiche le message converti en entier
        
    // if(Serial.available() > 0 && flag != 0) 
    // {
    //  Serial1.write("MATRIX_DONE\n");
    //  flag = 0;
    // }
