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

bool lireEntier(const char *ligne, long &valeur)
{
    char *fin;
    errno = 0;
    valeur = strtol(ligne, &fin, 10);
    if (fin == ligne || errno == ERANGE)
    {
        return false;
    }
    while (isspace(static_cast<unsigned char>(*fin)))
    {
        ++fin;
    }
    return *fin == '\0';
}

void loop() 
{
    menu();
    driver();

    static constexpr struct {
        const char *invite;
        long minimum;
        long maximum;
    } champs[] = {
        {"Numero du moteur (adresse Modbus 1 a 247)", 1, 247},
        {"Position absolue (en pas du variateur)", INT32_MIN, INT32_MAX},
        {"Vitesse (1 a 8000 tr/min)", 1, 8000},
        {"Acceleration (0 a 65535 ms)", 0, 65535},
        {"Deceleration (0 a 65535 ms)", 0, 65535},
    };
    static long valeurs[sizeof(champs) / sizeof(champs[0])];
    static size_t champ = 0;
    static char ligne[80];
    static size_t longueur = 0;
    static bool ligneInvalide = false;
    static bool afficherInvite = true;

    if (!Serial)
    {
        champ = 0;
        longueur = 0;
        ligneInvalide = false;
        afficherInvite = true;
        return;
    }
    if (afficherInvite)
    {
        Serial.println(champs[champ].invite);
        afficherInvite = false;
    }

    while (Serial.available() > 0)
    {
        const int caractere = Serial.read();
        if (caractere != '\n' && caractere != '\r')
        {
            if (caractere > 0 && longueur < sizeof(ligne) - 1)
            {
                ligne[longueur++] = static_cast<char>(caractere);
            }
            else
            {
                ligneInvalide = true;
            }
            continue;
        }
        if (longueur == 0 && !ligneInvalide)
        {
            continue;
        }

        ligne[longueur] = '\0';
        long valeur;
        if (ligneInvalide || !lireEntier(ligne, valeur)
            || valeur < champs[champ].minimum || valeur > champs[champ].maximum)
        {
            Serial.printf("Entrez un entier entre %ld et %ld, sans unite.\n",
                          champs[champ].minimum, champs[champ].maximum);
        }
        else
        {
            valeurs[champ++] = valeur;
        }
        if (champ == sizeof(champs) / sizeof(champs[0]))
        {
            const bool declenche = busMoteurs.deplacerAbsolu(
                static_cast<uint8_t>(valeurs[0]), static_cast<int32_t>(valeurs[1]),
                static_cast<uint16_t>(valeurs[2]), static_cast<uint32_t>(valeurs[3]),
                static_cast<uint32_t>(valeurs[4]));
            Serial.println(declenche ? "Declenchement confirme."
                                     : "Le variateur n'a pas confirme le declenchement.");
            champ = 0;
        }
        longueur = 0;
        ligneInvalide = false;
        afficherInvite = true;
        break;
    }
}

void menu()
{
   if (Serial1.available() > 0) 
    {
        String message = Serial1.readStringUntil('\n');
        // int message2;
        Serial.println(message);        // message.trim(); // Retire notamment le '\r' éventuel
        if (message == "START_MATRIX")
        {
            ecrire_matrice();
        }
        if (message == "FREE_DRIVE")
        {
            free_drive_flag=1;
        }
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
void free_drive()
{

}
void controle_moteur1()
{
    
}
void controle_moteur2()
{
    
}
void controle_moteur3()
{
    
}
void moteur_rotation()
{
    
}
void moteur_pince()
{

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
