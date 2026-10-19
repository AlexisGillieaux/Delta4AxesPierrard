#include <Arduino.h>

constexpr unsigned long BAUD_RATE = 115200;
int flag = 0;
int i = 0, j = 0;
int matrice[5][9999] = {0};  // Déclaration de la matrice globale
void menu();
void ecrire_matrice();
String message;
void setup() 
{
    Serial.begin(BAUD_RATE);   // Moniteur USB pour le débogage
    Serial1.begin(BAUD_RATE);  // UART matériel : RX1 broche 0, TX1 broche 1
    Serial.println("UART du Teensy 4.1 initialise.");
    
}

void loop() 
{
    menu();
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