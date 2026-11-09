#include <Arduino.h>
#include <StateMachine.h>

// CONFIGURATION

constexpr unsigned long BAUD_RATE = 115200;
constexpr unsigned long PERIODE_ENVOI_MS = 20;
constexpr unsigned long TIMEOUT_MATRICE_MS = 5000;

constexpr int NB_LIGNES = 5; // 4 positions moteurs + 1 vitesse commune.
constexpr int NB_COLONNES = 9999;
constexpr int TAILLE_MESSAGE = 128;

int matrice[NB_LIGNES][NB_COLONNES] = {0};

int flag = 0;
int i = 0;
int j = 0;
int controle_driver = 999;
int ecrire_raspberry_flag = 0;
int free_drive_flag = 0;

String message_raspberry;
String message;

int nombre_colonnes = 0;
int indice_trajectoire = 0;

unsigned long dernier_envoi = 0;
unsigned long derniere_reception = 0;

// Conditions des transitions
bool demande_matrice = false;
bool fin_matrice = false;
bool demande_execution = false;
bool fin_execution = false;
bool demande_free_drive = false;
bool demande_stop = false; // Arret logique uniquement : arret physique a implementer.
bool demande_reset = false;
bool erreur = false;

// DECLARATIONS DES FONCTIONS

void menu();
void ecrire_matrice(const String &ligne);
void driver();
void ecrireToRaspberry();
void free_drive();

void traiterMessage(const String &ligne);
void envoyer_positions_vitesses(int indice);
void envoyerDriver(HardwareSerial &port, int position, int vitesse);
void envoyerMessageRaspberry(const String &texte);
void declarerErreur(const String &texte);

// Etats
void etatAttente();
void etatReception();
void etatExecution();
void etatFreeDrive();
void etatErreur();

// Transitions
bool versMatrice();
bool versExecution();
bool versFreeDrive();
bool versAttenteReception();
bool versAttenteExecution();
bool versAttenteFreeDrive();
bool versErreur();
bool versReset();

// MACHINE A ETATS

StateMachine machine = StateMachine();

State* ATTENTE = machine.addState(&etatAttente);
State* RECEPTION_MATRICE = machine.addState(&etatReception);
State* EXECUTION_TRAJECTOIRE = machine.addState(&etatExecution);
State* FREE_DRIVE = machine.addState(&etatFreeDrive);
State* ERREUR = machine.addState(&etatErreur);

// SETUP

void setup()
{
    Serial.begin(BAUD_RATE);

   
    Serial1.begin(BAUD_RATE);

   
    Serial2.begin(BAUD_RATE);
    Serial3.begin(BAUD_RATE);
    Serial4.begin(BAUD_RATE);
    Serial5.begin(BAUD_RATE);

   
    ATTENTE->addTransition(&versErreur, ERREUR);
    ATTENTE->addTransition(&versMatrice, RECEPTION_MATRICE);
    ATTENTE->addTransition(&versExecution, EXECUTION_TRAJECTOIRE);
    ATTENTE->addTransition(&versFreeDrive, FREE_DRIVE);

   
    RECEPTION_MATRICE->addTransition(&versErreur, ERREUR);
    RECEPTION_MATRICE->addTransition(&versAttenteReception, ATTENTE);

   
    EXECUTION_TRAJECTOIRE->addTransition(&versErreur, ERREUR);
    EXECUTION_TRAJECTOIRE->addTransition(&versAttenteExecution, ATTENTE);

   
    FREE_DRIVE->addTransition(&versErreur, ERREUR);
    FREE_DRIVE->addTransition(&versAttenteFreeDrive, ATTENTE);

   
    ERREUR->addTransition(&versReset, ATTENTE);

    Serial.println("Teensy 4.1 initialise");
}

// LOOP PRINCIPALE

void loop()
{
   
    menu();

   
    machine.run();

   
    if (ecrire_raspberry_flag == 1)
    {
        ecrireToRaspberry();
        ecrire_raspberry_flag = 0;
    }
}

// MENU : RECEPTION RASPBERRY

void menu()
{
    while (Serial1.available() > 0)
    {
        char caractere = Serial1.read();

        if (caractere == '\n')
        {
            message.trim();

            if (message.length() > 0)
            {
                traiterMessage(message);
            }

            message = "";
        }
        else if (caractere != '\r')
        {
            if (message.length() < TAILLE_MESSAGE)
            {
                message += caractere;
            }
            else
            {
                message = "";
                declarerErreur("MESSAGE_TROP_LONG");
            }
        }
    }
}

// ANALYSE DES COMMANDES

void traiterMessage(const String &ligne)
{
    Serial.print("Raspberry : ");
    Serial.println(ligne);

    if (machine.isInState(ERREUR))
    {
        if (ligne == "RESET")
        {
            demande_reset = true;
        }
        return;
    }

    if (erreur)
        return;

   
    if (ligne == "STOP")
    {
        demande_stop = true;
        return;
    }

    if (machine.isInState(ATTENTE))
    {
        if (ligne == "START_MATRIX")
        {
            nombre_colonnes = 0;
            demande_matrice = true;
            derniere_reception = millis();
        }
        else if (ligne == "START_MOVE")
        {
            if (nombre_colonnes > 0)
            {
                demande_execution = true;
            }
            else
            {
                envoyerMessageRaspberry("NO_MATRIX");
            }
        }
        else if (ligne == "FREE_DRIVE")
        {
            demande_free_drive = true;
        }
    }
    else if (machine.isInState(RECEPTION_MATRICE))
    {
        if (ligne == "END_MATRIX")
        {
            if (nombre_colonnes > 0)
            {
                fin_matrice = true;
                flag = 1;

                envoyerMessageRaspberry("MATRIX_DONE");
            }
            else
            {
                declarerErreur("MATRIX_EMPTY");
            }
        }
        else
        {
            ecrire_matrice(ligne);
            derniere_reception = millis();
        }
    }
}

// ETAT ATTENTE

void etatAttente()
{
    if (machine.executeOnce)
    {
        Serial.println("ETAT : ATTENTE");

        demande_matrice = false;
        fin_matrice = false;
        demande_execution = false;
        fin_execution = false;
        demande_free_drive = false;
        demande_stop = false;
        free_drive_flag = 0;
    }
}

// ETAT RECEPTION MATRICE

void etatReception()
{
    if (machine.executeOnce)
    {
        Serial.println("ETAT : RECEPTION MATRICE");
        demande_matrice = false;
        derniere_reception = millis();
    }

    if (millis() - derniere_reception >
        TIMEOUT_MATRICE_MS)
    {
        declarerErreur("MATRIX_TIMEOUT");
    }
}

// ETAT EXECUTION TRAJECTOIRE

void etatExecution()
{
    if (machine.executeOnce)
    {
        Serial.println("ETAT : EXECUTION");

        indice_trajectoire = 0;
        dernier_envoi = millis();
        demande_execution = false;
        fin_execution = false;

        envoyerMessageRaspberry("MOVE_STARTED");
    }

    if (demande_stop || erreur)
        return;

    if (millis() - dernier_envoi >=
        PERIODE_ENVOI_MS)
    {
        dernier_envoi += PERIODE_ENVOI_MS;

        if (indice_trajectoire < nombre_colonnes)
        {
            envoyer_positions_vitesses(indice_trajectoire);

            indice_trajectoire++;
        }
        else
        {
            fin_execution = true;

            envoyerMessageRaspberry("COMMANDS_SENT");
        }
    }
}

// ETAT FREE DRIVE

void etatFreeDrive()
{
    if (machine.executeOnce)
    {
        Serial.println("ETAT : FREE DRIVE");

        demande_free_drive = false;
        free_drive_flag = 1;
    }

    if (!demande_stop && !erreur)
    {
        driver();
    }
}

// ETAT ERREUR

void etatErreur()
{
    if (machine.executeOnce)
    {
        Serial.println("ETAT : ERREUR");

        free_drive_flag = 0;
        demande_execution = false;
        demande_free_drive = false;
        demande_matrice = false;

        envoyerMessageRaspberry("ERROR");
    }
}

// TRANSITIONS

bool versMatrice()
{
    return demande_matrice;
}

bool versExecution()
{
    return demande_execution;
}

bool versFreeDrive()
{
    return demande_free_drive;
}

bool versErreur()
{
    return erreur;
}

bool versAttenteReception()
{
    return fin_matrice;
}

bool versAttenteExecution()
{
    return fin_execution || demande_stop;
}

bool versAttenteFreeDrive()
{
    return demande_stop;
}

bool versReset()
{
    if (demande_reset)
    {
        demande_reset = false;
        erreur = false;
        demande_stop = false;
        demande_matrice = false;
        demande_execution = false;
        demande_free_drive = false;
        fin_matrice = false;
        fin_execution = false;
        nombre_colonnes = 0;
        flag = 0;
        return true;
    }

    return false;
}

// ECRITURE DE LA MATRICE

void ecrire_matrice(const String &ligne)
{
    if (nombre_colonnes >= NB_COLONNES)
    {
        declarerErreur("MATRIX_FULL");
        return;
    }

    int valeurs[NB_LIGNES];
    int debut = 0;

    for (int k = 0; k < NB_LIGNES; k++)
    {
        while (debut < (int)ligne.length() &&
               ligne[debut] == ' ')
        {
            debut++;
        }

        if (debut >= (int)ligne.length())
        {
            declarerErreur("MATRIX_FORMAT");
            return;
        }

        int fin = ligne.indexOf(' ', debut);

        if (fin == -1)
            fin = ligne.length();

        String valeur = ligne.substring(debut, fin);
        valeur.trim();

       
        int debutChiffres =
            (valeur[0] == '-') ? 1 : 0;

        if (debutChiffres >= (int)valeur.length())
        {
            declarerErreur("MATRIX_FORMAT");
            return;
        }

        for (int c = debutChiffres;
             c < (int)valeur.length(); c++)
        {
            if (!isDigit(valeur[c]))
            {
                declarerErreur("MATRIX_FORMAT");
                return;
            }
        }

        valeurs[k] = valeur.toInt();
        debut = fin + 1;
    }

   
    while (debut < (int)ligne.length() &&
           ligne[debut] == ' ')
    {
        debut++;
    }

    if (debut < (int)ligne.length())
    {
        declarerErreur("MATRIX_FORMAT");
        return;
    }

   
    for (int k = 0; k < NB_LIGNES; k++)
    {
        matrice[k][nombre_colonnes] = valeurs[k];
    }

    nombre_colonnes++;

    Serial.print("Colonne recue : ");
    Serial.println(nombre_colonnes);
}

// ENVOI POSITION ET VITESSE AUX DRIVERS

void envoyer_positions_vitesses(int indice)
{
   
   
   

    int vitesse = matrice[4][indice];

    envoyerDriver(Serial2, matrice[0][indice], vitesse);

    envoyerDriver(Serial3, matrice[1][indice], vitesse);

    envoyerDriver(Serial4, matrice[2][indice], vitesse);

    envoyerDriver(Serial5, matrice[3][indice], vitesse);
}

void envoyerDriver(HardwareSerial &port, int position, int vitesse)
{
   
   

    port.print(position);
    port.print(",");
    port.println(vitesse);
}

// DRIVER

void driver()
{
    if (free_drive_flag == 1)
    {
        free_drive();
    }
}

// FREE DRIVE

void free_drive()
{
   
   

    static unsigned long dernierFreeDrive = 0;

    if (millis() - dernierFreeDrive >= 100)
    {
        dernierFreeDrive = millis();

        Serial2.println("FREE_DRIVE");
        Serial3.println("FREE_DRIVE");
        Serial4.println("FREE_DRIVE");
        Serial5.println("FREE_DRIVE");
    }
}

// ENVOI VERS RASPBERRY

void ecrireToRaspberry()
{
    Serial1.println(message_raspberry);
}

void envoyerMessageRaspberry(const String &texte)
{
    message_raspberry = texte;
    ecrire_raspberry_flag = 1;
}

// GESTION DES ERREURS

void declarerErreur(const String &texte)
{
    Serial.print("ERREUR : ");
    Serial.println(texte);

    erreur = true;
    free_drive_flag = 0;
}
