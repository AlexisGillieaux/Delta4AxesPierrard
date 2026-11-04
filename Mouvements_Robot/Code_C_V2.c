#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>

// Configuration du port série sous Windows
#define PORT "COM3"  // Modifiez par votre port (ex: "COM3", "COM4"...)
#define BAUDRATE CBR_9600

// Fonction de calcul du CRC16 Modbus (Équivalent de crcmod avec 0x18005, rev=True, init=0xFFFF)
unsigned short calculate_crc16(const unsigned char *data, int length) {
    unsigned short crc = 0xFFFF;
    for (int i = 0; i < length; i++) {
        crc ^= (unsigned short)data[i];
        for (int j = 8; j != 0; j--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// Convertit une chaîne hexadécimale en tableau d'octets
int hex_string_to_bytes(const char *hex_str, unsigned char *out_bytes) {
    int len = strlen(hex_str);
    int byte_len = len / 2;
    for (int i = 0; i < byte_len; i++) {
        sscanf(&hex_str[2 * i], "%2hhx", &out_bytes[i]);
    }
    return byte_len;
}

// Affiche un tableau d'octets au format hexadécimal
void print_hex(const unsigned char *data, int len) {
    for (int i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
}

int main() {
    // 1. Ouverture du port série sous Windows
    HANDLE hSerial = CreateFile(
        PORT,
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL
    );

    if (hSerial == INVALID_HANDLE_VALUE) {
        printf("Erreur : Impossible d'ouvrir le port %s\n", PORT);
        return 1;
    }

    // Configuration des paramètres du port (Baudrate, 8 bits, pas de parité, 1 stop bit)
    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(hSerial, &dcbSerialParams)) {
        printf("Erreur lors de la récupération de l'état du port\n");
        CloseHandle(hSerial);
        return 1;
    }

    dcbSerialParams.BaudRate = BAUDRATE;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity   = NOPARITY;

    if (!SetCommState(hSerial, &dcbSerialParams)) {
        printf("Erreur lors de la configuration du port série\n");
        CloseHandle(hSerial);
        return 1;
    }

    // Configuration des Timeouts de lecture/écriture
    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 500; // Timeout de 500ms
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    SetCommTimeouts(hSerial, &timeouts);

    printf("Connecté avec succès au port série : %s\n\n", PORT);

    // Définition des commandes sans CRC
    const char *commands_without_crc[] = {
        "010600000000",  // Set drive control mode to position mode C00.00=0
        "010611010000",  // Set position profile command to absolute positioning C11.01=0
        "010603000002",  // Set position command source to step increment C03.00=2
        "0106030D0001",  // Set running speed to 1 rpm C03.0D=1
        "0106030C1388",  // Set displacement to 5000 pulses C03.0C=5000
        "010604000022",  // Configure DI1 as position step increment enable C04.00=34 (0x22)
        "010604110001",  // Enable motor and start operation C04.11=1
        "010604010001",  // Trigger DI1 enable C04.01=1
        "010604110000",  // Emergency stop command, disable motor C04.11=0
        "010604010000"   // Disable DI1 C04.01=0
    };

    const char *descriptions[] = {
        "Set drive control mode to position mode (C00.00=0)",
        "Set position profile to absolute positioning (C11.01=0)",
        "Set position command source to step increment (C03.00=2)",
        "Set running speed to 1 rpm (C03.0D=1)",
        "Set displacement to 5000 pulses (C03.0C=5000)",
        "Configure DI1 as position step increment enable (C04.00=34)",
        "Enable motor and start operation (C04.11=1)",
        "Trigger DI1 enable (C04.01=1)",
        "Emergency stop command, disable motor (C04.11=0)",
        "Disable DI1 (C04.01=0)"
    };

    int num_commands = 10;

    // Envoi séquentiel des commandes
    for (int i = 0; i < num_commands; i++) {
        printf("Exécution de la commande %d/%d\n", i + 1, num_commands);
        
        unsigned char cmd_bytes[256];
        int cmd_len = hex_string_to_bytes(commands_without_crc[i], cmd_bytes);

        // Calcul du CRC16
        unsigned short crc = calculate_crc16(cmd_bytes, cmd_len);

        // Ajout du CRC en format Little-Endian
        cmd_bytes[cmd_len] = crc & 0xFF;
        cmd_bytes[cmd_len + 1] = (crc >> 8) & 0xFF;
        int full_len = cmd_len + 2;

        // Envoi de la commande via l'API Windows
        DWORD bytesWritten;
        WriteFile(hSerial, cmd_bytes, full_len, &bytesWritten, NULL);

        printf("Sending command: %s\n", descriptions[i]);
        printf("Data: ");
        print_hex(cmd_bytes, full_len);

        Sleep(300); // Pause de 300ms (Sleep prend des millisecondes sous Windows)

        // Lecture de la réponse (8 octets attendus)
        unsigned char response[256] = {0};
        DWORD bytesRead = 0;
        ReadFile(hSerial, response, 8, &bytesRead, NULL);

        if (bytesRead > 0) {
            printf("Device response: ");
            print_hex(response, bytesRead);
        } else {
            printf("Error: No response received!\n");
            CloseHandle(hSerial);
            return 1;
        }

        // Délais d'attente selon le type de commande
        if (strstr(descriptions[i], "Enable") || strstr(descriptions[i], "Trigger") || strstr(descriptions[i], "Emergency")) {
            Sleep(1000); // 1 seconde
        } else {
            Sleep(500);  // 500 ms
        }
        printf("\n");
    }

    // Fermeture du port série
    CloseHandle(hSerial);
    printf("All commands processed, serial port closed\n");
    return 0;
}