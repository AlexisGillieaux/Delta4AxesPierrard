#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>
#include <math.h>

#define PORT "COM3"
#define BAUDRATE CBR_9600

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Fonction de calcul du CRC16 Modbus
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

int send_modbus_frame(HANDLE hSerial, unsigned char *tx_data, int tx_len, unsigned char *rx_data, int rx_expected) {
    DWORD bytesWritten, bytesRead;
    unsigned short crc = calculate_crc16(tx_data, tx_len);
    tx_data[tx_len] = crc & 0xFF;
    tx_data[tx_len + 1] = (crc >> 8) & 0xFF;
    int full_len = tx_len + 2;

    WriteFile(hSerial, tx_data, full_len, &bytesWritten, NULL);
    Sleep(15);
    ReadFile(hSerial, rx_data, rx_expected, &bytesRead, NULL);
    return bytesRead;
}

int main() {
    HANDLE hSerial = CreateFile(PORT, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (hSerial == INVALID_HANDLE_VALUE) {
        printf("Erreur : Impossible d'ouvrir le port %s\n", PORT);
        return 1;
    }

    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    GetCommState(hSerial, &dcbSerialParams);
    dcbSerialParams.BaudRate = BAUDRATE;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    SetCommState(hSerial, &dcbSerialParams);

    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = 20;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    SetCommTimeouts(hSerial, &timeouts);

    printf("Connexion etablie. Configuration...\n");

    // 1. Passage en mode Couple
    unsigned char mode_cmd[] = { 0x01, 0x06, 0x20, 0x00, 0x00, 0x02 };
    unsigned char rx_buf[64];
    send_modbus_frame(hSerial, mode_cmd, 4, rx_buf, 8);

    // 2. Activation du moteur (S-ON)
    unsigned char son_cmd[] = { 0x01, 0x06, 0x04, 0x11, 0x00, 0x01 };
    send_modbus_frame(hSerial, son_cmd, 4, rx_buf, 8);

    printf("Mode actif avec compensation de gravite angulaire. (Ctrl+C pour quitter)...\n");

    // Paramètres à ajuster selon votre montage
    long target_position = 0;        // Position A de référence (en impulsions au démarrage = 0)
    double Kp = 0.05;                // Gain proportionnel de rappel (ajustez si besoin)
    
    // Nombre d'impulsions pour un tour complet du moteur (ex: 10000 ou 40000 selon le mode/encodeur ou réducteur)
    // Adaptez cette valeur selon votre configuration mécanique exacte :
    double pulses_per_rev = 10000.0; 
    
    // Couple maximal nécessaire lorsque le bras est à l'horizontale (90°)
    // À ajuster : augmentez cette valeur si la médaille et le bras 20cm tombent encore un peu.
    double max_gravity_torque = 150.0; 

    while (1) {
        // Demande de lecture de la position actuelle
        // Essai de lecture à l'adresse 0x0D0A (COD_0A)
        // Demande de lecture de U40_09 (Mechanical angle) -> Adresse Modbus 0x4009
    unsigned char read_pos_cmd[] = { 0x01, 0x03, 0x40, 0x09, 0x00, 0x01 }; // 1 registre de 16 bits
    unsigned char pos_resp[32];
    int bytes = send_modbus_frame(hSerial, read_pos_cmd, 6, pos_resp, 7);

        if (bytes >= 5) {
        // Reconstitution de la valeur de l'angle lue
        unsigned short raw_angle = (pos_resp[3] << 8) | pos_resp[4];
        printf("Angle mécanique lu : %u\n", raw_angle);
        }

        printf("Octets recus (%d) : ", bytes);
        for(int i = 0; i < bytes; i++) {
            printf("%02X ", pos_resp[i]);
        }
        printf("\n");

        Sleep(500); // Pause d'une demi-seconde pour ne pas spammer le terminal
    }

    
    // while (1) {
    //     // Lecture de la position actuelle du moteur
    //     unsigned char read_pos_cmd[] = { 0x01, 0x03, 0x10, 0x00, 0x00, 0x02 };
    //     unsigned char pos_resp[32];
    //     int bytes = send_modbus_frame(hSerial, read_pos_cmd, 6, pos_resp, 9);

    //     if (bytes >= 7) {
    //         long current_position = (pos_resp[3] << 24) | (pos_resp[4] << 16) | (pos_resp[5] << 8) | pos_resp[6];

    //         // 1. Calcul de l'angle actuel
    //         double angle_rad = ((double)current_position / pulses_per_rev) * 2.0 * M_PI;
    //         double angle_deg = angle_rad * (180.0 / M_PI); // Pour afficher en degrés lisibles

    //         // 2. Compensation de gravité dynamique
    //         double gravity_torque = max_gravity_torque * sin(angle_rad);

    //         // 3. Correction proportionnelle
    //         long position_error = target_position - current_position;
    //         double correction_torque = Kp * position_error;

    //         double computed_torque = gravity_torque + correction_torque;

    //         // Affichage pour diagnostic (regardez ce qui s'affiche quand vous bougez le bras à la main)
    //         printf("Pos: %ld | Angle calc: %.1f deg | Couple: %.1f\n", current_position, angle_deg, computed_torque);

    //         // Sécurité : limiter les valeurs de couple envoyées au variateur
    //         if (computed_torque > 500) computed_torque = 500;
    //         if (computed_torque < -500) computed_torque = -500;

    //         short torque_val = (short)computed_torque;

    //         // 5. Envoi de la consigne de couple au registre C03.41
    //         unsigned char set_torque_cmd[8];
    //         set_torque_cmd[0] = 0x01;
    //         set_torque_cmd[1] = 0x06;
    //         set_torque_cmd[2] = 0x23;
    //         set_torque_cmd[3] = 0x41;
    //         set_torque_cmd[4] = (torque_val >> 8) & 0xFF;
    //         set_torque_cmd[5] = torque_val & 0xFF;

    //         unsigned char torque_resp[16];
    //         send_modbus_frame(hSerial, set_torque_cmd, 6, torque_resp, 8);
    //     }

    //     Sleep(20); // Boucle à ~50 Hz
    // }

    CloseHandle(hSerial);
    return 0;
}