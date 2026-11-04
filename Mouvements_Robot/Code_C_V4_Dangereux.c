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
    Sleep(10); // Petit délai de battement bus
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
    timeouts.ReadIntervalTimeout = 10;
    timeouts.ReadTotalTimeoutConstant = 30;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    SetCommTimeouts(hSerial, &timeouts);

    unsigned char rx_buf[64];

    // 1. Passage en mode Couple (C00.00 = 2) - Ajustez selon votre config de mode si nécessaire
    unsigned char mode_cmd[] = { 0x01, 0x06, 0x20, 0x00, 0x00, 0x02 };
    send_modbus_frame(hSerial, mode_cmd, 4, rx_buf, 8);

    // 2. Activation du moteur (S-ON)
    unsigned char son_cmd[] = { 0x01, 0x06, 0x04, 0x11, 0x00, 0x01 };
    send_modbus_frame(hSerial, son_cmd, 4, rx_buf, 8);

    printf("Mode Freedrive avec compensation active demarre ! (Ctrl+C pour quitter)...\n");

    // Paramètres du système
    double Kp = 0.02;                 // Rigidité autour de la position cible (ajustable)
    double max_gravity_torque = 150.0;// Couple max nécessaire à l'horizontale (ajustez selon le poids de la médaille)
    
    // On enregistre la position initiale (Position A) au lancement du programme (quand le bras est vers le bas à 0°)
    unsigned char init_pos_cmd[] = { 0x01, 0x03, 0x40, 0x09, 0x00, 0x01 };
    unsigned char init_resp[16];
    send_modbus_frame(hSerial, init_pos_cmd, 6, init_resp, 7);
    long target_position = (init_resp[3] << 8) | init_resp[4];

    while (1) {
        // A. Lecture de l'angle mécanique actuel (U40_09 -> 0x4009)
        unsigned char read_pos_cmd[] = { 0x01, 0x03, 0x40, 0x09, 0x00, 0x01 };
        unsigned char pos_resp[16];
        int bytes = send_modbus_frame(hSerial, read_pos_cmd, 6, pos_resp, 7);

        if (bytes >= 5) {
            long current_position = (pos_resp[3] << 8) | pos_resp[4];

            // Gestion du passage par zéro de l'encodeur (0 <-> 36000) pour l'erreur de position
            long position_error = target_position - current_position;
            if (position_error > 18000) position_error -= 36000;
            if (position_error < -18000) position_error += 36000;

            // B. Conversion en radians pour le calcul trigonométrique
            double angle_rad = ((double)current_position / 36000.0) * 2.0 * M_PI;

            // C. Calcul du couple de compensation de gravité (Sinus)
            double gravity_torque = max_gravity_torque * sin(angle_rad);

            // D. Ajout de la correction proportionnelle de maintien (Position A)
            double computed_torque = gravity_torque - (Kp * position_error); // Inversion du signe (-)

            // Sécurité couple (ex: max 30% du couple nominal)
            if (computed_torque > 300) computed_torque = 300;
            if (computed_torque < -300) computed_torque = -300;

            short torque_val = (short)computed_torque;

            // E. Envoi de la consigne de couple au registre C03.41 (0x0341)
            unsigned char set_torque_cmd[8];
            set_torque_cmd[0] = 0x01;
            set_torque_cmd[1] = 0x06;
            set_torque_cmd[2] = 0x03;
            set_torque_cmd[3] = 0x41;
            set_torque_cmd[4] = (torque_val >> 8) & 0xFF;
            set_torque_cmd[5] = torque_val & 0xFF;

            unsigned char torque_resp[16];
            send_modbus_frame(hSerial, set_torque_cmd, 6, torque_resp, 8);
        }

        // Boucle rapide (~20 ms pour un suivi fluide)
        Sleep(20);
    }

    CloseHandle(hSerial);
    return 0;
}