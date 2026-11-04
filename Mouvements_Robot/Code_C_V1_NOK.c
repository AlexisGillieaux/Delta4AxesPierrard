#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BAUD_RATE 115200
#define SLAVE_ADDRESS 0x01
#define REG_CONTROL_MODE 0x0000
#define REG_SPEED_SOURCE 0x0320
#define REG_SPEED_REFERENCE 0x0321
#define TEST_SPEED_RPM 25
#define TEST_DURATION_MS 5000

static volatile LONG stop_requested = 0;

static uint16_t calculate_crc(const uint8_t *buffer, size_t length)
{
    uint16_t crc = 0xFFFF;

    for (size_t pos = 0; pos < length; pos++) {
        crc ^= buffer[pos];
        for (int bit = 0; bit < 8; bit++) {
            if ((crc & 0x0001) != 0) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001);
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

static BOOL WINAPI console_control_handler(DWORD control_type)
{
    if (control_type == CTRL_C_EVENT || control_type == CTRL_BREAK_EVENT) {
        InterlockedExchange(&stop_requested, 1);
        return TRUE;
    }
    return FALSE;
}

static void print_windows_error(const char *operation)
{
    fprintf(stderr, "%s (code erreur Windows %lu).\n",
            operation, (unsigned long)GetLastError());
}

static int make_port_path(const char *port_name, char *port_path, size_t path_size)
{
    if (_strnicmp(port_name, "COM", 3) != 0 || port_name[3] == '\0') {
        return 0;
    }

    for (const char *digit = port_name + 3; *digit != '\0'; digit++) {
        if (*digit < '0' || *digit > '9') {
            return 0;
        }
    }

    int length = snprintf(port_path, path_size, "\\\\.\\%s", port_name);
    return length >= 0 && (size_t)length < path_size;
}

static HANDLE open_serial_port(const char *port_path)
{
    HANDLE serial_port = CreateFileA(
        port_path,
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL);

    if (serial_port == INVALID_HANDLE_VALUE) {
        print_windows_error("Impossible d'ouvrir le port série");
        return INVALID_HANDLE_VALUE;
    }

    DCB settings;
    memset(&settings, 0, sizeof(settings));
    settings.DCBlength = sizeof(settings);

    if (!GetCommState(serial_port, &settings)) {
        print_windows_error("Erreur GetCommState");
        CloseHandle(serial_port);
        return INVALID_HANDLE_VALUE;
    }

    settings.BaudRate = BAUD_RATE;
    settings.ByteSize = 8;
    settings.Parity = NOPARITY;
    settings.StopBits = ONESTOPBIT;
    settings.fBinary = TRUE;
    settings.fParity = FALSE;
    settings.fOutxCtsFlow = FALSE;
    settings.fOutxDsrFlow = FALSE;
    settings.fDtrControl = DTR_CONTROL_DISABLE;
    settings.fRtsControl = RTS_CONTROL_DISABLE;
    settings.fOutX = FALSE;
    settings.fInX = FALSE;

    if (!SetCommState(serial_port, &settings)) {
        print_windows_error("Erreur SetCommState");
        CloseHandle(serial_port);
        return INVALID_HANDLE_VALUE;
    }

    COMMTIMEOUTS timeouts;
    memset(&timeouts, 0, sizeof(timeouts));
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.ReadTotalTimeoutConstant = 1000;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 2000;

    if (!SetCommTimeouts(serial_port, &timeouts)) {
        print_windows_error("Erreur SetCommTimeouts");
        CloseHandle(serial_port);
        return INVALID_HANDLE_VALUE;
    }

    if (!PurgeComm(serial_port, PURGE_RXCLEAR | PURGE_TXCLEAR)) {
        print_windows_error("Erreur PurgeComm");
        CloseHandle(serial_port);
        return INVALID_HANDLE_VALUE;
    }

    return serial_port;
}

static int read_modbus_write_response(
    HANDLE serial_port,
    const uint8_t request[8])
{
    uint8_t response[8];
    size_t total_read = 0;
    ULONGLONG deadline = GetTickCount64() + 2000;

    while (total_read < sizeof(response)) {
        DWORD bytes_read = 0;
        if (!ReadFile(
                serial_port,
                response + total_read,
                (DWORD)(sizeof(response) - total_read),
                &bytes_read,
                NULL)) {
            print_windows_error("Erreur ReadFile");
            return 0;
        }

        total_read += bytes_read;
        if (bytes_read == 0 && GetTickCount64() >= deadline) {
            fprintf(stderr, "Délai dépassé : aucune réponse Modbus complète.\n");
            return 0;
        }
    }

    uint16_t received_crc = (uint16_t)response[6] |
                            (uint16_t)((uint16_t)response[7] << 8);
    if (calculate_crc(response, 6) != received_crc) {
        fprintf(stderr, "Réponse Modbus rejetée : CRC invalide.\n");
        return 0;
    }

    if (memcmp(response, request, 6) != 0) {
        fprintf(stderr, "Réponse Modbus inattendue (la commande n'est pas confirmée).\n");
        return 0;
    }

    return 1;
}

static int send_modbus_write(
    HANDLE serial_port,
    uint16_t reg_addr,
    int16_t value)
{
    uint8_t frame[8];
    frame[0] = SLAVE_ADDRESS;
    frame[1] = 0x06;
    frame[2] = (uint8_t)(reg_addr >> 8);
    frame[3] = (uint8_t)(reg_addr & 0xFF);
    frame[4] = (uint8_t)((uint16_t)value >> 8);
    frame[5] = (uint8_t)((uint16_t)value & 0xFF);

    uint16_t crc = calculate_crc(frame, 6);
    frame[6] = (uint8_t)(crc & 0xFF);
    frame[7] = (uint8_t)(crc >> 8);

    if (!PurgeComm(serial_port, PURGE_RXCLEAR)) {
        print_windows_error("Erreur PurgeComm avant l'envoi");
        return 0;
    }

    size_t total_written = 0;
    while (total_written < sizeof(frame)) {
        DWORD written = 0;
        if (!WriteFile(
                serial_port,
                frame + total_written,
                (DWORD)(sizeof(frame) - total_written),
                &written,
                NULL)) {
            print_windows_error("Erreur WriteFile");
            return 0;
        }
        if (written == 0) {
            fprintf(stderr, "Le port série n'a écrit aucune donnée.\n");
            return 0;
        }
        total_written += written;
    }

    if (!FlushFileBuffers(serial_port)) {
        print_windows_error("Erreur FlushFileBuffers");
        return 0;
    }

    return read_modbus_write_response(serial_port, frame);
}

static int run_speed_test(HANDLE serial_port)
{
    printf("Configuration du mode vitesse et activation forcée de l'ENABLE (DI5/S-ON)...\n");

    if (!send_modbus_write(serial_port, REG_CONTROL_MODE, 1)) {
        return 0;
    }
    if (!send_modbus_write(serial_port, REG_SPEED_SOURCE, 0)) {
        return 0;
    }
    if (!send_modbus_write(serial_port, REG_SPEED_REFERENCE, 0)) {
        return 0;
    }

    printf("DI5/S-ON forcé à l'état actif. Le moteur est autorisé à tourner.\n");
    printf("Consigne demandée : %d tr/min, sens +, durée maximale %d ms.\n",
           TEST_SPEED_RPM, TEST_DURATION_MS);

    if (!send_modbus_write(
            serial_port, REG_SPEED_REFERENCE, TEST_SPEED_RPM)) {
        fprintf(stderr,
                "La consigne de mouvement n'est pas confirmée. "
                "Tentative d'envoi d'une consigne nulle.\n");
        send_modbus_write(serial_port, REG_SPEED_REFERENCE, 0);
        send_modbus_write(serial_port, REG_CONTROL_MODE, 0);
        return 0;
    }

    ULONGLONG start = GetTickCount64();
    while (GetTickCount64() - start < TEST_DURATION_MS &&
           InterlockedCompareExchange(&stop_requested, 0, 0) == 0) {
        Sleep(10);
    }

    printf("Arrêt demandé : envoi d'une consigne de vitesse nulle et coupure de l'ENABLE...\n");
    if (!send_modbus_write(serial_port, REG_SPEED_REFERENCE, 0)) {
        fprintf(stderr,
                "ERREUR CRITIQUE : la consigne nulle n'a pas été confirmée. "
                "Utilisez immédiatement l'arrêt d'urgence matériel.\n");
        return 0;
    }

    if (!send_modbus_write(serial_port, REG_CONTROL_MODE, 0)) {
        fprintf(stderr,
                "ERREUR CRITIQUE : la désactivation de DI5/S-ON n'a pas été confirmée.\n");
        return 0;
    }

    printf("Consigne nulle confirmée et DI5/S-ON désactivé après %d ms.\n",
           TEST_DURATION_MS);
    return 1;
}

int main(int argc, char *argv[])
{
    if (argc != 2) {
        fprintf(stderr, "Utilisation : %s COMx (exemple : %s COM3)\n",
                argv[0], argv[0]);
        return EXIT_FAILURE;
    }

    char port_path[32];
    if (!make_port_path(argv[1], port_path, sizeof(port_path))) {
        fprintf(stderr, "Nom de port invalide : utilisez COM suivi d'un numéro (ex. COM3).\n");
        return EXIT_FAILURE;
    }

    HANDLE serial_port = open_serial_port(port_path);
    if (serial_port == INVALID_HANDLE_VALUE) {
        return EXIT_FAILURE;
    }

    if (!SetConsoleCtrlHandler(console_control_handler, TRUE)) {
        print_windows_error("Impossible d'installer le gestionnaire Ctrl+C");
        CloseHandle(serial_port);
        return EXIT_FAILURE;
    }

    printf("Liaison Modbus RTU sur %s (%d bauds, 8N1).\n", argv[1], BAUD_RATE);
    fprintf(stderr,
            "Vérifiez que le moteur est arrêté, DI5/S-ON est inactive et "
            "l'arrêt d'urgence est accessible.\n");

    int success = run_speed_test(serial_port);
    CloseHandle(serial_port);
    return success ? EXIT_SUCCESS : EXIT_FAILURE;
}
