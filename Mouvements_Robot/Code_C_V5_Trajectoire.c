/*
 * Code_C_V5_Trajectoire.c
 *
 * Fait suivre à UN moteur (variateur A6-RS, Modbus RTU) une trajectoire calculée
 * par Coords/trajectoires4Axes.py et exportée en CSV par exporter_trajectoire.py.
 *
 * Principe (manuel A6-RS, §4 "Multi-position", mode PP, et chapitre 9 Modbus) :
 *   - Le variateur est en mode position (C00.00 = 0). Sa consigne vient de la
 *     planification interne (C03.00 = 1) en mode PP (C11.00 = 3) : à chaque front
 *     sur l'entrée "position planning trigger" (FunIN.19), il parcourt le
 *     déplacement C11.06 (en impulsions) à la vitesse C11.08 (tr/min) en
 *     asservissant lui-même la position.
 *   - Pour chaque point de la trajectoire, le PC écrit le déplacement depuis le
 *     point précédent, la vitesse si elle change, puis donne un front sur DI6.
 *   - Résolution : le programme lit C00.02 (impulsions par tour moteur) dans le
 *     variateur. 360° valent exactement C00.02 impulsions et chaque position est
 *     arrondie depuis le point de départ : aucun reste ne s'accumule.
 *   - Plage absolue : les angles du bras (ceux de trajectoires4Axes.py, négatifs
 *     comme positifs) doivent rester dans [ANGLE_MIN_DEG ; ANGLE_MAX_DEG], un tour
 *     au plus. Le codeur est absolu sur un tour : son angle mécanique (U40.09) et le
 *     zéro enregistré par --zero donnent l'angle réel du bras à chaque lancement,
 *     même après un redémarrage. Le moteur rejoint d'abord le départ de la
 *     trajectoire (un petit déplacement test vérifie avant le sens du codeur), puis
 *     la suit. Une trajectoire qui sort de la plage est refusée.
 *   - Le débit du variateur est détecté automatiquement. Le pas de temps vaut
 *     PERIODE_MIN_MS (20 ms) si la liaison le permet, sinon il est allongé d'après
 *     le temps mesuré d'une trame.
 *   - S-ON (DI5) et le déclenchement (DI6) sont commandés par logiciel en inversant
 *     la logique NO/NC de ces entrées non câblées (C04.11 et C04.15), comme dans
 *     Bouger_Moteur_OK.py.
 *   - C0A.05 = 0 : pendant une trajectoire, les écritures Modbus ne sont PAS
 *     sauvegardées en EEPROM. Un redémarrage du variateur efface ces réglages.
 *
 * Réglage du variateur, une seule fois (SAUVEGARDÉ, effectif après redémarrage) :
 *     .\Code_C_V5_Trajectoire.exe --config-variateur COM5
 *   C00.02 = résolution du codeur (pleine résolution) et C0A.01 = 115200 bauds.
 *
 * Zéro, une fois par moteur (refaire si le moteur est démonté ou recalé) : moteur
 * coupé, placer le bras à theta = 0° (ou à un angle connu A), puis
 *     .\Code_C_V5_Trajectoire.exe --zero 1 COM5 [A]
 *   enregistre l'angle mécanique du codeur dans zero_moteur1.txt (à côté du .exe).
 *
 * Compilation (dans le dossier Mouvements_Robot) :
 *     gcc Code_C_V5_Trajectoire.c -o Code_C_V5_Trajectoire.exe
 *
 * Utilisation :
 *     .\Code_C_V5_Trajectoire.exe [fichier.csv] [moteur 1-4] [COMx | SIMU]
 *     défaut : trajectoire_moteurs.csv 1 COM3
 *     SIMU : n'ouvre pas de port série, affiche les trames qui seraient envoyées.
 */

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <conio.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ============================================================================
 *                         RÉGLAGES À ADAPTER
 * ========================================================================== */

#define PORT_DEFAUT         "COM3"
#define FICHIER_DEFAUT      "trajectoire_moteurs.csv"
#define MOTEUR_DEFAUT       1          /* colonne du CSV : 1 = θ1 ... 4 = θ4 */
#define ADRESSE_ESCLAVE     0x01       /* C0A.00 : adresse Modbus du variateur */
#define SENS_MOTEUR         1          /* +1, ou -1 si le moteur tourne dans le mauvais sens */

/* Cadence */
#define PERIODE_MIN_MS      20         /* pas de temps visé (allongé si la liaison est trop lente) */
#define MARGE_VITESSE       1.15       /* vitesse envoyée = vitesse nécessaire x marge, arrondie au
                                          tr/min supérieur : chaque déplacement finit avant le suivant */
#define TEMPS_ACCEL_MS      5          /* C11.0A / C11.0C (probablement ms pour 0 -> 1000 tr/min) */

/* Plage des angles du bras, dans le repère de trajectoires4Axes.py (un tour au plus).
   Toute trajectoire qui en sort est refusée et le moteur n'en sort jamais. */
#ifndef ANGLE_MIN_DEG
#define ANGLE_MIN_DEG       (-180.0)
#endif
#ifndef ANGLE_MAX_DEG
#define ANGLE_MAX_DEG       180.0
#endif

/* Sécurité */
#define VITESSE_MAX_RPM     60         /* vitesse maximale envoyée au moteur (tr/min) */
#define TOLERANCE_FIN_DEG   0.1        /* écart final toléré entre position mesurée et attendue */
#define VITESSE_APPROCHE_RPM 5         /* vitesse pour rejoindre le départ de la trajectoire */
#define SONDE_DEG           2.0        /* déplacement test qui vérifie le sens du codeur */
#define TOLERANCE_CODEUR_DEG 0.5       /* écart toléré entre l'angle du codeur et l'angle attendu */

/* Mode SIMU : angle du bras simulé au lancement */
#ifndef SIMU_THETA_DEPART_DEG
#define SIMU_THETA_DEPART_DEG 0.0
#endif

/* Communication */
#define TIMEOUT_REPONSE_MS  300        /* attente maximale d'une réponse Modbus */
#define NB_ESSAIS           2          /* tentatives par trame avant abandon */
#define CODE_115200_BAUDS   7          /* valeur de C0A.01 pour 115200 bauds */

/* Résolution utilisée en mode SIMU (codeur 23 bits) */
#define IMPULSIONS_PAR_TOUR_SIMU 8388608u

/* ============================================================================
 *                 PARAMÈTRES DU VARIATEUR (adresse = 0xGGII pour CGG.II)
 * ========================================================================== */

#define REG_MODE_CONTROLE      0x0000  /* C00.00 : 0 = mode position */
#define REG_IMPULSIONS_TOUR    0x0002  /* C00.02 : U32, impulsions par tour moteur (au redémarrage) */
#define REG_SOURCE_POSITION    0x0300  /* C03.00 : 1 = planification interne */
#define REG_DI5_LOGIQUE        0x0411  /* C04.11 : DI5 a la fonction S-ON par défaut (C04.10 = 1) */
#define REG_DI6_FONCTION       0x0414  /* C04.14 : fonction de DI6 (0 = aucune par défaut) */
#define REG_DI6_LOGIQUE        0x0415  /* C04.15 : logique de DI6 */
#define REG_DEBIT              0x0A01  /* C0A.01 : débit Modbus (au redémarrage) */
#define REG_STOCKAGE_EEPROM    0x0A05  /* C0A.05 : 0 = pas de sauvegarde EEPROM, 1 = sauvegarde */
#define REG_ORDRE_MOTS_32      0x0A06  /* C0A.06 : 0 = mot bas avant mot haut, 1 = l'inverse */
#define REG_PLAN_MODE          0x1100  /* C11.00 : 3 = mode PP */
#define REG_PLAN_TYPE          0x1101  /* C11.01 : 1 = déplacements relatifs */
#define REG_PLAN_GROUPE_DEBUT  0x1103  /* C11.03 : premier groupe exécuté */
#define REG_PLAN_GROUPE_FIN    0x1104  /* C11.04 : dernier groupe exécuté */
#define REG_PLAN_DEPLACEMENT   0x1106  /* C11.06 : I32, déplacement du groupe 1 (impulsions) */
#define REG_PLAN_VITESSE       0x1108  /* C11.08 : U16, vitesse du groupe 1 (tr/min) */
#define REG_PLAN_ACCEL         0x110A  /* C11.0A : U32, temps d'accélération (ms) */
#define REG_PLAN_DECEL         0x110C  /* C11.0C : U32, temps de décélération (ms) */
#define REG_PLAN_ATTENTE       0x110E  /* C11.0E : U32, temps d'attente (ms) */
#define REG_RESOLUTION_CODEUR  0x202A  /* R20.2A : résolution du codeur (bits) */
#define REG_ANGLE_MECANIQUE    0x4009  /* U40.09 : U16, angle mécanique du moteur (0,01°), absolu sur un tour */
#define REG_POSITION_REELLE    0x4016  /* U40.16 : I32, position absolue mesurée (impulsions) */
#define REG_ETAT_SERVO         0x410A  /* U41.0A : 0 pas prêt, 1 prêt, 2 en marche, 3 défaut */

#define FONCTION_DECLENCHEMENT 19      /* FunIN.19 : position planning trigger */
#define DI_NO                  0       /* entrée non câblée => inactive */
#define DI_NC                  1       /* entrée non câblée => ACTIVE */

#define ETAT_PAS_PRET          0
#define ETAT_EN_MARCHE         2
#define ETAT_DEFAUT            3

/* Débits essayés pour trouver le variateur, et code C0A.01 correspondant (indice = code) */
static const DWORD DEBITS_ESSAYES[] = {115200, 9600, 57600, 38400, 19200, 4800, 2400};
static const DWORD DEBIT_PAR_CODE[] = {0, 2400, 4800, 9600, 19200, 38400, 57600, 115200};

/* ============================================================================
 *                         ÉTAT GLOBAL
 * ========================================================================== */

static volatile LONG g_arret_demande = 0;   /* passe à 1 sur Ctrl+C */
static int g_simulation = 0;                /* 1 : port "SIMU", aucune trame envoyée */
static int g_afficher_trames = 1;           /* SIMU : afficher les trames */
static int g_silencieux = 0;                /* 1 : pas de message d'erreur (recherche du débit) */
static int g_mot_haut_dabord = 0;           /* valeur de C0A.06 dans le variateur */
static uint16_t g_simu_angle_meca = 0;      /* SIMU : valeur simulée de U40.09 (0,01°) */

typedef struct {
    double t_s;          /* instant du point (s) */
    double angle_deg;    /* angle du moteur choisi, lu dans le CSV */
    long impulsions;     /* position cible, relative au point de départ */
    int vitesse_rpm;     /* vitesse pour venir du point précédent */
} Point;

typedef struct {
    uint16_t reg;
    int32_t valeur;
    int bits32;          /* 1 : paramètre 32 bits, écrit avec la fonction 0x10 */
    const char *description;
} Parametre;

/* ============================================================================
 *                         OUTILS
 * ========================================================================== */

static int arret_demande(void)
{
    return InterlockedCompareExchange(&g_arret_demande, 0, 0) != 0;
}

static BOOL WINAPI gestionnaire_ctrl_c(DWORD type)
{
    if (type == CTRL_C_EVENT || type == CTRL_BREAK_EVENT) {
        InterlockedExchange(&g_arret_demande, 1);
        return TRUE;     /* le programme continue et coupe le moteur proprement */
    }
    return FALSE;
}

static void afficher_erreur_windows(const char *operation)
{
    fprintf(stderr, "%s (code erreur Windows %lu).\n", operation, (unsigned long)GetLastError());
}

static void afficher_hex(FILE *flux, const uint8_t *data, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        fprintf(flux, "%02X ", data[i]);
    }
    fputc('\n', flux);
}

static double maintenant_ms(void)
{
    static LARGE_INTEGER frequence = {0};
    LARGE_INTEGER compteur;
    if (frequence.QuadPart == 0) {
        QueryPerformanceFrequency(&frequence);
    }
    QueryPerformanceCounter(&compteur);
    return (double)compteur.QuadPart * 1000.0 / (double)frequence.QuadPart;
}

/* Attend jusqu'à l'instant echeance_ms (horloge de maintenant_ms), sauf Ctrl+C. */
static void attendre_jusqua(double echeance_ms)
{
    if (g_simulation) {
        return;
    }
    for (;;) {
        double reste = echeance_ms - maintenant_ms();
        if (reste <= 0.0 || arret_demande()) {
            return;
        }
        if (reste > 20.0) {
            Sleep((DWORD)(reste - 16.0));   /* Sleep a une précision d'environ 15 ms */
        } else {
            Sleep(0);                       /* attente active pour les dernières ms */
        }
    }
}

/* CRC16 Modbus, polynôme 0xA001 (manuel p. 335) */
static uint16_t crc16_modbus(const uint8_t *data, size_t n)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x0001) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

static int demander_confirmation(const char *question)
{
    char reponse[16];
    printf("%s Tapez o puis Entrée pour continuer : ", question);
    fflush(stdout);
    if (fgets(reponse, sizeof(reponse), stdin) == NULL) {
        return 0;
    }
    return reponse[0] == 'o' || reponse[0] == 'O';
}

/* Écrit dans chemin le dossier du .exe suivi de nom. */
static int chemin_a_cote_exe(const char *nom, char *chemin, size_t taille)
{
    DWORD n = GetModuleFileNameA(NULL, chemin, (DWORD)taille);
    if (n == 0 || n >= taille) {
        return 0;
    }
    char *separateur = strrchr(chemin, '\\');
    if (separateur == NULL) {
        return 0;
    }
    separateur[1] = '\0';
    if (strlen(chemin) + strlen(nom) + 1 > taille) {
        return 0;
    }
    strcat(chemin, nom);
    return 1;
}

/* ============================================================================
 *                         PORT SÉRIE
 * ========================================================================== */

/* "COM3" -> "\\.\COM3" (nécessaire à partir de COM10) */
static int chemin_port(const char *nom, char *chemin, size_t taille)
{
    if (_strnicmp(nom, "COM", 3) != 0 || nom[3] == '\0') {
        return 0;
    }
    for (const char *c = nom + 3; *c != '\0'; c++) {
        if (*c < '0' || *c > '9') {
            return 0;
        }
    }
    int n = snprintf(chemin, taille, "\\\\.\\%s", nom);
    return n >= 0 && (size_t)n < taille;
}

/* Affiche les ports COM présents sur le PC (noms de périphériques DOS "COMx"). */
static void lister_ports_com(void)
{
    char *noms = NULL;
    DWORD taille = 16384;
    for (;;) {
        char *nouveau = realloc(noms, taille);
        if (nouveau == NULL) {
            free(noms);
            return;
        }
        noms = nouveau;
        if (QueryDosDeviceA(NULL, noms, taille) != 0) {
            break;
        }
        if (GetLastError() != ERROR_INSUFFICIENT_BUFFER || taille > 4u * 1024u * 1024u) {
            free(noms);
            return;
        }
        taille *= 2;
    }

    int trouve = 0;
    fprintf(stderr, "Ports série présents sur ce PC :");
    for (const char *p = noms; *p != '\0'; p += strlen(p) + 1) {
        if (_strnicmp(p, "COM", 3) == 0 && p[3] >= '0' && p[3] <= '9') {
            fprintf(stderr, " %s", p);
            trouve = 1;
        }
    }
    fprintf(stderr, trouve ? "\n" : " AUCUN\n");
    free(noms);
}

static int regler_debit(HANDLE port, DWORD debit)
{
    DCB reglages;
    memset(&reglages, 0, sizeof(reglages));
    reglages.DCBlength = sizeof(reglages);
    if (!GetCommState(port, &reglages)) {
        afficher_erreur_windows("Erreur GetCommState");
        return 0;
    }
    reglages.BaudRate = debit;
    reglages.ByteSize = 8;
    reglages.Parity = NOPARITY;            /* C0A.02 = 0 : pas de parité, 1 bit de stop */
    reglages.StopBits = ONESTOPBIT;
    reglages.fBinary = TRUE;
    reglages.fParity = FALSE;
    reglages.fOutxCtsFlow = FALSE;
    reglages.fOutxDsrFlow = FALSE;
    reglages.fDtrControl = DTR_CONTROL_DISABLE;
    reglages.fRtsControl = RTS_CONTROL_DISABLE;
    reglages.fOutX = FALSE;
    reglages.fInX = FALSE;
    if (!SetCommState(port, &reglages)) {
        afficher_erreur_windows("Erreur SetCommState");
        return 0;
    }
    PurgeComm(port, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return 1;
}

/* Ouvre le port (sans le débit, réglé ensuite par trouver_variateur). */
static HANDLE ouvrir_port(const char *nom)
{
    char chemin[32];
    if (!chemin_port(nom, chemin, sizeof(chemin))) {
        fprintf(stderr, "Port invalide : COM suivi d'un numéro (ex. COM3), ou SIMU.\n");
        return INVALID_HANDLE_VALUE;
    }

    HANDLE port = CreateFileA(chemin, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                              OPEN_EXISTING, 0, NULL);
    if (port == INVALID_HANDLE_VALUE) {
        DWORD erreur = GetLastError();
        fprintf(stderr, "Impossible d'ouvrir le port %s (code erreur Windows %lu).\n",
                nom, (unsigned long)erreur);
        if (erreur == ERROR_FILE_NOT_FOUND) {
            fprintf(stderr, "=> Ce port n'existe pas : adaptateur USB-RS485 débranché, driver non "
                            "installé (Gestionnaire de périphériques > Ports (COM et LPT)), ou "
                            "l'adaptateur a un autre numéro.\n");
            lister_ports_com();
            fprintf(stderr, "Indiquez le bon port, par exemple :\n"
                            "    .\\Code_C_V5_Trajectoire.exe trajectoire_moteurs.csv 1 COM5\n");
        } else if (erreur == ERROR_ACCESS_DENIED) {
            fprintf(stderr, "=> Le port est déjà utilisé par un autre programme (script Python, "
                            "autre terminal, logiciel du variateur...) : fermez-le.\n");
        }
        return INVALID_HANDLE_VALUE;
    }

    /* ReadFile rend la main dès qu'un octet est arrivé, ou après 20 ms sans rien :
       la boucle de lecture gère elle-même le délai total. */
    COMMTIMEOUTS delais;
    memset(&delais, 0, sizeof(delais));
    delais.ReadIntervalTimeout = MAXDWORD;
    delais.ReadTotalTimeoutMultiplier = MAXDWORD;
    delais.ReadTotalTimeoutConstant = 20;
    delais.WriteTotalTimeoutMultiplier = 10;
    delais.WriteTotalTimeoutConstant = 500;
    if (!SetCommTimeouts(port, &delais)) {
        afficher_erreur_windows("Erreur SetCommTimeouts");
        CloseHandle(port);
        return INVALID_HANDLE_VALUE;
    }
    return port;
}

/* Lit jusqu'à "voulu" octets avant l'instant echeance_ms. Retourne le nombre lu. */
static size_t lire_octets(HANDLE port, uint8_t *buf, size_t voulu, double echeance_ms)
{
    size_t total = 0;
    while (total < voulu && maintenant_ms() < echeance_ms) {
        DWORD lus = 0;
        if (!ReadFile(port, buf + total, (DWORD)(voulu - total), &lus, NULL)) {
            afficher_erreur_windows("Erreur ReadFile");
            break;
        }
        total += lus;
    }
    return total;
}

/* ============================================================================
 *                         MODBUS RTU
 * ========================================================================== */

/* Réponse fabriquée en mode SIMU : écho pour une écriture ; pour une lecture,
   état "en marche", IMPULSIONS_PAR_TOUR_SIMU pour C00.02, g_simu_angle_meca pour
   U40.09 et zéro ailleurs. */
static void simuler_reponse(const uint8_t *req, uint8_t *rep, size_t len_rep)
{
    memset(rep, 0, len_rep);
    if (req[1] != 0x03) {
        memcpy(rep, req, 6);
        return;
    }
    uint16_t reg = (uint16_t)((req[2] << 8) | req[3]);
    rep[0] = req[0];
    rep[1] = req[1];
    rep[2] = (uint8_t)(len_rep - 5);
    if (reg == REG_ETAT_SERVO) {
        rep[4] = ETAT_EN_MARCHE;
    } else if (reg == REG_ANGLE_MECANIQUE) {
        rep[3] = (uint8_t)(g_simu_angle_meca >> 8);
        rep[4] = (uint8_t)g_simu_angle_meca;
    } else if (reg == REG_IMPULSIONS_TOUR && len_rep >= 9) {
        uint32_t v = IMPULSIONS_PAR_TOUR_SIMU;           /* mot bas puis mot haut */
        rep[3] = (uint8_t)(v >> 8);
        rep[4] = (uint8_t)v;
        rep[5] = (uint8_t)(v >> 24);
        rep[6] = (uint8_t)(v >> 16);
    }
}

/* Ajoute le CRC à req (qui doit avoir 2 octets de place en plus), envoie la trame
   et lit une réponse de len_rep octets. Vérifie CRC, adresse et code fonction.
   Retourne 1 si la réponse est valide. */
static int transaction(HANDLE port, uint8_t *req, size_t len_sans_crc,
                       uint8_t *rep, size_t len_rep)
{
    uint16_t crc = crc16_modbus(req, len_sans_crc);
    req[len_sans_crc] = (uint8_t)(crc & 0xFF);        /* CRC : octet bas d'abord */
    req[len_sans_crc + 1] = (uint8_t)(crc >> 8);
    size_t len_req = len_sans_crc + 2;

    if (g_simulation) {
        if (g_afficher_trames) {
            printf("        [SIMU] -> ");
            afficher_hex(stdout, req, len_req);
        }
        simuler_reponse(req, rep, len_rep);
        return 1;
    }

    int nb_essais = g_silencieux ? 1 : NB_ESSAIS;
    double timeout = g_silencieux ? 150.0 : TIMEOUT_REPONSE_MS;
    for (int essai = 1; essai <= nb_essais; essai++) {
        PurgeComm(port, PURGE_RXCLEAR | PURGE_TXCLEAR);

        DWORD ecrits = 0;
        if (!WriteFile(port, req, (DWORD)len_req, &ecrits, NULL) || ecrits != len_req) {
            afficher_erreur_windows("Erreur WriteFile");
            continue;
        }

        double echeance = maintenant_ms() + timeout;
        size_t n = lire_octets(port, rep, 2, echeance);
        if (n < 2) {
            if (!g_silencieux) {
                fprintf(stderr, "  Pas de réponse du variateur (essai %d/%d).\n", essai, nb_essais);
            }
            continue;
        }

        if (rep[1] & 0x80) {
            /* Réponse d'erreur : la trame est refusée, inutile de la renvoyer */
            n += lire_octets(port, rep + 2, 6, maintenant_ms() + 50.0);
            if (!g_silencieux) {
                fprintf(stderr, "  Le variateur refuse la trame (réponse d'erreur) : ");
                afficher_hex(stderr, rep, n);
                fprintf(stderr, "  Trame envoyée : ");
                afficher_hex(stderr, req, len_req);
            }
            return 0;
        }

        n += lire_octets(port, rep + 2, len_rep - 2, echeance);
        const char *probleme = NULL;
        if (n < len_rep) {
            probleme = "Réponse incomplète";
        } else if (crc16_modbus(rep, len_rep - 2) !=
                   (uint16_t)(rep[len_rep - 2] | (rep[len_rep - 1] << 8))) {
            probleme = "Réponse avec CRC invalide";
        } else if (rep[0] != req[0] || rep[1] != req[1]) {
            probleme = "Réponse d'un autre esclave ou d'une autre fonction";
        }
        if (probleme == NULL) {
            return 1;
        }
        if (!g_silencieux) {
            fprintf(stderr, "  %s (essai %d/%d).\n", probleme, essai, nb_essais);
        }
    }
    return 0;
}

/* Fonction 0x06 : écrit un paramètre 16 bits */
static int ecrire_16(HANDLE port, uint16_t reg, uint16_t valeur)
{
    uint8_t req[8], rep[8];
    req[0] = ADRESSE_ESCLAVE;
    req[1] = 0x06;
    req[2] = (uint8_t)(reg >> 8);
    req[3] = (uint8_t)(reg & 0xFF);
    req[4] = (uint8_t)(valeur >> 8);
    req[5] = (uint8_t)(valeur & 0xFF);
    if (!transaction(port, req, 6, rep, sizeof(rep))) {
        return 0;
    }
    if (memcmp(rep, req, 6) != 0) {             /* la réponse doit répéter la requête */
        fprintf(stderr, "  Écriture non confirmée par le variateur.\n");
        return 0;
    }
    if (reg == REG_ORDRE_MOTS_32) {
        g_mot_haut_dabord = (valeur == 1);       /* les trames 32 bits suivantes en dépendent */
    }
    return 1;
}

/* Fonction 0x10 : écrit un paramètre 32 bits (2 registres, dans l'ordre fixé par C0A.06).
   Le manuel interdit d'écrire un paramètre 32 bits avec 0x06. */
static int ecrire_32(HANDLE port, uint16_t reg, uint32_t valeur)
{
    uint16_t bas = (uint16_t)(valeur & 0xFFFF);
    uint16_t haut = (uint16_t)(valeur >> 16);
    uint16_t premier = g_mot_haut_dabord ? haut : bas;
    uint16_t second = g_mot_haut_dabord ? bas : haut;
    uint8_t req[13], rep[8];
    req[0] = ADRESSE_ESCLAVE;
    req[1] = 0x10;
    req[2] = (uint8_t)(reg >> 8);
    req[3] = (uint8_t)(reg & 0xFF);
    req[4] = 0x00;                    /* nombre de registres : 2 */
    req[5] = 0x02;
    req[6] = 0x04;                    /* nombre d'octets de données : 4 */
    req[7] = (uint8_t)(premier >> 8);
    req[8] = (uint8_t)(premier & 0xFF);
    req[9] = (uint8_t)(second >> 8);
    req[10] = (uint8_t)(second & 0xFF);
    if (!transaction(port, req, 11, rep, sizeof(rep))) {
        return 0;
    }
    if (memcmp(rep, req, 6) != 0) {
        fprintf(stderr, "  Écriture 32 bits non confirmée par le variateur.\n");
        return 0;
    }
    return 1;
}

/* Fonction 0x03 : lit nb registres (nb <= 4) */
static int lire_registres(HANDLE port, uint16_t reg, uint16_t nb, uint16_t *mots)
{
    uint8_t req[8], rep[5 + 2 * 4];
    if (nb < 1 || nb > 4) {
        return 0;
    }
    req[0] = ADRESSE_ESCLAVE;
    req[1] = 0x03;
    req[2] = (uint8_t)(reg >> 8);
    req[3] = (uint8_t)(reg & 0xFF);
    req[4] = 0x00;
    req[5] = (uint8_t)nb;
    if (!transaction(port, req, 6, rep, 5 + 2u * nb)) {
        return 0;
    }
    if (rep[2] != 2 * nb) {
        if (!g_silencieux) {
            fprintf(stderr, "  Lecture : nombre d'octets inattendu (%u).\n", rep[2]);
        }
        return 0;
    }
    for (uint16_t i = 0; i < nb; i++) {
        mots[i] = (uint16_t)((rep[3 + 2 * i] << 8) | rep[4 + 2 * i]);
    }
    return 1;
}

static int lire_16(HANDLE port, uint16_t reg, uint16_t *valeur)
{
    return lire_registres(port, reg, 1, valeur);
}

static int lire_32(HANDLE port, uint16_t reg, uint32_t *valeur)
{
    uint16_t mots[2];
    if (!lire_registres(port, reg, 2, mots)) {
        return 0;
    }
    uint16_t bas = g_mot_haut_dabord ? mots[1] : mots[0];
    uint16_t haut = g_mot_haut_dabord ? mots[0] : mots[1];
    *valeur = ((uint32_t)haut << 16) | bas;
    return 1;
}

/* Essaie les débits usuels jusqu'à ce que le variateur réponde (lecture de C0A.06,
   qui donne aussi l'ordre des mots 32 bits). Retourne le débit trouvé, ou 0. */
static DWORD trouver_variateur(HANDLE port)
{
    size_t nb = sizeof(DEBITS_ESSAYES) / sizeof(DEBITS_ESSAYES[0]);
    uint16_t ordre = 0;
    g_silencieux = 1;
    for (size_t i = 0; i < nb; i++) {
        if (regler_debit(port, DEBITS_ESSAYES[i]) && lire_16(port, REG_ORDRE_MOTS_32, &ordre)) {
            g_silencieux = 0;
            g_mot_haut_dabord = (ordre == 1);
            return DEBITS_ESSAYES[i];
        }
    }
    g_silencieux = 0;
    fprintf(stderr, "Le variateur ne répond à aucun débit (2400 à 115200 bauds). Vérifiez le câble "
                    "RS485 (A/B inversés ?), l'alimentation du variateur et son adresse "
                    "(C0A.00 = %d).\n", ADRESSE_ESCLAVE);
    return 0;
}

/* Temps maximal d'une trame courte (lecture d'un registre), mesuré sur 5 essais. */
static double mesurer_trame_ms(HANDLE port)
{
    uint16_t valeur;
    double t_max = 0.0;
    for (int i = 0; i < 6; i++) {
        double debut = maintenant_ms();
        if (!lire_16(port, REG_ORDRE_MOTS_32, &valeur)) {
            return -1.0;
        }
        if (i > 0) {                      /* la première trame peut être plus lente */
            t_max = fmax(t_max, maintenant_ms() - debut);
        }
    }
    return t_max;
}

/* ============================================================================
 *                         TRAJECTOIRE
 * ========================================================================== */

/* Lit le CSV "t_s,theta1_deg,theta2_deg,theta3_deg,theta4_deg" et garde la colonne
   du moteur choisi. Retourne un tableau alloué (à libérer) ou NULL. */
static Point *lire_csv(const char *chemin, int moteur, size_t *nb)
{
    FILE *f = fopen(chemin, "r");
    if (f == NULL) {
        fprintf(stderr, "Impossible d'ouvrir %s. Lancez d'abord exporter_trajectoire.py.\n", chemin);
        return NULL;
    }

    Point *points = NULL;
    size_t n = 0, capacite = 0;
    unsigned num_ligne = 0;
    char ligne[512];

    while (fgets(ligne, sizeof(ligne), f) != NULL) {
        num_ligne++;
        for (char *c = ligne; *c != '\0'; c++) {
            if (*c == ';') {
                *c = ',';                 /* accepte aussi le séparateur d'Excel */
            }
        }
        double v[5];
        if (sscanf(ligne, "%lf,%lf,%lf,%lf,%lf", &v[0], &v[1], &v[2], &v[3], &v[4]) != 5) {
            if (strspn(ligne, " \t\r\n") == strlen(ligne) || n == 0) {
                continue;                 /* ligne vide, ou en-tête avant les données */
            }
            fprintf(stderr, "%s, ligne %u : illisible.\n", chemin, num_ligne);
            goto erreur;
        }
        if (!isfinite(v[0]) || !isfinite(v[moteur])) {
            fprintf(stderr, "%s, ligne %u : valeur non finie (NaN).\n", chemin, num_ligne);
            goto erreur;
        }
        if (n > 0 && v[0] <= points[n - 1].t_s) {
            fprintf(stderr, "%s, ligne %u : le temps doit être strictement croissant.\n",
                    chemin, num_ligne);
            goto erreur;
        }
        if (n == capacite) {
            capacite = capacite ? 2 * capacite : 1024;
            Point *nouveau = realloc(points, capacite * sizeof(Point));
            if (nouveau == NULL) {
                fprintf(stderr, "Mémoire insuffisante.\n");
                goto erreur;
            }
            points = nouveau;
        }
        points[n].t_s = v[0];
        points[n].angle_deg = v[moteur];
        points[n].impulsions = 0;
        points[n].vitesse_rpm = 0;
        n++;
    }
    fclose(f);

    if (n == 0) {
        fprintf(stderr, "%s ne contient aucun point.\n", chemin);
        free(points);
        return NULL;
    }
    *nb = n;
    return points;

erreur:
    fclose(f);
    free(points);
    return NULL;
}

/* Refuse une trajectoire qui sort de la plage [ANGLE_MIN_DEG ; ANGLE_MAX_DEG]. */
static int verifier_plage(const Point *points, size_t n)
{
    double mini = points[0].angle_deg, maxi = points[0].angle_deg;
    for (size_t i = 1; i < n; i++) {
        mini = fmin(mini, points[i].angle_deg);
        maxi = fmax(maxi, points[i].angle_deg);
    }
    if (mini < ANGLE_MIN_DEG - 1e-9 || maxi > ANGLE_MAX_DEG + 1e-9) {
        fprintf(stderr, "Trajectoire refusée : l'angle va de %.2f° à %.2f°, hors de la plage "
                        "autorisée [%.0f° ; %.0f°] (ANGLE_MIN_DEG / ANGLE_MAX_DEG).\n",
                mini, maxi, ANGLE_MIN_DEG, ANGLE_MAX_DEG);
        return 0;
    }
    return 1;
}

/* Rééchantillonne la trajectoire au pas periode_s (interpolation linéaire). Tous les
   segments durent au moins periode_s : le dernier absorbe le reste et finit
   exactement sur le dernier point du CSV. Retourne un tableau alloué ou NULL. */
static Point *reechantillonner(const Point *src, size_t n, double periode_s, size_t *n_out)
{
    double duree = src[n - 1].t_s - src[0].t_s;
    size_t nb_segments = (size_t)floor(duree / periode_s + 1e-9);
    if (nb_segments == 0 && duree > 0.0) {
        nb_segments = 1;
    }
    size_t m = nb_segments + 1;
    Point *dst = calloc(m, sizeof(Point));
    if (dst == NULL) {
        fprintf(stderr, "Mémoire insuffisante.\n");
        return NULL;
    }

    size_t j = 0;
    for (size_t k = 0; k < m; k++) {
        double t = (k == m - 1) ? src[n - 1].t_s : src[0].t_s + (double)k * periode_s;
        while (j + 1 < n && src[j + 1].t_s < t) {
            j++;
        }
        double angle = src[j].angle_deg;
        if (j + 1 < n) {
            double a = (t - src[j].t_s) / (src[j + 1].t_s - src[j].t_s);
            a = fmin(1.0, fmax(0.0, a));
            angle += a * (src[j + 1].angle_deg - src[j].angle_deg);
        }
        dst[k].t_s = t;
        dst[k].angle_deg = angle;
    }
    *n_out = m;
    return dst;
}

/* Calcule les impulsions (360° = impulsions_par_tour exactement) et les vitesses,
   puis vérifie la vitesse maximale. */
static int preparer(Point *points, size_t n, uint32_t impulsions_par_tour)
{
    for (size_t i = 0; i < n; i++) {
        double tours = SENS_MOTEUR * (points[i].angle_deg - points[0].angle_deg) / 360.0;
        points[i].impulsions = lround(tours * (double)impulsions_par_tour);

        if (i == 0) {
            continue;
        }
        long pas = labs(points[i].impulsions - points[i - 1].impulsions);
        double dt = points[i].t_s - points[i - 1].t_s;
        double rpm = (double)pas / (double)impulsions_par_tour / dt * 60.0;
        int vitesse = (pas == 0) ? 0 : (int)ceil(rpm * MARGE_VITESSE);
        if (pas != 0 && vitesse < 1) {
            vitesse = 1;
        }
        if (vitesse > VITESSE_MAX_RPM) {
            fprintf(stderr, "Trajectoire refusée : le point %u (t = %.2f s) demande %d tr/min "
                            "(limite VITESSE_MAX_RPM = %d).\n",
                    (unsigned)i, points[i].t_s, vitesse, VITESSE_MAX_RPM);
            return 0;
        }
        points[i].vitesse_rpm = vitesse;
    }
    return 1;
}

static void afficher_resume(const char *fichier, int moteur, size_t n_lus, const Point *points,
                            size_t n, uint32_t impulsions_par_tour, int periode_ms,
                            double angle_actuel)
{
    double mini = points[0].angle_deg, maxi = points[0].angle_deg;
    int vitesse_max = 0;
    for (size_t i = 0; i < n; i++) {
        mini = fmin(mini, points[i].angle_deg);
        maxi = fmax(maxi, points[i].angle_deg);
        if (points[i].vitesse_rpm > vitesse_max) {
            vitesse_max = points[i].vitesse_rpm;
        }
    }
    printf("\nTrajectoire : %s, moteur %d (colonne theta%d)\n", fichier, moteur, moteur);
    printf("  Points CSV / envoyés : %u / %u (un toutes les %d ms)\n",
           (unsigned)n_lus, (unsigned)n, periode_ms);
    printf("  Durée                : %.2f s\n", points[n - 1].t_s - points[0].t_s);
    printf("  Angle                : %.3f° -> %.3f° (%+ld impulsions)\n",
           points[0].angle_deg, points[n - 1].angle_deg, points[n - 1].impulsions);
    printf("  Angles parcourus     : de %.2f° à %.2f° (plage autorisée [%.0f° ; %.0f°])\n",
           mini, maxi, ANGLE_MIN_DEG, ANGLE_MAX_DEG);
    printf("  Position actuelle    : %.2f° (codeur U40.09) -> rejoint le départ à %d tr/min\n",
           angle_actuel, VITESSE_APPROCHE_RPM);
    printf("  Vitesse max envoyée  : %d tr/min (limite %d)\n", vitesse_max, VITESSE_MAX_RPM);
    printf("  Résolution           : %lu impulsions/tour (C00.02) = %.7f° par impulsion\n\n",
           (unsigned long)impulsions_par_tour, 360.0 / (double)impulsions_par_tour);
}

/* ============================================================================
 *                         PILOTAGE DU VARIATEUR
 * ========================================================================== */

/* Ordre important : le moteur est coupé AVANT tout, et C0A.05 = 0 est écrit avant
   les autres réglages pour qu'aucun ne soit sauvegardé en EEPROM. */
static const Parametre CONFIGURATION[] = {
    {REG_DI5_LOGIQUE,       DI_NO, 0, "C04.11 = 0  : S-ON inactif (moteur libre) pendant la configuration"},
    {REG_DI6_LOGIQUE,       DI_NO, 0, "C04.15 = 0  : DI6 (déclenchement) au repos"},
    {REG_STOCKAGE_EEPROM,   0,     0, "C0A.05 = 0  : écritures Modbus non sauvegardées en EEPROM"},
    {REG_ORDRE_MOTS_32,     0,     0, "C0A.06 = 0  : paramètres 32 bits, mot bas d'abord"},
    {REG_MODE_CONTROLE,     0,     0, "C00.00 = 0  : mode position"},
    {REG_SOURCE_POSITION,   1,     0, "C03.00 = 1  : consigne = planification interne"},
    {REG_PLAN_MODE,         3,     0, "C11.00 = 3  : mode PP (un déplacement par front)"},
    {REG_PLAN_TYPE,         1,     0, "C11.01 = 1  : déplacements relatifs"},
    {REG_PLAN_GROUPE_DEBUT, 1,     0, "C11.03 = 1  : groupe de départ 1"},
    {REG_PLAN_GROUPE_FIN,   1,     0, "C11.04 = 1  : groupe de fin 1"},
    {REG_PLAN_DEPLACEMENT,  0,     1, "C11.06 = 0  : aucun déplacement en attente"},
    {REG_PLAN_ACCEL, TEMPS_ACCEL_MS, 1, "C11.0A      : temps d'accélération (ms)"},
    {REG_PLAN_DECEL, TEMPS_ACCEL_MS, 1, "C11.0C      : temps de décélération (ms)"},
    {REG_PLAN_ATTENTE,      0,     1, "C11.0E = 0  : pas d'attente après un déplacement"},
    {REG_DI6_FONCTION, FONCTION_DECLENCHEMENT, 0, "C04.14 = 19 : DI6 = déclenchement de la planification"},
};

static int configurer(HANDLE port)
{
    size_t nb = sizeof(CONFIGURATION) / sizeof(CONFIGURATION[0]);
    printf("Configuration du variateur :\n");
    for (size_t i = 0; i < nb; i++) {
        const Parametre *p = &CONFIGURATION[i];
        printf("  [%2u/%u] %s\n", (unsigned)(i + 1), (unsigned)nb, p->description);
        int ok = p->bits32 ? ecrire_32(port, p->reg, (uint32_t)p->valeur)
                           : ecrire_16(port, p->reg, (uint16_t)p->valeur);
        if (!ok) {
            fprintf(stderr, "Échec de la configuration : arrêt.\n");
            return 0;
        }
    }
    return 1;
}

static int mettre_sous_couple(HANDLE port)
{
    uint16_t etat = 0;
    if (!lire_16(port, REG_ETAT_SERVO, &etat)) {
        return 0;
    }
    if (etat == ETAT_DEFAUT) {
        fprintf(stderr, "Le variateur est en défaut : lisez le code sur son afficheur, "
                        "corrigez puis faites un reset.\n");
        return 0;
    }
    if (etat == ETAT_PAS_PRET) {
        fprintf(stderr, "Le variateur n'est pas prêt (alimentation de puissance coupée ?).\n");
        return 0;
    }

    printf("\nMise sous couple (C04.11 = 1 : DI5/S-ON actif)...\n");
    if (!ecrire_16(port, REG_DI5_LOGIQUE, DI_NC)) {
        return 0;
    }
    attendre_jusqua(maintenant_ms() + 500.0);

    if (!lire_16(port, REG_ETAT_SERVO, &etat)) {
        return 0;
    }
    if (etat == ETAT_DEFAUT) {
        fprintf(stderr, "Le variateur s'est mis en défaut à la mise sous couple.\n");
        return 0;
    }
    if (etat != ETAT_EN_MARCHE) {
        fprintf(stderr, "Attention : état du servo = %u au lieu de 2 (en marche). Le moteur ne "
                        "bougera pas si S-ON est inactif : vérifiez que DI5 a la fonction S-ON "
                        "(C04.10 = 1).\n", etat);
    }
    return 1;
}

/* Coupe le moteur. Toujours appelée après une tentative de mise sous couple. */
static void couper_moteur(HANDLE port)
{
    printf("\nCoupure du moteur (S-OFF)...\n");
    g_afficher_trames = 1;
    int ok_di6 = ecrire_16(port, REG_DI6_LOGIQUE, DI_NO);
    int ok_son = ecrire_16(port, REG_DI5_LOGIQUE, DI_NO);
    if (!ok_son) {
        fprintf(stderr, "ERREUR CRITIQUE : coupure non confirmée. "
                        "Utilisez l'arrêt d'urgence matériel !\n");
    } else if (!ok_di6) {
        fprintf(stderr, "Moteur coupé, mais DI6 n'a pas été remise au repos.\n");
    } else {
        printf("Moteur coupé.\n");
    }
}

/* Front sur DI6 : NC rend l'entrée active (le déplacement part), NO la remet au repos */
static int declencher(HANDLE port)
{
    return ecrire_16(port, REG_DI6_LOGIQUE, DI_NC) && ecrire_16(port, REG_DI6_LOGIQUE, DI_NO);
}

/* Écart signé fin - debut entre deux lectures de U40.16 (juste même si le compteur déborde). */
static long ecart_impulsions(uint32_t fin, uint32_t debut)
{
    return (long)(int32_t)(fin - debut);
}

/* Angle du bras (repère de trajectoires4Axes.py) pour l'angle mécanique meca_deg du
   moteur. Le codeur ne connaît l'angle qu'à un tour près : parmi les angles
   équivalents, garde ceux de la plage autorisée (à TOLERANCE_CODEUR_DEG près) et prend
   le plus proche de proche_de. Retourne 0 si aucun n'est dans la plage (*angle reçoit
   alors l'angle équivalent pris dans [ANGLE_MIN_DEG ; ANGLE_MIN_DEG + 360°[). */
static int angle_depuis_codeur(double meca_deg, double zero_deg, double proche_de, double *angle)
{
    double base = fmod(SENS_MOTEUR * (meca_deg - zero_deg), 360.0);
    int trouve = 0;
    for (int k = -2; k <= 2; k++) {
        double a = base + 360.0 * k;
        if (a < ANGLE_MIN_DEG - TOLERANCE_CODEUR_DEG || a > ANGLE_MAX_DEG + TOLERANCE_CODEUR_DEG) {
            continue;
        }
        if (!trouve || fabs(a - proche_de) < fabs(*angle - proche_de)) {
            *angle = a;
            trouve = 1;
        }
    }
    if (!trouve) {
        *angle = ANGLE_MIN_DEG + fmod(fmod(base - ANGLE_MIN_DEG, 360.0) + 360.0, 360.0);
    }
    return trouve;
}

/* Lit l'angle mécanique U40.09 et en déduit l'angle du bras (voir angle_depuis_codeur). */
static int lire_angle_bras(HANDLE port, double zero_deg, double proche_de, double *angle)
{
    uint16_t centiemes = 0;
    if (!lire_16(port, REG_ANGLE_MECANIQUE, &centiemes)) {
        return 0;
    }
    if (centiemes > 36000) {
        fprintf(stderr, "Angle mécanique illisible (U40.09 = %u).\n", centiemes);
        return 0;
    }
    if (!angle_depuis_codeur(centiemes / 100.0, zero_deg, proche_de, angle)) {
        fprintf(stderr, "Le codeur place le bras à %.2f°, hors de la plage autorisée [%.0f° ; %.0f°].\n"
                        "=> Le zéro est faux (refaites --zero), ou le bras est vraiment hors de la "
                        "plage : ramenez-le à la main, moteur coupé.\n",
                *angle, ANGLE_MIN_DEG, ANGLE_MAX_DEG);
        return 0;
    }
    return 1;
}

/* Amène le moteur de *angle (mesuré) à cible à VITESSE_APPROCHE_RPM, attend la fin du
   déplacement, puis vérifie l'arrivée avec le codeur. *angle devient l'angle mesuré. */
static int aller_a(HANDLE port, double *angle, double cible, double zero_deg,
                   uint32_t impulsions_par_tour)
{
    long pas = lround(SENS_MOTEUR * (cible - *angle) / 360.0 * (double)impulsions_par_tour);
    if (pas != 0) {
        uint32_t avant = 0;
        if (!lire_32(port, REG_POSITION_REELLE, &avant) ||
            !ecrire_32(port, REG_PLAN_DEPLACEMENT, (uint32_t)(int32_t)pas) ||
            !ecrire_16(port, REG_PLAN_VITESSE, VITESSE_APPROCHE_RPM) ||
            !declencher(port)) {
            return 0;
        }
        double duree_ms = fabs(cible - *angle) / (VITESSE_APPROCHE_RPM * 6.0) * 1000.0;
        double echeance = maintenant_ms() + 2.0 * duree_ms + 2000.0;
        long tolerance = lround(TOLERANCE_FIN_DEG / 360.0 * (double)impulsions_par_tour);
        while (!g_simulation) {
            attendre_jusqua(maintenant_ms() + 50.0);
            if (arret_demande()) {
                printf("Ctrl+C : arrêt demandé.\n");
                return 0;
            }
            uint32_t position = 0;
            if (!lire_32(port, REG_POSITION_REELLE, &position)) {
                return 0;
            }
            long fait = ecart_impulsions(position, avant);
            if (labs(fait - pas) <= tolerance) {
                attendre_jusqua(maintenant_ms() + 200.0);   /* stabilisation */
                break;
            }
            if (maintenant_ms() > echeance) {
                fprintf(stderr, "Le moteur n'a pas atteint %.2f° à temps (%+ld impulsions faites "
                                "sur %+ld).\n", cible, fait, pas);
                return 0;
            }
        }
    }
    if (g_simulation) {
        *angle = cible;
        return 1;
    }

    double mesure = 0.0;
    if (!lire_angle_bras(port, zero_deg, cible, &mesure)) {
        return 0;
    }
    if (fabs(mesure - cible) > TOLERANCE_CODEUR_DEG) {
        fprintf(stderr, "Incohérence : le codeur indique %.2f° au lieu de %.2f°. Moteur arrêté par "
                        "sécurité.\n=> Le codeur tourne en sens inverse des impulsions (C00.01 "
                        "modifié ?), C00.02 est faux, ou le zéro est faux (refaites --zero).\n",
                mesure, cible);
        return 0;
    }
    *angle = mesure;
    return 1;
}

/* Amène le moteur au départ de la trajectoire. Un premier déplacement de SONDE_DEG (vers
   le départ, ou vers l'intérieur de la plage) vérifie que le codeur tourne dans le sens
   des impulsions : sinon tout s'arrête avant que le moteur puisse sortir de la plage. */
static int rejoindre_depart(HANDLE port, double *angle, double depart, double zero_deg,
                            uint32_t impulsions_par_tour)
{
    double ecart = depart - *angle;
    double sonde;
    if (fabs(ecart) >= SONDE_DEG) {
        sonde = *angle + copysign(SONDE_DEG, ecart);
    } else if (*angle + SONDE_DEG <= ANGLE_MAX_DEG) {
        sonde = *angle + SONDE_DEG;
    } else {
        sonde = *angle - SONDE_DEG;
    }
    printf("\nVérification du sens du codeur : %.2f° -> %.2f° (%d tr/min)...\n",
           *angle, sonde, VITESSE_APPROCHE_RPM);
    if (!aller_a(port, angle, sonde, zero_deg, impulsions_par_tour)) {
        return 0;
    }
    printf("Mise en position de départ : %.2f° -> %.3f°...\n", *angle, depart);
    if (!aller_a(port, angle, depart, zero_deg, impulsions_par_tour)) {
        return 0;
    }
    printf("Moteur au départ (codeur : %.2f°).\n", *angle);
    return 1;
}

/* Envoie les points un par un. Le déplacement vers le point i est lancé à
   l'instant du point i-1 : le moteur arrive au point i vers l'instant du point i. */
static int executer(HANDLE port, const Point *points, size_t n, int periode_ms)
{
    /* Afficher chaque point ralentirait la boucle : environ 4 lignes par seconde */
    size_t pas_affichage = (periode_ms >= 250) ? 1 : (size_t)(250 / periode_ms);
    printf("\nDépart de la trajectoire (Ctrl+C pour arrêter)\n");
    double t0 = maintenant_ms();
    int derniere_vitesse = -1;
    double comm_max_ms = 0.0, retard_max_ms = 0.0;
    unsigned nb_trop_lents = 0;

    for (size_t i = 1; i < n; i++) {
        double echeance = t0 + (points[i - 1].t_s - points[0].t_s) * 1000.0;
        attendre_jusqua(echeance);
        if (arret_demande()) {
            printf("Ctrl+C : arrêt demandé.\n");
            return 0;
        }
        double debut = maintenant_ms();
        if (!g_simulation) {
            retard_max_ms = fmax(retard_max_ms, debut - echeance);
        }
        if (g_simulation && i == 3) {
            g_afficher_trames = 0;
            printf("        [SIMU] (trames suivantes masquées)\n");
        }

        long pas = points[i].impulsions - points[i - 1].impulsions;
        if (i % pas_affichage == 0 || i == n - 1 || i <= 2) {
            printf("  Point %4u/%u  t = %6.2f s  angle = %9.4f°  pas = %+8ld imp  vitesse = %2d tr/min\n",
                   (unsigned)i, (unsigned)(n - 1), points[i].t_s, points[i].angle_deg,
                   pas, points[i].vitesse_rpm);
        }
        if (pas == 0) {
            continue;                     /* le moteur reste immobile pendant ce segment */
        }

        if (!ecrire_32(port, REG_PLAN_DEPLACEMENT, (uint32_t)(int32_t)pas)) {
            return 0;
        }
        if (points[i].vitesse_rpm != derniere_vitesse) {
            if (!ecrire_16(port, REG_PLAN_VITESSE, (uint16_t)points[i].vitesse_rpm)) {
                return 0;
            }
            derniere_vitesse = points[i].vitesse_rpm;
        }
        if (!declencher(port)) {
            return 0;
        }
        double duree_comm = maintenant_ms() - debut;
        comm_max_ms = fmax(comm_max_ms, duree_comm);
        if (duree_comm > periode_ms) {
            nb_trop_lents++;
        }
    }

    /* Laisse le dernier déplacement se terminer */
    attendre_jusqua(t0 + (points[n - 1].t_s - points[0].t_s) * 1000.0 + 500.0);

    if (!g_simulation) {
        printf("\nModbus : %.1f ms au plus par point (période %d ms), retard max %.1f ms, "
               "%u point(s) plus lent(s) que la période.\n",
               comm_max_ms, periode_ms, retard_max_ms, nb_trop_lents);
        if (nb_trop_lents > 0) {
            printf("Attention : la communication n'a pas toujours suivi la cadence. Augmentez "
                   "PERIODE_MIN_MS, ou réduisez la latence de l'adaptateur USB (FTDI : "
                   "Gestionnaire de périphériques > port > Avancé > Latence = 1 ms).\n");
        }
    }
    return arret_demande() ? 0 : 1;
}

/* Compare la position mesurée à la position attendue en fin de trajectoire. */
static void verifier_position(HANDLE port, uint32_t depart, long attendu, uint32_t impulsions_par_tour,
                              double angle_attendu, double zero_deg)
{
    if (g_simulation) {
        printf("\n(SIMU : position finale non mesurée)\n");
        return;
    }
    uint32_t fin = 0;
    if (!lire_32(port, REG_POSITION_REELLE, &fin)) {
        return;
    }
    double angle = 0.0;
    if (lire_angle_bras(port, zero_deg, angle_attendu, &angle)) {
        printf("\nAngle final (codeur U40.09) : %.2f° (attendu %.3f°)\n", angle, angle_attendu);
    }
    long mesure = ecart_impulsions(fin, depart);
    long erreur = mesure - attendu;
    double deg_par_impulsion = 360.0 / (double)impulsions_par_tour;
    printf("\nPosition finale : %+ld impulsions depuis le départ (attendu %+ld), "
           "erreur %+ld impulsions = %+.4f°\n",
           mesure, attendu, erreur, (double)erreur * deg_par_impulsion);
    if (fabs((double)erreur * deg_par_impulsion) > TOLERANCE_FIN_DEG) {
        printf("Attention : écart supérieur à %.2f°. Un déclenchement a peut-être été manqué, ou "
               "le moteur n'a pas fini ses déplacements (augmentez MARGE_VITESSE ou "
               "PERIODE_MIN_MS).\n", TOLERANCE_FIN_DEG);
    }
}

static void attendre_touche_avant_coupure(void)
{
    if (g_simulation) {
        return;
    }
    printf("\nLe moteur maintient sa position. Appuyez sur une touche pour le couper (S-OFF).\n"
           "(Un bras sans frein tombera à la coupure.)\n");
    while (!_kbhit() && !arret_demande()) {
        Sleep(50);
    }
    if (_kbhit()) {
        _getch();
    }
}

/* ============================================================================
 *            RÉGLAGE UNIQUE : PLEINE RÉSOLUTION ET 115200 BAUDS
 * ========================================================================== */

static int configurer_variateur_une_fois(const char *nom_port)
{
    HANDLE port = ouvrir_port(nom_port);
    if (port == INVALID_HANDLE_VALUE) {
        return 0;
    }
    DWORD debit = trouver_variateur(port);
    uint16_t bits = 0, code_debit = 0;
    uint32_t impulsions = 0;
    int ok = debit != 0 &&
             lire_16(port, REG_RESOLUTION_CODEUR, &bits) &&
             lire_16(port, REG_DEBIT, &code_debit) &&
             lire_32(port, REG_IMPULSIONS_TOUR, &impulsions);
    if (!ok) {
        CloseHandle(port);
        return 0;
    }
    if (bits < 16 || bits > 31) {
        fprintf(stderr, "Résolution du codeur illisible (R20.2A = %u bits).\n", bits);
        CloseHandle(port);
        return 0;
    }

    uint32_t impulsions_cible = 1u << bits;
    printf("Variateur trouvé sur %s à %lu bauds.\n", nom_port, (unsigned long)debit);
    printf("  Codeur (R20.2A)      : %u bits = %lu points par tour\n",
           bits, (unsigned long)impulsions_cible);
    printf("  C00.02 actuel        : %lu impulsions/tour (%.7f° par impulsion)\n",
           (unsigned long)impulsions, impulsions ? 360.0 / (double)impulsions : 0.0);
    printf("  C0A.01 actuel        : %u (%lu bauds au prochain démarrage)\n", code_debit,
           (unsigned long)(code_debit < 8 ? DEBIT_PAR_CODE[code_debit] : 0));

    if (impulsions == impulsions_cible && code_debit == CODE_115200_BAUDS) {
        printf("\nRien à changer : pleine résolution et 115200 bauds sont déjà réglés.\n");
        CloseHandle(port);
        return 1;
    }

    printf("\nRéglages qui vont être SAUVEGARDÉS dans le variateur :\n"
           "  C00.02 = %lu (pleine résolution du codeur : %.7f° par impulsion)\n"
           "  C0A.01 = %d (115200 bauds)\n"
           "Ils prennent effet au prochain démarrage du variateur. Ensuite, les autres\n"
           "programmes devront parler à 115200 bauds (BAUDRATE dans Bouger_Moteur_OK.py),\n"
           "et les déplacements en impulsions (ex. C03.0C = 5000) seront beaucoup plus petits.\n",
           (unsigned long)impulsions_cible, 360.0 / (double)impulsions_cible, CODE_115200_BAUDS);
    if (!demander_confirmation("Le moteur va être coupé (S-OFF).")) {
        printf("Annulé, rien n'a été modifié.\n");
        CloseHandle(port);
        return 1;
    }

    ok = ecrire_16(port, REG_STOCKAGE_EEPROM, 1) &&          /* les écritures suivantes sont sauvegardées */
         ecrire_16(port, REG_DI5_LOGIQUE, DI_NO) &&            /* S-OFF (valeur par défaut) */
         ecrire_32(port, REG_IMPULSIONS_TOUR, impulsions_cible) &&
         ecrire_16(port, REG_DEBIT, CODE_115200_BAUDS);       /* en dernier : actif au redémarrage */
    CloseHandle(port);
    if (!ok) {
        fprintf(stderr, "Échec : le réglage n'est pas complet. Relancez la commande.\n");
        return 0;
    }
    printf("\nRéglages sauvegardés. COUPEZ puis RALLUMEZ le variateur pour les activer.\n");
    return 1;
}

/* ============================================================================
 *          ZÉRO : LIEN ENTRE L'ANGLE DU CODEUR ET LES ANGLES DU BRAS
 * ========================================================================== */

/* Fichier "zero_moteurN.txt", à côté du .exe. */
static int chemin_zero(int moteur, char *chemin, size_t taille)
{
    char nom[32];
    snprintf(nom, sizeof(nom), "zero_moteur%d.txt", moteur);
    return chemin_a_cote_exe(nom, chemin, taille);
}

/* Lit le zéro du moteur : l'angle mécanique U40.09 (degrés) quand son theta vaut 0°. */
static int lire_zero(int moteur, double *zero_deg)
{
    char chemin[MAX_PATH];
    if (!chemin_zero(moteur, chemin, sizeof(chemin))) {
        return 0;
    }
    FILE *f = fopen(chemin, "r");
    if (f == NULL) {
        return 0;
    }
    char ligne[256];
    int ok = 0;
    while (!ok && fgets(ligne, sizeof(ligne), f) != NULL) {
        if (ligne[0] != '#') {
            ok = sscanf(ligne, "%lf", zero_deg) == 1 && isfinite(*zero_deg) &&
                 *zero_deg >= 0.0 && *zero_deg < 360.0;
        }
    }
    fclose(f);
    return ok;
}

/* --zero : le bras est placé à la main (moteur coupé) à l'angle connu angle_bras. Lit
   l'angle mécanique U40.09 et enregistre le zéro dans zero_moteurN.txt. */
static int faire_zero(int moteur, const char *nom_port, double angle_bras)
{
    char chemin[MAX_PATH];
    if (!chemin_zero(moteur, chemin, sizeof(chemin))) {
        fprintf(stderr, "Chemin du fichier de zéro trop long.\n");
        return 0;
    }
    if (angle_bras < ANGLE_MIN_DEG || angle_bras > ANGLE_MAX_DEG) {
        fprintf(stderr, "L'angle %.3f° est hors de la plage autorisée [%.0f° ; %.0f°].\n",
                angle_bras, ANGLE_MIN_DEG, ANGLE_MAX_DEG);
        return 0;
    }

    HANDLE port = ouvrir_port(nom_port);
    if (port == INVALID_HANDLE_VALUE) {
        return 0;
    }
    uint16_t centiemes = 0;
    int ok = trouver_variateur(port) != 0 && lire_16(port, REG_ANGLE_MECANIQUE, &centiemes);
    CloseHandle(port);
    if (!ok) {
        return 0;
    }
    if (centiemes > 36000) {
        fprintf(stderr, "Angle mécanique illisible (U40.09 = %u).\n", centiemes);
        return 0;
    }

    double meca = centiemes / 100.0;
    double zero = fmod(meca - SENS_MOTEUR * angle_bras, 360.0);
    if (zero < 0.0) {
        zero += 360.0;
    }
    if (zero >= 360.0) {
        zero = 0.0;
    }
    double ancien = 0.0;
    int avait_zero = lire_zero(moteur, &ancien);

    printf("Angle mécanique du moteur (U40.09) : %.2f°\n", meca);
    printf("Bras à theta%d = %.3f°  =>  zéro du moteur %d = %.4f°", moteur, angle_bras, moteur, zero);
    if (avait_zero) {
        printf(" (ancien : %.4f°)", ancien);
    }
    printf("\n");
    char question[96];
    snprintf(question, sizeof(question), "Le bras est-il bien à theta%d = %.3f° ?", moteur, angle_bras);
    if (!demander_confirmation(question)) {
        printf("Annulé, rien n'a été enregistré.\n");
        return 1;
    }

    FILE *f = fopen(chemin, "w");
    if (f == NULL) {
        fprintf(stderr, "Impossible d'écrire %s.\n", chemin);
        return 0;
    }
    fprintf(f, "# Zéro du moteur %d : angle mécanique U40.09 (degrés) quand theta%d = 0°.\n"
               "# Mesuré par --zero : U40.09 = %.2f° avec le bras à theta%d = %.3f°.\n"
               "%.4f\n", moteur, moteur, meca, moteur, angle_bras, zero);
    if (fclose(f) != 0) {
        fprintf(stderr, "Erreur d'écriture de %s.\n", chemin);
        return 0;
    }
    printf("Zéro enregistré dans %s\n", chemin);
    return 1;
}

/* ============================================================================
 *                         PROGRAMME PRINCIPAL
 * ========================================================================== */

/* Si le fichier n'existe pas tel quel (programme lancé depuis un autre dossier),
   le cherche à côté du .exe. */
static const char *resoudre_chemin(const char *nom, char *chemin, size_t taille)
{
    if (GetFileAttributesA(nom) != INVALID_FILE_ATTRIBUTES || !chemin_a_cote_exe(nom, chemin, taille)) {
        return nom;
    }
    return (GetFileAttributesA(chemin) != INVALID_FILE_ATTRIBUTES) ? chemin : nom;
}

/* Lit un nombre décimal (accepte la virgule). */
static int lire_nombre(const char *texte, double *valeur)
{
    char copie[64];
    if (strlen(texte) >= sizeof(copie)) {
        return 0;
    }
    strcpy(copie, texte);
    for (char *c = copie; *c != '\0'; c++) {
        if (*c == ',') {
            *c = '.';
        }
    }
    char *fin = NULL;
    *valeur = strtod(copie, &fin);
    return fin != copie && *fin == '\0' && isfinite(*valeur);
}

static void afficher_utilisation(const char *programme)
{
    fprintf(stderr, "Utilisation : %s [fichier.csv] [moteur 1-4] [COMx | SIMU]\n"
                    "              %s --zero [moteur 1-4] [COMx] [angle actuel du bras, défaut 0]\n"
                    "              %s --config-variateur [COMx]\n", programme, programme, programme);
}

/* Vrai si le programme est seul dans sa console, c'est-à-dire lancé par double-clic :
   la fenêtre se fermerait alors dès la fin du programme. */
static int console_ouverte_par_double_clic(void)
{
    DWORD processus[2];
    return GetConsoleProcessList(processus, 2) == 1;
}

static int programme(int argc, char *argv[])
{
    if (!(ANGLE_MAX_DEG > ANGLE_MIN_DEG) || ANGLE_MAX_DEG - ANGLE_MIN_DEG > 360.0) {
        fprintf(stderr, "Réglage invalide : la plage [ANGLE_MIN_DEG ; ANGLE_MAX_DEG] doit faire "
                        "entre 0 et 360°.\n");
        return EXIT_FAILURE;
    }
    if (argc > 1 && strcmp(argv[1], "--config-variateur") == 0) {
        return configurer_variateur_une_fois((argc > 2) ? argv[2] : PORT_DEFAUT)
                   ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    if (argc > 1 && strcmp(argv[1], "--zero") == 0) {
        int moteur = (argc > 2) ? atoi(argv[2]) : MOTEUR_DEFAUT;
        double angle = 0.0;
        if (argc > 5 || moteur < 1 || moteur > 4 || (argc > 4 && !lire_nombre(argv[4], &angle))) {
            afficher_utilisation(argv[0]);
            return EXIT_FAILURE;
        }
        return faire_zero(moteur, (argc > 3) ? argv[3] : PORT_DEFAUT, angle)
                   ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    char chemin_csv[MAX_PATH];
    const char *fichier = resoudre_chemin((argc > 1) ? argv[1] : FICHIER_DEFAUT,
                                          chemin_csv, sizeof(chemin_csv));
    int moteur = (argc > 2) ? atoi(argv[2]) : MOTEUR_DEFAUT;
    const char *nom_port = (argc > 3) ? argv[3] : PORT_DEFAUT;
    if (argc > 4 || moteur < 1 || moteur > 4) {
        afficher_utilisation(argv[0]);
        return EXIT_FAILURE;
    }
    g_simulation = (_stricmp(nom_port, "SIMU") == 0);

    /* 1. Trajectoire brute, plage d'angles et zéro du moteur */
    size_t n_lus = 0;
    Point *brut = lire_csv(fichier, moteur, &n_lus);
    if (brut == NULL) {
        return EXIT_FAILURE;
    }
    if (!verifier_plage(brut, n_lus)) {
        free(brut);
        return EXIT_FAILURE;
    }
    double zero_deg = 0.0;
    if (!lire_zero(moteur, &zero_deg)) {
        if (!g_simulation) {
            char chemin_z[MAX_PATH];
            if (!chemin_zero(moteur, chemin_z, sizeof(chemin_z))) {
                strcpy(chemin_z, "zero_moteurN.txt");
            }
            fprintf(stderr, "Zéro du moteur %d inconnu (%s absent ou illisible) : sans lui, le "
                            "programme ne sait pas où est le bras dans la plage [%.0f° ; %.0f°].\n"
                            "Moteur coupé, placez le bras à theta%d = 0° (ou à un angle connu A), "
                            "puis lancez :\n    .\\Code_C_V5_Trajectoire.exe --zero %d %s [A]\n",
                    moteur, chemin_z, ANGLE_MIN_DEG, ANGLE_MAX_DEG, moteur, moteur, nom_port);
            free(brut);
            return EXIT_FAILURE;
        }
        printf("SIMU : pas de zéro enregistré pour le moteur %d, zéro simulé = 0°.\n", moteur);
    }
    if (g_simulation) {
        double meca = fmod(zero_deg + SENS_MOTEUR * SIMU_THETA_DEPART_DEG, 360.0);
        g_simu_angle_meca = (uint16_t)(lround((meca < 0.0 ? meca + 360.0 : meca) * 100.0) % 36000);
    }

    /* 2. Variateur : débit, résolution, vitesse de la liaison */
    HANDLE port = INVALID_HANDLE_VALUE;
    uint32_t impulsions_par_tour = IMPULSIONS_PAR_TOUR_SIMU;
    int periode_ms = PERIODE_MIN_MS;
    if (g_simulation) {
        printf("Mode SIMU : aucune trame n'est envoyée (résolution simulée : %lu impulsions/tour).\n",
               (unsigned long)impulsions_par_tour);
    } else {
        port = ouvrir_port(nom_port);
        if (port == INVALID_HANDLE_VALUE) {
            free(brut);
            return EXIT_FAILURE;
        }
        DWORD debit = trouver_variateur(port);
        double trame_ms = (debit != 0) ? mesurer_trame_ms(port) : -1.0;
        if (trame_ms < 0.0 || !lire_32(port, REG_IMPULSIONS_TOUR, &impulsions_par_tour)) {
            CloseHandle(port);
            free(brut);
            return EXIT_FAILURE;
        }
        if (impulsions_par_tour == 0) {
            fprintf(stderr, "C00.02 = 0 : le variateur utilise le réducteur électronique "
                            "C03.02/C03.04, que ce programme ne gère pas. Lancez "
                            "--config-variateur.\n");
            CloseHandle(port);
            free(brut);
            return EXIT_FAILURE;
        }
        /* Un point = déplacement (32 bits) + 2 fronts, parfois la vitesse : ~3,5 trames */
        int periode_liaison = (int)ceil(3.5 * trame_ms * 1.25);
        if (periode_liaison > periode_ms) {
            periode_ms = periode_liaison;
        }
        printf("Variateur trouvé sur %s à %lu bauds : %.1f ms par trame -> un point toutes les %d ms.\n",
               nom_port, (unsigned long)debit, trame_ms, periode_ms);
        if (periode_ms > PERIODE_MIN_MS) {
            printf("  (Pour descendre à %d ms : passez à 115200 bauds avec --config-variateur, et\n"
                   "   réglez la latence de l'adaptateur USB à 1 ms si c'est un FTDI.)\n",
                   PERIODE_MIN_MS);
        }
    }

    /* 3. Position actuelle du bras, rééchantillonnage au pas choisi, impulsions, vitesses */
    double angle_actuel = 0.0;
    size_t n = 0;
    Point *points = NULL;
    if (lire_angle_bras(port, zero_deg, brut[0].angle_deg, &angle_actuel)) {
        points = reechantillonner(brut, n_lus, periode_ms / 1000.0, &n);
    }
    free(brut);
    if (points == NULL || !preparer(points, n, impulsions_par_tour)) {
        free(points);
        if (port != INVALID_HANDLE_VALUE) {
            CloseHandle(port);
        }
        return EXIT_FAILURE;
    }
    afficher_resume(fichier, moteur, n_lus, points, n, impulsions_par_tour, periode_ms, angle_actuel);

    int mouvement = 0;
    for (size_t i = 1; i < n; i++) {
        mouvement |= (points[i].impulsions != points[i - 1].impulsions);
    }
    if (!mouvement) {
        printf("Ce moteur ne bouge pas sur cette trajectoire : rien à faire.\n");
        free(points);
        if (port != INVALID_HANDLE_VALUE) {
            CloseHandle(port);
        }
        return EXIT_SUCCESS;
    }

    if (!g_simulation &&
        !demander_confirmation("ATTENTION : le moteur va rejoindre le départ puis suivre la "
                               "trajectoire. Rien ne doit gêner sa course et l'arrêt d'urgence "
                               "doit être à portée de main.")) {
        printf("Annulé.\n");
        CloseHandle(port);
        free(points);
        return EXIT_SUCCESS;
    }
    SetConsoleCtrlHandler(gestionnaire_ctrl_c, TRUE);

    /* 4. Configuration, mise sous couple, trajectoire, coupure */
    int succes = configurer(port);
    if (succes) {
        uint32_t position_depart = 0;
        /* L'angle est relu une fois le moteur sous couple : le bras a pu bouger avant */
        succes = mettre_sous_couple(port) &&
                 lire_angle_bras(port, zero_deg, points[0].angle_deg, &angle_actuel) &&
                 rejoindre_depart(port, &angle_actuel, points[0].angle_deg, zero_deg,
                                  impulsions_par_tour) &&
                 lire_32(port, REG_POSITION_REELLE, &position_depart);
        if (succes) {
            succes = executer(port, points, n, periode_ms);
        }
        if (succes) {
            verifier_position(port, position_depart, points[n - 1].impulsions, impulsions_par_tour,
                              points[n - 1].angle_deg, zero_deg);
            attendre_touche_avant_coupure();
        }
        couper_moteur(port);
    }

    if (port != INVALID_HANDLE_VALUE) {
        CloseHandle(port);
    }
    free(points);
    printf(succes ? "\nTrajectoire terminée.\n" : "\nTrajectoire interrompue.\n");
    return succes ? EXIT_SUCCESS : EXIT_FAILURE;
}

int main(int argc, char *argv[])
{
    SetConsoleOutputCP(CP_UTF8);        /* affichage correct des accents et des ° */
    int code = programme(argc, argv);
    if (console_ouverte_par_double_clic()) {
        printf("\nAppuyez sur une touche pour fermer cette fenêtre...");
        fflush(stdout);
        _getch();
    }
    return code;
}
