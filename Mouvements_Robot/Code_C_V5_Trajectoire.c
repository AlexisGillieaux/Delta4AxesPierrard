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
 *     point précédent, la vitesse, puis donne un front sur DI6.
 *   - Les déplacements sont RELATIFS (C11.01 = 1) mais calculés à partir des
 *     positions absolues arrondies : les arrondis ne s'accumulent pas. En relatif,
 *     un déplacement ne peut jamais dépasser ce que le programme a vérifié.
 *   - S-ON (DI5) et le déclenchement (DI6) sont commandés par logiciel en inversant
 *     la logique NO/NC de ces entrées non câblées (C04.11 et C04.15), comme dans
 *     Bouger_Moteur_OK.py.
 *   - C0A.05 = 0 : les écritures Modbus ne sont PAS sauvegardées en EEPROM. Un
 *     redémarrage du variateur rend tous ses réglages d'origine.
 *
 * Compilation (dans le dossier Mouvements_Robot) :
 *     gcc Code_C_V5_Trajectoire.c -o Code_C_V5_Trajectoire.exe
 *
 * Utilisation :
 *     .\Code_C_V5_Trajectoire.exe [fichier.csv] [moteur 1-4] [COMx | SIMU]
 *     défaut : trajectoire_moteurs.csv 1 COM3
 *     SIMU : n'ouvre pas de port série, affiche les trames qui seraient envoyées.
 *
 * Le point de départ de la trajectoire est la position ACTUELLE du moteur : le
 * moteur fait les mêmes déplacements que le θ choisi, à partir de là où il est.
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

#define BAUDRATE            9600       /* doit être égal à C0A.01 du variateur (3 = 9600 bauds) */
#define ADRESSE_ESCLAVE     0x01       /* C0A.00 : adresse Modbus du variateur */

/* Conversion angle -> impulsions */
#define IMPULSIONS_PAR_TOUR 10000.0    /* C00.02 : unités de consigne par tour moteur (défaut 10000) */
#define RAPPORT_REDUCTION   1.0        /* tours moteur pour 1 tour du bras (1.0 = entraînement direct) */
#define SENS_MOTEUR         1          /* +1, ou -1 si le moteur tourne dans le mauvais sens */

/* Cadence */
#define PERIODE_MS          200        /* écart minimal entre deux points envoyés. À 9600 bauds,
                                          un point demande jusqu'à ~100 ms de Modbus. */
#define MARGE_VITESSE       1.15       /* vitesse envoyée = vitesse nécessaire x marge, arrondie au
                                          tr/min supérieur : chaque déplacement finit avant le suivant */
#define TEMPS_ACCEL_MS      20         /* C11.0A / C11.0C (probablement ms pour 0 -> 1000 tr/min) */

/* Sécurité : la trajectoire est refusée au-delà */
#define COURSE_MAX_DEG      180.0      /* écart maximal au point de départ, en degrés du θ choisi */
#define VITESSE_MAX_RPM     60         /* vitesse maximale envoyée au moteur (tr/min) */

/* Communication */
#define TIMEOUT_REPONSE_MS  300        /* attente maximale d'une réponse Modbus */
#define NB_ESSAIS           2          /* tentatives par trame avant abandon */

/* ============================================================================
 *                 PARAMÈTRES DU VARIATEUR (adresse = 0xGGII pour CGG.II)
 * ========================================================================== */

#define REG_MODE_CONTROLE      0x0000  /* C00.00 : 0 = mode position */
#define REG_SOURCE_POSITION    0x0300  /* C03.00 : 1 = planification interne */
#define REG_DI5_LOGIQUE        0x0411  /* C04.11 : DI5 a la fonction S-ON par défaut (C04.10 = 1) */
#define REG_DI6_FONCTION       0x0414  /* C04.14 : fonction de DI6 (0 = aucune par défaut) */
#define REG_DI6_LOGIQUE        0x0415  /* C04.15 : logique de DI6 */
#define REG_STOCKAGE_EEPROM    0x0A05  /* C0A.05 : 0 = pas de sauvegarde EEPROM */
#define REG_ORDRE_MOTS_32      0x0A06  /* C0A.06 : 0 = mot bas avant mot haut */
#define REG_PLAN_MODE          0x1100  /* C11.00 : 3 = mode PP */
#define REG_PLAN_TYPE          0x1101  /* C11.01 : 1 = déplacements relatifs */
#define REG_PLAN_GROUPE_DEBUT  0x1103  /* C11.03 : premier groupe exécuté */
#define REG_PLAN_GROUPE_FIN    0x1104  /* C11.04 : dernier groupe exécuté */
#define REG_PLAN_DEPLACEMENT   0x1106  /* C11.06 : I32, déplacement du groupe 1 (impulsions) */
#define REG_PLAN_VITESSE       0x1108  /* C11.08 : U16, vitesse du groupe 1 (tr/min) */
#define REG_PLAN_ACCEL         0x110A  /* C11.0A : U32, temps d'accélération (ms) */
#define REG_PLAN_DECEL         0x110C  /* C11.0C : U32, temps de décélération (ms) */
#define REG_PLAN_ATTENTE       0x110E  /* C11.0E : U32, temps d'attente (ms) */
#define REG_POSITION_REELLE    0x4016  /* U40.16 : I32, position absolue mesurée (impulsions) */
#define REG_ETAT_SERVO         0x410A  /* U41.0A : 0 pas prêt, 1 prêt, 2 en marche, 3 défaut */

#define FONCTION_DECLENCHEMENT 19      /* FunIN.19 : position planning trigger */
#define DI_NO                  0       /* entrée non câblée => inactive */
#define DI_NC                  1       /* entrée non câblée => ACTIVE */

#define ETAT_PAS_PRET          0
#define ETAT_EN_MARCHE         2
#define ETAT_DEFAUT            3

/* ============================================================================
 *                         ÉTAT GLOBAL
 * ========================================================================== */

static volatile LONG g_arret_demande = 0;   /* passe à 1 sur Ctrl+C */
static int g_simulation = 0;                /* 1 : port "SIMU", aucune trame envoyée */

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
            Sleep(0);
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

static HANDLE ouvrir_port(const char *chemin)
{
    HANDLE port = CreateFileA(chemin, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                              OPEN_EXISTING, 0, NULL);
    if (port == INVALID_HANDLE_VALUE) {
        afficher_erreur_windows("Impossible d'ouvrir le port série");
        return INVALID_HANDLE_VALUE;
    }

    DCB reglages;
    memset(&reglages, 0, sizeof(reglages));
    reglages.DCBlength = sizeof(reglages);
    if (!GetCommState(port, &reglages)) {
        afficher_erreur_windows("Erreur GetCommState");
        CloseHandle(port);
        return INVALID_HANDLE_VALUE;
    }
    reglages.BaudRate = BAUDRATE;
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
        CloseHandle(port);
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
    PurgeComm(port, PURGE_RXCLEAR | PURGE_TXCLEAR);
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

/* Réponse fabriquée en mode SIMU : écho pour une écriture, zéros pour une lecture
   (sauf l'état du servo, qui vaut "en marche"). */
static void simuler_reponse(const uint8_t *req, uint8_t *rep, size_t len_rep)
{
    memset(rep, 0, len_rep);
    if (req[1] == 0x03) {
        rep[0] = req[0];
        rep[1] = req[1];
        rep[2] = (uint8_t)(len_rep - 5);
        if (req[2] == (REG_ETAT_SERVO >> 8) && req[3] == (REG_ETAT_SERVO & 0xFF)) {
            rep[4] = ETAT_EN_MARCHE;
        }
    } else {
        memcpy(rep, req, 6);
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
        printf("        [SIMU] -> ");
        afficher_hex(stdout, req, len_req);
        simuler_reponse(req, rep, len_rep);
        return 1;
    }

    for (int essai = 1; essai <= NB_ESSAIS; essai++) {
        PurgeComm(port, PURGE_RXCLEAR | PURGE_TXCLEAR);

        DWORD ecrits = 0;
        if (!WriteFile(port, req, (DWORD)len_req, &ecrits, NULL) || ecrits != len_req) {
            afficher_erreur_windows("Erreur WriteFile");
            continue;
        }

        double echeance = maintenant_ms() + TIMEOUT_REPONSE_MS;
        size_t n = lire_octets(port, rep, 2, echeance);
        if (n < 2) {
            fprintf(stderr, "  Pas de réponse du variateur (essai %d/%d).\n", essai, NB_ESSAIS);
            continue;
        }

        if (rep[1] & 0x80) {
            /* Réponse d'erreur : la trame est refusée, inutile de la renvoyer */
            n += lire_octets(port, rep + 2, 6, maintenant_ms() + 50.0);
            fprintf(stderr, "  Le variateur refuse la trame (réponse d'erreur) : ");
            afficher_hex(stderr, rep, n);
            fprintf(stderr, "  Trame envoyée : ");
            afficher_hex(stderr, req, len_req);
            return 0;
        }

        n += lire_octets(port, rep + 2, len_rep - 2, echeance);
        if (n < len_rep) {
            fprintf(stderr, "  Réponse incomplète (%u/%u octets, essai %d/%d).\n",
                    (unsigned)n, (unsigned)len_rep, essai, NB_ESSAIS);
            continue;
        }
        uint16_t crc_recu = (uint16_t)(rep[len_rep - 2] | (rep[len_rep - 1] << 8));
        if (crc16_modbus(rep, len_rep - 2) != crc_recu) {
            fprintf(stderr, "  Réponse avec CRC invalide (essai %d/%d).\n", essai, NB_ESSAIS);
            continue;
        }
        if (rep[0] != req[0] || rep[1] != req[1]) {
            fprintf(stderr, "  Réponse d'un autre esclave ou d'une autre fonction (essai %d/%d).\n",
                    essai, NB_ESSAIS);
            continue;
        }
        return 1;
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
    return 1;
}

/* Fonction 0x10 : écrit un paramètre 32 bits (2 registres, mot bas d'abord car C0A.06 = 0).
   Le manuel interdit d'écrire un paramètre 32 bits avec 0x06. */
static int ecrire_32(HANDLE port, uint16_t reg, int32_t valeur)
{
    uint32_t v = (uint32_t)valeur;
    uint16_t bas = (uint16_t)(v & 0xFFFF);
    uint16_t haut = (uint16_t)(v >> 16);
    uint8_t req[13], rep[8];
    req[0] = ADRESSE_ESCLAVE;
    req[1] = 0x10;
    req[2] = (uint8_t)(reg >> 8);
    req[3] = (uint8_t)(reg & 0xFF);
    req[4] = 0x00;                    /* nombre de registres : 2 */
    req[5] = 0x02;
    req[6] = 0x04;                    /* nombre d'octets de données : 4 */
    req[7] = (uint8_t)(bas >> 8);
    req[8] = (uint8_t)(bas & 0xFF);
    req[9] = (uint8_t)(haut >> 8);
    req[10] = (uint8_t)(haut & 0xFF);
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
        fprintf(stderr, "  Lecture : nombre d'octets inattendu (%u).\n", rep[2]);
        return 0;
    }
    for (uint16_t i = 0; i < nb; i++) {
        mots[i] = (uint16_t)((rep[3 + 2 * i] << 8) | rep[4 + 2 * i]);
    }
    return 1;
}

static int lire_32(HANDLE port, uint16_t reg, int32_t *valeur)
{
    uint16_t mots[2];
    if (!lire_registres(port, reg, 2, mots)) {
        return 0;
    }
    *valeur = (int32_t)(((uint32_t)mots[1] << 16) | mots[0]);   /* mot bas d'abord */
    return 1;
}

static int lire_etat_servo(HANDLE port, uint16_t *etat)
{
    return lire_registres(port, REG_ETAT_SERVO, 1, etat);
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

/* Garde un point au plus toutes les periode_s secondes. Le premier et le dernier
   point sont toujours gardés (le dernier remplace l'avant-dernier s'il est trop proche). */
static size_t decimer(Point *points, size_t n, double periode_s)
{
    if (n <= 2) {
        return n;
    }
    size_t k = 1;
    for (size_t i = 1; i < n - 1; i++) {
        if (points[i].t_s - points[k - 1].t_s >= periode_s - 1e-9) {
            points[k++] = points[i];
        }
    }
    if (k > 1 && points[n - 1].t_s - points[k - 1].t_s < periode_s - 1e-9) {
        k--;
    }
    points[k++] = points[n - 1];
    return k;
}

/* Calcule les impulsions et vitesses, et vérifie les limites de sécurité. */
static int preparer(Point *points, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        double ecart_deg = points[i].angle_deg - points[0].angle_deg;
        if (fabs(ecart_deg) > COURSE_MAX_DEG) {
            fprintf(stderr, "Trajectoire refusée : le point %u (t = %.2f s) est à %.1f° du départ "
                            "(limite COURSE_MAX_DEG = %.1f°).\n",
                    (unsigned)i, points[i].t_s, ecart_deg, COURSE_MAX_DEG);
            return 0;
        }
        double tours_moteur = SENS_MOTEUR * RAPPORT_REDUCTION * ecart_deg / 360.0;
        points[i].impulsions = lround(tours_moteur * IMPULSIONS_PAR_TOUR);

        if (i == 0) {
            continue;
        }
        long pas = labs(points[i].impulsions - points[i - 1].impulsions);
        double dt = points[i].t_s - points[i - 1].t_s;
        double rpm = (double)pas / IMPULSIONS_PAR_TOUR / dt * 60.0;
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

static void afficher_resume(const char *fichier, int moteur, size_t n_lus,
                            const Point *points, size_t n)
{
    double ecart_max = 0.0;
    int vitesse_max = 0;
    for (size_t i = 0; i < n; i++) {
        ecart_max = fmax(ecart_max, fabs(points[i].angle_deg - points[0].angle_deg));
        if (points[i].vitesse_rpm > vitesse_max) {
            vitesse_max = points[i].vitesse_rpm;
        }
    }
    printf("Trajectoire : %s, moteur %d (colonne theta%d)\n", fichier, moteur, moteur);
    printf("  Points lus / envoyés : %u / %u (au plus un toutes les %d ms)\n",
           (unsigned)n_lus, (unsigned)n, PERIODE_MS);
    printf("  Durée                : %.2f s\n", points[n - 1].t_s - points[0].t_s);
    printf("  Angle                : %.3f° -> %.3f° (%+ld impulsions moteur)\n",
           points[0].angle_deg, points[n - 1].angle_deg, points[n - 1].impulsions);
    printf("  Écart max au départ  : %.2f° (limite %.1f°)\n", ecart_max, COURSE_MAX_DEG);
    printf("  Vitesse max envoyée  : %d tr/min (limite %d)\n", vitesse_max, VITESSE_MAX_RPM);
    printf("  Conversion           : %.0f impulsions/tour moteur, réduction %.3f, sens %+d\n\n",
           IMPULSIONS_PAR_TOUR, RAPPORT_REDUCTION, SENS_MOTEUR);
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
        int ok = p->bits32 ? ecrire_32(port, p->reg, p->valeur)
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
    if (!lire_etat_servo(port, &etat)) {
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

    if (!lire_etat_servo(port, &etat)) {
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

/* Envoie les points un par un. Le déplacement vers le point i est lancé à
   l'instant du point i-1 : le moteur arrive au point i vers l'instant du point i. */
static int executer(HANDLE port, const Point *points, size_t n)
{
    printf("\nDépart de la trajectoire (Ctrl+C pour arrêter)\n");
    double t0 = maintenant_ms();
    int derniere_vitesse = -1;
    double comm_max_ms = 0.0, retard_max_ms = 0.0;

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

        long pas = points[i].impulsions - points[i - 1].impulsions;
        printf("  Point %3u/%u  t = %6.2f s  angle = %8.3f°  pas = %+6ld imp  vitesse = %2d tr/min\n",
               (unsigned)i, (unsigned)(n - 1), points[i].t_s, points[i].angle_deg,
               pas, points[i].vitesse_rpm);
        if (pas == 0) {
            continue;                     /* le moteur reste immobile pendant ce segment */
        }

        if (!ecrire_32(port, REG_PLAN_DEPLACEMENT, (int32_t)pas)) {
            return 0;
        }
        if (points[i].vitesse_rpm != derniere_vitesse) {
            if (!ecrire_16(port, REG_PLAN_VITESSE, (uint16_t)points[i].vitesse_rpm)) {
                return 0;
            }
            derniere_vitesse = points[i].vitesse_rpm;
        }
        /* Front sur DI6 : NC rend l'entrée active (le moteur part), NO la remet au repos */
        if (!ecrire_16(port, REG_DI6_LOGIQUE, DI_NC) ||
            !ecrire_16(port, REG_DI6_LOGIQUE, DI_NO)) {
            return 0;
        }
        comm_max_ms = fmax(comm_max_ms, maintenant_ms() - debut);
    }

    /* Laisse le dernier déplacement se terminer */
    attendre_jusqua(t0 + (points[n - 1].t_s - points[0].t_s) * 1000.0 + 500.0);

    if (!g_simulation) {
        printf("\nModbus : %.0f ms au plus par point (période %d ms), retard max %.0f ms.\n",
               comm_max_ms, PERIODE_MS, retard_max_ms);
        if (comm_max_ms > PERIODE_MS || retard_max_ms > PERIODE_MS / 2.0) {
            printf("Attention : la communication ne suit pas la cadence. Augmentez PERIODE_MS "
                   "ou passez le variateur à 115200 bauds (C0A.01 = 7, puis redémarrage).\n");
        }
    }
    return arret_demande() ? 0 : 1;
}

/* Compare la position mesurée à la position attendue en fin de trajectoire. */
static void verifier_position(HANDLE port, int32_t depart, long attendu_relatif)
{
    int32_t fin = 0;
    if (g_simulation) {
        printf("\n(SIMU : position finale non mesurée)\n");
        return;
    }
    if (!lire_32(port, REG_POSITION_REELLE, &fin)) {
        return;
    }
    long mesure = (long)fin - (long)depart;
    long erreur = mesure - attendu_relatif;
    double deg_par_impulsion = 360.0 / (IMPULSIONS_PAR_TOUR * RAPPORT_REDUCTION);
    printf("\nPosition finale : %+ld impulsions depuis le départ (attendu %+ld), "
           "erreur %+ld impulsions = %+.3f°\n",
           mesure, attendu_relatif, erreur, erreur * deg_par_impulsion);
    if (!g_simulation && labs(erreur) > 20) {
        printf("Attention : écart important. Un déclenchement a peut-être été manqué, ou le "
               "moteur n'a pas fini ses déplacements (augmentez MARGE_VITESSE ou PERIODE_MS).\n");
    }
}

static int demander_confirmation(void)
{
    char reponse[16];
    printf("ATTENTION : le moteur va tourner. Rien ne doit gêner sa course et l'arrêt\n"
           "d'urgence doit être à portée de main. Tapez o puis Entrée pour démarrer : ");
    fflush(stdout);
    if (fgets(reponse, sizeof(reponse), stdin) == NULL) {
        return 0;
    }
    return reponse[0] == 'o' || reponse[0] == 'O';
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
 *                         PROGRAMME PRINCIPAL
 * ========================================================================== */

int main(int argc, char *argv[])
{
    SetConsoleOutputCP(CP_UTF8);        /* affichage correct des accents et des ° */

    const char *fichier = (argc > 1) ? argv[1] : FICHIER_DEFAUT;
    int moteur = (argc > 2) ? atoi(argv[2]) : MOTEUR_DEFAUT;
    const char *nom_port = (argc > 3) ? argv[3] : PORT_DEFAUT;
    if (argc > 4 || moteur < 1 || moteur > 4) {
        fprintf(stderr, "Utilisation : %s [fichier.csv] [moteur 1-4] [COMx | SIMU]\n", argv[0]);
        return EXIT_FAILURE;
    }
    g_simulation = (_stricmp(nom_port, "SIMU") == 0);

    /* 1. Trajectoire */
    size_t n_lus = 0;
    Point *points = lire_csv(fichier, moteur, &n_lus);
    if (points == NULL) {
        return EXIT_FAILURE;
    }
    size_t n = decimer(points, n_lus, PERIODE_MS / 1000.0);
    if (!preparer(points, n)) {
        free(points);
        return EXIT_FAILURE;
    }
    afficher_resume(fichier, moteur, n_lus, points, n);

    int mouvement = 0;
    for (size_t i = 1; i < n; i++) {
        mouvement |= (points[i].impulsions != points[i - 1].impulsions);
    }
    if (!mouvement) {
        printf("Ce moteur ne bouge pas sur cette trajectoire : rien à faire.\n");
        free(points);
        return EXIT_SUCCESS;
    }

    /* 2. Port série */
    HANDLE port = INVALID_HANDLE_VALUE;
    if (g_simulation) {
        printf("Mode SIMU : aucune trame n'est envoyée.\n\n");
    } else {
        char chemin[32];
        if (!chemin_port(nom_port, chemin, sizeof(chemin))) {
            fprintf(stderr, "Port invalide : COM suivi d'un numéro (ex. COM3), ou SIMU.\n");
            free(points);
            return EXIT_FAILURE;
        }
        port = ouvrir_port(chemin);
        if (port == INVALID_HANDLE_VALUE) {
            free(points);
            return EXIT_FAILURE;
        }
        printf("Port %s ouvert (%d bauds, 8N1, esclave %d).\n", nom_port, BAUDRATE, ADRESSE_ESCLAVE);
        if (!demander_confirmation()) {
            printf("Annulé.\n");
            CloseHandle(port);
            free(points);
            return EXIT_SUCCESS;
        }
    }
    SetConsoleCtrlHandler(gestionnaire_ctrl_c, TRUE);

    /* 3. Configuration, mise sous couple, trajectoire, coupure */
    int succes = configurer(port);
    if (succes) {
        int32_t position_depart = 0;
        succes = mettre_sous_couple(port);
        if (succes) {
            succes = lire_32(port, REG_POSITION_REELLE, &position_depart);
            if (succes) {
                printf("Position de départ mesurée (U40.16) : %ld impulsions\n", (long)position_depart);
            }
        }
        if (succes) {
            succes = executer(port, points, n);
        }
        if (succes) {
            verifier_position(port, position_depart, points[n - 1].impulsions);
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
