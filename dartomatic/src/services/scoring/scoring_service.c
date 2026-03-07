#include "appbus/client.h"
#include "ipc/appbus_ipc.h"
#include "appbus/topics.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/*
 * scoring_service (V2)
 * --------------------
 * Rôle :
 *   - écouter evt/impact/triangulated
 *   - convertir (x_mm, y_mm) en score réel de fléchettes
 *   - publier evt/hit/scored
 *
 * Hypothèses :
 *   - x_mm / y_mm sont exprimés en millimètres
 *   - l'origine (0,0) est le centre de la cible
 *   - +X = droite
 *   - +Y = haut
 *
 * IMPORTANT :
 *   - le vrai plateau a un ordre de secteurs spécifique
 *   - il faut souvent un OFFSET angulaire pour aligner le "20" avec le haut réel
 */

/* ========================================================= */
/* Parsing JSON minimaliste                                  */
/* ========================================================= */

static int json_get_u64(const char* json, const char* key, unsigned long long* out) {
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\":", key);
    const char* p = strstr(json, pat);
    if (!p) return 0;
    p += strlen(pat);
    return (sscanf(p, "%llu", out) == 1);
}

static int json_get_double(const char* json, const char* key, double* out) {
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\":", key);
    const char* p = strstr(json, pat);
    if (!p) return 0;
    p += strlen(pat);
    return (sscanf(p, "%lf", out) == 1);
}

/* ========================================================= */
/* Géométrie plateau standard (en mm)                        */
/* ========================================================= */

/*
 * Valeurs classiques d'une cible acier standard :
 * - bullseye (50) : diamètre 12.7 mm  => rayon 6.35 mm
 * - bull (25)     : diamètre 31.8 mm  => rayon 15.9 mm
 * - triple ring   : rayon intérieur ~99 mm, extérieur ~107 mm
 * - double ring   : rayon intérieur ~162 mm, extérieur ~170 mm
 *
 * NOTE :
 * si ta cible réelle diffère un peu, tu pourras ajuster ces constantes.
 */
static const double R_BULLSEYE_MM    = 6.35;
static const double R_BULL_OUTER_MM  = 15.90;
static const double R_TRIPLE_IN_MM   = 99.0;
static const double R_TRIPLE_OUT_MM  = 107.0;
static const double R_DOUBLE_IN_MM   = 162.0;
static const double R_DOUBLE_OUT_MM  = 170.0;

/*
 * Ordre réel des secteurs de fléchettes.
 * En partant du 20 en haut, puis en tournant dans le sens horaire :
 *
 *   20, 1, 18, 4, 13, 6, 10, 15, 2, 17,
 *    3,19,  7,16,  8,11, 14, 9,12, 5
 */
static const int BOARD_NUMBERS[20] = {
    20, 1, 18, 4, 13, 6, 10, 15, 2, 17,
     3,19,  7,16,  8,11, 14, 9,12, 5
};

/*
 * Offset angulaire de calibration.
 *
 * Interprétation :
 *   - notre angle "board" vaut 0° au sommet de la cible (zone du 20),
 *     et augmente dans le sens horaire.
 *   - si ton repère XY n'est pas parfaitement aligné avec la vraie cible,
 *     tu peux corriger ici.
 *
 * Exemple :
 *   - 0.0  : aucun décalage
 *   - +9.0 : décale d'un demi-secteur
 *   - -18.0: décale d'un secteur dans l'autre sens
 *
 * À ajuster après essais réels.
 */
static const double BOARD_OFFSET_DEG = 72.0;

/* ========================================================= */
/* Types internes                                            */
/* ========================================================= */

typedef enum {
    HIT_OUT = 0,
    HIT_SINGLE,
    HIT_DOUBLE,
    HIT_TRIPLE,
    HIT_BULL,
    HIT_BULLSEYE
} HitRing;

typedef struct {
    HitRing ring;
    int sector;       /* 1..20, ou 25 / 50, ou 0 si OUT */
    int score;        /* score final */
    int multiplier;   /* 0, 1, 2, 3 */
    double r_mm;      /* distance au centre */
    double theta_deg; /* angle brut mathématique */
    double board_deg; /* angle corrigé pour le plateau */
} HitResult;

typedef struct {
    AppBusClient* bus;
} Ctx;

/* ========================================================= */
/* Helpers math                                              */
/* ========================================================= */

static double normalize_deg(double a) {
    while (a < 0.0)   a += 360.0;
    while (a >= 360.) a -= 360.0;
    return a;
}

static const char* ring_to_string(HitRing ring) {
    switch (ring) {
        case HIT_SINGLE:   return "SINGLE";
        case HIT_DOUBLE:   return "DOUBLE";
        case HIT_TRIPLE:   return "TRIPLE";
        case HIT_BULL:     return "BULL";
        case HIT_BULLSEYE: return "BULLSEYE";
        case HIT_OUT:
        default:           return "OUT";
    }
}

/* ========================================================= */
/* Scoring géométrique réel                                  */
/* ========================================================= */

/*
 * Détermine l'anneau à partir du rayon.
 */
static HitRing detect_ring(double r_mm) {
    if (r_mm <= R_BULLSEYE_MM)   return HIT_BULLSEYE;
    if (r_mm <= R_BULL_OUTER_MM) return HIT_BULL;

    if (r_mm > R_DOUBLE_OUT_MM)  return HIT_OUT;

    if (r_mm >= R_DOUBLE_IN_MM && r_mm <= R_DOUBLE_OUT_MM)
        return HIT_DOUBLE;

    if (r_mm >= R_TRIPLE_IN_MM && r_mm <= R_TRIPLE_OUT_MM)
        return HIT_TRIPLE;

    return HIT_SINGLE;
}

/*
 * Détermine le secteur 1..20 à partir de l'angle.
 *
 * Méthode :
 *   1) theta_deg = atan2(y, x) en degrés :
 *      - 0°   = +X
 *      - 90°  = +Y
 *      - angle croît anti-horaire
 *
 *   2) On transforme en angle "plateau" :
 *      - 0° au sommet (20 en haut)
 *      - angle croît dans le sens horaire
 *
 *   3) Chaque secteur fait 18°
 *      On ajoute 9° pour centrer correctement le secteur
 */
static int detect_sector(double theta_deg, double* board_deg_out) {
    /* Conversion math -> plateau */
    double board_deg = normalize_deg(90.0 - theta_deg + BOARD_OFFSET_DEG);

    if (board_deg_out) *board_deg_out = board_deg;

    /* Centre des secteurs */
    int index = (int)floor((board_deg + 9.0) / 18.0) % 20;
    return BOARD_NUMBERS[index];
}

/*
 * Score complet à partir de (x_mm, y_mm).
 */
static HitResult compute_hit(double x_mm, double y_mm) {
    HitResult hr;
    memset(&hr, 0, sizeof(hr));

    hr.r_mm = sqrt(x_mm * x_mm + y_mm * y_mm);
    hr.theta_deg = atan2(y_mm, x_mm) * 180.0 / M_PI;

    hr.ring = detect_ring(hr.r_mm);

    if (hr.ring == HIT_OUT) {
        hr.sector = 0;
        hr.score = 0;
        hr.multiplier = 0;
        detect_sector(hr.theta_deg, &hr.board_deg); /* utile pour debug */
        return hr;
    }

    if (hr.ring == HIT_BULLSEYE) {
        hr.sector = 50;
        hr.score = 50;
        hr.multiplier = 1;
        hr.board_deg = 0.0;
        return hr;
    }

    if (hr.ring == HIT_BULL) {
        hr.sector = 25;
        hr.score = 25;
        hr.multiplier = 1;
        hr.board_deg = 0.0;
        return hr;
    }

    hr.sector = detect_sector(hr.theta_deg, &hr.board_deg);

    switch (hr.ring) {
        case HIT_SINGLE:
            hr.multiplier = 1;
            hr.score = hr.sector;
            break;

        case HIT_DOUBLE:
            hr.multiplier = 2;
            hr.score = 2 * hr.sector;
            break;

        case HIT_TRIPLE:
            hr.multiplier = 3;
            hr.score = 3 * hr.sector;
            break;

        default:
            hr.multiplier = 0;
            hr.score = 0;
            break;
    }

    return hr;
}

/* ========================================================= */
/* Callback AppBus                                           */
/* ========================================================= */

static void on_bus_msg(const char* topic, const char* payload, size_t payload_len, void* user) {
    (void)payload_len;
    Ctx* ctx = (Ctx*)user;

    if (strcmp(topic, TOPIC_EVT_IMPACT_TRIANG) != 0) return;
    if (!payload) return;

    unsigned long long impact_id = 0;
    double x = 0.0, y = 0.0, z = 0.0, quality = 0.0;

    int ok = 1;
    ok &= json_get_u64(payload, "impact_id", &impact_id);
    ok &= json_get_double(payload, "x_mm", &x);
    ok &= json_get_double(payload, "y_mm", &y);

    /* Facultatifs en V2 */
    json_get_double(payload, "z_mm", &z);
    json_get_double(payload, "quality", &quality);

    if (!ok) {
        fprintf(stderr, "[scoring] payload invalide: %s\n", payload);
        return;
    }

    HitResult hr = compute_hit(x, y);

    /*
     * On publie un hit enrichi.
     * Ça servira plus tard au game engine (301, cricket, etc.).
     */
    char out[768];
    snprintf(out, sizeof(out),
             "{"
             "\"impact_id\":%llu,"
             "\"score\":%d,"
             "\"ring\":\"%s\","
             "\"sector\":%d,"
             "\"multiplier\":%d,"
             "\"x_mm\":%.2f,"
             "\"y_mm\":%.2f,"
             "\"z_mm\":%.2f,"
             "\"r_mm\":%.2f,"
             "\"theta_deg\":%.2f,"
             "\"board_deg\":%.2f,"
             "\"quality\":%.3f"
             "}",
             impact_id,
             hr.score,
             ring_to_string(hr.ring),
             hr.sector,
             hr.multiplier,
             x, y, z,
             hr.r_mm,
             hr.theta_deg,
             hr.board_deg,
             quality);

    if (appbus_publish(ctx->bus, TOPIC_EVT_HIT_SCORED, out) != 0) {
        fprintf(stderr, "[scoring] erreur publish hit_scored\n");
        return;
    }

    printf("[scoring] HIT scored -> %s\n", out);
    fflush(stdout);
}

/* ========================================================= */
/* main                                                      */
/* ========================================================= */

int main(int argc, char** argv) {
    const char* sock = (argc >= 2) ? argv[1] : APPBUS_DEFAULT_SOCK;

    printf("[scoring] démarrage, sock=%s\n", sock);
    printf("[scoring] offset plateau = %.2f deg\n", BOARD_OFFSET_DEG);

    AppBusClient* bus = appbus_connect(sock);
    if (!bus) {
        fprintf(stderr, "[scoring] impossible de se connecter au bus\n");
        return 2;
    }

    if (appbus_subscribe(bus, TOPIC_EVT_IMPACT_TRIANG) != 0) {
        fprintf(stderr, "[scoring] subscribe échoué\n");
        appbus_close(bus);
        return 3;
    }

    Ctx ctx = { .bus = bus };

    /* Boucle bloquante */
    int rc = appbus_poll(bus, on_bus_msg, &ctx);

    appbus_close(bus);
    return (rc == 0) ? 0 : 4;
}