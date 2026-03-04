#include "appbus/client.h"
#include "ipc/appbus_ipc.h"
#include "appbus/topics.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/*
 * scoring_service (V1)
 * --------------------
 * Rôle :
 *   - écouter evt/impact/triangulated
 *   - calculer un score "simple" pour valider la chaîne
 *   - publier evt/hit/scored
 *
 * IMPORTANT :
 *   Ici on ne fait PAS encore la vraie logique fléchettes.
 *   On veut juste vérifier : Impact -> Score -> Game.
 */

/* --------- parsing JSON "minimaliste" (pas robuste mais suffisant V1) ---------
 * On évite une dépendance à une lib JSON.
 * On cherche des patterns dans une string JSON:
 *   "impact_id":123
 *   "x_mm":12.3
 * etc.
 *
 * Limites :
 * - suppose que les clés existent
 * - suppose pas d'espaces bizarres ou formats exotiques
 * Pour V1 c'est OK.
 */

static int json_get_u64(const char* json, const char* key, unsigned long long* out) {
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\":", key);
    const char* p = strstr(json, pat);
    if (!p) return 0;
    p += strlen(pat);
    /* %llu lit un unsigned long long */
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

/*
 * Scoring très simple :
 * - On considère un rayon max (ex : 170 mm) pour dire "dans la cible"
 * - On calcule la distance r = sqrt(x^2 + y^2)
 * - score simple = 100 - (r / Rmax)*100, clamp [0..100]
 *
 * Ça permet de voir varier le score quand on change (x,y).
 */
static int score_simple(double x_mm, double y_mm) {
    const double Rmax = 170.0; /* rayon approximatif de la zone utile (à ajuster) */
    double r = sqrt(x_mm * x_mm + y_mm * y_mm);
    if (r > Rmax) return 0;
    double s = 100.0 - (r / Rmax) * 100.0;
    if (s < 0.0) s = 0.0;
    if (s > 100.0) s = 100.0;
    return (int)(s + 0.5); /* arrondi */
}

typedef struct {
    AppBusClient* bus;
} Ctx;

/* Callback appelée quand on reçoit un message Pub */
static void on_bus_msg(const char* topic, const char* payload, size_t payload_len, void* user) {
    (void)payload_len;
    Ctx* ctx = (Ctx*)user;

    /* On ne traite que les impacts */
    if (strcmp(topic, TOPIC_EVT_IMPACT_TRIANG) != 0) return;

    /* Sécurité : payload peut être NULL, ici normalement non */
    if (!payload) return;

    unsigned long long impact_id = 0;
    double x = 0.0, y = 0.0, z = 0.0, quality = 0.0;

    int ok = 1;
    ok &= json_get_u64(payload, "impact_id", &impact_id);
    ok &= json_get_double(payload, "x_mm", &x);
    ok &= json_get_double(payload, "y_mm", &y);
    /* z_mm et quality : on essaie, mais si absent on continue */
    json_get_double(payload, "z_mm", &z);
    json_get_double(payload, "quality", &quality);

    if (!ok) {
        fprintf(stderr, "[scoring] payload invalide: %s\n", payload);
        return;
    }

    int score = score_simple(x, y);

    /*
     * Publication d'un "hit" simplifié.
     * Plus tard on remplacera par :
     *   ring, sector, score officiel, etc.
     */
    char out[512];
    snprintf(out, sizeof(out),
             "{"
             "\"impact_id\":%llu,"
             "\"score\":%d,"
             "\"x_mm\":%.2f,"
             "\"y_mm\":%.2f,"
             "\"z_mm\":%.2f,"
             "\"quality\":%.3f"
             "}",
             impact_id, score, x, y, z, quality);

    if (appbus_publish(ctx->bus, TOPIC_EVT_HIT_SCORED, out) != 0) {
        fprintf(stderr, "[scoring] erreur publish hit_scored\n");
        return;
    }

    printf("[scoring] HIT scored -> %s\n", out);
    fflush(stdout);
}

int main(int argc, char** argv) {
    const char* sock = (argc >= 2) ? argv[1] : APPBUS_DEFAULT_SOCK;

    printf("[scoring] démarrage, sock=%s\n", sock);

    AppBusClient* bus = appbus_connect(sock);
    if (!bus) {
        fprintf(stderr, "[scoring] impossible de se connecter au bus\n");
        return 2;
    }

    /* On s'abonne aux impacts triangulés */
    if (appbus_subscribe(bus, TOPIC_EVT_IMPACT_TRIANG) != 0) {
        fprintf(stderr, "[scoring] subscribe échoué\n");
        appbus_close(bus);
        return 3;
    }

    Ctx ctx = { .bus = bus };

    /* Boucle bloquante : attend les impacts */
    int rc = appbus_poll(bus, on_bus_msg, &ctx);

    appbus_close(bus);
    return (rc == 0) ? 0 : 4;
}