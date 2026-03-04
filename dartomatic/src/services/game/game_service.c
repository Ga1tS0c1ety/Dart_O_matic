#include "appbus/client.h"
#include "ipc/appbus_ipc.h"
#include "appbus/topics.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * game_service (V1)
 * -----------------
 * Rôle :
 *  - écouter cmd/game/start : initialise (ou réinitialise) une partie
 *  - écouter evt/hit/scored : ajoute le score au total si partie active
 *  - publier evt/game/state : état courant (score_total, nb_tirs, active)
 *
 * NOTE :
 *  - Ici la logique de fléchettes (301/501/bust/double-out) n'est PAS encore implémentée.
 *  - On valide juste le pipeline: UI -> start, Impact -> Score -> GameState -> UI.
 */

/* Parsing JSON minimal (comme scoring_service) */
static int json_get_int(const char* json, const char* key, int* out) {
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\":", key);
    const char* p = strstr(json, pat);
    if (!p) return 0;
    p += strlen(pat);
    return (sscanf(p, "%d", out) == 1);
}

static int json_get_u64(const char* json, const char* key, unsigned long long* out) {
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\":", key);
    const char* p = strstr(json, pat);
    if (!p) return 0;
    p += strlen(pat);
    return (sscanf(p, "%llu", out) == 1);
}

typedef struct {
    AppBusClient* bus;

    int active;          /* 1 si partie en cours */
    int total_score;     /* somme des scores */
    int throws_count;    /* nombre de tirs pris en compte */
} GameCtx;

static void publish_state(GameCtx* g) {
    char out[256];
    snprintf(out, sizeof(out),
             "{"
             "\"active\":%d,"
             "\"total_score\":%d,"
             "\"throws\":%d"
             "}",
             g->active, g->total_score, g->throws_count);

    if (appbus_publish(g->bus, TOPIC_EVT_GAME_STATE, out) != 0) {
        fprintf(stderr, "[game] erreur publish evt/game/state\n");
        return;
    }

    printf("[game] STATE -> %s\n", out);
    fflush(stdout);
}

static void on_bus_msg(const char* topic, const char* payload, size_t payload_len, void* user) {
    (void)payload_len;
    GameCtx* g = (GameCtx*)user;
    if (!payload) payload = "";

    /* Commande start */
    if (strcmp(topic, TOPIC_CMD_GAME_START) == 0) {
        /*
         * V1 : on accepte start sans paramètres.
         * Si tu veux, plus tard tu mettras mode/joueurs dans le payload.
         */
        g->active = 1;
        g->total_score = 0;
        g->throws_count = 0;

        printf("[game] CMD start reçu -> reset game\n");
        publish_state(g);
        return;
    }

    /* Score d'un tir */
    if (strcmp(topic, TOPIC_EVT_HIT_SCORED) == 0) {
        if (!g->active) {
            /* Tant que la partie n'est pas démarrée, on ignore */
            printf("[game] hit reçu mais game inactive (ignore)\n");
            return;
        }

        int score = 0;
        unsigned long long impact_id = 0;

        int ok = 1;
        ok &= json_get_int(payload, "score", &score);
        json_get_u64(payload, "impact_id", &impact_id); /* optionnel */

        if (!ok) {
            fprintf(stderr, "[game] payload hit invalide: %s\n", payload);
            return;
        }

        g->total_score += score;
        g->throws_count += 1;

        printf("[game] HIT impact_id=%llu score=%d -> total=%d\n",
               impact_id, score, g->total_score);

        publish_state(g);
        return;
    }

    /* autres topics ignorés */
}

int main(int argc, char** argv) {
    const char* sock = (argc >= 2) ? argv[1] : APPBUS_DEFAULT_SOCK;

    printf("[game] démarrage, sock=%s\n", sock);

    AppBusClient* bus = appbus_connect(sock);
    if (!bus) {
        fprintf(stderr, "[game] impossible de se connecter au bus\n");
        return 2;
    }

    /* Abonnements */
    if (appbus_subscribe(bus, TOPIC_CMD_GAME_START) != 0) {
        fprintf(stderr, "[game] subscribe cmd/game/start échoué\n");
        appbus_close(bus);
        return 3;
    }
    if (appbus_subscribe(bus, TOPIC_EVT_HIT_SCORED) != 0) {
        fprintf(stderr, "[game] subscribe evt/hit/scored échoué\n");
        appbus_close(bus);
        return 3;
    }

    GameCtx ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.bus = bus;
    ctx.active = 0;
    ctx.total_score = 0;
    ctx.throws_count = 0;

    /* Publie un état initial */
    publish_state(&ctx);

    /* Boucle bloquante */
    int rc = appbus_poll(bus, on_bus_msg, &ctx);

    appbus_close(bus);
    return (rc == 0) ? 0 : 4;
}