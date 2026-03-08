#include "appbus/client.h"
#include "ipc/appbus_ipc.h"
#include "appbus/topics.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * game_service (V2)
 * -----------------
 * Rôle :
 *  - écouter cmd/game/start
 *  - écouter evt/hit/scored
 *  - maintenir un état de partie minimal mais réaliste
 *  - publier evt/game/state
 *
 * Mode V2 :
 *  - high_score solo
 *  - 10 manches par défaut
 *  - 3 fléchettes par manche
 *
 * Pourquoi ce mode ?
 *  - il reste très simple
 *  - il valide la vraie chaîne métier :
 *      RT -> scoring -> game -> UI
 *  - il prépare le terrain pour 501 / cricket plus tard
 */

/* ========================================================= */
/* Parsing JSON minimal                                      */
/* ========================================================= */

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

/* ========================================================= */
/* Contexte partie                                           */
/* ========================================================= */

typedef struct {
    AppBusClient* bus;

    /* État global */
    int active;            /* 1 si partie en cours */
    char mode[32];         /* ex: "high_score" */

    /* Progression */
    int round;             /* manche courante (1..max_rounds) */
    int max_rounds;        /* nombre total de manches */
    int current_dart;      /* 1..3 */

    /* Score */
    int score_total;       /* score cumulé */
    int last_hit;          /* dernier score reçu */

    /* Debug / suivi */
    unsigned long long last_impact_id;
} GameCtx;

/* ========================================================= */
/* Helpers                                                   */
/* ========================================================= */

/*
 * Réinitialise complètement l'état de partie.
 */
static void game_reset(GameCtx* g) {
    if (!g) return;

    g->active = 0;
    snprintf(g->mode, sizeof(g->mode), "high_score");

    g->round = 0;
    g->max_rounds = 10;
    g->current_dart = 1;

    g->score_total = 0;
    g->last_hit = 0;
    g->last_impact_id = 0;
}

/*
 * Démarre une nouvelle partie.
 * Payload JSON futur possible :
 *   {"max_rounds":5}
 *
 * Pour rester compatible avec ton UI actuelle, si payload = {},
 * on démarre avec les valeurs par défaut.
 */
static void game_start(GameCtx* g, const char* payload) {
    if (!g) return;

    game_reset(g);

    g->active = 1;
    g->round = 1;
    g->current_dart = 1;

    /* Optionnel : lecture d'un max_rounds si fourni */
    int max_rounds = 0;
    if (payload && json_get_int(payload, "max_rounds", &max_rounds)) {
        if (max_rounds > 0) {
            g->max_rounds = max_rounds;
        }
    }
}

/*
 * Passe au tir suivant.
 * Si on a déjà lancé 3 fléchettes, on passe à la manche suivante.
 * Si la manche dépasse max_rounds, la partie se termine.
 */
static void game_advance_after_hit(GameCtx* g) {
    if (!g) return;

    g->current_dart++;

    if (g->current_dart > 3) {
        g->current_dart = 1;
        g->round++;
    }

    if (g->round > g->max_rounds) {
        g->active = 0;
    }
}

/* ========================================================= */
/* Publication état                                          */
/* ========================================================= */

static void publish_state(GameCtx* g) {
    char out[512];

    snprintf(out, sizeof(out),
             "{"
             "\"active\":%d,"
             "\"mode\":\"%s\","
             "\"round\":%d,"
             "\"max_rounds\":%d,"
             "\"current_dart\":%d,"
             "\"score_total\":%d,"
             "\"last_hit\":%d,"
             "\"last_impact_id\":%llu"
             "}",
             g->active,
             g->mode,
             g->round,
             g->max_rounds,
             g->current_dart,
             g->score_total,
             g->last_hit,
             g->last_impact_id);

    if (appbus_publish(g->bus, TOPIC_EVT_GAME_STATE, out) != 0) {
        fprintf(stderr, "[game] erreur publish evt/game/state\n");
        return;
    }

    printf("[game] STATE -> %s\n", out);
    fflush(stdout);
}

/* ========================================================= */
/* Callback AppBus                                           */
/* ========================================================= */

static void on_bus_msg(const char* topic, const char* payload, size_t payload_len, void* user) {
    (void)payload_len;

    GameCtx* g = (GameCtx*)user;
    if (!payload) payload = "";

    /* ----------------------------------------------------- */
    /* Commande START                                        */
    /* ----------------------------------------------------- */
    if (strcmp(topic, TOPIC_CMD_GAME_START) == 0) {
        game_start(g, payload);

        printf("[game] CMD start reçu -> nouvelle partie\n");
        publish_state(g);
        return;
    }

    /* ----------------------------------------------------- */
    /* Hit scored                                            */
    /* ----------------------------------------------------- */
    if (strcmp(topic, TOPIC_EVT_HIT_SCORED) == 0) {
        if (!g->active) {
            /*
             * On ignore les hits si aucune partie n'est en cours.
             * C'est utile si RT tourne en permanence mais qu'on n'a pas encore "start".
             */
            printf("[game] hit reçu mais game inactive (ignore)\n");
            return;
        }

        int score = 0;
        unsigned long long impact_id = 0;

        int ok = 1;
        ok &= json_get_int(payload, "score", &score);
        json_get_u64(payload, "impact_id", &impact_id);

        if (!ok) {
            fprintf(stderr, "[game] payload hit invalide: %s\n", payload);
            return;
        }

        /*
         * Mise à jour de l'état métier :
         *  - on additionne le score
         *  - on garde le dernier hit
         *  - on avance le dart / round
         */
        g->score_total += score;
        g->last_hit = score;
        g->last_impact_id = impact_id;

        printf("[game] HIT impact_id=%llu score=%d -> total=%d (round=%d dart=%d)\n",
               impact_id, score, g->score_total, g->round, g->current_dart);

        game_advance_after_hit(g);
        publish_state(g);
        return;
    }

    /* ----------------------------------------------------- */
    /* Autres topics ignorés                                 */
    /* ----------------------------------------------------- */
}

/* ========================================================= */
/* main                                                      */
/* ========================================================= */

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
    game_reset(&ctx);

    /* Publie un état initial */
    publish_state(&ctx);

    /* Boucle bloquante */
    int rc = appbus_poll(bus, on_bus_msg, &ctx);

    appbus_close(bus);
    return (rc == 0) ? 0 : 4;
}