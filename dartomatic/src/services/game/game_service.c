#include "appbus/client.h"
#include "ipc/appbus_ipc.h"
#include "appbus/topics.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_PLAYERS 4
#define DARTS_PER_TURN 3

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
/* Joueur                                                    */
/* ========================================================= */

typedef struct {
    char name[32];
    int score;
} PlayerState;

/* ========================================================= */
/* Contexte partie                                           */
/* ========================================================= */

typedef struct {
    AppBusClient* bus;

    int active;
    char mode[32];

    int round;
    int max_rounds;

    int player_count;
    int current_player;       /* 0..player_count-1 */
    int current_dart;         /* 1..3 */

    int last_hit;
    unsigned long long last_impact_id;

    int waiting_board_clear;  /* 1 si on attend retrait des fléchettes */

    PlayerState players[MAX_PLAYERS];
} GameCtx;

/* ========================================================= */
/* Helpers                                                   */
/* ========================================================= */

static void game_reset(GameCtx* g) {
    if (!g) return;

    g->active = 0;
    snprintf(g->mode, sizeof(g->mode), "high_score");

    g->round = 0;
    g->max_rounds = 10;

    g->player_count = 2;
    g->current_player = 0;
    g->current_dart = 1;

    g->last_hit = 0;
    g->last_impact_id = 0;

    g->waiting_board_clear = 0;

    for (int i = 0; i < MAX_PLAYERS; i++) {
        snprintf(g->players[i].name, sizeof(g->players[i].name), "P%d", i + 1);
        g->players[i].score = 0;
    }
}

static void game_start(GameCtx* g, const char* payload) {
    if (!g) return;

    game_reset(g);

    int players = 0;
    int max_rounds = 0;

    if (payload) {
        json_get_int(payload, "players", &players);
        json_get_int(payload, "max_rounds", &max_rounds);
    }

    if (players >= 1 && players <= MAX_PLAYERS) {
        g->player_count = players;
    }

    if (max_rounds > 0) {
        g->max_rounds = max_rounds;
    }

    g->active = 1;
    g->round = 1;
    g->current_player = 0;
    g->current_dart = 1;
    g->waiting_board_clear = 0;
}

/*
 * Fin de tour :
 * - on passe au joueur suivant
 * - ou à la manche suivante si on boucle
 * - mais on ne permet pas de continuer tant que le plateau n'est pas confirmé "clear"
 */
static void game_finish_turn(GameCtx* g) {
    if (!g) return;

    g->current_dart = 1;
    g->current_player++;

    if (g->current_player >= g->player_count) {
        g->current_player = 0;
        g->round++;
    }

    if (g->round > g->max_rounds) {
        g->active = 0;
        g->waiting_board_clear = 0;
        return;
    }

    g->waiting_board_clear = 1;
}

/* ========================================================= */
/* Publication état                                          */
/* ========================================================= */

static void publish_state(GameCtx* g) {
    char players_json[512];
    players_json[0] = '\0';
    strcat(players_json, "[");

    for (int i = 0; i < g->player_count; i++) {
        char one[128];
        snprintf(one, sizeof(one),
                 "%s{"
                 "\"name\":\"%s\","
                 "\"score\":%d"
                 "}",
                 (i > 0) ? "," : "",
                 g->players[i].name,
                 g->players[i].score);
        strncat(players_json, one, sizeof(players_json) - strlen(players_json) - 1);
    }

    strcat(players_json, "]");

    char out[1024];
    snprintf(out, sizeof(out),
             "{"
             "\"active\":%d,"
             "\"mode\":\"%s\","
             "\"round\":%d,"
             "\"max_rounds\":%d,"
             "\"player_count\":%d,"
             "\"current_player\":%d,"
             "\"current_dart\":%d,"
             "\"last_hit\":%d,"
             "\"last_impact_id\":%llu,"
             "\"waiting_board_clear\":%d,"
             "\"players\":%s"
             "}",
             g->active,
             g->mode,
             g->round,
             g->max_rounds,
             g->player_count,
             g->current_player,
             g->current_dart,
             g->last_hit,
             g->last_impact_id,
             g->waiting_board_clear,
             players_json);

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

    /* START */
    if (strcmp(topic, TOPIC_CMD_GAME_START) == 0) {
        game_start(g, payload);
        printf("[game] CMD start reçu -> nouvelle partie (%d joueurs)\n", g->player_count);
        publish_state(g);
        return;
    }

    /* Confirmation plateau libre */
    if (strcmp(topic, TOPIC_CMD_BOARD_CLEAR_CONF) == 0) {
        if (!g->active) return;

        if (g->waiting_board_clear) {
            g->waiting_board_clear = 0;
            printf("[game] board clear confirmé -> reprise normale\n");
            publish_state(g);
        }
        return;
    }

    /* HIT */
    if (strcmp(topic, TOPIC_EVT_HIT_SCORED) == 0) {
        if (!g->active) {
            printf("[game] hit reçu mais game inactive (ignore)\n");
            return;
        }

        if (g->waiting_board_clear) {
            printf("[game] hit reçu mais attente retrait fléchettes (ignore)\n");
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

        g->players[g->current_player].score += score;
        g->last_hit = score;
        g->last_impact_id = impact_id;

        printf("[game] HIT impact_id=%llu player=%s score=%d -> total_joueur=%d (round=%d dart=%d)\n",
               impact_id,
               g->players[g->current_player].name,
               score,
               g->players[g->current_player].score,
               g->round,
               g->current_dart);

        g->current_dart++;

        if (g->current_dart > DARTS_PER_TURN) {
            game_finish_turn(g);
        }

        publish_state(g);
        return;
    }
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

    if (appbus_subscribe(bus, TOPIC_CMD_GAME_START) != 0) {
        fprintf(stderr, "[game] subscribe cmd/game/start échoué\n");
        appbus_close(bus);
        return 3;
    }

    if (appbus_subscribe(bus, TOPIC_CMD_BOARD_CLEAR_CONF) != 0) {
        fprintf(stderr, "[game] subscribe cmd/board/clear_confirmed échoué\n");
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

    publish_state(&ctx);

    int rc = appbus_poll(bus, on_bus_msg, &ctx);

    appbus_close(bus);
    return (rc == 0) ? 0 : 4;
}