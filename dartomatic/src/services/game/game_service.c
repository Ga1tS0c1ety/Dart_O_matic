#include "appbus/client.h"
#include "ipc/appbus_ipc.h"
#include "appbus/topics.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_PLAYERS 4
#define DARTS_PER_TURN 3
#define MAX_HISTORY 512

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
/* Modèle joueur                                             */
/* ========================================================= */

typedef struct {
    char name[32];
    int score;
} PlayerState;

/* ========================================================= */
/* Historique des darts                                      */
/* ========================================================= */

typedef struct {
    unsigned long long impact_id; /* 0 si manuel */
    int manual;                   /* 0=auto, 1=manuel */
    int player_index;
    int round_index;              /* 1..max_rounds */
    int dart_in_turn;             /* 1..3 */
    int score;
} DartRecord;

/* ========================================================= */
/* Contexte de partie                                        */
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

    int waiting_board_clear;  /* 1 si attente retrait */

    PlayerState players[MAX_PLAYERS];

    DartRecord history[MAX_HISTORY];
    int history_count;
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

    g->history_count = 0;
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
 * Recalcule entièrement l'état de partie à partir de l'historique.
 * C'est la base propre pour supporter undo / override.
 */
static void game_recompute_from_history(GameCtx* g) {
    if (!g) return;

    for (int i = 0; i < MAX_PLAYERS; i++) {
        g->players[i].score = 0;
    }

    g->round = g->active ? 1 : 0;
    g->current_player = 0;
    g->current_dart = 1;
    g->last_hit = 0;
    g->last_impact_id = 0;
    g->waiting_board_clear = 0;

    if (!g->active) return;

    for (int i = 0; i < g->history_count; i++) {
        DartRecord* d = &g->history[i];

        if (d->player_index >= 0 && d->player_index < g->player_count) {
            g->players[d->player_index].score += d->score;
        }

        g->last_hit = d->score;
        g->last_impact_id = d->impact_id;

        /* avancer la position courante */
        g->current_dart++;

        if (g->current_dart > DARTS_PER_TURN) {
            g->current_dart = 1;
            g->current_player++;

            if (g->current_player >= g->player_count) {
                g->current_player = 0;
                g->round++;
            }

            if (g->round <= g->max_rounds) {
                g->waiting_board_clear = 1;
            }
        } else {
            g->waiting_board_clear = 0;
        }
    }

    /*
     * Si le dernier état correspond à une fin de tour (3e dart),
     * on reste en attente de board clear.
     */
    if (g->history_count > 0) {
        DartRecord* last = &g->history[g->history_count - 1];
        if (last->dart_in_turn == DARTS_PER_TURN) {
            g->waiting_board_clear = 1;
        }
    }

    if (g->round > g->max_rounds) {
        g->active = 0;
        g->waiting_board_clear = 0;
    }
}

/*
 * Ajoute un dart dans l'historique, puis recalcule l'état.
 */
static int game_add_dart(GameCtx* g, unsigned long long impact_id, int manual, int score) {
    if (!g || !g->active) return 0;
    if (g->history_count >= MAX_HISTORY) return 0;
    if (g->waiting_board_clear) return 0;

    DartRecord d;
    memset(&d, 0, sizeof(d));

    d.impact_id = impact_id;
    d.manual = manual;
    d.player_index = g->current_player;
    d.round_index = g->round;
    d.dart_in_turn = g->current_dart;
    d.score = score;

    g->history[g->history_count++] = d;

    game_recompute_from_history(g);
    return 1;
}

/*
 * Annule le dernier dart.
 */
static int game_undo_last(GameCtx* g) {
    if (!g || !g->active) return 0;
    if (g->history_count <= 0) return 0;

    g->history_count--;
    game_recompute_from_history(g);
    return 1;
}

/*
 * Override du dernier dart :
 * stratégie simple = undo + add manuel corrigé
 */
static int game_override_last(GameCtx* g, int score) {
    if (!g || !g->active) return 0;
    if (g->history_count <= 0) return 0;

    if (!game_undo_last(g)) return 0;
    return game_add_dart(g, 0ULL, 1, score);
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
             "\"history_count\":%d,"
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
             g->history_count,
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

    /* ---------------- START ---------------- */
    if (strcmp(topic, TOPIC_CMD_GAME_START) == 0) {
        game_start(g, payload);
        printf("[game] CMD start reçu -> nouvelle partie (%d joueurs)\n", g->player_count);
        publish_state(g);
        return;
    }

    /* ---------------- BOARD CLEAR ---------------- */
    if (strcmp(topic, TOPIC_CMD_BOARD_CLEAR_CONF) == 0) {
        if (g->active && g->waiting_board_clear) {
            g->waiting_board_clear = 0;
            printf("[game] board clear confirmé -> reprise\n");
            publish_state(g);
        }
        return;
    }

    /* ---------------- UNDO ---------------- */
    if (strcmp(topic, TOPIC_CMD_GAME_UNDO) == 0) {
        if (game_undo_last(g)) {
            printf("[game] undo dernier dart\n");
            publish_state(g);
        } else {
            printf("[game] undo impossible\n");
        }
        return;
    }

    /* ---------------- OVERRIDE LAST ---------------- */
    if (strcmp(topic, TOPIC_CMD_GAME_OVERRIDE_LAST) == 0) {
        int score = 0;
        if (!json_get_int(payload, "score", &score)) {
            fprintf(stderr, "[game] override_last invalide: %s\n", payload);
            return;
        }

        if (game_override_last(g, score)) {
            printf("[game] override_last -> score=%d\n", score);
            publish_state(g);
        } else {
            printf("[game] override impossible\n");
        }
        return;
    }

    /* ---------------- ADD MANUAL HIT ---------------- */
    if (strcmp(topic, TOPIC_CMD_GAME_ADD_MANUAL_HIT) == 0) {
        int score = 0;
        if (!json_get_int(payload, "score", &score)) {
            fprintf(stderr, "[game] add_manual_hit invalide: %s\n", payload);
            return;
        }

        if (game_add_dart(g, 0ULL, 1, score)) {
            printf("[game] add_manual_hit -> score=%d\n", score);
            publish_state(g);
        } else {
            printf("[game] add_manual_hit impossible\n");
        }
        return;
    }

    /* ---------------- HIT AUTO ---------------- */
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

        if (game_add_dart(g, impact_id, 0, score)) {
            printf("[game] HIT auto impact_id=%llu score=%d\n", impact_id, score);
            publish_state(g);
        } else {
            printf("[game] hit auto ignoré\n");
        }
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

    if (appbus_subscribe(bus, TOPIC_CMD_GAME_START) != 0 ||
        appbus_subscribe(bus, TOPIC_CMD_GAME_UNDO) != 0 ||
        appbus_subscribe(bus, TOPIC_CMD_GAME_OVERRIDE_LAST) != 0 ||
        appbus_subscribe(bus, TOPIC_CMD_GAME_ADD_MANUAL_HIT) != 0 ||
        appbus_subscribe(bus, TOPIC_CMD_BOARD_CLEAR_CONF) != 0 ||
        appbus_subscribe(bus, TOPIC_EVT_HIT_SCORED) != 0) {
        fprintf(stderr, "[game] erreur subscribe\n");
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