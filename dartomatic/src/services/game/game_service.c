#include "appbus/client.h"
#include "ipc/appbus_ipc.h"
#include "appbus/topics.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_PLAYERS 4
#define DARTS_PER_TURN 3
#define MAX_HISTORY 512
#define CRICKET_TARGETS 7

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

static int json_get_string(const char* json, const char* key, char* out, size_t out_sz) {
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\":\"", key);
    const char* p = strstr(json, pat);
    if (!p) return 0;
    p += strlen(pat);

    const char* end = strchr(p, '"');
    if (!end) return 0;

    size_t n = (size_t)(end - p);
    if (n + 1 > out_sz) n = out_sz - 1;

    memcpy(out, p, n);
    out[n] = '\0';
    return 1;
}

/* ========================================================= */
/* Types métier                                              */
/* ========================================================= */

typedef enum {
    GAME_MODE_NONE = 0,
    GAME_MODE_HIGH_SCORE,
    GAME_MODE_301,
    GAME_MODE_501,
    GAME_MODE_CRICKET
} GameMode;

typedef struct {
    char name[32];

    /* High score */
    int score;

    /* X01 */
    int remaining;

    /* Cricket */
    int cricket_score;
    int cricket_marks[CRICKET_TARGETS]; /* 20,19,18,17,16,15,bull */
} PlayerState;

typedef struct {
    unsigned long long impact_id;
    int manual;

    int player_index;
    int round_index;
    int dart_in_turn;

    int score;
    char ring[16];
    int sector;

    int bust;
} DartRecord;

typedef struct {
    AppBusClient* bus;

    int active;
    GameMode mode;
    char mode_name[32];

    int round;
    int max_rounds;

    int player_count;
    int current_player;
    int current_dart;

    int waiting_board_clear;

    int last_hit;
    unsigned long long last_impact_id;

    PlayerState players[MAX_PLAYERS];

    DartRecord history[MAX_HISTORY];
    int history_count;
} GameCtx;

/* ========================================================= */
/* Helpers mode                                              */
/* ========================================================= */

static const char* mode_to_string(GameMode m) {
    switch (m) {
        case GAME_MODE_HIGH_SCORE: return "high_score";
        case GAME_MODE_301:        return "301";
        case GAME_MODE_501:        return "501";
        case GAME_MODE_CRICKET:    return "cricket";
        default:                   return "none";
    }
}

static GameMode mode_from_string(const char* s) {
    if (!s) return GAME_MODE_HIGH_SCORE;
    if (strcmp(s, "301") == 0) return GAME_MODE_301;
    if (strcmp(s, "501") == 0) return GAME_MODE_501;
    if (strcmp(s, "cricket") == 0) return GAME_MODE_CRICKET;
    if (strcmp(s, "high_score") == 0) return GAME_MODE_HIGH_SCORE;
    return GAME_MODE_HIGH_SCORE;
}

static int mode_start_score(GameMode m) {
    if (m == GAME_MODE_301) return 301;
    if (m == GAME_MODE_501) return 501;
    return 0;
}

static int ring_is_double(const char* ring) {
    if (!ring) return 0;
    return (strcmp(ring, "DOUBLE") == 0 || strcmp(ring, "BULLSEYE") == 0);
}

/* ========================================================= */
/* Helpers cricket                                           */
/* ========================================================= */

/* ordre interne : 20,19,18,17,16,15,bull */
static int cricket_target_index(int sector) {
    switch (sector) {
        case 20: return 0;
        case 19: return 1;
        case 18: return 2;
        case 17: return 3;
        case 16: return 4;
        case 15: return 5;
        case 25:
        case 50: return 6;
        default: return -1;
    }
}

static int cricket_marks_from_hit(const char* ring, int sector) {
    if (sector == 25 || sector == 50) {
        if (strcmp(ring, "BULLSEYE") == 0) return 2;
        return 1;
    }

    if (strcmp(ring, "TRIPLE") == 0) return 3;
    if (strcmp(ring, "DOUBLE") == 0) return 2;
    if (strcmp(ring, "SINGLE") == 0) return 1;
    return 0;
}

static int cricket_target_value(int idx) {
    static const int vals[CRICKET_TARGETS] = {20, 19, 18, 17, 16, 15, 25};
    return vals[idx];
}

static int cricket_all_closed(const PlayerState* p) {
    for (int i = 0; i < CRICKET_TARGETS; i++) {
        if (p->cricket_marks[i] < 3) return 0;
    }
    return 1;
}

static int cricket_has_highest_or_equal_score(const GameCtx* g, int player_index) {
    int my = g->players[player_index].cricket_score;
    for (int i = 0; i < g->player_count; i++) {
        if (i == player_index) continue;
        if (my < g->players[i].cricket_score) return 0;
    }
    return 1;
}

/* ========================================================= */
/* Reset / start                                             */
/* ========================================================= */

static void game_reset(GameCtx* g) {
    if (!g) return;

    g->active = 0;
    g->mode = GAME_MODE_HIGH_SCORE;
    snprintf(g->mode_name, sizeof(g->mode_name), "%s", mode_to_string(g->mode));

    g->round = 0;
    g->max_rounds = 10;

    g->player_count = 2;
    g->current_player = 0;
    g->current_dart = 1;

    g->waiting_board_clear = 0;

    g->last_hit = 0;
    g->last_impact_id = 0;

    for (int i = 0; i < MAX_PLAYERS; i++) {
        snprintf(g->players[i].name, sizeof(g->players[i].name), "P%d", i + 1);
        g->players[i].score = 0;
        g->players[i].remaining = 0;
        g->players[i].cricket_score = 0;
        memset(g->players[i].cricket_marks, 0, sizeof(g->players[i].cricket_marks));
    }

    g->history_count = 0;
}

static void game_start(GameCtx* g, const char* payload) {
    if (!g) return;

    game_reset(g);

    int players = 0;
    int max_rounds = 0;
    char mode_str[32] = {0};

    if (payload) {
        json_get_int(payload, "players", &players);
        json_get_int(payload, "max_rounds", &max_rounds);
        json_get_string(payload, "mode", mode_str, sizeof(mode_str));
    }

    if (players >= 1 && players <= MAX_PLAYERS) {
        g->player_count = players;
    }

    if (max_rounds > 0) {
        g->max_rounds = max_rounds;
    }

    g->mode = mode_from_string(mode_str[0] ? mode_str : "high_score");
    snprintf(g->mode_name, sizeof(g->mode_name), "%s", mode_to_string(g->mode));

    g->active = 1;
    g->round = 1;
    g->current_player = 0;
    g->current_dart = 1;
    g->waiting_board_clear = 0;

    if (g->mode == GAME_MODE_301 || g->mode == GAME_MODE_501) {
        int start_score = mode_start_score(g->mode);
        for (int i = 0; i < g->player_count; i++) {
            g->players[i].remaining = start_score;
        }
        if (max_rounds <= 0) g->max_rounds = 99;
    }

    if (g->mode == GAME_MODE_CRICKET) {
        if (max_rounds <= 0) g->max_rounds = 99;
    }
}

/* ========================================================= */
/* Publication état                                          */
/* ========================================================= */

static void publish_state(GameCtx* g) {
    char players_json[2048];
    players_json[0] = '\0';
    strcat(players_json, "[");

    for (int i = 0; i < g->player_count; i++) {
        char one[384];

        if (g->mode == GAME_MODE_HIGH_SCORE) {
            snprintf(one, sizeof(one),
                     "%s{\"name\":\"%s\",\"score\":%d}",
                     (i > 0) ? "," : "",
                     g->players[i].name,
                     g->players[i].score);
        }
        else if (g->mode == GAME_MODE_301 || g->mode == GAME_MODE_501) {
            snprintf(one, sizeof(one),
                     "%s{\"name\":\"%s\",\"remaining\":%d}",
                     (i > 0) ? "," : "",
                     g->players[i].name,
                     g->players[i].remaining);
        }
        else {
            snprintf(one, sizeof(one),
                     "%s{\"name\":\"%s\",\"score\":%d,"
                     "\"marks\":[%d,%d,%d,%d,%d,%d,%d]}",
                     (i > 0) ? "," : "",
                     g->players[i].name,
                     g->players[i].cricket_score,
                     g->players[i].cricket_marks[0],
                     g->players[i].cricket_marks[1],
                     g->players[i].cricket_marks[2],
                     g->players[i].cricket_marks[3],
                     g->players[i].cricket_marks[4],
                     g->players[i].cricket_marks[5],
                     g->players[i].cricket_marks[6]);
        }

        strncat(players_json, one, sizeof(players_json) - strlen(players_json) - 1);
    }

    strcat(players_json, "]");

    char out[4096];
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
             g->mode_name,
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
/* Recalcul historique                                       */
/* ========================================================= */

static void game_finish_turn(GameCtx* g) {
    g->current_dart = 1;
    g->current_player++;

    if (g->current_player >= g->player_count) {
        g->current_player = 0;
        g->round++;
    }

    if (g->mode == GAME_MODE_HIGH_SCORE) {
        if (g->round > g->max_rounds) {
            g->active = 0;
            g->waiting_board_clear = 0;
            return;
        }
    }

    g->waiting_board_clear = 1;
}

static void game_recompute_from_history(GameCtx* g) {
    if (!g) return;

    for (int i = 0; i < MAX_PLAYERS; i++) {
        g->players[i].score = 0;
        g->players[i].cricket_score = 0;
        memset(g->players[i].cricket_marks, 0, sizeof(g->players[i].cricket_marks));
    }

    if (g->mode == GAME_MODE_301 || g->mode == GAME_MODE_501) {
        int start_score = mode_start_score(g->mode);
        for (int i = 0; i < g->player_count; i++) {
            g->players[i].remaining = start_score;
        }
    }

    g->round = g->active ? 1 : 0;
    g->current_player = 0;
    g->current_dart = 1;
    g->last_hit = 0;
    g->last_impact_id = 0;
    g->waiting_board_clear = 0;

    if (!g->active) return;

    int turn_start_remaining[MAX_PLAYERS];
    for (int i = 0; i < g->player_count; i++) {
        turn_start_remaining[i] = g->players[i].remaining;
    }

    for (int i = 0; i < g->history_count; i++) {
        DartRecord* d = &g->history[i];
        int p = d->player_index;

        g->last_hit = d->score;
        g->last_impact_id = d->impact_id;

        if (g->mode == GAME_MODE_HIGH_SCORE) {
            g->players[p].score += d->score;
        }
        else if (g->mode == GAME_MODE_301 || g->mode == GAME_MODE_501) {
            if (d->dart_in_turn == 1) {
                turn_start_remaining[p] = g->players[p].remaining;
            }

            int after = g->players[p].remaining - d->score;
            int bust = 0;
            int finish = 0;

            if (after < 0) {
                bust = 1;
            } else if (after == 1) {
                bust = 1;
            } else if (after == 0) {
                if (ring_is_double(d->ring)) {
                    finish = 1;
                } else {
                    bust = 1;
                }
            }

            if (bust) {
                g->players[p].remaining = turn_start_remaining[p];
                d->bust = 1;
                g->current_dart = DARTS_PER_TURN;
            } else {
                g->players[p].remaining = after;
                d->bust = 0;

                if (finish) {
                    g->active = 0;
                    g->waiting_board_clear = 0;
                    return;
                }
            }
        }
        else if (g->mode == GAME_MODE_CRICKET) {
            int idx = cricket_target_index(d->sector);
            if (idx >= 0) {
                int marks = cricket_marks_from_hit(d->ring, d->sector);
                int old_marks = g->players[p].cricket_marks[idx];
                int new_marks = old_marks + marks;

                if (new_marks <= 3) {
                    g->players[p].cricket_marks[idx] = new_marks;
                } else {
                    g->players[p].cricket_marks[idx] = 3;

                    int extra = new_marks - 3;
                    int all_closed_by_others = 1;
                    for (int j = 0; j < g->player_count; j++) {
                        if (j == p) continue;
                        if (g->players[j].cricket_marks[idx] < 3) {
                            all_closed_by_others = 0;
                            break;
                        }
                    }

                    if (!all_closed_by_others) {
                        g->players[p].cricket_score += extra * cricket_target_value(idx);
                    }
                }
            }

            if (cricket_all_closed(&g->players[p]) &&
                cricket_has_highest_or_equal_score(g, p)) {
                g->active = 0;
                g->waiting_board_clear = 0;
                return;
            }
        }

        g->current_dart++;

        if (g->current_dart > DARTS_PER_TURN || d->bust) {
            game_finish_turn(g);
        } else {
            g->waiting_board_clear = 0;
        }
    }

    if (g->history_count > 0) {
        DartRecord* last = &g->history[g->history_count - 1];
        if (last->dart_in_turn == DARTS_PER_TURN || last->bust) {
            g->waiting_board_clear = 1;
        }
    }
}

/* ========================================================= */
/* Ajout / undo / override                                   */
/* ========================================================= */

static int game_add_dart(GameCtx* g,
                         unsigned long long impact_id,
                         int manual,
                         int score,
                         const char* ring,
                         int sector) {
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
    d.sector = sector;
    snprintf(d.ring, sizeof(d.ring), "%s", ring ? ring : "");

    g->history[g->history_count++] = d;
    game_recompute_from_history(g);
    return 1;
}

static int game_undo_last(GameCtx* g) {
    if (!g || !g->active) return 0;
    if (g->history_count <= 0) return 0;

    g->history_count--;
    game_recompute_from_history(g);
    return 1;
}

static int game_override_last(GameCtx* g, int score, const char* ring, int sector) {
    if (!g || !g->active) return 0;
    if (g->history_count <= 0) return 0;

    if (!game_undo_last(g)) return 0;
    return game_add_dart(g, 0ULL, 1, score, ring, sector);
}

/* ========================================================= */
/* Callback AppBus                                           */
/* ========================================================= */

static void on_bus_msg(const char* topic, const char* payload, size_t payload_len, void* user) {
    (void)payload_len;
    GameCtx* g = (GameCtx*)user;
    if (!payload) payload = "";

    if (strcmp(topic, TOPIC_CMD_GAME_START) == 0) {
        game_start(g, payload);
        printf("[game] CMD start reçu -> mode=%s joueurs=%d\n", g->mode_name, g->player_count);
        publish_state(g);
        return;
    }

    if (strcmp(topic, TOPIC_CMD_BOARD_CLEAR_CONF) == 0) {
        if (g->active && g->waiting_board_clear) {
            g->waiting_board_clear = 0;
            printf("[game] board clear confirmé -> reprise\n");
            publish_state(g);
        }
        return;
    }

    if (strcmp(topic, TOPIC_CMD_GAME_UNDO) == 0) {
        if (game_undo_last(g)) {
            printf("[game] undo dernier dart\n");
            publish_state(g);
        } else {
            printf("[game] undo impossible\n");
        }
        return;
    }

    if (strcmp(topic, TOPIC_CMD_GAME_OVERRIDE_LAST) == 0) {
        int score = 0;
        int sector = 0;
        char ring[16] = {0};

        if (!json_get_int(payload, "score", &score)) {
            fprintf(stderr, "[game] override_last invalide: %s\n", payload);
            return;
        }
        json_get_int(payload, "sector", &sector);
        json_get_string(payload, "ring", ring, sizeof(ring));

        if (game_override_last(g, score, ring, sector)) {
            printf("[game] override_last -> score=%d ring=%s sector=%d\n", score, ring, sector);
            publish_state(g);
        } else {
            printf("[game] override impossible\n");
        }
        return;
    }

    if (strcmp(topic, TOPIC_CMD_GAME_ADD_MANUAL_HIT) == 0) {
        int score = 0;
        int sector = 0;
        char ring[16] = {0};

        if (!json_get_int(payload, "score", &score)) {
            fprintf(stderr, "[game] add_manual_hit invalide: %s\n", payload);
            return;
        }
        json_get_int(payload, "sector", &sector);
        json_get_string(payload, "ring", ring, sizeof(ring));

        if (game_add_dart(g, 0ULL, 1, score, ring, sector)) {
            printf("[game] add_manual_hit -> score=%d ring=%s sector=%d\n", score, ring, sector);
            publish_state(g);
        } else {
            printf("[game] add_manual_hit impossible\n");
        }
        return;
    }

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
        int sector = 0;
        unsigned long long impact_id = 0;
        char ring[16] = {0};

        int ok = 1;
        ok &= json_get_int(payload, "score", &score);
        json_get_u64(payload, "impact_id", &impact_id);
        json_get_int(payload, "sector", &sector);
        json_get_string(payload, "ring", ring, sizeof(ring));

        if (!ok) {
            fprintf(stderr, "[game] payload hit invalide: %s\n", payload);
            return;
        }

        if (game_add_dart(g, impact_id, 0, score, ring, sector)) {
            printf("[game] HIT auto impact_id=%llu score=%d ring=%s sector=%d\n",
                   impact_id, score, ring, sector);
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