#include "appbus/client.h"
#include "ipc/appbus_ipc.h"
#include "appbus/topics.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#define UI_MAX_PLAYERS 8

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

typedef struct {
    char name[32];
    int score;
    int remaining;
} UiPlayer;

static int json_get_players(const char* json, UiPlayer* players, int max_players, const char* mode) {
    const char* p = strstr(json, "\"players\":[");
    if (!p) return 0;

    int count = 0;

    while ((p = strstr(p, "\"name\":\"")) != NULL && count < max_players) {
        p += strlen("\"name\":\"");

        const char* end_name = strchr(p, '"');
        if (!end_name) break;

        size_t n = (size_t)(end_name - p);
        if (n >= sizeof(players[count].name)) n = sizeof(players[count].name) - 1;

        memcpy(players[count].name, p, n);
        players[count].name[n] = '\0';

        players[count].score = 0;
        players[count].remaining = 0;

        if (strcmp(mode, "high_score") == 0) {
            const char* score_pos = strstr(end_name, "\"score\":");
            if (score_pos) {
                score_pos += strlen("\"score\":");
                sscanf(score_pos, "%d", &players[count].score);
            }
            p = end_name;
        } else {
            const char* rem_pos = strstr(end_name, "\"remaining\":");
            if (rem_pos) {
                rem_pos += strlen("\"remaining\":");
                sscanf(rem_pos, "%d", &players[count].remaining);
            }
            p = end_name;
        }

        count++;
    }

    return count;
}

typedef struct {
    AppBusClient* bus;
    pthread_mutex_t lock;

    int active;
    char mode[32];

    int round;
    int max_rounds;
    int player_count;
    int current_player;
    int current_dart;

    int last_hit;
    unsigned long long last_impact_id;
    int waiting_board_clear;
    int history_count;

    UiPlayer players[UI_MAX_PLAYERS];
    int players_count;

    int has_state;
} UiCtx;

static void ui_separator(void) {
    printf("==================================================\n");
}

static void ui_render(UiCtx* ctx) {
    pthread_mutex_lock(&ctx->lock);

    printf("\n");
    ui_separator();
    printf(" Dart'O'Matic - UI CLI\n");
    ui_separator();

    if (!ctx->has_state) {
        printf(" État : aucun état reçu\n");
        printf(" Commandes : [h] high  [1] 301  [5] 501  [u] undo  [o] override  [a] add_manual  [n] clear  [q] quit\n");
        printf("> ");
        fflush(stdout);
        pthread_mutex_unlock(&ctx->lock);
        return;
    }

    printf(" Partie active : %s\n", ctx->active ? "OUI" : "NON");
    printf(" Mode          : %s\n", ctx->mode[0] ? ctx->mode : "unknown");
    printf(" Manche        : %d / %d\n", ctx->round, ctx->max_rounds);
    printf(" Joueur courant: %d / %d\n", ctx->current_player + 1, ctx->player_count);
    printf(" Fléchette     : %d / 3\n", ctx->current_dart);
    printf(" Dernier hit   : %d\n", ctx->last_hit);
    printf(" Dernier impact: %llu\n", ctx->last_impact_id);
    printf(" Historique    : %d darts\n", ctx->history_count);
    printf(" Plateau libre : %s\n", ctx->waiting_board_clear ? "NON (retirer fléchettes)" : "OUI");

    ui_separator();
    printf(" Joueurs\n");
    ui_separator();

    if (ctx->players_count <= 0) {
        printf(" (aucun joueur)\n");
    } else {
        for (int i = 0; i < ctx->players_count; i++) {
            const char* marker = (i == ctx->current_player && ctx->active) ? " <==" : "";

            if (strcmp(ctx->mode, "high_score") == 0) {
                printf("  [%d] %-10s : score=%4d%s\n",
                       i + 1, ctx->players[i].name, ctx->players[i].score, marker);
            } else {
                printf("  [%d] %-10s : remaining=%4d%s\n",
                       i + 1, ctx->players[i].name, ctx->players[i].remaining, marker);
            }
        }
    }

    ui_separator();
    printf(" Commandes : [h] high  [1] 301  [5] 501  [u] undo  [o] override  [a] add_manual  [n] clear  [q] quit\n");
    printf("> ");
    fflush(stdout);

    pthread_mutex_unlock(&ctx->lock);
}

static void on_bus_msg(const char* topic, const char* payload, size_t payload_len, void* user) {
    (void)payload_len;

    UiCtx* ctx = (UiCtx*)user;
    if (strcmp(topic, TOPIC_EVT_GAME_STATE) != 0) return;
    if (!payload) return;

    int active = 0;
    int round = 0;
    int max_rounds = 0;
    int player_count = 0;
    int current_player = 0;
    int current_dart = 0;
    int last_hit = 0;
    int waiting_board_clear = 0;
    int history_count = 0;
    unsigned long long last_impact_id = 0;
    char mode[32] = {0};

    UiPlayer players[UI_MAX_PLAYERS];
    memset(players, 0, sizeof(players));

    json_get_int(payload, "active", &active);
    json_get_string(payload, "mode", mode, sizeof(mode));
    json_get_int(payload, "round", &round);
    json_get_int(payload, "max_rounds", &max_rounds);
    json_get_int(payload, "player_count", &player_count);
    json_get_int(payload, "current_player", &current_player);
    json_get_int(payload, "current_dart", &current_dart);
    json_get_int(payload, "last_hit", &last_hit);
    json_get_int(payload, "waiting_board_clear", &waiting_board_clear);
    json_get_int(payload, "history_count", &history_count);
    json_get_u64(payload, "last_impact_id", &last_impact_id);

    int parsed_players = json_get_players(payload, players, UI_MAX_PLAYERS, mode[0] ? mode : "high_score");

    pthread_mutex_lock(&ctx->lock);

    ctx->active = active;
    strncpy(ctx->mode, mode, sizeof(ctx->mode) - 1);
    ctx->mode[sizeof(ctx->mode) - 1] = '\0';

    ctx->round = round;
    ctx->max_rounds = max_rounds;
    ctx->player_count = player_count;
    ctx->current_player = current_player;
    ctx->current_dart = current_dart;
    ctx->last_hit = last_hit;
    ctx->waiting_board_clear = waiting_board_clear;
    ctx->history_count = history_count;
    ctx->last_impact_id = last_impact_id;

    ctx->players_count = parsed_players;
    for (int i = 0; i < parsed_players; i++) {
        ctx->players[i] = players[i];
    }

    ctx->has_state = 1;

    pthread_mutex_unlock(&ctx->lock);

    ui_render(ctx);
}

static void* rx_thread(void* arg) {
    UiCtx* ctx = (UiCtx*)arg;
    appbus_poll(ctx->bus, on_bus_msg, ctx);
    return NULL;
}

static void publish_score_command(AppBusClient* bus, const char* topic) {
    int score = 0;
    char ring[16] = {0};
    int sector = 0;

    printf("\n[UI] score ? ");
    fflush(stdout);
    if (scanf("%d", &score) != 1) {
        printf("[UI] saisie invalide\n> ");
        fflush(stdout);
        int c;
        while ((c = getchar()) != '\n' && c != EOF) {}
        return;
    }

    printf("[UI] ring ? (ex: SINGLE / DOUBLE / TRIPLE / BULL / BULLSEYE, vide=none) ");
    fflush(stdout);

    int c;
    while ((c = getchar()) != '\n' && c != EOF) {}

    if (!fgets(ring, sizeof(ring), stdin)) {
        ring[0] = '\0';
    } else {
        size_t len = strlen(ring);
        while (len > 0 && (ring[len - 1] == '\n' || ring[len - 1] == '\r')) {
            ring[--len] = '\0';
        }
    }

    printf("[UI] sector ? (0 si inconnu) ");
    fflush(stdout);
    if (scanf("%d", &sector) != 1) {
        sector = 0;
    }
    while ((c = getchar()) != '\n' && c != EOF) {}

    char payload[256];
    snprintf(payload, sizeof(payload),
             "{"
             "\"score\":%d,"
             "\"ring\":\"%s\","
             "\"sector\":%d"
             "}",
             score, ring, sector);

    if (appbus_publish(bus, topic, payload) != 0) {
        fprintf(stderr, "[UI] erreur publish %s\n", topic);
    } else {
        printf("[UI] envoyé %s %s\n> ", topic, payload);
        fflush(stdout);
    }
}

static void publish_start_mode(AppBusClient* bus, const char* mode) {
    char payload[128];
    snprintf(payload, sizeof(payload),
             "{"
             "\"mode\":\"%s\","
             "\"players\":2"
             "}",
             mode);

    if (appbus_publish(bus, TOPIC_CMD_GAME_START, payload) != 0) {
        fprintf(stderr, "[UI] erreur publish cmd/game/start\n");
    } else {
        printf("[UI] start envoyé mode=%s\n> ", mode);
        fflush(stdout);
    }
}

int main(int argc, char** argv) {
    const char* sock = (argc >= 2) ? argv[1] : APPBUS_DEFAULT_SOCK;

    printf("[UI] démarrage, sock=%s\n", sock);

    AppBusClient* bus = appbus_connect(sock);
    if (!bus) {
        fprintf(stderr, "[UI] impossible de se connecter au bus\n");
        return 2;
    }

    if (appbus_subscribe(bus, TOPIC_EVT_GAME_STATE) != 0) {
        fprintf(stderr, "[UI] subscribe evt/game/state échoué\n");
        appbus_close(bus);
        return 3;
    }

    UiCtx ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.bus = bus;
    pthread_mutex_init(&ctx.lock, NULL);

    pthread_t tid;
    if (pthread_create(&tid, NULL, rx_thread, &ctx) != 0) {
        fprintf(stderr, "[UI] erreur pthread_create\n");
        appbus_close(bus);
        pthread_mutex_destroy(&ctx.lock);
        return 4;
    }

    ui_render(&ctx);

    while (1) {
        int ch = getchar();
        if (ch == EOF) break;

        if (ch == 'h' || ch == 'H') {
            publish_start_mode(bus, "high_score");
        }
        else if (ch == '1') {
            publish_start_mode(bus, "301");
        }
        else if (ch == '5') {
            publish_start_mode(bus, "501");
        }
        else if (ch == 'u' || ch == 'U') {
            if (appbus_publish(bus, TOPIC_CMD_GAME_UNDO, "{}") != 0) {
                fprintf(stderr, "[UI] erreur publish cmd/game/undo\n");
            } else {
                printf("[UI] undo envoyé\n> ");
                fflush(stdout);
            }
        }
        else if (ch == 'o' || ch == 'O') {
            publish_score_command(bus, TOPIC_CMD_GAME_OVERRIDE_LAST);
        }
        else if (ch == 'a' || ch == 'A') {
            publish_score_command(bus, TOPIC_CMD_GAME_ADD_MANUAL_HIT);
        }
        else if (ch == 'n' || ch == 'N') {
            if (appbus_publish(bus, TOPIC_CMD_BOARD_CLEAR_CONF, "{}") != 0) {
                fprintf(stderr, "[UI] erreur publish cmd/board/clear_confirmed\n");
            } else {
                printf("[UI] board clear confirmé\n> ");
                fflush(stdout);
            }
        }
        else if (ch == 'q' || ch == 'Q') {
            printf("\n[UI] quit\n");
            break;
        }
        else if (ch == '\n' || ch == '\r') {
            /* ignore */
        }
        else {
            printf("[UI] commande inconnue '%c'\n> ", ch);
            fflush(stdout);
        }
    }

    appbus_close(bus);
    pthread_mutex_destroy(&ctx.lock);
    return 0;
}