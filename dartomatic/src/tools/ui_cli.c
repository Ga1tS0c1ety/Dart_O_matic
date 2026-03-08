#include "appbus/client.h"
#include "ipc/appbus_ipc.h"
#include "appbus/topics.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

/*
 * ui_cli (V2)
 * -----------
 * UI terminal simple :
 *  - écoute evt/game/state
 *  - affiche l'état de jeu de manière lisible
 *  - touche 's' : start game
 *  - touche 'q' : quitter
 *
 * Architecture :
 *  - 1 thread RX : bloque dans appbus_poll et reçoit les états
 *  - thread principal : lit le clavier et publie les commandes
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
/* Modèle d'état UI                                          */
/* ========================================================= */

typedef struct {
    AppBusClient* bus;
    pthread_mutex_t lock;

    int active;
    char mode[32];

    int round;
    int max_rounds;
    int current_dart;

    int score_total;
    int last_hit;
    unsigned long long last_impact_id;

    int has_state;
} UiCtx;

/* ========================================================= */
/* Affichage                                                 */
/* ========================================================= */

static void ui_print_separator(void) {
    printf("--------------------------------------------------\n");
}

static void ui_render(UiCtx* ctx) {
    pthread_mutex_lock(&ctx->lock);

    printf("\n");
    ui_print_separator();
    printf(" Dart'O'Matic - UI CLI\n");
    ui_print_separator();

    if (!ctx->has_state) {
        printf(" État : aucun état reçu pour l'instant\n");
    } else {
        printf(" Partie active : %s\n", ctx->active ? "OUI" : "NON");
        printf(" Mode          : %s\n", ctx->mode[0] ? ctx->mode : "unknown");
        printf(" Manche        : %d / %d\n", ctx->round, ctx->max_rounds);
        printf(" Fléchette     : %d / 3\n", ctx->current_dart);
        printf(" Score total   : %d\n", ctx->score_total);
        printf(" Dernier hit   : %d\n", ctx->last_hit);
        printf(" Dernier impact: %llu\n", ctx->last_impact_id);
    }

    ui_print_separator();
    printf(" Commandes : [s] start   [q] quit\n");
    printf("> ");
    fflush(stdout);

    pthread_mutex_unlock(&ctx->lock);
}

/* ========================================================= */
/* Callback AppBus                                           */
/* ========================================================= */

static void on_bus_msg(const char* topic, const char* payload, size_t payload_len, void* user) {
    (void)payload_len;

    UiCtx* ctx = (UiCtx*)user;

    if (strcmp(topic, TOPIC_EVT_GAME_STATE) != 0) return;
    if (!payload) return;

    int active = 0;
    int round = 0;
    int max_rounds = 0;
    int current_dart = 0;
    int score_total = 0;
    int last_hit = 0;
    unsigned long long last_impact_id = 0;
    char mode[32] = {0};

    /* On parse les champs utiles */
    json_get_int(payload, "active", &active);
    json_get_string(payload, "mode", mode, sizeof(mode));
    json_get_int(payload, "round", &round);
    json_get_int(payload, "max_rounds", &max_rounds);
    json_get_int(payload, "current_dart", &current_dart);
    json_get_int(payload, "score_total", &score_total);
    json_get_int(payload, "last_hit", &last_hit);
    json_get_u64(payload, "last_impact_id", &last_impact_id);

    pthread_mutex_lock(&ctx->lock);

    ctx->active = active;
    strncpy(ctx->mode, mode, sizeof(ctx->mode) - 1);
    ctx->mode[sizeof(ctx->mode) - 1] = '\0';

    ctx->round = round;
    ctx->max_rounds = max_rounds;
    ctx->current_dart = current_dart;
    ctx->score_total = score_total;
    ctx->last_hit = last_hit;
    ctx->last_impact_id = last_impact_id;
    ctx->has_state = 1;

    pthread_mutex_unlock(&ctx->lock);

    ui_render(ctx);
}

/* ========================================================= */
/* Thread réception                                          */
/* ========================================================= */

static void* rx_thread(void* arg) {
    UiCtx* ctx = (UiCtx*)arg;
    appbus_poll(ctx->bus, on_bus_msg, ctx);
    return NULL;
}

/* ========================================================= */
/* main                                                      */
/* ========================================================= */

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

    /* Thread RX */
    pthread_t tid;
    if (pthread_create(&tid, NULL, rx_thread, &ctx) != 0) {
        fprintf(stderr, "[UI] erreur pthread_create\n");
        appbus_close(bus);
        pthread_mutex_destroy(&ctx.lock);
        return 4;
    }

    ui_render(&ctx);

    /* Boucle clavier */
    while (1) {
        int ch = getchar();
        if (ch == EOF) break;

        if (ch == 's' || ch == 'S') {
            /*
             * Start game.
             * On garde "{}" pour rester compatible avec le game_service actuel.
             * Plus tard on pourra envoyer :
             *   {"mode":"high_score","max_rounds":10}
             */
            if (appbus_publish(bus, TOPIC_CMD_GAME_START, "{}") != 0) {
                fprintf(stderr, "[UI] erreur publish cmd/game/start\n");
            } else {
                printf("[UI] start envoyé\n");
                printf("> ");
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
            printf("[UI] commande inconnue '%c'\n", ch);
            printf("> ");
            fflush(stdout);
        }
    }

    /*
     * V2 : on reste simple.
     * Le thread RX sera arrêté quand le process se termine.
     */
    appbus_close(bus);
    pthread_mutex_destroy(&ctx.lock);
    return 0;
}