#include "appbus/client.h"
#include "ipc/appbus_ipc.h"
#include "appbus/topics.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

/*
 * ui_cli (V1)
 * -----------
 * UI en terminal, minimaliste :
 *  - affiche les evt/game/state
 *  - touche 's' : start game
 *  - touche 'q' : quitter
 *
 * Pour être simple :
 *  - 1 thread reçoit les messages (appbus_poll)
 *  - le thread principal lit le clavier et publie des commandes
 */

typedef struct {
    AppBusClient* bus;
} UiCtx;

/* Affiche tout message evt/game/state */
static void on_bus_msg(const char* topic, const char* payload, size_t payload_len, void* user) {
    (void)user;
    if (strcmp(topic, TOPIC_EVT_GAME_STATE) != 0) return;

    printf("\n[UI] GAME STATE -> %.*s\n", (int)payload_len, payload ? payload : "");
    printf("[UI] Commandes: (s) start  (q) quit\n> ");
    fflush(stdout);
}

/* Thread réception: bloque dans appbus_poll */
static void* rx_thread(void* arg) {
    UiCtx* ctx = (UiCtx*)arg;
    appbus_poll(ctx->bus, on_bus_msg, ctx);
    return NULL;
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

    UiCtx ctx = { .bus = bus };

    /* Thread qui écoute les messages du bus */
    pthread_t tid;
    if (pthread_create(&tid, NULL, rx_thread, &ctx) != 0) {
        fprintf(stderr, "[UI] erreur pthread_create\n");
        appbus_close(bus);
        return 4;
    }

    printf("[UI] Commandes: (s) start  (q) quit\n> ");
    fflush(stdout);

    /* Boucle clavier */
    while (1) {
        int ch = getchar();
        if (ch == EOF) break;

        if (ch == 's' || ch == 'S') {
            /* Publish start game */
            if (appbus_publish(bus, TOPIC_CMD_GAME_START, "{}") != 0) {
                fprintf(stderr, "[UI] erreur publish cmd/game/start\n");
            } else {
                printf("[UI] start envoyé\n> ");
                fflush(stdout);
            }
        } else if (ch == 'q' || ch == 'Q') {
            printf("\n[UI] quit\n");
            break;
        } else if (ch == '\n' || ch == '\r') {
            /* ignore */
        } else {
            printf("[UI] commande inconnue '%c'\n> ", ch);
            fflush(stdout);
        }
    }

    /*
     * V1 : on quitte brutalement (le thread rx sera terminé quand le process se termine).
     * Plus tard : on fera un shutdown propre.
     */
    appbus_close(bus);
    return 0;
}