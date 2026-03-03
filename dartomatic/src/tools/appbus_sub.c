#include "appbus/client.h"
#include "ipc/appbus_ipc.h"
#include <stdio.h>

/* Callback à chaque message reçu */
static void on_msg(const char* topic, const char* payload, size_t payload_len, void* user) {
    (void)user;
    printf("[SUB] %s -> %.*s\n", topic, (int)payload_len, payload ? payload : "");
    fflush(stdout);
}

/*
 * Outil de test :
 *   appbus_sub <topic> [sock]
 */
int main(int argc, char** argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <topic> [sock]\n", argv[0]);
        return 1;
    }

    const char* topic = argv[1];
    const char* sock = (argc >= 3) ? argv[2] : APPBUS_DEFAULT_SOCK;

    AppBusClient* c = appbus_connect(sock);
    if (!c) return 2;

    if (appbus_subscribe(c, topic) != 0) {
        fprintf(stderr, "subscribe failed\n");
        appbus_close(c);
        return 3;
    }

    /* boucle infinie : affiche tout ce qui arrive sur le topic */
    return (appbus_poll(c, on_msg, NULL) == 0) ? 0 : 4;
}