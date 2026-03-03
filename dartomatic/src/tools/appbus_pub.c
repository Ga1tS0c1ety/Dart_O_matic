#include "appbus/client.h"
#include "ipc/appbus_ipc.h"
#include <stdio.h>

/*
 * Outil de test :
 *   appbus_pub <topic> <json_payload> [sock]
 */
int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <topic> <json_payload> [sock]\n", argv[0]);
        return 1;
    }

    const char* topic = argv[1];
    const char* payload = argv[2];
    const char* sock = (argc >= 4) ? argv[3] : APPBUS_DEFAULT_SOCK;

    AppBusClient* c = appbus_connect(sock);
    if (!c) return 2;

    int rc = appbus_publish(c, topic, payload);
    appbus_close(c);

    return (rc == 0) ? 0 : 3;
}