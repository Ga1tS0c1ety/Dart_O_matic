#include "appbus/broker.h"
#include "ipc/appbus_ipc.h"
#include <stdio.h>

/*
 * appbusd : broker local (niveau 0)
 * Usage :
 *   ./appbusd             -> écoute sur /tmp/dart_appbus.sock
 *   ./appbusd /tmp/x.sock -> écoute sur un autre path
 */
int main(int argc, char** argv) {
    const char* path = (argc >= 2) ? argv[1] : APPBUS_DEFAULT_SOCK;
    return appbus_broker_run(path);
}