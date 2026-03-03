#pragma once

/*
 * Lance le broker AppBus (appbusd) :
 * - écoute sur un socket Unix STREAM
 * - accepte des clients
 * - gère SUB/UNSUB
 * - route les PUB vers les clients abonnés au topic
 */
int appbus_broker_run(const char* sock_path);