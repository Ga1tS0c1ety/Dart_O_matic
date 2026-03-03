#pragma once
#include <stddef.h>

/* Handle opaque : l'utilisateur n'a pas à connaître l'intérieur */
typedef struct AppBusClient AppBusClient;

/*
 * Callback appelée lorsqu'on reçoit un PUB (topic + payload).
 * payload_len : taille en octets (payload peut contenir n'importe quoi, pas forcément du texte)
 */
typedef void (*appbus_on_message_fn)(
    const char* topic,
    const char* payload,
    size_t payload_len,
    void* user
);

/* Connexion au broker */
AppBusClient* appbus_connect(const char* sock_path);

/* Fermer la connexion */
void appbus_close(AppBusClient* c);

/* Abonnement/désabonnement */
int appbus_subscribe(AppBusClient* c, const char* topic);
int appbus_unsubscribe(AppBusClient* c, const char* topic);

/* Publication (payload JSON sous forme de string) */
int appbus_publish(AppBusClient* c, const char* topic, const char* payload_json);

/*
 * Boucle de réception bloquante : lit les messages en continu.
 * À chaque PUB reçu, déclenche cb(topic, payload,...).
 */
int appbus_poll(AppBusClient* c, appbus_on_message_fn cb, void* user);