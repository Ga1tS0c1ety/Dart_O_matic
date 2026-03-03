// dartomatic/include/ipc/appbus_ipc.h

#pragma once
#include <stdint.h>

/*
 * Protocole AppBus V1 (niveau 0) :
 * - Transport : Unix Domain Socket STREAM (fiable, orienté flux)
 * - Framing : un header fixe + topic (string) + payload (bytes)
 * - Messages supportés : SUB / UNSUB / PUB
 *
 * Note :
 *  - topic et payload NE contiennent PAS le '\0' (pas de chaîne C).
 *  - magic/version permettent de vérifier qu'on parle le même protocole.
 */

#define APPBUS_MAGIC 0x41505042u /* 'APPB' en hex : sert de signature */
#define APPBUS_VERSION 1         /* version du protocole */

typedef enum {
    APPBUS_MSG_SUB   = 1, /* abonnement à un topic */
    APPBUS_MSG_UNSUB = 2, /* désabonnement */
    APPBUS_MSG_PUB   = 3  /* publication d'un message sur un topic */
} AppBusMsgType;

/*
 * Header envoyé au début de chaque message.
 * On le "pack" pour éviter que le compilateur ajoute du padding.
 */
#pragma pack(push, 1)
typedef struct {
    uint32_t magic;       /* APPBUS_MAGIC */
    uint16_t version;     /* APPBUS_VERSION */
    uint16_t type;        /* AppBusMsgType */
    uint32_t topic_len;   /* taille en octets du topic (sans \0) */
    uint32_t payload_len; /* taille en octets du payload (sans \0) */
} AppBusMsgHeader;
#pragma pack(pop)

/* Chemin par défaut du socket du broker AppBus */
#define APPBUS_DEFAULT_SOCK "/tmp/dart_appbus.sock"