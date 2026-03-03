#include "appbus/client.h"
#include "ipc/appbus_ipc.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

/* Structure interne du client */
struct AppBusClient {
    int fd;
};

/* Lit exactement n octets en mode bloquant */
static int read_full(int fd, void* buf, size_t n) {
    uint8_t* p = (uint8_t*)buf;
    size_t got = 0;
    while (got < n) {
        ssize_t r = read(fd, p + got, n - got);
        if (r == 0) return 0; /* broker fermé */
        if (r < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        got += (size_t)r;
    }
    return 1;
}

/* Écrit exactement n octets */
static int write_full(int fd, const void* buf, size_t n) {
    const uint8_t* p = (const uint8_t*)buf;
    size_t sent = 0;
    while (sent < n) {
        ssize_t w = write(fd, p + sent, n - sent);
        if (w < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        sent += (size_t)w;
    }
    return 1;
}

/*
 * Envoie un message AppBus (SUB/UNSUB/PUB).
 * - topic : string C (on envoie strlen(topic) octets)
 * - payload : bytes (pour PUB)
 */
static int send_msg(int fd, uint16_t type, const char* topic,
                    const char* payload, size_t payload_len) {
    size_t topic_len = strlen(topic);

    AppBusMsgHeader hdr;
    hdr.magic = APPBUS_MAGIC;
    hdr.version = APPBUS_VERSION;
    hdr.type = type;
    hdr.topic_len = (uint32_t)topic_len;
    hdr.payload_len = (uint32_t)payload_len;

    if (write_full(fd, &hdr, sizeof(hdr)) <= 0) return -1;
    if (write_full(fd, topic, topic_len) <= 0) return -1;
    if (payload_len > 0 && write_full(fd, payload, payload_len) <= 0) return -1;
    return 0;
}

AppBusClient* appbus_connect(const char* sock_path) {
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) { perror("socket"); return NULL; }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, sock_path, sizeof(addr.sun_path) - 1);

    if (connect(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("connect");
        close(fd);
        return NULL;
    }

    AppBusClient* c = (AppBusClient*)calloc(1, sizeof(AppBusClient));
    c->fd = fd;
    return c;
}

void appbus_close(AppBusClient* c) {
    if (!c) return;
    if (c->fd >= 0) close(c->fd);
    free(c);
}

int appbus_subscribe(AppBusClient* c, const char* topic) {
    return send_msg(c->fd, APPBUS_MSG_SUB, topic, NULL, 0);
}

int appbus_unsubscribe(AppBusClient* c, const char* topic) {
    return send_msg(c->fd, APPBUS_MSG_UNSUB, topic, NULL, 0);
}

int appbus_publish(AppBusClient* c, const char* topic, const char* payload_json) {
    /* payload_json peut être NULL -> payload vide */
    const char* p = payload_json ? payload_json : "";
    size_t len = payload_json ? strlen(payload_json) : 0;
    return send_msg(c->fd, APPBUS_MSG_PUB, topic, p, len);
}

/*
 * Boucle bloquante : reçoit les PUB routés par le broker.
 * On ignore SUB/UNSUB (normalement le broker ne renvoie que des PUB),
 * mais on vérifie quand même type par sécurité.
 */
int appbus_poll(AppBusClient* c, appbus_on_message_fn cb, void* user) {
    while (1) {
        AppBusMsgHeader hdr;
        int rr = read_full(c->fd, &hdr, sizeof(hdr));
        if (rr == 0) return 0; /* broker fermé */
        if (rr < 0) return -1;

        /* Validation */
        if (hdr.magic != APPBUS_MAGIC || hdr.version != APPBUS_VERSION) return -1;
        if (hdr.topic_len == 0 || hdr.topic_len > 4096) return -1;
        if (hdr.payload_len > (1024 * 1024)) return -1;

        /* Lecture topic */
        char* topic = (char*)malloc(hdr.topic_len + 1);
        if (!topic) return -1;
        if (read_full(c->fd, topic, hdr.topic_len) <= 0) { free(topic); return -1; }
        topic[hdr.topic_len] = '\0';

        /* Lecture payload */
        char* payload = NULL;
        if (hdr.payload_len > 0) {
            payload = (char*)malloc(hdr.payload_len + 1);
            if (!payload) { free(topic); return -1; }
            if (read_full(c->fd, payload, hdr.payload_len) <= 0) {
                free(topic); free(payload); return -1;
            }
            payload[hdr.payload_len] = '\0'; /* pratique pour JSON */
        } else {
            payload = (char*)calloc(1, 1); /* payload vide */
        }

        /* On appelle le callback uniquement pour PUB */
        if (hdr.type == APPBUS_MSG_PUB && cb) {
            cb(topic, payload, hdr.payload_len, user);
        }

        free(topic);
        free(payload);
    }
}