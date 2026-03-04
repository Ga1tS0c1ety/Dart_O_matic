#include "appbus/broker.h"
#include "ipc/appbus_ipc.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

/* Limites (V1/V2) */
#define MAX_CLIENTS 64
#define MAX_SUBS    64
#define MAX_TOPIC   256
#define MAX_PAYLOAD (1024 * 1024) /* 1 MB max */
#define RX_TMP_CHUNK 4096         /* taille buffer temporaire de read() */

typedef enum {
    ST_HDR = 0,
    ST_TOPIC = 1,
    ST_PAYLOAD = 2
} ParseState;

/*
 * Client avec état de parsing (vrai fix)
 * - sockets non-bloquants
 * - on accepte que les données arrivent en morceaux (STREAM)
 * - on reconstitue un message complet via ST_HDR -> ST_TOPIC -> ST_PAYLOAD
 */
typedef struct {
    int fd;
    int alive;

    /* subscriptions */
    char subs[MAX_SUBS][MAX_TOPIC];
    int  sub_count;

    /* --- RX parsing state --- */
    ParseState st;
    AppBusMsgHeader hdr;

    uint32_t need;  /* nombre d’octets attendus pour l’étape courante */
    uint32_t have;  /* nombre d’octets déjà reçus pour l’étape courante */

    char topic[MAX_TOPIC]; /* buffer topic (max 256) */
    char* payload;         /* buffer payload (malloc si payload_len > 0) */
} Client;

/* ========================= UTILS FD ========================= */

static int set_nonblock(int fd) {
    int fl = fcntl(fd, F_GETFL, 0);
    if (fl < 0) return -1;
    return fcntl(fd, F_SETFL, fl | O_NONBLOCK);
}

/*
 * Écriture non-bloquante.
 * - Retour  1 : tout écrit
 * - Retour  0 : client mort (EPIPE) -> à drop
 * - Retour -2 : EAGAIN (socket plein) -> on choisit de drop en V2 simple
 * - Retour -1 : autre erreur
 *
 * NOTE : un broker “pro” garderait un buffer TX par client.
 * Ici, pour rester simple ET non-bloquant, on DROP le client lent.
 */
static int write_full_nb(int fd, const void* buf, size_t n) {
    const uint8_t* p = (const uint8_t*)buf;
    size_t sent = 0;

    while (sent < n) {
        ssize_t w = write(fd, p + sent, n - sent);
        if (w < 0) {
            if (errno == EINTR) continue;
            if (errno == EPIPE) return 0;
            if (errno == EAGAIN || errno == EWOULDBLOCK) return -2;
            return -1;
        }
        sent += (size_t)w;
    }
    return 1;
}

/* ========================= SUBS ========================= */

static int client_has_sub(const Client* c, const char* topic) {
    for (int i = 0; i < c->sub_count; i++) {
        if (strcmp(c->subs[i], topic) == 0) return 1;
    }
    return 0;
}

static void client_add_sub(Client* c, const char* topic) {
    if (c->sub_count >= MAX_SUBS) return;
    if (client_has_sub(c, topic)) return;

    strncpy(c->subs[c->sub_count], topic, MAX_TOPIC - 1);
    c->subs[c->sub_count][MAX_TOPIC - 1] = '\0';
    c->sub_count++;

    printf("[appbusd] fd=%d SUB '%s'\n", c->fd, topic);
}

static void client_remove_sub(Client* c, const char* topic) {
    for (int i = 0; i < c->sub_count; i++) {
        if (strcmp(c->subs[i], topic) == 0) {
            if (i != c->sub_count - 1) {
                memcpy(c->subs[i], c->subs[c->sub_count - 1], MAX_TOPIC);
            }
            c->sub_count--;
            printf("[appbusd] fd=%d UNSUB '%s'\n", c->fd, topic);
            return;
        }
    }
}

/* ========================= DROP / RESET ========================= */

static void reset_rx_state(Client* c) {
    c->st = ST_HDR;
    c->need = (uint32_t)sizeof(AppBusMsgHeader);
    c->have = 0;

    /* topic buffer : pas besoin de clear, on écrase */
    if (c->payload) {
        free(c->payload);
        c->payload = NULL;
    }
}

static void drop_client(Client* c) {
    if (c->payload) {
        free(c->payload);
        c->payload = NULL;
    }
    if (c->fd >= 0) close(c->fd);
    c->fd = -1;
    c->alive = 0;
    c->sub_count = 0;
}

/* ========================= VALIDATION ========================= */

static int valid_header(const AppBusMsgHeader* hdr) {
    if (hdr->magic != APPBUS_MAGIC) return 0;
    if (hdr->version != APPBUS_VERSION) return 0;
    if (hdr->type != APPBUS_MSG_SUB &&
        hdr->type != APPBUS_MSG_UNSUB &&
        hdr->type != APPBUS_MSG_PUB) return 0;

    if (hdr->topic_len == 0 || hdr->topic_len >= MAX_TOPIC) return 0;
    if (hdr->payload_len > MAX_PAYLOAD) return 0;
    return 1;
}

/* ========================= ROUTAGE ========================= */

/*
 * On a un message complet (hdr + topic + payload).
 * - Si SUB/UNSUB : maj abonnements du client
 * - Si PUB : route vers abonnés
 */
static void handle_complete_message(Client clients[], int src_idx) {
    Client* src = &clients[src_idx];

    const AppBusMsgHeader* hdr = &src->hdr;
    const char* topic = src->topic;
    const char* payload = src->payload; /* peut être NULL si payload_len==0 */

    if (hdr->type == APPBUS_MSG_SUB) {
        client_add_sub(src, topic);
        return;
    }

    if (hdr->type == APPBUS_MSG_UNSUB) {
        client_remove_sub(src, topic);
        return;
    }

    /* PUB */
    printf("[appbusd] fd=%d PUB '%s' (%u bytes)\n", src->fd, topic, hdr->payload_len);

    for (int i = 0; i < MAX_CLIENTS; i++) {
        Client* dst = &clients[i];
        if (!dst->alive) continue;
        if (!client_has_sub(dst, topic)) continue;

        /* On renvoie framing identique */
        int ok = write_full_nb(dst->fd, hdr, sizeof(*hdr));
        if (ok <= 0) { drop_client(dst); continue; }

        ok = write_full_nb(dst->fd, topic, hdr->topic_len);
        if (ok <= 0) { drop_client(dst); continue; }

        if (hdr->payload_len > 0) {
            ok = write_full_nb(dst->fd, payload, hdr->payload_len);
            if (ok <= 0) { drop_client(dst); continue; }
        }
    }
}

/* ========================= PARSING NON-BLOQUANT ========================= */

/*
 * Consomme des octets reçus (tmp[off..]) en fonction de l’état ST_HDR/ST_TOPIC/ST_PAYLOAD.
 * Peut produire 0..N messages complets.
 */
static int consume_bytes(Client clients[], int idx, const uint8_t* data, size_t len) {
    Client* c = &clients[idx];
    size_t off = 0;

    while (off < len) {
        size_t avail = len - off;

        if (c->st == ST_HDR) {
            /* Copier dans hdr */
            size_t take = avail;
            size_t remaining = (size_t)c->need - (size_t)c->have;
            if (take > remaining) take = remaining;

            memcpy(((uint8_t*)&c->hdr) + c->have, data + off, take);
            c->have += (uint32_t)take;
            off += take;

            if (c->have == c->need) {
                /* Header complet -> validation */
                if (!valid_header(&c->hdr)) {
                    fprintf(stderr, "[appbusd] header invalide depuis fd=%d\n", c->fd);
                    return -1;
                }

                /* Prépare lecture topic */
                c->st = ST_TOPIC;
                c->need = c->hdr.topic_len;
                c->have = 0;
            }
        }
        else if (c->st == ST_TOPIC) {
            size_t take = avail;
            size_t remaining = (size_t)c->need - (size_t)c->have;
            if (take > remaining) take = remaining;

            memcpy(c->topic + c->have, data + off, take);
            c->have += (uint32_t)take;
            off += take;

            if (c->have == c->need) {
                /* Topic complet -> termine string */
                c->topic[c->hdr.topic_len] = '\0';

                /* Prépare lecture payload */
                c->st = ST_PAYLOAD;
                c->need = c->hdr.payload_len;
                c->have = 0;

                if (c->hdr.payload_len > 0) {
                    c->payload = (char*)malloc(c->hdr.payload_len);
                    if (!c->payload) return -1;
                } else {
                    c->payload = NULL;
                }

                /* Cas payload vide : message complet immédiatement */
                if (c->hdr.payload_len == 0) {
                    handle_complete_message(clients, idx);
                    reset_rx_state(c);
                }
            }
        }
        else { /* ST_PAYLOAD */
            size_t take = avail;
            size_t remaining = (size_t)c->need - (size_t)c->have;
            if (take > remaining) take = remaining;

            memcpy(c->payload + c->have, data + off, take);
            c->have += (uint32_t)take;
            off += take;

            if (c->have == c->need) {
                /* Payload complet -> message complet */
                handle_complete_message(clients, idx);
                reset_rx_state(c);
            }
        }
    }

    return 1;
}

/*
 * Appelé quand select() indique que le fd est lisible.
 * On lit ce qu’on peut, puis on “consume”.
 */
static int on_client_readable(Client clients[], int idx) {
    Client* c = &clients[idx];

    uint8_t tmp[RX_TMP_CHUNK];

    while (1) {
        ssize_t r = read(c->fd, tmp, sizeof(tmp));
        if (r == 0) {
            /* déconnexion */
            return 0;
        }
        if (r < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) return 1; /* fini pour l’instant */
            return -1;
        }

        /* On a reçu r octets -> parsing */
        int cr = consume_bytes(clients, idx, tmp, (size_t)r);
        if (cr < 0) return -1;

        /* Continue la boucle : peut rester des données en socket */
    }
}

/* ========================= MAIN BROKER ========================= */

int appbus_broker_run(const char* sock_path) {
    int srv = socket(AF_UNIX, SOCK_STREAM, 0);
    if (srv < 0) { perror("socket"); return -1; }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, sock_path, sizeof(addr.sun_path) - 1);

    unlink(sock_path);

    if (bind(srv, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(srv);
        return -1;
    }
    if (listen(srv, 16) < 0) {
        perror("listen");
        close(srv);
        return -1;
    }

    /* srv en non-bloquant */
    if (set_nonblock(srv) != 0) {
        perror("fcntl");
        close(srv);
        return -1;
    }

    Client clients[MAX_CLIENTS];
    for (int i = 0; i < MAX_CLIENTS; i++) {
        clients[i].fd = -1;
        clients[i].alive = 0;
        clients[i].sub_count = 0;
        clients[i].payload = NULL;
        reset_rx_state(&clients[i]);
    }

    printf("[appbusd] écoute sur %s\n", sock_path);

    while (1) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(srv, &rfds);
        int maxfd = srv;

        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (!clients[i].alive) continue;
            FD_SET(clients[i].fd, &rfds);
            if (clients[i].fd > maxfd) maxfd = clients[i].fd;
        }

        int r = select(maxfd + 1, &rfds, NULL, NULL, NULL);
        if (r < 0) {
            if (errno == EINTR) continue;
            perror("select");
            break;
        }

        /* Accept nouveaux clients */
        if (FD_ISSET(srv, &rfds)) {
            while (1) {
                int cfd = accept(srv, NULL, NULL);
                if (cfd < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) break; /* plus de clients */
                    perror("accept");
                    break;
                }

                set_nonblock(cfd);

                int placed = 0;
                for (int i = 0; i < MAX_CLIENTS; i++) {
                    if (!clients[i].alive) {
                        clients[i].fd = cfd;
                        clients[i].alive = 1;
                        clients[i].sub_count = 0;
                        reset_rx_state(&clients[i]);
                        placed = 1;
                        printf("[appbusd] client connecté fd=%d\n", cfd);
                        break;
                    }
                }

                if (!placed) {
                    fprintf(stderr, "[appbusd] trop de clients, drop fd=%d\n", cfd);
                    close(cfd);
                }
            }
        }

        /* Lire/parsing pour chaque client lisible */
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (!clients[i].alive) continue;
            if (!FD_ISSET(clients[i].fd, &rfds)) continue;

            int cr = on_client_readable(clients, i);
            if (cr == 0) {
                printf("[appbusd] client déconnecté fd=%d\n", clients[i].fd);
                drop_client(&clients[i]);
            } else if (cr < 0) {
                fprintf(stderr, "[appbusd] erreur protocole/IO, drop fd=%d\n", clients[i].fd);
                drop_client(&clients[i]);
            }
        }
    }

    close(srv);
    unlink(sock_path);
    return 0;
}