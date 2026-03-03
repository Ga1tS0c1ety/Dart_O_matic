#include "appbus/broker.h"
#include "ipc/appbus_ipc.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

/* Limites V1 (simples, suffisantes pour démarrer) */
#define MAX_CLIENTS 64
#define MAX_SUBS    64
#define MAX_TOPIC   256
#define MAX_PAYLOAD (1024 * 1024) /* 1 MB max */

/*
 * Structure représentant un client connecté au broker.
 * - fd : file descriptor du socket client
 * - subs[] : liste des topics auxquels il est abonné
 */
typedef struct {
    int  fd;
    int  alive;
    char subs[MAX_SUBS][MAX_TOPIC];
    int  sub_count;
} Client;

/* Met un fd en non-bloquant : utile pour ne pas bloquer le broker */
static int set_nonblock(int fd) {
    int fl = fcntl(fd, F_GETFL, 0);
    if (fl < 0) return -1;
    return fcntl(fd, F_SETFL, fl | O_NONBLOCK);
}

/*
 * read_full : lit exactement n octets.
 * Retour :
 *  1  = OK
 *  0  = le client a fermé la connexion
 * -2  = non-bloquant : pas assez de données dispo pour le moment
 * -1  = erreur
 */
static int read_full(int fd, void* buf, size_t n) {
    uint8_t* p = (uint8_t*)buf;
    size_t got = 0;

    while (got < n) {
        ssize_t r = read(fd, p + got, n - got);
        if (r == 0) return 0; /* client fermé */
        if (r < 0) {
            if (errno == EINTR) continue; /* signal -> on réessaie */
            if (errno == EAGAIN || errno == EWOULDBLOCK) return -2; /* pas prêt */
            return -1;
        }
        got += (size_t)r;
    }
    return 1;
}

/* write_full : écrit exactement n octets (ou erreur) */
static int write_full(int fd, const void* buf, size_t n) {
    const uint8_t* p = (const uint8_t*)buf;
    size_t sent = 0;

    while (sent < n) {
        ssize_t w = write(fd, p + sent, n - sent);
        if (w < 0) {
            if (errno == EINTR) continue;
            if (errno == EPIPE) return 0; /* client mort */
            return -1;
        }
        sent += (size_t)w;
    }
    return 1;
}

/* Vérifie si un client est abonné à un topic donné */
static int client_has_sub(const Client* c, const char* topic) {
    for (int i = 0; i < c->sub_count; i++) {
        if (strcmp(c->subs[i], topic) == 0) return 1;
    }
    return 0;
}

/* Ajoute un topic à la liste des abonnements du client */
static void client_add_sub(Client* c, const char* topic) {
    if (c->sub_count >= MAX_SUBS) return;
    if (client_has_sub(c, topic)) return;

    strncpy(c->subs[c->sub_count], topic, MAX_TOPIC - 1);
    c->subs[c->sub_count][MAX_TOPIC - 1] = '\0';
    c->sub_count++;

    printf("[appbusd] fd=%d SUB '%s'\n", c->fd, topic);
}

/* Retire un topic des abonnements */
static void client_remove_sub(Client* c, const char* topic) {
    for (int i = 0; i < c->sub_count; i++) {
        if (strcmp(c->subs[i], topic) == 0) {
            /* On remplace l'élément supprimé par le dernier (swap) */
            if (i != c->sub_count - 1) {
                memcpy(c->subs[i], c->subs[c->sub_count - 1], MAX_TOPIC);
            }
            c->sub_count--;
            printf("[appbusd] fd=%d UNSUB '%s'\n", c->fd, topic);
            return;
        }
    }
}

/* Ferme et “oublie” un client */
static void drop_client(Client* c) {
    if (c->fd >= 0) close(c->fd);
    c->fd = -1;
    c->alive = 0;
    c->sub_count = 0;
}

/*
 * Traite UN message venant du client clients[idx].
 * - lit header
 * - lit topic
 * - lit payload
 * - exécute SUB/UNSUB ou route PUB
 */
static int handle_one_message(Client clients[], int idx) {
    Client* c = &clients[idx];

    AppBusMsgHeader hdr;
    int rr = read_full(c->fd, &hdr, sizeof(hdr));
    if (rr == 0) return 0;   /* client a fermé */
    if (rr == -2) return 1;  /* pas assez de données pour le moment */
    if (rr < 0) return -1;   /* erreur */

    /* Validation du header */
    if (hdr.magic != APPBUS_MAGIC || hdr.version != APPBUS_VERSION) {
        fprintf(stderr, "[appbusd] header invalide depuis fd=%d\n", c->fd);
        return -1;
    }
    if (hdr.topic_len == 0 || hdr.topic_len >= MAX_TOPIC) return -1;
    if (hdr.payload_len > MAX_PAYLOAD) return -1;

    /* Lecture topic (string sans \0) */
    char topic[MAX_TOPIC];
    memset(topic, 0, sizeof(topic));
    rr = read_full(c->fd, topic, hdr.topic_len);
    if (rr <= 0) return rr;

    /* Lecture payload (bytes) */
    char* payload = NULL;
    if (hdr.payload_len > 0) {
        payload = (char*)malloc(hdr.payload_len);
        if (!payload) return -1;
        rr = read_full(c->fd, payload, hdr.payload_len);
        if (rr <= 0) { free(payload); return rr; }
    }

    /* Actions selon type */
    if (hdr.type == APPBUS_MSG_SUB) {
        client_add_sub(c, topic);
        /* pas d'ACK en V1 : on garde simple */
    } else if (hdr.type == APPBUS_MSG_UNSUB) {
        client_remove_sub(c, topic);
    } else if (hdr.type == APPBUS_MSG_PUB) {

        /* Route le message vers tous les clients abonnés à ce topic */
        printf("[appbusd] fd=%d PUB '%s' (%u bytes)\n",
               c->fd, topic, hdr.payload_len);

        for (int i = 0; i < MAX_CLIENTS; i++) {
            Client* dst = &clients[i];
            if (!dst->alive) continue;
            if (!client_has_sub(dst, topic)) continue;

            /* On renvoie exactement le même framing (header + topic + payload) */
            int ok = write_full(dst->fd, &hdr, sizeof(hdr));
            if (ok <= 0) { drop_client(dst); continue; }

            ok = write_full(dst->fd, topic, hdr.topic_len);
            if (ok <= 0) { drop_client(dst); continue; }

            if (hdr.payload_len > 0) {
                ok = write_full(dst->fd, payload, hdr.payload_len);
                if (ok <= 0) { drop_client(dst); continue; }
            }
        }

    } else {
        free(payload);
        return -1;
    }

    free(payload);
    return 1;
}

int appbus_broker_run(const char* sock_path) {
    int srv = socket(AF_UNIX, SOCK_STREAM, 0);
    if (srv < 0) { perror("socket"); return -1; }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, sock_path, sizeof(addr.sun_path) - 1);

    /* On supprime le socket précédent si existant */
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

    /* Non-bloquant : on ne veut jamais bloquer sur accept() */
    if (set_nonblock(srv) != 0) {
        perror("fcntl");
        close(srv);
        return -1;
    }

    /* Tableau de clients */
    Client clients[MAX_CLIENTS];
    for (int i = 0; i < MAX_CLIENTS; i++) {
        clients[i].fd = -1;
        clients[i].alive = 0;
        clients[i].sub_count = 0;
    }

    printf("[appbusd] écoute sur %s\n", sock_path);

    while (1) {
        /* select() attend des événements lecture sur srv + clients */
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

        /* Nouveau client ? */
        if (FD_ISSET(srv, &rfds)) {
            int cfd = accept(srv, NULL, NULL);
            if (cfd >= 0) {
                set_nonblock(cfd);

                int placed = 0;
                for (int i = 0; i < MAX_CLIENTS; i++) {
                    if (!clients[i].alive) {
                        clients[i].fd = cfd;
                        clients[i].alive = 1;
                        clients[i].sub_count = 0;
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

        /* Messages des clients */
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (!clients[i].alive) continue;
            if (!FD_ISSET(clients[i].fd, &rfds)) continue;

            /*
             * On essaye de vider plusieurs messages en rafale (jusqu'à 8)
             * pour éviter de revenir trop souvent dans select().
             */
            for (int k = 0; k < 8; k++) {
                int hr = handle_one_message(clients, i);

                if (hr == 0) { /* déconnexion */
                    printf("[appbusd] client déconnecté fd=%d\n", clients[i].fd);
                    drop_client(&clients[i]);
                    break;
                }
                if (hr == -2) break; /* plus de données pour l'instant */
                if (hr < 0) {
                    fprintf(stderr, "[appbusd] erreur protocole, drop fd=%d\n", clients[i].fd);
                    drop_client(&clients[i]);
                    break;
                }
            }
        }
    }

    close(srv);
    unlink(sock_path);
    return 0;
}