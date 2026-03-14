#include "rt/rt_orchestrator.h"
#include "rt/mpu_adapter.h"
#include "rt/mpu6050_thread.h"
#include "ipc/rt_ipc.h"
#include "rt/triangulation.h"
#include "vision/camera_model.h"
#include "rt/rt_bridge_out.h"

#include "appbus/client.h"
#include "appbus/topics.h"
#include "ipc/appbus_ipc.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <pthread.h>
#include <stdint.h>
#include <errno.h>
#include <time.h>

/* ===== RT IPC paths ===== */
#define SOCKET_BASE       "/tmp/dart_cam_"
#define RT_MASTER_SOCKET  "/tmp/dart_master.sock"

/* ========================================================= */
/* Temps                                                     */
/* ========================================================= */

static uint64_t now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000ULL);
}

/* ========================================================= */
/* Réception AppBus côté RT                                  */
/* ========================================================= */

/*
 * Flags pilotés par le thread AppBus RX.
 *
 * - board_clear_confirmed :
 *     info métier/UI : le joueur a retiré les fléchettes
 *
 * - refresh_reference_requested :
 *     demande technique : les caméras doivent rafraîchir leur référence
 *
 * Règle choisie :
 *   toute commande manuelle demande un refresh caméra.
 */
typedef struct {
    volatile int board_clear_confirmed;
    volatile int refresh_reference_requested;
} RtBusFlags;

typedef struct {
    AppBusClient* bus;
    RtBusFlags* flags;
} RtBusThreadCtx;

/*
 * Callback AppBus :
 * le RT consomme plusieurs commandes, mais son action reste simple :
 *   -> lever un flag
 * la boucle principale fera le vrai travail.
 */
static void on_rt_bus_msg(const char* topic, const char* payload, size_t payload_len, void* user) {
    (void)payload;
    (void)payload_len;

    RtBusFlags* flags = (RtBusFlags*)user;
    if (!flags) return;

    if (strcmp(topic, TOPIC_CMD_BOARD_CLEAR_CONF) == 0) {
        flags->board_clear_confirmed = 1;
        flags->refresh_reference_requested = 1;
        return;
    }

    if (strcmp(topic, TOPIC_CMD_GAME_UNDO) == 0 ||
        strcmp(topic, TOPIC_CMD_GAME_OVERRIDE_LAST) == 0 ||
        strcmp(topic, TOPIC_CMD_GAME_ADD_MANUAL_HIT) == 0) {
        flags->refresh_reference_requested = 1;
        return;
    }
}

/* Thread bloquant de réception AppBus */
static void* rt_bus_rx_thread(void* arg) {
    RtBusThreadCtx* ctx = (RtBusThreadCtx*)arg;
    if (!ctx || !ctx->bus || !ctx->flags) return NULL;

    appbus_poll(ctx->bus, on_rt_bus_msg, ctx->flags);
    return NULL;
}

/* ========================================================= */
/* main                                                      */
/* ========================================================= */

int main(void) {
    /* ===================================================== */
    /* Init orchestrateur RT                                 */
    /* ===================================================== */

    RtOrchestrator orch;
    RtOrchestratorParams op;
    memset(&op, 0, sizeof(op));

    op.n_cams = 4;
    op.cam_ids[0] = 0;
    op.cam_ids[1] = 2;
    op.cam_ids[2] = 4;
    op.cam_ids[3] = 6;

    op.min_cams = 2;
    op.window_ms = 800;
    op.cooldown_ms = 250;

    if (rt_orch_init(&orch, op) != 0) {
        fprintf(stderr, "[RT] rt_orch_init failed\n");
        return 1;
    }

    /* ===================================================== */
    /* Chargement calibration caméras pour triangulation     */
    /* ===================================================== */

    CameraModel cam_models[8];
    const char* intr_pat = "data/cam_param/camera_params_%d.yaml";
    const char* extr_pat = "data/cam_param/camera_extrinsics_%d.yaml";

    if (triangulation_load_cameras(cam_models, op.cam_ids, op.n_cams, intr_pat, extr_pat) != 0) {
        fprintf(stderr, "[RT] ERREUR: impossible de charger les paramètres caméras (intr/extr)\n");
        return 1;
    }

    /* ===================================================== */
    /* Init MPU                                              */
    /* ===================================================== */

    MpuAdapter mpu;
    if (mpu_adapter_init(&mpu) != 0) {
        fprintf(stderr, "[RT] mpu_adapter_init failed\n");
        return 1;
    }

    pthread_t mpu_tid;
    pthread_create(&mpu_tid, NULL, mpu_thread, &mpu);

    /* ===================================================== */
    /* Socket RT master                                      */
    /* ===================================================== */

    /*
     * Sert à :
     *   - recevoir les observations caméras
     *   - envoyer des commandes aux caméras via sendto()
     */
    int master_sock = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (master_sock < 0) {
        perror("socket");
        return 1;
    }

    struct sockaddr_un master_addr;
    memset(&master_addr, 0, sizeof(master_addr));
    master_addr.sun_family = AF_UNIX;
    strncpy(master_addr.sun_path, RT_MASTER_SOCKET, sizeof(master_addr.sun_path) - 1);

    unlink(RT_MASTER_SOCKET);
    if (bind(master_sock, (struct sockaddr*)&master_addr, sizeof(master_addr)) < 0) {
        perror("bind");
        return 1;
    }

    /* Adresses cibles des cam_process */
    struct sockaddr_un cam_addrs[8];
    for (int i = 0; i < op.n_cams; i++) {
        memset(&cam_addrs[i], 0, sizeof(cam_addrs[i]));
        cam_addrs[i].sun_family = AF_UNIX;
        snprintf(cam_addrs[i].sun_path, sizeof(cam_addrs[i].sun_path),
                 "%s%d.sock", SOCKET_BASE, op.cam_ids[i]);
    }

    printf("[RT] prêt. IPC RT: %s\n", RT_MASTER_SOCKET);
    printf("[RT] cams: {%d,%d,%d,%d} window=%dms min=%d cooldown=%dms\n",
           op.cam_ids[0], op.cam_ids[1], op.cam_ids[2], op.cam_ids[3],
           op.window_ms, op.min_cams, op.cooldown_ms);

    /* ===================================================== */
    /* AppBus                                                */
    /* ===================================================== */

    /*
     * bus_pub :
     *   utilisé pour publier evt/impact/triangulated
     */
    AppBusClient* bus_pub = rt_bridge_out_init(APPBUS_DEFAULT_SOCK);
    if (!bus_pub) {
        fprintf(stderr, "[RT] WARN: AppBus indisponible pour publication.\n");
    }

    /*
     * bus_cmd :
     *   utilisé pour recevoir les commandes UI / arbitrage.
     *
     * On prend une 2e connexion séparée :
     *   - bus_pub : publication
     *   - bus_cmd : réception bloquante via thread
     */
    AppBusClient* bus_cmd = appbus_connect(APPBUS_DEFAULT_SOCK);
    if (!bus_cmd) {
        fprintf(stderr, "[RT] WARN: AppBus indisponible pour commandes.\n");
    } else {
        if (appbus_subscribe(bus_cmd, TOPIC_CMD_BOARD_CLEAR_CONF) != 0 ||
            appbus_subscribe(bus_cmd, TOPIC_CMD_GAME_UNDO) != 0 ||
            appbus_subscribe(bus_cmd, TOPIC_CMD_GAME_OVERRIDE_LAST) != 0 ||
            appbus_subscribe(bus_cmd, TOPIC_CMD_GAME_ADD_MANUAL_HIT) != 0) {
            fprintf(stderr, "[RT] WARN: erreur subscribe commandes AppBus\n");
            appbus_close(bus_cmd);
            bus_cmd = NULL;
        }
    }

    RtBusFlags bus_flags;
    memset(&bus_flags, 0, sizeof(bus_flags));

    pthread_t bus_tid;
    RtBusThreadCtx bus_ctx;
    memset(&bus_ctx, 0, sizeof(bus_ctx));
    bus_ctx.bus = bus_cmd;
    bus_ctx.flags = &bus_flags;

    if (bus_cmd) {
        pthread_create(&bus_tid, NULL, rt_bus_rx_thread, &bus_ctx);
    }

    /* ===================================================== */
    /* Boucle principale RT                                  */
    /* ===================================================== */

    while (1) {
        fd_set rfds;
        FD_ZERO(&rfds);

        FD_SET(master_sock, &rfds);

        int mfd = mpu_adapter_fd(&mpu);
        FD_SET(mfd, &rfds);

        int maxfd = master_sock;
        if (mfd > maxfd) maxfd = mfd;

        /* Tick régulier 1 ms */
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 1000;

        int r = select(maxfd + 1, &rfds, NULL, NULL, &tv);

        uint64_t t = now_us();

        /* Tick orchestrateur / aggregator */
        rt_orch_tick(&orch, t);

        /* ------------------------------------------------- */
        /* Commandes AppBus -> flags RT                      */
        /* ------------------------------------------------- */

        if (bus_flags.board_clear_confirmed) {
            bus_flags.board_clear_confirmed = 0;
            printf("[RT] board clear confirmé\n");
        }

        if (bus_flags.refresh_reference_requested) {
            bus_flags.refresh_reference_requested = 0;

            printf("[RT] refresh références caméras\n");

            RtCamCommand cmd;
            memset(&cmd, 0, sizeof(cmd));
            cmd.cmd = RT_CAM_CMD_SET_REFERENCE;
            cmd.impact_id = 0;
            cmd.ts_us = t;

            for (int i = 0; i < op.n_cams; i++) {
                sendto(master_sock, &cmd, sizeof(cmd), 0,
                       (struct sockaddr*)&cam_addrs[i],
                       sizeof(struct sockaddr_un));
            }
        }

        /* ------------------------------------------------- */
        /* MPU -> trigger nouvel impact                      */
        /* ------------------------------------------------- */

        if (r > 0 && FD_ISSET(mfd, &rfds)) {
            unsigned char buf[64];

            /* vider le pipe MPU */
            while (read(mfd, buf, sizeof(buf)) > 0) {}

            int accepted = rt_orch_on_mpu_impact(&orch, t);
            if (accepted) {
                uint64_t impact_id = rt_orch_current_impact_id(&orch);

                printf("[RT] MPU impact ACCEPTED -> impact_id=%llu\n",
                       (unsigned long long)impact_id);

                RtCamCommand trig;
                memset(&trig, 0, sizeof(trig));
                trig.cmd = RT_CAM_CMD_TRIGGER;
                trig.impact_id = impact_id;
                trig.ts_us = t;

                for (int i = 0; i < op.n_cams; i++) {
                    sendto(master_sock, &trig, sizeof(trig), 0,
                           (struct sockaddr*)&cam_addrs[i],
                           sizeof(struct sockaddr_un));
                }
            }
        }

        /* ------------------------------------------------- */
        /* Réception observations caméras                    */
        /* ------------------------------------------------- */

        if (r > 0 && FD_ISSET(master_sock, &rfds)) {
            while (1) {
                RtObservationMsg obs;
                ssize_t rr = recv(master_sock, &obs, sizeof(obs), MSG_DONTWAIT);
                if (rr < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) break;
                    perror("[RT] recv");
                    break;
                }

                if (rr != (ssize_t)sizeof(obs)) {
                    /* paquet incomplet ou autre -> ignore */
                    continue;
                }

                rt_orch_on_observation(&orch, &obs);

                printf("[RT] obs cam_id=%d impact_id=%llu ts_us=%llu u=%.1f v=%.1f conf=%.2f\n",
                       obs.cam_id,
                       (unsigned long long)obs.impact_id,
                       (unsigned long long)obs.ts_us,
                       obs.u, obs.v, obs.conf);
            }
        }

        /* ------------------------------------------------- */
        /* Bundle prêt -> triangulation -> publish           */
        /* ------------------------------------------------- */

        ImpactBundle b;
        if (rt_orch_poll_bundle(&orch, &b)) {
            printf("[RT] BUNDLE READY impact_id=%llu obs=%d\n",
                   (unsigned long long)b.impact_id, b.obs_count);

            TriangulationResult tr;
            int rc = triangulation_from_bundle(&b, cam_models, op.cam_ids, op.n_cams, &tr);

            if (rc == 0) {
                double X_mm = tr.X * 1000.0;
                double Y_mm = tr.Y * 1000.0;
                double Z_mm = tr.Z * 1000.0;

                printf("[RT][TRIANG] impact_id=%llu X=%.1fmm Y=%.1fmm Z=%.1fmm err=%.2fpx pair=(%d,%d)\n",
                       (unsigned long long)b.impact_id,
                       X_mm, Y_mm, Z_mm,
                       tr.reproj_err_px,
                       op.cam_ids[tr.cam_i], op.cam_ids[tr.cam_j]);

                if (bus_pub) {
                    (void)rt_bridge_publish_triangulated(
                        bus_pub,
                        b.impact_id,
                        b.ts_trigger_us,
                        X_mm, Y_mm, Z_mm,
                        tr.reproj_err_px,
                        b.obs_count,
                        op.cam_ids[tr.cam_i],
                        op.cam_ids[tr.cam_j]
                    );
                }
            } else {
                printf("[RT][TRIANG] impact_id=%llu ECHEC rc=%d (pas de paire valide)\n",
                       (unsigned long long)b.impact_id, rc);
            }
        }
    }

    return 0;
}