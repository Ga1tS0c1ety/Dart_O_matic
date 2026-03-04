#include "rt/rt_orchestrator.h"
#include "rt/mpu_adapter.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <pthread.h>
#include <sys/time.h>
#include <stdint.h>

/* ==== Ton protocole actuel (temporaire) ==== */
#define SOCKET_BASE   "/tmp/dart_cam_"
#define MASTER_SOCKET "/tmp/dart_master.sock"
#define CMD_TRIGGER   1

typedef struct { int cmd; } CamCommand;
typedef struct { int camera_id; double u, v; } ImpactMessage;

/* now_us */
static uint64_t now_us(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
}

extern void* mpu_thread(void* arg);

int main(void) {
    /* === Init orchestrateur === */
    RtOrchestrator orch;
    RtOrchestratorParams op;
    memset(&op, 0, sizeof(op));
    op.n_cams = 4;
    op.cam_ids[0] = 0;
    op.cam_ids[1] = 2;
    op.cam_ids[2] = 4;
    op.cam_ids[3] = 6;
    op.min_cams = 2;
    op.window_ms = 150;
    op.cooldown_ms = 250;

    if (rt_orch_init(&orch, op) != 0) {
        fprintf(stderr, "[RT] rt_orch_init failed\n");
        return 1;
    }

    /* === Init MPU adapter (pipe) === */
    MpuAdapter mpu;
    if (mpu_adapter_init(&mpu) != 0) {
        fprintf(stderr, "[RT] mpu_adapter_init failed\n");
        return 1;
    }

    /* === Lance thread MPU === */
    pthread_t mpu_tid;
    pthread_create(&mpu_tid, NULL, mpu_thread, &mpu);

    /* === Socket master (réception cams) === */
    int master_sock = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (master_sock < 0) { perror("socket"); return 1; }

    struct sockaddr_un master_addr = {0};
    master_addr.sun_family = AF_UNIX;
    strncpy(master_addr.sun_path, MASTER_SOCKET, sizeof(master_addr.sun_path)-1);

    unlink(MASTER_SOCKET);
    if (bind(master_sock, (struct sockaddr*)&master_addr, sizeof(master_addr)) < 0) {
        perror("bind");
        return 1;
    }

    /* === Adresses des cams (pour trigger) === */
    struct sockaddr_un cam_addrs[4];
    for (int i = 0; i < op.n_cams; i++) {
        memset(&cam_addrs[i], 0, sizeof(cam_addrs[i]));
        cam_addrs[i].sun_family = AF_UNIX;
        snprintf(cam_addrs[i].sun_path, sizeof(cam_addrs[i].sun_path),
                 "%s%d.sock", SOCKET_BASE, op.cam_ids[i]);
    }

    printf("[RT] prêt. MPU->orchestrateur OK. En attente d'impacts.\n");

    while (1) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(master_sock, &rfds);
        FD_SET(mpu_adapter_fd(&mpu), &rfds);

        int maxfd = master_sock;
        int mfd = mpu_adapter_fd(&mpu);
        if (mfd > maxfd) maxfd = mfd;

        /* timeout 1ms : permet tick régulier */
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 1000;

        int r = select(maxfd + 1, &rfds, NULL, NULL, &tv);

        uint64_t t = now_us();
        rt_orch_tick(&orch, t);

        /* ==== MPU event ==== */
        if (r > 0 && FD_ISSET(mfd, &rfds)) {
            /* vider pipe */
            unsigned char buf[64];
            while (read(mfd, buf, sizeof(buf)) > 0) {}

            int accepted = rt_orch_on_mpu_impact(&orch, t);
            if (accepted) {
                printf("[RT] MPU impact accepted -> impact_id=%llu\n",
                       (unsigned long long)rt_orch_current_impact_id(&orch));

                /* TEMPORAIRE: trigger cams avec ton ancienne commande */
                CamCommand cmd = { .cmd = CMD_TRIGGER };
                for (int i = 0; i < op.n_cams; i++) {
                    sendto(master_sock, &cmd, sizeof(cmd), 0,
                           (struct sockaddr*)&cam_addrs[i], sizeof(struct sockaddr_un));
                }
            } else {
                /* ignoré (ARMED/COOLDOWN) */
            }
        }

        /* ==== Observations cams (format ancien) ==== */
        if (r > 0 && FD_ISSET(master_sock, &rfds)) {
            while (1) {
                ImpactMessage msg;
                ssize_t rr = recv(master_sock, &msg, sizeof(msg), MSG_DONTWAIT);
                if (rr != sizeof(msg)) break;

                RtObservationMsg obs;
                memset(&obs, 0, sizeof(obs));
                obs.impact_id = rt_orch_current_impact_id(&orch); /* V1: impact courant */
                obs.cam_id = msg.camera_id;
                obs.ts_us = t;
                obs.u = msg.u;
                obs.v = msg.v;
                obs.conf = 1.0f;

                rt_orch_on_observation(&orch, &obs);
            }
        }

        /* ==== Bundle prêt ? ==== */
        ImpactBundle b;
        if (rt_orch_poll_bundle(&orch, &b)) {
            printf("[RT] BUNDLE READY impact_id=%llu obs=%d\n",
                   (unsigned long long)b.impact_id, b.obs_count);
            for (int i = 0; i < b.obs_count; i++) {
                printf("   - cam_index=%d u=%.1f v=%.1f conf=%.2f\n",
                       b.obs[i].cam_index, b.obs[i].u, b.obs[i].v, b.obs[i].conf);
            }
        }
    }

    return 0;
}