#include "rt/rt_orchestrator.h"
#include "rt/mpu_adapter.h"
#include "rt/mpu6050_thread.h"
#include "ipc/rt_ipc.h"
#include "rt/triangulation.h"
#include "vision/camera_model.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <pthread.h>
#include <sys/time.h>
#include <stdint.h>
#include <errno.h>

/* ===== RT IPC paths ===== */
#define SOCKET_BASE       "/tmp/dart_cam_"
#define RT_MASTER_SOCKET  "/tmp/dart_master.sock"

/* now_us */
#include <time.h>
static uint64_t now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000ULL);
}

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

        /* === Chargement modèles caméra pour triangulation === */
    CameraModel cam_models[8]; /* assez grand */
    const char* intr_pat = "data/cam_param/camera_params_%d.yaml";
    const char* extr_pat = "data/cam_param/camera_extrinsics_%d.yaml";

    if (triangulation_load_cameras(cam_models, op.cam_ids, op.n_cams, intr_pat, extr_pat) != 0) {
        fprintf(stderr, "[RT] ERREUR: impossible de charger les paramètres caméras (intr/extr)\n");
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

    /* === Socket RT master (réception obs) === */
    int master_sock = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (master_sock < 0) { perror("socket"); return 1; }

    struct sockaddr_un master_addr = {0};
    master_addr.sun_family = AF_UNIX;
    strncpy(master_addr.sun_path, RT_MASTER_SOCKET, sizeof(master_addr.sun_path)-1);

    unlink(RT_MASTER_SOCKET);
    if (bind(master_sock, (struct sockaddr*)&master_addr, sizeof(master_addr)) < 0) {
        perror("bind");
        return 1;
    }

    /* === Adresses des cams (pour trigger) === */
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

    while (1) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(master_sock, &rfds);
        int mfd = mpu_adapter_fd(&mpu);
        FD_SET(mfd, &rfds);

        int maxfd = master_sock;
        if (mfd > maxfd) maxfd = mfd;

        /* timeout 1ms : tick régulier */
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 1000;

        int r = select(maxfd + 1, &rfds, NULL, NULL, &tv);

        uint64_t t = now_us();
        rt_orch_tick(&orch, t);

        /* DEBUG : vérifier que la fenêtre expire bien */
        static uint64_t last = 0;
        if (t - last > 50 * 1000) { // toutes les 50ms
            last = t;
            printf("[RT][DBG] state=%d impact=%llu now=%llu\n",
                rt_orch_state(&orch),
                (unsigned long long)rt_orch_current_impact_id(&orch),
                (unsigned long long)t);
        }

        /* ==== MPU event ==== */
        if (r > 0 && FD_ISSET(mfd, &rfds)) {
            /* vider le pipe (consommer tous les impacts en attente) */
            unsigned char buf[64];
            while (read(mfd, buf, sizeof(buf)) > 0) {}

            int accepted = rt_orch_on_mpu_impact(&orch, t);
            if (accepted) {
                uint64_t impact_id = rt_orch_current_impact_id(&orch);
                printf("[RT] MPU impact ACCEPTED -> impact_id=%llu\n",
                       (unsigned long long)impact_id);

                /*
                 * ✅ NOUVEAU : trigger RT propre envoyé aux caméras
                 * Chaque cam renverra ce même impact_id dans RtObservationMsg.
                 */
                RtTriggerCmd trig;
                trig.impact_id = impact_id;
                trig.ts_us = t;

                for (int i = 0; i < op.n_cams; i++) {
                    sendto(master_sock, &trig, sizeof(trig), 0,
                           (struct sockaddr*)&cam_addrs[i],
                           sizeof(struct sockaddr_un));
                }
            }
        }

        /* ==== Observations cams (RT IPC propre) ==== */
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
                    /* paquet incomplet -> ignore */
                    continue;
                }

                /* push dans orchestrateur -> aggregator */
                rt_orch_on_observation(&orch, &obs);

                printf("[RT] obs cam_id=%d impact_id=%llu ts_us=%llu u=%.1f v=%.1f conf=%.2f\n",
       obs.cam_id, (unsigned long long)obs.impact_id,
       (unsigned long long)obs.ts_us,
       obs.u, obs.v, obs.conf);

                
            }
        }

        /* ==== Bundle prêt ? ==== */
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

                printf("[RT][TRIANG] impact_id=%llu  X=%.1fmm Y=%.1fmm Z=%.1fmm  err=%.2fpx  pair=(%d,%d)\n",
                       (unsigned long long)b.impact_id,
                       X_mm, Y_mm, Z_mm,
                       tr.reproj_err_px,
                       op.cam_ids[tr.cam_i], op.cam_ids[tr.cam_j]);

                /*
                 * Prochaine étape :
                 *   -> publier evt/impact/triangulated sur AppBus
                 */
            } else {
                printf("[RT][TRIANG] impact_id=%llu  ECHEC rc=%d (pas de paire valide)\n",
                       (unsigned long long)b.impact_id, rc);
            }
        }
    }

    return 0;
}