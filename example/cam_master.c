// cam_master.c
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <pthread.h>

#include "triangulation_master.h"
#include "mpu6050_thread.h"

/* ================= CONSTANTES ================= */

#define SOCKET_BASE "/tmp/dart_cam_"
#define MASTER_SOCKET "/tmp/dart_master.sock"
#define CMD_TRIGGER 1

/* ================= STRUCTURES ================= */

typedef struct {
    int cmd;
} CamCommand;

typedef struct {
    int camera_id;
    double u, v;
} ImpactMessage;

/* ================= MATRICES EXTRINSEQUE ================= */

void init_manual_extrinsics(CameraInfo* cams, int n_cams)
{

    for (int i = 0; i < n_cams; ++i) {
        char filename[256];
        snprintf(filename, sizeof(filename),
                 "cam_param/camera_extrinsics_%d.yaml",
                 cams[i].camera_id);

        if (load_extrinsics_yaml(filename, &cams[i].cam_model) == 0) {
            printf("[EXTRINSIC] Cam %d charge (%s)\n",
                   cams[i].camera_id, filename);
        } else {
            fprintf(stderr,
                    "[EXTRINSIC] Erreur extrinsques %d\n",
            cams[i].camera_id);
        }
    }
}

/* ================= MPU ================= */

volatile sig_atomic_t mpu_impact = 0;

void mpu_handler(int sig)
{
    if (sig == SIGUSR1)
        mpu_impact = 1;
}

/* ================= MAIN ================= */
int main(void)
{
    /* === CAMRAS CONFIG === */
    CameraInfo cameras[] = {
        {0, "cam_param/camera_params_0.yaml", {0}, 0, 0, 0},
        {2, "cam_param/camera_params_2.yaml", {0}, 0, 0, 0},
        {4, "cam_param/camera_params_4.yaml", {0}, 0, 0, 0},
        {6, "cam_param/camera_params_6.yaml", {0}, 0, 0, 0},
    };
    int n_cams = 4;

    /* ================= SIGNAL MPU ================= */

    struct sigaction sa = {0};
    sa.sa_handler = mpu_handler;
    sa.sa_flags = SA_RESTART;
    sigaction(SIGUSR1, &sa, NULL);

    /* ================= TRIANGULATION ================= */

    init_cameras(cameras, n_cams);
    init_manual_extrinsics(cameras, n_cams);

    /* ================= SOCKET MASTER ================= */

    int master_sock = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (master_sock < 0) {
        perror("socket");
        return -1;
    }

    struct sockaddr_un master_addr = {0};
    master_addr.sun_family = AF_UNIX;
    strcpy(master_addr.sun_path, MASTER_SOCKET);

    unlink(MASTER_SOCKET);

    if (bind(master_sock,
             (struct sockaddr*)&master_addr,
             sizeof(master_addr)) < 0) {
        perror("bind");
        return -1;
    }

    /* ================= SOCKETS CAMRAS ================= */

    struct sockaddr_un cam_addrs[4];

    for (int i = 0; i < n_cams; i++) {
        memset(&cam_addrs[i], 0, sizeof(struct sockaddr_un));
        cam_addrs[i].sun_family = AF_UNIX;
        snprintf(cam_addrs[i].sun_path,
                 sizeof(cam_addrs[i].sun_path),
                 "%s%d.sock",
                 SOCKET_BASE,
                 cameras[i].camera_id);
    }

    /* ================= MPU THREAD ================= */

    pthread_t mpu_tid;
    pthread_create(&mpu_tid, NULL, mpu_thread, NULL);

    printf("[MASTER] System ready, waiting impacts\n");

    /* ================= MAIN LOOP ================= */

    while (1) {

        /* ===== IMPACT MPU ===== */
        if (mpu_impact) {
            mpu_impact = 0;

            CamCommand cmd = { .cmd = CMD_TRIGGER };

            printf("[MASTER] MPU impact -> trigger cameras\n");

            for (int i = 0; i < n_cams; i++) {
                sendto(master_sock,
                       &cmd,
                       sizeof(cmd),
                       0,
                       (struct sockaddr*)&cam_addrs[i],
                       sizeof(struct sockaddr_un));
            }
        }

        /* ===== RCEPTION VISUELLE ===== */
        ImpactMessage msg;
        ssize_t r = recv(master_sock, &msg, sizeof(msg), MSG_DONTWAIT);

        if (r == sizeof(msg)) {
            printf("[MASTER] Cam %d -> u=%.1f v=%.1f\n",
                   msg.camera_id, msg.u, msg.v);

            handle_impact(cameras,
                          n_cams,
                          msg.camera_id,
                          msg.u,
                          msg.v);
        }

        usleep(1000);
    }

    close(master_sock);
    unlink(MASTER_SOCKET);
    return 0;
}
