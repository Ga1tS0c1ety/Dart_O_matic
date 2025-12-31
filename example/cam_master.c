#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include "triangulation_master.h"

#define SOCKET_PATH "/tmp/dart_ipc.sock"

typedef struct {
    int camera_id;
    double u, v;
} ImpactMessage;


void init_manual_extrinsics(CameraInfo* cams, int n_cams)
{
    if (n_cams < 2) {
        fprintf(stderr, "[EXTRINSIC] Au moins 2 caméras requises\n");
        return;
    }

    /* ===== PARAMÈTRES À AJUSTER ===== */
    const double baseline = 0.45; // distance réelle entre caméras (m)
    /* ================================= */

    /* === Caméra 0 : référence === */
    {
        CameraModel* cam = &cams[0].cam_model;

        memset(cam->RT.R, 0, sizeof(cam->RT.R));
        cam->RT.R[0] = 1.0;
        cam->RT.R[4] = 1.0;
        cam->RT.R[8] = 1.0;

        cam->RT.t[0] = 0.0;
        cam->RT.t[1] = 0.0;
        cam->RT.t[2] = 0.0;

        printf("[EXTRINSIC] Cam %d = référence\n", cams[0].camera_id);
    }

    /* === Caméra 1 : perpendiculaire (90°) === */
    {
        CameraModel* cam = &cams[1].cam_model;

        /* Rotation Ry(-90°) */
        memset(cam->RT.R, 0, sizeof(cam->RT.R));
        cam->RT.R[2] = -1.0;
        cam->RT.R[4] =  1.0;
        cam->RT.R[6] =  1.0;

        /* Translation */
        cam->RT.t[0] = baseline;
        cam->RT.t[1] = 0.0;
        cam->RT.t[2] = 0.0;

        printf("[EXTRINSIC] Cam %d = perpendiculaire\n", cams[1].camera_id);
    }
}


int main(void)
{
    // Définir tes caméras
    CameraInfo cameras[] = {
    {2, "cam_param/camera_params_2.yaml", {0}, 0.0, 0.0, 0},
    {4, "cam_param/camera_params_4.yaml", {0}, 0.0, 0.0, 0}
    };

    int n_cams = 2;

    init_cameras(cameras, n_cams);
    init_manual_extrinsics(cameras, n_cams);

    unlink(SOCKET_PATH);
    int sock = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return -1; }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strcpy(addr.sun_path, SOCKET_PATH);

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind"); return -1;
    }

    printf("[MASTER] En attente des impacts...\n");

    while (1) {
        ImpactMessage msg;
        ssize_t r = recv(sock, &msg, sizeof(msg), 0);
        if (r == sizeof(msg)) {
            printf("[IMPACT] Cam %d → u=%.1f v=%.1f\n",
                   msg.camera_id, msg.u, msg.v);
            handle_impact(cameras, n_cams, msg.camera_id, msg.u, msg.v);
        }
    }

    close(sock);
    unlink(SOCKET_PATH);
    return 0;
}
