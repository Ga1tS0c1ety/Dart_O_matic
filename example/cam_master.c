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
    int cam_ref = 4;
    for (int i = 0; i < n_cams; ++i) {
        char filename[256];
        snprintf(filename, sizeof(filename),
                 "cam_param/camera_extrinsics_r%d%d.yaml",cam_ref,
                 cams[i].camera_id);

        if (load_extrinsics_yaml(filename, &cams[i].cam_model) == 0) {
            printf("[EXTRINSIC] Cam %d chargée depuis %s\n",
                   cams[i].camera_id, filename);
        } else {
            fprintf(stderr,
                    "[EXTRINSIC] ÉCHEC chargement extrinsèques cam %d%d\n",cam_ref,
                    cams[i].camera_id);
        }
    }
}



int main(void)
{
    // Définir tes caméras
    CameraInfo cameras[] = {
    {2, "cam_param/camera_params_2.yaml", {0}, 0.0, 0.0, 0},
    {4, "cam_param/camera_params_4.yaml", {0}, 0.0, 0.0, 0},
   {6, "cam_param/camera_params_6.yaml", {0}, 0.0, 0.0, 0},
   {8, "cam_param/camera_params_8.yaml", {0}, 0.0, 0.0, 0},
    };

    int n_cams = 4;

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
