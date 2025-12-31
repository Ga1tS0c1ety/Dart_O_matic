// cam_process.c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>

#include "../include/usb_camera.h"
#include "../include/dart_detector.h"

#define SOCKET_PATH "/tmp/dart_ipc.sock"

typedef struct {
    int camera_id;
    double u;
    double v;
} ImpactMessage;

int main(int argc, char** argv)
{
    if (argc != 2) {
        printf("Usage: %s <camera_id>\n", argv[0]);
        return -1;
    }

    int camera_id = atoi(argv[1]);

    // ==================== SOCKET IPC ====================
    int sock = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket");
        return -1;
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strcpy(addr.sun_path, SOCKET_PATH);

    // ==================== CAMERA ====================
    if (usb_camera_init(camera_id, 1280, 720) != 0) {
        printf("Erreur ouverture caméra %d\n", camera_id);
        return -1;
    }

    int w, h;
    usb_camera_get_size(&w, &h);
    dart_detector_init(w, h);

    size_t buf_size = (size_t)w * h * 3;
    unsigned char* buffer = (unsigned char*)malloc(buf_size);

    printf("[CAM %d] Prête (%dx%d)\n", camera_id, w, h);

    // ==================== LOOP ====================
    while (1) {

        if (usb_camera_read(buffer, buf_size) != 0) {
            printf("[CAM %d] Fin du stream\n", camera_id);
            break;
        }

        double u, v;
        int detected = dart_detector_process(buffer, buf_size, &u, &v);

        if (detected) {
            ImpactMessage msg;
            msg.camera_id = camera_id;
            msg.u = u;
            msg.v = v;

            sendto(sock, &msg, sizeof(msg), 0,
                   (struct sockaddr*)&addr, sizeof(addr));
        }
    }

    free(buffer);
    dart_detector_close();
    usb_camera_close();
    close(sock);
    return 0;
}
