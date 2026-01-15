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
#define SNAPSHOT_DELAY_US 100000  // 100 ms → 10 Hz par caméra

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

    /* ==================== SOCKET IPC ==================== */
    int sock = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket");
        return -1;
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strcpy(addr.sun_path, SOCKET_PATH);

    printf("[CAM %d] Process démarré (mode snapshot)\n", camera_id);

    /* ==================== LOOP SNAPSHOT ==================== */
    while (1) {

        /* --- OUVERTURE CAMERA --- */
        if (usb_camera_init(camera_id, 1280, 720) != 0) {
            printf("[CAM %d] Échec ouverture caméra\n", camera_id);
            usleep(500000); // 500 ms avant retry
            continue;
        }

        int w, h;
        usb_camera_get_size(&w, &h);

        static int detector_initialized = 0;
        if (!detector_initialized) {
            dart_detector_init(w, h);
            detector_initialized = 1;
        }

        size_t buf_size = (size_t)w * h * 3;
        unsigned char* buffer = (unsigned char*)malloc(buf_size);
        if (!buffer) {
            perror("malloc");
            usb_camera_close();
            break;
        }

        /* --- CAPTURE UNE IMAGE --- */
        if (usb_camera_read(buffer, buf_size) == 0) {

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

        /* --- NETTOYAGE CAMERA --- */
        free(buffer);
        usb_camera_close();

        /* --- DÉLAI POUR STABILITÉ USB --- */
        usleep(SNAPSHOT_DELAY_US);
    }

    dart_detector_close();
    close(sock);
    return 0;
}
