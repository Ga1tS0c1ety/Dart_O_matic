// cam_process.c
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>

#include "../include/usb_camera.h"
#include "../include/dart_detector.h"

/* ================= CONSTANTES ================= */

#define SOCKET_BASE "/tmp/dart_cam_"
#define CMD_TRIGGER 1

/* ================= STRUCTURES ================= */

typedef struct {
    int cmd;   // CMD_TRIGGER
} CamCommand;

typedef struct {
    int camera_id;
    double u, v;
} ImpactMessage;

/* ================= UTILS ================= */

static long frame_diff(const unsigned char* a,
                       const unsigned char* b,
                       size_t size)
{
    long diff = 0;
    for (size_t i = 0; i < size; i += 16)
        diff += abs(a[i] - b[i]);
    return diff;
}

/* ================= MAIN ================= */

int main(int argc, char** argv)
{
    if (argc != 2) {
        printf("Usage: %s <camera_id>\n", argv[0]);
        return -1;
    }

    int camera_id = atoi(argv[1]);

    /* ================= SOCKET IPC ================= */

    int sock = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket");
        return -1;
    }

    struct sockaddr_un addr = {0};
    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, sizeof(addr.sun_path),
             "%s%d.sock", SOCKET_BASE, camera_id);

    unlink(addr.sun_path);

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        return -1;
    }
    
    /* ================= SOCKET MASTER ================= */
    int sock_master = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (sock_master < 0) { perror("socket master"); exit(-1); }

    struct sockaddr_un master_addr = {0};
    master_addr.sun_family = AF_UNIX;
    strcpy(master_addr.sun_path, "/tmp/dart_master.sock");


    /* ================= CAMERA INIT ================= */

    while (usb_camera_init(camera_id, 1280, 720) != 0) {
    printf("[CAM %d] Camera open failed, retry...\n", camera_id);
    fflush(stdout);
    usleep(500000); // 500 ms
    }

    int w, h;
    usb_camera_get_size(&w, &h);
    dart_detector_init(w, h);

    size_t buf_size = (size_t)w * h * 3;
    unsigned char* prev = (unsigned char*)malloc(buf_size);
    unsigned char* curr = (unsigned char*)malloc(buf_size);

    /* ================= WARMUP + REFERENCE ================= */

    printf("[CAM %d] Warmup...\n", camera_id);
    fflush(stdout);

    const long WARMUP_DIFF   = 500000;
    const int  WARMUP_STABLE = 10;
    int stable = 0;

    usb_camera_read(prev, buf_size);

    while (stable < WARMUP_STABLE) {
        usb_camera_read(curr, buf_size);
        long d = frame_diff(prev, curr, buf_size);
     //   printf("Diff cam %d = %ld\n", camera_id,d);
        stable = (d < WARMUP_DIFF) ? stable + 1 : 0;
        memcpy(prev, curr, buf_size);
    }

    dart_detector_set_reference(curr, buf_size);

    printf("[CAM %d] Reference ready, waiting trigger\n", camera_id);
    fflush(stdout);

    /* ================= MAIN LOOP ================= */

    while (1) {

        CamCommand cmd;
        ssize_t n = recv(sock, &cmd, sizeof(cmd), 0);
        if (n <= 0)
            continue;

        if (cmd.cmd != CMD_TRIGGER)
            continue;

        /* ===== POST IMPACT CAPTURE ===== */

        int stable = 0;
        const long POST_DIFF   = 500000;
        const int  POST_STABLE = 5;

        while (stable < POST_STABLE) {
            memcpy(prev, curr, buf_size);
            usb_camera_read(curr, buf_size);
            long d = frame_diff(prev, curr, buf_size);
            stable = (d < POST_DIFF) ? stable + 1 : 0;
        }

        double u, v;
        if (dart_detector_process(curr, buf_size, &u, &v)) {

            ImpactMessage msg = { camera_id, u, v };

            sendto(sock_master, &msg, sizeof(msg), 0,
       (struct sockaddr*)&master_addr, sizeof(master_addr));

            printf("[CAM %d] Impact sent u=%.1f v=%.1f\n",
                   camera_id, u, v);
            fflush(stdout);
        }

        dart_detector_set_reference(curr, buf_size);
    }
}
