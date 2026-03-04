// cam_process.cpp (RT IPC propre + confidence)
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/time.h>
#include <errno.h>

#include "ipc/rt_ipc.h"
#include "vision/dart_detector.h"
#include "vision/usb_camera.h"

#define SOCKET_BASE       "/tmp/dart_cam_"
#define RT_MASTER_SOCKET  "/tmp/dart_master.sock"

static uint64_t now_us() {
    timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
}

static long frame_diff(const unsigned char* a, const unsigned char* b, size_t size) {
    long diff = 0;
    for (size_t i = 0; i < size; i += 16)
        diff += std::abs((int)a[i] - (int)b[i]);
    return diff;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::printf("Usage: %s <camera_id>\n", argv[0]);
        return -1;
    }

    int camera_id = std::atoi(argv[1]);

    /* Socket d'écoute trigger */
    int sock = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return -1; }

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s%d.sock", SOCKET_BASE, camera_id);
    unlink(addr.sun_path);

    if (bind(sock, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        return -1;
    }

    /* Socket envoi vers RT master */
    int sock_master = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (sock_master < 0) { perror("socket master"); return -1; }

    sockaddr_un master_addr{};
    master_addr.sun_family = AF_UNIX;
    std::strncpy(master_addr.sun_path, RT_MASTER_SOCKET, sizeof(master_addr.sun_path) - 1);

    /* Init caméra */
    while (usb_camera_init(camera_id, 1280, 720) != 0) {
        std::printf("[CAM %d] Camera open failed, retry...\n", camera_id);
        std::fflush(stdout);
        usleep(500000);
    }

    int w=0, h=0;
    usb_camera_get_size(&w, &h);
    dart_detector_init(w, h);

    size_t buf_size = (size_t)w * h * 3;
    unsigned char* prev = (unsigned char*)std::malloc(buf_size);
    unsigned char* curr = (unsigned char*)std::malloc(buf_size);
    if (!prev || !curr) {
        std::fprintf(stderr, "[CAM %d] malloc failed\n", camera_id);
        return -1;
    }

    /* Warmup + ref */
    std::printf("[CAM %d] Warmup...\n", camera_id);
    std::fflush(stdout);

    const long WARMUP_DIFF   = 500000;
    const int  WARMUP_STABLE = 10;
    int stable = 0;

    usb_camera_read(prev, buf_size);
    while (stable < WARMUP_STABLE) {
        usb_camera_read(curr, buf_size);
        long d = frame_diff(prev, curr, buf_size);
        stable = (d < WARMUP_DIFF) ? stable + 1 : 0;
        std::memcpy(prev, curr, buf_size);
    }

    dart_detector_set_reference(curr, buf_size);
    std::printf("[CAM %d] Reference ready, waiting RtTriggerCmd\n", camera_id);
    std::fflush(stdout);

    while (1) {
        RtTriggerCmd trig{};
        ssize_t n = recv(sock, &trig, sizeof(trig), 0);
        if (n <= 0) continue;
        if (n != (ssize_t)sizeof(trig)) continue;

        /* Capture post-impact */
        int stable2 = 0;
        const long POST_DIFF   = 500000;
        const int  POST_STABLE = 5;

        while (stable2 < POST_STABLE) {
            std::memcpy(prev, curr, buf_size);
            usb_camera_read(curr, buf_size);
            long d = frame_diff(prev, curr, buf_size);
            stable2 = (d < POST_DIFF) ? stable2 + 1 : 0;
        }

        double u=0, v=0;
        float conf = 0.f;

        int ok = dart_detector_process(curr, buf_size, &u, &v, &conf);
        if (ok == 1) {
            RtObservationMsg msg{};
            msg.impact_id = trig.impact_id;
            msg.cam_id    = camera_id;
            msg.ts_us     = now_us();
            msg.u         = u;
            msg.v         = v;
            msg.conf      = conf;   // ✅ confidence calculée par le detector

            sendto(sock_master, &msg, sizeof(msg), 0,
                   (sockaddr*)&master_addr, sizeof(master_addr));

            std::printf("[CAM %d] sent impact_id=%llu u=%.1f v=%.1f conf=%.2f\n",
                        camera_id, (unsigned long long)msg.impact_id, u, v, conf);
            std::fflush(stdout);
        }

        /* nouvelle référence */
        dart_detector_set_reference(curr, buf_size);
    }

    return 0;
}