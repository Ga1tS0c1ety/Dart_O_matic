#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>

#include "../include/usb_camera.h"
#include "../include/dart_detector.h"
#include "mpu6050_thread.h"

/* ================= SIGNAL ================= */

volatile sig_atomic_t impact_signal = 0;

void impact_handler(int sig)
{
    if (sig == SIGUSR1)
        impact_signal = 1;
}

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
        printf("Usage : %s <camera_id>\n", argv[0]);
        return -1;
    }

    int camera_id = atoi(argv[1]);
    
    /* Signal */
    struct sigaction sa;
    sa.sa_handler = impact_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    sigaction(SIGUSR1, &sa, NULL);

    /* Camera */
    usb_camera_init(camera_id, 1280, 720);

    int w, h;
    usb_camera_get_size(&w, &h);
    dart_detector_init(w, h);

    size_t buf_size = (size_t)w * h * 3;

    unsigned char* frame_prev = (unsigned char*)malloc(buf_size);
    unsigned char* frame_curr = (unsigned char*)malloc(buf_size);

    if (!frame_prev || !frame_curr) {
        perror("malloc");
        return 1;
    }

    /* ================= ADAPTIVE WARMUP ================= */

    printf("Camera warmup (adaptive)...\n");
    fflush(stdout);

    const long WARMUP_DIFF = 500000;
    const int WARMUP_STABLE = 10;

    int stable_count = 0;

    usb_camera_read(frame_prev, buf_size);

    while (stable_count < WARMUP_STABLE) {

        usb_camera_read(frame_curr, buf_size);

        long diff = frame_diff(frame_prev, frame_curr, buf_size);
        if (diff < WARMUP_DIFF)
            stable_count++;
        else
            stable_count = 0;

        memcpy(frame_prev, frame_curr, buf_size);
    }

    dart_detector_set_reference(frame_curr, buf_size);

    printf("Camera stable. Reference captured.\n");
    fflush(stdout);

    /* ================= MPU THREAD ================= */

    pthread_t mpu_tid;
    pthread_create(&mpu_tid, NULL, mpu_thread, NULL);

    printf("System ready. Waiting for impact...\n");
    fflush(stdout);

    /* ================= POST IMPACT LOGIC ================= */

    int waiting_visual = 0;
    int visual_stable = 0; 

    const long POST_DIFF = 500000;
    const int POST_STABLE_FRAMES = 5;

    /* ================= MAIN LOOP ================= */

    while (1) {

        memcpy(frame_prev, frame_curr, buf_size);
        usb_camera_read(frame_curr, buf_size);

        /* Impact received */
        if (impact_signal) {
            impact_signal = 0;
            waiting_visual = 1;
            visual_stable = 0;

            printf("Impact detected, waiting visual stability\n");
            fflush(stdout);
        }

        /* Wait for arrow to stop */
        if (waiting_visual) {

            long diff = frame_diff(frame_prev, frame_curr, buf_size);

            if (diff < POST_DIFF)
                visual_stable++;
            else
                visual_stable = 0;

            if (visual_stable >= POST_STABLE_FRAMES) {

                printf("Visual stable, processing\n");
                fflush(stdout);

                double u, v;
                int detected = dart_detector_process(
                    frame_curr,
                    buf_size,
                    &u,
                    &v
                );

                if (detected) {
                    printf("IMPACT detected at u=%.1f v=%.1f\n", u, v);
                    fflush(stdout);
                }

                dart_detector_set_reference(frame_curr, buf_size);

                waiting_visual = 0;
            }
        }

        usleep(1000);
    }

    return 0;
}
