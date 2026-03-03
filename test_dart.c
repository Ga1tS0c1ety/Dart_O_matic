#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>

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

int main(void)
{
    /* Signal */
    struct sigaction sa;
    sa.sa_handler = impact_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    sigaction(SIGUSR1, &sa, NULL);

    /* Camera */
    usb_camera_init(2, 1280, 720);

    int w, h;
    usb_camera_get_size(&w, &h);
    dart_detector_init(w, h);

    size_t buf_size = (size_t)w * h * 3;
    unsigned char* buffer = malloc(buf_size);

    if (!buffer) {
        perror("malloc");
        return 1;
    }

    /* ================= CAMERA WARMUP ================= */

    printf("Camera warmup...\n");
    fflush(stdout);

    for (int i = 0; i < 30; i++) {
        usb_camera_read(buffer, buf_size);
        usleep(30000); /* ~30 fps */
    }

    /* ================= REFERENCE ================= */

    usb_camera_read(buffer, buf_size);
    dart_detector_set_reference(buffer, buf_size);

    printf("Reference captured\n");
    fflush(stdout);

    /* ================= MPU THREAD ================= */

    pthread_t mpu_tid;
    pthread_create(&mpu_tid, NULL, mpu_thread, NULL);

    printf("System ready. Waiting for impact...\n");
    fflush(stdout);

    /* ================= MAIN LOOP ================= */

    while (1) {

        if (impact_signal) {
            impact_signal = 0;

            /* Short post-impact delay */
            usleep(5000);

            printf("Impact signal received\n");
            fflush(stdout);

            usb_camera_read(buffer, buf_size);

            double u, v;
            int detected = dart_detector_process(buffer, buf_size, &u, &v);

            if (detected) {
                printf("IMPACT detected at u=%.1f v=%.1f\n", u, v);
                fflush(stdout);
            }
        }

        usleep(10);
    }

    free(buffer);
    dart_detector_close();
    usb_camera_close();
    return 0;
}
