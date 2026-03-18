#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <pthread.h>
#include <sys/select.h>

#include "vision/usb_camera.h"
#include "vision/dart_detector.h"
#include "rt/mpu_adapter.h"
#include "rt/mpu6050_thread.h"

/*
 * debug_dart_detector
 * -------------------
 * Tool de debug vision piloté par le MPU, version cohérente
 * avec l'architecture actuelle (MpuAdapter + pipe).
 *
 * Usage:
 *   ./build/debug_dart_detector <camera_id>
 *
 * Fonctionnement:
 *   - ouvre 1 caméra
 *   - capture une référence stable
 *   - démarre le thread MPU
 *   - attend un impact via MpuAdapter
 *   - attend que la scène se stabilise visuellement
 *   - lance dart_detector_process()
 *   - met à jour la référence
 *
 * Important:
 *   - prévu pour UNE seule caméra à la fois
 *   - pour voir les fenêtres debug du detector,
 *     active le debug dans dart_detector.cpp
 */

/* ========================================================= */
/* Utils                                                     */
/* ========================================================= */

static long frame_diff(const unsigned char* a,
                       const unsigned char* b,
                       size_t size)
{
    long diff = 0;
    for (size_t i = 0; i < size; i += 16) {
        diff += std::abs((int)a[i] - (int)b[i]);
    }
    return diff;
}

/* ========================================================= */
/* main                                                      */
/* ========================================================= */

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::printf("Usage: %s <camera_id>\n", argv[0]);
        return -1;
    }

    int camera_id = std::atoi(argv[1]);
    if (camera_id < 0) {
        std::fprintf(stderr, "[DEBUG_DART] camera_id invalide\n");
        return -1;
    }

    /* ----------------------------------------------------- */
    /* Caméra                                                */
    /* ----------------------------------------------------- */
    usb_camera_set_display_enabled(1);
    if (usb_camera_init(camera_id, 1280, 720) != 0) {
        std::fprintf(stderr, "[DEBUG_DART] impossible d'ouvrir la caméra %d\n", camera_id);
        return -1;
    }

    


    int w = 0, h = 0;
    usb_camera_get_size(&w, &h);

    dart_detector_set_debug_enabled(1);

    if (dart_detector_init(w, h) != 0) {
        std::fprintf(stderr, "[DEBUG_DART] dart_detector_init failed\n");
        usb_camera_close();
        return -1;
    }
    

    size_t buf_size = (size_t)w * h * 3;
    unsigned char* frame_prev = (unsigned char*)std::malloc(buf_size);
    unsigned char* frame_curr = (unsigned char*)std::malloc(buf_size);

    if (!frame_prev || !frame_curr) {
        std::fprintf(stderr, "[DEBUG_DART] malloc failed\n");
        dart_detector_close();
        usb_camera_close();
        dart_detector_set_debug_enabled(0);
        usb_camera_set_display_enabled(0);
        return -1;
    }

    /* ----------------------------------------------------- */
    /* Warmup + capture référence                            */
    /* ----------------------------------------------------- */
    std::printf("[DEBUG_DART] warmup camera...\n");
    std::fflush(stdout);

    const long WARMUP_DIFF = 500000;
    const int WARMUP_STABLE = 10;
    int stable_count = 0;

    if (usb_camera_read(frame_prev, buf_size) != 0) {
        std::fprintf(stderr, "[DEBUG_DART] lecture initiale impossible\n");
        std::free(frame_prev);
        std::free(frame_curr);
        dart_detector_close();
        usb_camera_close();
        dart_detector_set_debug_enabled(0);
        usb_camera_set_display_enabled(0);
        return -1;
    }

    while (stable_count < WARMUP_STABLE) {
        if (usb_camera_read(frame_curr, buf_size) != 0) {
            std::fprintf(stderr, "[DEBUG_DART] erreur lecture pendant warmup\n");
            std::free(frame_prev);
            std::free(frame_curr);
            dart_detector_close();
            usb_camera_close();
            dart_detector_set_debug_enabled(0);
        usb_camera_set_display_enabled(0);
            return -1;
        }

        long d = frame_diff(frame_prev, frame_curr, buf_size);
        stable_count = (d < WARMUP_DIFF) ? (stable_count + 1) : 0;

        std::memcpy(frame_prev, frame_curr, buf_size);
    }

    dart_detector_set_reference(frame_curr, buf_size);

    std::printf("[DEBUG_DART] référence initiale capturée\n");
    std::fflush(stdout);

    /* ----------------------------------------------------- */
    /* MPU Adapter + thread MPU                              */
    /* ----------------------------------------------------- */
    MpuAdapter mpu;
    if (mpu_adapter_init(&mpu) != 0) {
        std::fprintf(stderr, "[DEBUG_DART] mpu_adapter_init failed\n");
        std::free(frame_prev);
        std::free(frame_curr);
        dart_detector_close();
        usb_camera_close();
        dart_detector_set_debug_enabled(0);
        usb_camera_set_display_enabled(0);
        return -1;
    }

    pthread_t mpu_tid;
    if (pthread_create(&mpu_tid, NULL, mpu_thread, &mpu) != 0) {
        std::fprintf(stderr, "[DEBUG_DART] impossible de lancer le thread MPU\n");
        std::free(frame_prev);
        std::free(frame_curr);
        dart_detector_close();
        usb_camera_close();
        dart_detector_set_debug_enabled(0);
        usb_camera_set_display_enabled(0);
        return -1;
    }

    std::printf("[DEBUG_DART] système prêt - attente impact MPU\n");
    std::fflush(stdout);

    /* ----------------------------------------------------- */
    /* Logique post-impact                                   */
    /* ----------------------------------------------------- */
    int waiting_visual = 0;
    int visual_stable = 0;

    const long POST_DIFF = 500000;
    const int POST_STABLE_FRAMES = 5;

    int mfd = mpu_adapter_fd(&mpu);

    while (1) {
        /*
         * Lire en continu la caméra pour garder un flux vivant,
         * et en parallèle surveiller le pipe MPU.
         */
        std::memcpy(frame_prev, frame_curr, buf_size);

        if (usb_camera_read(frame_curr, buf_size) != 0) {
            std::fprintf(stderr, "[DEBUG_DART] erreur lecture caméra\n");
            break;
        }

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(mfd, &rfds);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 1000; /* 1 ms */

        int r = select(mfd + 1, &rfds, NULL, NULL, &tv);

        if (r > 0 && FD_ISSET(mfd, &rfds)) {
            unsigned char buf[64];
            while (read(mfd, buf, sizeof(buf)) > 0) {}

            waiting_visual = 1;
            visual_stable = 0;

            std::printf("[DEBUG_DART] impact MPU reçu -> attente stabilité visuelle\n");
            std::fflush(stdout);
        }

        if (waiting_visual) {
            long diff = frame_diff(frame_prev, frame_curr, buf_size);

            if (diff < POST_DIFF) {
                visual_stable++;
            } else {
                visual_stable = 0;
            }

            if (visual_stable >= POST_STABLE_FRAMES) {
                std::printf("[DEBUG_DART] scène stable -> traitement\n");
                std::fflush(stdout);

                double u = 0.0, v = 0.0;
                float conf = 0.0f;

                int rc = dart_detector_process(frame_curr, buf_size, &u, &v, &conf);
                if (rc == 1) {
                    std::printf("[DEBUG_DART] IMPACT détecté u=%.1f v=%.1f conf=%.2f\n",
                                u, v, conf);
                } else if (rc == 0) {
                    std::printf("[DEBUG_DART] aucun impact détecté\n");
                } else {
                    std::printf("[DEBUG_DART] erreur detector\n");
                }
                std::fflush(stdout);

                dart_detector_set_reference(frame_curr, buf_size);

                waiting_visual = 0;
                visual_stable = 0;

                std::printf("[DEBUG_DART] référence mise à jour - attente prochain impact\n");
                std::fflush(stdout);
            }
        }

        usleep(1000);
    }

    std::free(frame_prev);
    std::free(frame_curr);

    dart_detector_close();
    usb_camera_close();
    dart_detector_set_debug_enabled(0);
    usb_camera_set_display_enabled(0);
    return 0;
}