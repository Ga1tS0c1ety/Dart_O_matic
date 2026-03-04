#include "appbus/client.h"
#include "ipc/appbus_ipc.h"
#include "appbus/topics.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>

/*
 * bridge_fake_impact
 * ------------------
 * Simule la sortie du RT middleware.
 *
 * Publie sur :
 *   evt/impact/triangulated
 *
 * Payload JSON :
 * {
 *   "impact_id":1,
 *   "ts_us":123456789,
 *   "x_mm":12.3,
 *   "y_mm":-45.6,
 *   "z_mm":3.2,
 *   "quality":0.92
 * }
 */

static double frand_range(double a, double b) {
    double r = (double)rand() / (double)RAND_MAX;
    return a + (b - a) * r;
}

/* Timestamp en microsecondes */
static uint64_t now_us() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000ULL + tv.tv_usec;
}

int main(int argc, char** argv) {

    double rate_hz = (argc >= 2) ? atof(argv[1]) : 2.0;
    if (rate_hz <= 0.0) rate_hz = 2.0;

    const char* sock = (argc >= 3) ? argv[2] : APPBUS_DEFAULT_SOCK;

    unsigned int period_us = (unsigned int)(1000000.0 / rate_hz);

    printf("[bridge_fake_impact] rate=%.2f Hz, sock=%s\n", rate_hz, sock);

    AppBusClient* c = appbus_connect(sock);
    if (!c) {
        fprintf(stderr, "Impossible de se connecter à AppBus\n");
        return 2;
    }

    srand((unsigned int)time(NULL));

    uint64_t impact_id = 1;

    while (1) {

        /* Position 3D simulée */
        double x_mm = frand_range(-170.0, 170.0);
        double y_mm = frand_range(-170.0, 170.0);

        /*
         * z_mm :
         * Pour une cible réelle, z serait proche de 0
         * (plan de la cible).
         * On simule une petite variation.
         */
        double z_mm = frand_range(-5.0, 5.0);

        double quality = frand_range(0.80, 0.99);

        uint64_t ts = now_us();

        char payload[512];
        snprintf(payload, sizeof(payload),
                 "{"
                 "\"impact_id\":%llu,"
                 "\"ts_us\":%llu,"
                 "\"x_mm\":%.2f,"
                 "\"y_mm\":%.2f,"
                 "\"z_mm\":%.2f,"
                 "\"quality\":%.3f"
                 "}",
                 (unsigned long long)impact_id,
                 (unsigned long long)ts,
                 x_mm, y_mm, z_mm, quality);

        int rc = appbus_publish(c, TOPIC_EVT_IMPACT_TRIANG, payload);
        if (rc != 0) {
            fprintf(stderr, "Erreur publish\n");
            appbus_close(c);
            return 3;
        }

        printf("[bridge_fake_impact] %s\n", payload);
        fflush(stdout);

        impact_id++;
        usleep(period_us);
    }

    appbus_close(c);
    return 0;
}