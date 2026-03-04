#include "rt/aggregator.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>

/* now en microsecondes */
static uint64_t now_us() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
}

/* random int dans [a, b] */
static int irand(int a, int b) {
    return a + (rand() % (b - a + 1));
}

int main() {
    srand((unsigned int)time(NULL));

    Aggregator ag;
    AggregatorParams p = {
        .n_cams = 4,
        .min_cams = 2,
        .window_ms = 150
    };

    if (aggregator_init(&ag, p) != 0) {
        fprintf(stderr, "aggregator_init failed\n");
        return 1;
    }

    printf("=== rt_agg_test ===\n");
    printf("n_cams=%d min_cams=%d window_ms=%d\n", p.n_cams, p.min_cams, p.window_ms);

    uint64_t impact_id = 1;

    while (1) {
        uint64_t ts = now_us();
        printf("\n[TEST] TRIGGER impact_id=%llu\n", (unsigned long long)impact_id);
        aggregator_on_trigger(&ag, impact_id, ts);

        /*
         * On simule des observations : certaines cams répondent,
         * d'autres peuvent être "en retard" ou ne pas répondre.
         */
        int will_send[4];
        for (int i = 0; i < 4; i++) {
            /* 80% de chance qu'une caméra réponde */
            will_send[i] = (irand(0, 99) < 80) ? 1 : 0;
        }

        uint64_t start = now_us();
        ImpactBundle b;

        while (aggregator_is_active(&ag)) {
            uint64_t t = now_us();

            /* Simule l'arrivée d'observations pendant ~120ms */
            for (int cam = 0; cam < 4; cam++) {
                if (!will_send[cam]) continue;

                /* Envoyer une obs à un moment aléatoire entre 5ms et 120ms */
                int delay_us = irand(5000, 120000);

                if ((t - start) >= (uint64_t)delay_us) {
                    /* On envoie UNE observation et on désactive */
                    RtObservationMsg msg;
                    msg.impact_id = impact_id;
                    msg.cam_id = cam; /* ici cam_id == index pour le test */
                    msg.ts_us = t;
                    msg.u = (double)irand(100, 1180);
                    msg.v = (double)irand(100, 620);
                    msg.conf = (float)irand(50, 99) / 100.0f;

                    aggregator_on_observation(&ag, cam, &msg);
                    will_send[cam] = 0;

                    printf("[TEST] obs cam=%d u=%.1f v=%.1f conf=%.2f\n",
                           cam, msg.u, msg.v, msg.conf);
                }
            }

            aggregator_tick(&ag, now_us());

            if (aggregator_poll_ready(&ag, &b)) {
                printf("[TEST] READY impact_id=%llu obs_count=%d\n",
                       (unsigned long long)b.impact_id, b.obs_count);

                for (int k = 0; k < b.obs_count; k++) {
                    printf("   - cam=%d u=%.1f v=%.1f conf=%.2f\n",
                           b.obs[k].cam_index, b.obs[k].u, b.obs[k].v, b.obs[k].conf);
                }
            }

            usleep(1000); /* 1ms */
        }

        impact_id++;
        usleep(300000); /* 300ms entre tirs */
    }

    return 0;
}