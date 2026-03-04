#include "rt/rt_orchestrator.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>

static uint64_t now_us() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
}

static int irand(int a, int b) { return a + (rand() % (b - a + 1)); }

static const char* state_str(OrchState s) {
    switch (s) {
        case ORCH_IDLE: return "IDLE";
        case ORCH_ARMED: return "ARMED";
        case ORCH_COOLDOWN: return "COOLDOWN";
        default: return "?";
    }
}

int main() {
    srand((unsigned int)time(NULL));

    RtOrchestrator o;
    RtOrchestratorParams p;
    p.n_cams = 4;
    p.cam_ids[0] = 0;
    p.cam_ids[1] = 2;
    p.cam_ids[2] = 4;
    p.cam_ids[3] = 6;
    p.min_cams = 2;
    p.window_ms = 150;
    p.cooldown_ms = 250;

    if (rt_orch_init(&o, p) != 0) {
        fprintf(stderr, "rt_orch_init failed\n");
        return 1;
    }

    printf("=== rt_orch_sim ===\n");
    printf("cams={%d,%d,%d,%d} min=%d window=%dms cooldown=%dms\n",
           p.cam_ids[0], p.cam_ids[1], p.cam_ids[2], p.cam_ids[3],
           p.min_cams, p.window_ms, p.cooldown_ms);

    uint64_t next_trigger = now_us() + 300000ULL;

    while (1) {
        uint64_t t = now_us();

        /* Simule un impact MPU toutes ~800ms */
        if (t >= next_trigger) {
            OrchState before = rt_orch_state(&o);
            int accepted = rt_orch_on_mpu_impact(&o, t);
            OrchState after = rt_orch_state(&o);

            if (accepted) {
                printf("\n[SIM] MPU impact ACCEPTED -> impact_id=%llu (%s -> %s)\n",
                       (unsigned long long)rt_orch_current_impact_id(&o),
                       state_str(before), state_str(after));
            } else {
                printf("\n[SIM] MPU impact IGNORED (state=%s)\n", state_str(before));
            }

            /* Prochain trigger */
            next_trigger = t + (uint64_t)irand(700000, 900000);
        }

        /*
         * Si ARMED, on envoie des observations aléatoires :
         * - 80% chance qu’une caméra réponde
         * - délai aléatoire 5..120ms depuis le trigger
         *
         * Pour rester simple, on génère quelques messages au fil du temps.
         */
        if (rt_orch_state(&o) == ORCH_ARMED) {
            uint64_t impact_id = rt_orch_current_impact_id(&o);

            /* tenter d'envoyer 0..2 observations par tick */
            int tries = irand(0, 2);
            for (int k = 0; k < tries; k++) {
                int cam_hw_ids[4] = {0,2,4,6};
                int cam = cam_hw_ids[irand(0, 3)];

                /* 80% chance de "réponse" */
                if (irand(0, 99) >= 80) continue;

                RtObservationMsg msg;
                msg.impact_id = impact_id;
                msg.cam_id = cam;
                msg.ts_us = t;
                msg.u = (double)irand(100, 1180);
                msg.v = (double)irand(100, 620);
                msg.conf = (float)irand(50, 99) / 100.0f;

                rt_orch_on_observation(&o, &msg);

                printf("[SIM] obs cam_id=%d u=%.1f v=%.1f conf=%.2f\n",
                       msg.cam_id, msg.u, msg.v, msg.conf);
            }
        }

        rt_orch_tick(&o, t);

        ImpactBundle b;
        if (rt_orch_poll_bundle(&o, &b)) {
            printf("[SIM] BUNDLE READY impact_id=%llu obs=%d -> state=%s\n",
                   (unsigned long long)b.impact_id, b.obs_count, state_str(rt_orch_state(&o)));

            for (int i = 0; i < b.obs_count; i++) {
                printf("      - cam_index=%d u=%.1f v=%.1f conf=%.2f\n",
                       b.obs[i].cam_index, b.obs[i].u, b.obs[i].v, b.obs[i].conf);
            }
        }

        usleep(2000); /* 2ms */
    }

    return 0;
}