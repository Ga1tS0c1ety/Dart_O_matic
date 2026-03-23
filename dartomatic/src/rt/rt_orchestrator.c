#include "rt/rt_orchestrator.h"
#include <string.h>
#include <stdio.h>

static uint64_t ms_to_us(int ms) { return (uint64_t)ms * 1000ULL; }

/* Retourne l'index 0..n_cams-1 pour un cam_id hardware, sinon -1 */
static int cam_id_to_index(const RtOrchestrator* o, int cam_id) {
    for (int i = 0; i < o->p.n_cams; i++) {
        if (o->p.cam_ids[i] == cam_id) return i;
    }
    return -1;
}

int rt_orch_init(RtOrchestrator* o, RtOrchestratorParams p) {
    if (!o) return -1;
    if (p.n_cams <= 0 || p.n_cams > 8) return -1;
    if (p.min_cams <= 0 || p.min_cams > p.n_cams) return -1;
    if (p.window_ms <= 0) return -1;
    if (p.cooldown_ms < 0) return -1;

    memset(o, 0, sizeof(*o));
    o->p = p;

    AggregatorParams ap = {
        .n_cams = p.n_cams,
        .min_cams = p.min_cams,
        .window_ms = p.window_ms
    };
    if (aggregator_init(&o->ag, ap) != 0) return -1;

    rt_orch_reset(o);
    return 0;
}

void rt_orch_reset(RtOrchestrator* o) {
    if (!o) return;

    o->state = ORCH_IDLE;
    o->next_impact_id = 1;
    o->current_impact_id = 0;
    o->cooldown_until_us = 0;

    aggregator_reset(&o->ag);
}

OrchState rt_orch_state(const RtOrchestrator* o) {
    return o ? o->state : ORCH_IDLE;
}

uint64_t rt_orch_current_impact_id(const RtOrchestrator* o) {
    return o ? o->current_impact_id : 0;
}

int rt_orch_on_mpu_impact(RtOrchestrator* o, uint64_t now_us) {
    if (!o) return 0;

    /*
     * Gating :
     * - si COOLDOWN: ignore
     * - si ARMED: ignore (anti-rebond simple)
     * - si IDLE: accepte -> trigger
     */
    if (o->state == ORCH_COOLDOWN) return 0;
    if (o->state == ORCH_ARMED) return 0;

    /* IDLE -> ARMED */
    o->current_impact_id = o->next_impact_id++;
    aggregator_on_trigger(&o->ag, o->current_impact_id, now_us);
    o->state = ORCH_ARMED;

    /* (Ici, dans la vraie version, tu enverras RtTriggerCmd aux cam_process) */
    return 1;
}

void rt_orch_on_observation(RtOrchestrator* o, const RtObservationMsg* msg) {
    if (!o || !msg) return;

    /* On ne collecte que si un impact est en cours */
    if (o->state != ORCH_ARMED) return;

    int cam_index = cam_id_to_index(o, msg->cam_id);
    if (cam_index < 0) {
        /* cam inconnue -> ignore */
        return;
    }

    aggregator_on_observation(&o->ag, cam_index, msg);
}

void rt_orch_tick(RtOrchestrator* o, uint64_t now_us) {
    if (!o) return;

    if (o->state == ORCH_COOLDOWN) {
        if (now_us >= o->cooldown_until_us) {
            o->state = ORCH_IDLE;
            printf("[RT][ORCH] cooldown fini -> IDLE\n");
        }
        return;
    }

    if (o->state == ORCH_ARMED) {
        aggregator_tick(&o->ag, now_us);

        /*
         * Cas important :
         * l'aggregator peut s'être reset sans bundle
         * (ex: fenêtre expirée avec pas assez d'obs).
         * Dans ce cas, il faut sortir de ARMED,
         * sinon les prochains impacts MPU seront ignorés.
         */
        if (!aggregator_is_active(&o->ag)) {
            o->state = ORCH_COOLDOWN;
            o->cooldown_until_us = now_us + ms_to_us(o->p.cooldown_ms);
            printf("[RT][ORCH] aggregator inactif sans bundle -> COOLDOWN\n");
        }
        return;
    }
}

int rt_orch_poll_bundle(RtOrchestrator* o, ImpactBundle* out) {
    if (!o || !out) return 0;
    if (o->state != ORCH_ARMED) return 0;

    /*
     * Si aggregator sort un bundle :
     * - on le renvoie
     * - on passe en cooldown
     */
    if (aggregator_poll_ready(&o->ag, out)) {
        o->state = ORCH_COOLDOWN;
        o->cooldown_until_us = out->ts_trigger_us + ms_to_us(o->p.cooldown_ms);
        return 1;
    }

    return 0;
}