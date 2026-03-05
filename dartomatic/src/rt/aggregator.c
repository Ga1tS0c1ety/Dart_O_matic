#include "rt/aggregator.h"
#include <string.h>
#include <stdio.h>

/* Convertit ms -> us */
static uint64_t ms_to_us(int ms) {
    return (uint64_t)ms * 1000ULL;
}

int aggregator_init(Aggregator* ag, AggregatorParams p) {
    if (!ag) return -1;
    if (p.n_cams <= 0 || p.n_cams > 8) return -1;
    if (p.min_cams <= 0 || p.min_cams > p.n_cams) return -1;
    if (p.window_ms <= 0) return -1;

    memset(ag, 0, sizeof(*ag));
    ag->p = p;
    aggregator_reset(ag);
    return 0;
}

void aggregator_reset(Aggregator* ag) {
    if (!ag) return;
    ag->active = 0;
    ag->impact_id = 0;
    ag->ts_trigger_us = 0;
    ag->deadline_us = 0;
    ag->expired = 0;

    for (int i = 0; i < 8; i++) {
        ag->has_obs[i] = 0;
        memset(&ag->best_obs[i], 0, sizeof(AggObservation));
        ag->best_obs[i].cam_index = i;
    }
}

int aggregator_is_active(const Aggregator* ag) {
    return ag ? ag->active : 0;
}

void aggregator_on_trigger(Aggregator* ag, uint64_t impact_id, uint64_t ts_us) {
    if (!ag) return;

    /* V1 : on écrase l'état et on repart proprement */
    aggregator_reset(ag);

    ag->active = 1;
    ag->impact_id = impact_id;
    ag->ts_trigger_us = ts_us;
    ag->deadline_us = ts_us + ms_to_us(ag->p.window_ms);

    printf("[AGG][DBG] TRIGGER impact_id=%llu ts=%llu deadline=%llu window_ms=%d\n",
       (unsigned long long)ag->impact_id,
       (unsigned long long)ag->ts_trigger_us,
       (unsigned long long)ag->deadline_us,
       ag->p.window_ms);
    ag->expired = 0;
}

void aggregator_on_observation(Aggregator* ag, int cam_index, const RtObservationMsg* msg) {
    if (!ag || !msg) return;
    if (!ag->active) return;

    /* On ne prend que les obs pour l'impact courant */
    if (msg->impact_id != ag->impact_id) return;

    if (cam_index < 0 || cam_index >= ag->p.n_cams) return;

    /*
     * Politique : 1 obs par caméra.
     * Si on reçoit plusieurs obs, on garde celle avec meilleure confidence.
     */
    if (!ag->has_obs[cam_index] || msg->conf > ag->best_obs[cam_index].conf) {
        ag->has_obs[cam_index] = 1;
        ag->best_obs[cam_index].cam_index = cam_index;
        ag->best_obs[cam_index].u = msg->u;
        ag->best_obs[cam_index].v = msg->v;
        ag->best_obs[cam_index].conf = msg->conf;
        ag->best_obs[cam_index].ts_us = msg->ts_us;
    }
}

void aggregator_tick(Aggregator* ag, uint64_t now_us) {
    if (!ag) return;
    if (!ag->active) return;

    // log rare: toutes les ~1000 itérations si tu veux éviter le spam,
    // mais pour debug tu peux le laisser brut quelques secondes
    if (now_us >= ag->deadline_us && !ag->expired) {
        printf("[AGG][DBG] EXPIRE now=%llu deadline=%llu impact_id=%llu\n",
               (unsigned long long)now_us,
               (unsigned long long)ag->deadline_us,
               (unsigned long long)ag->impact_id);
        ag->expired = 1;
    } else if (!ag->expired) {
        // optionnel: log une fois pour voir l'écart (à enlever après)
        static int once = 0;
        if (!once) {
            once = 1;
            printf("[AGG][DBG] TICK now=%llu deadline=%llu (delta=%lldus)\n",
                   (unsigned long long)now_us,
                   (unsigned long long)ag->deadline_us,
                   (long long)(ag->deadline_us - now_us));
        }
    }
}

static int count_obs(const Aggregator* ag) {
    int c = 0;
    for (int i = 0; i < ag->p.n_cams; i++) {
        if (ag->has_obs[i]) c++;
    }
    return c;
}

int aggregator_poll_ready(Aggregator* ag, ImpactBundle* out) {
    if (!ag || !out) return 0;
    if (!ag->active) return 0;

    int c = count_obs(ag);

    /*
     * Stratégie V1 :
     * - si on a toutes les caméras -> prêt immédiatement
     * - sinon, on attend la fin de la fenêtre (expired)
     * - à l'expiration, on valide si c >= min_cams
     *
     * Avantage : tu collectes un maximum d'obs dans la fenêtre.
     */
    int all_cams = (c == ag->p.n_cams);
    int ready_now = all_cams || ag->expired;

    if (!ready_now) return 0;
    if (c < ag->p.min_cams) {
        /* fenêtre finie mais pas assez d'observations -> on jette cet impact */
        aggregator_reset(ag);
        return 0;
    }

    /* Construire le bundle */
    memset(out, 0, sizeof(*out));
    out->impact_id = ag->impact_id;
    out->ts_trigger_us = ag->ts_trigger_us;

    int k = 0;
    for (int i = 0; i < ag->p.n_cams; i++) {
        if (ag->has_obs[i]) {
            out->obs[k++] = ag->best_obs[i];
        }
    }
    out->obs_count = k;

    /* Consommé -> reset pour le prochain tir */
    aggregator_reset(ag);
    return 1;
}