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
    ag->expired = 0;

    printf("[AGGR] trigger impact_id=%llu deadline=%llu\n",
           (unsigned long long)ag->impact_id,
           (unsigned long long)ag->deadline_us);
}

void aggregator_on_observation(Aggregator* ag, int cam_index, const RtObservationMsg* msg) {
    if (!ag || !msg) return;

    if (!ag->active) {
        printf("[AGGR] obs ignorée cam=%d impact_id=%llu : aggregator inactif\n",
               msg->cam_id, (unsigned long long)msg->impact_id);
        return;
    }

    /* On ne prend que les obs pour l'impact courant */
    if (msg->impact_id != ag->impact_id) {
        printf("[AGGR] obs ignorée cam=%d impact_id=%llu : impact courant=%llu\n",
               msg->cam_id,
               (unsigned long long)msg->impact_id,
               (unsigned long long)ag->impact_id);
        return;
    }

    if (cam_index < 0 || cam_index >= ag->p.n_cams) {
        printf("[AGGR] obs ignorée cam=%d impact_id=%llu : cam_index invalide\n",
               msg->cam_id, (unsigned long long)msg->impact_id);
        return;
    }

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

        printf("[AGGR] obs acceptée impact_id=%llu cam_index=%d cam_id=%d conf=%.2f\n",
               (unsigned long long)ag->impact_id,
               cam_index,
               msg->cam_id,
               msg->conf);
    } else {
        printf("[AGGR] obs rejetée impact_id=%llu cam_index=%d cam_id=%d conf=%.2f < best=%.2f\n",
               (unsigned long long)ag->impact_id,
               cam_index,
               msg->cam_id,
               msg->conf,
               ag->best_obs[cam_index].conf);
    }
}

void aggregator_tick(Aggregator* ag, uint64_t now_us) {
    if (!ag) return;
    if (!ag->active) return;

    if (!ag->expired && now_us >= ag->deadline_us) {
        ag->expired = 1;
        printf("[AGGR] impact_id=%llu fenêtre expirée\n",
               (unsigned long long)ag->impact_id);
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
     * Nouvelle stratégie V1.1 :
     * - si on a au moins min_cams -> prêt immédiatement
     * - sinon on attend la fin de la fenêtre
     * - à l'expiration, si toujours pas assez d'obs -> rejet
     *
     * Avantage :
     * - beaucoup moins de latence
     * - évite d'attendre inutilement 800 ms quand 2 cams suffisent
     */
    int enough_cams = (c >= ag->p.min_cams);
    int ready_now = enough_cams || ag->expired;

    if (!ready_now) return 0;

    if (c < ag->p.min_cams) {
        printf("[AGGR] impact_id=%llu rejeté : obs=%d < min=%d\n",
               (unsigned long long)ag->impact_id,
               c,
               ag->p.min_cams);
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

    printf("[AGGR] impact_id=%llu prêt : obs=%d\n",
           (unsigned long long)out->impact_id,
           out->obs_count);

    /* Consommé -> reset pour le prochain tir */
    aggregator_reset(ag);
    return 1;
}