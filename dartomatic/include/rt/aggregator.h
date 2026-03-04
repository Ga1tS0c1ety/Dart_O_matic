#pragma once
#include <stdint.h>
#include <stddef.h>
#include "ipc/rt_ipc.h"

/*
 * Aggregator
 * ----------
 * Rôle :
 *  - recevoir un "trigger" (impact_id + ts)
 *  - recevoir des observations caméras (u,v,conf) corrélées par impact_id
 *  - attendre une fenêtre temporelle
 *  - sortir un bundle prêt pour la triangulation :
 *      ImpactBundle = impact_id + liste obs (min_cams)
 *
 * Politique V1 :
 *  - un seul impact actif à la fois (simple)
 *  - window_ms = 150ms
 *  - min_cams = 2
 *  - 1 obs par caméra (on garde la meilleure confidence)
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int n_cams;          /* nombre de caméras attendues (ex: 4) */
    int min_cams;        /* minimum d'observations pour valider (ex: 2) */
    int window_ms;       /* fenêtre d'attente (ex: 150ms) */
} AggregatorParams;

/* Une observation "nettoyée" côté aggregator */
typedef struct {
    int    cam_index;    /* index 0..n_cams-1 (voir mapping cam_id->index) */
    double u;
    double v;
    float  conf;
    uint64_t ts_us;
} AggObservation;

/* Bundle prêt pour triangulation */
typedef struct {
    uint64_t impact_id;
    uint64_t ts_trigger_us;
    int obs_count;
    AggObservation obs[8]; /* assez pour 4 cams; garde un peu de marge */
} ImpactBundle;

typedef struct Aggregator Aggregator;

/* Création / destruction */
int  aggregator_init(Aggregator* ag, AggregatorParams p);
void aggregator_reset(Aggregator* ag);

/* Démarre un nouvel impact (arm) */
void aggregator_on_trigger(Aggregator* ag, uint64_t impact_id, uint64_t ts_us);

/*
 * Injection d'une observation caméra
 * - cam_index : 0..n_cams-1 (c'est volontairement indépendant de cam_id)
 * - impact_id doit matcher l'impact courant, sinon ignoré.
 */
void aggregator_on_observation(Aggregator* ag, int cam_index, const RtObservationMsg* msg);

/*
 * Tick temps (appeler régulièrement, ex toutes 1-2ms)
 * - now_us : timestamp courant
 * Cette fonction met à jour l'état "expired" quand la fenêtre est passée.
 */
void aggregator_tick(Aggregator* ag, uint64_t now_us);

/*
 * Récupère un bundle prêt si disponible.
 * Retour :
 *  1 si bundle écrit dans out
 *  0 sinon
 *
 * IMPORTANT :
 *  - Une fois renvoyé, l'aggregator repasse en "idle" (prêt pour prochain tir).
 */
int aggregator_poll_ready(Aggregator* ag, ImpactBundle* out);

/* Debug : savoir si un impact est en cours */
int aggregator_is_active(const Aggregator* ag);

#ifdef __cplusplus
}
#endif

/* La struct interne est exposée ici uniquement pour éviter malloc (C pur). */
struct Aggregator {
    AggregatorParams p;

    int active;              /* 1 si un impact est en cours */
    uint64_t impact_id;
    uint64_t ts_trigger_us;
    uint64_t deadline_us;    /* ts_trigger + window */

    int expired;             /* 1 si on a dépassé deadline */

    /* stockage par caméra */
    int has_obs[8];          /* n_cams max 8 ici */
    AggObservation best_obs[8];
};