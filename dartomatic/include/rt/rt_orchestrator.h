#pragma once
#include <stdint.h>
#include "rt/aggregator.h"
#include "ipc/rt_ipc.h"

/*
 * RT Orchestrator
 * ---------------
 * Rôle :
 *  - recevoir un événement "impact" (MPU6050 ou autre)
 *  - appliquer gating: cooldown + anti-rebond simple (ne pas retrigger en ARMED)
 *  - générer un impact_id
 *  - "armer" l'aggregator (ouvrir une fenêtre de collecte)
 *  - recevoir les observations caméras et les pousser à l'aggregator
 *  - décider quand un bundle est prêt -> transition vers COOLDOWN
 *
 * Ce module NE fait PAS :
 *  - triangulation
 *  - publication AppBus
 * Il prépare seulement des ImpactBundle fiables.
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ORCH_IDLE = 0,
    ORCH_ARMED = 1,
    ORCH_COOLDOWN = 2
} OrchState;

typedef struct {
    /* Caméras attendues */
    int n_cams;        /* ex 4 */
    int cam_ids[8];    /* ex {0,2,4,6} ; taille max 8 */

    /* Agrégation */
    int min_cams;      /* ex 2 */
    int window_ms;     /* ex 150 */

    /* Gating */
    int cooldown_ms;   /* ex 250 */
} RtOrchestratorParams;

typedef struct RtOrchestrator RtOrchestrator;

/* init/reset */
int  rt_orch_init(RtOrchestrator* o, RtOrchestratorParams p);
void rt_orch_reset(RtOrchestrator* o);

/* état */
OrchState rt_orch_state(const RtOrchestrator* o);

/*
 * Entrée MPU (ou simulateur) :
 * - appelle quand tu détectes un impact.
 * - now_us : timestamp en microsecondes
 *
 * Retour :
 *  1 si un nouveau trigger a été accepté (passage IDLE->ARMED)
 *  0 si ignoré (ARMED ou COOLDOWN)
 */
int rt_orch_on_mpu_impact(RtOrchestrator* o, uint64_t now_us);

/*
 * Entrée observation caméra :
 * - msg->cam_id peut être un id hardware (0,2,4,6)
 * - le mapping cam_id->cam_index est fait ici.
 */
void rt_orch_on_observation(RtOrchestrator* o, const RtObservationMsg* msg);

/*
 * Tick :
 * - à appeler régulièrement (ex toutes 1-2ms)
 * - gère expiration fenêtre aggregator
 * - gère fin cooldown
 */
void rt_orch_tick(RtOrchestrator* o, uint64_t now_us);

/*
 * Récupérer un bundle prêt.
 * Retour :
 *  1 si bundle prêt (écrit dans out)
 *  0 sinon
 *
 * Un bundle prêt signifie :
 *  - triangulation possible (>= min_cams)
 *  - orchestrateur passe en COOLDOWN
 */
int rt_orch_poll_bundle(RtOrchestrator* o, ImpactBundle* out);

/* pour debug */
uint64_t rt_orch_current_impact_id(const RtOrchestrator* o);

#ifdef __cplusplus
}
#endif

/* Impl interne exposée pour éviter malloc (C pur) */
struct RtOrchestrator {
    RtOrchestratorParams p;

    OrchState state;

    uint64_t next_impact_id;
    uint64_t current_impact_id;

    uint64_t cooldown_until_us;

    Aggregator ag; /* aggregator intégré (composition) */
};