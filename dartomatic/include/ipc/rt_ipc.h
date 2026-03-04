// dartomatic/include/ipc/rt_ipc.h
#pragma once
#include <stdint.h>

/*
 * Contrats RT IPC (cam_process <-> RT middleware)
 *
 * Ici on définit uniquement les structures nécessaires au RT pipeline.
 * Ces structs peuvent être utilisés :
 *  - en IPC (Unix socket DGRAM/STREAM)
 *  - en appels internes (tests / simulateurs)
 */

/* Commande envoyée par l'orchestrateur aux caméras */
typedef struct {
    uint64_t impact_id; /* identifiant unique du tir */
    uint64_t ts_us;     /* timestamp microsecondes (moment du trigger) */
} RtTriggerCmd;

/* Observation renvoyée par une caméra après traitement */
typedef struct {
    uint64_t impact_id; /* doit correspondre au trigger reçu */
    int32_t  cam_id;    /* id caméra (ex: 0,2,4,6 ou index 0..3 selon ta convention) */
    uint64_t ts_us;     /* timestamp microsecondes (moment où l'observation est produite) */
    double   u;         /* coordonnée image (pixels ou normalisée) */
    double   v;         /* coordonnée image */
    float    conf;      /* confiance [0..1] */
} RtObservationMsg;