#pragma once
#include <stdint.h>

/*
 * Contrats RT IPC (cam_process <-> RT middleware)
 *
 * Structures échangées entre :
 *  - rt_main
 *  - cam_process
 */

/* Types de commandes envoyées du RT vers les caméras */
typedef enum {
    RT_CAM_CMD_TRIGGER       = 1,  /* capture post-impact + détection */
    RT_CAM_CMD_SET_REFERENCE = 2   /* rafraîchir la référence de fond */
} RtCamCmdType;

/*
 * Commande générique envoyée à une caméra.
 *
 * cmd       : type de commande
 * impact_id : utilisé pour TRIGGER
 * ts_us     : timestamp associé à la commande
 */
typedef struct {
    int32_t  cmd;
    uint64_t impact_id;
    uint64_t ts_us;
} RtCamCommand;

/*
 * Observation renvoyée par une caméra après traitement.
 *
 * impact_id : doit correspondre au trigger reçu
 * cam_id    : identifiant matériel caméra
 * ts_us     : timestamp de production de l'observation
 * u, v      : coordonnées image
 * conf      : confiance [0..1]
 */
typedef struct {
    uint64_t impact_id;
    int32_t  cam_id;
    uint64_t ts_us;
    double   u;
    double   v;
    float    conf;
} RtObservationMsg;