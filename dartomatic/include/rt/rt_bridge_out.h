#ifndef RT_BRIDGE_OUT_H
#define RT_BRIDGE_OUT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque : client AppBus */
typedef struct AppBusClient AppBusClient;

/*
 * Initialise le bridge : connexion au broker AppBus.
 * sock_path : ex "/tmp/dart_appbus.sock" (APPBUS_DEFAULT_SOCK)
 * Retour : pointeur client AppBus, ou NULL si erreur.
 */
AppBusClient* rt_bridge_out_init(const char* sock_path);

/*
 * Publie l'event evt/impact/triangulated sur l'AppBus.
 * X_mm/Y_mm/Z_mm : en millimètres (comme tu affiches dans RT)
 * reproj_err_px  : erreur reprojection en pixels
 * cams_used      : nombre d'observations utilisées
 * cam_i/cam_j    : identifiants hardware des 2 cams de la paire gagnante (optionnel mais utile)
 *
 * Retour : 0 si OK, sinon erreur.
 */
int rt_bridge_publish_triangulated(
    AppBusClient* c,
    uint64_t impact_id,
    uint64_t ts_us,
    double X_mm, double Y_mm, double Z_mm,
    double reproj_err_px,
    int cams_used,
    int cam_i, int cam_j
);

/* Ferme le bridge */
void rt_bridge_out_close(AppBusClient* c);

#ifdef __cplusplus
}
#endif

#endif /* RT_BRIDGE_OUT_H */