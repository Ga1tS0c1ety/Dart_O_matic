#include "rt/rt_bridge_out.h"

#include "appbus/client.h"
#include "appbus/topics.h"
#include "ipc/appbus_ipc.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * Bridge RT -> AppBus
 * Publie evt/impact/triangulated avec des clés JSON cohérentes avec scoring_service :
 *   impact_id, ts_us, x_mm, y_mm, z_mm, quality
 */

AppBusClient* rt_bridge_out_init(const char* sock_path)
{
    if (!sock_path) sock_path = APPBUS_DEFAULT_SOCK;

    AppBusClient* c = appbus_connect(sock_path);
    if (!c) {
        fprintf(stderr, "[RT->AppBus] ERREUR: connexion AppBus impossible (%s)\n", sock_path);
        return NULL;
    }

    printf("[RT->AppBus] connecté à %s\n", sock_path);
    return c;
}

static double quality_from_reproj(double reproj_err_px)
{
    /*
     * Heuristique V1 simple :
     * - plus l'erreur reprojection est faible, plus quality est proche de 1
     * - quality = 1 / (1 + err)
     */
    if (reproj_err_px < 0.0) reproj_err_px = 0.0;
    return 1.0 / (1.0 + reproj_err_px);
}

int rt_bridge_publish_triangulated(
    AppBusClient* c,
    uint64_t impact_id,
    uint64_t ts_us,
    double x_mm, double y_mm, double z_mm,
    double reproj_err_px,
    int cams_used,
    int cam_i, int cam_j
) {
    (void)cams_used;
    (void)cam_i;
    (void)cam_j;

    if (!c) return -1;

    char payload[512];
    int n = snprintf(payload, sizeof(payload),
        "{"
          "\"impact_id\":%llu,"
          "\"ts_us\":%llu,"
          "\"x_mm\":%.3f,"
          "\"y_mm\":%.3f,"
          "\"z_mm\":%.3f,"
          "\"quality\":%.3f"
        "}",
        (unsigned long long)impact_id,
        (unsigned long long)ts_us,
        x_mm, y_mm, z_mm,
        quality_from_reproj(reproj_err_px)
    );

    if (n <= 0 || (size_t)n >= sizeof(payload)) {
        fprintf(stderr, "[RT->AppBus] ERREUR: payload trop long\n");
        return -2;
    }

    /* IMPORTANT: on utilise le topic défini chez toi */
    const char* topic = TOPIC_EVT_IMPACT_TRIANG;

    int rc = appbus_publish(c, topic, payload);
    if (rc != 0) {
        fprintf(stderr, "[RT->AppBus] publish FAIL topic=%s\n", topic);
        return -3;
    }

    printf("[RT->AppBus] PUB %s %s\n", topic, payload);
    return 0;
}

void rt_bridge_out_close(AppBusClient* c)
{
    if (c) appbus_close(c);
}