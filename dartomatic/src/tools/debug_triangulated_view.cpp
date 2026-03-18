#include "appbus/client.h"
#include "ipc/appbus_ipc.h"
#include "appbus/topics.h"

#include <opencv2/opencv.hpp>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

/*
 * debug_triangulated_view
 * -----------------------
 * Outil de debug abonné à evt/impact/triangulated.
 *
 * Rôle :
 *   - écouter les impacts triangulés publiés par le RT
 *   - afficher le point (X_mm, Y_mm, Z_mm) sur une vue top-view du plateau
 *
 * Usage :
 *   ./build/debug_triangulated_view
 *   ./build/debug_triangulated_view /tmp/dart_appbus.sock
 */

/* ========================================================= */
/* Config affichage plateau                                  */
/* ========================================================= */

#define IMG_SIZE              900
#define CENTER                (IMG_SIZE / 2)
#define SCALE                 1.5
#define DARTBOARD_RADIUS_MM   225.0
#define DOUBLE_INNER_MM       170.0
#define TRIPLE_OUTER_MM       107.0
#define TRIPLE_INNER_MM       99.0
#define BULL_OUTER_MM         31.8
#define BULL_INNER_MM         12.7

static const char* WINDOW_NAME = "TRIANGULATED_TOPVIEW";

/* ========================================================= */
/* Parsing JSON minimal                                      */
/* ========================================================= */

static int json_get_u64(const char* json, const char* key, unsigned long long* out)
{
    char pat[64];
    std::snprintf(pat, sizeof(pat), "\"%s\":", key);

    const char* p = std::strstr(json, pat);
    if (!p) return 0;
    p += std::strlen(pat);

    return (std::sscanf(p, "%llu", out) == 1);
}

static int json_get_int(const char* json, const char* key, int* out)
{
    char pat[64];
    std::snprintf(pat, sizeof(pat), "\"%s\":", key);

    const char* p = std::strstr(json, pat);
    if (!p) return 0;
    p += std::strlen(pat);

    return (std::sscanf(p, "%d", out) == 1);
}

static int json_get_double(const char* json, const char* key, double* out)
{
    char pat[64];
    std::snprintf(pat, sizeof(pat), "\"%s\":", key);

    const char* p = std::strstr(json, pat);
    if (!p) return 0;
    p += std::strlen(pat);

    return (std::sscanf(p, "%lf", out) == 1);
}

/* ========================================================= */
/* Render                                                    */
/* ========================================================= */

static void render_dartboard_topview(double X_mm,
                                     double Y_mm,
                                     double Z_mm,
                                     unsigned long long impact_id,
                                     double quality,
                                     int obs_count,
                                     int cam_a,
                                     int cam_b)
{
    cv::Mat img(IMG_SIZE, IMG_SIZE, CV_8UC3, cv::Scalar(20, 20, 20));

    /* Double extérieur */
    cv::circle(img,
               cv::Point(CENTER, CENTER),
               static_cast<int>(DARTBOARD_RADIUS_MM * SCALE),
               cv::Scalar(255, 255, 255),
               2,
               cv::LINE_AA);

    /* Bord intérieur du double */
    cv::circle(img,
               cv::Point(CENTER, CENTER),
               static_cast<int>(DOUBLE_INNER_MM * SCALE),
               cv::Scalar(180, 180, 180),
               1,
               cv::LINE_AA);

    /* Triple */
    cv::circle(img,
               cv::Point(CENTER, CENTER),
               static_cast<int>(TRIPLE_OUTER_MM * SCALE),
               cv::Scalar(220, 220, 100),
               2,
               cv::LINE_AA);

    cv::circle(img,
               cv::Point(CENTER, CENTER),
               static_cast<int>(TRIPLE_INNER_MM * SCALE),
               cv::Scalar(220, 220, 100),
               2,
               cv::LINE_AA);

    /* Bull extérieur */
    cv::circle(img,
               cv::Point(CENTER, CENTER),
               static_cast<int>(BULL_OUTER_MM * SCALE),
               cv::Scalar(40, 40, 220),
               2,
               cv::LINE_AA);

    /* Bull intérieur */
    cv::circle(img,
               cv::Point(CENTER, CENTER),
               static_cast<int>(BULL_INNER_MM * SCALE),
               cv::Scalar(0, 120, 255),
               -1,
               cv::LINE_AA);

    /* Centre */
    cv::circle(img,
               cv::Point(CENTER, CENTER),
               4,
               cv::Scalar(0, 255, 0),
               -1,
               cv::LINE_AA);

    /* Monde -> image
     * X vers la droite
     * Y vers le haut
     */
    int px = CENTER + static_cast<int>(X_mm * SCALE);
    int py = CENTER - static_cast<int>(Y_mm * SCALE);

    /* Point impact */
    cv::circle(img,
               cv::Point(px, py),
               6,
               cv::Scalar(0, 0, 255),
               -1,
               cv::LINE_AA);

    cv::circle(img,
               cv::Point(px, py),
               10,
               cv::Scalar(220, 220, 255),
               2,
               cv::LINE_AA);

    /* Texte debug */
    char txt[128];

    std::snprintf(txt, sizeof(txt), "impact_id = %llu", impact_id);
    cv::putText(img, txt, cv::Point(15, 35),
                cv::FONT_HERSHEY_SIMPLEX, 0.65,
                cv::Scalar(200, 200, 200), 2);

    std::snprintf(txt, sizeof(txt), "X = %.1f mm", X_mm);
    cv::putText(img, txt, cv::Point(15, 65),
                cv::FONT_HERSHEY_SIMPLEX, 0.65,
                cv::Scalar(200, 200, 200), 2);

    std::snprintf(txt, sizeof(txt), "Y = %.1f mm", Y_mm);
    cv::putText(img, txt, cv::Point(15, 95),
                cv::FONT_HERSHEY_SIMPLEX, 0.65,
                cv::Scalar(200, 200, 200), 2);

    std::snprintf(txt, sizeof(txt), "Z = %.1f mm", Z_mm);
    cv::putText(img, txt, cv::Point(15, 125),
                cv::FONT_HERSHEY_SIMPLEX, 0.65,
                cv::Scalar(200, 200, 200), 2);

    double r = std::hypot(X_mm, Y_mm);
    std::snprintf(txt, sizeof(txt), "r = %.1f mm", r);
    cv::putText(img, txt, cv::Point(15, 155),
                cv::FONT_HERSHEY_SIMPLEX, 0.65,
                cv::Scalar(200, 200, 200), 2);

    std::snprintf(txt, sizeof(txt), "reproj_err = %.3f px", quality);
    cv::putText(img, txt, cv::Point(15, 185),
                cv::FONT_HERSHEY_SIMPLEX, 0.65,
                cv::Scalar(200, 200, 200), 2);

    std::snprintf(txt, sizeof(txt), "obs=%d pair=(%d,%d)", obs_count, cam_a, cam_b);
    cv::putText(img, txt, cv::Point(15, 215),
                cv::FONT_HERSHEY_SIMPLEX, 0.65,
                cv::Scalar(200, 200, 200), 2);

    if (r > DARTBOARD_RADIUS_MM) {
        cv::putText(img, "HORS PLATEAU", cv::Point(15, 255),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(0, 0, 255), 2);
    }

    cv::imshow(WINDOW_NAME, img);
    cv::waitKey(1);
}

/* ========================================================= */
/* Context                                                   */
/* ========================================================= */

typedef struct {
    AppBusClient* bus;
} ViewCtx;

/* ========================================================= */
/* Callback AppBus                                           */
/* ========================================================= */

static void on_bus_msg(const char* topic,
                       const char* payload,
                       size_t payload_len,
                       void* user)
{
    (void)payload_len;
    (void)user;

    if (std::strcmp(topic, TOPIC_EVT_IMPACT_TRIANG) != 0) return;
    if (!payload) return;

    unsigned long long impact_id = 0;
    unsigned long long ts_us = 0;
    double x_mm = 0.0;
    double y_mm = 0.0;
    double z_mm = 0.0;
    double reproj_err_px = 0.0;
    int obs_count = 0;
    int cam_a = -1;
    int cam_b = -1;

    int ok = 1;
    ok &= json_get_u64(payload, "impact_id", &impact_id);
    ok &= json_get_double(payload, "x_mm", &x_mm);
    ok &= json_get_double(payload, "y_mm", &y_mm);
    ok &= json_get_double(payload, "z_mm", &z_mm);

    json_get_u64(payload, "ts_us", &ts_us);
    json_get_double(payload, "reproj_err_px", &reproj_err_px);
    json_get_int(payload, "obs_count", &obs_count);
    json_get_int(payload, "cam_a", &cam_a);
    json_get_int(payload, "cam_b", &cam_b);

    if (!ok) {
        std::fprintf(stderr, "[TRIANG_VIEW] payload invalide: %s\n", payload);
        return;
    }

    std::printf("[TRIANG_VIEW] impact_id=%llu ts=%llu x=%.1f y=%.1f z=%.1f err=%.2f obs=%d pair=(%d,%d)\n",
                impact_id, ts_us, x_mm, y_mm, z_mm, reproj_err_px, obs_count, cam_a, cam_b);
    std::fflush(stdout);

    render_dartboard_topview(x_mm, y_mm, z_mm,
                             impact_id,
                             reproj_err_px,
                             obs_count,
                             cam_a, cam_b);
}

/* ========================================================= */
/* main                                                      */
/* ========================================================= */

int main(int argc, char** argv)
{
    const char* sock = (argc >= 2) ? argv[1] : APPBUS_DEFAULT_SOCK;

    std::printf("[TRIANG_VIEW] démarrage, sock=%s\n", sock);
    std::fflush(stdout);

    cv::namedWindow(WINDOW_NAME, cv::WINDOW_AUTOSIZE);

    AppBusClient* bus = appbus_connect(sock);
    if (!bus) {
        std::fprintf(stderr, "[TRIANG_VIEW] impossible de se connecter au bus\n");
        cv::destroyWindow(WINDOW_NAME);
        return 2;
    }

    if (appbus_subscribe(bus, TOPIC_EVT_IMPACT_TRIANG) != 0) {
        std::fprintf(stderr, "[TRIANG_VIEW] subscribe échoué sur %s\n", TOPIC_EVT_IMPACT_TRIANG);
        appbus_close(bus);
        cv::destroyWindow(WINDOW_NAME);
        return 3;
    }

    ViewCtx ctx;
    std::memset(&ctx, 0, sizeof(ctx));
    ctx.bus = bus;

    std::printf("[TRIANG_VIEW] écoute de %s...\n", TOPIC_EVT_IMPACT_TRIANG);
    std::printf("[TRIANG_VIEW] ferme la fenêtre ou Ctrl+C pour quitter.\n");
    std::fflush(stdout);

    int rc = appbus_poll(bus, on_bus_msg, &ctx);

    appbus_close(bus);
    cv::destroyWindow(WINDOW_NAME);

    return (rc == 0) ? 0 : 4;
}