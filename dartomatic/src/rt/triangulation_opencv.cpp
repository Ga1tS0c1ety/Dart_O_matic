/*
 * triangulation_opencv.cpp
 * ------------------------
 * Implémentation OpenCV de la triangulation RT.
 *
 * Important :
 *  - L'API est en C (extern "C") pour être appelée depuis rt_main.c
 *  - On utilise ton backend existant :
 *      triangulate_point_opencv()
 *      undistort_point_opencv()
 *      project_point_opencv_distorted()
 *      load_calibration_params()
 *      load_extrinsics_yaml()
 */

#include "rt/triangulation.h"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <cfloat>

/* Petit helper clamp */
static inline double dclamp(double x, double a, double b) {
    return (x < a) ? a : (x > b) ? b : x;
}

/*
 * Logique "orientation" reprise de ton code historique.
 * Pour tes IDs {0,2,4,6} : (id/2)%2 donne 0,1,0,1 -> alternance.
 * => permet d'éviter les paires (gauche+droit) ou (haut+bas) si colinéaires.
 */
static inline int cam_orientation(int cam_id) {
    return (cam_id / 2) % 2;
}

extern "C" int triangulation_load_cameras(CameraModel* cams,
                                          const int* cam_ids,
                                          int n_cams,
                                          const char* intr_pattern,
                                          const char* extr_pattern)
{
    if (!cams || !cam_ids || n_cams <= 0 || !intr_pattern || !extr_pattern)
        return -1;

    for (int i = 0; i < n_cams; i++) {
        char intr[256];
        char extr[256];

        std::snprintf(intr, sizeof(intr), intr_pattern, cam_ids[i]);
        std::snprintf(extr, sizeof(extr), extr_pattern, cam_ids[i]);

        /* 1) Intrinsics */
        load_calibration_params(intr, &cams[i]);

        /* 2) Extrinsics */
        if (load_extrinsics_yaml(extr, &cams[i]) != 0) {
            std::fprintf(stderr, "[TRIANG] ERREUR: extrinsics non chargées pour cam_id=%d (%s)\n",
                         cam_ids[i], extr);
            return -2;
        }

        std::printf("[TRIANG] Cam %d chargée (intr=%s extr=%s)\n",
                    cam_ids[i], intr, extr);
    }

    return 0;
}

/*
 * Calcule l'erreur reprojection moyenne (pixels) sur 2 caméras
 * en reprojetant le point 3D (X,Y,Z) et comparant aux points observés (distordus).
 */
static double reproj_error_pair(const CameraModel* c0, int cam_id0, double u0, double v0,
                                const CameraModel* c1, int cam_id1, double u1, double v1,
                                double X, double Y, double Z)
{
    (void)cam_id0; (void)cam_id1; /* pas utilisé, mais utile si tu veux log */

    double u_proj, v_proj;
    double e0 = 0.0, e1 = 0.0;

    project_point_opencv_distorted(c0, X, Y, Z, &u_proj, &v_proj);
    e0 = std::hypot(u_proj - u0, v_proj - v0);

    project_point_opencv_distorted(c1, X, Y, Z, &u_proj, &v_proj);
    e1 = std::hypot(u_proj - u1, v_proj - v1);

    return 0.5 * (e0 + e1);
}

extern "C" int triangulation_from_bundle(const ImpactBundle* b,
                                         const CameraModel* cams,
                                         const int* cam_ids,
                                         int n_cams,
                                         TriangulationResult* out)
{
    if (!b || !cams || !cam_ids || !out || n_cams <= 0)
        return -1;

    if (b->obs_count < 2)
        return -2;

    double best_err = DBL_MAX;
    double best_X = 0, best_Y = 0, best_Z = 0;
    int best_i = -1, best_j = -1;

    /*
     * V1 : boucle sur toutes les paires d'observations disponibles.
     * On utilise cam_index (index dans cam_ids[]) pour accéder au modèle correspondant.
     */
    for (int a = 0; a < b->obs_count; a++) {
        int i = b->obs[a].cam_index;
        if (i < 0 || i >= n_cams) continue;

        int cam_id_i = cam_ids[i];
        int ori_i = cam_orientation(cam_id_i);

        for (int bb = a + 1; bb < b->obs_count; bb++) {
            int j = b->obs[bb].cam_index;
            if (j < 0 || j >= n_cams) continue;

            int cam_id_j = cam_ids[j];
            int ori_j = cam_orientation(cam_id_j);

            /* On garde seulement des paires perpendiculaires (croix) */
            if (ori_i == ori_j)
                continue;

            /* Points observés (distordus) */
            double u_i = b->obs[a].u;
            double v_i = b->obs[a].v;
            double u_j = b->obs[bb].u;
            double v_j = b->obs[bb].v;

            /* Triangulation opencv attend des points "observed", elle undistort déjà en interne chez toi,
               mais ton ancien handle_impact faisait aussi undistort avant.
               Ici on reste proche de ton code "handle_impact" : on undistort d'abord. */
            ObservedPoint2D pts[2];
            CameraModel pair[2];

            pts[0].u = u_i;
pts[0].v = v_i;
pts[1].u = u_j;
pts[1].v = v_j;

            pair[0] = cams[i];
            pair[1] = cams[j];

            double X, Y, Z;
            if (triangulate_point_opencv(pts, pair, 2, &X, &Y, &Z) != 0)
                continue;

            /* Erreur reprojection sur pixels distordus (comme ton code historique) */
            double err = reproj_error_pair(&cams[i], cam_id_i, u_i, v_i,
                                           &cams[j], cam_id_j, u_j, v_j,
                                           X, Y, Z);

            /* Option (V1.1) : pénaliser une paire si conf faible.
               Ici on garde simple mais on peut faire :
                  err /= (0.5*(conf_i+conf_j)+eps)
             */
            double conf_i = dclamp((double)b->obs[a].conf, 0.0, 1.0);
            double conf_j = dclamp((double)b->obs[bb].conf, 0.0, 1.0);
            double conf_mean = 0.5 * (conf_i + conf_j);
            if (conf_mean > 1e-6) {
                /* petit bonus aux paires très confiantes */
                err = err / (0.5 + conf_mean); /* borne, évite division trop agressive */
            }

            if (err < best_err) {
                best_err = err;
                best_X = X;
                best_Y = Y;
                best_Z = Z;
                best_i = i;
                best_j = j;
            }
        }
    }

    if (best_i < 0)
        return -3;

    out->X = best_X;
    out->Y = best_Y;
    out->Z = best_Z;
    out->reproj_err_px = best_err;
    out->used_cam_count = 2;
    out->cam_i = best_i;
    out->cam_j = best_j;

    return 0;
}