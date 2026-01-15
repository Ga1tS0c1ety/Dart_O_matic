// triangulation_master.c
#include "triangulation_master.h"
#include "triangulation.h" // ta fonction triangulate_point()
#include <stdio.h>
#include <string.h>
#include <math.h>

void init_cameras(CameraInfo* cams, int n_cams) {
    for(int i=0;i<n_cams;i++){
        load_calibration_params(cams[i].calib_file, &cams[i].cam_model);
        cams[i].last_u = cams[i].last_v = 0.0;
        cams[i].has_impact = 0;
    }
}

void handle_impact(CameraInfo* cams, int n_cams,
                   int camera_id, double u, double v)
{
    /* ==================== UPDATE ÉTAT ==================== */
    for (int i = 0; i < n_cams; i++) {
        if (cams[i].camera_id == camera_id) {
            cams[i].last_u = u;
            cams[i].last_v = v;
            cams[i].has_impact = 1;

            break;
        }
    }




    /* ==================== AU MOINS 2 CAMÉRAS ==================== */
    int ready = 0;
    for (int i = 0; i < n_cams; i++)
        if (cams[i].has_impact) ready++;

    if (ready < 2)
        return;

    /* ==================== MEILLEURE SOLUTION ==================== */
    double best_err = 1e9;
    double best_X = 0, best_Y = 0, best_Z = 0;
    int best_i = -1, best_j = -1;

    /* ==================== BOUCLE SUR LES PAIRES ==================== */
    for (int i = 0; i < n_cams; i++) {
        if (!cams[i].has_impact) continue;

        for (int j = i + 1; j < n_cams; j++) {
            if (!cams[j].has_impact) continue;

            /* ======== TEST PERPENDICULARITÉ ======== */
            int ori_i = (cams[i].camera_id / 2) % 2;
            int ori_j = (cams[j].camera_id / 2) % 2;
            if (ori_i == ori_j)
                continue;

            /* ======== DÉ-DISTORSION ======== */
            ObservedPoint2D pts[2];
            CameraModel models[2];

            undistort_point_opencv(&cams[i].cam_model,
                                    cams[i].last_u, cams[i].last_v,
                                    &pts[0].u, &pts[0].v);

            undistort_point_opencv(&cams[j].cam_model,
                                    cams[j].last_u, cams[j].last_v,
                                    &pts[1].u, &pts[1].v);

            models[0] = cams[i].cam_model;
            models[1] = cams[j].cam_model;

            /* ======== TRIANGULATION 2 CAMÉRAS ======== */
            double X, Y, Z;
            if (triangulate_point_opencv(pts, models, 2,&X, &Y, &Z) != 0)
                continue;

            /* ======== ERREUR DE REPROJECTION ======== */
            double err_sum = 0.0;

            for (int k = 0; k < 2; k++) {
                double u_proj, v_proj;
                project_point_opencv_distorted(&models[k],
                                                X, Y, Z,
                                                &u_proj, &v_proj);

                double du = u_proj - cams[(k == 0 ? i : j)].last_u;
                double dv = v_proj - cams[(k == 0 ? i : j)].last_v;
                err_sum += sqrt(du*du + dv*dv);
            }

            double err_mean = err_sum / 2.0;

            /* ======== SÉLECTION MEILLEURE PAIRE ======== */
            if (err_mean < best_err) {
                best_err = err_mean;
                best_X = X;
                best_Y = Y;
                best_Z = Z;
                best_i = i;
                best_j = j;
            }
        }
    }

    /* ==================== RÉSULTAT ==================== */
    if (best_i >= 0) {
        printf(
            "[TRIANG-PAIR] Best pair: Cam %d & Cam %d | err=%.2f px\n",
            cams[best_i].camera_id,
            cams[best_j].camera_id,
            best_err
        );

        printf(
            "[TRIANG-PAIR] Point 3D : X=%.2f Y=%.2f Z=%.2f\n",
            best_X, best_Y, best_Z
        );

        print_dartboard_polar(best_X, best_Y, best_Z);
    } else {
        printf("[TRIANG-PAIR] Aucune paire valide trouvée\n");
    }

    /* ==================== RESET ÉTAT ==================== */
    for (int i = 0; i < n_cams; i++)
        cams[i].has_impact = 0;
}


