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

void handle_impact(CameraInfo* cams, int n_cams, int camera_id, double u, double v){
    for(int i=0;i<n_cams;i++){
        if(cams[i].camera_id == camera_id){
            cams[i].last_u = u;
            cams[i].last_v = v;
            cams[i].has_impact = 1;
            break;
        }
    }

    //test 1
   // cams[0].last_u = 730;
    //cams[0].last_v = 387;
    

    //cams[1].last_u = 590;
    //cams[1].last_v = 360;

    
    int ready = 0;
    for(int i=0;i<n_cams;i++) if(cams[i].has_impact) ready++;
    if(ready<n_cams) return;

    ObservedPoint2D points[n_cams];
    CameraModel cam_models[n_cams];
    int idx=0;
    for(int i=0;i<n_cams && idx<n_cams;i++){
        if(cams[i].has_impact){
            // Dé-distorsion
            double x_corr, y_corr;
            undistort_point_opencv(&cams[i].cam_model, cams[i].last_u, cams[i].last_v, &x_corr, &y_corr);

            points[idx].u = x_corr;
            points[idx].v = y_corr;
            cam_models[idx] = cams[i].cam_model;
            idx++;
        }
    }

    double X,Y,Z;
    if(triangulate_point_opencv(points, cam_models, n_cams, &X,&Y,&Z)==0){
        printf("[TRIANG] Point 3D : X=%.2f Y=%.2f Z=%.2f\n", X,Y,Z);
        for (int i = 0; i < n_cams; i++)
        {
            double u_proj, v_proj;

            // Reprojection du point 3D vers l'image
            project_point_opencv_distorted(&cams[i].cam_model,
                                        X, Y, Z,
                                        &u_proj, &v_proj);

            // Mesure observée
            double u_obs = cams[i].last_u;
            double v_obs = cams[i].last_v;

            // Erreur de reprojection (pixels)
            double du = u_proj - u_obs;
            double dv = v_proj - v_obs;
            double err = sqrt(du*du + dv*dv);

            printf(
                "[REPROJ] Cam %d | obs=(%.1f, %.1f) proj=(%.1f, %.1f) "
                "err=%.2f px %s\n",
                cams[i].camera_id,
                u_obs, v_obs,
                u_proj, v_proj,
                err,
                (err < 3.0 ? "[OK]" :
                err < 8.0 ? "[ACCEPTABLE]" : "[MAUVAIS]")
            );
        }

        print_dartboard_polar(X, Y, Z);
}
    else
        printf("[TRIANG] Triangulation impossible (rayons parallèles)\n");

    for(int i=0;i<n_cams;i++) cams[i].has_impact = 0;
}

