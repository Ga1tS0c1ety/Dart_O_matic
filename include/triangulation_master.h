// triangulation_master.h
#ifndef TRIANGULATION_MASTER_H
#define TRIANGULATION_MASTER_H

#include "camera_model.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int camera_id;
    char calib_file[256];
    CameraModel cam_model;
    double last_u, last_v;
    int has_impact;
} CameraInfo;

// Initialise les caméras : charger les fichiers de calibration
void init_cameras(CameraInfo* cams, int n_cams);

// Met à jour un impact et triangule si possible
void handle_impact(CameraInfo* cams, int n_cams, int camera_id, double u, double v);

#ifdef __cplusplus
}
#endif

#endif // TRIANGULATION_MASTER_H
