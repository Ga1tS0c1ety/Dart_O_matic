#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include "camera_model.h"

#ifdef __cplusplus
extern "C" {
#endif

// Structure pour un point 2D observé par une caméra
typedef struct {
    int camera_id;
    double u, v;
} ObservedPoint2D;

// Triangule un point 3D à partir de N observations
// points : tableau d'ObservedPoint2D
// cams   : tableau de CameraModel correspondant aux camera_id
// n      : nombre de caméras (>=2)
// X,Y,Z  : sortie 3D
int triangulate_point(const ObservedPoint2D* points,
                      const CameraModel* cams,
                      int n,
                      double* X, double* Y, double* Z);

int triangulate_point_opencv(const ObservedPoint2D* points,
                      const CameraModel* cams,
                      int n,
                      double* X, double* Y, double* Z);

void cartesian_to_dartboard_polar(double X, double Y, double Z,
                                  double *r, double *theta_deg, double *height_z);

void print_dartboard_polar(double X, double Y, double Z);

int triangulate_point_fixed_z(const ObservedPoint2D* points,
                             const CameraModel* cams,
                             int n,
                             double fixed_z,
                             double* X, double* Y, double* Z);

#ifdef __cplusplus
}
#endif

#endif
