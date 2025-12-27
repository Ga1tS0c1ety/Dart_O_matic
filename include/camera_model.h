#ifndef CAMERA_MODEL_H
#define CAMERA_MODEL_H

// Supprime cette ligne qui cause le problème
// #include <opencv2/core.hpp>

// Ajoute ces includes standards si tu en as besoin ailleurs (optionnel ici)
// Mais surtout : NE RIEN INCLURE D'OPENCV DANS LE HEADER COMMUN

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double fx, fy;
    double cx, cy;
    double s;
} IntrinsicParams;

typedef struct {
    double R[9];
    double t[3];
} ExtrinsicParams;

typedef struct {
    IntrinsicParams K;
    ExtrinsicParams RT;
} CameraModel;

void project_point(const CameraModel* cam,
                   double X, double Y, double Z,
                   double* u, double* v);

void project_point_opencv(const CameraModel* cam,
                          double X, double Y, double Z,
                          double* u, double* v);

#ifdef __cplusplus
}
#endif

#endif