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

    // Coefficients de distorsion (modèle OpenCV complet)
    double k1, k2, k3, k4, k5, k6;  // radiaux (k4,k5,k6 pour modèle rational)
    double p1, p2;                  // tangentiels
    double s1, s2, s3, s4;          // thin prism distortion
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

// Nouvelle fonction : projection avec distorsion complète (modèle OpenCV avancé)
void project_point_distorted(const CameraModel* cam,
                             double X, double Y, double Z,
                             double* u, double* v);

void project_point_opencv(const CameraModel* cam,
                          double X, double Y, double Z,
                          double* u, double* v);

// Nouvelle fonction : projection OpenCV avec distorsion complète (14 coeffs)
void project_point_opencv_distorted(const CameraModel* cam,
                                    double X, double Y, double Z,
                                    double* u, double* v);

void load_calibration_params(const char* filename, CameraModel* model);

// Dé-distorsion d'un point 2D (inverse de project_point_distorted)
// u_in,v_in : coordonnées projetées avec distorsion
// u_out,v_out : coordonnées corrigées (non distordues)
void undistort_point(const CameraModel* cam, double u_in, double v_in, double* u_out, double* v_out);

void undistort_point_opencv(const CameraModel* cam, double u_in, double v_in,
                            double* u_out, double* v_out);

#ifdef __cplusplus
}
#endif

#endif