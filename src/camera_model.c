// src/camera_model.c

#include "camera_model.h"

void project_point(const CameraModel* cam,
                   const double X, const double Y, const double Z,
                   double* u, double* v)
{
    // 1. Transformation extrinsèque
    double Xc = cam->RT.R[0]*X + cam->RT.R[1]*Y + cam->RT.R[2]*Z + cam->RT.t[0];
    double Yc = cam->RT.R[3]*X + cam->RT.R[4]*Y + cam->RT.R[5]*Z + cam->RT.t[1];
    double Zc = cam->RT.R[6]*X + cam->RT.R[7]*Y + cam->RT.R[8]*Z + cam->RT.t[2];

    // 2. Normalisation
    double x = Xc / Zc;
    double y = Yc / Zc;

    // 3. Application des paramètres intrinsèques
    *u = cam->K.fx * x + cam->K.s * y + cam->K.cx;
    *v = cam->K.fy * y + cam->K.cy;
}
