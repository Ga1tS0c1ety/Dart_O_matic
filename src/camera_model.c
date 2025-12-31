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

// Version avec distorsion complète 
void project_point_distorted(const CameraModel* cam,
                             double X, double Y, double Z,
                             double* u, double* v)
{
    if (Z <= 0.0) {
        *u = *v = -1000.0;
        return;
    }

    // 1. Transformation extrinsèque
    double Xc = cam->RT.R[0]*X + cam->RT.R[1]*Y + cam->RT.R[2]*Z + cam->RT.t[0];
    double Yc = cam->RT.R[3]*X + cam->RT.R[4]*Y + cam->RT.R[5]*Z + cam->RT.t[1];
    double Zc = cam->RT.R[6]*X + cam->RT.R[7]*Y + cam->RT.R[8]*Z + cam->RT.t[2];

    // 2. Normalisation
    double x = Xc / Zc;
    double y = Yc / Zc;

    double r2 = x*x + y*y;
    double r4 = r2 * r2;
    double r6 = r4 * r2;

    // Numérateur radial
    double radial_num = 1.0 + cam->K.k1 * r2 + cam->K.k2 * r4 + cam->K.k3 * r6;

    // Dénominateur (modèle rational)
    double radial_den = 1.0 + cam->K.k4 * r2 + cam->K.k5 * r4 + cam->K.k6 * r6;
    if (radial_den == 0.0) radial_den = 1.0;  // évite division par zéro (rare)

    double radial_factor = radial_num / radial_den;

    // Application radiale
    double xd = x * radial_factor;
    double yd = y * radial_factor;

    // Distorsion tangentielle
    xd += 2.0 * cam->K.p1 * x * y + cam->K.p2 * (r2 + 2.0 * x*x);
    yd += cam->K.p1 * (r2 + 2.0 * y*y) + 2.0 * cam->K.p2 * x * y;

    // Thin prism distortion
    xd += cam->K.s1 * r2 + cam->K.s2 * r4;
    yd += cam->K.s3 * r2 + cam->K.s4 * r4;

    // 3. Application des intrinsèques
    *u = cam->K.fx * xd + cam->K.s * yd + cam->K.cx;
    *v = cam->K.fy * yd + cam->K.cy;
}

// Iterative undistortion using Newton-Raphson method
void undistort_point(const CameraModel* cam, double u_in, double v_in, double* u_out, double* v_out)
{
    // Coordonnées normalisées
    double x = (u_in - cam->K.cx)/cam->K.fx;
    double y = (v_in - cam->K.cy)/cam->K.fy;

    double x0 = x, y0 = y;

    // itérations pour convergence
    for(int iter=0; iter<5; iter++)
    {
        double r2 = x*x + y*y;
        double r4 = r2*r2;
        double r6 = r4*r2;

        // Radial factor (rational model)
        double radial_num = 1.0 + cam->K.k1*r2 + cam->K.k2*r4 + cam->K.k3*r6;
        double radial_den = 1.0 + cam->K.k4*r2 + cam->K.k5*r4 + cam->K.k6*r6;
        if(radial_den == 0.0) radial_den = 1.0;
        double radial = radial_num / radial_den;

        // Tangential
        double dx = 2.0*cam->K.p1*x*y + cam->K.p2*(r2 + 2.0*x*x);
        double dy = cam->K.p1*(r2 + 2.0*y*y) + 2.0*cam->K.p2*x*y;

        // Thin prism
        dx += cam->K.s1*r2 + cam->K.s2*r4;
        dy += cam->K.s3*r2 + cam->K.s4*r4;

        double x_dist = x*radial + dx;
        double y_dist = y*radial + dy;

        // Correction
        x = x0 - (x_dist - x0);
        y = y0 - (y_dist - y0);
    }

    *u_out = x;
    *v_out = y;
}