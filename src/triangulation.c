#include "triangulation.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

#ifdef __cplusplus
extern "C" {
#endif

// ------------------- Utilitaires -------------------
static void svd_4x4(const double A[8][4], int nrows, double X[4]) {
    // Méthode simple : on calcule ATA et on résout valeur propre minimale
    // ATA = A^T * A  (4x4)
    double ATA[4][4] = {{0}};
    for(int i=0;i<nrows;i++){
        for(int j=0;j<4;j++){
            for(int k=0;k<4;k++){
                ATA[j][k] += A[i][j]*A[i][k];
            }
        }
    }

    // Eigen decomposition simplifiée : on prend approximation via puissance inverse
    // Ici petite taille, on peut itérer pour vecteur propre minimal
    double vec[4] = {1,1,1,1};
    double tmp[4];
    for(int iter=0;iter<1000;iter++){
        // y = ATA * x
        for(int i=0;i<4;i++){
            tmp[i]=0;
            for(int j=0;j<4;j++) tmp[i] += ATA[i][j]*vec[j];
        }
        // normalisation
        double norm = sqrt(tmp[0]*tmp[0]+tmp[1]*tmp[1]+tmp[2]*tmp[2]+tmp[3]*tmp[3]);
        for(int i=0;i<4;i++) vec[i] = tmp[i]/(norm+DBL_EPSILON);
    }
    for(int i=0;i<4;i++) X[i] = vec[i];
}

// ------------------- Triangulation N caméras -------------------
int triangulate_point(const ObservedPoint2D* points,
                        const CameraModel* cams,
                        int n,
                        double* X, double* Y, double* Z)
{
    if(n<2 || !points || !cams || !X || !Y || !Z) return -1;

    int nrows = 2*n;
    if(nrows>8) nrows=8; // pour SVD 4x4 simplifié, max 4 caméras, sinon SVD complète nécessaire

    double A[8][4];  // support pour max 4 caméras (8 lignes)
    memset(A,0,sizeof(A));

    for(int i=0;i<n;i++){
        const CameraModel* cam = &cams[i];
        double u = points[i].u;
        double v = points[i].v;

        // coord. normalisée
        double x = (u - cam->K.cx)/cam->K.fx;
        double y = (v - cam->K.cy)/cam->K.fy;

        // projection matrix 3x4
        double P[3][4];
        for(int r=0;r<3;r++){
            for(int c=0;c<3;c++) P[r][c] = cam->RT.R[r*3+c];
            P[r][3] = cam->RT.t[r];
        }

        // ligne 2*i : u*P[2,:]-P[0,:]
        for(int j=0;j<4;j++) A[2*i][j] = x*P[2][j] - P[0][j];
        // ligne 2*i+1 : v*P[2,:]-P[1,:]
        for(int j=0;j<4;j++) A[2*i+1][j] = y*P[2][j] - P[1][j];
    }

    double Xh[4];
    svd_4x4(A,nrows,Xh);

    *X = Xh[0]/Xh[3];
    *Y = Xh[1]/Xh[3];
    *Z = Xh[2]/Xh[3];

    return 0;
}


// ------------------- Nouvelle fonction : triangulation avec Z fixé (n=2 uniquement) -------------------
int triangulate_point_fixed_z(const ObservedPoint2D* points,
                             const CameraModel* cams,
                             int n,
                             double fixed_z,
                             double* X_out, double* Y_out, double* Z_out)
{
    if (n != 2 || !points || !cams || !X_out || !Y_out || !Z_out) return -1;

    double A[4][2] = {{0.0}};   // 4 équations × 2 inconnues (X,Y)
    double b[4]    = {0.0};

    int eq = 0;

    for (int i = 0; i < 2; i++)
    {
        const CameraModel* cam = &cams[i];
        double u = points[i].u;
        double v = points[i].v;

        // On construit la matrice de projection P = K * [R|t]
        double P[3][4] = {{0.0}};

        // Remplissage de P
        for (int r = 0; r < 3; r++)
        {
            for (int c = 0; c < 3; c++)
            {
                P[r][c] = 0.0;
                for (int k = 0; k < 3; k++)
                {
                    // K * R
                    double K_row[3] = {0.0};
                    if (r == 0) K_row[0] = cam->K.fx, K_row[1] = 0.0,          K_row[2] = cam->K.cx;
                    if (r == 1) K_row[0] = 0.0,       K_row[1] = cam->K.fy,     K_row[2] = cam->K.cy;
                    if (r == 2) K_row[0] = 0.0,       K_row[1] = 0.0,           K_row[2] = 1.0;

                    P[r][c] += K_row[k] * cam->RT.R[k*3 + c];
                }
            }

            // K * t pour la colonne de translation
            P[r][3] = 0.0;
            for (int k = 0; k < 3; k++)
            {
                double K_row[3] = {0.0};
                if (r == 0) K_row[0] = cam->K.fx, K_row[1] = 0.0,          K_row[2] = cam->K.cx;
                if (r == 1) K_row[0] = 0.0,       K_row[1] = cam->K.fy,     K_row[2] = cam->K.cy;
                if (r == 2) K_row[0] = 0.0,       K_row[1] = 0.0,           K_row[2] = 1.0;

                P[r][3] += K_row[k] * cam->RT.t[k];
            }
        }

        // Maintenant on construit les équations (comme dans la triangulation classique, mais avec Z fixé)
        // eq1:  u * (p31 X + p32 Y + p33 Z + p34)  =  p11 X + p12 Y + p13 Z + p14
        A[eq][0] = P[0][0] - u * P[2][0];
        A[eq][1] = P[0][1] - u * P[2][1];
        b[eq]    = u * (P[2][2] * fixed_z + P[2][3]) - (P[0][2] * fixed_z + P[0][3]);
        eq++;

        // eq2:  v * (p31 X + p32 Y + p33 Z + p34)  =  p21 X + p22 Y + p23 Z + p24
        A[eq][0] = P[1][0] - v * P[2][0];
        A[eq][1] = P[1][1] - v * P[2][1];
        b[eq]    = v * (P[2][2] * fixed_z + P[2][3]) - (P[1][2] * fixed_z + P[1][3]);
        eq++;
    }

    // Résolution moindre carrés 4×2 → système normal 2×2
    double ATA[2][2] = {{0.0}};
    double ATb[2]    = {0.0};

    for (int i = 0; i < 4; i++)
    {
        ATA[0][0] += A[i][0] * A[i][0];
        ATA[0][1] += A[i][0] * A[i][1];
        ATA[1][0] += A[i][1] * A[i][0];
        ATA[1][1] += A[i][1] * A[i][1];

        ATb[0] += A[i][0] * b[i];
        ATb[1] += A[i][1] * b[i];
    }

    double det = ATA[0][0] * ATA[1][1] - ATA[0][1] * ATA[1][0];
    if (fabs(det) < 1e-6) return -2; // mal conditionné

    double inv_det = 1.0 / det;

    *X_out = ( ATA[1][1] * ATb[0] - ATA[0][1] * ATb[1] ) * inv_det;
    *Y_out = ( ATA[0][0] * ATb[1] - ATA[1][0] * ATb[0] ) * inv_det;
    *Z_out = fixed_z;

    return 0;
}

// Fonction pour convertir cartésien → polaire (pour cible de fléchettes)
void cartesian_to_dartboard_polar(double X, double Y, double Z,
                                  double *r, double *theta_deg, double *height_z)
{
    // Distance radiale dans le plan XY
    *r = sqrt(X * X + Y * Y);

    // Angle en radians, puis conversion en degrés
    // atan2(Y, X) : Y en premier pour convention mathématique standard
    double theta_rad = atan2(Y, X);
    *theta_deg = theta_rad * 180.0 / M_PI;

    // Normalisation optionnelle entre 0 et 360° (au lieu de -180 à +180)
    if (*theta_deg < 0.0) {
        *theta_deg += 360.0;
    }

    // Hauteur perpendiculaire (Z reste tel quel)
    *height_z = Z;
}

// Version avec affichage direct (pratique pour tests)
void print_dartboard_polar(double X, double Y, double Z)
{
    double r, theta_deg, height_z;
    cartesian_to_dartboard_polar(X, Y, Z, &r, &theta_deg, &height_z);

    printf("Point 3D : (X=%.2f, Y=%.2f, Z=%.2f) mm\n", X, Y, Z);
    printf("→ Polaire :\n");
    printf("   Distance au centre (r) : %.2f mm\n", r);
    printf("   Angle (θ)             : %.2f ° (0° = +X, sens anti-horaire)\n", theta_deg);
    printf("   Hauteur (Z)           : %.2f mm\n\n", height_z);
}




#ifdef __cplusplus
}
#endif
