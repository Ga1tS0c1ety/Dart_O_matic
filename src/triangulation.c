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

#ifdef __cplusplus
}
#endif
