#include "triangulation.h"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <iostream>

#define MAX_ITERS 50
#define EPS_JAC   1e-6
#define LAMBDA_INIT 1e-3

/* ========================================================= */
/* Erreur de reprojection                                    */
/* ========================================================= */
static void reprojection_error(const ObservedPoint2D* points,
                               const CameraModel* cams,
                               int n,
                               double X, double Y, double Z,
                               double* err)
{
    int k = 0;
    for (int i = 0; i < n; i++)
    {
        double u_proj, v_proj;
        project_point_opencv_distorted(&cams[i], X, Y, Z, &u_proj, &v_proj);

        err[k++] = points[i].u - u_proj;
        err[k++] = points[i].v - v_proj;
    }
}

static double l2_norm(const double* e, int n)
{
    double s = 0.0;
    for (int i = 0; i < n; i++) s += e[i]*e[i];
    return std::sqrt(s);
}

/* ========================================================= */
/* Solve 3x3 system                                          */
/* ========================================================= */
static int solve_3x3(double A[3][3], double b[3], double x[3])
{
    double M[3][4];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) M[i][j] = A[i][j];
        M[i][3] = b[i];
    }

    for (int i = 0; i < 3; i++) {
        double piv = M[i][i];
        if (fabs(piv) < 1e-12) return -1;
        for (int j = i; j < 4; j++) M[i][j] /= piv;
        for (int k = 0; k < 3; k++) {
            if (k == i) continue;
            double f = M[k][i];
            for (int j = i; j < 4; j++)
                M[k][j] -= f * M[i][j];
        }
    }

    for (int i = 0; i < 3; i++)
        x[i] = M[i][3];

    return 0;
}

/* ========================================================= */
/* TRIANGULATION + REFINEMENT (DEBUG)                        */
/* ========================================================= */
int triangulate_point_opencv(const ObservedPoint2D* points,
                             const CameraModel* cams,
                             int n,
                             double* X, double* Y, double* Z)
{
    if (n < 2) return -1;

    printf("\n[TRIANG][DBG] ===== NOUVELLE TRIANGULATION =====\n");

    /* ---------- 1. Projection matrices ---------- */
    std::vector<cv::Mat> projMats;
    std::vector<cv::Point2d> imgPts;

    for (int i = 0; i < n; ++i)
    {
        cv::Mat K = (cv::Mat_<double>(3,3) <<
            cams[i].K.fx, cams[i].K.s,  cams[i].K.cx,
            0.0,          cams[i].K.fy, cams[i].K.cy,
            0.0,          0.0,           1.0);

        cv::Mat R(3,3,CV_64F,(void*)cams[i].RT.R);
        cv::Mat t(3,1,CV_64F,(void*)cams[i].RT.t);

        cv::Mat Rt;
        cv::hconcat(R, t, Rt);

        printf("[TRIANG][DBG] Cam %d\n", i);
        printf("[TRIANG][DBG] K=\n");
        std::cout << K << std::endl;
        printf("[TRIANG][DBG] RT=\n");
        std::cout << Rt << std::endl;

        projMats.push_back(K * Rt);
        imgPts.emplace_back(points[i].u, points[i].v);

        printf("[TRIANG][DBG] point obs = (%.2f %.2f)\n",
               points[i].u, points[i].v);
    }

    /* ---------- 2. Triangulation initiale ---------- */
    cv::Mat Xh(4,1,CV_64F);
    cv::Mat p1 = (cv::Mat_<double>(2,1) << imgPts[0].x, imgPts[0].y);
    cv::Mat p2 = (cv::Mat_<double>(2,1) << imgPts[1].x, imgPts[1].y);
    cv::triangulatePoints(projMats[0], projMats[1], p1, p2, Xh);

    double x[3] = {
        Xh.at<double>(0) / Xh.at<double>(3),
        Xh.at<double>(1) / Xh.at<double>(3),
        Xh.at<double>(2) / Xh.at<double>(3)
    };

    printf("[TRIANG][DBG] Init X = (%.4f %.4f %.4f)\n",
           x[0], x[1], x[2]);

    /* ---------- 3. Erreur initiale ---------- */
    int m = 2 * n;
    double err[m], err_p[m], err_m[m];
    reprojection_error(points, cams, n, x[0], x[1], x[2], err);
    double prev = l2_norm(err, m);

    printf("[TRIANG][DBG] Err init = %.3f px\n", prev);

    /* ---------- 4. Raffinement LM ---------- */
    double lambda = LAMBDA_INIT;

    for (int it = 0; it < MAX_ITERS; it++)
    {
        double J[m][3];

        for (int k = 0; k < 3; k++)
        {
            double xp[3] = {x[0],x[1],x[2]};
            double xm[3] = {x[0],x[1],x[2]};
            xp[k] += EPS_JAC;
            xm[k] -= EPS_JAC;

            reprojection_error(points,cams,n,xp[0],xp[1],xp[2],err_p);
            reprojection_error(points,cams,n,xm[0],xm[1],xm[2],err_m);

            for (int i = 0; i < m; i++)
                J[i][k] = (err_p[i] - err_m[i]) / (2*EPS_JAC);
        }

        double JTJ[3][3] = {{0}};
        double JTe[3] = {0};

        for (int i = 0; i < m; i++)
            for (int a = 0; a < 3; a++) {
                JTe[a] += J[i][a] * err[i];
                for (int b = 0; b < 3; b++)
                    JTJ[a][b] += J[i][a] * J[i][b];
            }

        for (int i = 0; i < 3; i++)
            JTJ[i][i] += lambda;

        double dx[3];
        if (solve_3x3(JTJ, JTe, dx) != 0) {
            printf("[TRIANG][DBG] Solve failed\n");
            break;
        }

        double xn[3] = {x[0]+dx[0], x[1]+dx[1], x[2]+dx[2]};
        reprojection_error(points,cams,n,xn[0],xn[1],xn[2],err);
        double now = l2_norm(err,m);

        printf("[TRIANG][DBG] it=%02d err=%.3f λ=%.2e Δ=(%.3g %.3g %.3g)\n",
               it, now, lambda, dx[0], dx[1], dx[2]);

        if (now < prev) {
            memcpy(x, xn, sizeof(x));
            prev = now;
            lambda *= 0.5;
        } else {
            lambda *= 10.0;
        }
    }

    printf("[TRIANG][DBG] Final X = (%.4f %.4f %.4f)\n",
           x[0], x[1], x[2]);
    printf("[TRIANG][DBG] Final reproj err = %.3f px\n", prev);

    *X = x[0];
    *Y = x[1];
    *Z = x[2];
    return 0;
}
