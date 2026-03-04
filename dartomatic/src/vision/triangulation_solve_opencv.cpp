#include "vision/triangulation_solve.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>   // circle, putText
#include <opencv2/highgui.hpp>   // imshow, waitKey
#include <opencv2/calib3d.hpp>
#include <vector>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <fstream>

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


static void reprojection_error_undist(const ObservedPoint2D* points,
                                       const CameraModel* cams,
                                       int n,
                                       double X, double Y, double Z,
                                       double* err)
{
    int k = 0;
    for (int i = 0; i < n; i++) {
        double u_proj, v_proj;
        project_point_no_distortion(&cams[i], X, Y, Z, &u_proj, &v_proj);
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
    if (n < 2) {
        printf("[TRIANG] Error: at least 2 cameras required\n");
        return -1;
    }

    printf("\n[TRIANG][DBG] ===== TRIANGULATION (%d cameras) =====\n", n);

    /* ---------- 1. Construction des matrices de projection ---------- */
    std::vector<cv::Mat> projMats;
    std::vector<cv::Point2d> imgPts;

    for (int i = 0; i < n; ++i) {
    double K_data[9] = {
        cams[i].K.fx, cams[i].K.s,  cams[i].K.cx,
        0.0,          cams[i].K.fy, cams[i].K.cy,
        0.0,          0.0,          1.0
    };
    double R_data[9] = {
        cams[i].RT.R[0], cams[i].RT.R[1], cams[i].RT.R[2],
        cams[i].RT.R[3], cams[i].RT.R[4], cams[i].RT.R[5],
        cams[i].RT.R[6], cams[i].RT.R[7], cams[i].RT.R[8]
    };
    double t_data[3] = { cams[i].RT.t[0], cams[i].RT.t[1], cams[i].RT.t[2] };

    cv::Mat K(3, 3, CV_64F, K_data);
    cv::Mat R(3, 3, CV_64F, R_data);
    cv::Mat t(3, 1, CV_64F, t_data);

    cv::Mat Rt;
    cv::hconcat(R, t, Rt);
    projMats.push_back(K * Rt);

    double u_undist, v_undist;
    undistort_point_opencv(&cams[i], points[i].u, points[i].v,
                           &u_undist, &v_undist);
    imgPts.emplace_back(u_undist, v_undist);

    printf("[TRIANG][DBG] Cam %d\n", i);
    printf("[TRIANG][DBG] K=\n");
    std::cout << K << std::endl;
    printf("[TRIANG][DBG] RT=\n");
    std::cout << Rt << std::endl;
    printf("[TRIANG][DBG] point obs = (%.3f %.3f)\n", points[i].u, points[i].v);
}

    /* ---------- 2. Initialisation du point 3D ---------- */
    cv::Mat Xh(4,1,CV_64F);

    if (n == 2) {
        cv::Mat p1 = (cv::Mat_<double>(2,1) << imgPts[0].x, imgPts[0].y);
        cv::Mat p2 = (cv::Mat_<double>(2,1) << imgPts[1].x, imgPts[1].y);
        cv::triangulatePoints(projMats[0], projMats[1], p1, p2, Xh);
    } else {
        cv::Mat A(2 * n, 4, CV_64F, cv::Scalar(0.0));
        for (int i = 0; i < n; ++i) {
            double u = imgPts[i].x;
            double v = imgPts[i].y;
            cv::Mat row0 = projMats[i].row(0);
            cv::Mat row1 = projMats[i].row(1);
            cv::Mat row2 = projMats[i].row(2);
            A.row(2*i)     = u * row2 - row0;
            A.row(2*i + 1) = v * row2 - row1;
        }
        cv::SVD svd(A, cv::SVD::FULL_UV);
        Xh = svd.vt.row(svd.vt.rows - 1).t();
    }

    double w = Xh.at<double>(3, 0);
    if (std::abs(w) < 1e-12) {
        printf("[TRIANG] Error: w ~ 0\n");
        return -1;
    }

    double x[3] = {
        Xh.at<double>(0, 0) / w,
        Xh.at<double>(1, 0) / w,
        Xh.at<double>(2, 0) / w
    };

    printf("[TRIANG][DBG] Point 3D initial = (%.4f %.4f %.4f)\n", x[0], x[1], x[2]);

    /* ---------- 3. Undistort les points pour le LM ---------- */
    ObservedPoint2D points_undist[n];
    for (int i = 0; i < n; i++) {
        undistort_point_opencv(&cams[i], points[i].u, points[i].v,
                               &points_undist[i].u, &points_undist[i].v);
    }


    /* ---------- 4. Erreur initiale ---------- */
    int m = 2 * n;
    double err[m];
    reprojection_error_undist(points_undist, cams, n, x[0], x[1], x[2], err);
    double prev = l2_norm(err, m);
    printf("[TRIANG][DBG] Erreur reprojection initiale = %.3f px\n", prev);

    /* ---------- 6. Erreur finale sur points distordus ---------- */
    reprojection_error(points, cams, n, x[0], x[1], x[2], err);
    double final_err = l2_norm(err, m) / n;
    printf("[TRIANG][DBG] Point 3D final = (%.4f %.4f %.4f)\n", x[0], x[1], x[2]);
    printf("[TRIANG][DBG] Erreur reprojection finale = %.3f px\n", final_err);

    *X = x[0];
    *Y = x[1];
    *Z = x[2];

    return 0;
}

// Config - adjust these values
#define IMG_SIZE              900
#define CENTER                (IMG_SIZE / 2)
#define SCALE                 1.5               // pixels per mm - tune this
#define DARTBOARD_RADIUS_MM   225.0             // outer radius (double ring)
#define DOUBLE_INNER_MM       170.0             // approx inner edge of double ring
#define TRIPLE_OUTER_MM       107.0             // approx outer edge of triple ring
#define TRIPLE_INNER_MM       99.0              // approx inner edge of triple ring
#define BULL_OUTER_MM         31.8              // outer bull (25 points)
#define BULL_INNER_MM         12.7              // inner bullseye (50 points)

static cv::Mat dart_img;


void render_dartboard_topview(double X_mm, double Y_mm, double Z_mm)
{
    // Create image only once
    if (dart_img.empty()) {
        dart_img = cv::Mat(IMG_SIZE, IMG_SIZE, CV_8UC3);
    }

    // Clear background
    dart_img.setTo(cv::Scalar(20, 20, 20));

    // Outer circle (double ring border)
    cv::circle(dart_img,
               cv::Point(CENTER, CENTER),
               static_cast<int>(DARTBOARD_RADIUS_MM * SCALE),
               cv::Scalar(255, 255, 255),
               2,
               cv::LINE_AA);

    // Inner edge of double ring (start of single area)
    cv::circle(dart_img,
               cv::Point(CENTER, CENTER),
               static_cast<int>(DOUBLE_INNER_MM * SCALE),
               cv::Scalar(180, 180, 180),
               1,
               cv::LINE_AA);

    // Triple ring - outer and inner borders
    cv::circle(dart_img,
               cv::Point(CENTER, CENTER),
               static_cast<int>(TRIPLE_OUTER_MM * SCALE),
               cv::Scalar(220, 220, 100),  // light yellow for triple
               2,
               cv::LINE_AA);

    cv::circle(dart_img,
               cv::Point(CENTER, CENTER),
               static_cast<int>(TRIPLE_INNER_MM * SCALE),
               cv::Scalar(220, 220, 100),
               2,
               cv::LINE_AA);

    // Outer bull (25 points ring)
    cv::circle(dart_img,
               cv::Point(CENTER, CENTER),
               static_cast<int>(BULL_OUTER_MM * SCALE),
               cv::Scalar(40, 40, 220),    // dark blue-ish
               2,
               cv::LINE_AA);

    // Inner bullseye (50 points)
    cv::circle(dart_img,
               cv::Point(CENTER, CENTER),
               static_cast<int>(BULL_INNER_MM * SCALE),
               cv::Scalar(0, 120, 255),    // bright red
               -1,
               cv::LINE_AA);

    // Small green center dot (for visibility)
    cv::circle(dart_img,
               cv::Point(CENTER, CENTER),
               4,
               cv::Scalar(0, 255, 0),
               -1);

    // World to image (X right, Y up)
    int px = CENTER + static_cast<int>(X_mm * SCALE);
    int py = CENTER - static_cast<int>(Y_mm * SCALE);

    // Impact point
    cv::circle(dart_img,
               cv::Point(px, py),
               6,
               cv::Scalar(0, 0, 255),
               -1,
               cv::LINE_AA);

    // Optional: white outline for better visibility
    cv::circle(dart_img,
               cv::Point(px, py),
               10,
               cv::Scalar(220, 220, 255),
               2,
               cv::LINE_AA);

    // Z debug text
    char txt[64];
    snprintf(txt, sizeof(txt), "Z = %.1f mm", Z_mm);
    cv::putText(dart_img,
                txt,
                cv::Point(15, 35),
                cv::FONT_HERSHEY_SIMPLEX,
                0.7,
                cv::Scalar(200, 200, 200),
                2);

    // Distance + out of board
    double r = hypot(X_mm, Y_mm);
    if (r > DARTBOARD_RADIUS_MM) {
        cv::putText(dart_img,
                    "HORS PLATEAU",
                    cv::Point(15, 70),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.8,
                    cv::Scalar(0, 0, 255),
                    2);
    }

    // Show and wait (as you wanted)
    cv::imshow("Dartboard - Top View", dart_img);
    cv::waitKey(1);
}
