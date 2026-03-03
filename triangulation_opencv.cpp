#include "triangulation.h"
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
        printf("[TRIANG] Erreur : au moins 2 cameras necessaires\n");
        return -1;
    }

    printf("\n[TRIANG] Triangulation OpenCV avec %d cameras\n", n);

    std::vector<cv::Mat> projMats;
    std::vector<cv::Point2d> imgPtsUndistorted;

    for (int i = 0; i < n; ++i) {
        // Intrinsques
        cv::Mat K = (cv::Mat_<double>(3,3) <<
            cams[i].K.fx, cams[i].K.s, cams[i].K.cx,
            0.0,          cams[i].K.fy, cams[i].K.cy,
            0.0,          0.0,          1.0);

        // Extrinsques : monde -> camera (depuis load_extrinsics_yaml)
        cv::Mat R(3, 3, CV_64F, (void*)cams[i].RT.R);
        cv::Mat t(3, 1, CV_64F, (void*)cams[i].RT.t);

        cv::Mat Rt;
        cv::hconcat(R, t, Rt);

        projMats.push_back(K * Rt);

        // Point observe -> undistort (obligatoire !)
        double u_undist, v_undist;
        undistort_point_opencv(&cams[i], points[i].u, points[i].v, &u_undist, &v_undist);
        imgPtsUndistorted.emplace_back(u_undist, v_undist);

        // Debug
        cv::Mat pos = -R.t() * t;
        printf("[TRIANG] Cam %d - pos monde : %.3f %.3f %.3f\n", 
               i, pos.at<double>(0), pos.at<double>(1), pos.at<double>(2));
        printf("[TRIANG] Cam %d - point original : (%.1f, %.1f) -> undist : (%.1f, %.1f)\n", 
               i, points[i].u, points[i].v, u_undist, v_undist);
    }

    // Triangulation
    cv::Mat points4D;

    if (n == 2) {
        cv::Mat p1 = (cv::Mat_<double>(2,1) << imgPtsUndistorted[0].x, imgPtsUndistorted[0].y);
        cv::Mat p2 = (cv::Mat_<double>(2,1) << imgPtsUndistorted[1].x, imgPtsUndistorted[1].y);
        cv::triangulatePoints(projMats[0], projMats[1], p1, p2, points4D);
    } else {
        cv::Mat A(2*n, 4, CV_64F);
        for (int i = 0; i < n; ++i) {
            cv::Mat proj = projMats[i];
            double u = imgPtsUndistorted[i].x;
            double v = imgPtsUndistorted[i].y;
            A.row(2*i)     = u * proj.row(2) - proj.row(0);
            A.row(2*i + 1) = v * proj.row(2) - proj.row(1);
        }
        cv::SVD svd(A, cv::SVD::FULL_UV);
        points4D = svd.vt.row(3).t();
    }

    double w = points4D.at<double>(3);
    if (std::abs(w) < 1e-8) {
        printf("[TRIANG] w trop petit\n");
        return -1;
    }

    double x = points4D.at<double>(0) / w;
    double y = points4D.at<double>(1) / w;
    double z = points4D.at<double>(2) / w;

    printf("[TRIANG] Point 3D : X=%.3f Y=%.3f Z=%.3f\n", x, y, z);

    // Erreur reprojection (sur points undistorted pour info)
    int m = 2 * n;
    double err[m];
    reprojection_error(points, cams, n, x, y, z, err);  // attention : reproj sur points distordus !
    double reproj_err = l2_norm(err, m) / n;

    printf("[TRIANG] Erreur reprojection moyenne : %.3f px\n", reproj_err);

    *X = x;
    *Y = y;
    *Z = z;

    return 0;
}


/* ========================================================= */
/* Correction config                                         */
/* ========================================================= */


void load_correction(TriangCorrection* c) {
    c->offset_X = c->offset_Y = c->offset_Z = 0.0;
    c->scale_X  = c->scale_Y  = c->scale_Z  = 1.0;

    FILE* f = fopen(CFG_FILE, "r");
    if (!f) return;
    fscanf(f, "offset_X=%lf\n", &c->offset_X);
    fscanf(f, "offset_Y=%lf\n", &c->offset_Y);
    fscanf(f, "offset_Z=%lf\n", &c->offset_Z);
    fscanf(f, "scale_X=%lf\n",  &c->scale_X);
    fscanf(f, "scale_Y=%lf\n",  &c->scale_Y);
    fscanf(f, "scale_Z=%lf\n",  &c->scale_Z);
    fclose(f);
}

void save_correction(TriangCorrection* c) {
    FILE* f = fopen(CFG_FILE, "w");
    if (!f) { printf("[CORRECTION] Cannot write %s\n", CFG_FILE); return; }
    fprintf(f, "offset_X=%.4f\n", c->offset_X);
    fprintf(f, "offset_Y=%.4f\n", c->offset_Y);
    fprintf(f, "offset_Z=%.4f\n", c->offset_Z);
    fprintf(f, "scale_X=%.4f\n",  c->scale_X);
    fprintf(f, "scale_Y=%.4f\n",  c->scale_Y);
    fprintf(f, "scale_Z=%.4f\n",  c->scale_Z);
    fclose(f);
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
static cv::Point2d g_triang_result = {0, 0};
static bool        g_clicked       = false;

static void on_mouse(int event, int x, int y, int flags, void* userdata)
{
    if (event != cv::EVENT_LBUTTONDOWN) return;

    // Pixel -> mm
    double click_X =  (x - CENTER) / (double)SCALE;
    double click_Y = -(y - CENTER) / (double)SCALE;

    double off_x = click_X - g_triang_result.x;
    double off_y = click_Y - g_triang_result.y;

    printf("[CORRECTION] Click : X=%.1f Y=%.1f mm\n", click_X, click_Y);
    printf("[CORRECTION] Offset computed : dX=%.1f dY=%.1f mm\n", off_x, off_y);

    // Load existing correction
    TriangCorrection corr;
    load_correction(&corr);

    // Accumulate
    corr.offset_X += off_x;
    corr.offset_Y += off_y;

    // Save
    save_correction(&corr);
    printf("[CORRECTION] Saved : offset_X=%.4f offset_Y=%.4f\n",
           corr.offset_X, corr.offset_Y);

    // Draw clicked point in green
    cv::circle(dart_img, cv::Point(x, y), 6,  cv::Scalar(0,255,0), -1, cv::LINE_AA);
    cv::circle(dart_img, cv::Point(x, y), 10, cv::Scalar(0,255,0), 2,  cv::LINE_AA);

    char lbl[64];
    snprintf(lbl, sizeof(lbl), "dX=%.1f dY=%.1f mm", off_x, off_y);
    cv::putText(dart_img, lbl, cv::Point(15, 100),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2);

    cv::imshow("Dartboard - Top View", dart_img);
    g_clicked = true;
}


void render_dartboard_topview(double X_mm, double Y_mm, double Z_mm)
{
    // Store triangulation result for mouse callback
    g_triang_result = {X_mm, Y_mm};
    g_clicked = false;
    
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

        static bool window_created = false;
    if (!window_created) {
        cv::namedWindow("Dartboard - Top View", cv::WINDOW_AUTOSIZE);
        cv::setMouseCallback("Dartboard - Top View", on_mouse, nullptr);
        window_created = true;
    }
    cv::imshow("Dartboard - Top View", dart_img);
    
    // Attend le clic (ou 's' pour skipper)
    printf("[CORRECTION] Cliquez sur la vraie position, ou appuyez sur 's' pour skipper\n");
    while (!g_clicked) {
        int key = cv::waitKey(50);
        if (key == 's' || key == 'S') break;
    }
}
