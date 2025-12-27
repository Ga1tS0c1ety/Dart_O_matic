#include "camera_model.h"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>

extern "C" {

// fonction C exposée pour projeter un point via OpenCV
void project_point_opencv(const CameraModel* cam,
                          double X, double Y, double Z,
                          double* u, double* v)
{
    cv::Mat K = (cv::Mat_<double>(3,3) <<
                 cam->K.fx, cam->K.s, cam->K.cx,
                 0, cam->K.fy, cam->K.cy,
                 0, 0, 1);

    cv::Mat R = (cv::Mat_<double>(3,3) <<
                 cam->RT.R[0], cam->RT.R[1], cam->RT.R[2],
                 cam->RT.R[3], cam->RT.R[4], cam->RT.R[5],
                 cam->RT.R[6], cam->RT.R[7], cam->RT.R[8]);

    cv::Mat t = (cv::Mat_<double>(3,1) <<
                 cam->RT.t[0], cam->RT.t[1], cam->RT.t[2]);

    cv::Mat rvec;
    cv::Rodrigues(R, rvec);

    std::vector<cv::Point3d> pts3D = { cv::Point3d(X, Y, Z) };
    std::vector<cv::Point2d> pts2D;

    cv::projectPoints(pts3D, rvec, t, K, cv::Mat(), pts2D);

    *u = pts2D[0].x;
    *v = pts2D[0].y;
}

// Nouvelle fonction : projection OpenCV avec distorsion complète (14 coeffs)
void project_point_opencv_distorted(const CameraModel* cam,
                                    double X, double Y, double Z,
                                    double* u, double* v)
{
    cv::Mat K = (cv::Mat_<double>(3,3) <<
                 cam->K.fx, cam->K.s, cam->K.cx,
                 0,         cam->K.fy, cam->K.cy,
                 0,         0,        1);

    cv::Mat R = (cv::Mat_<double>(3,3) <<
                 cam->RT.R[0], cam->RT.R[1], cam->RT.R[2],
                 cam->RT.R[3], cam->RT.R[4], cam->RT.R[5],
                 cam->RT.R[6], cam->RT.R[7], cam->RT.R[8]);

    cv::Mat t = (cv::Mat_<double>(3,1) <<
                 cam->RT.t[0], cam->RT.t[1], cam->RT.t[2]);

    cv::Mat rvec;
    cv::Rodrigues(R, rvec);

    // === Distorsion complète (14 coefficients OpenCV) ===
    cv::Mat distCoeffs = (cv::Mat_<double>(1,14) << 
        cam->K.k1, cam->K.k2, cam->K.p1, cam->K.p2, cam->K.k3,
        cam->K.k4, cam->K.k5, cam->K.k6,
        cam->K.s1, cam->K.s2, cam->K.s3, cam->K.s4,
        0.0, 0.0);  // k7 et k8 inutilisés par OpenCV

    std::vector<cv::Point3d> pts3D = { cv::Point3d(X, Y, Z) };
    std::vector<cv::Point2d> pts2D;

    cv::projectPoints(pts3D, rvec, t, K, distCoeffs, pts2D);

    *u = pts2D[0].x;
    *v = pts2D[0].y;
}

}
