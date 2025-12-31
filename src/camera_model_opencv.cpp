#include "camera_model.h"
#include <iostream>
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

void load_calibration_params(const char* filename, CameraModel* model)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[USB_CAMERA] Impossible d'ouvrir " << filename << " (utilisation des valeurs par défaut)" << std::endl;
        return;
    }

    cv::Mat cameraMatrix, distCoeffs;
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;

    // Intrinsèques
    model->K.fx = cameraMatrix.at<double>(0,0);
    model->K.fy = cameraMatrix.at<double>(1,1);
    model->K.cx = cameraMatrix.at<double>(0,2);
    model->K.cy = cameraMatrix.at<double>(1,2);
    model->K.s  = 0.0;  // skew généralement 0

    // Distorsion complète
    model->K.k1 = distCoeffs.at<double>(0);
    model->K.k2 = distCoeffs.at<double>(1);
    model->K.p1 = distCoeffs.at<double>(2);
    model->K.p2 = distCoeffs.at<double>(3);
    model->K.k3 = distCoeffs.at<double>(4);
    model->K.k4 = distCoeffs.at<double>(5);
    model->K.k5 = distCoeffs.at<double>(6);
    model->K.k6 = distCoeffs.at<double>(7);
    model->K.s1 = distCoeffs.at<double>(8);
    model->K.s2 = distCoeffs.at<double>(9);
    model->K.s3 = distCoeffs.at<double>(10);
    model->K.s4 = distCoeffs.at<double>(11);

    fs.release();
    std::cout << "[USB_CAMERA] Calibration chargée depuis " << filename << std::endl;
    std::cout << "[USB_CAMERA] fx=" << model->K.fx << " fy=" << model->K.fy 
              << " cx=" << model->K.cx << " cy=" << model->K.cy << std::endl;
}

void undistort_point_opencv(const CameraModel* cam, double u_in, double v_in,
                            double* u_out, double* v_out)
{
    cv::Mat K = (cv::Mat_<double>(3,3) <<
                 cam->K.fx, cam->K.s, cam->K.cx,
                 0, cam->K.fy, cam->K.cy,
                 0, 0, 1);

    cv::Mat distCoeffs = (cv::Mat_<double>(1,14) <<
        cam->K.k1, cam->K.k2, cam->K.p1, cam->K.p2, cam->K.k3,
        cam->K.k4, cam->K.k5, cam->K.k6,
        cam->K.s1, cam->K.s2, cam->K.s3, cam->K.s4,
        0.0, 0.0);  // k7,k8 inutilisés

    std::vector<cv::Point2d> pts_in  = { cv::Point2d(u_in, v_in) };
    std::vector<cv::Point2d> pts_out;

    cv::undistortPoints(pts_in, pts_out, K, distCoeffs, cv::noArray(), K);

    *u_out = pts_out[0].x;
    *v_out = pts_out[0].y;
}