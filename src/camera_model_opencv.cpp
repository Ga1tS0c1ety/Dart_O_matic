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

}
