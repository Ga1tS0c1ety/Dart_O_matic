#include "triangulation.h"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>
#include <cmath>

int triangulate_point_opencv(const ObservedPoint2D* points,
                             const CameraModel* cams,
                             int n,
                             double* X, double* Y, double* Z)
{
    if (n < 2) return -1; // au moins 2 caméras nécessaires

    std::vector<cv::Mat> projMats;      // matrices de projection 3x4
    std::vector<cv::Point2d> normPoints; // points normalisés

    // Préparer les matrices de projection et les points
    for (int i = 0; i < n; ++i)
    {
        // Matrice intrinsèque
        cv::Mat K = (cv::Mat_<double>(3,3) << 
            cams[i].K.fx, cams[i].K.s, cams[i].K.cx,
            0,           cams[i].K.fy, cams[i].K.cy,
            0,           0,            1
        );

        std::cout << "matrice K : " << K << std::endl;

        // Rotation et translation
        cv::Mat R = cv::Mat(3, 3, CV_64F, (void*)cams[i].RT.R);
        cv::Mat t = cv::Mat(3, 1, CV_64F, (void*)cams[i].RT.t);

        cv::Mat Rt;
        cv::hconcat(R, t, Rt);

        std::cout << "matrice RT : " << Rt << std::endl;

        cv::Mat P = K * Rt;
        projMats.push_back(P);

        // Point 2D
        cv::Point2d p(points[i].u, points[i].v);
        normPoints.push_back(p);
    }

    // Triangulation linéaire multi-vues (avec la première paire)
    cv::Mat X_homog(4,1,CV_64F);
    {
        cv::Mat pts1 = (cv::Mat_<double>(2,1) << normPoints[0].x, normPoints[0].y);
        cv::Mat pts2 = (cv::Mat_<double>(2,1) << normPoints[1].x, normPoints[1].y);
        cv::triangulatePoints(projMats[0], projMats[1], pts1, pts2, X_homog);
    }

    // Conversion en coordonnées cartésiennes
    double x = X_homog.at<double>(0) / X_homog.at<double>(3);
    double y = X_homog.at<double>(1) / X_homog.at<double>(3);
    double z = X_homog.at<double>(2) / X_homog.at<double>(3);

    // Vérification profondeur positive
    for (int i = 0; i < n; ++i)
    {
        cv::Mat R = cv::Mat(3, 3, CV_64F, (void*)cams[i].RT.R);
        cv::Mat t = cv::Mat(3, 1, CV_64F, (void*)cams[i].RT.t);
        cv::Mat Xc = R * (cv::Mat_<double>(3,1) << x, y, z) + t;
        if (Xc.at<double>(2) <= 0)
            return -2; // point derrière la caméra
    }

    // Assignation des coordonnées
    *X = x;
    *Y = y;
    *Z = z;

    return 0; // succès
}

