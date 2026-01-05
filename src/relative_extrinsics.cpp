#include "relative_extrinsics.h"
#include <opencv2/opencv.hpp>
#include <iostream>

static bool load_extrinsic(const char* file, cv::Mat& R, cv::Mat& t)
{
    cv::FileStorage fs(file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Impossible d'ouvrir " << file << std::endl;
        return false;
    }

    fs["R"] >> R;
    fs["t"] >> t;
    fs.release();

    if (R.empty() || t.empty()) {
        std::cerr << "Fichier invalide : " << file << std::endl;
        return false;
    }

    R.convertTo(R, CV_64F);
    t.convertTo(t, CV_64F);

    return true;
}

int compute_relative_extrinsics(
    const char* cam1_yaml,
    const char* cam2_yaml,
    const char* output_yaml)
{
    cv::Mat R1, t1, R2, t2;

    if (!load_extrinsic(cam1_yaml, R1, t1)) return -1;
    if (!load_extrinsic(cam2_yaml, R2, t2)) return -1;

    /* === Calcul relatif === */
    cv::Mat R21 = R2 * R1.t();
    cv::Mat t21 = t2 - R21 * t1;

    std::cout << "[RELATIVE] R_21:\n" << R21 << std::endl;
    std::cout << "[RELATIVE] t_21:\n" << t21.t() << std::endl;

    /* === Sauvegarde === */
    cv::FileStorage fs(output_yaml, cv::FileStorage::WRITE);
    fs << "R" << R21;
    fs << "t" << t21;
    fs.release();

    std::cout << "[RELATIVE] Sauvegarde : " << output_yaml << std::endl;

    return 0;
}
