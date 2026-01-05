#include "extrinsic_calibration.h"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>

static std::vector<cv::Point2f> clicked_points;
static bool done = false;

static void on_mouse(int event, int x, int y, int, void*)
{
    if (event == cv::EVENT_LBUTTONDOWN) {
        clicked_points.emplace_back((float)x, (float)y);
        std::cout << "[CLICK] (" << x << ", " << y << ")" << std::endl;
    }
}

extern "C"
int live_calibrate_extrinsics(int camera_index,
                              int width, int height,
                              const char* intrinsic_file,
                              const char* output_file)
{
    /* === Points 3D réels (EN CM, exemple) === */
    std::vector<cv::Point3f> object_points = {
        {  0.0f,  0.0f, 0.0f }, //centre
        { 0.0f,  0.0f, -100.0f }, //haut
        { 0.0f, 0.0f, 100.0f }, //bas
        {  100.0f, 0.0f, 100.0f },//bas-droite
        {  -100.0f,  0.0f, 100.0f },//bas gauchee
        {  -100.0f, 0.0f, -100.0f },//haut gauchee
        {  100.0f,  0.0f, -100.0f },//haut droite
        {  100.0f,  0.0f, 0.0f },//droite
        {  -100.0f,  0.0f, 0.0f }//gauche
    };

    std::cout << "[EXTRINSIC] Points 3D utilises :" << std::endl;
    for (auto& p : object_points)
        std::cout << "  (" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;

    /* === Charger intrinsics === */
    cv::Mat K, dist;
    cv::FileStorage fs(intrinsic_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Impossible d'ouvrir " << intrinsic_file << std::endl;
        return -1;
    }
    fs["camera_matrix"] >> K;
    fs["distortion_coefficients"] >> dist;
    fs.release();

    /* === Caméra === */
    cv::VideoCapture cap(camera_index, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);

    if (!cap.isOpened()) {
        std::cerr << "Erreur ouverture caméra" << std::endl;
        return -1;
    }

    cv::namedWindow("Extrinsic Calibration");
    cv::setMouseCallback("Extrinsic Calibration", on_mouse);

    std::cout << "\n[INSTRUCTIONS]\n"
              << "- Clique sur les points dans l'ORDRE DEFINI\n"
              << "- Appuie sur 'c' quand tous les points sont cliques\n"
              << "- Appuie sur 'r' pour reset\n"
              << "- Appuie sur 'q' pour quitter\n";

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        for (size_t i = 0; i < clicked_points.size(); ++i) {
            cv::circle(frame, clicked_points[i], 4, {0,255,0}, -1);
            cv::putText(frame, std::to_string(i),
                        clicked_points[i] + cv::Point2f(5, -5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,255,0}, 1);
        }

        cv::imshow("Extrinsic Calibration", frame);
        char key = (char)cv::waitKey(10);

        if (key == 'q') return -1;

        if (key == 'r') {
            clicked_points.clear();
            std::cout << "[RESET]\n";
        }

        if (key == 'c') {
            if (clicked_points.size() != object_points.size()) {
                std::cout << "[ERREUR] "
                          << clicked_points.size()
                          << " points cliques, attendu "
                          << object_points.size() << std::endl;
                continue;
            }
            break;
        }
    }

    /* === solvePnP === */
    cv::Mat rvec, tvec;
    bool ok = cv::solvePnP(object_points,
                           clicked_points,
                           K, dist,
                           rvec, tvec,
                           false,
                           cv::SOLVEPNP_ITERATIVE);

    if (!ok) {
        std::cerr << "solvePnP a echoue\n";
        return -1;
    }

    cv::Mat R;
    cv::Rodrigues(rvec, R);

    std::cout << "\n[EXTRINSIC] Rotation R:\n" << R << std::endl;
    std::cout << "[EXTRINSIC] Translation t:\n" << tvec.t() << std::endl;

    /* === Sauvegarde === */
    cv::FileStorage out(output_file, cv::FileStorage::WRITE);
    out << "R" << R;
    out << "t" << tvec;
    out.release();

    std::cout << "[EXTRINSIC] Sauvegarde dans " << output_file << std::endl;

    return 0;
}
