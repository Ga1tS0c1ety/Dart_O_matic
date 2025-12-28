// src/calibration.cpp
#include "calibration.h"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>

extern "C" int live_calibrate_camera(int camera_index,
                                     int width, int height,
                                     int board_width_corners,
                                     int board_height_corners,
                                     float square_size_mm,
                                     const char* output_file)
{
    cv::VideoCapture cap;
    if (!cap.open(camera_index, cv::CAP_V4L2)) {
        std::cerr << "[LIVE_CALIBRATION] Erreur ouverture caméra index " << camera_index << std::endl;
        return -1;
    }

    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

    int cam_width = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int cam_height = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    std::cout << "[LIVE_CALIBRATION] Caméra ouverte : " << cam_width << "x" << cam_height << std::endl;

    cv::Size board_size(board_width_corners, board_height_corners);

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;

    std::vector<cv::Point3f> corners_3d;
    for (int i = 0; i < board_size.height; ++i) {
        for (int j = 0; j < board_size.width; ++j) {
            corners_3d.push_back(cv::Point3f(j * square_size_mm, i * square_size_mm, 0.0f));
        }
    }

    cv::Mat frame, gray;
    int valid_count = 0;

    std::cout << "[LIVE_CALIBRATION] Appuie sur ESPACE quand le damier est bien détecté (coins verts)" << std::endl;
    std::cout << "[LIVE_CALIBRATION] Appuie sur 'c' pour lancer la calibration (quand tu as assez de vues)" << std::endl;
    std::cout << "[LIVE_CALIBRATION] Appuie sur 'q' pour quitter sans calibrer" << std::endl;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, board_size, corners,
                                               cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        cv::Mat display = frame.clone();

        if (found) {
            cv::cornerSubPix(gray, corners, cv::Size(11,11), cv::Size(-1,-1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));

            cv::drawChessboardCorners(display, board_size, corners, found);

            cv::putText(display, "Damier detecte - Appuie ESPACE pour ajouter", cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2);
        } else {
            cv::putText(display, "Damier NON detecte", cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,255), 2);
        }

        cv::putText(display, "Vues valides: " + std::to_string(valid_count), cv::Point(10, 70),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,255,0), 2);

        cv::imshow("Live Calibration - Bouge le telephone", display);

        char key = (char)cv::waitKey(1);
        if (key == 'q' || key == 'Q') {
            std::cout << "[LIVE_CALIBRATION] Quitte sans calibrer" << std::endl;
            break;
        }

        if (found && (key == ' ')) {
            object_points.push_back(corners_3d);
            image_points.push_back(corners);
            valid_count++;
            std::cout << "[LIVE_CALIBRATION] Vue ajoutee ! Total : " << valid_count << std::endl;
        }

        if (key == 'c' || key == 'C') {
            if (valid_count < 10) {
                std::cout << "[LIVE_CALIBRATION] Pas assez de vues (" << valid_count << "). Continue." << std::endl;
            } else {
                break;  // on lance la calibration
            }
        }
    }

    cap.release();
    cv::destroyAllWindows();

    if (valid_count < 10) {
        std::cerr << "[LIVE_CALIBRATION] ERREUR : seulement " << valid_count << " vues valides. Calibration annulée." << std::endl;
        return -1;
    }

    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;

    int flags = cv::CALIB_RATIONAL_MODEL | cv::CALIB_THIN_PRISM_MODEL;

    double rms = cv::calibrateCamera(object_points, image_points, cv::Size(cam_width, cam_height),
                                     cameraMatrix, distCoeffs, rvecs, tvecs, flags);

    std::cout << "\n[LIVE_CALIBRATION] === CALIBRATION TERMINÉE ===" << std::endl;
    std::cout << "[LIVE_CALIBRATION] Vues utilisées : " << valid_count << std::endl;
    std::cout << "[LIVE_CALIBRATION] Erreur RMS : " << rms << " pixels" << std::endl;

    cv::FileStorage fs(output_file, cv::FileStorage::WRITE);
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs << "rms_error" << rms;
    fs.release();

    std::cout << "[LIVE_CALIBRATION] Paramètres sauvegardés dans " << output_file << std::endl;

    return 0;
}