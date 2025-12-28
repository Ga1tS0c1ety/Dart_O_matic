// src/usb_camera.cpp (modification importante)
#include "usb_camera.h"
#include "../include/camera_model.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/core.hpp>  // pour FileStorage
//#define OPENCV

static cv::VideoCapture cap;
static cv::Mat frame_raw;
static cv::Mat frame_processed;
static int cam_width = 0;
static int cam_height = 0;
static bool display_enabled = true;  // on affiche par défaut dans les exemples

int usb_camera_init(int camera_index, int width, int height) {
    if (!cap.open(camera_index, cv::CAP_V4L2)) {  // garde V4L2 pour stabilité
        std::cerr << "[USB_CAMERA] Erreur ouverture caméra index " << camera_index << std::endl;
        return -1;
    }

    // === FORCE MJPG EN PREMIER (très important) ===
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));

    // Ensuite la résolution
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);

    // Bonus : buffer minimal + FPS (optionnel)
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    // cap.set(cv::CAP_PROP_FPS, 30);  // parfois ignoré, mais tu peux tester

    // Récupère les valeurs réelles
    cam_width = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
    cam_height = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    double fps = cap.get(cv::CAP_PROP_FPS);

    std::cout << "[USB_CAMERA] Initialisée : " << cam_width << "x" << cam_height 
              << " @ ~" << fps << " fps en MJPG" << std::endl;

    cv::namedWindow("Caméra USB - Projection 3D", cv::WINDOW_AUTOSIZE);
    return 0;
}

static void load_calibration_params(const char* filename, CameraModel* model)
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



int usb_camera_read(unsigned char* output_buffer, size_t buffer_size) {
    if (!cap.isOpened()) return -1;

    cap >> frame_raw;
    if (frame_raw.empty()) return -1;

    frame_processed = frame_raw.clone();  // copie pour traitement

    // === Projection des points 3D ===
CameraModel model = {
        .K = {
            .fx = 600.0,
            .fy = 600.0,
            .cx = (double)cam_width / 2,
            .cy = (double)cam_height / 2,
            .s  = 0.0,
            // Distorsion : mise à zéro par défaut (à calibrer plus tard)
            .k1 = 0.0, .k2 = 0.0, .k3 = 0.0,
            .k4 = 0.0, .k5 = 0.0, .k6 = 0.0,
            .p1 = 0.0, .p2 = 0.0,
            .s1 = 0.0, .s2 = 0.0, .s3 = 0.0, .s4 = 0.0
        },
        .RT = {
            .R = {1,0,0, 0,1,0, 0,0,1},
            .t = {0,0,0}
        }
    };

    double test_points[][3] = {
        {0.0, 0.0, 1.0},
        {0.2, 0.0, 1.0}, {-0.2, 0.0, 1.0},
        {0.0, 0.2, 1.0}, {0.0, -0.2, 1.0},
        {0.15, 0.15, 0.8}, {-0.1, -0.15, 1.2}
    };

    // Après le std::cout de l'initialisation
    load_calibration_params("camera_params.yaml", &model);

// === Reprojection complète de l'image sur un plan Z = constant (vue 3D aplatie) ===
    double Z_plan = 5.0;  // Distance du plan virtuel en mètres (ajuste si tu veux : 0.5, 1.5, etc.)

    cv::Mat projected_image(cam_height, cam_width, CV_8UC3, cv::Scalar(50,50,50));  // fond gris foncé

    for (int v = 0; v < cam_height; ++v) {
        for (int u = 0; u < cam_width; ++u) {
            // Coordonnées normalisées (sans distorsion)
            double x = (u - model.K.cx) / model.K.fx;
            double y = (v - model.K.cy) / model.K.fy;

            // Point 3D sur le plan Z = Z_plan
            double X = x * Z_plan;
            double Y = y * Z_plan;
            double Z = Z_plan;

            double u_proj, v_proj;

            // Projection avec distorsion (choix entre OpenCV et manuelle)
#ifdef OPENCV
            project_point_opencv_distorted(&model, X, Y, Z, &u_proj, &v_proj);
#else
            project_point_distorted(&model, X, Y, Z, &u_proj, &v_proj);
#endif

            int ui = cvRound(u_proj);
            int vi = cvRound(v_proj);

            // Si le pixel projeté tombe dans l'image, on copie la couleur originale
            if (ui >= 0 && ui < cam_width && vi >= 0 && vi < cam_height) {
                projected_image.at<cv::Vec3b>(vi, ui) = frame_raw.at<cv::Vec3b>(v, u);
            }
        }
    }

    // Optionnel : superposer l'image originale en transparence pour comparer
   // cv::addWeighted(frame_raw, 0.4, projected_image, 0.6, 0.0, frame_processed);

    // Ou afficher seulement la vue projetée :
     frame_processed = projected_image.clone();

    // Texte info
    std::string text = "Vue 3D projetee sur plan Z = " + std::to_string(Z_plan) + " m";
    cv::putText(frame_processed, text, cv::Point(10, cam_height - 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

    // Copie dans le buffer fourni
    size_t required = (size_t)cam_width * cam_height * 3;
    if (buffer_size < required) return -1;
    std::memcpy(output_buffer, frame_processed.data, required);

    // Affichage
    if (display_enabled) {
        cv::imshow("Caméra USB - Projection 3D", frame_processed);
        cv::waitKey(1);
    }

    return 0;
}

void usb_camera_get_size(int* width, int* height) {
    if (width) *width = cam_width;
    if (height) *height = cam_height;
}

void usb_camera_close(void) {
    display_enabled = false;
    cv::destroyAllWindows();
    if (cap.isOpened()) cap.release();
}