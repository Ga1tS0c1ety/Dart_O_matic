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