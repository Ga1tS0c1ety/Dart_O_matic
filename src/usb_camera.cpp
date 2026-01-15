// src/usb_camera.cpp (modification importante)
#include "usb_camera.h"
#include "../include/camera_model.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/core.hpp>  // pour FileStorage
//#define OPENCV

//#define DEBUG

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

    #ifdef DEBUG
    std::cout << "[USB_CAMERA] Initialisée : " << cam_width << "x" << cam_height 
             << " @ ~" << fps << " fps en MJPG" << std::endl;

    cv::namedWindow("Caméra USB - Projection 3D", cv::WINDOW_AUTOSIZE);
    #endif
    return 0;
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
        #ifdef DEBUG
        cv::imshow("Caméra USB - Projection 3D", frame_processed);
        cv::waitKey(1);
        #endif
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