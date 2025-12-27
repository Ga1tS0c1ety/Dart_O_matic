// src/usb_camera.cpp (modification importante)
#include "usb_camera.h"
#include "../include/camera_model.h"
#include <opencv2/opencv.hpp>
#include <iostream>

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

int usb_camera_read(unsigned char* output_buffer, size_t buffer_size) {
    if (!cap.isOpened()) return -1;

    cap >> frame_raw;
    if (frame_raw.empty()) return -1;

    frame_processed = frame_raw.clone();  // copie pour traitement

    // === Projection des points 3D ===
    CameraModel model = {
        .K = {600.0, 600.0, (double)cam_width/2, (double)cam_height/2, 0.0},
        .RT = {{1,0,0, 0,1,0, 0,0,1}, {0,0,0}}
    };

    double test_points[][3] = {
        {0.0, 0.0, 1.0},
        {0.2, 0.0, 1.0}, {-0.2, 0.0, 1.0},
        {0.0, 0.2, 1.0}, {0.0, -0.2, 1.0},
        {0.15, 0.15, 0.8}, {-0.1, -0.15, 1.2}
    };

    for (int i = 0; i < 7; ++i) {
        double u, v;
        project_point_opencv(&model, test_points[i][0], test_points[i][1], test_points[i][2], &u, &v);
        if (u >= 0 && u < cam_width && v >= 0 && v < cam_height && test_points[i][2] > 0) {
            cv::circle(frame_processed, cv::Point(cvRound(u), cvRound(v)), 12, cv::Scalar(0,255,0), -1);
            cv::putText(frame_processed, std::to_string(i+1), cv::Point(cvRound(u)+15, cvRound(v)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,255,0), 2);
        }
    }

    // Copie dans le buffer fourni
    size_t required = (size_t)cam_width * cam_height * 3;
    if (buffer_size < required) return -1;
    std::memcpy(output_buffer, frame_processed.data, required);

    // Affichage (seulement ici, en C++)
    if (display_enabled) {
        cv::imshow("Caméra USB - Projection 3D", frame_processed);
        if (cv::waitKey(1) == 'q') {
            // On pourrait retourner un code spécial, mais pour l'exemple simple on continue
        }
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