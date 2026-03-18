// src/usb_camera.cpp (modification importante)
#include "vision/usb_camera.h"
//#include "../include/camera_model.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/core.hpp>  // pour FileStorage
#include <unistd.h>
//#define OPENCV

static cv::VideoCapture cap;
static cv::Mat frame_raw;
static cv::Mat frame_processed;
static int cam_width = 0;
static int cam_height = 0;
static bool display_enabled = false;  // on n'affiche pas par défaut dans les exemples

int usb_camera_init(int camera_index, int width, int height)
{
    // Petit décalage pour éviter négociation USB simultanée
    usleep(500000 * camera_index);

    if (!cap.open(camera_index, cv::CAP_V4L2)) {
        std::cerr << "[USB_CAMERA] Erreur ouverture caméra index "
                  << camera_index << std::endl;
        return -1;
    }

    // === FORCER MJPG ===
    cap.set(cv::CAP_PROP_FOURCC,
            cv::VideoWriter::fourcc('M','J','P','G'));

    // === FORCER RESOLUTION ===
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);

    // === FORCER FPS (important en MJPG 720p) ===
    cap.set(cv::CAP_PROP_FPS, 30);

    // === BUFFER : NE PAS METTRE 1 EN MULTI-CAM ===
    // 3 ou 4 = bon compromis latence / stabilité
    cap.set(cv::CAP_PROP_BUFFERSIZE, 4);

    // === Vérification réelle du FOURCC ===
    int fourcc = (int)cap.get(cv::CAP_PROP_FOURCC);
    char fcc[] = {
        (char)(fourcc & 0xFF),
        (char)((fourcc >> 8) & 0xFF),
        (char)((fourcc >> 16) & 0xFF),
        (char)((fourcc >> 24) & 0xFF),
        0
    };

    // === Récupération valeurs réelles ===
    cam_width  = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
    cam_height = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    double fps = cap.get(cv::CAP_PROP_FPS);

    std::cout << "[USB_CAMERA] Cam " << camera_index
              << " | FOURCC: " << fcc
              << " | " << cam_width << "x" << cam_height
              << " @ ~" << fps << " fps"
              << std::endl;

    // Vérification critique
    if (std::string(fcc) != "MJPG") {
        std::cerr << "[USB_CAMERA] WARNING: MJPG non appliqué!"
                  << std::endl;
    }

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
        cv::imshow("Caméra USB - Projection 3D", frame_processed);
        cv::waitKey(1);
    }

    return 0;
}

void usb_camera_get_size(int* width, int* height) {
    if (width) *width = cam_width;
    if (height) *height = cam_height;
}

void usb_camera_set_display_enabled(int enabled) {
    display_enabled = (enabled != 0);
}

int usb_camera_get_display_enabled(void) {
    return display_enabled ? 1 : 0;
}

void usb_camera_close(void) {
    display_enabled = false;
    cv::destroyAllWindows();
    if (cap.isOpened()) cap.release();
}

