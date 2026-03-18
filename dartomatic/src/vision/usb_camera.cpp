#include "vision/usb_camera.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/core.hpp>
#include <unistd.h>
#include <cstring>

static cv::VideoCapture cap;
static cv::Mat frame_raw;
static cv::Mat frame_processed;
static int cam_width = 0;
static int cam_height = 0;

static bool display_enabled = false;
static bool window_created = false;

static const char* USB_CAMERA_WINDOW_NAME = "USB_CAMERA_DEBUG";

int usb_camera_init(int camera_index, int width, int height)
{
    /* Petit décalage pour éviter négociation USB simultanée */
    usleep(500000 * camera_index);

    if (!cap.open(camera_index, cv::CAP_V4L2)) {
        std::cerr << "[USB_CAMERA] Erreur ouverture caméra index "
                  << camera_index << std::endl;
        return -1;
    }

    /* FORCER MJPG */
    cap.set(cv::CAP_PROP_FOURCC,
            cv::VideoWriter::fourcc('M','J','P','G'));

    /* FORCER RESOLUTION */
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);

    /* FORCER FPS */
    cap.set(cv::CAP_PROP_FPS, 30);

    /* BUFFER */
    cap.set(cv::CAP_PROP_BUFFERSIZE, 4);

    int fourcc = (int)cap.get(cv::CAP_PROP_FOURCC);
    char fcc[] = {
        (char)(fourcc & 0xFF),
        (char)((fourcc >> 8) & 0xFF),
        (char)((fourcc >> 16) & 0xFF),
        (char)((fourcc >> 24) & 0xFF),
        0
    };

    cam_width  = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
    cam_height = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    double fps = cap.get(cv::CAP_PROP_FPS);

    std::cout << "[USB_CAMERA] Cam " << camera_index
              << " | FOURCC: " << fcc
              << " | " << cam_width << "x" << cam_height
              << " @ ~" << fps << " fps"
              << std::endl;

    if (std::string(fcc) != "MJPG") {
        std::cerr << "[USB_CAMERA] WARNING: MJPG non appliqué!"
                  << std::endl;
    }

    window_created = false;
    return 0;
}

int usb_camera_read(unsigned char* output_buffer, size_t buffer_size)
{
    if (!cap.isOpened()) return -1;

    cap >> frame_raw;
    if (frame_raw.empty()) return -1;

    frame_processed = frame_raw.clone();

    size_t required = (size_t)cam_width * cam_height * 3;
    if (buffer_size < required) return -1;

    std::memcpy(output_buffer, frame_processed.data, required);

    if (display_enabled) {
        if (!window_created) {
            cv::namedWindow(USB_CAMERA_WINDOW_NAME, cv::WINDOW_AUTOSIZE);
            window_created = true;
        }

        cv::imshow(USB_CAMERA_WINDOW_NAME, frame_processed);
        cv::waitKey(1);
    }

    return 0;
}

void usb_camera_get_size(int* width, int* height)
{
    if (width) *width = cam_width;
    if (height) *height = cam_height;
}

void usb_camera_set_display_enabled(int enabled)
{
    display_enabled = (enabled != 0);

    if (!display_enabled && window_created) {
        cv::destroyWindow(USB_CAMERA_WINDOW_NAME);
        window_created = false;
    }
}

int usb_camera_get_display_enabled(void)
{
    return display_enabled ? 1 : 0;
}

void usb_camera_close(void)
{
    if (window_created) {
        cv::destroyWindow(USB_CAMERA_WINDOW_NAME);
        window_created = false;
    }

    display_enabled = false;

    if (cap.isOpened()) {
        cap.release();
    }
}