#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "vision/usb_camera.h"

/*
 * debug_camera
 * ------------
 * Tool simple pour valider :
 *  - ouverture caméra
 *  - flux vidéo
 *  - dimensions réelles
 *
 * Usage :
 *   ./build/debug_camera <camera_id>
 *
 * Quit :
 *   Ctrl+C dans le terminal
 *
 * Note :
 *   l'affichage éventuel dépend de ton usb_camera.cpp
 *   (si DEBUG / display est activé côté implémentation).
 */

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::printf("Usage: %s <camera_id>\n", argv[0]);
        return -1;
    }

    int camera_id = std::atoi(argv[1]);
    if (camera_id < 0) {
        std::fprintf(stderr, "[DEBUG_CAMERA] camera_id invalide\n");
        return -1;
    }

    if (usb_camera_init(camera_id, 1280, 720) != 0) {
        usb_camera_set_display_enabled(1);
        std::fprintf(stderr, "[DEBUG_CAMERA] erreur ouverture caméra %d\n", camera_id);
        return -1;
    }

    int w = 0, h = 0;
    usb_camera_get_size(&w, &h);

    std::printf("[DEBUG_CAMERA] caméra %d prête (%dx%d)\n", camera_id, w, h);
    std::printf("[DEBUG_CAMERA] Ctrl+C pour quitter\n");
    std::fflush(stdout);

    size_t buf_size = (size_t)w * h * 3;
    unsigned char* frame_buffer = (unsigned char*)std::malloc(buf_size);
    if (!frame_buffer) {
        std::fprintf(stderr, "[DEBUG_CAMERA] malloc failed\n");
        usb_camera_close();
        return -1;
    }

    unsigned long frame_count = 0;

    while (1) {
        if (usb_camera_read(frame_buffer, buf_size) != 0) {
            std::fprintf(stderr, "[DEBUG_CAMERA] erreur lecture frame\n");
            break;
        }

        frame_count++;

        if ((frame_count % 120UL) == 0UL) {
            std::printf("[DEBUG_CAMERA] %lu frames lues\n", frame_count);
            std::fflush(stdout);
        }
    }

    std::free(frame_buffer);
    usb_camera_close();
    usb_camera_set_display_enabled(0);
    return 0;
}