#include <stdio.h>
#include <stdlib.h>
#include "../include/calibration.h"

int main(int argc, char** argv) {
    if (argc != 2) {
        printf("Usage : %s <camera_id>\n", argv[0]);
        return -1;
    }

    int camera_id = atoi(argv[1]);
    if (camera_id < 0) {
        printf("ID caméra invalide.\n");
        return -1;
    }

    // Construire le nom du fichier de sortie
    char filename[256];
    snprintf(filename, sizeof(filename), "cam_param/camera_params_%d.yaml", camera_id);

    // Paramètres de calibration
    int width = 1280;
    int height = 720;
    int board_width = 9;
    int board_height = 6;
    float square_size = 8.0f; // mm

    int ret = live_calibrate_camera(camera_id, width, height,
                                    board_width, board_height,
                                    square_size, filename);

    if (ret == 0) {
        printf("Calibration live réussie ! Fichier : %s\n", filename);
    } else {
        printf("Calibration live échouée ou annulée.\n");
    }

    return 0;
}
