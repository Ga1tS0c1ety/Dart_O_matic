#include <stdio.h>
#include "../include/calibration.h"

int main(void) {
    // Index de ta caméra USB (teste 0, 1, 2...)
    // Résolution souhaitée
    // 9x6 coins internes
    // Taille d'un carré mesurée sur ton téléphone (ex: 12.5 mm)
    int ret = live_calibrate_camera(2, 1280, 720, 9, 6, 12.5f, "camera_params.yaml");

    if (ret == 0) {
        printf("Calibration live réussie ! Fichier : camera_params.yaml\n");
    } else {
        printf("Calibration live échouée ou annulée.\n");
    }

    return 0;
}