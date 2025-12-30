// example/test_dart_detector.c
#include <stdio.h>
#include <stdlib.h>   // <--- Pour malloc et free
#include "../include/usb_camera.h"
#include "../include/dart_detector.h"

int main(void) {
    if (usb_camera_init(4, 1280, 720) != 0) {
        printf("Erreur ouverture caméra\n");
        return -1;
    }

    int w, h;
    usb_camera_get_size(&w, &h);

    dart_detector_init(w, h);

    size_t buf_size = (size_t)w * h * 3;
    unsigned char* buffer = (unsigned char*)malloc(buf_size);  // <--- Cast explicite
    if (!buffer) {
        printf("Erreur allocation mémoire\n");
        usb_camera_close();
        return -1;
    }

    printf("Détection de tir en live. Appuie sur 'q' pour quitter.\n");

    while (1) {
        if (usb_camera_read(buffer, buf_size) != 0) {
            printf("Fin du stream\n");
            break;
        }

        double impact_u, impact_v;
        int detected = dart_detector_process(buffer, buf_size, &impact_u, &impact_v);

        if (detected) {
            printf("TIR DÉTECTÉ ! Point d'impact estimé : u=%.1f v=%.1f\n", impact_u, impact_v);
        }
    }

    free(buffer);
    dart_detector_close();
    usb_camera_close();

    return 0;
}