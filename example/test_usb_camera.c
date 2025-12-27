// example/test_usb_camera.c
#include <stdio.h>
#include <stdlib.h>
#include "../include/usb_camera.h"

int main(void)
{
    int width, height;

    // Change l'index selon ta caméra USB (teste 0, 1, 2, 3...)
    if (usb_camera_init(2, 1280, 720) != 0) {
        printf("Erreur : impossible d'ouvrir la caméra USB (index 2)\n");
        return -1;
    }

    usb_camera_get_size(&width, &height);
    size_t buffer_size = (size_t)width * height * 3;

    unsigned char *frame_buffer = (unsigned char *)malloc(buffer_size);
    if (frame_buffer == NULL) {
        printf("Erreur allocation mémoire\n");
        usb_camera_close();
        return -1;
    }

    printf("Caméra USB prête ! Le flux s'affiche dans une fenêtre OpenCV.\n");
    printf("Appuie sur 'q' dans la fenêtre pour quitter.\n");

    while (1) {
        if (usb_camera_read(frame_buffer, buffer_size) != 0) {
            printf("Fin du stream ou erreur de capture\n");
            break;
        }

        // L'affichage est géré entièrement à l'intérieur de usb_camera.cpp
        // Ici on ne fait rien d'autre → propre et modulaire
    }

    free(frame_buffer);
    usb_camera_close();
    return 0;
}