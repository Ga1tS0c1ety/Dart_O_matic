#ifndef USB_CAMERA_H
#define USB_CAMERA_H

#include <stddef.h>  // pour size_t

#ifdef __cplusplus
extern "C" {
#endif

// Initialise la caméra USB (index = 0, 1, 2...)
int usb_camera_init(int camera_index, int width, int height);

// Lit une frame brute + applique la projection de points 3D
// L'image résultante (BGR, 8-bit) est copiée dans output_buffer
// Retourne 0 si OK, -1 si erreur ou fin de stream
// Tu dois fournir un buffer d'au moins width * height * 3 octets
int usb_camera_read(unsigned char* output_buffer, size_t buffer_size);

// Optionnel : récupère les dimensions actuelles
void usb_camera_get_size(int* width, int* height);

// Ferme proprement
void usb_camera_close(void);

#ifdef __cplusplus
}
#endif

#endif