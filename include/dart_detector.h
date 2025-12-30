#ifndef DART_DETECTOR_H
#define DART_DETECTOR_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise le détecteur de tir
 * @param width, height : résolution de la caméra
 * @return 0 si succès, -1 si erreur
 */
int dart_detector_init(int width, int height);

/**
 * @brief Traite une nouvelle frame et détecte un éventuel tir
 * @param input_frame : buffer BGR de la frame courante (même format que usb_camera_read)
 * @param frame_size : taille du buffer (width * height * 3)
 * @param impact_u, impact_v : pointeur pour récupérer les coordonnées du point d'impact (en pixels)
 * @return 1 si un tir est détecté (impact valide), 0 si rien, -1 si erreur
 */
int dart_detector_process(const unsigned char* input_frame, size_t frame_size,
                          double* impact_u, double* impact_v);

/**
 * @brief Ferme le détecteur (libère la mémoire)
 */
void dart_detector_close(void);

#ifdef __cplusplus
}
#endif

#endif // DART_DETECTOR_H