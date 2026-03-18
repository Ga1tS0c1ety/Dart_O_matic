#ifndef DART_DETECTOR_H
#define DART_DETECTOR_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

int dart_detector_init(int width, int height);

/*
 * Traite une frame et détecte un tir.
 * Retour:
 *  1 si impact détecté
 *  0 si rien
 * -1 si erreur
 *
 * impact_u / impact_v : coordonnées en pixels
 * confidence : qualité [0..1] (si != NULL)
 */
int dart_detector_process(const unsigned char* input_frame, size_t frame_size,
                          double* impact_u, double* impact_v,
                          float* confidence);

void dart_detector_set_reference(const unsigned char* frame, size_t frame_size);
void dart_detector_close(void);

void dart_detector_set_debug_enabled(int enabled);
int dart_detector_get_debug_enabled(void);

#ifdef __cplusplus
}
#endif

#endif // DART_DETECTOR_H