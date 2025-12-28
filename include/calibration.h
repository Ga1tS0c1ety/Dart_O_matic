#ifndef CALIBRATION_H
#define CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

// Nouvelle fonction pour calibration live
int live_calibrate_camera(int camera_index,          // index de ta caméra USB
                          int width, int height,     // résolution souhaitée
                          int board_width_corners,   // ex: 9
                          int board_height_corners,  // ex: 6
                          float square_size_mm,      // taille réelle d'un carré en mm
                          const char* output_file);  // ex: "camera_params.yaml"

#ifdef __cplusplus
}
#endif

#endif