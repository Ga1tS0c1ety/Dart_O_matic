#ifndef VISION_CALIBRATION_H
#define VISION_CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

int live_calibrate_camera(int camera_index,
                          int width, int height,
                          int board_width_corners,
                          int board_height_corners,
                          float square_size_mm,
                          const char* output_file);

#ifdef __cplusplus
}
#endif

#endif