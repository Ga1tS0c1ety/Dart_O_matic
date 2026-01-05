#ifndef EXTRINSIC_CALIBRATION_H
#define EXTRINSIC_CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

int live_calibrate_extrinsics(int camera_index,
                              int width, int height,
                              const char* intrinsic_file,
                              const char* output_file);

#ifdef __cplusplus
}
#endif

#endif
