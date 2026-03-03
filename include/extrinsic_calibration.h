#ifndef EXTRINSIC_CALIBRATION_H
#define EXTRINSIC_CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

int live_calibrate_extrinsics(int cam_id,
                              int w,int h,
                              const char* intrinsic,
                              const char* output);

#ifdef __cplusplus
}
#endif

#endif
