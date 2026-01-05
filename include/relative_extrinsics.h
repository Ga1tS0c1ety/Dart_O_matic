#ifndef RELATIVE_EXTRINSICS_H
#define RELATIVE_EXTRINSICS_H

#ifdef __cplusplus
extern "C" {
#endif

int compute_relative_extrinsics(
    const char* cam1_yaml,
    const char* cam2_yaml,
    const char* output_yaml
);

#ifdef __cplusplus
}
#endif

#endif
