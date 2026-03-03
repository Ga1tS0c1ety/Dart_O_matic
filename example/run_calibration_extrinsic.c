#include <stdio.h>
#include <stdlib.h>
#include "../include/extrinsic_calibration.h"

int main(int argc, char** argv)
{
    if (argc != 3) {
        printf("Usage : %s <camera_id> <intrinsic_yaml>\n", argv[0]);
        return -1;
    }

    int camera_id = atoi(argv[1]);
    const char* intrinsic_file = argv[2];

    char output_file[256];
    snprintf(output_file, sizeof(output_file),
             "cam_param/camera_extrinsics_%d.yaml", camera_id);

    printf("\n=== LIVE EXTRINSIC DEBUGGER ===\n");
    printf("Camera ID : %d\n", camera_id);
    printf("Intrinsic : %s\n", intrinsic_file);
    printf("Extrinsic : %s\n\n", output_file);

    int ret = live_calibrate_extrinsics(camera_id,
                                        1280, 720,
                                        intrinsic_file,
                                        output_file);

    if (ret == 0)
        printf("\n[OK] Extrinsiques sauvegardees : %s\n", output_file);
    else
        printf("\n[FAIL] Calibration annulee\n");

    return ret;
}
