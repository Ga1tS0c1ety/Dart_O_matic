#include <stdio.h>
#include <stdlib.h>
#include "relative_extrinsics.h"

int main(int argc, char** argv)
{
    if (argc != 4) {
        fprintf(stderr,
            "Usage : %s <cam1_extrinsic.yaml> <cam2_extrinsic.yaml> <output_relative.yaml>\n",
            argv[0]);
        return EXIT_FAILURE;
    }

    const char* cam1_yaml = argv[1];
    const char* cam2_yaml = argv[2];
    const char* out_yaml  = argv[3];

    printf("[RELATIVE] Cam1 : %s\n", cam1_yaml);
    printf("[RELATIVE] Cam2 : %s\n", cam2_yaml);
    printf("[RELATIVE] Output : %s\n", out_yaml);

    int ret = compute_relative_extrinsics(
        cam1_yaml,
        cam2_yaml,
        out_yaml
    );

    if (ret != 0) {
        fprintf(stderr, "[RELATIVE] ERREUR : calcul impossible\n");
        return EXIT_FAILURE;
    }

    printf("[RELATIVE] OK : extrinsèque relative calculée\n");
    return EXIT_SUCCESS;
}
