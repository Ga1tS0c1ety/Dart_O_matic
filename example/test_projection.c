// example/test_projection.c

#include <stdio.h>
#include "../include/camera_model.h"

// déclaration de la fonction C++ wrapper
#ifdef __cplusplus
extern "C" {
#endif
void project_point_opencv(const CameraModel* cam,
                          double X, double Y, double Z,
                          double* u, double* v);
#ifdef __cplusplus
}
#endif

int main() {
    CameraModel cam = {
        .K = {800, 800, 320, 240, 0, 0, 0,0,0,0,0,0,0,0,0,0,0},
        .RT = {
            {1,0,0, 0,1,0, 0,0,1},
            {0,0,0}
        }
    };

    double u, v;
    project_point_opencv(&cam, 1.0, 2.0, 5.0, &u, &v);
    printf("Point projeté (OpenCV) : u=%f, v=%f\n", u, v);

    return 0;
}
