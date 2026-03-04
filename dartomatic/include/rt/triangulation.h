#pragma once
/*
 * triangulation.h
 * --------------
 * API "RT" de triangulation : prend un ImpactBundle (u,v,conf par caméra)
 * et renvoie un point 3D (X,Y,Z) + une erreur reprojection.
 *
 * Conçu pour être appelé depuis rt_main.c (C), mais implémenté en C++ (OpenCV).
 */

#include <stdint.h>
#include "rt/aggregator.h"          /* ImpactBundle */
#include "vision/camera_model.h"    /* CameraModel */
#include "vision/triangulation_solve.h" 

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double X;              /* mètres */
    double Y;              /* mètres */
    double Z;              /* mètres */
    double reproj_err_px;  /* erreur moyenne en pixels */
    int used_cam_count;    /* ici 2 (V1: meilleure paire) */
    int cam_i;             /* index cam utilisé (dans cam_ids[]) */
    int cam_j;             /* index cam utilisé (dans cam_ids[]) */
} TriangulationResult;

/*
 * Charge les modèles caméra (intrinsics + extrinsics) pour chaque cam_id.
 *
 * intr_pattern/extr_pattern : printf patterns, ex:
 *   "data/cam_param/camera_params_%d.yaml"
 *   "data/cam_param/camera_extrinsics_%d.yaml"
 *
 * cams[] doit avoir taille >= n_cams.
 */
int triangulation_load_cameras(CameraModel* cams,
                               const int* cam_ids,
                               int n_cams,
                               const char* intr_pattern,
                               const char* extr_pattern);

/*
 * Triangule un point 3D à partir d'un bundle.
 * V1 : essaie toutes les paires de caméras "perpendiculaires"
 *      et garde celle qui minimise l'erreur de reprojection.
 *
 * cam_ids[] : nécessaire pour la logique "orientation" (0,2,4,6 -> croix)
 */
int triangulation_from_bundle(const ImpactBundle* b,
                              const CameraModel* cams,
                              const int* cam_ids,
                              int n_cams,
                              TriangulationResult* out);

#ifdef __cplusplus
}
#endif