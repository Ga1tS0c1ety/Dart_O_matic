#pragma once

/*
 * mpu6050_thread.h
 * ----------------
 * Interface du thread de lecture du capteur MPU6050.
 *
 * Ce thread :
 *  - lit l'accéléromètre à haute fréquence
 *  - applique le filtrage / détection d'impact
 *  - lorsqu'un impact est détecté, il notifie le RT middleware
 *    via mpu_adapter_notify_impact().
 *
 * Le thread doit recevoir en argument un pointeur vers MpuAdapter.
 */

#include <pthread.h>
#include "rt/mpu_adapter.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Fonction thread principale.
 *
 * arg doit être un pointeur vers :
 *      MpuAdapter*
 *
 * Exemple d'utilisation dans main_rt :
 *
 *      MpuAdapter mpu;
 *      mpu_adapter_init(&mpu);
 *
 *      pthread_t tid;
 *      pthread_create(&tid, NULL, mpu_thread, &mpu);
 */
void* mpu_thread(void* arg);

#ifdef __cplusplus
}
#endif