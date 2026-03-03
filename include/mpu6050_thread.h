#ifndef MPU6050_THREAD_H
#define MPU6050_THREAD_H

#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

void* mpu_thread(void* arg);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_THREAD_H
