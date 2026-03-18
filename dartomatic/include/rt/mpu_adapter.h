#pragma once

/*
 * mpu_adapter
 * -----------
 * But :
 *  - fournir un mécanisme propre pour remonter "impact détecté" depuis le thread MPU
 *    vers la boucle principale RT (qui fait select()).
 *
 * On utilise un pipe() :
 *  - le thread MPU écrit 1 octet dans le pipe (notify)
 *  - main_rt surveille le fd en lecture dans select()
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int rfd; /* read end */
    int wfd; /* write end */
} MpuAdapter;

/* Initialise le pipe (rfd/wfd) en non-bloquant */
int  mpu_adapter_init(MpuAdapter* a);

/* Ferme les fds */
void mpu_adapter_close(MpuAdapter* a);

/* Retourne le fd à surveiller dans select() */
int  mpu_adapter_fd(const MpuAdapter* a);

/* À appeler depuis le thread MPU quand un impact est détecté */
int  mpu_adapter_notify_impact(MpuAdapter* a);

#ifdef __cplusplus
}
#endif