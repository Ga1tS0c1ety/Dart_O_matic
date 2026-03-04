#include "rt/mpu_adapter.h"

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

/* Met un fd en non-bloquant */
static int set_nonblock(int fd) {
    int fl = fcntl(fd, F_GETFL, 0);
    if (fl < 0) return -1;
    return fcntl(fd, F_SETFL, fl | O_NONBLOCK);
}

int mpu_adapter_init(MpuAdapter* a) {
    if (!a) return -1;

    int fds[2];
    if (pipe(fds) != 0) return -1;

    a->rfd = fds[0];
    a->wfd = fds[1];

    /* Important : non-bloquant côté thread MPU et côté main loop */
    set_nonblock(a->rfd);
    set_nonblock(a->wfd);

    return 0;
}

void mpu_adapter_close(MpuAdapter* a) {
    if (!a) return;
    if (a->rfd >= 0) close(a->rfd);
    if (a->wfd >= 0) close(a->wfd);
    a->rfd = -1;
    a->wfd = -1;
}

int mpu_adapter_fd(const MpuAdapter* a) {
    return a ? a->rfd : -1;
}

int mpu_adapter_notify_impact(MpuAdapter* a) {
    if (!a) return -1;

    /*
     * On écrit 1 octet.
     * Si le pipe est plein (EAGAIN), on ignore : ça évite de bloquer le thread MPU
     * et ça agit comme un "anti-rebond" naturel (des impacts en attente existent déjà).
     */
    unsigned char b = 1;
    ssize_t w = write(a->wfd, &b, 1);
    if (w == 1) return 0;

    if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        return 0; /* on drop l'event, mais sans bloquer */
    }
    return -1;
}