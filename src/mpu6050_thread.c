// mpu6050_thread_adapted.c
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <pthread.h>

/* ================= CONFIG ================= */

#define MPU6050_ADDR       0x68
#define PWR_MGMT_1         0x6B
#define ACCEL_XOUT_H       0x3B

#define FS_HZ              2000
#define DT                 (1.0 / FS_HZ)

#define HP_WINDOW          150
#define ENERGY_WINDOW      25
#define IMPACT_THRESHOLD   1.0f
#define MIN_CONSECUTIVE    3
#define LOCKOUT_MS         100
#define STARTUP_IGNORE_MS  200

/* ================= STRUCTURES ================= */

typedef struct {
    float ax, ay, az;
    float norm;
} AccelSample;

typedef struct {
    float buffer[HP_WINDOW];
    int index;
    float sum;
} MovingAverage;

/* ================= TEMPS ================= */

static inline double now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

/* ================= MPU6050 ================= */

static int mpu6050_init(const char *bus)
{
    int fd = open(bus, O_RDWR);
    if (fd < 0) { perror("[MPU] I2C open"); return -1; }
    if (ioctl(fd, I2C_SLAVE, MPU6050_ADDR) < 0) { perror("[MPU] Not found"); close(fd); return -1; }
    char cfg[2] = { PWR_MGMT_1, 0x00 };
    write(fd, cfg, 2);
    return fd;
}

static AccelSample mpu6050_read(int fd)
{
    char data[6];
    write(fd, (char[]){ACCEL_XOUT_H}, 1);
    read(fd, data, 6);

    int16_t ax = (data[0] << 8) | data[1];
    int16_t ay = (data[2] << 8) | data[3];
    int16_t az = (data[4] << 8) | data[5];

    AccelSample s;
    s.ax = ax / 16384.0f;
    s.ay = ay / 16384.0f;
    s.az = az / 16384.0f;
    s.norm = sqrtf(s.ax*s.ax + s.ay*s.ay + s.az*s.az);
    return s;
}

/* ================= SIGNAL PROCESSING ================= */

static void ma_init(MovingAverage *ma)
{
    ma->index = 0;
    ma->sum = 0.0f;
    for (int i = 0; i < HP_WINDOW; i++)
        ma->buffer[i] = 0.0f;
}

static float ma_update(MovingAverage *ma, float x)
{
    ma->sum -= ma->buffer[ma->index];
    ma->buffer[ma->index] = x;
    ma->sum += x;
    ma->index = (ma->index + 1) % HP_WINDOW;
    return ma->sum / HP_WINDOW;
}

/* ================= MPU THREAD ================= */

void* mpu_thread(void* arg)
{
    (void)arg;

    pid_t pid = getpid();

    int fd = mpu6050_init("/dev/i2c-1");
    if (fd < 0) return NULL;

    MovingAverage ma;
    ma_init(&ma);

    float energy_buf[ENERGY_WINDOW] = {0};
    int e_idx = 0;

    float prev_norm = 0.0f;
    int above_cnt = 0;

    double t0 = now_ms();
    double last_impact = 0.0;
    int armed = 0;

    FILE *log = fopen("impact_log.csv", "w");
    fprintf(log, "t_ms,ax,ay,az,norm,dnorm,energy,impact\n");

    printf("[MPU] Initialisation...\n");
    fflush(stdout);

    while (1) {
        AccelSample s = mpu6050_read(fd);

        /* high-pass via moyenne glissante */
        float mean = ma_update(&ma, s.norm);
        float ahp = s.norm - mean;

        /* variation de norme */
        float dnorm = fabsf(s.norm - prev_norm);
        prev_norm = s.norm;

        /* energie courte */
        energy_buf[e_idx] = dnorm * dnorm;
        e_idx = (e_idx + 1) % ENERGY_WINDOW;

        float energy = 0.0f;
        for (int i = 0; i < ENERGY_WINDOW; i++)
            energy += energy_buf[i];

        double t = now_ms() - t0;

        /* ================= WARM-UP ================= */
        if (!armed) {
            if (t >= STARTUP_IGNORE_MS) {
                armed = 1;
                last_impact = t;
                printf("[MPU] Stabilisation OK, system armed\n");
                fflush(stdout);
            }
            usleep(1000000 / FS_HZ);
            continue;
        }

        /* ================= DETECTION IMPACT ================= */
        int impact = 0;

        if (energy > IMPACT_THRESHOLD)
            above_cnt++;
        else
            above_cnt = 0;

        if (above_cnt >= MIN_CONSECUTIVE &&
            (t - last_impact) > LOCKOUT_MS)
        {
            impact = 1;
            last_impact = t;
            above_cnt = 0;

            printf("[MPU] IMPACT detected (energy=%.3f)\n", energy);
            fflush(stdout);
            kill(pid, SIGUSR1);
        }

        /* ================= LOG ================= */
        fprintf(log, "%.3f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%d\n",
                t, s.ax, s.ay, s.az, s.norm, dnorm, energy, impact);

        usleep(1000000 / FS_HZ);
    }

    fclose(log);
    close(fd);
    return NULL;
}
