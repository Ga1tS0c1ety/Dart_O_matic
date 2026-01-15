#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

#define CAM_PROCESS_EXEC "./bin/cam_process"
#define MASTER_EXEC "./bin/cam_master"

int main() {
    // Liste des IDs de caméra à lancer (modifie-les toi-même)
    int camera_ids[] = {2, 4, 6 , 8};  
    int cam_count = sizeof(camera_ids) / sizeof(camera_ids[0]);

    pid_t pids[cam_count + 1]; // +1 pour master
    int pid_index = 0;

    // Lancer master_process
    pid_t pid = fork();
    if (pid == 0) {
        execl(MASTER_EXEC, MASTER_EXEC, NULL);
        perror("execl master_process");
        exit(-1);
    } else if (pid > 0) {
        pids[pid_index++] = pid;
    }

    // Lancer cam_process pour chaque caméra
    for (int i = 0; i < cam_count; i++) {
        pid = fork();
        if (pid == 0) {
            char cam_str[8];
            snprintf(cam_str, sizeof(cam_str), "%d", camera_ids[i]);
            execl(CAM_PROCESS_EXEC, CAM_PROCESS_EXEC, cam_str, NULL);
            perror("execl cam_process");
            exit(-1);
        } else if (pid > 0) {
            pids[pid_index++] = pid;
        }
        usleep(800000);

    }

    // Attendre tous les enfants
    for (int i = 0; i < pid_index; i++) {
        int status;
        waitpid(pids[i], &status, 0);
        printf("[SUPERVISEUR] Processus %d terminé avec code %d\n",
               pids[i], WEXITSTATUS(status));
    }

    printf("[SUPERVISEUR] Tous les processus terminés.\n");
    return 0;
}
