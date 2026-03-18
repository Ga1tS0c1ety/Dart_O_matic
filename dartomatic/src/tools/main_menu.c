#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <unistd.h>
#include <sys/wait.h>
#include <errno.h>

/*
 * main_menu
 * ---------
 * Lanceur opérateur de haut niveau.
 *
 * Idée :
 *   - le script run_ui.sh démarre toute l'infra
 *   - puis lance main_menu
 *   - main_menu permet de lancer les outils existants
 *
 * Outils supportés :
 *   - ui_cli
 *   - calib_intrinsic
 *   - calib_extrinsic_aruco
 *   - debug_camera
 *   - debug_dart_detector
 *   - debug_triangulated_view
 *
 * Hypothèse :
 *   - main_menu est lancé depuis build/, ou au moins son chemin argv[0]
 *     permet de retrouver le dossier contenant les autres exécutables.
 */

static void trim_newline(char* s)
{
    if (!s) return;
    size_t n = strlen(s);
    while (n > 0 && (s[n - 1] == '\n' || s[n - 1] == '\r')) {
        s[n - 1] = '\0';
        n--;
    }
}

static int read_line(char* buf, size_t sz)
{
    if (!fgets(buf, (int)sz, stdin)) return 0;
    trim_newline(buf);
    return 1;
}

static int ask_int(const char* prompt, int* out_value)
{
    char buf[128];

    printf("%s", prompt);
    fflush(stdout);

    if (!read_line(buf, sizeof(buf))) return 0;
    if (buf[0] == '\0') return 0;

    char* end = NULL;
    long v = strtol(buf, &end, 10);
    if (end == buf || *end != '\0') return 0;

    *out_value = (int)v;
    return 1;
}

static int ask_double(const char* prompt, double* out_value)
{
    char buf[128];

    printf("%s", prompt);
    fflush(stdout);

    if (!read_line(buf, sizeof(buf))) return 0;
    if (buf[0] == '\0') return 0;

    char* end = NULL;
    double v = strtod(buf, &end);
    if (end == buf || *end != '\0') return 0;

    *out_value = v;
    return 1;
}

static int get_bin_dir(const char* argv0, char* out_dir, size_t out_sz)
{
    if (!argv0 || !out_dir || out_sz == 0) return -1;

    /*
     * Cas simple :
     *   argv0 = ./build/main_menu
     *   on coupe au dernier '/'
     */
    const char* slash = strrchr(argv0, '/');
    if (slash) {
        size_t len = (size_t)(slash - argv0);
        if (len >= out_sz) return -1;
        memcpy(out_dir, argv0, len);
        out_dir[len] = '\0';
        return 0;
    }

    /*
     * Si argv0 ne contient pas '/', on suppose qu'on est déjà dans build
     * ou que l'exécutable est accessible par PATH.
     * On choisit "." comme base.
     */
    snprintf(out_dir, out_sz, ".");
    return 0;
}

static int make_exe_path(char* out, size_t out_sz, const char* bin_dir, const char* exe_name)
{
    if (!out || !bin_dir || !exe_name) return -1;
    int n = snprintf(out, out_sz, "%s/%s", bin_dir, exe_name);
    if (n < 0 || (size_t)n >= out_sz) return -1;
    return 0;
}

static int run_child(char* const argv[])
{
    pid_t pid = fork();
    if (pid < 0) {
        perror("[main_menu] fork");
        return -1;
    }

    if (pid == 0) {
        execv(argv[0], argv);
        perror("[main_menu] execv");
        _exit(127);
    }

    int status = 0;
    if (waitpid(pid, &status, 0) < 0) {
        perror("[main_menu] waitpid");
        return -1;
    }

    if (WIFEXITED(status)) {
        int code = WEXITSTATUS(status);
        printf("\n[main_menu] processus terminé avec code %d\n", code);
        return code;
    }

    if (WIFSIGNALED(status)) {
        int sig = WTERMSIG(status);
        printf("\n[main_menu] processus terminé par signal %d\n", sig);
        return 128 + sig;
    }

    return 0;
}

static void press_enter_to_continue(void)
{
    char buf[8];
    printf("[main_menu] Entrée pour continuer...");
    fflush(stdout);
    (void)read_line(buf, sizeof(buf));
}

static void print_main_menu(void)
{
    printf("\n");
    printf("==================================================\n");
    printf(" Dart'O'Matic - Main Menu\n");
    printf("==================================================\n");
    printf(" 1. Jouer\n");
    printf(" 2. Calibration\n");
    printf(" 3. Debug\n");
    printf(" q. Quitter\n");
    printf("--------------------------------------------------\n");
    printf("> ");
    fflush(stdout);
}

static void print_calib_menu(void)
{
    printf("\n");
    printf("==================================================\n");
    printf(" Calibration\n");
    printf("==================================================\n");
    printf(" 1. Calibration intrinseque\n");
    printf(" 2. Calibration extrinseque ArUco\n");
    printf(" b. Retour\n");
    printf("--------------------------------------------------\n");
    printf("> ");
    fflush(stdout);
}

static void print_debug_menu(void)
{
    printf("\n");
    printf("==================================================\n");
    printf(" Debug\n");
    printf("==================================================\n");
    printf(" 1. Debug camera\n");
    printf(" 2. Debug dart detector\n");
    printf(" 3. Debug triangulated view (ecoute evt)\n");
    printf(" b. Retour\n");
    printf("--------------------------------------------------\n");
    printf("> ");
    fflush(stdout);
}

static void launch_ui_cli(const char* bin_dir, const char* sock_path)
{
    char exe[PATH_MAX];
    if (make_exe_path(exe, sizeof(exe), bin_dir, "ui_cli") != 0) {
        fprintf(stderr, "[main_menu] chemin ui_cli trop long\n");
        return;
    }

    char* argv[] = {
        exe,
        (char*)sock_path,
        NULL
    };

    printf("[main_menu] lancement ui_cli...\n");
    fflush(stdout);
    (void)run_child(argv);
}

static void launch_calib_intrinsic(const char* bin_dir)
{
    int cam_id = 0;
    if (!ask_int("camera_id ? ", &cam_id)) {
        printf("[main_menu] saisie invalide\n");
        return;
    }

    char exe[PATH_MAX];
    char cam_id_str[32];

    if (make_exe_path(exe, sizeof(exe), bin_dir, "calib_intrinsic") != 0) {
        fprintf(stderr, "[main_menu] chemin calib_intrinsic trop long\n");
        return;
    }

    snprintf(cam_id_str, sizeof(cam_id_str), "%d", cam_id);

    char* argv[] = {
        exe,
        cam_id_str,
        NULL
    };

    printf("[main_menu] lancement calib_intrinsic cam=%d...\n", cam_id);
    fflush(stdout);
    (void)run_child(argv);
}

static void launch_calib_extrinsic(const char* bin_dir)
{
    int cam_id = 0;
    double marker_len = 0.18;
    double threshold = 0.5;

    if (!ask_int("camera_id ? ", &cam_id)) {
        printf("[main_menu] saisie invalide\n");
        return;
    }

    if (!ask_double("marker_length_m ? (ex: 0.18) ", &marker_len)) {
        printf("[main_menu] valeur invalide\n");
        return;
    }

    if (!ask_double("reproj_threshold_px ? (ex: 0.5) ", &threshold)) {
        printf("[main_menu] valeur invalide\n");
        return;
    }

    char exe[PATH_MAX];
    char calib_yaml[PATH_MAX];
    char output_yaml[PATH_MAX];
    char cam_id_str[32];
    char marker_str[64];
    char threshold_str[64];

    if (make_exe_path(exe, sizeof(exe), bin_dir, "calib_extrinsic_aruco") != 0) {
        fprintf(stderr, "[main_menu] chemin calib_extrinsic_aruco trop long\n");
        return;
    }

    snprintf(cam_id_str, sizeof(cam_id_str), "%d", cam_id);
    snprintf(marker_str, sizeof(marker_str), "%.6f", marker_len);
    snprintf(threshold_str, sizeof(threshold_str), "%.6f", threshold);

    snprintf(calib_yaml, sizeof(calib_yaml),
             "data/cam_param/camera_params_%d.yaml", cam_id);

    snprintf(output_yaml, sizeof(output_yaml),
             "data/cam_param/camera_extrinsics_%d.yaml", cam_id);

    char* argv[] = {
        exe,
        cam_id_str,
        calib_yaml,
        output_yaml,
        marker_str,
        threshold_str,
        NULL
    };

    printf("[main_menu] lancement calib_extrinsic_aruco cam=%d...\n", cam_id);
    fflush(stdout);
    (void)run_child(argv);
}

static void launch_debug_camera(const char* bin_dir)
{
    int cam_id = 0;
    if (!ask_int("camera_id ? ", &cam_id)) {
        printf("[main_menu] saisie invalide\n");
        return;
    }

    char exe[PATH_MAX];
    char cam_id_str[32];

    if (make_exe_path(exe, sizeof(exe), bin_dir, "debug_camera") != 0) {
        fprintf(stderr, "[main_menu] chemin debug_camera trop long\n");
        return;
    }

    snprintf(cam_id_str, sizeof(cam_id_str), "%d", cam_id);

    char* argv[] = {
        exe,
        cam_id_str,
        NULL
    };

    printf("[main_menu] lancement debug_camera cam=%d...\n", cam_id);
    fflush(stdout);
    (void)run_child(argv);
}

static void launch_debug_dart_detector(const char* bin_dir)
{
    int cam_id = 0;
    if (!ask_int("camera_id ? ", &cam_id)) {
        printf("[main_menu] saisie invalide\n");
        return;
    }

    char exe[PATH_MAX];
    char cam_id_str[32];

    if (make_exe_path(exe, sizeof(exe), bin_dir, "debug_dart_detector") != 0) {
        fprintf(stderr, "[main_menu] chemin debug_dart_detector trop long\n");
        return;
    }

    snprintf(cam_id_str, sizeof(cam_id_str), "%d", cam_id);

    char* argv[] = {
        exe,
        cam_id_str,
        NULL
    };

    printf("[main_menu] lancement debug_dart_detector cam=%d...\n", cam_id);
    fflush(stdout);
    (void)run_child(argv);
}

static void launch_debug_triangulated_view(const char* bin_dir, const char* sock_path)
{
    char exe[PATH_MAX];

    if (make_exe_path(exe, sizeof(exe), bin_dir, "debug_triangulated_view") != 0) {
        fprintf(stderr, "[main_menu] chemin debug_triangulated_view trop long\n");
        return;
    }

    char* argv[] = {
        exe,
        (char*)sock_path,
        NULL
    };

    printf("[main_menu] lancement debug_triangulated_view...\n");
    fflush(stdout);
    (void)run_child(argv);
}

static void calibration_menu_loop(const char* bin_dir)
{
    char choice[32];

    while (1) {
        print_calib_menu();

        if (!read_line(choice, sizeof(choice))) return;

        if (strcmp(choice, "1") == 0) {
            launch_calib_intrinsic(bin_dir);
            press_enter_to_continue();
        }
        else if (strcmp(choice, "2") == 0) {
            launch_calib_extrinsic(bin_dir);
            press_enter_to_continue();
        }
        else if (strcmp(choice, "b") == 0 || strcmp(choice, "B") == 0) {
            return;
        }
        else {
            printf("[main_menu] choix inconnu\n");
        }
    }
}

static void debug_menu_loop(const char* bin_dir, const char* sock_path)
{
    char choice[32];

    while (1) {
        print_debug_menu();

        if (!read_line(choice, sizeof(choice))) return;

        if (strcmp(choice, "1") == 0) {
            launch_debug_camera(bin_dir);
            press_enter_to_continue();
        }
        else if (strcmp(choice, "2") == 0) {
            launch_debug_dart_detector(bin_dir);
            press_enter_to_continue();
        }
        else if (strcmp(choice, "3") == 0) {
            launch_debug_triangulated_view(bin_dir, sock_path);
            press_enter_to_continue();
        }
        else if (strcmp(choice, "b") == 0 || strcmp(choice, "B") == 0) {
            return;
        }
        else {
            printf("[main_menu] choix inconnu\n");
        }
    }
}

int main(int argc, char** argv)
{
    const char* sock_path = (argc >= 2) ? argv[1] : "/tmp/dart_appbus.sock";

    char bin_dir[PATH_MAX];
    if (get_bin_dir(argv[0], bin_dir, sizeof(bin_dir)) != 0) {
        fprintf(stderr, "[main_menu] impossible de déterminer le dossier bin\n");
        return 1;
    }

    printf("[main_menu] bin_dir=%s\n", bin_dir);
    printf("[main_menu] sock=%s\n", sock_path);

    char choice[32];

    while (1) {
        print_main_menu();

        if (!read_line(choice, sizeof(choice))) break;

        if (strcmp(choice, "1") == 0) {
            launch_ui_cli(bin_dir, sock_path);
            press_enter_to_continue();
        }
        else if (strcmp(choice, "2") == 0) {
            calibration_menu_loop(bin_dir);
        }
        else if (strcmp(choice, "3") == 0) {
            debug_menu_loop(bin_dir, sock_path);
        }
        else if (strcmp(choice, "q") == 0 || strcmp(choice, "Q") == 0) {
            printf("[main_menu] bye\n");
            break;
        }
        else {
            printf("[main_menu] choix inconnu\n");
        }
    }

    return 0;
}