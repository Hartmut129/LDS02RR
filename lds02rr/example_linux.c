/* Linux example: read an LDS02RR via a UART / USB-UART adapter.
 *
 * Output is the same as the Arduino sketch (lds02rr_serial.ino), so the
 * Python viewer works with both:
 *     S,<rpm>,d0,d1,...,d359      one line per revolution, mm, 0 = invalid
 *     # ...                       status every 2 s
 *
 * Motor: either powered separately, or driven by a sysfs PWM channel
 * (e.g. Raspberry Pi with dtoverlay=pwm) through a MOSFET. With -p the
 * driver regulates the speed to 300 RPM (above ~350 RPM the lidar skips
 * frames because 115200 baud cannot carry them all). 500 Hz PWM, because
 * ~31 kHz caused checksum errors from switching noise on the Arduino rig.
 *
 * Build: gcc -O2 -Wall -o lds02rr_linux example_linux.c lds02rr.c
 * Run:   ./lds02rr_linux /dev/ttyUSB0
 *        ./lds02rr_linux -p 0:0 /dev/ttyAMA0          # PWM on pwmchip0/pwm0
 *        ./lds02rr_linux /dev/ttyUSB0 | python3 ../python/lidar_view.py --port -
 */
#define _DEFAULT_SOURCE
#include "lds02rr.h"
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define PWM_PERIOD_NS  2000000u     /* 500 Hz */
#define DEFAULT_RPM    300.0f

static volatile sig_atomic_t stop;
static int pwm_chip = -1, pwm_ch = -1;
static char pwm_dir[96];

static void on_signal(int sig) { (void)sig; stop = 1; }

static uint32_t millis(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

/* ---------- sysfs PWM ---------- */

static int write_sysfs(const char *path, unsigned long val)
{
    char buf[24];
    int fd = open(path, O_WRONLY), ok;
    if (fd < 0)
        return -1;
    snprintf(buf, sizeof(buf), "%lu", val);
    ok = write(fd, buf, strlen(buf)) > 0;
    close(fd);
    return ok ? 0 : -1;
}

static int pwm_attr(const char *attr, unsigned long val)
{
    char path[128];
    snprintf(path, sizeof(path), "%s/%s", pwm_dir, attr);
    return write_sysfs(path, val);
}

static int pwm_open(void)
{
    char path[96];
    int i;

    snprintf(pwm_dir, sizeof(pwm_dir), "/sys/class/pwm/pwmchip%d/pwm%d",
             pwm_chip, pwm_ch);
    if (access(pwm_dir, F_OK) != 0) {
        snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/export", pwm_chip);
        if (write_sysfs(path, (unsigned long)pwm_ch) != 0 && errno != EBUSY) {
            perror(path);
            return -1;
        }
    }
    /* udev may need a moment to create the files / fix permissions */
    for (i = 0; i < 50; i++) {
        if (pwm_attr("duty_cycle", 0) == 0)
            break;
        usleep(20000);
    }
    if (pwm_attr("period", PWM_PERIOD_NS) != 0 || pwm_attr("enable", 1) != 0) {
        fprintf(stderr, "cannot configure %s: %s\n", pwm_dir, strerror(errno));
        return -1;
    }
    return 0;
}

static void pwm_close(void)
{
    pwm_attr("duty_cycle", 0);
    pwm_attr("enable", 0);
}

static void set_motor(void *user, float duty)
{
    (void)user;
    pwm_attr("duty_cycle", (unsigned long)(duty * PWM_PERIOD_NS));
}

/* ---------- serial ---------- */

static int open_serial(const char *dev)
{
    struct termios tio;
    int fd = open(dev, O_RDONLY | O_NOCTTY);
    if (fd < 0) {
        perror(dev);
        return -1;
    }
    if (tcgetattr(fd, &tio) != 0) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }
    cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;      /* ignore modem lines, enable RX */
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;                /* 100 ms read timeout */
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }
    tcflush(fd, TCIFLUSH);
    return fd;
}

/* ---------- output ---------- */

static int summary;

static void on_scan(void *user, const lds02rr_point_t *scan, float rpm)
{
    int a, n = 0;
    (void)user;

    if (summary) {
        for (a = 0; a < 360; a++)
            n += scan[a].valid;
        printf("rpm %3d  valid %3d/360  front %4u  left %4u  back %4u  right %4u mm\n",
               (int)(rpm + 0.5f), n, scan[0].distance_mm, scan[90].distance_mm,
               scan[180].distance_mm, scan[270].distance_mm);
        return;
    }
    printf("S,%.1f", rpm);
    for (a = 0; a < 360; a++)
        printf(",%u", scan[a].valid ? scan[a].distance_mm : 0);
    putchar('\n');
}

static void usage(const char *prog)
{
    fprintf(stderr,
            "usage: %s [-p CHIP:CHANNEL] [-r RPM] [-s] DEVICE\n"
            "  -p  drive the motor with /sys/class/pwm/pwmchipCHIP/pwmCHANNEL\n"
            "  -r  target speed (default %.0f RPM, needs -p)\n"
            "  -s  short summary line per scan instead of all 360 values\n",
            prog, DEFAULT_RPM);
}

int main(int argc, char **argv)
{
    static lds02rr_t lidar;
    lds02rr_config_t cfg;
    float target_rpm = DEFAULT_RPM;
    uint32_t last_status, last_frames = 0;
    uint8_t buf[512];
    int fd, opt;

    while ((opt = getopt(argc, argv, "p:r:sh")) != -1) {
        switch (opt) {
        case 'p':
            if (sscanf(optarg, "%d:%d", &pwm_chip, &pwm_ch) != 2) {
                usage(argv[0]);
                return 1;
            }
            break;
        case 'r': target_rpm = (float)atof(optarg); break;
        case 's': summary = 1; break;
        default:  usage(argv[0]); return 1;
        }
    }
    if (optind != argc - 1) {
        usage(argv[0]);
        return 1;
    }

    fd = open_serial(argv[optind]);
    if (fd < 0)
        return 1;
    if (pwm_chip >= 0 && pwm_open() != 0)
        return 1;

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, on_signal);         /* viewer closed */
    setvbuf(stdout, NULL, _IOLBF, 0);   /* line-buffered also when piped */

    memset(&cfg, 0, sizeof(cfg));
    cfg.on_scan    = on_scan;
    cfg.set_motor  = pwm_chip >= 0 ? set_motor : NULL;
    cfg.target_rpm = target_rpm;
    lds02rr_init(&lidar, &cfg);
    lds02rr_tick(&lidar, millis());     /* driver needs the time before start */
    lds02rr_motor_start(&lidar);

    printf("# LDS02RR started on %s%s\n", argv[optind],
           pwm_chip >= 0 ? ", motor control on" : "");
    last_status = millis();

    while (!stop) {
        ssize_t n = read(fd, buf, sizeof(buf));
        uint32_t now = millis();

        if (n > 0)
            lds02rr_feed(&lidar, buf, (size_t)n);
        else if (n < 0 && errno != EINTR) {
            perror("read");
            break;
        }
        lds02rr_tick(&lidar, now);

        if (now - last_status >= 2000) {
            const lds02rr_stats_t *st = lds02rr_get_stats(&lidar);
            last_status = now;
            if (st->frames_ok == last_frames)
                printf("# no valid lidar frames - check RX wiring, GND, motor\n");
            last_frames = st->frames_ok;
            printf("# rpm %d  pwm %d%%  frames ok %lu  crc err %lu  scans %lu\n",
                   (int)(lds02rr_get_rpm(&lidar) + 0.5f),
                   (int)(lds02rr_get_duty(&lidar) * 100.0f + 0.5f),
                   (unsigned long)st->frames_ok,
                   (unsigned long)st->checksum_errors,
                   (unsigned long)st->scans);
        }
    }

    if (pwm_chip >= 0)
        pwm_close();                    /* never leave the motor running */
    close(fd);
    return 0;
}
