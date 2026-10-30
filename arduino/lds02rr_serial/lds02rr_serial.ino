/*
 * lds02rr_serial.ino - Read an LDS02RR lidar on Serial1, print scans on Serial (USB)
 *
 * Board: Arduino Mega 2560.
 * Wiring: lidar TX -> RX1 (pin 19), common GND,
 *         motor driven from D2 (PWM) through a logic-level MOSFET/transistor
 *         (do not power the motor directly from the pin).
 * The driver regulates the motor to TARGET_RPM using the speed the lidar
 * reports. 300 RPM is the nominal speed; above ~350 RPM the lidar skips
 * frames because 115200 baud cannot carry them all.
 * Serial Monitor: 500000 baud.
 *
 * Output: one line per revolution, 1° resolution:
 *     S,<rpm>,d0,d1,...,d359      distances in mm, 0 = no valid reading
 * Status/diagnostic lines start with '#' so a parser can skip them.
 *
 * Set OUTPUT_MODE to OUTPUT_SUMMARY for a short human-readable line instead.
 */
#include "lds02rr.h"

#define OUTPUT_SUMMARY 0
#define OUTPUT_FULL    1
#ifndef OUTPUT_MODE
#define OUTPUT_MODE    OUTPUT_FULL
#endif

#define USB_BAUD       500000   /* exact on 16 MHz AVR (921600 is ~8.5% off) */

#define MOTOR_PWM_PIN  2        /* Mega D2 = Timer3 (OC3B) */
#define TARGET_RPM     300.0f
#ifndef PWM_31KHZ
#define PWM_31KHZ      0        /* 0: default ~490 Hz (measured: 0 crc errors)
                                   1: ~31 kHz, inaudible, but caused crc
                                      errors (switching noise) on the test rig.
                                   Affects all Timer3 pins: D2, D3, D5 */
#endif

/* Print the first N frames that fail the checksum as hex (0 = off) */
#define DEBUG_BAD_FRAMES 5

static lds02rr_t lidar;
static uint32_t last_status_ms;

/* Copy of the last completed scan, printed from loop() a bit at a time so
 * Serial1 keeps being drained while we print. */
static uint16_t out_mm[LDS02RR_POINTS_PER_SCAN];
static float    out_rpm;
static uint16_t out_valid;
static bool     out_ready;
static uint16_t out_pos;
static uint32_t scans_dropped;

static void on_scan(void *user, const lds02rr_point_t *scan, float rpm)
{
    (void)user;
    if (out_ready) {            /* still printing the previous one */
        scans_dropped++;
        return;
    }
    out_valid = 0;
    for (uint16_t a = 0; a < LDS02RR_POINTS_PER_SCAN; a++) {
        out_mm[a] = scan[a].valid ? scan[a].distance_mm : 0;
        out_valid += scan[a].valid;
    }
    out_rpm = rpm;
    out_pos = 0;
    out_ready = true;
}

#if DEBUG_BAD_FRAMES
static uint8_t bad_dumped;

static void on_bad_frame(void *user, const uint8_t *f, uint16_t calc)
{
    (void)user;
    if (bad_dumped >= DEBUG_BAD_FRAMES)
        return;
    bad_dumped++;
    char hex[4];
    Serial.print("# bad frame:");
    for (uint8_t i = 0; i < LDS02RR_FRAME_LEN; i++) {
        snprintf(hex, sizeof(hex), " %02X", f[i]);
        Serial.print(hex);
    }
    Serial.print("  calc crc ");
    Serial.println(calc, HEX);
}
#endif

static void set_motor(void *user, float duty)
{
    (void)user;
    analogWrite(MOTOR_PWM_PIN, (int)(duty * 255.0f + 0.5f));
}

static void poll_lidar()
{
    uint8_t buf[64];
    int n;
    while ((n = Serial1.available()) > 0) {
        if (n > (int)sizeof(buf)) n = sizeof(buf);
        n = Serial1.readBytes(buf, n);
        lds02rr_feed(&lidar, buf, (size_t)n);
    }
}

#if OUTPUT_MODE == OUTPUT_FULL
/* Prints a few values per call; returns true when the line is done. */
static bool print_step()
{
    if (out_pos == 0) {
        Serial.print("S,");
        Serial.print(out_rpm, 1);
    }
    for (uint8_t i = 0; i < 16 && out_pos < LDS02RR_POINTS_PER_SCAN; i++) {
        Serial.print(',');
        Serial.print(out_mm[out_pos++]);
    }
    if (out_pos < LDS02RR_POINTS_PER_SCAN)
        return false;
    Serial.println();
    return true;
}
#else
static bool print_step()
{
    char line[96];
    snprintf(line, sizeof(line),
             "rpm %3d  valid %3u/360  front %4u  left %4u  back %4u  right %4u mm",
             (int)(out_rpm + 0.5f), out_valid,
             out_mm[0], out_mm[90], out_mm[180], out_mm[270]);
    Serial.println(line);
    return true;
}
#endif

static void print_status()
{
    const lds02rr_stats_t *st = lds02rr_get_stats(&lidar);
    char line[96];
    snprintf(line, sizeof(line),
             "# rpm %d  pwm %d%%  frames ok %lu  crc err %lu  scans %lu  dropped %lu",
             (int)(lds02rr_get_rpm(&lidar) + 0.5f),
             (int)(lds02rr_get_duty(&lidar) * 100.0f + 0.5f),
             (unsigned long)st->frames_ok, (unsigned long)st->checksum_errors,
             (unsigned long)st->scans, (unsigned long)scans_dropped);
    Serial.println(line);
}

void setup()
{
    Serial.begin(USB_BAUD);

    pinMode(MOTOR_PWM_PIN, OUTPUT);
#if PWM_31KHZ
    TCCR3B = (TCCR3B & 0xF8) | 0x01;   /* Timer3 prescaler 1 -> 31.4 kHz */
#endif

    /* 115200 on a 16 MHz Mega is really 117647 (+2.1%); measured to work
     * with ~98% good frames, the neighbouring AVR rates get none. */
    Serial1.begin(LDS02RR_BAUDRATE);

    lds02rr_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.on_scan    = on_scan;
    cfg.set_motor  = set_motor;
    cfg.target_rpm = TARGET_RPM;
#if DEBUG_BAD_FRAMES
    cfg.on_bad_frame = on_bad_frame;
#endif
    lds02rr_init(&lidar, &cfg);
    lds02rr_tick(&lidar, millis());     /* give the driver the current time */
    lds02rr_motor_start(&lidar);

    Serial.println("# LDS02RR started");
    last_status_ms = millis();
}

void loop()
{
    static uint32_t last_frames;

    poll_lidar();
    lds02rr_tick(&lidar, millis());     /* stall detection / restart */

    if (out_ready && print_step())
        out_ready = false;

    /* Status every 2 s; doubles as a "no data" warning */
    uint32_t now = millis();
    if (!out_ready && now - last_status_ms >= 2000) {
        last_status_ms = now;
        uint32_t frames = lds02rr_get_stats(&lidar)->frames_ok;
        if (frames == last_frames)
            Serial.println("# no valid lidar frames - check RX wiring, GND, motor");
        last_frames = frames;
        print_status();
    }
}
