/*
 * lds02rr.h - Generic, platform-independent driver for the LDS02RR 360° lidar
 *
 * The LDS02RR (Xiaomi / Roborock / Dreame vacuum lidar) streams 22-byte
 * frames over UART at 115200 8N1 (3.3 V logic):
 *
 *   byte 0      : 0xFA                      start byte
 *   byte 1      : 0xA0..0xF9                index -> 90 frames per revolution
 *   byte 2..3   : speed, little endian      RPM = value / 64
 *   byte 4..19  : 4 measurements x 4 bytes
 *                   [0..1] distance (LE)   bits 0..13 = distance mm
 *                                          bit 14     = strength warning
 *                                          bit 15     = invalid data
 *                   [2..3] signal strength (LE)
 *   byte 20..21 : checksum, little endian
 *
 * Angle of measurement i (0..3) in frame index n: (n - 0xA0) * 4 + i degrees.
 *
 * The module has no motor controller: the host must drive the motor
 * (typ. 5 V via PWM/MOSFET). This driver includes an optional PI loop that
 * regulates the speed using the RPM reported in the frames.
 *
 * The driver does no I/O itself. Feed it received bytes, and optionally
 * give it a callback to set motor PWM duty. No dynamic allocation.
 */
#ifndef LDS02RR_H
#define LDS02RR_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LDS02RR_BAUDRATE        115200u
#define LDS02RR_FRAME_LEN       22u
#define LDS02RR_START_BYTE      0xFAu
#define LDS02RR_INDEX_MIN       0xA0u
#define LDS02RR_INDEX_MAX       0xF9u
#define LDS02RR_POINTS_PER_SCAN 360u
#define LDS02RR_DEFAULT_RPM     300.0f   /* ~5 Hz scan rate */

typedef struct {
    uint16_t angle_deg;    /* 0..359 */
    uint16_t distance_mm;  /* 0 if invalid */
    uint16_t strength;     /* raw signal strength */
    uint8_t  valid;        /* 1 = distance usable */
    uint8_t  weak;         /* 1 = strength warning flag set */
} lds02rr_point_t;

/* Called for every decoded measurement (4 per frame). */
typedef void (*lds02rr_point_cb_t)(void *user, const lds02rr_point_t *pt);

/* Called once per full revolution with a 360-entry array indexed by angle.
 * Entries not received during this revolution have valid = 0. */
typedef void (*lds02rr_scan_cb_t)(void *user, const lds02rr_point_t *scan,
                                  float rpm);

/* Set motor PWM duty, 0.0 .. 1.0. */
typedef void (*lds02rr_motor_cb_t)(void *user, float duty);

/* Called for every frame that fails the checksum (diagnostics). */
typedef void (*lds02rr_bad_frame_cb_t)(void *user, const uint8_t *frame,
                                       uint16_t calc_crc);

typedef struct {
    lds02rr_point_cb_t on_point;   /* optional */
    lds02rr_scan_cb_t  on_scan;    /* optional */
    lds02rr_bad_frame_cb_t on_bad_frame; /* optional */
    lds02rr_motor_cb_t set_motor;  /* optional: NULL = no speed control */
    void              *user;       /* passed to all callbacks */

    float target_rpm;              /* 0 -> LDS02RR_DEFAULT_RPM */
    float kp, ki;                  /* 0,0 -> defaults */
    float duty_start;              /* initial / kick-start duty, 0 -> 0.6 */
    float duty_min, duty_max;      /* 0,0 -> 0.2 .. 1.0 */
    uint32_t stall_timeout_ms;     /* 0 -> 500 ms */
} lds02rr_config_t;

typedef struct {
    uint32_t frames_ok;
    uint32_t checksum_errors;
    uint32_t index_errors;
    uint32_t scans;
} lds02rr_stats_t;

typedef struct {
    lds02rr_config_t cfg;
    lds02rr_stats_t  stats;

    /* parser */
    uint8_t  frame[LDS02RR_FRAME_LEN];
    uint8_t  pos;
    int16_t  last_index;           /* -1 = none yet */

    /* scan assembly */
    lds02rr_point_t scan[LDS02RR_POINTS_PER_SCAN];
    float    rpm;                  /* last reported speed */

    /* motor control */
    uint8_t  motor_on;
    float    duty;
    float    integ;
    uint32_t last_frame_ms;
    uint32_t now_ms;
} lds02rr_t;

/* Initialise the driver. cfg may be NULL for defaults. */
void lds02rr_init(lds02rr_t *dev, const lds02rr_config_t *cfg);

/* Feed received UART bytes (any chunk size, from ISR/DMA/poll loop). */
void lds02rr_feed(lds02rr_t *dev, const uint8_t *data, size_t len);
void lds02rr_feed_byte(lds02rr_t *dev, uint8_t b);

/* Motor control (no-ops when cfg.set_motor is NULL). */
void lds02rr_motor_start(lds02rr_t *dev);
void lds02rr_motor_stop(lds02rr_t *dev);
void lds02rr_set_target_rpm(lds02rr_t *dev, float rpm);

/* Call periodically (e.g. every 10..100 ms) with a monotonic millisecond
 * clock. Detects a stalled motor / missing data and kick-starts it. */
void lds02rr_tick(lds02rr_t *dev, uint32_t now_ms);

/* Accessors */
float lds02rr_get_rpm(const lds02rr_t *dev);
float lds02rr_get_duty(const lds02rr_t *dev);
const lds02rr_stats_t *lds02rr_get_stats(const lds02rr_t *dev);

/* Exposed for testing / custom parsers: checksum of a 22-byte frame. */
uint16_t lds02rr_checksum(const uint8_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* LDS02RR_H */
