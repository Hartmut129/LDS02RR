/*
 * lds02rr.c - Generic, platform-independent driver for the LDS02RR lidar
 */
#include "lds02rr.h"
#include <string.h>

#define DEF_KP        0.0005f
#define DEF_KI        0.0010f
#define DEF_DUTY      0.6f
#define DEF_DUTY_MIN  0.2f
#define DEF_DUTY_MAX  1.0f
#define DEF_STALL_MS  500u

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static void motor_apply(lds02rr_t *dev, float duty)
{
    dev->duty = duty;
    if (dev->cfg.set_motor)
        dev->cfg.set_motor(dev->cfg.user, duty);
}

static void clear_scan(lds02rr_t *dev)
{
    uint16_t a;
    memset(dev->scan, 0, sizeof(dev->scan));
    for (a = 0; a < LDS02RR_POINTS_PER_SCAN; a++)
        dev->scan[a].angle_deg = a;
}

void lds02rr_init(lds02rr_t *dev, const lds02rr_config_t *cfg)
{
    memset(dev, 0, sizeof(*dev));
    if (cfg)
        dev->cfg = *cfg;

    if (dev->cfg.target_rpm <= 0.0f)  dev->cfg.target_rpm = LDS02RR_DEFAULT_RPM;
    if (dev->cfg.kp == 0.0f && dev->cfg.ki == 0.0f) {
        dev->cfg.kp = DEF_KP;
        dev->cfg.ki = DEF_KI;
    }
    if (dev->cfg.duty_start <= 0.0f)  dev->cfg.duty_start = DEF_DUTY;
    if (dev->cfg.duty_min == 0.0f && dev->cfg.duty_max == 0.0f) {
        dev->cfg.duty_min = DEF_DUTY_MIN;
        dev->cfg.duty_max = DEF_DUTY_MAX;
    }
    if (dev->cfg.stall_timeout_ms == 0) dev->cfg.stall_timeout_ms = DEF_STALL_MS;

    dev->last_index = -1;
    clear_scan(dev);
}

uint16_t lds02rr_checksum(const uint8_t *frame)
{
    uint32_t chk = 0;
    uint8_t i;
    for (i = 0; i < 10; i++)
        chk = (chk << 1) + rd_u16(&frame[2 * i]);
    chk = (chk & 0x7FFFu) + (chk >> 15);
    return (uint16_t)(chk & 0x7FFFu);
}

/* PI speed loop, run once per revolution (~5 Hz) */
static void motor_regulate(lds02rr_t *dev)
{
    float err, duty;
    if (!dev->motor_on || !dev->cfg.set_motor)
        return;

    err = dev->cfg.target_rpm - dev->rpm;
    dev->integ += err;
    /* anti-windup: keep integral term within the duty range */
    dev->integ = clampf(dev->integ, -1.0f / dev->cfg.ki, 1.0f / dev->cfg.ki);

    duty = dev->cfg.duty_start + dev->cfg.kp * err + dev->cfg.ki * dev->integ;
    motor_apply(dev, clampf(duty, dev->cfg.duty_min, dev->cfg.duty_max));
}

static void finish_scan(lds02rr_t *dev)
{
    dev->stats.scans++;
    if (dev->cfg.on_scan)
        dev->cfg.on_scan(dev->cfg.user, dev->scan, dev->rpm);
    motor_regulate(dev);
    clear_scan(dev);
}

/* Returns 0 if the frame failed the checksum. */
static int process_frame(lds02rr_t *dev)
{
    const uint8_t *f = dev->frame;
    uint8_t index = f[1];
    uint16_t base, crc;
    uint8_t i;

    crc = lds02rr_checksum(f);
    if (crc != rd_u16(&f[20])) {
        dev->stats.checksum_errors++;
        if (dev->cfg.on_bad_frame)
            dev->cfg.on_bad_frame(dev->cfg.user, f, crc);
        return 0;
    }
    dev->stats.frames_ok++;
    dev->last_frame_ms = dev->now_ms;
    dev->rpm = (float)rd_u16(&f[2]) / 64.0f;

    /* Index jumped back by more than half a turn -> wrapped around, previous
     * revolution is complete. Repeated or slightly out-of-order frames (the
     * sensor re-sends after a truncated frame) and gaps (it skips frames when
     * spinning too fast for 115200 baud) must not end a scan. */
    if (dev->last_index - (int16_t)index > 45)
        finish_scan(dev);
    dev->last_index = index;

    base = (uint16_t)(index - LDS02RR_INDEX_MIN) * 4u;
    for (i = 0; i < 4; i++) {
        const uint8_t *m = &f[4 + 4 * i];
        uint16_t raw = rd_u16(m);
        lds02rr_point_t *pt = &dev->scan[base + i];

        pt->angle_deg   = base + i;
        pt->valid       = (raw & 0x8000u) ? 0 : 1;
        pt->weak        = (raw & 0x4000u) ? 1 : 0;
        pt->distance_mm = pt->valid ? (raw & 0x3FFFu) : 0;
        pt->strength    = rd_u16(m + 2);

        if (dev->cfg.on_point)
            dev->cfg.on_point(dev->cfg.user, pt);
    }
    return 1;
}

void lds02rr_feed_byte(lds02rr_t *dev, uint8_t b)
{
    if (dev->pos == 0) {
        if (b != LDS02RR_START_BYTE)
            return;                         /* hunt for start byte */
    } else if (dev->pos == 1) {
        if (b < LDS02RR_INDEX_MIN || b > LDS02RR_INDEX_MAX) {
            dev->stats.index_errors++;
            dev->pos = (b == LDS02RR_START_BYTE) ? 1 : 0;  /* resync */
            return;
        }
    }

    dev->frame[dev->pos++] = b;
    if (dev->pos == LDS02RR_FRAME_LEN) {
        dev->pos = 0;
        if (!process_frame(dev)) {
            /* We may have locked onto a 0xFA inside the data. Re-parse the
             * bytes after the false start so a real frame start in there
             * is not lost. */
            uint8_t tmp[LDS02RR_FRAME_LEN - 1];
            memcpy(tmp, &dev->frame[1], sizeof(tmp));
            lds02rr_feed(dev, tmp, sizeof(tmp));
        }
    }
}

void lds02rr_feed(lds02rr_t *dev, const uint8_t *data, size_t len)
{
    while (len--)
        lds02rr_feed_byte(dev, *data++);
}

void lds02rr_motor_start(lds02rr_t *dev)
{
    dev->motor_on = 1;
    dev->integ = 0.0f;
    dev->last_frame_ms = dev->now_ms;
    motor_apply(dev, dev->cfg.duty_start);
}

void lds02rr_motor_stop(lds02rr_t *dev)
{
    dev->motor_on = 0;
    dev->integ = 0.0f;
    dev->rpm = 0.0f;
    motor_apply(dev, 0.0f);
}

void lds02rr_set_target_rpm(lds02rr_t *dev, float rpm)
{
    if (rpm > 0.0f)
        dev->cfg.target_rpm = rpm;
}

void lds02rr_tick(lds02rr_t *dev, uint32_t now_ms)
{
    dev->now_ms = now_ms;
    if (!dev->motor_on || !dev->cfg.set_motor)
        return;

    /* No valid frames for a while: motor stalled or not spinning yet.
     * Kick it with full duty, then let the PI loop take over again. */
    if ((uint32_t)(now_ms - dev->last_frame_ms) > dev->cfg.stall_timeout_ms) {
        dev->rpm = 0.0f;
        dev->integ = 0.0f;
        dev->last_frame_ms = now_ms;
        motor_apply(dev, dev->cfg.duty_max);
    }
}

float lds02rr_get_rpm(const lds02rr_t *dev)
{
    return dev->rpm;
}

float lds02rr_get_duty(const lds02rr_t *dev)
{
    return dev->duty;
}

const lds02rr_stats_t *lds02rr_get_stats(const lds02rr_t *dev)
{
    return &dev->stats;
}
