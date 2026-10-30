/* Host-side test: feeds synthetic frames through the driver.
 * Build: gcc -Wall -Wextra -o test_lds02rr test_lds02rr.c lds02rr.c */
#include "lds02rr.h"
#include <stdio.h>
#include <string.h>

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)

static int scans_seen;
static lds02rr_point_t last_scan[360];
static float last_duty = -1.0f;

static void on_scan(void *u, const lds02rr_point_t *scan, float rpm)
{
    (void)u; (void)rpm;
    scans_seen++;
    memcpy(last_scan, scan, sizeof(last_scan));
}

static void set_motor(void *u, float duty) { (void)u; last_duty = duty; }

/* Build a valid frame; distance = angle*10 mm, angle 90 marked invalid. */
static void make_frame(uint8_t *f, uint8_t index, float rpm)
{
    uint16_t spd = (uint16_t)(rpm * 64.0f), chk;
    int i;
    f[0] = LDS02RR_START_BYTE;
    f[1] = index;
    f[2] = spd & 0xFF; f[3] = spd >> 8;
    for (i = 0; i < 4; i++) {
        uint16_t angle = (uint16_t)((index - LDS02RR_INDEX_MIN) * 4 + i);
        uint16_t d = (uint16_t)(angle * 10);
        uint16_t s = 500;
        if (angle == 90) d |= 0x8000;
        f[4 + 4 * i] = d & 0xFF; f[5 + 4 * i] = d >> 8;
        f[6 + 4 * i] = s & 0xFF; f[7 + 4 * i] = s >> 8;
    }
    chk = lds02rr_checksum(f);
    f[20] = chk & 0xFF; f[21] = chk >> 8;
}

static void feed_rev(lds02rr_t *dev, float rpm, int corrupt_index)
{
    uint8_t f[22];
    int idx;
    for (idx = LDS02RR_INDEX_MIN; idx <= LDS02RR_INDEX_MAX; idx++) {
        make_frame(f, (uint8_t)idx, rpm);
        if (idx == corrupt_index) f[10] ^= 0x55;
        lds02rr_feed(dev, f, sizeof(f));
    }
}

int main(void)
{
    static lds02rr_t dev;
    lds02rr_config_t cfg;
    const uint8_t junk[] = { 0x00, 0xFA, 0x12, 0x55, 0xFA, 0xFA };

    memset(&cfg, 0, sizeof(cfg));
    cfg.on_scan = on_scan;
    cfg.set_motor = set_motor;
    lds02rr_init(&dev, &cfg);

    /* Known checksum vector check: checksum must fit in 15 bits */
    {
        uint8_t f[22];
        make_frame(f, 0xA0, 300.0f);
        CHECK(lds02rr_checksum(f) <= 0x7FFF);
    }

    lds02rr_motor_start(&dev);
    CHECK(last_duty > 0.59f && last_duty < 0.61f);

    lds02rr_feed(&dev, junk, sizeof(junk));   /* garbage before data */
    feed_rev(&dev, 250.0f, -1);
    CHECK(scans_seen == 0);                    /* not complete until wrap */
    feed_rev(&dev, 250.0f, 0xB0);              /* 2nd rev, one bad frame */
    CHECK(scans_seen == 1);

    CHECK(last_scan[0].valid && last_scan[0].distance_mm == 0);
    CHECK(last_scan[123].valid && last_scan[123].distance_mm == 1230);
    CHECK(last_scan[359].valid && last_scan[359].distance_mm == 3590);
    CHECK(!last_scan[90].valid && last_scan[90].distance_mm == 0);
    CHECK(last_scan[200].strength == 500);

    CHECK(lds02rr_get_rpm(&dev) > 249.9f && lds02rr_get_rpm(&dev) < 250.1f);
    CHECK(last_duty > 0.6f);                   /* too slow -> speed up */

    feed_rev(&dev, 250.0f, -1);                /* wrap -> 2nd scan */
    CHECK(scans_seen == 2);
    CHECK(!last_scan[64].valid);               /* frame 0xB0 was dropped */
    CHECK(lds02rr_get_stats(&dev)->checksum_errors == 1);
    CHECK(lds02rr_get_stats(&dev)->frames_ok == 3 * 90 - 1);

    /* false start (0xFA + valid index) directly before a real frame:
     * the real frame must still be decoded after resync */
    {
        const uint8_t fake[] = { 0xFA, 0xA5 };
        uint8_t f[22];
        uint32_t ok = lds02rr_get_stats(&dev)->frames_ok;
        make_frame(f, 0xA1, 250.0f);
        lds02rr_feed(&dev, fake, sizeof(fake));
        lds02rr_feed(&dev, f, sizeof(f));
        CHECK(lds02rr_get_stats(&dev)->frames_ok == ok + 1);
        CHECK(lds02rr_get_stats(&dev)->checksum_errors == 2);
    }

    /* repeated index (sensor re-sends) and gaps must not end a scan */
    {
        uint8_t f[22];
        int s = scans_seen;
        make_frame(f, 0xA3, 250.0f); lds02rr_feed(&dev, f, sizeof(f));
        make_frame(f, 0xA2, 250.0f); lds02rr_feed(&dev, f, sizeof(f));
        make_frame(f, 0xA8, 250.0f); lds02rr_feed(&dev, f, sizeof(f));
        CHECK(scans_seen == s);
        make_frame(f, 0xF8, 250.0f); lds02rr_feed(&dev, f, sizeof(f));
        make_frame(f, 0xA1, 250.0f); lds02rr_feed(&dev, f, sizeof(f));
        CHECK(scans_seen == s + 1);
    }

    /* stall detection */
    lds02rr_tick(&dev, 10000);
    CHECK(last_duty == 1.0f);

    lds02rr_motor_stop(&dev);
    CHECK(last_duty == 0.0f);

    printf(fails ? "%d FAILURE(S)\n" : "All tests passed\n", fails);
    return fails != 0;
}
