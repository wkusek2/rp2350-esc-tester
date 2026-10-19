#include "kiss.h"
#include "pico/stdlib.h"
#include <string.h>

static uint8_t crc8(const uint8_t *d, int n) {
    uint8_t crc = 0;
    for (int i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

bool kiss_feed(kiss_parser_t *p, uint8_t b, kiss_tlm_t *out) {
    if (p->len >= sizeof(p->buf)) { memmove(p->buf, p->buf + 1, p->len - 1); p->len--; }
    p->buf[p->len++] = b;
    while (p->len >= 10) {
        if (crc8(p->buf, 9) == p->buf[9]) {
            const uint8_t *f = p->buf;
            out->temp_C  = (int8_t)f[0];
            out->volt_cV = (uint16_t)(f[1] << 8 | f[2]);
            out->curr_cA = (uint16_t)(f[3] << 8 | f[4]);
            out->mAh     = (uint16_t)(f[5] << 8 | f[6]);
            out->erpm    = (uint32_t)(f[7] << 8 | f[8]) * 100;
            out->t_ms    = to_ms_since_boot(get_absolute_time());
            out->valid   = true;
            out->n_ok++; p->n_ok++;
            memmove(p->buf, p->buf + 10, p->len - 10); p->len -= 10;
            return true;
        }
        p->n_crc++;
        memmove(p->buf, p->buf + 1, p->len - 1); p->len--;
    }
    return false;
}
