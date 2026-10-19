#include "press.h"
#include "pico/stdlib.h"
#include <string.h>

#define CMD_RESET     0x1E
#define CMD_ADC_READ  0x00
#define CMD_PROM_READ 0xA0
#define CMD_D1_1024   0x44     // OSR 1024
#define CMD_D2_1024   0x54
#define CONV_US_1024  2500
#define TEMP_EVERY    20
#define I2C_TO_US     2000
#define GRAVITY       9.80665f

static bool wr1(i2c_inst_t *i2c, uint8_t addr, uint8_t cmd) { return i2c_write_timeout_us(i2c, addr, &cmd, 1, false, I2C_TO_US) == 1; }
static bool rdn(i2c_inst_t *i2c, uint8_t addr, uint8_t cmd, uint8_t *d, int n) {
    if (i2c_write_timeout_us(i2c, addr, &cmd, 1, true, I2C_TO_US) != 1) return false;
    return i2c_read_timeout_us(i2c, addr, d, (size_t)n, false, I2C_TO_US) == n;
}

// CRC-4 wg AN520; prom8: 8 slow z juz wyzerowanymi bitami CRC
static uint8_t crc4(const uint16_t *prom8) {
    uint16_t n_rem = 0;
    for (int i = 0; i < 16; i++) {
        n_rem ^= (i & 1) ? (prom8[i >> 1] & 0x00FF) : (prom8[i >> 1] >> 8);
        for (int b = 0; b < 8; b++) n_rem = (n_rem & 0x8000) ? (uint16_t)((n_rem << 1) ^ 0x3000) : (uint16_t)(n_rem << 1);
    }
    return (n_rem >> 12) & 0x0F;
}

static bool read_prom(press_bus_t *pb, press_t *s) {
    int words = (s->type == PS_MS5837_02BA) ? 7 : 8;
    for (int i = 0; i < words; i++) {
        uint8_t d[2];
        if (!rdn(pb->i2c, s->addr, (uint8_t)(CMD_PROM_READ + i * 2), d, 2)) return false;
        s->C[i] = (uint16_t)((d[0] << 8) | d[1]);
    }
    uint16_t prom[8]; memcpy(prom, s->C, sizeof prom);
    uint8_t crc_read;
    if (s->type == PS_MS5837_02BA) { prom[7] = 0; crc_read = (uint8_t)((prom[0] & 0xF000) >> 12); prom[0] &= 0x0FFF; }
    else                           { crc_read = (uint8_t)(prom[7] & 0x000F); prom[7] &= 0xFF00; }
    return crc4(prom) == crc_read;
}

static void compensate(press_t *s) {
    const uint16_t *C = s->C; int64_t D1 = s->D1, D2 = s->D2;
    int64_t dT = D2 - (int64_t)C[5] * 256;
    int64_t TEMP = 2000 + dT * C[6] / 8388608;
    int64_t OFF, SENS, P;
    if (s->type == PS_MS5837_02BA) {
        OFF  = (int64_t)C[2] * 131072 + (C[4] * dT) / 64;
        SENS = (int64_t)C[1] * 65536 + (C[3] * dT) / 128;
        int64_t Ti = 0, OFFi = 0, SENSi = 0;
        if (TEMP < 2000) { Ti = 11 * dT * dT / 34359738368LL; OFFi = 31 * (TEMP - 2000) * (TEMP - 2000) / 8; SENSi = 63 * (TEMP - 2000) * (TEMP - 2000) / 32; }
        OFF -= OFFi; SENS -= SENSi; TEMP -= Ti;
    } else {
        OFF  = (int64_t)C[2] * 262144 + (C[4] * dT) / 32;
        SENS = (int64_t)C[1] * 131072 + (C[3] * dT) / 128;
        int64_t T2 = 0, OFF2 = 0, SENS2 = 0;
        if (TEMP < 2000) {
            T2 = 3 * dT * dT / 8589934592LL; OFF2 = 3 * (TEMP - 2000) * (TEMP - 2000) / 8; SENS2 = 7 * (TEMP - 2000) * (TEMP - 2000) / 8;
            if (TEMP < -1500) SENS2 += 3 * (TEMP + 1500) * (TEMP + 1500);
        }
        TEMP -= T2; OFF -= OFF2; SENS -= SENS2;
    }
    P = (D1 * SENS / 2097152 - OFF) / 32768;
    s->p_mbar = (float)P / 100.0f; s->t_C = (float)TEMP / 100.0f;
}

static bool init_one(press_bus_t *pb, press_t *s) {
    s->present = false;
    if (!wr1(pb->i2c, s->addr, CMD_RESET)) return false;
    sleep_ms(10);
    if (!read_prom(pb, s)) return false;
    s->present = true; s->fails = 0;
    return true;
}

void press_init(press_bus_t *pb, i2c_inst_t *i2c) {
    memset(pb, 0, sizeof *pb);
    pb->i2c = i2c; pb->rho = 997.0f;
    pb->s[0].type = PS_MS5837_02BA; pb->s[0].addr = 0x76; pb->s[0].name = "MS5837-02BA";
    pb->s[1].type = PS_MS5803_05BA; pb->s[1].addr = 0x77; pb->s[1].name = "MS5803-05BA";
    for (int i = 0; i < 2; i++) init_one(pb, &pb->s[i]);
    pb->state = 0; pb->t_next = get_absolute_time();
}

// stany: 0 start D2 (co TEMP_EVERY cykli) lub D1, 1 czekaj D2, 2 odczyt D2 + start D1, 3 czekaj D1, 4 odczyt D1
void press_poll(press_bus_t *pb) {
    if (absolute_time_diff_us(get_absolute_time(), pb->t_next) > 0) return;
    switch (pb->state) {
    case 0: {
        bool need_temp = (pb->cycle % TEMP_EVERY) == 0;
        for (int i = 0; i < 2; i++) if (pb->s[i].present) wr1(pb->i2c, pb->s[i].addr, need_temp ? CMD_D2_1024 : CMD_D1_1024);
        pb->t_next = make_timeout_time_us(CONV_US_1024);
        pb->state = need_temp ? 2 : 4;
        break;
    }
    case 2: {   // odczyt D2, start D1
        for (int i = 0; i < 2; i++) {
            press_t *s = &pb->s[i]; if (!s->present) continue;
            uint8_t d[3];
            if (rdn(pb->i2c, s->addr, CMD_ADC_READ, d, 3)) { uint32_t v = ((uint32_t)d[0] << 16) | (d[1] << 8) | d[2]; if (v) s->D2 = v; }
            wr1(pb->i2c, s->addr, CMD_D1_1024);
        }
        pb->t_next = make_timeout_time_us(CONV_US_1024);
        pb->state = 4;
        break;
    }
    case 4: {   // odczyt D1, kompensacja
        for (int i = 0; i < 2; i++) {
            press_t *s = &pb->s[i]; if (!s->present) continue;
            uint8_t d[3]; uint32_t v = 0;
            if (rdn(pb->i2c, s->addr, CMD_ADC_READ, d, 3)) v = ((uint32_t)d[0] << 16) | (d[1] << 8) | d[2];
            if (v && s->D2) { s->D1 = v; compensate(s); s->n_ok++; s->fails = 0; s->last_ms = to_ms_since_boot(get_absolute_time()); }
            else if (++s->fails >= 5) { s->n_err++; init_one(pb, s); }
        }
        pb->cycle++;
        pb->t_next = make_timeout_time_us(1000);
        pb->state = 0;
        break;
    }
    default: pb->state = 0;
    }
}

void press_zero(press_bus_t *pb) { for (int i = 0; i < 2; i++) if (pb->s[i].present && pb->s[i].n_ok) pb->s[i].p0_mbar = pb->s[i].p_mbar; }

float press_depth_m(const press_bus_t *pb, int i) {
    const press_t *s = &pb->s[i];
    if (!s->present || s->p0_mbar == 0) return 0;
    return (s->p_mbar - s->p0_mbar) * 100.0f / (pb->rho * GRAVITY);
}
