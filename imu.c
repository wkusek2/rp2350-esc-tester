#include "imu.h"
#include "pico/stdlib.h"
#include <math.h>
#include <string.h>

// rejestry LSM6DSO32
#define REG_WHO_AM_I   0x0F
#define REG_CTRL1_XL   0x10
#define REG_CTRL2_G    0x11
#define REG_CTRL3_C    0x12
#define REG_CTRL4_C    0x13
#define REG_CTRL6_C    0x15
#define REG_CTRL8_XL   0x17
#define REG_INT1_CTRL  0x0D
#define REG_OUT_TEMP_L 0x20
#define WHO_AM_I_VAL   0x6C

#define GYRO_SENS  (0.0175f)      // deg/s / LSB przy +-500 dps
#define ACC_SENS   (0.000122f)    // g / LSB przy +-4 g
#define I2C_TO_US  2000

#define MAHONY_KP  0.8f
#define MAHONY_KI  0.01f

static bool wr(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t b[2] = { reg, val };
    return i2c_write_timeout_us(i2c, addr, b, 2, false, I2C_TO_US) == 2;
}
static bool rd(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t *dst, int n) {
    if (i2c_write_timeout_us(i2c, addr, &reg, 1, true, I2C_TO_US) != 1) return false;
    return i2c_read_timeout_us(i2c, addr, dst, (size_t)n, false, I2C_TO_US) == n;
}

bool imu_init(imu_t *imu, i2c_inst_t *i2c, uint8_t addr) {
    memset(imu, 0, sizeof *imu);
    imu->addr = addr; imu->q0 = 1.0f;
    imu->map[0] = 0; imu->map[1] = 1; imu->map[2] = 2; imu->sign[0] = imu->sign[1] = imu->sign[2] = 1;
    uint8_t who = 0;
    if (!rd(i2c, addr, REG_WHO_AM_I, &who, 1) || who != WHO_AM_I_VAL) return false;
    bool ok = true;
    ok &= wr(i2c, addr, REG_CTRL3_C, 0x44);     // BDU + IF_INC (autoinkrementacja adresu)
    ok &= wr(i2c, addr, REG_CTRL1_XL, 0x52);    // 208 Hz, +-4 g, LPF2 wlaczony
    ok &= wr(i2c, addr, REG_CTRL2_G, 0x54);     // 208 Hz, +-500 dps
    ok &= wr(i2c, addr, REG_CTRL4_C, 0x02);     // LPF1 zyro wlaczony
    ok &= wr(i2c, addr, REG_CTRL6_C, 0x03);     // FTYPE: LPF1 zyro ~ 30 Hz przy 208 Hz ODR
    ok &= wr(i2c, addr, REG_CTRL8_XL, 0x40);    // HPCF_XL: LPF2 akcel ODR/20 (~10 Hz)
    ok &= wr(i2c, addr, REG_INT1_CTRL, 0x02);   // INT1 = DRDY zyro
    imu->present = ok;
    return ok;
}

static int axis_idx(const char *s, int8_t *sign) {
    *sign = 1;
    if (*s == '-') { *sign = -1; s++; } else if (*s == '+') s++;
    switch (*s) { case 'x': case 'X': return 0; case 'y': case 'Y': return 1; case 'z': case 'Z': return 2; }
    return -1;
}
bool imu_set_axes(imu_t *imu, const char *fwd, const char *left, const char *up) {
    int8_t s[3]; int m[3];
    m[0] = axis_idx(fwd, &s[0]); m[1] = axis_idx(left, &s[1]); m[2] = axis_idx(up, &s[2]);
    if (m[0] < 0 || m[1] < 0 || m[2] < 0 || m[0] == m[1] || m[1] == m[2] || m[0] == m[2]) return false;
    for (int i = 0; i < 3; i++) { imu->map[i] = (uint8_t)m[i]; imu->sign[i] = s[i]; }
    imu->q0 = 1; imu->q1 = imu->q2 = imu->q3 = 0;
    return true;
}

void imu_start_cal(imu_t *imu) { imu->calibrating = true; imu->cal_n = 0; imu->cal_acc[0] = imu->cal_acc[1] = imu->cal_acc[2] = 0; }

static void mahony(imu_t *imu, float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    static float ix, iy, iz;   // calka bledu
    float q0 = imu->q0, q1 = imu->q1, q2 = imu->q2, q3 = imu->q3;
    float n = sqrtf(ax * ax + ay * ay + az * az);
    if (n > 0.5f && n < 1.5f) {        // uzyj akcelerometru tylko gdy ~1 g (brak duzych przyspieszen)
        ax /= n; ay /= n; az /= n;
        // kierunek grawitacji z kwaternionu (z = gora -> grawitacja +z w ukladzie pojazdu w spoczynku)
        float vx = 2.0f * (q1 * q3 - q0 * q2);
        float vy = 2.0f * (q0 * q1 + q2 * q3);
        float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
        float ex = ay * vz - az * vy, ey = az * vx - ax * vz, ez = ax * vy - ay * vx;
        ix += MAHONY_KI * ex * dt; iy += MAHONY_KI * ey * dt; iz += MAHONY_KI * ez * dt;
        gx += MAHONY_KP * ex + ix; gy += MAHONY_KP * ey + iy; gz += MAHONY_KP * ez + iz;
    }
    gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;
    float qa = q0, qb = q1, qc = q2;
    q0 += -qb * gx - qc * gy - q3 * gz;
    q1 += qa * gx + qc * gz - q3 * gy;
    q2 += qa * gy - qb * gz + q3 * gx;
    q3 += qa * gz + qb * gy - qc * gx;
    n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    imu->q0 = q0 / n; imu->q1 = q1 / n; imu->q2 = q2 / n; imu->q3 = q3 / n;
    const float R2D = 57.29578f;
    imu->roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * R2D;
    float sp = 2.0f * (q0 * q2 - q3 * q1); if (sp > 1) sp = 1; if (sp < -1) sp = -1;
    imu->pitch = asinf(sp) * R2D;
    imu->yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * R2D;
}

bool imu_read(imu_t *imu, i2c_inst_t *i2c, float dt_s) {
    uint8_t b[14];
    if (!rd(i2c, imu->addr, REG_OUT_TEMP_L, b, 14)) { imu->n_err++; return false; }
    int16_t t  = (int16_t)(b[0] | (b[1] << 8));
    int16_t gr[3], ar[3];
    for (int i = 0; i < 3; i++) { gr[i] = (int16_t)(b[2 + 2 * i] | (b[3 + 2 * i] << 8)); ar[i] = (int16_t)(b[8 + 2 * i] | (b[9 + 2 * i] << 8)); }
    imu->temp_C = 25.0f + (float)t / 256.0f;
    float gs[3], as[3];   // w osiach czujnika
    for (int i = 0; i < 3; i++) { gs[i] = (float)gr[i] * GYRO_SENS; as[i] = (float)ar[i] * ACC_SENS; }
    if (imu->calibrating) {
        for (int i = 0; i < 3; i++) imu->cal_acc[i] += gs[i];
        if (++imu->cal_n >= 400) {
            for (int i = 0; i < 3; i++) imu->gbias[i] = imu->cal_acc[i] / (float)imu->cal_n;
            imu->calibrating = false;
        }
    }
    for (int i = 0; i < 3; i++) gs[i] -= imu->gbias[i];
    float g[3], a[3];     // w osiach pojazdu
    for (int i = 0; i < 3; i++) { g[i] = imu->sign[i] * gs[imu->map[i]]; a[i] = imu->sign[i] * as[imu->map[i]]; }
    imu->gx = g[0]; imu->gy = g[1]; imu->gz = g[2]; imu->ax = a[0]; imu->ay = a[1]; imu->az = a[2];
    const float D2R = 0.017453293f;
    if (dt_s > 0 && dt_s < 0.1f) mahony(imu, g[0] * D2R, g[1] * D2R, g[2] * D2R, a[0], a[1], a[2], dt_s);
    imu->n_samples++;
    imu->last_ms = to_ms_since_boot(get_absolute_time());
    return true;
}
