#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"
// LSM6DSO32 (Adafruit) na I2C + fuzja Mahony (6-DoF: roll, pitch, yaw wzgledny)

typedef struct {
    bool  present;
    uint8_t addr;
    // ostatnia probka w ukladzie POJAZDU (x = przod, y = lewo, z = gora), po remapie osi
    float gx, gy, gz;        // deg/s
    float ax, ay, az;        // g
    float temp_C;
    // orientacja
    float roll, pitch, yaw;  // deg
    float q0, q1, q2, q3;
    // kalibracja
    float gbias[3];          // deg/s, w osiach czujnika
    bool  calibrating; int cal_n; float cal_acc[3];
    uint32_t n_samples, n_err;
    uint32_t last_ms;
    // remap: body[i] = sign[i] * sensor[map[i]]
    uint8_t map[3]; int8_t sign[3];
} imu_t;

bool imu_init(imu_t *imu, i2c_inst_t *i2c, uint8_t addr);       // true gdy WHO_AM_I ok
bool imu_read(imu_t *imu, i2c_inst_t *i2c, float dt_s);         // odczyt + krok filtra; false = blad I2C
void imu_start_cal(imu_t *imu);                                 // zbiera 400 probek (2 s @ 208 Hz) biasu zyro
// mapowanie osi: s = "x", "-x", "y", "-y", "z", "-z" dla kolejno: przod, lewo, gora
bool imu_set_axes(imu_t *imu, const char *fwd, const char *left, const char *up);
