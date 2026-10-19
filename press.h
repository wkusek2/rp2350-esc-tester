#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"
// Czujniki cisnienia TE MS5837-02BA (0x76) i MS5803-05BA (0x77) na jednej magistrali I2C,
// odczyt nieblokujacy (maszyna stanow), port z main.py (MicroPython) z projektu czujnika.

typedef enum { PS_MS5837_02BA = 0, PS_MS5803_05BA = 1 } press_type_t;

typedef struct {
    press_type_t type;
    uint8_t addr;
    const char *name;
    bool  present;
    uint16_t C[8];
    uint32_t D1, D2;
    float p_mbar, t_C;
    float p0_mbar;            // odniesienie (0 m)
    uint32_t n_ok, n_err;
    uint32_t last_ms;
    int fails;
} press_t;

typedef struct {
    press_t s[2];
    i2c_inst_t *i2c;
    int state; absolute_time_t t_next; int cycle;
    float rho;                // gestosc wody kg/m3 (997 slodka, 1025 morska)
} press_bus_t;

void  press_init(press_bus_t *pb, i2c_inst_t *i2c);      // skan + PROM + CRC; brak czujnika = present=false
void  press_poll(press_bus_t *pb);                        // wolac czesto z petli glownej (nieblokujace)
void  press_zero(press_bus_t *pb);                        // ustaw p0 = aktualne cisnienie (powierzchnia)
float press_depth_m(const press_bus_t *pb, int i);        // glebokosc z czujnika i
