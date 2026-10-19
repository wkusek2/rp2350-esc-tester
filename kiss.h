#pragma once
#include <stdint.h>
#include <stdbool.h>
// Telemetria KISS (BLHeli_32 / AM32): 10 bajtow, 115200 8N1
typedef struct {
    int8_t   temp_C;
    uint16_t volt_cV;     // 0.01 V
    uint16_t curr_cA;     // 0.01 A
    uint16_t mAh;
    uint32_t erpm;        // ramka niesie eRPM/100
    uint32_t t_ms;        // czas odbioru
    uint32_t n_ok;
    bool     valid;
} kiss_tlm_t;

typedef struct {
    uint8_t buf[32];
    uint8_t len;
    uint32_t n_ok, n_crc;
} kiss_parser_t;

// podaje bajt; zwraca true gdy zdekodowano pelna ramke do *out
bool kiss_feed(kiss_parser_t *p, uint8_t b, kiss_tlm_t *out);
