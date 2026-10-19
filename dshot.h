#pragma once
#include <stdint.h>
#include <stdbool.h>

#define N_MOTORS 8
#define DSHOT_WORDS 6   // 192 probek odpowiedzi

typedef struct {
    uint32_t erpm;        // ostatnie poprawne eRPM (0 = stoi / brak)
    uint8_t  edt_temp;    // EDT: temperatura [C]
    uint16_t edt_volt_cV; // EDT: napiecie [0.01 V]
    uint8_t  edt_curr_A;  // EDT: prad [A]
    uint8_t  edt_status;  // EDT: status
    uint8_t  edt_stress;
    uint32_t n_ok, n_crc, n_none;   // liczniki ramek bidir
    uint32_t last_ok_ms;  // czas ostatniej poprawnej ramki
} dshot_tlm_t;

void dshot_init(const uint8_t pins[N_MOTORS], uint32_t bitrate);
void dshot_set_rate(uint32_t bitrate);
void dshot_suspend(void);
void dshot_resume(void);
// wysyla 8 ramek rownolegle, czeka na odpowiedzi i dekoduje
void dshot_send_all(const uint16_t value[N_MOTORS], const bool telem[N_MOTORS], bool edt_on, dshot_tlm_t tlm[N_MOTORS]);
uint16_t dshot_frame(uint16_t value, bool telem);
