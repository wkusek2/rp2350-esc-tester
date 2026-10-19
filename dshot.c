#include "dshot.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "dshot_bidir.pio.h"
#include <string.h>

static PIO s_pio[N_MOTORS];
static uint s_sm[N_MOTORS];
static int  s_dma[N_MOTORS];
static uint32_t s_bitrate;
static uint s_offset[2];
static uint8_t s_pins[N_MOTORS];
static uint32_t s_buf[N_MOTORS][DSHOT_WORDS] __attribute__((aligned(4)));

// GCR 5 bit -> 4 bity (-1 = niepoprawny symbol)
static const int8_t GCR_TABLE[32] = {
    -1,-1,-1,-1,-1,-1,-1,-1,-1, 9,10,11,-1,13,14,15,
    -1,-1, 2, 3,-1, 5, 6, 7,-1, 0, 8, 1,-1, 4,12,-1 };

uint16_t dshot_frame(uint16_t value, bool telem) {
    uint16_t p = (uint16_t)((value & 0x7FF) << 1) | (telem ? 1 : 0);
    uint16_t crc = (~(p ^ (p >> 4) ^ (p >> 8))) & 0x0F;   // bidir: CRC odwrocone
    return (uint16_t)((p << 4) | crc);
}

void dshot_init(const uint8_t pins[N_MOTORS], uint32_t bitrate) {
    s_bitrate = bitrate;
    PIO pios[2] = { pio0, pio1 };
    memcpy(s_pins, pins, N_MOTORS);
    for (int p = 0; p < 2; p++) s_offset[p] = pio_add_program(pios[p], &dshot_bidir_program);
    for (int m = 0; m < N_MOTORS; m++) {
        PIO pio = pios[m / 4];
        uint sm = (uint)(m % 4);
        pio_sm_claim(pio, sm);
        s_pio[m] = pio; s_sm[m] = sm;
        dshot_bidir_program_init(pio, sm, s_offset[m / 4], pins[m], bitrate);
        int ch = dma_claim_unused_channel(true);
        s_dma[m] = ch;
        dma_channel_config c = dma_channel_get_default_config(ch);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, false);
        channel_config_set_write_increment(&c, true);
        channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));
        dma_channel_configure(ch, &c, s_buf[m], &pio->rxf[sm], DSHOT_WORDS, false);
    }
}

// zatrzymuje PIO i oddaje piny jako GPIO (wejscie z pull-up) - np. dla trybu 4-way
void dshot_suspend(void) {
    for (int m = 0; m < N_MOTORS; m++) {
        dma_channel_abort(s_dma[m]);
        pio_sm_set_enabled(s_pio[m], s_sm[m], false);
        gpio_set_function(s_pins[m], GPIO_FUNC_SIO);
        gpio_put(s_pins[m], 1); gpio_set_dir(s_pins[m], GPIO_IN); gpio_pull_up(s_pins[m]);
    }
}
void dshot_resume(void) {
    for (int m = 0; m < N_MOTORS; m++)
        dshot_bidir_program_init(s_pio[m], s_sm[m], s_offset[m / 4], s_pins[m], s_bitrate);
}

void dshot_set_rate(uint32_t bitrate) {
    s_bitrate = bitrate;
    for (int m = 0; m < N_MOTORS; m++) {
        pio_sm_set_clkdiv(s_pio[m], s_sm[m], (float)clock_get_hz(clk_sys) / (float)(bitrate * 40));
        pio_sm_clkdiv_restart(s_pio[m], s_sm[m]);
    }
}

static inline int sample(const uint32_t *w, int i) { return (int)((w[i >> 5] >> (31 - (i & 31))) & 1u); }

// zwraca: 1 = ok (*out16 = 12 bit danych << 4 | crc), 0 = brak odpowiedzi, -1 = blad CRC/GCR
static int decode(const uint32_t *w, uint16_t *out16) {
    if (sample(w, 0) != 0) return 0;              // linia caly czas high = timeout
    uint32_t raw = 0; int nbits = 0;
    int level = 0, run = 0;
    for (int i = 0; i < DSHOT_WORDS * 32; i++) {
        int s = sample(w, i);
        if (s == level) { run++; continue; }
        int bits = (run + 4) >> 3;                 // 8 probek na bit GCR
        for (int b = 0; b < bits && nbits < 21; b++) { raw = (raw << 1) | (uint32_t)level; nbits++; }
        level = s; run = 1;
        if (nbits >= 21) break;
    }
    if (level == 0 && nbits < 21) return -1;       // ramka urwana w stanie low
    while (nbits < 21) { raw = (raw << 1) | 1u; nbits++; }   // koncowe jedynki = idle
    uint32_t gcr = (raw ^ (raw >> 1)) & 0xFFFFF;
    uint16_t v = 0;
    for (int k = 0; k < 4; k++) {
        int8_t d = GCR_TABLE[(gcr >> (15 - 5 * k)) & 0x1F];
        if (d < 0) return -1;
        v = (uint16_t)((v << 4) | (uint8_t)d);
    }
    uint16_t cs = v ^ (v >> 8); cs ^= cs >> 4;
    if ((cs & 0xF) != 0xF) return -1;
    *out16 = v;
    return 1;
}

void dshot_send_all(const uint16_t value[N_MOTORS], const bool telem[N_MOTORS], bool edt_on, dshot_tlm_t tlm[N_MOTORS]) {
    for (int m = 0; m < N_MOTORS; m++) {
        dma_channel_abort(s_dma[m]);
        pio_sm_clear_fifos(s_pio[m], s_sm[m]);
        dma_channel_set_write_addr(s_dma[m], s_buf[m], true);
    }
    for (int m = 0; m < N_MOTORS; m++)
        pio_sm_put(s_pio[m], s_sm[m], (uint32_t)dshot_frame(value[m], telem[m]) << 16);
    // TX 640 + timeout ~2100 + probkowanie 768 cykli PIO = ~3500 cykli (+margines)
    uint32_t us = (uint32_t)(4000ull * 1000000ull / ((uint64_t)s_bitrate * 40ull)) + 20;
    sleep_us(us);
    uint32_t now = to_ms_since_boot(get_absolute_time());
    for (int m = 0; m < N_MOTORS; m++) {
        if (dma_channel_is_busy(s_dma[m])) { tlm[m].n_none++; continue; }
        uint16_t v16;
        int r = decode(s_buf[m], &v16);
        if (r == 0) { tlm[m].n_none++; continue; }
        if (r < 0)  { tlm[m].n_crc++;  continue; }
        tlm[m].n_ok++; tlm[m].last_ok_ms = now;
        uint16_t v = v16 >> 4;
        if (edt_on && (v & 0x100) == 0) {          // EDT: bit 8 = 0 -> ramka rozszerzona
            uint8_t val = (uint8_t)(v & 0xFF);
            switch (v & 0xF00) {
                case 0x200: tlm[m].edt_temp = val; break;
                case 0x400: tlm[m].edt_volt_cV = (uint16_t)val * 25; break;   // 0.25 V
                case 0x600: tlm[m].edt_curr_A = val; break;
                case 0xC00: tlm[m].edt_stress = val; break;
                case 0xE00: tlm[m].edt_status = val; break;
                default: break;   // 0x800/0xA00 debug, inne - ignoruj
            }
            continue;
        }
        if (v == 0x0FFF) { tlm[m].erpm = 0; continue; }
        uint32_t period_us = (uint32_t)(v & 0x1FF) << (v >> 9);
        tlm[m].erpm = period_us ? (60000000u + period_us / 2) / period_us : 0;
    }
}
