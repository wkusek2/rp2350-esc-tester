// Test 2x 4-in-1 ESC (BLHeli_32 / AM32) na RP2350: bidirectional DShot (PIO),
// telemetria KISS (UART0/UART1), prad z CUR (ADC). Sterowanie przez USB CDC.
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/stdio_usb.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "dshot.h"
#include "kiss.h"
#include "fourway.h"
#include "imu.h"
#include "press.h"
#include "hardware/i2c.h"

// ---- pinout ----
static const uint8_t MOTOR_PINS[N_MOTORS] = { 2, 3, 4, 5, 10, 11, 12, 13 };
#define TLM0_RX_PIN 1    // UART0 RX  <- ESC #1 TLM
#define TLM1_RX_PIN 9    // UART1 RX  <- ESC #2 TLM
#define CUR0_PIN   26    // ADC0      <- ESC #1 CUR
#define CUR1_PIN   27    // ADC1      <- ESC #2 CUR
#define I2C_SDA_PIN 14   // I2C1 SDA  <- IMU LSM6DSO32 + MS5837 (0x76) + MS5803 (0x77)
#define I2C_SCL_PIN 15   // I2C1 SCL
#define IMU_INT_PIN  6   // LSM6DSO32 INT1 (DRDY zyro)

#define DSHOT_LOOP_US   1000      // 1 kHz
#define TLM_REQ_MS      10        // co ile ms zadanie telemetrii KISS (na ESC, rotacja silnikow)
#define USB_FAILSAFE_MS 1000

// ---- stan wspoldzielony core0 <-> core1 ----
static volatile uint16_t g_throttle[N_MOTORS];   // 0 lub 48..2047
static volatile uint16_t g_cmd_val[N_MOTORS];
static volatile uint8_t  g_cmd_cnt[N_MOTORS];    // ile razy jeszcze wyslac komende
static volatile bool     g_edt = false;
static volatile uint32_t g_rate = 300000;
static volatile uint32_t g_rate_req = 0;
static volatile uint32_t g_loop_cnt = 0;
static volatile int8_t   g_tlm_req_motor[2] = { -1, -1 };  // ostatni silnik z bitem telemetrii (per ESC)
static volatile uint32_t g_tlm_req_ms[2];
static dshot_tlm_t g_tlm[N_MOTORS];
static int bb_scan(uint sda, uint scl, char *out, int outlen);
static volatile bool g_req_4way, g_4way_active;   // tryb 4-way (BLHeliSuite)
static uint32_t g_4way_enter_ms;
static bool g_4way_quiet;                          // tryb wszedl z Suite - zadnych tekstow na porcie                   // kiedy zatrzymano DShot (linie ciche)
#define ESC_QUIET_MS 5000                          // BLHeli_32: tyle ciszy (linia high) zanim wejdzie w bootloader
static volatile int  g_req_probe = -1, g_probe_mode, g_probe_hold;             // bl <m>: diagnostyka bootloadera
static char g_probe_out[2048];
static volatile bool g_probe_done;

// ---- core1: petla DShot ----
static void core1_main(void) {
    dshot_init(MOTOR_PINS, g_rate);
    fourway_init(MOTOR_PINS, N_MOTORS);
    absolute_time_t next = get_absolute_time();
    uint16_t val[N_MOTORS]; bool tel[N_MOTORS];
    uint32_t last_req_ms[2] = { 0, 0 };
    uint8_t rr[2] = { 0, 0 };
    while (true) {
        if (g_req_probe >= 0) {
            dshot_suspend();
            if (g_probe_mode == 99) {   // skan I2C bit-bangiem na pinach silnikow GPIO4/5 (stare okablowanie czujnikow)
                int n = 0; char o[128]; int f;
                for (int k = 0; k < 2; k++) { uint pp = k ? 5 : 4; gpio_set_function(pp, GPIO_FUNC_SIO); gpio_set_dir(pp, GPIO_IN); gpio_set_pulls(pp, false, true); sleep_ms(5);
                    int hi = 0; for (int i = 0; i < 100; i++) { hi += gpio_get(pp); sleep_us(50); } n += snprintf(g_probe_out + n, sizeof g_probe_out - n, "GPIO%u z pull-down: %d%% high; ", pp, hi); }
                f = bb_scan(4, 5, o, sizeof o); n += snprintf(g_probe_out + n, sizeof g_probe_out - n, "\nbit-bang SDA=GPIO4 SCL=GPIO5:%s (%d)\n", f ? o : " nic", f);
                f = bb_scan(5, 4, o, sizeof o); n += snprintf(g_probe_out + n, sizeof g_probe_out - n, "bit-bang SDA=GPIO5 SCL=GPIO4:%s (%d)\n", f ? o : " nic", f);
            } else
            fourway_probe(g_req_probe, g_probe_mode, g_probe_hold, g_probe_out, sizeof g_probe_out);
            dshot_resume();
            g_req_probe = -1; g_probe_done = true;
            next = get_absolute_time();
            continue;
        }
        if (g_req_4way) {                       // BLHeliSuite: oddaj piny, bit-bang na tym rdzeniu
            dshot_suspend();
            g_4way_active = true;
            fourway_task();                     // wraca po cmd_InterfaceExit lub fourway_exit_req
            dshot_resume();
            for (int m = 0; m < N_MOTORS; m++) g_tlm[m].n_ok = g_tlm[m].n_crc = g_tlm[m].n_none = 0;
            g_req_4way = false; g_4way_active = false;
            next = get_absolute_time();
            continue;
        }
        next = delayed_by_us(next, DSHOT_LOOP_US);
        if (g_rate_req) { dshot_set_rate(g_rate_req); g_rate = g_rate_req; g_rate_req = 0; }
        uint32_t now = to_ms_since_boot(get_absolute_time());
        for (int m = 0; m < N_MOTORS; m++) { tel[m] = false; }
        for (int e = 0; e < 2; e++) {
            if (now - last_req_ms[e] >= TLM_REQ_MS) {
                last_req_ms[e] = now;
                int m = e * 4 + rr[e]; rr[e] = (uint8_t)((rr[e] + 1) & 3);
                tel[m] = true; g_tlm_req_motor[e] = (int8_t)m; g_tlm_req_ms[e] = now;
            }
        }
        for (int m = 0; m < N_MOTORS; m++) {
            if (g_cmd_cnt[m]) { val[m] = g_cmd_val[m]; tel[m] = true; g_cmd_cnt[m]--; }
            else val[m] = g_throttle[m];
        }
        dshot_send_all(val, tel, g_edt, g_tlm);
        g_loop_cnt++;
        sleep_until(next);
    }
}

// ---- UART telemetria ----
static volatile uint8_t u_ring[2][256];
static volatile uint8_t u_head[2], u_tail[2];
static volatile uint32_t u_bytes[2];
static void uart_rx_irq(int e) {
    uart_inst_t *u = e ? uart1 : uart0;
    while (uart_is_readable(u)) {
        uint8_t b = uart_getc(u);
        uint8_t h = u_head[e];
        u_ring[e][h] = b; u_head[e] = (uint8_t)(h + 1); u_bytes[e]++;
    }
}
static void uart0_irq(void) { uart_rx_irq(0); }
static void uart1_irq(void) { uart_rx_irq(1); }

static void tlm_uart_init(uart_inst_t *u, uint rx_pin, uint irq, irq_handler_t h) {
    uart_init(u, 115200);
    // RX standardowo na GPIO 1,5,9,13,...; na pinie TX (0,4,8,12,...) uzyj UART_AUX (zamiana TX/RX, tylko RP2350)
    gpio_set_function(rx_pin, (rx_pin % 4 == 1) ? GPIO_FUNC_UART : GPIO_FUNC_UART_AUX);
    uart_set_format(u, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(u, true);
    irq_set_exclusive_handler(irq, h);
    irq_set_enabled(irq, true);
    uart_set_irq_enables(u, true, false);
}

// ---- stan core0 ----
static kiss_parser_t k_par[2];
static kiss_tlm_t    k_tlm[N_MOTORS];
static uint32_t k_frames[2], k_crc_err[2];
static int   g_poles = 14;
static bool  g_3d = false;
static uint32_t g_log_ms = 500;
static float g_cur_mv_per_a = 25.0f;   // skala czujnika pradu ESC (BF scale 250 = 25 mV/A) - ustaw 'cs'
static float g_cur_off_mv[2] = { 0, 0 };
static imu_t g_imu;
static press_bus_t g_press;

static uint16_t pct_to_dshot(int pct) {
    if (pct == 0) return 0;
    if (g_3d) {
        // 3D: 48..1047 = kierunek 1 (ujemne), 1048..2047 = kierunek 2 (dodatnie)
        if (pct < 0) { if (pct < -100) pct = -100; return (uint16_t)(48 + (-pct) * 999 / 100); }
        if (pct > 100) pct = 100;
        return (uint16_t)(1048 + pct * 999 / 100);
    }
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return (uint16_t)(48 + pct * 1999 / 100);
}

static int dshot_to_pct(uint16_t v) {
    if (v < 48) return 0;
    if (g_3d) return v < 1048 ? -(int)((v - 48) * 100 / 999) : (int)((v - 1048) * 100 / 999);
    return (int)((v - 48) * 100 / 1999);
}

static void stop_all(void) { for (int m = 0; m < N_MOTORS; m++) g_throttle[m] = 0; }

static bool any_running(void) { for (int m = 0; m < N_MOTORS; m++) if (g_throttle[m]) return true; return false; }

// parsuje "all" / "1".."8" -> maska bitowa, 0 = blad
static uint8_t parse_motor(const char *s) {
    if (!s) return 0;
    if (!strcmp(s, "all") || !strcmp(s, "a")) return 0xFF;
    if (!strcmp(s, "e1")) return 0x0F;
    if (!strcmp(s, "e2")) return 0xF0;
    int n = atoi(s);
    if (n < 1 || n > 8) return 0;
    return (uint8_t)(1 << (n - 1));
}

static void send_cmd(uint8_t mask, int cmd, int n) {
    if (any_running()) { printf("BLAD: komendy DShot tylko przy zatrzymanych silnikach (s)\n"); return; }
    for (int m = 0; m < N_MOTORS; m++) if (mask & (1 << m)) { g_cmd_val[m] = (uint16_t)cmd; g_cmd_cnt[m] = (uint8_t)n; }
    printf("cmd %d x%d -> maska 0x%02X\n", cmd, n, mask);
}

static float read_cur_mv(int ch) {
    adc_select_input((uint)ch);
    uint32_t acc = 0;
    for (int i = 0; i < 32; i++) acc += adc_read();
    return (float)acc / 32.0f * 3300.0f / 4095.0f;
}


// diagnostyka linii TLM: procent probek w stanie wysokim z pull-up / pull-down / bez
static void print_pins(void) {
    const uint pins[4] = { TLM0_RX_PIN, TLM1_RX_PIN, 0, 8 };
    for (int e = 0; e < 4; e++) {
        uint p = pins[e];
        gpio_set_function(p, GPIO_FUNC_SIO); gpio_set_dir(p, false);
        int hi[3];
        const char *names[3] = { "bez", "pull-up", "pull-down" };
        for (int mode = 0; mode < 3; mode++) {
            gpio_set_pulls(p, mode == 1, mode == 2);
            sleep_ms(20); hi[mode] = 0;
            for (int i = 0; i < 1000; i++) { hi[mode] += gpio_get(p); sleep_us(10); }
        }
        gpio_set_pulls(p, false, false);
        if (e < 2) gpio_set_function(p, (p % 4 == 1) ? GPIO_FUNC_UART : GPIO_FUNC_UART_AUX); else gpio_set_function(p, GPIO_FUNC_NULL);
        printf("%s%d (GPIO%u): high %s=%d%% %s=%d%% %s=%d%%  -> %s\n", e < 2 ? "TLM ESC" : "pin obok ESC", (e % 2) + 1, p,
               names[0], hi[0] / 10, names[1], hi[1] / 10, names[2], hi[2] / 10,
               (hi[1] > 900 && hi[2] < 100) ? "PLYWA (brak polaczenia z ESC)" :
               (hi[1] > 900 && hi[2] > 900) ? "trzymana wysoko (OK, idle UART)" :
               (hi[1] < 100) ? "trzymana nisko (zwarcie do GND / zly pin)" : "aktywna / niejednoznaczna");
    }
    // czy GPIO TLM ESC1 i ESC2 to ten sam sygnal?
    {
        gpio_set_function(TLM0_RX_PIN, GPIO_FUNC_SIO); gpio_set_dir(TLM0_RX_PIN, false);
        gpio_set_function(TLM1_RX_PIN, GPIO_FUNC_SIO); gpio_set_dir(TLM1_RX_PIN, false);
        int same = 0, lo0 = 0, lo1 = 0;
        for (int i = 0; i < 20000; i++) {
            uint32_t v = gpio_get_all();
            int a = (v >> TLM0_RX_PIN) & 1, b = (v >> TLM1_RX_PIN) & 1;
            same += (a == b); lo0 += !a; lo1 += !b;
            sleep_us(2);
        }
        gpio_set_function(TLM0_RX_PIN, (TLM0_RX_PIN % 4 == 1) ? GPIO_FUNC_UART : GPIO_FUNC_UART_AUX);
        gpio_set_function(TLM1_RX_PIN, (TLM1_RX_PIN % 4 == 1) ? GPIO_FUNC_UART : GPIO_FUNC_UART_AUX);
        printf("korelacja TLM1 vs TLM2: identyczne %d.%02d%% probek, low: %d / %d  -> %s\n", same / 200, (same % 200) / 2, lo0, lo1,
               same > 19900 ? "TEN SAM SYGNAL (wspolna siec / zwarcie)" : "rozne sygnaly");
    }
    printf("UART bajtow: ESC1=%lu ESC2=%lu\n", (unsigned long)u_bytes[0], (unsigned long)u_bytes[1]);
    for (int e = 0; e < 2; e++) printf("CUR ESC%d (GPIO%u): %.1f mV\n", e + 1, e ? CUR1_PIN : CUR0_PIN, read_cur_mv(e));
}

static void print_help(void) {
    printf(
      "Komendy:\n"
      "  t <m|all|e1|e2> <pct>   gaz w %% (0 = stop; w trybie 3D -100..100)\n"
      "  raw <m> <0..2047>       surowa wartosc DShot\n"
      "  s                       STOP wszystkie\n"
      "  cmd <m> <0..47> [n]     komenda DShot (domyslnie 6x), tylko gdy stoi\n"
      "  edt                     wlacz Extended DShot Telemetry (cmd 13)\n"
      "  3d on|off               tryb 3D (cmd 10/9) - potem 'save'\n"
      "  dir <m> n|r             kierunek normal/reverse (cmd 20/21) - potem 'save'\n"
      "  save                    zapis ustawien w ESC (cmd 12)\n"
      "  beep [1-5]              beep (cmd 1-5)\n"
      "  poles <n>               liczba biegunow (do RPM), teraz %d\n"
      "  rate 150|300|600|1200   predkosc DShot (kbit/s), teraz %lu\n"
      "  cs <mV/A>               skala czujnika pradu, teraz %.1f;  cz = zeruj offset\n"
      "  log <ms>|off            okres wydruku statusu (teraz %lu)\n"
      "  stat                    jednorazowy status\n"
      "  zero                    zeruj liczniki\n"
      "  env / imucal / axes <f> <l> <g> / p0 / rho <kg/m3>   IMU i cisnienie\n"
      "  pins                    diagnostyka linii TLM/CUR\n"
      "  4way                    tryb BLHeliSuite32 (interfejs 4way-if); wyjscie: Disconnect w Suite lub wpisz exit\n",
      g_poles, (unsigned long)(g_rate / 1000), g_cur_mv_per_a, (unsigned long)g_log_ms);
}

static void print_env(void) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (g_imu.present)
        printf("IMU roll=%6.1f pitch=%6.1f yaw=%6.1f | gyro %6.1f %6.1f %6.1f dps | acc %5.2f %5.2f %5.2f g | T=%.1fC n=%lu err=%lu%s (%lums temu)\n",
               g_imu.roll, g_imu.pitch, g_imu.yaw, g_imu.gx, g_imu.gy, g_imu.gz, g_imu.ax, g_imu.ay, g_imu.az, g_imu.temp_C,
               (unsigned long)g_imu.n_samples, (unsigned long)g_imu.n_err, g_imu.calibrating ? " KALIBRACJA" : "", (unsigned long)(now - g_imu.last_ms));
    else printf("IMU: brak (LSM6DSO32 nie odpowiada na I2C1)\n");
    for (int i = 0; i < 2; i++) {
        const press_t *ps = &g_press.s[i];
        if (!ps->present) { printf("%s: brak\n", ps->name); continue; }
        printf("%s P=%8.2f mbar T=%5.2fC h=%7.3f m (p0=%.2f) ok=%lu err=%lu (%lums temu)\n", ps->name, ps->p_mbar, ps->t_C,
               press_depth_m(&g_press, i), ps->p0_mbar, (unsigned long)ps->n_ok, (unsigned long)ps->n_err, (unsigned long)(now - ps->last_ms));
    }
}

static void print_status(void) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    printf("--- t=%lu.%03lus loop=%lu DShot%lu edt=%s 3d=%s poles=%d usb=%s ---\n",
           (unsigned long)(now / 1000), (unsigned long)(now % 1000), (unsigned long)g_loop_cnt,
           (unsigned long)(g_rate / 1000), g_edt ? "on" : "off", g_3d ? "on" : "off", g_poles,
           stdio_usb_connected() ? "ok" : "BRAK");
    for (int m = 0; m < N_MOTORS; m++) {
        dshot_tlm_t t = g_tlm[m];
        uint32_t rpm = t.erpm / (uint32_t)(g_poles / 2);
        bool fresh = t.n_ok && (now - t.last_ok_ms) < 200;
        printf("M%d thr=%4d%% (%4u) | bidir %s eRPM=%6lu RPM=%6lu ok=%lu crc=%lu none=%lu",
               m + 1, dshot_to_pct(g_throttle[m]), g_throttle[m], fresh ? "OK " : "-- ",
               (unsigned long)t.erpm, (unsigned long)rpm,
               (unsigned long)t.n_ok, (unsigned long)t.n_crc, (unsigned long)t.n_none);
        if (g_edt) printf(" | EDT T=%3uC V=%2u.%02u I=%3uA st=0x%02X", t.edt_temp,
                          t.edt_volt_cV / 100, t.edt_volt_cV % 100, t.edt_curr_A, t.edt_status);
        kiss_tlm_t k = k_tlm[m];
        if (k.valid) printf(" | KISS T=%3dC V=%2u.%02u I=%2u.%02uA mAh=%4u eRPM=%6lu (%lums temu)",
                            k.temp_C, k.volt_cV / 100, k.volt_cV % 100, k.curr_cA / 100, k.curr_cA % 100,
                            k.mAh, (unsigned long)k.erpm, (unsigned long)(now - k.t_ms));
        else printf(" | KISS --");
        printf("\n");
    }
    print_env();
    for (int e = 0; e < 2; e++) {
        float mv = read_cur_mv(e);
        printf("ESC%d CUR=%6.1f mV -> %5.2f A | KISS ramek=%lu crc_err=%lu\n", e + 1, mv,
               (mv - g_cur_off_mv[e]) / g_cur_mv_per_a, (unsigned long)k_frames[e], (unsigned long)k_crc_err[e]);
    }
}

static void start_4way(bool quiet) {
    stop_all();
    g_4way_quiet = quiet;
    if (!quiet) printf("4WAY: tryb BLHeliSuite32 na tym porcie. DShot zatrzymany. Wyjscie: Disconnect w Suite lub 'exit'\n");
    stdio_flush();
    stdio_set_translate_crlf(&stdio_usb, false);
    fourway_reset();
    g_4way_enter_ms = to_ms_since_boot(get_absolute_time());
    g_req_4way = true;
}


// ---- minimalny MSP v1/v2 (BLHeliSuite32 "Betaflight/Cleanflight" przed przejsciem na 4-way) ----
static uint8_t  msp_st, msp_ver, msp_flag, msp_idx, msp_crc, msp_buf[64];
static uint16_t msp_len, msp_cmd;
// log do diagnostyki: co Suite wysyla
static struct { uint8_t ver; uint16_t cmd; uint16_t len; uint8_t ok; } msp_log[32]; static int msp_log_n;
static uint8_t msp_raw[96]; static int msp_raw_n;
static uint8_t crc8_dvb(uint8_t crc, uint8_t a) { crc ^= a; for (int i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1); return crc; }
static void msp_reply(uint16_t cmd, const uint8_t *d, uint16_t n, bool err) {
    if (msp_ver == 2) {
        uint8_t h[5] = { 0, (uint8_t)cmd, (uint8_t)(cmd >> 8), (uint8_t)n, (uint8_t)(n >> 8) }, crc = 0;
        putchar_raw('$'); putchar_raw('X'); putchar_raw(err ? '!' : '>');
        for (int i = 0; i < 5; i++) { putchar_raw(h[i]); crc = crc8_dvb(crc, h[i]); }
        for (int i = 0; i < n; i++) { putchar_raw(d[i]); crc = crc8_dvb(crc, d[i]); }
        putchar_raw(crc);
    } else {
        uint8_t crc = (uint8_t)(n ^ cmd);
        putchar_raw('$'); putchar_raw('M'); putchar_raw(err ? '!' : '>'); putchar_raw((uint8_t)n); putchar_raw((uint8_t)cmd);
        for (int i = 0; i < n; i++) { putchar_raw(d[i]); crc ^= d[i]; }
        putchar_raw(crc);
    }
    stdio_flush();
}
static void msp_handle(uint16_t cmd) {
    uint8_t r[64]; int n = 0; bool ok = true;
    memset(r, 0, sizeof r);
    switch (cmd) {
    case 1:   r[0] = 0; r[1] = 1; r[2] = 45; n = 3; break;                      // MSP_API_VERSION 1.45
    case 2:   memcpy(r, "BTFL", 4); n = 4; break;                               // MSP_FC_VARIANT
    case 3:   r[0] = 4; r[1] = 4; r[2] = 3; n = 3; break;                       // MSP_FC_VERSION 4.4.3
    case 4:   // MSP_BOARD_INFO (API 1.45): id(4) hwrev(2) fcType(1) cap(1) nameLen(1) name boardLen(1) mfrLen(1) sig(32) mcu(1) cfgState(1) rate(2) problems(4)
              memcpy(r, "RP23", 4); r[8] = 7; memcpy(r + 9, "ESCTEST", 7); r[16] = 0; r[17] = 0; r[50] = 0; n = 9 + 7 + 1 + 1 + 32 + 1 + 1 + 2 + 4; break;
    case 5:   memcpy(r, "Oct 19 2026" "00:00:00" "0000000", 26); n = 26; break;  // MSP_BUILD_INFO
    case 68:  n = 0; break;                                                     // MSP_SET_REBOOT - ack, ponizej wyjscie z 4-way
    case 36:  n = 4; break;                                                     // MSP_FEATURE_CONFIG: maska 0
    case 90:  r[0] = 1; r[1] = 1; r[2] = 0; r[3] = 6; r[4] = 480 & 0xFF; r[5] = 480 >> 8; r[6] = 550 & 0xFF; r[7] = 550 >> 8; // MSP_ADVANCED_CONFIG: protokol 6 = DSHOT300
              r[8] = 0; r[9] = 0; r[10] = 0; r[11] = 0; r[12] = 48; r[13] = 125; r[14] = 0; r[15] = 0; r[16] = 0; r[17] = 1; r[18] = 0; r[19] = 0; n = 20; break;
    case 101:                                                                   // MSP_STATUS (API 1.45): nieuzbrojony, zero flag
    case 150: n = 22; break;                                                    // MSP_STATUS_EX
    case 119: r[0] = 0; r[1] = 1; r[2] = 2; n = 3; break;                       // MSP_BOXIDS: ARM, ANGLE, HORIZON
    case 116: memcpy(r, "ARM;ANGLE;HORIZON;", 18); n = 18; break;               // MSP_BOXNAMES
    case 104: for (int i = 0; i < 8; i++) { r[2*i] = 1000 & 0xFF; r[2*i+1] = 1000 >> 8; } n = 16; break; // MSP_MOTOR
    case 131: r[0] = 1070 & 0xFF; r[1] = 1070 >> 8; r[2] = 2000 & 0xFF; r[3] = 2000 >> 8; r[4] = 1000 & 0xFF; r[5] = 1000 >> 8; r[6] = N_MOTORS; r[7] = 14; r[8] = 1; r[9] = 0; n = 10; break; // MSP_MOTOR_CONFIG
    case 160: memcpy(r, "\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0a\x0b\x0c", 12); n = 12; break; // MSP_UID
    case 245: r[0] = N_MOTORS; n = 1; break;                                    // MSP_SET_4WAY_IF
    default:  ok = false; break;
    }
    if (msp_log_n < 32) { msp_log[msp_log_n].ver = msp_ver; msp_log[msp_log_n].cmd = cmd; msp_log[msp_log_n].len = msp_len; msp_log[msp_log_n].ok = ok; msp_log_n++; }
    // Suite gada MSP tuz przed 4-way: juz teraz zatrzymaj DShot (cisza na liniach), a odpowiedzi
    // MSP opozniaj (max 0.8 s kazda), az uzbiera sie ESC_QUIET_MS - inaczej Suite nie doczeka pierwszego ESC
    if (!g_req_4way) start_4way(true);
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - g_4way_enter_ms < ESC_QUIET_MS &&
           to_ms_since_boot(get_absolute_time()) - t0 < 350) sleep_ms(5);   // timeout MSP w Suite < 0.85 s
    if (!ok) { msp_reply(cmd, NULL, 0, true); return; }
    msp_reply(cmd, r, (uint16_t)n, false);
    if (cmd == 68) fourway_exit_req = true;   // Suite 'reboot FC' na koniec sesji -> wroc do DShot
}
static void print_msplog(void) {
    printf("MSP log (%d ramek): ", msp_log_n);
    for (int i = 0; i < msp_log_n; i++) printf("v%d:%u(len %u)%s ", msp_log[i].ver, msp_log[i].cmd, msp_log[i].len, msp_log[i].ok ? "" : "!");
    printf("\nsurowe (%d B): ", msp_raw_n);
    for (int i = 0; i < msp_raw_n; i++) printf("%02X ", msp_raw[i]);
    printf("\n"); msp_log_n = 0; msp_raw_n = 0;
}
// zwraca true gdy bajt skonsumowany przez parser MSP
static bool msp_feed(uint8_t ch) {
    if (msp_raw_n < (int)sizeof msp_raw) msp_raw[msp_raw_n++] = ch;
    switch (msp_st) {
    case 0: if (ch == '$') { msp_st = 1; return true; } return false;
    case 1: if (ch == 'M') { msp_ver = 1; msp_st = 2; } else if (ch == 'X') { msp_ver = 2; msp_st = 2; } else msp_st = 0; return msp_st != 0;
    case 2: msp_st = (ch == '<') ? (msp_ver == 1 ? 3 : 10) : 0; return msp_st != 0;
    // v1
    case 3: msp_len = ch; msp_crc = ch; msp_idx = 0; msp_st = 4; return true;
    case 4: msp_cmd = ch; msp_crc ^= ch; msp_st = msp_len ? 5 : 6; return true;
    case 5: if (msp_idx < sizeof msp_buf) msp_buf[msp_idx] = ch; msp_idx++; msp_crc ^= ch; if (msp_idx >= msp_len) msp_st = 6; return true;
    case 6: msp_st = 0; if (ch == msp_crc) msp_handle(msp_cmd); return true;
    // v2: flag, cmd lo, cmd hi, len lo, len hi, payload, crc8
    case 10: msp_flag = ch; msp_crc = crc8_dvb(0, ch); msp_st = 11; return true;
    case 11: msp_cmd = ch; msp_crc = crc8_dvb(msp_crc, ch); msp_st = 12; return true;
    case 12: msp_cmd |= (uint16_t)ch << 8; msp_crc = crc8_dvb(msp_crc, ch); msp_st = 13; return true;
    case 13: msp_len = ch; msp_crc = crc8_dvb(msp_crc, ch); msp_st = 14; return true;
    case 14: msp_len |= (uint16_t)ch << 8; msp_crc = crc8_dvb(msp_crc, ch); msp_idx = 0; msp_st = msp_len ? 15 : 16; return true;
    case 15: if (msp_idx < sizeof msp_buf) msp_buf[msp_idx] = ch; msp_idx++; msp_crc = crc8_dvb(msp_crc, ch); if (msp_idx >= msp_len) msp_st = 16; return true;
    case 16: msp_st = 0; if (ch == msp_crc) msp_handle(msp_cmd); return true;
    }
    msp_st = 0; return false;
}

// skan I2C bit-bangiem (open-drain przez kierunek pinu) - do wykrycia zamienionych SDA/SCL
static int bb_scan(uint sda, uint scl, char *out, int outlen) {
    gpio_set_function(sda, GPIO_FUNC_SIO); gpio_set_function(scl, GPIO_FUNC_SIO);
    gpio_put(sda, 0); gpio_put(scl, 0); gpio_set_dir(sda, GPIO_IN); gpio_set_dir(scl, GPIO_IN);
    gpio_set_pulls(sda, true, false); gpio_set_pulls(scl, true, false);
    #define SDA_H gpio_set_dir(sda, GPIO_IN)
    #define SDA_L gpio_set_dir(sda, GPIO_OUT)
    #define SCL_H gpio_set_dir(scl, GPIO_IN)
    #define SCL_L gpio_set_dir(scl, GPIO_OUT)
    int n = 0, found = 0;
    for (int k = 0; k < 9; k++) { SCL_L; sleep_us(5); SCL_H; sleep_us(5); }   // bus recovery
    for (int a = 0x08; a < 0x78; a++) {
        SDA_H; SCL_H; sleep_us(5); SDA_L; sleep_us(5); SCL_L; sleep_us(5);      // START
        uint8_t b = (uint8_t)(a << 1);
        for (int i = 7; i >= 0; i--) { if (b & (1 << i)) SDA_H; else SDA_L; sleep_us(3); SCL_H; sleep_us(5); SCL_L; sleep_us(3); }
        SDA_H; sleep_us(3); SCL_H; sleep_us(5); int ack = !gpio_get(sda); SCL_L; sleep_us(3);
        SDA_L; sleep_us(5); SCL_H; sleep_us(5); SDA_H; sleep_us(10);           // STOP
        if (ack) { n += snprintf(out + n, outlen - n, " 0x%02X", a); found++; }
    }
    gpio_set_function(sda, GPIO_FUNC_I2C); gpio_set_function(scl, GPIO_FUNC_I2C);
    return found;
}

static void handle_line(char *line) {
    char *tok[4] = { 0 }; int n = 0;
    for (char *p = strtok(line, " \t\r\n"); p && n < 4; p = strtok(NULL, " \t\r\n")) tok[n++] = p;
    if (!n) return;
    for (char *p = tok[0]; *p; p++) *p = (char)tolower((unsigned char)*p);
    const char *c = tok[0];
    if (!strcmp(c, "help") || !strcmp(c, "?")) print_help();
    else if (!strcmp(c, "s") || !strcmp(c, "stop")) { stop_all(); printf("STOP\n"); }
    else if (!strcmp(c, "t") || !strcmp(c, "raw")) {
        uint8_t mask = parse_motor(tok[1]);
        if (!mask || n < 3) { printf("uzycie: %s <1-8|all|e1|e2> <wartosc>\n", c); return; }
        int v = atoi(tok[2]);
        uint16_t d = !strcmp(c, "t") ? pct_to_dshot(v) : (uint16_t)((v > 0 && v < 48) ? 48 : (v > 2047 ? 2047 : (v < 0 ? 0 : v)));
        for (int m = 0; m < N_MOTORS; m++) if (mask & (1 << m)) g_throttle[m] = d;
        printf("maska 0x%02X -> DShot %u (%d%%)\n", mask, d, dshot_to_pct(d));
    }
    else if (!strcmp(c, "cmd")) {
        uint8_t mask = parse_motor(tok[1]);
        if (!mask || n < 3) { printf("uzycie: cmd <m> <0..47> [n]\n"); return; }
        int cmd = atoi(tok[2]); int rep = n > 3 ? atoi(tok[3]) : 6;
        if (cmd < 0 || cmd > 47 || rep < 1 || rep > 255) { printf("zly zakres\n"); return; }
        send_cmd(mask, cmd, rep);
    }
    else if (!strcmp(c, "edt")) { send_cmd(0xFF, 13, 6); g_edt = true; }
    else if (!strcmp(c, "3d")) {
        if (n < 2) { printf("uzycie: 3d on|off\n"); return; }
        bool on = !strcmp(tok[1], "on");
        send_cmd(0xFF, on ? 10 : 9, 6); g_3d = on; stop_all();
        printf("3D=%s (pamietaj 'save')\n", on ? "on" : "off");
    }
    else if (!strcmp(c, "dir")) {
        uint8_t mask = parse_motor(tok[1]);
        if (!mask || n < 3) { printf("uzycie: dir <m> n|r\n"); return; }
        send_cmd(mask, tok[2][0] == 'r' ? 21 : 20, 6);
    }
    else if (!strcmp(c, "save")) send_cmd(0xFF, 12, 6);
    else if (!strcmp(c, "beep")) { int b = n > 1 ? atoi(tok[1]) : 1; if (b < 1 || b > 5) b = 1; send_cmd(0xFF, b, 1); }
    else if (!strcmp(c, "poles")) { int p = n > 1 ? atoi(tok[1]) : 0; if (p >= 2 && p <= 64) g_poles = p & ~1; printf("poles=%d\n", g_poles); }
    else if (!strcmp(c, "rate")) {
        int r = n > 1 ? atoi(tok[1]) : 0;
        if (r != 150 && r != 300 && r != 600 && r != 1200) { printf("rate 150|300|600|1200\n"); return; }
        g_rate_req = (uint32_t)r * 1000; printf("DShot%d\n", r);
    }
    else if (!strcmp(c, "cs")) { if (n > 1) g_cur_mv_per_a = (float)atof(tok[1]); printf("cs=%.2f mV/A\n", g_cur_mv_per_a); }
    else if (!strcmp(c, "cz")) { for (int e = 0; e < 2; e++) g_cur_off_mv[e] = read_cur_mv(e); printf("offset %.1f / %.1f mV\n", g_cur_off_mv[0], g_cur_off_mv[1]); }
    else if (!strcmp(c, "log")) { g_log_ms = (n > 1 && strcmp(tok[1], "off")) ? (uint32_t)atoi(tok[1]) : 0; printf("log=%lu\n", (unsigned long)g_log_ms); }
    else if (!strcmp(c, "stat")) print_status();
    else if (!strcmp(c, "pins")) print_pins();
    else if (!strcmp(c, "4way")) start_4way(false);
    else if (!strcmp(c, "msplog")) print_msplog();
    else if (!strcmp(c, "i2c")) {       // diagnostyka I2C1: poziomy linii z pull-down (1 = zewnetrzny pull-up) + skan adresow
        int lv[2];
        const uint pins2[2] = { I2C_SDA_PIN, I2C_SCL_PIN };
        for (int k = 0; k < 2; k++) {
            gpio_set_function(pins2[k], GPIO_FUNC_SIO); gpio_set_dir(pins2[k], GPIO_IN); gpio_set_pulls(pins2[k], false, true);
            sleep_ms(5); lv[k] = 0; for (int i = 0; i < 100; i++) { lv[k] += gpio_get(pins2[k]); sleep_us(50); }
            gpio_set_pulls(pins2[k], true, false); gpio_set_function(pins2[k], GPIO_FUNC_I2C);
        }
        printf("SDA(GPIO%d) z pull-down: %d%% high, SCL(GPIO%d): %d%% high  -> %s\n", I2C_SDA_PIN, lv[0], I2C_SCL_PIN, lv[1],
               (lv[0] > 90 && lv[1] > 90) ? "zewnetrzne pull-upy sa (cos podpiete)" : "BRAK zewnetrznych pull-upow - czujniki nie sa na tych pinach albo brak zasilania");
        {   // test zwarcia SDA-SCL: jedna linia na 0, czytaj druga
            gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_SIO); gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_SIO);
            gpio_set_dir(I2C_SDA_PIN, GPIO_OUT); gpio_put(I2C_SDA_PIN, 0); gpio_set_dir(I2C_SCL_PIN, GPIO_IN); gpio_set_pulls(I2C_SCL_PIN, true, false); sleep_ms(2);
            int scl_when_sda0 = gpio_get(I2C_SCL_PIN);
            gpio_set_dir(I2C_SDA_PIN, GPIO_IN); gpio_set_pulls(I2C_SDA_PIN, true, false);
            gpio_set_dir(I2C_SCL_PIN, GPIO_OUT); gpio_put(I2C_SCL_PIN, 0); sleep_ms(2);
            int sda_when_scl0 = gpio_get(I2C_SDA_PIN);
            gpio_set_dir(I2C_SCL_PIN, GPIO_IN);
            gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C); gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
            printf("SDA=0 -> SCL=%d, SCL=0 -> SDA=%d  -> %s\n", scl_when_sda0, sda_when_scl0,
                   (scl_when_sda0 && sda_when_scl0) ? "linie niezalezne (OK)" : "ZWARCIE SDA-SCL albo linia nie moze zejsc do 0");
        }
        printf("skan I2C1:");
        int found = 0;
        for (int a = 0x08; a < 0x78; a++) { uint8_t d; if (i2c_read_timeout_us(i2c1, (uint8_t)a, &d, 1, false, 2000) >= 0) { printf(" 0x%02X", a); found++; } }
        printf("%s (%d)\n", found ? "" : " nic", found);
        { char o[128]; int f;
          f = bb_scan(I2C_SDA_PIN, I2C_SCL_PIN, o, sizeof o); printf("bit-bang SDA=GPIO%d SCL=GPIO%d:%s (%d)\n", I2C_SDA_PIN, I2C_SCL_PIN, f ? o : " nic", f);
          f = bb_scan(I2C_SCL_PIN, I2C_SDA_PIN, o, sizeof o); printf("bit-bang ZAMIENIONE SDA=GPIO%d SCL=GPIO%d:%s (%d)\n", I2C_SCL_PIN, I2C_SDA_PIN, f ? o : " nic", f);
          i2c_init(i2c1, 400000); }
        if (!imu_init(&g_imu, i2c1, 0x6A)) imu_init(&g_imu, i2c1, 0x6B);
        if (g_imu.present) imu_start_cal(&g_imu);
        press_init(&g_press, i2c1);
        printf("IMU %s, MS5837 %s, MS5803 %s\n", g_imu.present ? "OK" : "brak", g_press.s[0].present ? "OK" : "brak", g_press.s[1].present ? "OK" : "brak");
    }
    else if (!strcmp(c, "env")) print_env();
    else if (!strcmp(c, "imucal")) { imu_start_cal(&g_imu); printf("kalibracja zyro: nie ruszac 2 s\n"); }
    else if (!strcmp(c, "axes")) {
        if (n < 4 || !imu_set_axes(&g_imu, tok[1], tok[2], tok[3])) { printf("uzycie: axes <przod> <lewo> <gora>, np. axes x y z albo axes -y x z\n"); return; }
        printf("osie: przod=%s lewo=%s gora=%s\n", tok[1], tok[2], tok[3]);
    }
    else if (!strcmp(c, "p0")) { press_zero(&g_press); printf("p0 = %.2f / %.2f mbar\n", g_press.s[0].p0_mbar, g_press.s[1].p0_mbar); }
    else if (!strcmp(c, "rho")) { if (n > 1) g_press.rho = (float)atof(tok[1]); printf("rho=%.1f kg/m3\n", g_press.rho); }
    else if (!strcmp(c, "pwm")) {       // 1000..2000 jak PWM w trybie 3D: 1500 = stop, <1500 wstecz, >1500 naprzod
        uint8_t mask = parse_motor(tok[1]);
        if (!mask || n < 3) { printf("uzycie: pwm <1-8|all|e1|e2> <1000..2000>\n"); return; }
        int v = atoi(tok[2]); if (v < 1000) v = 1000; if (v > 2000) v = 2000;
        uint16_t d = (v == 1500) ? 0 : (v < 1500 ? (uint16_t)(48 + (1500 - v) * 999 / 500) : (uint16_t)(1048 + (v - 1500) * 999 / 500));
        for (int m = 0; m < N_MOTORS; m++) if (mask & (1 << m)) g_throttle[m] = d;
        printf("pwm %d -> DShot %u\n", v, d);
    }
    else if (!strcmp(c, "rpm")) {       // jedna linia: eRPM 8 silnikow + prad obu ESC
        printf("eRPM");
        for (int m = 0; m < N_MOTORS; m++) printf(" %6lu", (unsigned long)g_tlm[m].erpm);
        printf(" | I %.2f %.2f A\n", (read_cur_mv(0) - g_cur_off_mv[0]) / g_cur_mv_per_a, (read_cur_mv(1) - g_cur_off_mv[1]) / g_cur_mv_per_a);
    }

    else if (!strcmp(c, "info")) {      // DShot cmd 6 (ESC info) -> surowe bajty z TLM
        int m = n > 1 ? atoi(tok[1]) : 0;
        if (m < 1 || m > 8) { printf("uzycie: info <1-8>\n"); return; }
        if (any_running()) { printf("najpierw s\n"); return; }
        int e = (m - 1) / 4;
        u_tail[e] = u_head[e];                               // wyczysc bufor
        g_cmd_val[m - 1] = 6; g_cmd_cnt[m - 1] = 1;
        uint32_t t0 = to_ms_since_boot(get_absolute_time());
        uint8_t buf[256]; int len = 0;
        while (to_ms_since_boot(get_absolute_time()) - t0 < 400) {
            while (u_tail[e] != u_head[e] && len < 256) { buf[len++] = u_ring[e][u_tail[e]]; u_tail[e]++; }
            sleep_ms(1);
        }
        printf("ESC info M%d: %d bajtow w 400 ms\n", m, len);
        for (int i = 0; i < len; i++) { printf("%02X ", buf[i]); if ((i & 15) == 15) printf("\n"); }
        printf("\n ascii: ");
        for (int i = 0; i < len; i++) putchar((buf[i] >= 32 && buf[i] < 127) ? buf[i] : '.');
        printf("\n");
    }

    else if (!strcmp(c, "bl")) {
        int m = n > 1 ? atoi(tok[1]) : 0;
        if (m < 1 || m > 8) { printf("uzycie: bl <1-8>\n"); return; }
        if (any_running()) { printf("najpierw s\n"); return; }
        g_probe_done = false; g_probe_mode = n > 2 ? atoi(tok[2]) : 0; g_probe_hold = n > 3 ? atoi(tok[3]) : 2500; g_req_probe = m - 1;
        while (!g_probe_done) sleep_ms(10);
        printf("bootloader ESC M%d:\n%s", m, g_probe_out);
    }
    else if (!strcmp(c, "zero")) {
        for (int m = 0; m < N_MOTORS; m++) { g_tlm[m].n_ok = g_tlm[m].n_crc = g_tlm[m].n_none = 0; k_tlm[m].n_ok = 0; }
        k_frames[0] = k_frames[1] = k_crc_err[0] = k_crc_err[1] = 0; printf("wyzerowano\n");
    }
    else printf("nieznana komenda '%s' (help)\n", c);
}

int main(void) {
    stdio_init_all();
    adc_init(); adc_gpio_init(CUR0_PIN); adc_gpio_init(CUR1_PIN);
    tlm_uart_init(uart0, TLM0_RX_PIN, UART0_IRQ, uart0_irq);
    tlm_uart_init(uart1, TLM1_RX_PIN, UART1_IRQ, uart1_irq);
    multicore_launch_core1(core1_main);

    // I2C1: IMU + czujniki cisnienia
    i2c_init(i2c1, 400000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C); gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN); gpio_pull_up(I2C_SCL_PIN);
    gpio_init(IMU_INT_PIN); gpio_set_dir(IMU_INT_PIN, GPIO_IN); gpio_pull_down(IMU_INT_PIN);
    if (!imu_init(&g_imu, i2c1, 0x6A)) imu_init(&g_imu, i2c1, 0x6B);
    if (g_imu.present) imu_start_cal(&g_imu);
    press_init(&g_press, i2c1);
    uint32_t imu_last_us = time_us_32();

    char line[64]; int ll = 0;
    uint32_t last_log = 0, usb_lost_ms = 0;
    bool was_connected = false;
    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // USB: powitanie po polaczeniu, failsafe po rozlaczeniu
        bool conn = stdio_usb_connected();
        if (conn && !was_connected) { sleep_ms(50); printf("\nESC TEST RP2350 - 8x bidir DShot, KISS tlm, ADC. 'help'\n"); usb_lost_ms = 0; }
        if (!conn) { if (!usb_lost_ms) usb_lost_ms = now ? now : 1; else if (now - usb_lost_ms > USB_FAILSAFE_MS && any_running()) stop_all(); }
        else usb_lost_ms = 0;
        was_connected = conn;

        // tryb 4-way: pompuj bajty USB <-> core1; 'exit' lub utrata USB konczy
        if (g_req_4way) {
            static char tail[5]; static uint32_t last_host_ms; int ch;
            if (!last_host_ms) last_host_ms = now;
            // sledzenie ramek 4-way: bajty WEWNATRZ ramki ('/' cmd adrH adrL len [param...] crcH crcL)
            // ida prosto do core1 - w danych binarnych moze byc '$' i nie wolno ich oddac parserowi MSP
            static int fw_hdr, fw_remaining; static uint8_t fw_len;
            while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
                last_host_ms = now;
                if (fw_remaining > 0) { fourway_host_put((uint8_t)ch); fw_remaining--; continue; }
                if (fw_hdr > 0) {
                    fourway_host_put((uint8_t)ch);
                    if (fw_hdr == 1) { fw_len = (uint8_t)ch; fw_remaining = (fw_len ? fw_len : 256) + 2; }
                    fw_hdr--; continue;
                }
                if (ch == '/') { fourway_host_put((uint8_t)ch); fw_hdr = 4; continue; }
                if (msp_feed((uint8_t)ch)) continue;
                memmove(tail, tail + 1, 4); tail[4] = (char)ch;
                if (!memcmp(tail + 1, "exit", 4)) fourway_exit_req = true;
            }
            int ob; bool any = false;
            while ((ob = fourway_host_get()) >= 0) { putchar_raw(ob); any = true; }
            if (any) stdio_flush();
            // utrata USB NIE konczy trybu 4-way (silniki i tak stoja); konczy 'exit' albo InterfaceExit z Suite
            if (now - last_host_ms > 300000) fourway_exit_req = true;    // 5 min bez Suite -> wroc do DShot
            if (fourway_exit_req || !g_req_4way) {
                last_host_ms = 0; fw_hdr = 0; fw_remaining = 0;
                while (g_req_4way) { while ((ob = fourway_host_get()) >= 0) putchar_raw(ob); tight_loop_contents(); }
                stdio_set_translate_crlf(&stdio_usb, true);
                if (!g_4way_quiet) printf("\n4WAY: koniec, DShot wznowiony\n");
                ll = 0;
            }
            sleep_ms(1);
            continue;
        }

        // CLI ('/' na poczatku linii = ramka 4-way od BLHeliSuite -> automatyczne wejscie w tryb)
        int ch;
        while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
            if ((ll == 0 || msp_st) && msp_feed((uint8_t)ch)) { if (g_req_4way) break; continue; }
            if (ll == 0 && ch == '/') { start_4way(true); fourway_host_put((uint8_t)ch); break; }
            if (ch == '\r' || ch == '\n') { line[ll] = 0; if (ll) handle_line(line); ll = 0; }
            else if (ch == 8 || ch == 127) { if (ll) ll--; }
            else if (ll < (int)sizeof(line) - 1 && ch >= 32) line[ll++] = (char)ch;
        }

        // KISS z obu UART
        for (int e = 0; e < 2; e++) {
            while (u_tail[e] != u_head[e]) {
                uint8_t b = u_ring[e][u_tail[e]]; u_tail[e]++;
                kiss_tlm_t k;
                uint32_t before = k_par[e].n_crc;
                if (kiss_feed(&k_par[e], b, &k)) {
                    int m = g_tlm_req_motor[e];
                    if (m >= 0) { k.n_ok = k_tlm[m].n_ok + 1; k_tlm[m] = k; }
                    k_frames[e]++;
                }
                k_crc_err[e] += k_par[e].n_crc - before;
            }
        }

        // IMU: po przerwaniu DRDY (INT1 high) albo awaryjnie co 10 ms; cisnienie: maszyna stanow
        if (g_imu.present) {
            uint32_t tu = time_us_32();
            if (gpio_get(IMU_INT_PIN) || tu - imu_last_us > 10000) {
                float dt = (float)(tu - imu_last_us) * 1e-6f; imu_last_us = tu;
                imu_read(&g_imu, i2c1, dt);
            }
        }
        press_poll(&g_press);

        if (g_log_ms && now - last_log >= g_log_ms && conn) { last_log = now; print_status(); }
        sleep_ms(1);
    }
}
