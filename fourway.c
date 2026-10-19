// Port Betaflight serial_4way.c + serial_4way_avrootloader.c (GPL-3.0, autor 4712)
// na RP2350. Obslugiwane bootloadery: BLHeli (SiLabs / Atmel / ARM = BLHeli_32).
#include "fourway.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <string.h>

// ---------------- bufory host <-> core1 ----------------
#define RING 1024
static volatile uint8_t rx_ring[RING]; static volatile uint16_t rx_h, rx_t;   // host -> core1
static volatile uint8_t tx_ring[RING]; static volatile uint16_t tx_h, tx_t;   // core1 -> host
volatile bool fourway_exit_req;

void fourway_host_put(uint8_t b) { uint16_t n = (rx_h + 1) & (RING - 1); if (n != rx_t) { rx_ring[rx_h] = b; rx_h = n; } }
int  fourway_host_get(void) { if (tx_t == tx_h) return -1; uint8_t b = tx_ring[tx_t]; tx_t = (tx_t + 1) & (RING - 1); return b; }
static void out_byte(uint8_t b) {
    uint16_t n = (tx_h + 1) & (RING - 1);
    while (n == tx_t) { if (fourway_exit_req) return; }
    tx_ring[tx_h] = b; tx_h = n;
}
// true = odebrano; false = timeout (timeout_us == 0: czekaj bez konca, ale reaguj na exit)
static bool in_byte(uint8_t *b, uint32_t timeout_us) {
    uint32_t t0 = time_us_32();
    while (rx_t == rx_h) {
        if (fourway_exit_req) return false;
        if (timeout_us && (time_us_32() - t0) > timeout_us) return false;
    }
    *b = rx_ring[rx_t]; rx_t = (rx_t + 1) & (RING - 1);
    return true;
}

// ---------------- piny ESC ----------------
static const uint8_t *s_pins; static int s_n;
static uint8_t selected_esc;
static uint32_t s_quiet_since[16];   // millis() ostatniej aktywnosci TX na linii danego ESC
static bool s_inv;   // true = UART odwrocony (idle low) - test dla ESC w trybie bidir
static inline bool ESC_IS_HI(void) { return gpio_get(s_pins[selected_esc]) ^ s_inv; }
static inline void ESC_SET_HI(void) { gpio_put(s_pins[selected_esc], !s_inv); }
static inline void ESC_SET_LO(void) { gpio_put(s_pins[selected_esc], s_inv); }
static inline void ESC_INPUT(void)  { gpio_set_pulls(s_pins[selected_esc], !s_inv, s_inv); gpio_set_dir(s_pins[selected_esc], GPIO_IN); }
static inline void ESC_OUTPUT(void) { gpio_set_dir(s_pins[selected_esc], GPIO_OUT); }

void fourway_init(const uint8_t *pins, int n) { s_pins = pins; s_n = n; }
void fourway_reset(void) { rx_h = rx_t = tx_h = tx_t = 0; fourway_exit_req = false; }

static inline uint32_t micros(void) { return time_us_32(); }
static inline uint32_t millis(void) { return to_ms_since_boot(get_absolute_time()); }

// ---------------- bootloader BLHeli: software UART 19200 8N1 ----------------
#define START_BIT_TIMEOUT_MS 2
#define BIT_TIME        52
#define BIT_TIME_HALVE  (BIT_TIME >> 1)
#define BIT_TIME_3_4    (BIT_TIME_HALVE + (BIT_TIME_HALVE >> 1))
#define START_BIT_TIME  (BIT_TIME_3_4)

#define brSUCCESS       0x30
#define brERRORVERIFY   0xC0
#define brERRORCOMMAND  0xC1
#define brERRORCRC      0xC2
#define brNONE          0xFF

#define RestartBootloader   0
#define CMD_PROG_FLASH      0x01
#define CMD_ERASE_FLASH     0x02
#define CMD_READ_FLASH_SIL  0x03
#define CMD_VERIFY_FLASH_ARM 0x04
#define CMD_READ_EEPROM     0x04
#define CMD_PROG_EEPROM     0x05
#define CMD_READ_FLASH_ATM  0x07
#define CMD_KEEP_ALIVE      0xFD
#define CMD_SET_ADDRESS     0xFF
#define CMD_SET_BUFFER      0xFE

#define imC2 0
#define imSIL_BLB 1
#define imATM_BLB 2
#define imSK 3
#define imARM_BLB 4

typedef union __attribute__((packed)) { uint8_t bytes[2]; uint16_t word; } u8_16;
typedef union __attribute__((packed)) { uint8_t bytes[4]; uint16_t words[2]; uint32_t dword; } u8_32;
typedef struct { uint8_t D_NUM_BYTES, D_FLASH_ADDR_H, D_FLASH_ADDR_L; uint8_t *D_PTR_I; } ioMem_t;

static u8_32 DeviceInfo;
static uint8_t CurrentInterfaceMode;
static inline bool isMcuConnected(void) { return DeviceInfo.bytes[0] > 0; }

static uint8_t suart_getc_(uint8_t *bt) {
    uint32_t wait_time = millis() + START_BIT_TIMEOUT_MS;
    while (ESC_IS_HI()) { if (millis() >= wait_time) return 0; }
    uint32_t btime = micros() + START_BIT_TIME;
    uint16_t bitmask = 0; uint8_t bit = 0;
    while ((int32_t)(micros() - btime) < 0);
    while (1) {
        if (ESC_IS_HI()) bitmask |= (1 << bit);
        btime += BIT_TIME; bit++;
        if (bit == 10) break;
        while ((int32_t)(micros() - btime) < 0);
    }
    if ((bitmask & 1) || (!(bitmask & (1 << 9)))) return 0;   // start/stop bit
    *bt = (uint8_t)(bitmask >> 1);
    return 1;
}

static void suart_putc_(const uint8_t *tx_b) {
    uint16_t bitmask = (uint16_t)((*tx_b << 2) | 1 | (1 << 10));
    uint32_t btime = micros();
    while (1) {
        if (bitmask & 1) ESC_SET_HI(); else ESC_SET_LO();
        btime += BIT_TIME;
        bitmask >>= 1;
        if (bitmask == 0) break;
        while ((int32_t)(micros() - btime) < 0);
    }
}

static u8_16 CRC_16, LastCRC_16;
static void ByteCrc(const uint8_t *bt) {
    uint8_t xb = *bt;
    for (uint8_t i = 0; i < 8; i++) {
        if (((xb & 0x01) ^ (CRC_16.word & 0x0001)) != 0) { CRC_16.word >>= 1; CRC_16.word ^= 0xA001; }
        else CRC_16.word >>= 1;
        xb >>= 1;
    }
}

static uint8_t BL_ReadBuf(uint8_t *pstring, uint8_t len) {
    CRC_16.word = 0; LastCRC_16.word = 0;
    uint8_t LastACK = brNONE;
    do {
        if (!suart_getc_(pstring)) goto timeout;
        ByteCrc(pstring); pstring++; len--;
    } while (len > 0);
    if (isMcuConnected()) {
        if (!suart_getc_(&LastCRC_16.bytes[0])) goto timeout;
        if (!suart_getc_(&LastCRC_16.bytes[1])) goto timeout;
        if (!suart_getc_(&LastACK)) goto timeout;
        if (CRC_16.word != LastCRC_16.word) LastACK = brERRORCRC;
    } else {
        if (!suart_getc_(&LastACK)) goto timeout;
    }
timeout:
    return LastACK == brSUCCESS;
}

static void BL_SendBuf(uint8_t *pstring, uint8_t len) {
    ESC_OUTPUT();
    CRC_16.word = 0;
    do { suart_putc_(pstring); ByteCrc(pstring); pstring++; len--; } while (len > 0);
    if (isMcuConnected()) { suart_putc_(&CRC_16.bytes[0]); suart_putc_(&CRC_16.bytes[1]); }
    ESC_INPUT();
    // UWAGA: nie zerujemy s_quiet_since - ESC raz w bootloaderze ignoruje krotkie transmisje
    // (Suite wysyla DeviceReset tuz przed InitFlash i daje tylko 1.5 s na odpowiedz)
}

static uint8_t BL_ConnectEx(u8_32 *pDeviceInfo) {
    #define BootMsgLen 4
    uint8_t BootInfo[9];
    uint8_t BootInit[] = {0,0,0,0,0,0,0,0,0x0D,'B','L','H','e','l','i',0xF4,0x7D};
    BL_SendBuf(BootInit, 17);
    if (!BL_ReadBuf(BootInfo, BootMsgLen + 4)) return 0;
    if (BootInfo[0] != '4' || BootInfo[1] != '7' || BootInfo[2] != '1') return 0;
    pDeviceInfo->bytes[2] = BootInfo[BootMsgLen - 1];
    pDeviceInfo->bytes[1] = BootInfo[BootMsgLen];
    pDeviceInfo->bytes[0] = BootInfo[BootMsgLen + 1];
    return 1;
}

static uint8_t BL_GetACK(uint32_t Timeout) {
    uint8_t LastACK = brNONE;
    while (!(suart_getc_(&LastACK)) && (Timeout)) { Timeout--; }
    return LastACK;
}

static uint8_t BL_SendCMDKeepAlive(void) {
    uint8_t sCMD[] = {CMD_KEEP_ALIVE, 0};
    BL_SendBuf(sCMD, 2);
    return BL_GetACK(1) == brERRORCOMMAND;
}

static void BL_SendCMDRunRestartBootloader(u8_32 *pDeviceInfo) {
    uint8_t sCMD[] = {RestartBootloader, 0};
    pDeviceInfo->bytes[0] = 1;
    BL_SendBuf(sCMD, 2);
}

static uint8_t BL_SendCMDSetAddress(ioMem_t *pMem) {
    if ((pMem->D_FLASH_ADDR_H == 0xFF) && (pMem->D_FLASH_ADDR_L == 0xFF)) return 1;
    uint8_t sCMD[] = {CMD_SET_ADDRESS, 0, pMem->D_FLASH_ADDR_H, pMem->D_FLASH_ADDR_L};
    BL_SendBuf(sCMD, 4);
    return BL_GetACK(2) == brSUCCESS;
}

static uint8_t BL_SendCMDSetBuffer(ioMem_t *pMem) {
    uint8_t sCMD[] = {CMD_SET_BUFFER, 0, 0, pMem->D_NUM_BYTES};
    if (pMem->D_NUM_BYTES == 0) sCMD[2] = 1;
    BL_SendBuf(sCMD, 4);
    if (BL_GetACK(2) != brNONE) return 0;
    BL_SendBuf(pMem->D_PTR_I, pMem->D_NUM_BYTES);
    return BL_GetACK(40) == brSUCCESS;
}

static uint8_t BL_ReadA(uint8_t cmd, ioMem_t *pMem) {
    if (BL_SendCMDSetAddress(pMem)) {
        uint8_t sCMD[] = {cmd, pMem->D_NUM_BYTES};
        BL_SendBuf(sCMD, 2);
        return BL_ReadBuf(pMem->D_PTR_I, pMem->D_NUM_BYTES);
    }
    return 0;
}

static uint8_t BL_WriteA(uint8_t cmd, ioMem_t *pMem, uint32_t timeout) {
    if (BL_SendCMDSetAddress(pMem)) {
        if (!BL_SendCMDSetBuffer(pMem)) return 0;
        uint8_t sCMD[] = {cmd, 0x01};
        BL_SendBuf(sCMD, 2);
        return BL_GetACK(timeout) == brSUCCESS;
    }
    return 0;
}

static uint8_t BL_ReadFlash(uint8_t mode, ioMem_t *pMem) {
    return BL_ReadA(mode == imATM_BLB ? CMD_READ_FLASH_ATM : CMD_READ_FLASH_SIL, pMem);
}
static uint8_t BL_ReadEEprom(ioMem_t *pMem) { return BL_ReadA(CMD_READ_EEPROM, pMem); }
static uint8_t BL_PageErase(ioMem_t *pMem) {
    if (BL_SendCMDSetAddress(pMem)) {
        uint8_t sCMD[] = {CMD_ERASE_FLASH, 0x01};
        BL_SendBuf(sCMD, 2);
        return BL_GetACK(3000 / START_BIT_TIMEOUT_MS) == brSUCCESS;
    }
    return 0;
}
static uint8_t BL_WriteEEprom(ioMem_t *pMem) { return BL_WriteA(CMD_PROG_EEPROM, pMem, 3000 / START_BIT_TIMEOUT_MS); }
static uint8_t BL_WriteFlash(ioMem_t *pMem)  { return BL_WriteA(CMD_PROG_FLASH, pMem, 500 / START_BIT_TIMEOUT_MS); }
static uint8_t BL_VerifyFlash(ioMem_t *pMem) {
    if (BL_SendCMDSetAddress(pMem)) {
        if (!BL_SendCMDSetBuffer(pMem)) return 0;
        uint8_t sCMD[] = {CMD_VERIFY_FLASH_ARM, 0x01};
        BL_SendBuf(sCMD, 2);
        return BL_GetACK(40 / START_BIT_TIMEOUT_MS);
    }
    return 0;
}

#define ATMEL_DEVICE_MATCH ((p->words[0] == 0x9307) || (p->words[0] == 0x930A) || (p->words[0] == 0x930F) || (p->words[0] == 0x940B))
#define SILABS_DEVICE_MATCH ((p->words[0] > 0xE800) && (p->words[0] < 0xF900))
#define ARM_DEVICE_MATCH ((p->bytes[1] > 0x00) && (p->bytes[1] < 0x90) && (p->bytes[0] == 0x06))

static uint8_t Connect(u8_32 *p) {
    for (uint8_t I = 0; I < 3; ++I) {
        if (BL_ConnectEx(p)) {
            if (SILABS_DEVICE_MATCH)     { CurrentInterfaceMode = imSIL_BLB; return 1; }
            else if (ATMEL_DEVICE_MATCH) { CurrentInterfaceMode = imATM_BLB; return 1; }
            else if (ARM_DEVICE_MATCH)   { CurrentInterfaceMode = imARM_BLB; return 1; }
        }
    }
    return 0;
}


// Diagnostyka: wysyla BootInit do ESC i raportuje, co wrocilo (uruchamiac na core1, piny juz jako GPIO)
#include <stdio.h>
int fourway_probe(int esc, int mode, int hold_ms, char *out, int outlen) {
    int n = 0;
    selected_esc = (uint8_t)esc;
    gpio_set_function(s_pins[esc], GPIO_FUNC_SIO);
    gpio_put(s_pins[esc], 1); gpio_set_dir(s_pins[esc], GPIO_IN); gpio_pull_up(s_pins[esc]);
    int got = 0; uint32_t t_start = millis();
    int max_attempts = 3;
    for (int attempt = 0; attempt < max_attempts && n < outlen - 160; attempt++) {
        // stan spoczynkowy przed proba: 0 = wejscie z pull-up, 1 = wyjscie HIGH, 2 = wyjscie LOW
        // 0 = wejscie pull-up; 1 = wyjscie HIGH; 2 = LOW 300ms -> init od razu;
        // 3 = LOW 300ms -> HIGH 100ms -> init; 4 = LOW 300ms -> HIGH 500ms -> init (reboot ESC jak w BF/AM32)
        // 5 = UART ODWROCONY, idle = wejscie z pull-down; 6 = UART odwrocony, idle = wyjscie LOW
        s_inv = (mode >= 5);
        if (mode == 0) { gpio_set_pulls(s_pins[esc], true, false); gpio_set_dir(s_pins[esc], GPIO_IN); }
        else if (mode == 5) { gpio_set_pulls(s_pins[esc], false, true); gpio_set_dir(s_pins[esc], GPIO_IN); }
        else { gpio_put(s_pins[esc], mode == 1); gpio_set_dir(s_pins[esc], GPIO_OUT); }
        int hold_edges = 0;
        { uint32_t w = millis(); int lastv = gpio_get(s_pins[esc]); while (millis() - w < (uint32_t)hold_ms) { int v = gpio_get(s_pins[esc]); if (v != lastv) hold_edges++; lastv = v; } }   // BLHeli_32: linia high > 2 s = wejscie w bootloader
        gpio_put(s_pins[esc], !s_inv);
        if (mode == 3 || mode == 4) { uint32_t w = millis(); while (millis() - w < (mode == 3 ? 100u : 500u)); }
        int idle = ESC_IS_HI();
        DeviceInfo.words[0] = 0;
        uint8_t BootInit[] = {0,0,0,0,0,0,0,0,0x0D,'B','L','H','e','l','i',0xF4,0x7D};
        uint32_t t0 = micros();
        BL_SendBuf(BootInit, 17);
        uint32_t t_sent = micros() - t0;
        // obserwuj linie 30 ms: zbocza opadajace i czas do pierwszego low
        int edges = 0, last = 1; uint32_t first_low = 0; t0 = micros();
        uint8_t rx[9]; int nrx = 0;
        while (micros() - t0 < 200000) {
            int v = ESC_IS_HI();
            if (last && !v) { edges++; if (!first_low) first_low = micros() - t0; 
                // sprobuj odczytac bajty UART od tego zbocza
                uint32_t btime = micros() + START_BIT_TIME; uint16_t bm = 0;
                while ((int32_t)(micros() - btime) < 0);
                for (int b = 0; b < 10; b++) { if (ESC_IS_HI()) bm |= (1 << b); btime += BIT_TIME; if (b < 9) while ((int32_t)(micros() - btime) < 0); }
                if (nrx < 9) rx[nrx++] = (uint8_t)(bm >> 1);
                v = ESC_IS_HI();
            }
            last = v;
        }
        if (edges == 0 && mode >= 2) continue;
        if (edges) got++;
        n += snprintf(out + n, outlen - n, "t=%lus tryb %d hold=%dms proba %d: idle=%d zbocza_w_hold=%d tx=%luus zbocza=%d pierwsze_low=%luus rx=", (unsigned long)((millis() - t_start) / 1000), mode, hold_ms, attempt + 1, idle, hold_edges, (unsigned long)t_sent, edges, (unsigned long)first_low);
        for (int i = 0; i < nrx && n < outlen - 8; i++) n += snprintf(out + n, outlen - n, "%02X ", rx[i]);
        n += snprintf(out + n, outlen - n, "%s", (nrx >= 3 && rx[0] == '4' && rx[1] == '7' && rx[2] == '1') ? " <- BOOTLOADER OK" : "");
        n += snprintf(out + n, outlen - n, "\n");
        
    }
    s_inv = false; gpio_set_pulls(s_pins[esc], true, false);
    n += snprintf(out + n, outlen - n, "koniec: %lu s, odpowiedzi z ESC: %d\n", (unsigned long)((millis() - t_start) / 1000), got);
    return n;
}

// ---------------- protokol 4-way (host) ----------------
#define SERIAL_4WAY_INTERFACE_NAME_STR "m4wFCIntf"
#define SERIAL_4WAY_VERSION ((uint16_t)(20 * 1000 + 0 * 100 + 6))
#define SERIAL_4WAY_VERSION_HI ((uint8_t)(SERIAL_4WAY_VERSION / 100))
#define SERIAL_4WAY_VERSION_LO ((uint8_t)(SERIAL_4WAY_VERSION % 100))
#define SERIAL_4WAY_PROTOCOL_VER 108

#define CMD_TIMEOUT_US  50000
#define ARG_TIMEOUT_US  25000
#define DAT_TIMEOUT_US  10000
#define CRC_TIMEOUT_US  10000

#define cmd_Remote_Escape 0x2E
#define cmd_Local_Escape  0x2F
#define cmd_InterfaceTestAlive 0x30
#define cmd_ProtocolGetVersion 0x31
#define cmd_InterfaceGetName 0x32
#define cmd_InterfaceGetVersion 0x33
#define cmd_InterfaceExit 0x34
#define cmd_DeviceReset 0x35
#define cmd_DeviceInitFlash 0x37
#define cmd_DeviceEraseAll 0x38
#define cmd_DevicePageErase 0x39
#define cmd_DeviceRead 0x3A
#define cmd_DeviceWrite 0x3B
#define cmd_DeviceC2CK_LOW 0x3C
#define cmd_DeviceReadEEprom 0x3D
#define cmd_DeviceWriteEEprom 0x3E
#define cmd_InterfaceSetMode 0x3F
#define cmd_DeviceVerify 0x40

#define ACK_OK                  0x00
#define ACK_I_INVALID_CMD       0x02
#define ACK_I_INVALID_CRC       0x03
#define ACK_I_VERIFY_ERROR      0x04
#define ACK_I_INVALID_CHANNEL   0x08
#define ACK_I_INVALID_PARAM     0x09
#define ACK_D_GENERAL_ERROR     0x0F

static uint16_t _crc_xmodem_update(uint16_t crc, uint8_t data) {
    crc = crc ^ ((uint16_t)data << 8);
    for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    return crc;
}

static u8_16 CRC_in, CRCout;
// zwraca true = timeout
static bool ReadByteCrc(uint8_t *data, uint32_t timeout_us) {
    if (!in_byte(data, timeout_us)) return true;
    CRC_in.word = _crc_xmodem_update(CRC_in.word, *data);
    return false;
}
static void WriteByteCrc(uint8_t b) { out_byte(b); CRCout.word = _crc_xmodem_update(CRCout.word, b); }

#define SET_DISCONNECTED DeviceInfo.words[0] = 0
#define INTF_MODE_IDX 3
#define DeviceInfoSize 4

void fourway_task(void) {
    static uint8_t ParamBuf[256];
    uint8_t ESC, I_PARAM_LEN, CMD, ACK_OUT, O_PARAM_LEN, *O_PARAM;
    u8_16 CRC_check, Dummy;
    ioMem_t ioMem;
    SET_DISCONNECTED; CurrentInterfaceMode = imARM_BLB; selected_esc = 0;
    for (int i = 0; i < s_n; i++) {
        gpio_set_function(s_pins[i], GPIO_FUNC_SIO);
        gpio_put(s_pins[i], 1); gpio_set_dir(s_pins[i], GPIO_IN); gpio_pull_up(s_pins[i]);
        s_quiet_since[i] = millis();
    }
    bool isExitScheduled = false;
    while (!isExitScheduled && !fourway_exit_req) {
        bool timedOut = false;
        do {
            CRC_in.word = 0;
            if (ReadByteCrc(&ESC, 0)) return;        // exit zadany z core0
        } while (ESC != cmd_Local_Escape);
        Dummy.word = 0; O_PARAM = &Dummy.bytes[0]; O_PARAM_LEN = 1;
        ioMem.D_FLASH_ADDR_H = ioMem.D_FLASH_ADDR_L = 0; CMD = 0; I_PARAM_LEN = 0; CRC_check.word = 0;
        timedOut = ReadByteCrc(&CMD, CMD_TIMEOUT_US) ||
                   ReadByteCrc(&ioMem.D_FLASH_ADDR_H, ARG_TIMEOUT_US) ||
                   ReadByteCrc(&ioMem.D_FLASH_ADDR_L, ARG_TIMEOUT_US) ||
                   ReadByteCrc(&I_PARAM_LEN, ARG_TIMEOUT_US);
        if (!timedOut) {
            uint8_t i = I_PARAM_LEN; uint8_t *InBuff = ParamBuf;
            do { timedOut = ReadByteCrc(InBuff++, DAT_TIMEOUT_US); } while ((--i > 0) && !timedOut);
            for (int8_t k = 1; (k >= 0) && !timedOut; k--) timedOut = !in_byte(&CRC_check.bytes[k], CRC_TIMEOUT_US);
        }
        ACK_OUT = ((CRC_check.word == CRC_in.word) && !timedOut) ? ACK_OK : ACK_I_INVALID_CRC;
        if (ACK_OUT == ACK_OK) {
            ioMem.D_PTR_I = ParamBuf;
            switch (CMD) {
            case cmd_InterfaceTestAlive:
                if (isMcuConnected()) {
                    if (!BL_SendCMDKeepAlive()) ACK_OUT = ACK_D_GENERAL_ERROR;
                    if (ACK_OUT != ACK_OK) SET_DISCONNECTED;
                }
                break;
            case cmd_ProtocolGetVersion: Dummy.bytes[0] = SERIAL_4WAY_PROTOCOL_VER; break;
            case cmd_InterfaceGetName:
                O_PARAM_LEN = (uint8_t)strlen(SERIAL_4WAY_INTERFACE_NAME_STR);
                O_PARAM = (uint8_t *)SERIAL_4WAY_INTERFACE_NAME_STR; break;
            case cmd_InterfaceGetVersion:
                O_PARAM_LEN = 2; Dummy.bytes[0] = SERIAL_4WAY_VERSION_HI; Dummy.bytes[1] = SERIAL_4WAY_VERSION_LO; break;
            case cmd_InterfaceExit: isExitScheduled = true; break;
            case cmd_InterfaceSetMode:
                if (((ParamBuf[0] <= imATM_BLB) || (ParamBuf[0] == imARM_BLB)) && (ParamBuf[0] >= imSIL_BLB)) CurrentInterfaceMode = ParamBuf[0];
                else ACK_OUT = ACK_I_INVALID_PARAM;
                break;
            case cmd_DeviceReset: {
                bool rebootEsc = false;
                if (ParamBuf[0] < s_n) { selected_esc = ParamBuf[0]; if (ioMem.D_FLASH_ADDR_L == 1) rebootEsc = true; }
                else { ACK_OUT = ACK_I_INVALID_CHANNEL; break; }
                BL_SendCMDRunRestartBootloader(&DeviceInfo);
                if (rebootEsc) {
                    ESC_OUTPUT(); ESC_SET_LO();
                    uint32_t m = millis(); while (millis() - m < 300);
                    ESC_SET_HI(); ESC_INPUT();
                }
                SET_DISCONNECTED;
                break;
            }
            case cmd_DeviceInitFlash:
                SET_DISCONNECTED;
                if (ParamBuf[0] < s_n) selected_esc = ParamBuf[0];
                else { ACK_OUT = ACK_I_INVALID_CHANNEL; break; }
                O_PARAM_LEN = DeviceInfoSize; O_PARAM = (uint8_t *)&DeviceInfo;
                // BLHeli_32 wchodzi w bootloader dopiero po >2 s linii wysokiej bez sygnalu
                {
                    bool ok = false;
                    for (int attempt = 0; attempt < 2 && !ok && !fourway_exit_req; attempt++) {
                        while (millis() - s_quiet_since[selected_esc] < 5000) { if (fourway_exit_req) break; }
                        ok = Connect(&DeviceInfo);
                    }
                    if (ok) DeviceInfo.bytes[INTF_MODE_IDX] = CurrentInterfaceMode;
                    else { SET_DISCONNECTED; ACK_OUT = ACK_D_GENERAL_ERROR; }
                }
                break;
            case cmd_DevicePageErase:
                if (CurrentInterfaceMode == imSIL_BLB || CurrentInterfaceMode == imARM_BLB) {
                    Dummy.bytes[0] = ParamBuf[0];
                    ioMem.D_FLASH_ADDR_H = (uint8_t)(CurrentInterfaceMode == imARM_BLB ? (Dummy.bytes[0] << 2) : (Dummy.bytes[0] << 1));
                    ioMem.D_FLASH_ADDR_L = 0;
                    if (!BL_PageErase(&ioMem)) ACK_OUT = ACK_D_GENERAL_ERROR;
                } else ACK_OUT = ACK_I_INVALID_CMD;
                break;
            case cmd_DeviceRead:
                ioMem.D_NUM_BYTES = ParamBuf[0];
                if (!BL_ReadFlash(CurrentInterfaceMode, &ioMem)) ACK_OUT = ACK_D_GENERAL_ERROR;
                if (ACK_OUT == ACK_OK) { O_PARAM_LEN = ioMem.D_NUM_BYTES; O_PARAM = ParamBuf; }
                break;
            case cmd_DeviceReadEEprom:
                ioMem.D_NUM_BYTES = ParamBuf[0];
                if (CurrentInterfaceMode == imATM_BLB) { if (!BL_ReadEEprom(&ioMem)) ACK_OUT = ACK_D_GENERAL_ERROR; }
                else ACK_OUT = ACK_I_INVALID_CMD;
                if (ACK_OUT == ACK_OK) { O_PARAM_LEN = ioMem.D_NUM_BYTES; O_PARAM = ParamBuf; }
                break;
            case cmd_DeviceWrite:
                ioMem.D_NUM_BYTES = I_PARAM_LEN;
                if (!BL_WriteFlash(&ioMem)) ACK_OUT = ACK_D_GENERAL_ERROR;
                break;
            case cmd_DeviceWriteEEprom:
                ioMem.D_NUM_BYTES = I_PARAM_LEN;
                if (CurrentInterfaceMode == imATM_BLB) ACK_OUT = BL_WriteEEprom(&ioMem) ? ACK_OK : ACK_D_GENERAL_ERROR;
                else ACK_OUT = ACK_I_INVALID_CMD;
                break;
            case cmd_DeviceVerify:
                if (CurrentInterfaceMode == imARM_BLB) {
                    ioMem.D_NUM_BYTES = I_PARAM_LEN;
                    switch (BL_VerifyFlash(&ioMem)) {
                        case brSUCCESS: ACK_OUT = ACK_OK; break;
                        case brERRORVERIFY: ACK_OUT = ACK_I_VERIFY_ERROR; break;
                        default: ACK_OUT = ACK_D_GENERAL_ERROR; break;
                    }
                } else ACK_OUT = ACK_I_INVALID_CMD;
                break;
            default: ACK_OUT = ACK_I_INVALID_CMD;
            }
        }
        CRCout.word = 0;
        WriteByteCrc(cmd_Remote_Escape);
        WriteByteCrc(CMD);
        WriteByteCrc(ioMem.D_FLASH_ADDR_H);
        WriteByteCrc(ioMem.D_FLASH_ADDR_L);
        WriteByteCrc(O_PARAM_LEN);
        uint8_t i = O_PARAM_LEN;
        do { WriteByteCrc(*O_PARAM); O_PARAM++; i--; } while (i > 0);
        WriteByteCrc(ACK_OUT);
        out_byte(CRCout.bytes[1]);
        out_byte(CRCout.bytes[0]);
    }
}
