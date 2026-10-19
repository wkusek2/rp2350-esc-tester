#pragma once
#include <stdint.h>
#include <stdbool.h>
// Interfejs 4-way (BLHeliSuite32 <-> bootloader ESC przez linie sygnalowa silnika).
// Port z Betaflight (io/serial_4way*.c, GPL-3.0, autor 4712).
//
// core0: pompuje bajty USB <-> bufory (fourway_host_put / fourway_host_get)
// core1: fourway_task() - parsowanie ramek i bit-bang do ESC (bez przerwan)

void fourway_init(const uint8_t *pins, int n);
void fourway_reset(void);             // wywolac PRZED wejsciem w tryb (czysci bufory)
void fourway_host_put(uint8_t b);     // bajt od hosta (USB) -> core1
int  fourway_host_get(void);          // bajt do hosta, -1 gdy brak
void fourway_task(void);              // blokuje do cmd_InterfaceExit lub fourway_exit_req
extern volatile bool fourway_exit_req;
int  fourway_probe(int esc, int mode, int hold_ms, char *out, int outlen);   // diagnostyka bootloadera (core1)
