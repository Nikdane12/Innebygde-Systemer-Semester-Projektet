/*
 * main.c - Oppgave 8: RPi sender tekst, AVR printer til terminal
 * 16 MHz external crystal, 38400 baud
 * USART3 (PORTB PB0) = PC terminal
 * USART2 (PORTF ALT1 PF4/PF5) = RPi
 */
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/cpufunc.h>
#include <stdio.h>
#include <stdint.h>
#include "usart.h"

#define RX_BUF_SIZE 32

static void xosc_16MHz_init(void){
    ccp_write_io((void*)&CLKCTRL.XOSCHFCTRLA,
        CLKCTRL_RUNSTBY_bm |
        CLKCTRL_FRQRANGE_16M_gc |
        CLKCTRL_SELHF_XTAL_gc |
        CLKCTRL_CSUTHF_4K_gc |
        CLKCTRL_ENABLE_bm);
    while(!(CLKCTRL.MCLKSTATUS & CLKCTRL_EXTS_bm));
    ccp_write_io((void*)&CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_EXTCLK_gc);
    while(CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm);
}

int main(void){
    xosc_16MHz_init();
    usart_init();

    printf("Waiting for RPi...\r\n");

    char buf[RX_BUF_SIZE];
    uint8_t i = 0;

    for(;;){
        /* Block until a byte arrives from RPi */
        while(!(USART2.STATUS & USART_RXCIF_bm)){}
        char c = USART2.RXDATAL;

        if (c == '\r' || c == '\n'){
            if (i > 0){
                buf[i] = '\0';
                printf("RPi: %s\r\n", buf);
                i = 0;
            }
        } else if (i < RX_BUF_SIZE - 1){
            buf[i++] = c;
        }
    }
}
