/*
 * main.c - Oppgave 8: Kommunikasjon med IO-kort
 * 16 MHz external crystal, 9600 baud, USART3 (PF0/PF1)
 */
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/cpufunc.h>
#include <stdio.h>
#include <stdint.h>
#include "usart.h"

#define RX_BUF_SIZE 32

static void xosc_16Mhz_init(void){
    ccp_write_io((void*)&CLKCTRL.XOSCHFCTRLA,
        CLKCTRL_RUNSTBY_bm |
        CLKCTRL_FRQRANGE_16M_gc |
        CLKCTRL_SELHF_XTAL_gc |
        CLKCTRL_CSUTHF_4K_gc |
        CLKCTRL_ENABLE_bm);
    while(!(CLKCTRL.MCLKSTATUS & CLKCTRL_EXTS_bm));
    ccp_write_io((void*)&CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_EXTCLK_gc);
    while((CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm));
}

static inline uint8_t usart3_rx_ready(void){ return (USART3.STATUS & USART_RXCIF_bm) != 0; }
static inline uint8_t usart3_read_byte(void){ return USART3.RXDATAL; }

int main(void){
    xosc_16Mhz_init();
    usart_init();
    PORTB.DIRCLR = PIN1_bm;
    USART3.CTRLB |= USART_RXEN_bm;

    printf("READY\r\n");

    char buf[RX_BUF_SIZE];
    uint8_t i = 0;

    for(;;){
        if (usart3_rx_ready()){
            uint8_t c = usart3_read_byte();
            if (c == '\r' || c == '\n'){
                if (i > 0){
                    buf[i] = '\0';
                    printf("GOT:%s\r\n", buf);
                    i = 0;
                }
            } else if (i < RX_BUF_SIZE - 1){
                buf[i++] = c;
            }
        }
    }
}
