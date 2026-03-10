/*
 * main.c - Oppgave 8: Kommunikasjon med RPi
 * 16 MHz external crystal, 38400 baud
 * USART3 (PORTB) = PC terminal, USART2 (PORTF ALT1) = RPi
 */
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/cpufunc.h>
#include <util/delay.h>
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

/* Wait for a full line from RPi over USART2 */
static void recv_line(char *buf){
    uint8_t i = 0;
    while (i < RX_BUF_SIZE - 1){
        while(!(USART2.STATUS & USART_RXCIF_bm)){}  /* wait for byte */
        char c = USART2.RXDATAL;
        if (c == '\r' || c == '\n') break;
        buf[i++] = c;
    }
    buf[i] = '\0';
}

int main(void){
    xosc_16MHz_init();
    usart_init();

    fprintf(&usart2_stdout, "AVR READY\r\n");
    printf("AVR READY\r\n");

    char buf[RX_BUF_SIZE];

    for(;;){
        /* Send a message to RPi */
        fprintf(&usart2_stdout, "Hello from AVR\r\n");
        printf("Sent to RPi: Hello from AVR\r\n");

        _delay_ms(500);

        /* Wait for reply from RPi */
        recv_line(buf);
        fprintf(&usart2_stdout, "GOT:%s\r\n", buf);
        printf("RPi said: %s\r\n", buf);

        _delay_ms(1000);
    }
}
