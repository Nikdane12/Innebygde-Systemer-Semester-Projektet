/*
 * main.c - Forwards USART3 (PORTB RX) to USART2 (PORTF TX)
 * 16 MHz external crystal, 38400 baud
 */
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/cpufunc.h>
#include <util/delay.h>
#include <stdio.h>
#include "usart.h"

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

	for(;;){
		/* Send on USART3 (loopback receives it internally) */
		printf("Hello from AVR\r\n");

		/* Forward each received byte to USART2 (PORTF) */
		while(USART3.STATUS & USART_RXCIF_bm){
			char c = USART3.RXDATAL;
			while(!(USART2.STATUS & USART_DREIF_bm)){}
			USART2.TXDATAL = c;
		}
		_delay_ms(1000);
	}
}