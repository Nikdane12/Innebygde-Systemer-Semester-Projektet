#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/cpufunc.h>
#include <stdio.h>
#include "usart.h"

static void xosc_16Mhz_init(void){
    ccp_write_io((void*)&CLKCTRL.XOSCHFCTRLA,
        CLKCTRL_RUNSTBY_bm | CLKCTRL_FRQRANGE_16M_gc |
        CLKCTRL_SELHF_XTAL_gc | CLKCTRL_CSUTHF_4K_gc | CLKCTRL_ENABLE_bm);
    while(!(CLKCTRL.MCLKSTATUS & CLKCTRL_EXTS_bm));
    ccp_write_io((void*)&CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_EXTCLK_gc);
    while((CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm));
}

int main(void){
    xosc_16Mhz_init();
    usart_init();

    printf("READY\r\n");

    while(1){
        // Wait for a character
        int c = getchar();
        // Echo it back
        putchar(c);
    }
}