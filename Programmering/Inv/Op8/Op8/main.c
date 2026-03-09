// main.c — 16 MHz, accepts "####>" OR "####<Enter>" to set servo on PE0 (P4)
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>
#include <ctype.h>
#include <avr/cpufunc.h>  
#include "usart.h"


static inline uint16_t us_to_counts(uint16_t us){ return us; }

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

static void servo_init_pe0(void){
    PORTMUX.TCAROUTEA = (PORTMUX.TCAROUTEA & (uint8_t)~0x07u) | 0x04u;

    PORTE.DIRSET   = PIN0_bm;
    PORTE.PIN0CTRL |= PORT_INVEN_bm;

    //Single-slope PWM
    TCA0.SINGLE.CTRLA = 0;
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP0EN_bm;
    TCA0.SINGLE.CTRLA |= TCA_SINGLE_CLKSEL_DIV16_gc;  //16 MHz / 16 = 1 MHz
    TCA0.SINGLE.PER    = 19999;                       //20 ms - 1
    TCA0.SINGLE.CMP0BUF = us_to_counts(1500);         //center
    TCA0.SINGLE.CTRLA  |= TCA_SINGLE_ENABLE_bm;
}

static inline void servo_set_us(uint16_t us){
    if (us < 800)  us = 800;
    if (us > 2200) us = 2200;
    TCA0.SINGLE.CMP0BUF = us;
}

static uint16_t acc = 0;
static uint8_t  have_digit = 0;
static uint8_t  skip_next_lf = 0;

static inline uint8_t usart3_rx_ready(void){ return (USART3.STATUS & USART_RXCIF_bm) != 0; }
static inline uint8_t usart3_read_byte(void){ return USART3.RXDATAL; }

static void usart_poll_and_update_servo(void){
	while (usart3_rx_ready()){
		uint8_t ch = usart3_read_byte();

		if (skip_next_lf && ch == '\n') { 
			skip_next_lf = 0; continue; 
			}

		if (ch == '>'){
			uint16_t v = have_digit ? acc : 1500;
			if (v < 800)  v = 800;
			if (v > 2200) v = 2200;
			TCA0.SINGLE.CMP0BUF = v;
			printf("OK %u us\r\n", v);

			acc = 0;
			have_digit = 0;
			skip_next_lf = (ch == '\r'); 
			continue;
		}

		if (isspace(ch)) continue; 

		if (ch >= '0' && ch <= '9'){
			have_digit = 1;
			uint32_t t = (uint32_t)acc * 10u + (uint16_t)(ch - '0');
			if (t > 10000u) t = 10000u;
			acc = (uint16_t)t;
		}
		
	}
}

int main(void){
    xosc_16Mhz_init();          
    usart_init();               
   
    PORTB.DIRCLR = PIN1_bm;    
    USART3.CTRLB |= USART_RXEN_bm;

    servo_init_pe0();

    printf("Type 800>, 1500>, 2200> ");

    for(;;){
        usart_poll_and_update_servo(); 
        _delay_ms(1);
    }
}
