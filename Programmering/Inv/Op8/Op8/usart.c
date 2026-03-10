/*
 * usart.c
 *
 * Created: 19.09.2025 12:49:40
 *  Author: olava
 */
#include "usart.h"
#include <avr/io.h>

/* USART3 - PORTB */
static int usart3_putchar(char c, FILE *stream){
    while(!(USART3.STATUS & USART_DREIF_bm)){}
    USART3.TXDATAL = c;
    return 0;
}

/* USART2 - PORTF ALT1 (PF4/PF5) */
static int usart2_putchar(char c, FILE *stream){
    while(!(USART2.STATUS & USART_DREIF_bm)){}
    USART2.TXDATAL = c;
    return 0;
}

/* Streams declared AFTER the functions they reference */
FILE usart3_stdout = FDEV_SETUP_STREAM(usart3_putchar, NULL, _FDEV_SETUP_WRITE);
FILE usart2_stdout = FDEV_SETUP_STREAM(usart2_putchar, NULL, _FDEV_SETUP_WRITE);

void usart_init(void){
	/* USART3 - PORTB with internal loopback */
	USART3.BAUD  = 1667;
	USART3.CTRLA = USART_LBME_bm;                  /* internal loopback */
	USART3.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
	PORTB.DIRSET = PIN0_bm;    /* PB0 = TX */
	PORTB.DIRCLR = PIN1_bm;    /* PB1 = RX */

	/* USART2 - PORTF ALT1 */
	PORTMUX.USARTROUTEA = PORTMUX_USART2_ALT1_gc;
	USART2.BAUD  = 1667;
	USART2.CTRLB = USART_TXEN_bm;
	PORTF.DIRSET = PIN4_bm;    /* PF4 = TX */

	stdout = &usart3_stdout;
}