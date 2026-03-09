/*
 * usart.c
 *
 * Created: 19.09.2025 12:49:40
 *  Author: olava
 */ 

#include "usart.h"
#include <avr/io.h>

static int usart3_putchar(char c, FILE *stream){
    while(!(USART3.STATUS & USART_DREIF_bm)){}
    USART3.TXDATAL = c;
    return 0;
}

static int usart3_getchar(FILE *stream){
    while(!(USART3.STATUS & USART_RXCIF_bm)){}
    return USART3.RXDATAL;
}

static FILE usart3_stdout = FDEV_SETUP_STREAM(usart3_putchar, NULL, _FDEV_SETUP_WRITE);
static FILE usart3_stdin  = FDEV_SETUP_STREAM(NULL, usart3_getchar, _FDEV_SETUP_READ);

void usart_puts(const char *s){
    while (*s) {
        while (!(USART3.STATUS & USART_DREIF_bm)) {}
        USART3.TXDATAL = *s++;
    }
}

void usart_init(void){
    USART3.BAUD  = 1667;
    USART3.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
    PORTF.DIRSET = PIN0_bm;         // PF0 = TX output
    PORTF.DIRCLR = PIN1_bm;         // PF1 = RX input
    stdout = &usart3_stdout;
    stdin  = &usart3_stdin;
}
