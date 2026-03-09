/*
 * usart.c
 *
 * Created: 19.09.2025 12:49:40
 *  Author: olava
 */ 

#include "usart.h"
#include <avr/io.h>

/* Change USART below to match whichever port the board's USB-serial uses */
#define UART        USART0
#define UART_PORT   PORTA
#define UART_TX_PIN PIN0_bm   /* PA0 = TX */
#define UART_RX_PIN PIN1_bm   /* PA1 = RX */

static int usart_putchar(char c, FILE *stream){
    while(!(UART.STATUS & USART_DREIF_bm)){}
    UART.TXDATAL = c;
    return 0;
}

static int usart_getchar(FILE *stream){
    while(!(UART.STATUS & USART_RXCIF_bm)){}
    return UART.RXDATAL;
}

static FILE usart_stdout = FDEV_SETUP_STREAM(usart_putchar, NULL, _FDEV_SETUP_WRITE);
static FILE usart_stdin  = FDEV_SETUP_STREAM(NULL, usart_getchar, _FDEV_SETUP_READ);

void usart_puts(const char *s){
    while (*s) {
        while (!(UART.STATUS & USART_DREIF_bm)) {}
        UART.TXDATAL = *s++;
    }
}

void usart_init(void){
    UART.BAUD  = 1667;
    UART.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
    UART_PORT.DIRSET = UART_TX_PIN;
    UART_PORT.DIRCLR = UART_RX_PIN;
    stdout = &usart_stdout;
    stdin  = &usart_stdin;
}
