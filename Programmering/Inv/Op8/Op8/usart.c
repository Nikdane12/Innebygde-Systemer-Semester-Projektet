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
static FILE usart3_stdout = FDEV_SETUP_STREAM(usart3_putchar, NULL, _FDEV_SETUP_WRITE);

void usart_init(void){
    USART3.BAUD  = 1667;            
    USART3.CTRLB = USART_TXEN_bm;   
    PORTB.DIRSET = PIN0_bm;         
    stdout = &usart3_stdout;        //printf -> USART3
}
