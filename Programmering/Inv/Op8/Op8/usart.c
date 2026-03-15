#include "usart.h"
#include <avr/io.h>

//USART3 PC terminal
static int usart3_putchar(char c, FILE *stream){
    while(!(USART3.STATUS & USART_DREIF_bm)){}
    USART3.TXDATAL = c;
    return 0;
}

// USART2 RPI
static int usart2_putchar(char c, FILE *stream){
    while(!(USART2.STATUS & USART_DREIF_bm)){}
    USART2.TXDATAL = c;
    return 0;
}

FILE usart3_stdout = FDEV_SETUP_STREAM(usart3_putchar, NULL, _FDEV_SETUP_WRITE);
FILE usart2_stdout = FDEV_SETUP_STREAM(usart2_putchar, NULL, _FDEV_SETUP_WRITE);

void usart_init(void){
    // USART3 - PORTB PC terminal
    USART3.BAUD  = 1667;
    USART3.CTRLB = USART_TXEN_bm;
    PORTB.DIRSET = PIN0_bm; 

    // USART2 - PORTF RPI
    PORTMUX.USARTROUTEA = PORTMUX_USART2_ALT1_gc;
    USART2.BAUD  = 1667;
    USART2.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
    PORTF.DIRSET = PIN4_bm;
    PORTF.DIRCLR = PIN5_bm;    

    stdout = &usart3_stdout;
}
