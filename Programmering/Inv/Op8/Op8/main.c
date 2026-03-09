/*
 * main.c - Oppgave 8: Kommunikasjon med IO-kort
 * 4 MHz internal oscillator, 9600 baud
 * Uses usart.c which already sets up TX, RX, stdin and stdout
 */
#define F_CPU 4000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "usart.h"

#define RX_BUF_SIZE 32

static void recv_line(char *buf)
{
    uint8_t i = 0;
    int c;
    while (i < (RX_BUF_SIZE - 1)) {
        c = getchar();              /* uses stdin from usart.c */
        if (c == '\n' || c == '\r') break;
        buf[i++] = (char)c;
    }
    buf[i] = '\0';
}

int main(void)
{
    usart_init();   /* sets up TX/RX on PF0/PF1, stdin, stdout, 9600 baud */

    usart_puts("READY\r\n");

    char rx_buf[RX_BUF_SIZE];

    while (1)
    {
        recv_line(rx_buf);
        usart_puts("GOT:");
        usart_puts(rx_buf);
        usart_puts("\r\n");
    }
}