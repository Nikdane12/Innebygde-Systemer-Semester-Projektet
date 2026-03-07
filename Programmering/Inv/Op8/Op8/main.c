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
#include "adc.h"

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
    usart_init();   /* sets up TX/RX on PB0/PB1, stdin, stdout, 9600 baud */

    printf("READY\r\n");

    char rx_buf[RX_BUF_SIZE];

    while (1)
    {
        recv_line(rx_buf);

        if (strcmp(rx_buf, "PING") == 0) {
            printf("PONG\r\n");
        }
        else if (strcmp(rx_buf, "ADC_TMP") == 0) {
            adc0_init_tmp_freerun();
            _delay_ms(1);
            uint16_t raw  = adc0_read12_wait();
            uint16_t mv   = adc_to_mV_2048(raw);
            int16_t  degC = tmp235_C_from_mV(mv);
            printf("TMP:%u:%d\r\n", mv, (int)degC);
        }
        else if (strcmp(rx_buf, "ADC_POT") == 0) {
            adc0_init_pot_ain4_vdd_freerun();
            _delay_ms(1);
            uint16_t raw = adc0_read12_wait();
            printf("POT:%u\r\n", raw);
        }
        else if (strcmp(rx_buf, "LED_ON") == 0) {
            PORTD.DIRSET = PIN6_bm;
            PORTD.OUTSET = PIN6_bm;
            printf("LED:ON\r\n");
        }
        else if (strcmp(rx_buf, "LED_OFF") == 0) {
            PORTD.DIRSET = PIN6_bm;
            PORTD.OUTCLR = PIN6_bm;
            printf("LED:OFF\r\n");
        }
        else {
            printf("ERR:UNKNOWN\r\n");
        }
    }
}