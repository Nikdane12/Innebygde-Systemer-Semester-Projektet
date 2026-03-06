/*
 * main.c  –  Oppgave 8: Kommunikasjon med IO-kort
 *
 * Protocol (ASCII, newline-terminated):
 *   Pi sends  →  AVR replies
 *   "ADC_TMP\n"  → "TMP:<millivolts>:<celsius>\n"
 *   "ADC_POT\n"  → "POT:<raw_adc>\n"
 *   "LED_ON\n"   → "LED:ON\n"
 *   "LED_OFF\n"  → "LED:OFF\n"
 *   "PING\n"     → "PONG\n"   (connectivity test)
 *
 * Hardware (AVR128DB-series, 4 MHz internal oscillator):
 *   TMP235 on AIN5 (PD5), 2.048 V ref
 *   Potentiometer on AIN4 (PD4), VDD ref
 *   USART3 TX on PB0  (usart.c)
 *   USART3 RX on PB1  (enabled below)
 *   Debug LED on PD6
 */

#define F_CPU 4000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "usart.h"
#include "adc.h"

/* ── USART3 blocking RX ──────────────────────────────────────────── */
static char usart3_getchar(void)
{
    while (!(USART3.STATUS & USART_RXCIF_bm)) { }
    return (char)USART3.RXDATAL;
}

static void usart_rx_enable(void)
{
    PORTB.DIRCLR  = PIN1_bm;          /* PB1 = RX input              */
    USART3.CTRLB |= USART_RXEN_bm;
}

/* ── Receive one newline-terminated line ─────────────────────────── */
#define RX_BUF_SIZE 32

static void recv_line(char *buf)
{
    uint8_t i = 0;
    char c;
    while (i < (RX_BUF_SIZE - 1)) {
        c = usart3_getchar();
        if (c == '\n' || c == '\r') break;
        buf[i++] = c;
    }
    buf[i] = '\0';
}

/* ── Main ────────────────────────────────────────────────────────── */
int main(void)
{
    usart_init();          /* sets stdout → USART3, 9600 baud         */
    usart_rx_enable();

    char rx_buf[RX_BUF_SIZE];

    printf("READY\n");    /* tell the Pi the MCU is alive             */

    while (1)
    {
        recv_line(rx_buf);

        /* Temperature measurement (TMP235 on AIN5, 2.048 V ref) */
        if (strcmp(rx_buf, "ADC_TMP") == 0) {
            adc0_init_tmp_freerun();
            _delay_ms(1);
            uint16_t raw  = adc0_read12_wait();
            uint16_t mv   = adc_to_mV_2048(raw);
            int16_t  degC = tmp235_C_from_mV(mv);
            printf("TMP:%u:%d\n", mv, (int)degC);
        }

        /* Potentiometer reading (AIN4, VDD ref) */
        else if (strcmp(rx_buf, "ADC_POT") == 0) {
            adc0_init_pot_ain4_vdd_freerun();
            _delay_ms(1);
            uint16_t raw = adc0_read12_wait();
            printf("POT:%u\n", raw);
        }

        /* LED control on PD6 */
        else if (strcmp(rx_buf, "LED_ON") == 0) {
            PORTD.DIRSET = PIN6_bm;
            PORTD.OUTSET = PIN6_bm;
            printf("LED:ON\n");
        }
        else if (strcmp(rx_buf, "LED_OFF") == 0) {
            PORTD.DIRSET = PIN6_bm;
            PORTD.OUTCLR = PIN6_bm;
            printf("LED:OFF\n");
        }

        /* Connectivity test */
        else if (strcmp(rx_buf, "PING") == 0) {
            printf("PONG\n");
        }

        else {
            printf("ERR:UNKNOWN\n");
        }
    }
}