/*
 * main.c - IO-kort: USART command handler for RPi
 * USART2 (PF4/PF5) = RPi, USART3 (PB0) = PC terminal
 * 16 MHz external crystal, 38400 baud
 *
 * Message protocol (text commands from RPi):
 *   "ADC\n"           -> measure AIN4 (pot), reply "ADC:1234\n"
 *   "TMP\n"           -> measure AIN5 (temp), reply "TMP:23\n"
 *   "LED:1\n"         -> LED0 on
 *   "LED:0\n"         -> LED0 off
 *   "SERVO:90\n"      -> set servo to 90 degrees
 *   "BUZZ:500\n"      -> buzzer at 500 Hz for 200 ms
 *   "BUZZ:0\n"        -> buzzer off
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "usart.h"
#include "adc.h"
#include "analog.h"
#include "buzzer.h"
#include "pwm.h"

/* -------- Clock -------- */
static void xosc_16MHz_init(void){
    ccp_write_io((void*)&CLKCTRL.XOSCHFCTRLA,
        CLKCTRL_RUNSTBY_bm |
        CLKCTRL_FRQRANGE_16M_gc |
        CLKCTRL_SELHF_XTAL_gc |
        CLKCTRL_CSUTHF_4K_gc |
        CLKCTRL_ENABLE_bm);
    while(!(CLKCTRL.MCLKSTATUS & CLKCTRL_EXTS_bm));
    ccp_write_io((void*)&CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_EXTCLK_gc);
    while(CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm);
}

/* -------- LED (PB3 = LED0 per schematic) -------- */
static void led_init(void){
    PORTB.DIRSET = PIN3_bm;
    PORTB.OUTCLR = PIN3_bm;   /* off */
}
static void led_set(uint8_t on){
    if(on) PORTB.OUTSET = PIN3_bm;
    else   PORTB.OUTCLR = PIN3_bm;
}

/* -------- Servo angle to PWM ticks -------- */
/* PER=40000, 1ms=2000 ticks, 2ms=4000 ticks */
static uint16_t angle_to_ticks(uint8_t deg){
    if(deg > 180) deg = 180;
    return (uint16_t)(2000u + ((uint32_t)deg * 2000u) / 180u);
}

/* -------- Receive a line from USART2 (RPi) -------- */
static uint8_t usart2_getline(char *buf, uint8_t maxlen){
    uint8_t i = 0;
    while(i < maxlen - 1){
        while(!(USART2.STATUS & USART_RXCIF_bm)){}
        char c = USART2.RXDATAL;
        if(c == '\n' || c == '\r'){
            break;
        }
        buf[i++] = c;
    }
    buf[i] = '\0';
    return i;
}

/* -------- Send string on USART2 (RPi) -------- */
static void usart2_puts(const char *s){
    while(*s){
        while(!(USART2.STATUS & USART_DREIF_bm)){}
        USART2.TXDATAL = *s++;
    }
}

/* -------- Main -------- */
int main(void){
    xosc_16MHz_init();
    usart_init();
    led_init();
    buzzer_init();
    pwm_1_servo_init();
    sei();

    /* printf goes to PC terminal on PORTB */
    stdout = &usart3_stdout;
    printf("IO-kort ready\r\n");

    char cmd[32];

    for(;;){
        /* Wait for command from RPi on USART2 */
        usart2_getline(cmd, sizeof(cmd));
        printf("CMD: %s\r\n", cmd);   /* echo to PC terminal for debugging */

        if(strcmp(cmd, "ADC") == 0){
            adc0_init_pot_ain4_vdd_freerun();
            uint16_t raw = adc0_read12_wait();
            char reply[16];
            snprintf(reply, sizeof(reply), "ADC:%u\n", raw);
            usart2_puts(reply);
            printf("ADC: %u\r\n", raw);

        } else if(strcmp(cmd, "TMP") == 0){
            adc0_init_tmp_freerun();
            uint16_t raw = adc0_read12_wait();
            uint16_t mv  = adc_to_mV_2048(raw);
            int16_t  deg = tmp235_C_from_mV(mv);
            char reply[16];
            snprintf(reply, sizeof(reply), "TMP:%d\n", deg);
            usart2_puts(reply);
            printf("TMP: %d C\r\n", deg);

        } else if(strncmp(cmd, "LED:", 4) == 0){
            uint8_t on = (uint8_t)atoi(cmd + 4);
            led_set(on);
            usart2_puts("OK\n");
            printf("LED: %d\r\n", on);

        } else if(strncmp(cmd, "SERVO:", 6) == 0){
            uint8_t deg = (uint8_t)atoi(cmd + 6);
            pwm_1_servo_set_pos(angle_to_ticks(deg));
            usart2_puts("OK\n");
            printf("SERVO: %d deg\r\n", deg);

        } else if(strncmp(cmd, "BUZZ:", 5) == 0){
            uint16_t freq = (uint16_t)atoi(cmd + 5);
            if(freq == 0){
                buzzer_stop();
            } else {
                buzzer_beep(freq, 200);
            }
            usart2_puts("OK\n");
            printf("BUZZ: %u Hz\r\n", freq);

        } else {
            usart2_puts("ERR\n");
            printf("Unknown: %s\r\n", cmd);
        }
    }
}