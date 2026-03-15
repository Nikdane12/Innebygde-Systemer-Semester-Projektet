/*
 * 38400 baud
 *
 *   "ADC\n"     
 *   "TMP\n"       
 *   "LED:n\n"     
 *   "SERVO:X\n"     
 *   "BUZZ:X\n"      
 *   "BUZZ:0\n"        
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

//LED
#define LED_MASK (PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm)

static void led_init(void){
    PORTC.DIRSET = LED_MASK;
    PORTC.OUTSET = LED_MASK;
}
static void led_toggle(uint8_t n){
    uint8_t pin = (1 << (n & 0x03));
    PORTC.OUTTGL = pin;
}

//Servo
static uint16_t angle_to_ticks(uint8_t deg){
    if(deg > 180) deg = 180;
    return (uint16_t)(1000u + ((uint32_t)deg * 4000u) / 180u);
}

//USART2 (RPi)
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

static void usart2_puts(const char *s){
    while(*s){
        while(!(USART2.STATUS & USART_DREIF_bm)){}
        USART2.TXDATAL = *s++;
    }
}

//Main
int main(void){
    xosc_16MHz_init();
    usart_init();
    led_init();
    buzzer_init();
    pwm_1_servo_init();
    sei();

    //Printf goes to PC terminal
    stdout = &usart3_stdout;
    printf("IO-kort ready\r\n");

    char cmd[32];

    for(;;){
        // Wait for command
        usart2_getline(cmd, sizeof(cmd));
        printf("CMD: %s\r\n", cmd);   //Echo command

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
            uint8_t n = (uint8_t)atoi(cmd + 4);
            led_toggle(n);
            usart2_puts("OK\n");
            printf("LED%d toggled\r\n", n);

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