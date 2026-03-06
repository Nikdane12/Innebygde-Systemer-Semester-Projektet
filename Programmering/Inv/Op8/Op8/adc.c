/*
 * adc.c
 * Two sections: TMP on AIN5 (2.048 V) and POT on AIN4 (VDD)
 */

#include "adc.h"
#include <avr/io.h>
#include <stdint.h>

#define F_CPU 4000000UL
#include <util/delay.h>

/* -------- Section 1: Temperature reader (AIN5, 2.048 V) -------- */
void adc0_init_tmp_freerun(void)
{
    VREF.ADC0REF = VREF_REFSEL_2V048_gc;   // internal 2.048 V reference
    _delay_us(50);                          // settle time

    ADC0.CTRLC   = ADC_PRESC_DIV16_gc;     // 4 MHz / 16 = 250 kHz
    ADC0.SAMPCTRL = 10;

    ADC0.MUXPOS   = ADC_MUXPOS_AIN5_gc;    // external TMP on AIN5
    ADC0.CTRLA    = ADC_ENABLE_bm | ADC_FREERUN_bm;
    ADC0.COMMAND  = ADC_STCONV_bm;         // start free-run
}

/* -------- Section 2: Potentiometer task (AIN4, VDD) -------- */
void adc0_init_pot_ain4_vdd_freerun(void)
{
    // Optional: disable digital input on PD4 (AIN4) for cleaner analog reads
#if defined(PORTD) && defined(PORT_ISC_INPUT_DISABLE_gc)
    PORTD.PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc;
#endif

    VREF.ADC0REF = VREF_REFSEL_VDD_gc;     // use VDD as reference (full-scale = VDD)
    _delay_us(50);

    ADC0.CTRLC   = ADC_PRESC_DIV16_gc;     // 4 MHz / 16 = 250 kHz
    ADC0.SAMPCTRL = 10;

    ADC0.MUXPOS   = ADC_MUXPOS_AIN4_gc;    // R1 on AIN4
    ADC0.CTRLA    = ADC_ENABLE_bm | ADC_FREERUN_bm;
    ADC0.COMMAND  = ADC_STCONV_bm;         // start free-run
}

/* -------- Common read (12-bit) -------- */
uint16_t adc0_read12_wait(void)
{
    while(!(ADC0.INTFLAGS & ADC_RESRDY_bm)) { }
    ADC0.INTFLAGS = ADC_RESRDY_bm;         // clear ready flag
    return ADC0.RES;                        // 12-bit value (0..4095)
}
