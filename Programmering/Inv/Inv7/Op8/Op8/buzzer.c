#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include "buzzer.h"
#include <util/delay.h>
#include <avr/interrupt.h>

static volatile uint8_t buzzer_running = 0;

//calculate CCMP for a given frequency
static uint16_t buzzer_ccmp_from_freq(uint16_t freq_hz)
{
    if (freq_hz < 20)    freq_hz = 20;       //cutoff very low tones
    if (freq_hz > 10000) freq_hz = 10000;    //cutoff high tones

    uint32_t denom = 4u * (uint32_t)freq_hz;			//DIV2 prescaler + toggle-per-interrupt
    uint32_t ticks = (F_CPU + (denom/2u)) / denom;		// nearest integer to F_CPU / (4*f)
    if (ticks == 0) ticks = 1;
    uint32_t ccmp = ticks - 1u;
    if (ccmp > 0xFFFFu) ccmp = 0xFFFFu;
    return (uint16_t)ccmp;
}

void buzzer_init(void)
{
    //PF2 as output
    PORTF.DIRSET = PIN2_bm;
#if BUZZER_ACTIVE_HIGH
    PORTF.OUTCLR = PIN2_bm;   //OFF = low
#else
    PORTF.OUTSET = PIN2_bm;   //OFF = high
#endif

    //Disabled until start_tone()
    TCB0.CTRLA  = 0;                          //disable while configuring
    TCB0.CTRLB  = TCB_CNTMODE_INT_gc;         //periodic interrupt (INT mode)

    TCB0.CTRLA  = TCB_CLKSEL_DIV2_gc;         
    TCB0.INTFLAGS = TCB_CAPT_bm;              //clear pending
    TCB0.INTCTRL  = TCB_CAPT_bm;              //enable interrupt
}

//Start continuous tone at freq_hz
void buzzer_start_tone(uint16_t freq_hz)
{
    uint16_t ccmp = buzzer_ccmp_from_freq(freq_hz);
    TCB0.CCMP = ccmp;
    TCB0.CNT  = 0;
    TCB0.INTFLAGS = TCB_CAPT_bm;
    TCB0.CTRLA |= TCB_ENABLE_bm;              //start TCB0
    buzzer_running = 1;
}

//Stop tone
void buzzer_stop(void)
{
    TCB0.CTRLA &= ~TCB_ENABLE_bm;             //stop TCB0
    buzzer_running = 0;
#if BUZZER_ACTIVE_HIGH
    PORTF.OUTCLR = PIN2_bm;                   //drive OFF
#else
    PORTF.OUTSET = PIN2_bm;                   //drive OFF
#endif
}

//play a single beep at freq_hz for duration_ms
void buzzer_beep(uint16_t freq_hz, uint16_t duration_ms)
{
    buzzer_start_tone(freq_hz);
    while (duration_ms--) { _delay_ms(1); }
    buzzer_stop();
}

//repeat on/off pattern
void buzzer_pattern(uint16_t freq_hz, uint8_t repeat, uint16_t on_ms, uint16_t off_ms)
{
    for (uint8_t i = 0; i < repeat; i++) {
        buzzer_start_tone(freq_hz);
        for (uint16_t t = 0; t < on_ms; t++)  _delay_ms(1);
        buzzer_stop();
        if (i + 1u < repeat) {
            for (uint16_t t = 0; t < off_ms; t++) _delay_ms(1);
        }
    }
}

//sweep from f_start to f_end in steps of step_ms each
void buzzer_sweep(uint16_t f_start_hz, uint16_t f_end_hz,
                  uint8_t steps, uint16_t step_ms)
{
    if (steps == 0) return;
    int32_t df = (int32_t)f_end_hz - (int32_t)f_start_hz;
    buzzer_start_tone(f_start_hz);
    for (uint8_t i = 1; i <= steps; i++) {
        uint16_t f = (uint16_t)((int32_t)f_start_hz + (df * i) / steps);
        TCB0.CCMP = buzzer_ccmp_from_freq(f);         //live pitch change
        for (uint16_t t = 0; t < step_ms; t++) _delay_ms(1);
    }
    buzzer_stop();
}

ISR(TCB0_INT_vect)
{
    PORTF.OUTTGL = PIN2_bm;                  //toggling generates the tone
    TCB0.INTFLAGS = TCB_CAPT_bm;             //clear interrupt
}
