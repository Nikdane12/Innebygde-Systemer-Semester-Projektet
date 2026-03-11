/*
 * pwm.c
 *
 * Created: 28.09.2023 09:33:57
 *  Author: pol022
 */ 

#include "pwm.h"
#include <avr/io.h>

/*servos*/
void pwm_1_servo_init(){
	// selecting where TCA0 WO-pins is distributed
	PORTMUX.TCAROUTEA |= PORTMUX_TCA0_PORTE_gc;
	// setting these pins as output and inverting for easier functionality
	PORTE.DIR |= (1<<0);
	PORTE.PIN0CTRL |= PORT_INVEN_bm; // transistors invert output
	// pwm_frequency should be 50 Hz
	// 16MHz / DIV8 = 2MHz timer clock -> 1ms=2000 ticks, PER=40000 -> 50Hz
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV8_gc | TCA_SINGLE_ENABLE_bm;
	// enabling output pins on compare channels and setting wave generation mode to single-slope pwm
	TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0_bm  | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
	// interrupts not necessary in this mode
	TCA0.SINGLE.PER = 40000; // this gives PWM frequency = 50Hz
}

void pwm_3_servo_init(){
	// selecting where TCA0 WO-pins is distributed
	PORTMUX.TCAROUTEA |= PORTMUX_TCA0_PORTE_gc;
	// setting these pins as output and inverting for easier functionality
	PORTE.DIR |= (1<<0) | (1<<1) | (1<<2);
	PORTE.PINCONFIG |= PORT_INVEN_bm;
	PORTE.PINCTRLSET |= (1<<0) | (1<<1) | (1<<2);
	// pwm_frequency should be 50 Hz. 4M/(1*50) > 2^16 -> DIV2
	// starting timer and selecting DIV 2 division:
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV2_gc | TCA_SINGLE_ENABLE_bm;
	// enabling output pins on compare channels and setting wave generation mode to single-slope pwm
	TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0_bm | TCA_SINGLE_CMP1_bm | TCA_SINGLE_CMP2_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
	// interrupts not necessary in this mode
	TCA0.SINGLE.PER = 40000; // this gives PWM frequency = 50Hz
}

// normally min = 1ms and max = 2ms
// 50Hz -> 20 ms, 1ms = 40000/20 = 2000
// 40000/20 * 2 = 4000
void pwm_1_servo_set_pos(uint16_t pos){
		TCA0.SINGLE.CMP0BUF = pos;
}

void pwm_3_servo_set_pos(uint16_t pos1, uint16_t pos2, uint16_t pos3){
		TCA0.SINGLE.CMP0BUF = pos1;
		TCA0.SINGLE.CMP1BUF = pos2;
		TCA0.SINGLE.CMP2BUF = pos3;
}

/* LEDS */
// 1) Good resolution
// 2) no flickering
// 3) no flickering on cameras

// average camera frame rate is 24-30 fps
// frequencies over 70 Hz should be fine


void pwm_3_leds_init(){
	// selecting where TCA0 WO-pins is distributed
	PORTMUX.TCAROUTEA |= PORTMUX_TCA0_PORTC_gc;
	// setting these pins as output and inverting for easier functionality
	PORTC.DIR |= (1<<0) | (1<<1) | (1<<2);
	PORTC.PINCONFIG |= PORT_INVEN_bm;
	PORTC.PINCTRLSET |= (1<<0) | (1<<1) | (1<<2);
	// starting timer and selecting no division: 4M/(1* 2^16) = 61 Hz
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;
	// enabling output pins on compare channels and setting wave generation mode to single-slope pwm
	TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0_bm | TCA_SINGLE_CMP1_bm | TCA_SINGLE_CMP2_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
	// interrupts not necessary in this mode
	TCA0.SINGLE.PER = 50000; // this gives PWM frequency = 80Hz
}

void pwm_4_leds_init(){
	// selecting where TCA0 WO-pins is distributed
	PORTMUX.TCAROUTEA |= PORTMUX_TCA0_PORTC_gc;
	// setting these pins as output and inverting for easier functionality
	PORTC.DIR |= (1<<0) | (1<<1) | (1<<2) | (1<<3);
	PORTC.PINCONFIG |= PORT_INVEN_bm;
	PORTC.PINCTRLSET |= (1<<0) | (1<<1) | (1<<2) | (1<<3);
	// selecting split mode on the peripheral
	TCA0.SPLIT.CTRLD |= TCA_SPLIT_ENABLE_bm;
	// starting timer and selecting no division: 4M/(64* 2^8) = 244 Hz
	TCA0.SPLIT.CTRLA = TCA_SPLIT_CLKSEL_DIV64_gc | TCA_SPLIT_ENABLE_bm;
	// selecting the outputs
	TCA0.SPLIT.CTRLB = TCA_SPLIT_LCMP0EN_bm | TCA_SPLIT_LCMP1EN_bm | TCA_SPLIT_LCMP2EN_bm | TCA_SPLIT_HCMP0EN_bm;
	// selecting pwm-frequency by manipulating the PER Registers
	TCA0.SPLIT.LPER = 255; // we want full resolution here
	TCA0.SPLIT.HPER = 255;
}

void pwm_3_leds_set_dc(uint16_t dc1, uint16_t dc2, uint16_t dc3){
	TCA0.SINGLE.CMP0BUF = dc1;
	TCA0.SINGLE.CMP1BUF = dc2;
	TCA0.SINGLE.CMP2BUF = dc3;
}

void pwm_4_leds_set_dc(uint8_t dc1, uint8_t dc2, uint8_t dc3, uint8_t dc4){
		// no buffer registers here
		// if LPER/HPER is 255 buffering will not do anything
		TCA0.SPLIT.LCMP0 = dc1;
		TCA0.SPLIT.LCMP1 = dc2;
		TCA0.SPLIT.LCMP2 = dc3;
		TCA0.SPLIT.HCMP0 = dc4;
}