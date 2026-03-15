#include <avr/io.h>
#include "analog.h"
#include <avr/interrupt.h>
#include <util/delay.h>
#include "usart.h"

//linear transform from the set A to the set B with scaling accordingly
int32_t linear_map(int32_t value, int32_t a_min,int32_t a_max,int32_t b_min, int32_t b_max){
	return b_min + (b_max - b_min)*(value - a_min)/(a_max - a_min);
}

// dac functions
void dac_init(){
	VREF.DAC0REF |= VREF_REFSEL_VDD_gc;
	_delay_us(25);
	PORTD.PIN6CTRL &= ~PORT_ISC_gm;
	PORTD.PIN6CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	PORTD.PIN6CTRL &= ~PORT_PULLUPEN_bm;
	//DAC0.CTRLA |= (1<<DAC_OUTEN_bp) | (1<<DAC_ENABLE_bp);
	DAC0.CTRLA |= DAC_ENABLE_bm | DAC_RUNSTDBY_bm;
}
void dac_set_lvl(uint16_t value){
	/* Store the two LSBs in DAC0.DATAL at MSB-location in register */
	DAC0.DATAL = (value & 0x03) << 6;
	/* Store the eight MSBs in DAC0.DATAH */
	DAC0.DATAH = value >> 2;
}

// adc functions
void adc_init_freerunning(uint8_t channel){
		VREF.ADC0REF |= VREF_REFSEL_VDD_gc;
		_delay_us(25);
		ADC0.CTRLA |= ADC_FREERUN_bm;
		ADC0.CTRLC = ADC_PRESC_DIV16_gc;
		ADC0.MUXPOS = channel;
		ADC0.CTRLA |= ADC_ENABLE_bm;
		ADC0.COMMAND |= ADC_STCONV_bm; // starting _first_ conversion (Free-running mode)
		//ADC0.INTCTRL |= ADC_RESRDY_bm;
}

void adc_init_single_conversions(uint8_t channel){
	VREF.ADC0REF |= VREF_REFSEL_VDD_gc;
	_delay_us(25);
	ADC0.CTRLC |= ADC_PRESC_DIV2_gc;
	ADC0.MUXPOS = channel;
	ADC0.CTRLA |= ADC_ENABLE_bm;	
	ADC0.INTCTRL |= ADC_RESRDY_bm; // interrupt on result ready
	//ADC0.COMMAND |= ADC_STCONV_bm; // starting _first_ conversion
}

void adc_init_interrupt_on_change(uint8_t channel){
	VREF.ADC0REF |= VREF_REFSEL_VDD_gc;
	_delay_us(25);
	ADC0.CTRLC |= ADC_PRESC_DIV2_gc;
	ADC0.MUXPOS = channel;
	ADC0.MUXNEG = ADC_MUXNEG_DAC0_gc;
	ADC0.WINLT = 2000;
	ADC0.WINHT = 2003;
	ADC0.CTRLA |= ADC_ENABLE_bm | ADC_CONVMODE_bm | ADC_RESSEL_10BIT_gc;
	ADC0.CTRLE |= ADC_WINCM_OUTSIDE_gc;
	ADC0.INTCTRL |= ADC_RESRDY_bm | ADC_WCMP_bm; // interrupt on result ready, and if 
	ADC0.COMMAND |= ADC_STCONV_bm; // starting _first_ conversion
}



ISR(ADC0_WCMP_vect){
	outside = 1;
	//ADC0.INTFLAGS |= ADC_WCMP_bm;
	result = ADC0.RES;
}


ISR(ADC0_RESRDY_vect){
	result = ADC0.RES;
	//ADC0.COMMAND |= ADC_STCONV_bm;// starting again.
	res_ready = 1;
}

void adc_switch_channel(uint8_t channel){
	ADC0.MUXPOS = channel;
}
	
// polling!
uint16_t adc_get_result(){
	while(!(ADC0.INTFLAGS & ADC_RESRDY_bm))// waiting
		;
	return ADC0.RES;
}














// temperatur sensor
void adc_temp_init(){
	t_offset = SIGROW.TEMPSENSE1;
	t_slope = SIGROW.TEMPSENSE0;
	VREF.ADC0REF |= VREF_REFSEL_2V048_gc;
	_delay_us(25);
	ADC0.CTRLA |= ADC_FREERUN_bm;
	ADC0.CTRLC = ADC_PRESC_DIV16_gc;
	ADC0.MUXPOS = ADC_MUXPOS_TEMPSENSE_gc;
	ADC0.CTRLD |= ADC_INITDLY_DLY128_gc;
	ADC0.CTRLB |= ADC_SAMPNUM_ACC4_gc;
	ADC0.SAMPCTRL = 8; // calculated from 28us*F_CPU/DIV (+1)
	ADC0.CTRLA |= ADC_ENABLE_bm;
	ADC0.COMMAND |= ADC_STCONV_bm; // starting _first_ conversion (Free-running mode)
	
}
int16_t adc_get_temp_celsius_rounded(){
	uint16_t raw_data = adc_get_result();
	uint32_t tempK = t_offset - raw_data;
	tempK *= t_slope;
	tempK += 0x0800; // rounding (setting msb=1)
	tempK >>=12; // rounding p. 2
	return (int16_t)tempK - 273;
}

void adc_get_temp_celsius(int16_t *integer_value, uint16_t *decimals){
	uint16_t raw_data = adc_get_result()/4;
	uint32_t tempK_int = t_offset - raw_data;
	uint32_t tmp1;
	tempK_int *= t_slope;
	tempK_int += 2048;
	tmp1 = tempK_int;
	tempK_int >>=12;//rounding to whole deg.
	// three decimals
	//*decimals = tmp1>>10;
	*integer_value = tempK_int - 273;
	*decimals = ((tmp1 & 2)>>1)*50;
	*decimals += (tmp1 & 1)*25;
}

void adc_init_mic(){
	VREF.ADC0REF |= VREF_REFSEL_VDD_gc;
	_delay_us(25);
	ADC0.CTRLA |= ADC_FREERUN_bm;
	//ADC0.CTRLC = ADC_PRESC_DIV16_gc;
	ADC0.CTRLC = ADC_PRESC_DIV2_gc;
	ADC0.MUXPOS = 7;
	ADC0.CTRLA |= ADC_ENABLE_bm;
	ADC0.COMMAND |= ADC_STCONV_bm; // starting _first_ conversion (Free-running mode)
	ADC0.INTCTRL |= ADC_WCMP_bm;
	ADC0.CTRLE |= ADC_WINCM_ABOVE_gc;
	ADC0.WINHT = 2000;
}