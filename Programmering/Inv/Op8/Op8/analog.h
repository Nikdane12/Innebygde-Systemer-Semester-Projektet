#ifndef ANALOG_H_
#define ANALOG_H_

volatile uint16_t result;
volatile uint8_t res_ready;
volatile uint8_t outside;

//linear transform from the set A to the set B with scaling accordingly
int32_t linear_map(int32_t value, int32_t a_min,int32_t a_max,int32_t b_min, int32_t b_max);

void ac_init_interrupt_on_change(uint8_t channel);

// dac functions
void dac_init();
void dac_set_lvl(uint16_t value);

// adc functions
void adc_init_freerunning(uint8_t channel);
void adc_init_single_conversions(uint8_t channel);
void adc_switch_channel(uint8_t channel);
uint16_t adc_get_result();

// microphone
volatile uint8_t sound_detected;
void adc_init_mic();

// Temperature sensor calibration data
static uint16_t t_offset;
static uint16_t t_slope;
// temperature sensor
void adc_temp_init();
int16_t adc_get_temp_celsius_rounded();
void adc_get_temp_celsius(int16_t *integer_value, uint16_t *decimals);

#endif /* ANALOG_H_ */
