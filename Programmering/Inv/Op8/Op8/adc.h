#ifndef ADC_H
#define ADC_H

#include <stdint.h>

//ADC0 for external TMP on PD5 (AIN5):
void adc0_init_tmp_freerun(void);

//Wait for RESRDY, clear flag, return 12-bit result
uint16_t adc0_read12_wait(void);

//Convert 12-bit ADC to millivolts for 2.048 V ref
static inline uint16_t adc_to_mV_2048(uint16_t adc12){
    uint32_t mv = (uint32_t)adc12 * 2048u + 2048u;
    return (uint16_t)(mv >> 12);                   //4096
}

//10 mV/�C, 500 mV @ 0�C
static inline int16_t tmp235_C_from_mV(uint16_t mv){
    return (int16_t)(((int32_t)mv - 500) / 10);
}

#endif
