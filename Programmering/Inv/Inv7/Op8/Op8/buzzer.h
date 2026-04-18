#ifndef BUZZER_H
#define BUZZER_H

#include <avr/io.h>
#include <stdint.h>

#ifndef BUZZER_ACTIVE_HIGH
#define BUZZER_ACTIVE_HIGH 1        //set to 0 if buzzer is active-LOW
#endif

#ifdef __cplusplus
extern "C" {
#endif

void buzzer_init(void);

//start/stop a continuous tone
void buzzer_start_tone(uint16_t freq_hz);
void buzzer_stop(void);

void buzzer_beep(uint16_t freq_hz, uint16_t duration_ms);
void buzzer_pattern(uint16_t freq_hz, uint8_t repeat, uint16_t on_ms, uint16_t off_ms);
void buzzer_sweep(uint16_t f_start_hz, uint16_t f_end_hz, uint8_t steps, uint16_t step_ms);

#ifdef __cplusplus
}
#endif

#endif /* BUZZER_H */
