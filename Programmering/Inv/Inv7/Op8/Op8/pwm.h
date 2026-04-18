#ifndef PWM_H_
#define PWM_H_
#include <avr/io.h>

void pwm_1_servo_init();
void pwm_1_servo_set_pos(uint16_t pos);
void pwm_3_servo_init();
void pwm_3_servo_set_pos(uint16_t pos1, uint16_t pos2, uint16_t pos3);
void pwm_3_leds_init();
void pwm_4_leds_init();
void pwm_3_leds_set_dc(uint16_t dc1, uint16_t dc2, uint16_t dc3);
void pwm_4_leds_set_dc(uint8_t dc1, uint8_t dc2, uint8_t dc3, uint8_t dc4);




#endif /* PWM_H_ */