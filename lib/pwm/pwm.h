#ifndef PWM_H
#define PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <Arduino.h>

void pwm_init(void);
void pwm_set(uint16_t duty);        // 0..1023
void pwm_set_porcentaje(float pct); // 0..100 %

#ifdef __cplusplus
}
#endif

#endif
