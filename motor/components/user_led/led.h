#ifndef _LED_H_
#define _LED_H

#include "driver/gpio.h"

void LED_init(void);
void LED_PWM_on(float duty_set, uint32_t fade_time);
void LED_off(void);

#endif
