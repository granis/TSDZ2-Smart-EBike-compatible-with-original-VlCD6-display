/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _PWM_H
#define _PWM_H

#include "main.h"

void pwm_init_bipolar_4q(void);
#if ENABLE_BACKWARDS_RESISTANCE_OFF
void disable_pwm(void);
void enable_pwm(void);
#endif

#endif /* _PWM_H_ */
