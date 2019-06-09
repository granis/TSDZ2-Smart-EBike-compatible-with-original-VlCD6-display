/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <stdint.h>
#include "main.h"

#if 0
extern volatile uint8_t ui8_duty_cycle_target;
extern volatile uint16_t ui16_motor_speed_erps;
extern volatile uint8_t ui8_adc_motor_phase_current;
extern volatile uint8_t ui8_pas_1;
extern volatile uint8_t ui8_pas_2;
extern volatile uint16_t ui16_torque_sensor_throttle_processed_value;
extern volatile uint8_t ui8_adc_battery_current;
extern volatile uint8_t ui8_foc_angle;
#endif
extern volatile uint8_t ui8_duty_cycle;
extern volatile uint8_t ui8_adc_motor_phase_current_offset;

/***************************************************************************************/
// Motor interface
void hall_sensor_init(void); // must be called before using the motor
void motor_init(void); // must be called before using the motor
void motor_enable_PWM(void);
void motor_disable_PWM(void);
void motor_set_pwm_duty_cycle_target(uint8_t ui8_value);
void motor_set_current_max(uint8_t value); // steps of 0.25A each step
void motor_set_pwm_duty_cycle_ramp_up_inverse_step(uint16_t value); // each step = 64us
void motor_set_pwm_duty_cycle_ramp_down_inverse_step(uint16_t value); // each step = 64us
uint16_t ui16_motor_get_motor_speed_erps(void);
void motor_controller_set_state(uint8_t state);
void motor_controller_reset_state(uint8_t state);
uint8_t motor_controller_state_is_set(uint8_t state);
void motor_controller(void);
uint8_t motor_get_adc_battery_current_filtered_10b(void);
uint16_t motor_get_adc_battery_voltage_filtered_10b(void);
void motor_set_adc_battery_voltage_cut_off(uint8_t ui8_value);
/***************************************************************************************/

#endif /* _MOTOR_H_ */
