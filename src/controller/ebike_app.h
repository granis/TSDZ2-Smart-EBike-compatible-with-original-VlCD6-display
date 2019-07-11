/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho and EndlessCadence, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _EBIKE_APP_H_
#define _EBIKE_APP_H_

#include <stdint.h>
#include "main.h"

typedef struct _configuration_variables
{
	uint8_t ui8_assist_level_factor_x10;
  uint8_t ui8_battery_max_current;
  uint8_t ui8_motor_power_div10;
  uint16_t ui16_battery_low_voltage_cut_off_x10;
  uint16_t ui16_wheel_perimeter;
  uint8_t ui8_lights;
  uint8_t ui8_walk_assist;
  uint8_t ui8_offroad_mode;
  uint8_t ui8_emtb_mode;
  uint8_t ui8_emtb_enabled_on_startup;
  uint8_t ui8_wheel_max_speed;
  uint8_t ui8_motor_type;
  uint8_t ui8_motor_assistance_startup_without_pedal_rotation;
  uint8_t ui8_target_battery_max_power_div25;
  uint8_t configuration_variables;
  uint8_t ui8_startup_motor_power_boost_feature_enabled;
  uint8_t ui8_startup_motor_power_boost_assist_level;
  uint8_t ui8_startup_motor_power_boost_state;
  uint8_t ui8_startup_motor_power_boost_limit_to_max_power;
  uint8_t ui8_startup_motor_power_boost_time;
  uint8_t ui8_startup_motor_power_boost_fade_time;
  uint8_t ui8_temperature_limit_feature_enabled;
  uint8_t ui8_motor_temperature_min_value_to_limit;
  uint8_t ui8_motor_temperature_max_value_to_limit;
  uint8_t ui8_temperature_current_limiting_value;
  uint16_t ui16_motor_temperature_x2;
  uint8_t ui8_motor_temperature;
  uint8_t ui8_street_feature_enabled;
  uint8_t ui8_street_enabled_on_startup;
  uint8_t ui8_street_speed_limit;
  uint8_t ui8_street_power_limit_enabled;
  uint8_t ui8_street_power_limit_div25;
	uint8_t ui8_working_status;
	uint8_t ui8_function_code;
	uint8_t ui8_fault_code;
	uint8_t ui8_battery_cells_number;
	uint16_t ui16_battery_pack_resistance_x1000;
	uint16_t ui16_oem_wheel_speed_factor;
	uint8_t ui8_assist_level_power[5];
	uint8_t ui8_startup_motor_power_boost[4];
	uint8_t ui8_walk_assist_percentage_current;
	uint8_t ui8_walk_assist_pwm_duty_cycle;
	uint8_t ui8_walk_assist_pwm_duty_cycle_level[5];
	uint8_t ui8_walk_assist_ramp_time;
	uint8_t ui8_walk_assist_off_delay_pwm;
	uint16_t ui16_walk_assist_off_delay_time;
	uint8_t ui8_debug_var;
} struct_configuration_variables;

extern volatile uint8_t ui8_adc_torque_sensor_min_value;
extern volatile uint8_t ui8_adc_torque_sensor_max_value;
extern volatile uint8_t ui8_adc_battery_current_offset;
extern volatile uint8_t ui8_ebike_app_state;
extern volatile uint8_t ui8_adc_target_battery_max_current;
extern volatile uint16_t ui16_pas_pwm_cycles_ticks;
extern volatile uint8_t ui8_pas_direction;
extern volatile uint8_t ui8_pedaling_direction;
extern volatile uint8_t ui8_pas_cadence_rpm;
extern volatile uint16_t ui16_wheel_speed_sensor_pwm_cycles_ticks;
extern volatile uint8_t ui8_wheel_speed_sensor_is_disconnected;
extern volatile uint32_t ui32_wheel_speed_sensor_tick_counter;

void ebike_app_init(void);
void ebike_app_controller(void);
struct_configuration_variables* get_configuration_variables(void);

#if ENABLE_DEBUG_FIRMWARE
void buttons_init(void);
void leds_init(void);
void led1_set_state(uint8_t ui8_state);
void led2_set_state(uint8_t ui8_state);
#endif

#endif /* _EBIKE_APP_H_ */
