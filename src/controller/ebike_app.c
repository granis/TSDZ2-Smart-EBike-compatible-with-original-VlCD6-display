/*
 * TongSheng TSDZ2 motor controller firmware
 *
 * Copyright (C) Casainho and EndlessCadence, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>

#include "ebike_app.h"
#include "stm8s.h"
#include "stm8s_gpio.h"
#include "interrupts.h"
#include "adc.h"
#include "utils.h"
#include "motor.h"
#include "pwm.h"
#include "uart.h"
#include "brake.h"
#include "eeprom.h"
#include "utils.h"
#include "lights.h"
#include "pins.h"

#if ENABLE_DEBUG_FIRMWARE
uint8_t led1_status = 0;
uint8_t led2_status = 0;
#endif

uint8_t ui8_adc_battery_max_current = ADC_BATTERY_CURRENT_MAX;
uint8_t ui8_target_battery_max_power_x10 = ADC_BATTERY_CURRENT_MAX;

volatile uint8_t ui8_throttle = 0;
volatile uint8_t ui8_torque_sensor_value1 = 0;
volatile uint8_t ui8_torque_sensor = 0;
volatile uint8_t ui8_torque_sensor_raw = 0;
volatile uint8_t ui8_adc_torque_sensor_min_value;
volatile uint8_t ui8_adc_torque_sensor_max_value;
volatile uint8_t ui8_adc_battery_current_offset;
volatile uint8_t ui8_ebike_app_state = EBIKE_APP_STATE_MOTOR_STOP;
volatile uint8_t ui8_adc_target_battery_max_current;
uint8_t ui8_adc_battery_current_max;

volatile uint16_t ui16_pas_pwm_cycles_ticks = (uint16_t) PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS;
volatile uint8_t ui8_motor_enabled = 1;
volatile uint8_t ui8_pedaling_direction = 0;
volatile uint8_t ui8_pas_direction = 0;
uint8_t ui8_pas_cadence_rpm = 0;
uint16_t ui16_pedal_torque_x10;
uint16_t ui16_pedal_power_x10;
uint8_t ui8_pedal_human_power = 0;
uint8_t ui8_startup_boost_enable = 0;
uint8_t ui8_startup_boost_fade_enable = 0;
uint8_t ui8_startup_boost_state_machine = 0;
uint8_t ui8_startup_boost_no_torque = 0;
uint8_t ui8_startup_boost_timer = 0;
uint8_t ui8_startup_boost_fade_steps = 0;
uint16_t ui16_startup_boost_fade_variable_x256;
uint16_t ui16_startup_boost_fade_variable_step_amount_x256;

// wheel speed
volatile uint16_t ui16_wheel_speed_sensor_pwm_cycles_ticks = (uint16_t) WHEEL_SPEED_SENSOR_MAX_PWM_CYCLE_TICKS;
uint8_t ui8_wheel_speed_max = 0;
float f_wheel_speed_x10;
uint16_t ui16_wheel_speed_x10;
volatile uint32_t ui32_wheel_speed_sensor_tick_counter = 0;
float f_oem_wheel_speed;
uint16_t ui16_oem_wheel_speed;

volatile struct_configuration_variables configuration_variables;

// UART
volatile uint8_t ui8_received_package_flag = 0;
volatile uint8_t ui8_rx_buffer[7];
volatile uint8_t ui8_rx_counter = 0;
volatile uint8_t ui8_tx_buffer[9];
volatile uint8_t ui8_tx_counter = 0;
volatile uint8_t ui8_i;
volatile uint8_t ui8_checksum;
volatile uint8_t ui8_byte_received;
volatile uint8_t ui8_state_machine = 0;
volatile uint8_t ui8_uart_received_first_package = 0;
static uint16_t ui16_crc_rx;
static uint16_t ui16_crc_tx;
static uint8_t ui8_master_comm_package_id = 0;
static uint8_t ui8_slave_comm_package_id = 0;

uint8_t ui8_tstr_state_machine = STATE_NO_PEDALLING;
uint8_t ui8_rtst_counter = 0;

uint16_t ui16_adc_motor_temperatured_accumulated = 0;
uint8_t ui8_overtemperature = 0;

uint8_t ui8_adc_battery_target_current;

// safe tests
uint8_t safe_tests_state_machine = 0;
uint8_t safe_tests_state_machine_counter = 0;
static void ebike_control_motor(void);
static void ebike_app_set_battery_max_current(uint8_t ui8_value);
static void ebike_app_set_target_adc_battery_max_current(uint8_t ui8_value);

static void communications_controller(void);
static void uart_send_package(void);

static void throttle_read(void);
static void read_pas_cadence(void);
static void torque_sensor_read(void);

static void calc_pedal_force_and_torque(void);
static void calc_wheel_speed(void);
static void calc_motor_temperature(void);
static uint16_t calc_filtered_battery_voltage(void);

static void apply_street_mode(uint16_t ui16_battery_voltage, uint8_t *ui8_max_speed, uint8_t *ui8_target_current);
static void apply_speed_limit(uint16_t ui16_speed_x10, uint8_t ui8_max_speed, uint8_t *ui8_target_current);
static void apply_temperature_limiting(uint8_t *ui8_target_current);

#if ENABLE_THROTTLE
static void apply_throttle(uint8_t ui8_throttle_value, uint8_t *ui8_motor_enable, uint8_t *ui8_target_current);
#endif

volatile uint32_t ui32_battery_voltage_accumulated_x10000 = 0;
volatile uint16_t ui16_battery_current_accumulated_x5 = 0;
volatile uint8_t ui8_battery_state_of_charge = 0;
volatile uint8_t ui8_timer_counter = 0;
volatile uint8_t ui8_startup_counter = 0;
volatile uint8_t ui8_lights_counter = 0;
volatile uint16_t ui16_walk_assist_delay_off = 0;
volatile uint8_t ui8_walk_assist_delay_off_flag = 0;
volatile uint8_t ui8_display_ready_flag = 0;
volatile uint8_t ui8_enable_walk_assist = 0;
volatile uint8_t ui8_walk_assist_flag = 0;
volatile uint8_t ui8_walk_assist = 0;
volatile uint8_t ui8_walk_assist_current_per_cent = 0;
volatile uint8_t ui8_walk_assist_start = 0;
volatile uint8_t ui8_walk_assist_pwm = 0;
volatile uint8_t ui8_walk_assist_pwm_step = 0;
volatile uint8_t ui8_default_flag = 0;
volatile uint8_t ui8_lights_flag = 0;
volatile uint8_t ui8_mode_flag = 0;
volatile uint8_t ui8_boost_flag = 0;

static void check_battery_soc(void);
static void walk_assist_read(void);
static void apply_walk_assist(uint8_t ui8_walk_assist_value, uint8_t *ui8_motor_enable, uint8_t *ui8_target_current);

static void boost_run_statemachine(void);
static uint8_t apply_boost(uint8_t ui8_pas_cadence, uint8_t ui8_max_current_boost_state, uint8_t *ui8_target_current);
static void apply_boost_fade_out(uint8_t *ui8_target_current);

static void safe_tests(void);

void ebike_app_init(void)
{
  // init variables with the stored value on EEPROM
  eeprom_init_variables();
  ebike_app_set_battery_max_current(ADC_BATTERY_CURRENT_MAX);

	// clear function code
	configuration_variables.ui8_function_code = NO_FUNCTION;

	if(configuration_variables.ui8_street_enabled_on_startup)
		// enable street mode on startup!
		configuration_variables.ui8_offroad_mode = 0;
	else
		// enable offroad mode on startup!
		configuration_variables.ui8_offroad_mode = 1;
}

void ebike_app_controller(void)
{
	check_battery_soc();
	walk_assist_read();
	throttle_read();
  torque_sensor_read();
  read_pas_cadence();
  calc_pedal_force_and_torque();
  calc_wheel_speed();
  calc_motor_temperature();
  ebike_control_motor();
  communications_controller();
	#if ENABLE_DEBUG_FIRMWARE
	#if 1
  if(configuration_variables.ui8_working_status & 0x04)
  	led1_set_state(1);
  else
  	led1_set_state(0);
	#endif

	#if 0
  uint8_t ui8_i;
  uint16_t ui16_temp;

	ui16_temp = motor_get_adc_battery_voltage_filtered_10b();

	// adc 10 bits battery voltage
	ui8_tx_buffer[0] = 0xAA;
	ui8_tx_buffer[1] = 0;
	ui8_tx_buffer[2] = (uint8_t)(ui16_temp >> 8);
	ui8_tx_buffer[3] = (uint8_t)(ui16_temp & 0xFF);
	ui8_tx_buffer[4] = 0;
	ui8_tx_buffer[5] = 0;
	ui8_tx_buffer[6] = 0;
	ui8_tx_buffer[7] = 0;
	ui8_tx_buffer[8] = 0x55;

	// send the full package to UART
	for(ui8_i = 0; ui8_i < 9; ui8_i++)
	{
		putchar(ui8_tx_buffer[ui8_i]);
	}
	#endif

  #if 0
  led1_set_state(led1_status);
  if(led1_status)
  	led1_status = 0;
  else
  	led1_status = 1;
	#endif

  #if 0
  if(GPIO_ReadInputPin(BUTTON1__PORT, BUTTON1__PIN) == 0)
  	led1_set_state(1);
  else
  	led1_set_state(0);
	#endif
	#endif
}

#if ENABLE_DEBUG_FIRMWARE
void buttons_init(void)
{
	// button1: input pull-up, no external interrupt
	GPIO_Init(BUTTON1__PORT,
	    			BUTTON1__PIN,
						GPIO_MODE_IN_PU_NO_IT);

	// button2: input pull-up, no external interrupt
	GPIO_Init(BUTTON2__PORT,
	    			BUTTON2__PIN,
						GPIO_MODE_IN_PU_NO_IT);
}

void leds_init(void)
{
	// led1: output pus-pull
	GPIO_Init(LED1__PORT,
            LED1__PIN,
            GPIO_MODE_OUT_PP_LOW_SLOW);

	// led2: output pus-pull
	GPIO_Init(LED2__PORT,
            LED2__PIN,
            GPIO_MODE_OUT_PP_LOW_SLOW);
}

void led1_set_state(uint8_t ui8_state)
{
  if(ui8_state)
    GPIO_WriteHigh(LED1__PORT, LED1__PIN);
  else
    GPIO_WriteLow(LED1__PORT, LED1__PIN);
}

void led2_set_state(uint8_t ui8_state)
{
  if(ui8_state)
    GPIO_WriteHigh(LED2__PORT, LED2__PIN);
  else
    GPIO_WriteLow(LED2__PORT, LED2__PIN);
}
#endif

static void ebike_control_motor(void)
{
  static uint32_t ui32_temp;
  uint8_t ui8_tmp_pas_cadence_rpm;
  uint32_t ui32_adc_max_battery_current_x4;
  uint8_t ui8_adc_max_battery_current = 0;
  uint8_t ui8_adc_max_battery_power_current = 0;
  uint8_t ui8_startup_enable;
  uint8_t ui8_boost_enabled_and_applied = 0;
  uint8_t ui8_tmp_max_speed;
  uint16_t ui16_battery_voltage_filtered = calc_filtered_battery_voltage();

  // calc max battery current for boost state
  // calc max battery current for regular state
	uint8_t ui8_adc_max_battery_current_boost_state = 0;

	// controller works with no less than 15V so calculate the target current only for higher voltages
  if(ui16_battery_voltage_filtered > 15)
  {
    // 1.6 = 1 / 0.625 (each adc step for current)
    // 25 * 1.6 = 40
    // 40 * 4 = 160
    if(configuration_variables.ui8_startup_motor_power_boost_assist_level > 0)
    {
      ui32_temp = (uint32_t) ui16_pedal_torque_x10 * (uint32_t) configuration_variables.ui8_startup_motor_power_boost_assist_level;
      ui32_temp /= 10;

      // 1.6 = 1 / 0.625 (each adc step for current)
      // 1.6 * 8 = ~13
      ui32_temp = (ui32_temp * 13) / ((uint32_t) ui16_battery_voltage_filtered);
      ui8_adc_max_battery_current_boost_state = ui32_temp >> 3;
      ui8_limit_max(&ui8_adc_max_battery_current_boost_state, 255);
    }

    if(configuration_variables.ui8_assist_level_factor_x10 > 0)
    {
			#if ENABLE_LAST_BETA_RELEASE || ENABLE_LAST_APP_BATTERY_TARGET_CODE
			// when ui8_motor_assistance_startup_without_pedal_rotation == 0
			if(configuration_variables.ui8_motor_assistance_startup_without_pedal_rotation == 0)
			{
			 ui32_temp = (uint32_t) ui16_pedal_power_x10 * (uint32_t) configuration_variables.ui8_assist_level_factor_x10;
			 ui32_temp /= 100;
			}
			else
			{
        if(ui8_pas_cadence_rpm)
        {
          ui32_temp = (uint32_t) ui16_pedal_power_x10 * (uint32_t) configuration_variables.ui8_assist_level_factor_x10;
          ui32_temp /= 100;
        }
        else
        {
          ui32_temp = (uint32_t) ui16_pedal_torque_x10 * (uint32_t) configuration_variables.ui8_assist_level_factor_x10;
          ui32_temp /= 100;
        }
			}
			#else
			ui32_temp = (uint32_t) ui16_pedal_power_x10 * (uint32_t) configuration_variables.ui8_assist_level_factor_x10;
			ui32_temp /= 100;
			#endif

      // 1.6 = 1 / 0.625 (each adc step for current)
      // 1.6 * 8 = ~13
      ui32_temp = (ui32_temp * 13) / ((uint32_t) ui16_battery_voltage_filtered);
      ui8_adc_max_battery_current = ui32_temp >> 3;
      ui8_limit_max(&ui8_adc_max_battery_current, 255);
			#if ENABLE_LAST_BETA_RELEASE || ENABLE_LAST_APP_BATTERY_TARGET_CODE
			ui8_adc_battery_target_current = ui8_adc_max_battery_current;
			#endif
    }

    if(configuration_variables.ui8_target_battery_max_power_div25 > 0) //TODO: add real feature toggle for max power feature
    {
      ui32_adc_max_battery_current_x4 = (((uint32_t) configuration_variables.ui8_target_battery_max_power_div25) * 160) / ((uint32_t) ui16_battery_voltage_filtered);
      ui8_adc_max_battery_power_current = ui32_adc_max_battery_current_x4 >> 2;
    }
  }

  // start when we press the pedals
  ui8_startup_enable = (configuration_variables.ui8_assist_level_factor_x10 && ui8_torque_sensor) ? 1 : 0;
	
	// get pas cadence rpm
  ui8_tmp_pas_cadence_rpm = ui8_pas_cadence_rpm;
	
	#if ENABLE_LAST_BETA_RELEASE || ENABLE_LAST_APP_PAS_CADENCE_RPM_CODE
  // let's cheat next value, only to cheat apply_boost()
  if(configuration_variables.ui8_motor_assistance_startup_without_pedal_rotation)
  {
    if(ui8_pas_cadence_rpm < 10){ui8_tmp_pas_cadence_rpm = 10;}
  }
	#else
  if(configuration_variables.ui8_motor_assistance_startup_without_pedal_rotation)
  {
    if(ui8_pas_cadence_rpm < 10){ui8_tmp_pas_cadence_rpm = 10;}
  }
  else
  {
    if(ui8_pas_cadence_rpm < 10){ui8_tmp_pas_cadence_rpm = 0;}
  }
	#endif
	
  if(configuration_variables.ui8_startup_motor_power_boost_feature_enabled)
  {
    boost_run_statemachine();  
    ui8_boost_enabled_and_applied = apply_boost(ui8_tmp_pas_cadence_rpm, ui8_adc_max_battery_current_boost_state, &ui8_adc_battery_target_current);
  }
	
	#if !ENABLE_LAST_BETA_RELEASE && !ENABLE_LAST_APP_BATTERY_TARGET_CODE
  if(!ui8_boost_enabled_and_applied)
  {
    ui8_adc_battery_target_current = ui8_adc_max_battery_current;
  }
	#endif
	
  /* Boost: make transition from boost to regular level */
  if(configuration_variables.ui8_startup_motor_power_boost_feature_enabled)
  {
    apply_boost_fade_out(&ui8_adc_battery_target_current);
  }  

	#if ENABLE_THROTTLE
  /* Throttle */
  apply_throttle(ui8_throttle, &ui8_startup_enable, &ui8_adc_battery_target_current);
	#endif
	
	// Walk Assist active:
	if(ui8_walk_assist_flag)
	{
		ui8_adc_battery_target_current = (ui8_adc_battery_current_max	* ui8_walk_assist_current_per_cent) / 100;
		apply_walk_assist(ui8_walk_assist, &ui8_startup_enable, &ui8_adc_battery_target_current);
	}	

	// get max wheel speed from eeprom
  ui8_tmp_max_speed = configuration_variables.ui8_wheel_max_speed;

  /* Street mode (limit speed enabled if street mode is active) */
  if(configuration_variables.ui8_street_feature_enabled)
  {
    apply_street_mode(ui16_battery_voltage_filtered, &ui8_tmp_max_speed, &ui8_adc_battery_target_current);
  }

  /* Speed limit */
  apply_speed_limit(ui16_wheel_speed_x10, ui8_tmp_max_speed, &ui8_adc_battery_target_current);

  /* User configured max power on display */
  if(configuration_variables.ui8_target_battery_max_power_div25 > 0) //TODO: add real feature toggle for max power feature
  {
    // limit the current to max value defined by user on LCD max power, if:
    // - user defined to make that limitation
    // - we are not on boost or fade state
    if((configuration_variables.ui8_startup_motor_power_boost_limit_to_max_power == 1)||
       (!((ui8_boost_enabled_and_applied == 1)||
       (ui8_startup_boost_fade_enable == 1))))
    {
      // now let's limit the target battery current to battery max current (use min value of both)
      ui8_adc_battery_target_current = ui8_min(ui8_adc_battery_target_current, ui8_adc_max_battery_power_current);
    }
  }

  /* Limit current if motor temperature too high and this feature is enabled by the user */
  if(configuration_variables.ui8_temperature_limit_feature_enabled)
  {
    apply_temperature_limiting(&ui8_adc_battery_target_current);
		
		// verify if motor overtemperature   
		if(!ui8_overtemperature)
		{
			if(configuration_variables.ui8_motor_temperature >= configuration_variables.ui8_motor_temperature_max_value_to_limit)
				ui8_overtemperature = 1;
		}
		else
		{
			if(configuration_variables.ui8_motor_temperature <= configuration_variables.ui8_motor_temperature_min_value_to_limit)
				ui8_overtemperature = 0;
		}
  }
  else
  {
		// clear overtemperature status
		ui8_overtemperature = 0;
  }
	
  // execute some safe tests
  safe_tests();	
		
	#if ENABLE_LAST_BETA_RELEASE || ENABLE_LAST_APP_PWM_DUTY_CYCLE_CODE /////////////////////////////////////
  // let's force our target current to 0 if brake is set or if there are errors
  if((brake_is_set())||(configuration_variables.ui8_error_states != ERROR_STATE_NO_ERRORS))
  {
    ui8_adc_battery_target_current = 0;
  }
	
	#if ENABLE_BACKWARDS_RESISTANCE_OFF
  // check to see if we should enable the motor
  if((ui8_motor_enabled == 0)&&
	   (ui16_motor_get_motor_speed_erps() == 0)&&
     (ui8_adc_battery_target_current))
  {
    {
      ui8_motor_enabled = 1;
      enable_pwm();
    }
  }

  // check to see if we should disable the motor
  if((ui8_motor_enabled)&&
     (ui16_motor_get_motor_speed_erps() == 0)&&
     (ui8_adc_battery_target_current == 0)&&
     (ui8_duty_cycle == 0))
  {
    ui8_motor_enabled = 0;
    disable_pwm();
  }
	
  // apply the target current if motor is enable and if not, reset the duty_cycle controller
  if(ui8_motor_enabled)
  {
    // finally set the target battery current to the battery current controller
    ebike_app_set_target_adc_battery_max_current(ui8_adc_battery_target_current);
  }
  else
  {
    ui8_duty_cycle = 0;
  }	
	#else
	// finally set the target battery current to the current controller
  ebike_app_set_target_adc_battery_max_current(ui8_adc_battery_target_current);	
	#endif
	
	// we must have a target positive current and brakes must not be pressed
	if((ui8_adc_battery_target_current)&&(!brake_is_set()))
  {
		// walk assist active?
		if(ui8_walk_assist_flag)
		{
			// walk assist start now?
			if(ui8_walk_assist_start == 0)
			{
				// walk assist started!
				ui8_walk_assist_start = 1;

				// pwm start from 0
				ui8_walk_assist_pwm = 0;

				// calculate step for pwm ramp-up
				if(ui8_walk_assist_pwm < configuration_variables.ui8_walk_assist_ramp_time)
					ui8_walk_assist_pwm_step = 1;
				else
					ui8_walk_assist_pwm_step = configuration_variables.ui8_walk_assist_pwm_duty_cycle / configuration_variables.ui8_walk_assist_ramp_time;
			}

			// walk assist pwm duty cycle ramp-up!
			if((ui8_walk_assist_pwm + ui8_walk_assist_pwm_step) <= configuration_variables.ui8_walk_assist_pwm_duty_cycle)
				ui8_walk_assist_pwm += ui8_walk_assist_pwm_step;
			else
				ui8_walk_assist_pwm = configuration_variables.ui8_walk_assist_pwm_duty_cycle;

			// motor run for walking assistance!
			motor_set_pwm_duty_cycle_target(ui8_walk_assist_pwm);
		}
		else
		{
			motor_set_pwm_duty_cycle_target(255);
		}
	}		
  else
  {
  	motor_set_pwm_duty_cycle_target(0);
		
		ui8_walk_assist_flag = 0;
		ui8_walk_assist_start = 0;
		ui8_walk_assist_delay_off_flag = 0;
		ui8_walk_assist_current_per_cent = 0;		
	}
	#else ///////////////////////////////////////////////////////////////////////////////////////////
	// finally set the target battery current to the current controller
  ebike_app_set_target_adc_battery_max_current(ui8_adc_battery_target_current);	

	if((ui8_adc_battery_target_current)&& // we must have a target positive current
     (ui8_startup_enable)&& // startup enabled
     (!brake_is_set())&& // brakes must not be pressed
     (configuration_variables.ui8_error_states == ERROR_STATE_NO_ERRORS)) // we must have no errors
  {
  	#if ENABLE_BACKWARDS_RESISTANCE_OFF
		// start pwm motor:
		if(ui16_motor_get_motor_speed_erps() == 0)
		{
			if(!ui8_motor_enabled)
			{
				// enable pwm...
				enable_pwm();
				ui8_motor_enabled = 1;
			}
		}
		#endif

		// walk assist active?
		if(ui8_walk_assist_flag)
		{
			// walk assist start now?
			if(ui8_walk_assist_start == 0)
			{
				// walk assist started!
				ui8_walk_assist_start = 1;

				// pwm start from 0
				ui8_walk_assist_pwm = 0;

				// calculate step for pwm ramp-up
				if(ui8_walk_assist_pwm < configuration_variables.ui8_walk_assist_ramp_time)
					ui8_walk_assist_pwm_step = 1;
				else
					ui8_walk_assist_pwm_step = configuration_variables.ui8_walk_assist_pwm_duty_cycle / configuration_variables.ui8_walk_assist_ramp_time;
			}

			// walk assist pwm duty cycle ramp-up!
			if((ui8_walk_assist_pwm + ui8_walk_assist_pwm_step) <= configuration_variables.ui8_walk_assist_pwm_duty_cycle)
				ui8_walk_assist_pwm += ui8_walk_assist_pwm_step;
			else
				ui8_walk_assist_pwm = configuration_variables.ui8_walk_assist_pwm_duty_cycle;

			// motor run for walking assistance!
			motor_set_pwm_duty_cycle_target(ui8_walk_assist_pwm);
		}
		else
		{
			motor_set_pwm_duty_cycle_target(255);
		}
  }
  else
  {
  	motor_set_pwm_duty_cycle_target(0);

		#if ENABLE_BACKWARDS_RESISTANCE_OFF
		if((ui8_motor_enabled)&&
			 (ui16_motor_get_motor_speed_erps() == 0)&&
			 (ui8_adc_battery_target_current == 0)&&
			 (ui8_duty_cycle == 0))
		{
			// disable pwm...
			disable_pwm();
			ui8_motor_enabled = 0;
		}
		#endif

    ui8_walk_assist_flag = 0;
    ui8_walk_assist_start = 0;
		ui8_walk_assist_delay_off_flag = 0;
    ui8_walk_assist_current_per_cent = 0;
  }
	#endif
}

static void check_battery_soc(void)
{
	uint16_t ui16_fluctuate_battery_voltage_x10;
	uint16_t ui16_battery_voltage_filtered_x10;
	uint16_t ui16_battery_current_filtered_x5;
	uint16_t ui16_battery_voltage_soc_x10;
	uint16_t ui16_adc_battery_voltage;
	uint8_t ui8_battery_cells_number_x10;
	uint8_t ui8_battery_current_x5;

	// adc 10 bits battery voltage
	ui16_adc_battery_voltage = motor_get_adc_battery_voltage_filtered_10b();

	// low pass filter battery voltage
	ui32_battery_voltage_accumulated_x10000 -= ui32_battery_voltage_accumulated_x10000 >> SOC_BATTERY_VOLTAGE_FILTER_COEFFICIENT;
	ui32_battery_voltage_accumulated_x10000 += (uint32_t) ui16_adc_battery_voltage * SOC_ADC_BATTERY_VOLTAGE_PER_ADC_STEP_X10000;
	ui16_battery_voltage_filtered_x10 = ((uint32_t) (ui32_battery_voltage_accumulated_x10000 >> SOC_BATTERY_VOLTAGE_FILTER_COEFFICIENT)) / 1000;

	// battery current x5
	ui8_battery_current_x5 = (uint8_t) ((float) motor_get_adc_battery_current_filtered_10b() * 0.826);

	// low pass filter battery current
	ui16_battery_current_accumulated_x5 -= ui16_battery_current_accumulated_x5 >> SOC_BATTERY_CURRENT_FILTER_COEFFICIENT;
	ui16_battery_current_accumulated_x5 += (uint16_t) ui8_battery_current_x5;
	ui16_battery_current_filtered_x5 = ui16_battery_current_accumulated_x5 >> SOC_BATTERY_CURRENT_FILTER_COEFFICIENT;

  // calculate flutuate voltage, that depends on the current and battery pack resistance
  ui16_fluctuate_battery_voltage_x10 = (uint16_t) ((((uint32_t) configuration_variables.ui16_battery_pack_resistance_x1000) * ((uint32_t) ui16_battery_current_filtered_x5)) / ((uint32_t) 500));
  // now add fluctuate voltage value
  ui16_battery_voltage_soc_x10 = ui16_battery_voltage_filtered_x10 + ui16_fluctuate_battery_voltage_x10;

	// to keep same scale as voltage of x10
	ui8_battery_cells_number_x10 = configuration_variables.ui8_battery_cells_number * 10;

	// update battery level value only at minimun every 100ms and this helps to visual filter the fast changing values
	if(ui8_timer_counter++ >= 1)
	{
		ui8_timer_counter = 0;

		#if ENABLE_VLCD6_COMPATIBILITY
		if(ui16_battery_voltage_soc_x10 > ((uint16_t) ((float) ui8_battery_cells_number_x10 * LI_ION_CELL_VOLTS_5))) {ui8_battery_state_of_charge = 6;}				// 4 bars --> full + overvoltage
		else if(ui16_battery_voltage_soc_x10 > ((uint16_t) ((float) ui8_battery_cells_number_x10 * LI_ION_CELL_VOLTS_4))) {ui8_battery_state_of_charge = 5;}	// 4 bars --> full
		else if(ui16_battery_voltage_soc_x10 > ((uint16_t) ((float) ui8_battery_cells_number_x10 * LI_ION_CELL_VOLTS_3))) {ui8_battery_state_of_charge = 4;}	// 3 bars
		else if(ui16_battery_voltage_soc_x10 > ((uint16_t) ((float) ui8_battery_cells_number_x10 * LI_ION_CELL_VOLTS_2))) {ui8_battery_state_of_charge = 3;}	// 2 bars
		else if(ui16_battery_voltage_soc_x10 > ((uint16_t) ((float) ui8_battery_cells_number_x10 * LI_ION_CELL_VOLTS_1))) {ui8_battery_state_of_charge = 2;}	// 1 bar
		else if(ui16_battery_voltage_soc_x10 > ((uint16_t) ((float) ui8_battery_cells_number_x10 * LI_ION_CELL_VOLTS_0))) {ui8_battery_state_of_charge = 1;}	// blink --> empty
		else{ui8_battery_state_of_charge = 0;} // undervoltage
		#endif

		#if ENABLE_VLCD5_COMPATIBILITY
		if(ui16_battery_voltage_soc_x10 > ((uint16_t) ((float) ui8_battery_cells_number_x10 * LI_ION_CELL_VOLTS_7))) {ui8_battery_state_of_charge = 8;}   		// 6 bars --> full + overvoltage
		else if(ui16_battery_voltage_soc_x10 > ((uint16_t) ((float) ui8_battery_cells_number_x10 * LI_ION_CELL_VOLTS_6))) {ui8_battery_state_of_charge = 7;}	// 6 bars --> full
		else if(ui16_battery_voltage_soc_x10 > ((uint16_t) ((float) ui8_battery_cells_number_x10 * LI_ION_CELL_VOLTS_5))) {ui8_battery_state_of_charge = 6;}	// 5 bars
		else if(ui16_battery_voltage_soc_x10 > ((uint16_t) ((float) ui8_battery_cells_number_x10 * LI_ION_CELL_VOLTS_4))) {ui8_battery_state_of_charge = 5;}	// 4 bars
		else if(ui16_battery_voltage_soc_x10 > ((uint16_t) ((float) ui8_battery_cells_number_x10 * LI_ION_CELL_VOLTS_3))) {ui8_battery_state_of_charge = 4;}	// 3 bars
		else if(ui16_battery_voltage_soc_x10 > ((uint16_t) ((float) ui8_battery_cells_number_x10 * LI_ION_CELL_VOLTS_2))) {ui8_battery_state_of_charge = 3;}	// 2 bars
		else if(ui16_battery_voltage_soc_x10 > ((uint16_t) ((float) ui8_battery_cells_number_x10 * LI_ION_CELL_VOLTS_1))) {ui8_battery_state_of_charge = 2;}	// 1 bar
		else if(ui16_battery_voltage_soc_x10 > ((uint16_t) ((float) ui8_battery_cells_number_x10 * LI_ION_CELL_VOLTS_0))) {ui8_battery_state_of_charge = 1;}	// blink --> empty
		else{ui8_battery_state_of_charge = 0;} // undervoltage
		#endif
	}
}

static void communications_controller(void)
{
	#if !ENABLE_DEBUG_UART
	uint32_t ui32_temp;
	uint8_t ui8_assist_level;
	uint8_t ui8_rx_check_code;

	// increment walk_assist_delay_off
	ui16_walk_assist_delay_off++;

	// increment lights_counter
	ui8_lights_counter++;

	if(ui8_received_package_flag)
	{
		// verify check code of the package
		ui8_rx_check_code = 0x00;
		for(ui8_i = 0; ui8_i < 6; ui8_i++)
		{
			ui8_rx_check_code += ui8_rx_buffer[ui8_i];
		}

		// see if check code is ok...
		if(ui8_rx_check_code == ui8_rx_buffer[6])
		{
			// enable walk assist
			ui8_enable_walk_assist = 1;

			// mask assist level from display
			ui8_assist_level = ui8_rx_buffer[1] & 0x5E; // mask: 01011110
			
			switch(ui8_assist_level)
			{
				case ASSIST_PEDAL_LEVEL0:
					// assist level 0
					configuration_variables.ui8_assist_level_factor_x10 = 0;

					// startup motor power boost 0
					configuration_variables.ui8_startup_motor_power_boost_assist_level = 0;

					// walk assist level 0
					configuration_variables.ui8_walk_assist_pwm_duty_cycle = configuration_variables.ui8_walk_assist_pwm_duty_cycle_level[0];

					#if ENABLE_WALK_ASSIST_OFF_DELAY
					// disable walk assist for level 0
					ui8_enable_walk_assist = 0;
					#endif
					break;

				case ASSIST_PEDAL_LEVEL1:
					// assist level 1
					configuration_variables.ui8_assist_level_factor_x10 = configuration_variables.ui8_assist_level_power[0];

					// startup motor power boost 1
					configuration_variables.ui8_startup_motor_power_boost_assist_level = configuration_variables.ui8_startup_motor_power_boost[0];

					#if ENABLE_WALK_ASSIST_OFF_DELAY
					// walk assist level 1
					configuration_variables.ui8_walk_assist_pwm_duty_cycle = configuration_variables.ui8_walk_assist_off_delay_pwm;
					#else
					// walk assist level 1
					configuration_variables.ui8_walk_assist_pwm_duty_cycle = configuration_variables.ui8_walk_assist_pwm_duty_cycle_level[1];
					#endif
					break;

				case ASSIST_PEDAL_LEVEL2:
					// assist level 2
					configuration_variables.ui8_assist_level_factor_x10 = configuration_variables.ui8_assist_level_power[1];

					// startup motor power boost 2
					configuration_variables.ui8_startup_motor_power_boost_assist_level = configuration_variables.ui8_startup_motor_power_boost[1];

					// walk assist level 2
					configuration_variables.ui8_walk_assist_pwm_duty_cycle = configuration_variables.ui8_walk_assist_pwm_duty_cycle_level[2];

					#if ENABLE_WALK_ASSIST_OFF_DELAY
					// disable walk assist for level 2
					ui8_enable_walk_assist = 0;
					#endif
					break;

				case ASSIST_PEDAL_LEVEL3:
					// assist level 3
					configuration_variables.ui8_assist_level_factor_x10 = configuration_variables.ui8_assist_level_power[2];

					// startup motor power boost 3
					configuration_variables.ui8_startup_motor_power_boost_assist_level = configuration_variables.ui8_startup_motor_power_boost[2];

					// walk assist level 3
					configuration_variables.ui8_walk_assist_pwm_duty_cycle = configuration_variables.ui8_walk_assist_pwm_duty_cycle_level[3];
					break;

				case ASSIST_PEDAL_LEVEL4:
					// assist level 4
					configuration_variables.ui8_assist_level_factor_x10 = configuration_variables.ui8_assist_level_power[3];

					// startup motor power boost 4
					configuration_variables.ui8_startup_motor_power_boost_assist_level = configuration_variables.ui8_startup_motor_power_boost[3];

					// walk assist level 4
					configuration_variables.ui8_walk_assist_pwm_duty_cycle = configuration_variables.ui8_walk_assist_pwm_duty_cycle_level[4];
					break;

				default:
					// nothing
					break;
			}

			//-----------------------------------------------------
			// ASSIST LEVEL0: ENABLE DEFAULT FUNCTIONS
			//-----------------------------------------------------
			// assist pedal level 0:
			if(ui8_assist_level == ASSIST_PEDAL_LEVEL0)
			{
				// Display down button pressed:
				if(ui8_rx_buffer[1] & 0x01)
				{
					// lights off:
					if(!ui8_lights_flag)
					{
						// default flag cleared
						if(!ui8_default_flag)
						{
							// set default flag
							ui8_default_flag = 1;

							// backlight on: default functions enabled only on display (E02)
							configuration_variables.ui8_function_code = DEFAULT_ENABLED_ON_OEM;

							// clear lights counter
							ui8_lights_counter = 0;
						}

						// after some seconds: switch on lights (if enabled) and abort function
						if(ui8_lights_counter >= DELAY_LIGHTS_ON)
						{
							#if ENABLE_LIGHTS_FROM_OEM
							// set lights flag
							ui8_lights_flag = 1;
							#endif

							// clear default flag
							ui8_default_flag = 0;
						}
					}
				}
				else
				{
					// lights off:
					if(!ui8_lights_flag)
					{
						// default flag active:
						if(ui8_default_flag)
						{
							// clear default flag
							ui8_default_flag = 0;

							if(configuration_variables.ui8_street_enabled_on_startup)
								// enable street mode on startup!
								configuration_variables.ui8_offroad_mode = 0;
							else
								// enable offroad mode on startup!
								configuration_variables.ui8_offroad_mode = 1;

							// disable power boost
							configuration_variables.ui8_startup_motor_power_boost_feature_enabled = 0;

							// default functions enabled!
							configuration_variables.ui8_function_code = DEFAULT_FUNCTION_ENABLED;
						}
					}
					// lights on:
					else
					{
						// clear lights flag
						ui8_lights_flag = 0;
					}
				}
			}
			else
			{
				// clear default flag
				ui8_default_flag = 0;
			}

			//-----------------------------------------------------
			// ASSIST LEVEL1: OFFROAD/STREET MODE SELECTION
			//-----------------------------------------------------
			// assist pedal level 1:
			if(ui8_assist_level == ASSIST_PEDAL_LEVEL1)
			{
				// Display down button pressed:
				if(ui8_rx_buffer[1] & 0x01)
				{
					// lights off:
					if(!ui8_lights_flag)
					{
						// mode flag cleared
						if(!ui8_mode_flag)
						{
							// set mode flag
							ui8_mode_flag = 1;

							if(configuration_variables.ui8_offroad_mode == 1)
								// backlight on: street function enabled only on display (E03)
								configuration_variables.ui8_function_code = STREET_ENABLED_ON_OEM;
							else
								// backlight on: offroad function enabled only on display (E04)
								configuration_variables.ui8_function_code = OFFROAD_ENABLED_ON_OEM;

								// clear lights counter
								ui8_lights_counter = 0;
						}

						// after some seconds: switch on lights (if enabled) and abort function
						if(ui8_lights_counter >= DELAY_LIGHTS_ON)
						{
							#if ENABLE_LIGHTS_FROM_OEM
							// set lights flag
							ui8_lights_flag = 1;
							#endif

							// clear mode flag
							ui8_mode_flag = 0;
						}
					}
				}
				else
				{
					// lights off:
					if(!ui8_lights_flag)
					{
						// mode flag active:
						if(ui8_mode_flag)
						{
							// clear default flag
							ui8_mode_flag = 0;

							if(configuration_variables.ui8_offroad_mode == 1)
							{
								// enable street mode
								configuration_variables.ui8_offroad_mode = 0;

								// street mode enabled!
								configuration_variables.ui8_function_code = STREET_FUNCTION_ENABLED;
							}
							else
							{
								// enable offroad mode
								configuration_variables.ui8_offroad_mode = 1;

								// offroad mode enabled!
								configuration_variables.ui8_function_code = OFFROAD_FUNCTION_ENABLED;
							}
						}
					}
					// lights on:
					else
					{
						// clear lights flag
						ui8_lights_flag = 0;
					}
				}
			}
			else
			{
				// clear default flag
				ui8_mode_flag = 0;
			}

			//-----------------------------------------------------
			// ASSIST LEVEL2: ENABLE/DISABLE POWER BOOST FUNCTION
			//-----------------------------------------------------
			// assist pedal level 2:
			if(ui8_assist_level == ASSIST_PEDAL_LEVEL2)
			{
				// Display down button pressed:
				if(ui8_rx_buffer[1] & 0x01)
				{
					// lights off:
					if(!ui8_lights_flag)
					{
						// mode boost cleared
						if(!ui8_boost_flag)
						{
							// set boost flag
							ui8_boost_flag = 1;

							if(configuration_variables.ui8_startup_motor_power_boost_feature_enabled == 0)
								// backlight on: power boost enabled only on display (E05)
								configuration_variables.ui8_function_code = BOOST_ENABLED_ON_OEM;
							else
								// backlight on: power boost disables only on display (E01)
								configuration_variables.ui8_function_code = BOOST_DISABLED_ON_OEM;

							// clear lights counter
							ui8_lights_counter = 0;
						}

						// after some seconds: switch on lights (if enabled) and abort function
						if(ui8_lights_counter >= DELAY_LIGHTS_ON)
						{
							#if ENABLE_LIGHTS_FROM_OEM
							// set lights flag
							ui8_lights_flag = 1;
							#endif

							// clear boost flag
							ui8_boost_flag = 0;
						}
					}
				}
				else
				{
					// lights off:
					if(!ui8_lights_flag)
					{
						// boost flag active:
						if(ui8_boost_flag)
						{
							// clear boost flag
							ui8_boost_flag = 0;

							if(configuration_variables.ui8_startup_motor_power_boost_feature_enabled == 0)
							{
								// enable power boost function
								configuration_variables.ui8_startup_motor_power_boost_feature_enabled = 1;

								// power boost function enabled!
								configuration_variables.ui8_function_code = BOOST_FUNCTION_ENABLED;
							}
							else
							{
								// disable power boost function
								configuration_variables.ui8_startup_motor_power_boost_feature_enabled = 0;

								// power boost function disabled!
								configuration_variables.ui8_function_code = BOOST_FUNCTION_DISABLED;
							}
						}
					}
					// lights on:
					else
					{
						// clear lights flag
						ui8_lights_flag = 0;
					}
				}
			}
			else
			{
				// clear boost flag
				ui8_boost_flag = 0;
			}

			//-----------------------------------------------------
			// ASSIST LEVEL3: SWITCH ON/SWITCH OFF LIGHTS
			//-----------------------------------------------------
			// assist pedal level 3:
			if(ui8_assist_level == ASSIST_PEDAL_LEVEL3)
			{
				#if ENABLE_LIGHTS_FROM_OEM
				if(ui8_rx_buffer[1] & 0x01)
					// Display lights enabled: set lights flag
					ui8_lights_flag = 1;
				else
					// Display lights disabled: clear lights flag
					ui8_lights_flag = 0;
				#endif
			}

			//-----------------------------------------------------
			// ASSIST LEVEL4: SWITCH ON/SWITCH OFF LIGHTS
			//-----------------------------------------------------
			// assist pedal level 4:
			if(ui8_assist_level == ASSIST_PEDAL_LEVEL4)
			{
				#if ENABLE_LIGHTS_FROM_OEM
				if(ui8_rx_buffer[1] & 0x01)
					// Display lights enabled: set lights flag
					ui8_lights_flag = 1;
				else
					// Display lights disabled: clear lights flag
					ui8_lights_flag = 0;
				#endif
			}

			#if ENABLE_LIGHTS_FROM_OEM
			// lights on: abort pending function!
			if(ui8_lights_flag)
				configuration_variables.ui8_function_code = PENDING_FUNCTION_ABORTED;

			// switch on/switch off lights
			configuration_variables.ui8_lights = ui8_lights_flag;
			lights_set_state(configuration_variables.ui8_lights);
			#endif

			//-----------------------------------------------------
			// ENABLE/DISABLE WALK ASSIST FUNCTION
			//-----------------------------------------------------
			#if ENABLE_WALK_ASSIST_FROM_OEM
			// walk assist enabled?
			if(ui8_enable_walk_assist)
			{
				// walk assist button pressed?
				if(ui8_rx_buffer[1] & 0x20)
				{
					// set walk assist flag
					ui8_walk_assist_flag = 1;

					// clear walk_assist_delay_off counter
					ui16_walk_assist_delay_off = 0;

					// get walk assist percentage current (%)
					ui8_walk_assist_current_per_cent = configuration_variables.ui8_walk_assist_percentage_current;
				}
				else
				{
					#if ENABLE_WALK_ASSIST_OFF_DELAY
					if(ui8_assist_level == ASSIST_PEDAL_LEVEL1)
					{
						// walk assist active and walk assist started?
						if((ui8_walk_assist_flag)&&(ui8_walk_assist_start))
						{
							// walk assist delay off active
							ui8_walk_assist_delay_off_flag = 1;							
							
							// walk assist delay off expired
							if(ui16_walk_assist_delay_off >= configuration_variables.ui16_walk_assist_off_delay_time)
							{
								// walk assist stop!
								ui8_walk_assist_current_per_cent = 0;
							}
						}
					}
					else
					{
						ui8_walk_assist_current_per_cent = 0;
					}
					#else
					ui8_walk_assist_current_per_cent = 0;
					#endif
				}
			}
			else
			{
				ui8_walk_assist_current_per_cent = 0;
			}
			#else
			ui8_walk_assist_current_per_cent = 0;
			#endif

			// battery max current
			ebike_app_set_battery_max_current(configuration_variables.ui8_battery_max_current);

			// battery low voltage cut-off: calc the value in ADC steps and set it up
			ui32_temp = ((uint32_t) configuration_variables.ui16_battery_low_voltage_cut_off_x10 << 8) / ((uint32_t) ADC8BITS_BATTERY_VOLTAGE_PER_ADC_STEP_INVERSE_X256);
			ui32_temp /= 10;
			motor_set_adc_battery_voltage_cut_off((uint8_t) ui32_temp);

			#if ENABLE_WHEEL_PERIMETER_FROM_OEM
			// get wheel perimeter from display
			configuration_variables.ui16_wheel_perimeter = ui8_rx_buffer[3] * 80; // wheel diameter (inch) * 3.14 * 2.54 * 10
			#else
			#if 1
			uint16_t ui16_temp;
			uint8_t ui8_temp;

			// get wheel perimeter from eeprom
			ui16_temp = FLASH_ReadByte(ADDR_WHEEL_PERIMETER_0);
			ui8_temp = FLASH_ReadByte(ADDR_WHEEL_PERIMETER_1);
			ui16_temp += (((uint16_t) ui8_temp << 8) & 0xFF00);
			configuration_variables.ui16_wheel_perimeter = ui16_temp;
			#else
			uint8_t ui8_index;

			// convert wheel diameter in index (es. 12" = 1... 26" = 15... 31" = 20)
			ui8_index = ui8_rx_buffer[3] - 11;
			#endif
			#endif

			#if ENABLE_WHEEL_MAX_SPEED_FROM_OEM
			// offroad mode active: get wheel max speed from display
			if(configuration_variables.ui8_offroad_mode)
				configuration_variables.ui8_wheel_max_speed = ui8_rx_buffer[5];
			#else
			// offroad mode active: get wheel max speed from eeprom
			if(configuration_variables.ui8_offroad_mode)
				configuration_variables.ui8_wheel_max_speed = FLASH_ReadByte(ADDR_WHEEL_MAX_SPEED);
			#endif

			#if ENABLE_EEPROM_WRITE_IF_CHANGED
			// verify if any configuration_variables did change and if so, save all of them in the EEPROM
			eeprom_write_if_values_changed();
			#endif

			// signal that we processed the full package
			ui8_received_package_flag = 0;

			ui8_uart_received_first_package = 1;
		}

		// enable UART2 receive interrupt as we are now ready to receive a new package
		UART2->CR2 |= (1 << 5);
	}

	// send packege to display
	uart_send_package();
	#endif
}

static void uart_send_package(void)
{
 	#if !ENABLE_DEBUG_UART
	uint8_t ui8_tx_check_code;

	// send the data to the LCD
	// start up byte
	ui8_tx_buffer[0] = 0x43;

	// clear fault code
	configuration_variables.ui8_fault_code = NO_FAULT;

	// initialize working status
	configuration_variables.ui8_working_status &= 0xFE; // bit0 = 0 (battery normal)

	#if ENABLE_VLCD6_COMPATIBILITY
	switch(ui8_battery_state_of_charge)
	{
		case 0:
			configuration_variables.ui8_working_status |= 0x01; // bit0 = 1 (battery undervoltage)
			ui8_tx_buffer[1] = 0x00;
			break;

		case 1:
			ui8_tx_buffer[1] = 0x00; // Battery 0/4 (empty and blinking)
			break;

		case 2:
			ui8_tx_buffer[1] = 0x02; // Battery 1/4
			break;

		case 3:
			ui8_tx_buffer[1] = 0x06; // Battery 2/4
			break;

		case 4:
			ui8_tx_buffer[1] = 0x09; // Battery 3/4
			break;

		case 5:
			ui8_tx_buffer[1] = 0x0C; // Battery 4/4 (full)
			break;

		case 6:
			ui8_tx_buffer[1] = 0x0C; // Battery 4/4 (full)
			configuration_variables.ui8_fault_code = OVERVOLTAGE; // Fault overvoltage
			break;
	}
	#endif

	#if ENABLE_VLCD5_COMPATIBILITY
	switch(ui8_battery_state_of_charge)
	{
		case 0:
			configuration_variables.ui8_working_status |= 0x01; // bit0 = 1 (battery undervoltage)
			ui8_tx_buffer[1] = 0x00;
			break;

		case 1:
			ui8_tx_buffer[1] = 0x00; // Battery 0/6 (empty and blinking)
			break;

		case 2:
			ui8_tx_buffer[1] = 0x02; // Battery 1/6
			break;

		case 3:
			ui8_tx_buffer[1] = 0x04; // Battery 2/6
			break;

		case 4:
			ui8_tx_buffer[1] = 0x06; // Battery 3/6
			break;

		case 5:
			ui8_tx_buffer[1] = 0x08; // Battery 4/6
			break;

		case 6:
			ui8_tx_buffer[1] = 0x0A; // Battery 5/6
			break;

		case 7:
			ui8_tx_buffer[1] = 0x0C; // Battery 6/6 (full)
			break;

		case 8:
			ui8_tx_buffer[1] = 0x0C; // Battery 6/6 (full)
			configuration_variables.ui8_fault_code = OVERVOLTAGE; // Fault overvoltage
			break;
	}
	#endif

	// reserved
	ui8_tx_buffer[3] = 0x46;
	ui8_tx_buffer[4] = 0x46;

	// verify if some errors are pending....
	if(configuration_variables.ui8_fault_code == NO_FAULT)
	{
		if(configuration_variables.ui8_error_states & ERROR_STATE_EBIKE_WHEEL_BLOCKED)
			configuration_variables.ui8_fault_code = EBIKE_WHEEL_BLOCKED;
		else if(ui8_overtemperature)
			configuration_variables.ui8_fault_code = OVERTEMPERATURE;		
	}

	// transmit to display function code or fault code
	if(configuration_variables.ui8_function_code != NO_FUNCTION)
	{
		// function code
		ui8_tx_buffer[5] = configuration_variables.ui8_function_code;
	}
	else
	{
		// fault code
		ui8_tx_buffer[5] = configuration_variables.ui8_fault_code;
	}

	// wheel speed
	if(ui16_oem_wheel_speed == 0)
	{
		ui8_tx_buffer[6] = 0x07;
		ui8_tx_buffer[7] = 0x07;

		#if ENABLE_DISPLAY_WORKING_FLAG
		// bit7 = 0 (wheel not turning)
		configuration_variables.ui8_working_status &= 0x7F;
		#endif
	}
	else
	{
		ui8_tx_buffer[6] = (uint8_t) (ui16_oem_wheel_speed & 0xFF);
		ui8_tx_buffer[7] = (uint8_t) (ui16_oem_wheel_speed >> 8);

		#if ENABLE_DISPLAY_WORKING_FLAG
		// bit7 = 1 (wheel turning)
		configuration_variables.ui8_working_status |= 0x80;
		#endif
	}

	#if ENABLE_DISPLAY_ALWAYS_ON
	// set working flag used to hold display always on
	configuration_variables.ui8_working_status |= 0x04;
	#else
	// clear motor working, wheel turning and working flags
	configuration_variables.ui8_working_status &= 0x3B;

	// motor working or wheel turning?
	if(configuration_variables.ui8_working_status & 0xC0)
	{
		// set working flag used by diplay
		configuration_variables.ui8_working_status |= 0x04;
	}
	else
	{
		// clear working flag used by display
		configuration_variables.ui8_working_status &= 0xFB;
	}
	#endif

	// working status
	ui8_tx_buffer[2] = (configuration_variables.ui8_working_status & 0x1F);

	// prepare check code of the package
	ui8_tx_check_code = 0x00;
	for(ui8_i = 0; ui8_i < 8; ui8_i++)
	{
		ui8_tx_check_code += ui8_tx_buffer[ui8_i];
	}
	ui8_tx_buffer[8] = ui8_tx_check_code;

	// send the full package to UART
	for(ui8_i = 0; ui8_i <= 8; ui8_i++)
	{
		putchar(ui8_tx_buffer[ui8_i]);
	}
	#endif
}

// each 1 unit = 0.625 amps
static void ebike_app_set_target_adc_battery_max_current(uint8_t ui8_value)
{
  // limit max number of amps
  if(ui8_value > ui8_adc_battery_current_max)
    ui8_value = ui8_adc_battery_current_max;

  ui8_adc_target_battery_max_current = ui8_adc_battery_current_offset + ui8_value;
}

// in amps
static void ebike_app_set_battery_max_current(uint8_t ui8_value)
{
  // each 1 unit = 0.625 amps (0.625 * 256 = 160)
  ui8_adc_battery_current_max = ((((uint16_t) ui8_value) << 8) / 160);

  if(ui8_adc_battery_current_max > ADC_BATTERY_CURRENT_MAX)
    ui8_adc_battery_current_max = ADC_BATTERY_CURRENT_MAX;
}

static void calc_pedal_force_and_torque(void)
{
  uint16_t ui16_temp;

  // calculate torque on pedals
  ui16_temp = (uint16_t) ui8_torque_sensor * (uint16_t) PEDAL_TORQUE_X100;
  ui16_pedal_torque_x10 = ui16_temp / 10;

  // calculate power on pedals
  // formula for angular velocity in degrees: power  =  force  *  rotations per second  *  2  *  pi
  // formula for angular velocity in degrees: power  =  force  *  rotations per minute  *  2  *  pi / 60
  // (100 * 2 * pi) / 60 = 628

 	// NOTE
	/*
	Users did report that pedal human power is about 2x more.
	@casainho had the idea to evaluate the torque sensor peak signal (measuring peak signal every pedal rotation)
	as being a sinewaveand so the average would be:
	> [Average value = 0.637 � maximum or peak value, Vpk](https://www.electronics-tutorials.ws/accircuits/average-voltage.html)
	For a quick hack, we can just reduce actual value to 0.637.
	105 * (1/0.637) = 165
	*/
	ui16_pedal_power_x10 = (uint16_t) ((((uint32_t) ui16_temp * (uint32_t) ui8_pas_cadence_rpm)) / (uint16_t) PEDAL_POWER_DIVISOR);
}

static void calc_wheel_speed(void)
{
  // calc wheel speed in km/h
  if(ui16_wheel_speed_sensor_pwm_cycles_ticks < WHEEL_SPEED_SENSOR_MIN_PWM_CYCLE_TICKS)
  {
		// Esempio 1:
		// PWM_CYCLES_SECOND = 15625 Hz
		// ui16_wheel_speed_sensor_pwm_cycles_ticks = 1166 (high speed)
  	// ui16_wheel_perimeter = 2074 mm (26")
  	//
  	// 15625 / 1166 = 13.4 rps
  	// 13.4 * 2074 = 27792 mm/sec
  	// 27792 * 0.036 = 100.0 km/h x10
		//	
		// Esempio 2:
		// PWM_CYCLES_SECOND = 15625 Hz
		// ui16_wheel_speed_sensor_pwm_cycles_ticks = 32767 (low speed)
		// ui16_wheel_perimeter = 2074 mm (26")
		//
		// 15625 / 32767 = 0.47685 rps
		// 0.47685 * 2074 = 989 mm/sec
		// 989 * 0.036 = 3.56 km/h x10
			
		f_wheel_speed_x10 = ((float) PWM_CYCLES_SECOND) / ((float) ui16_wheel_speed_sensor_pwm_cycles_ticks); // rps
		f_wheel_speed_x10 *= configuration_variables.ui16_wheel_perimeter; // millimeters per second
		f_wheel_speed_x10 *= 0.036; // ((3600 / (1000 * 1000)) * 10) kms per hour * 10
		ui16_wheel_speed_x10 = (uint16_t) f_wheel_speed_x10;

		// display ready:
		if(ui8_display_ready_flag)
		{
			f_oem_wheel_speed = (((float) ui16_wheel_speed_sensor_pwm_cycles_ticks) * 10.0) / ((float) configuration_variables.ui16_oem_wheel_speed_factor);
			ui16_oem_wheel_speed = (uint16_t) f_oem_wheel_speed;
		}
		// display not ready:
		else
		{
			// clear wheel speed
			ui16_oem_wheel_speed = 0;

			// if startup done, set display ready
			if(ui8_startup_counter++ >= 40)
			{
				//ui8_startup_counter = 0;
				ui8_display_ready_flag = 1;
			}
		}
  }
  else
  {
		ui16_wheel_speed_x10 = 0;
		ui16_oem_wheel_speed = 0;
  }
}

static void calc_motor_temperature(void)
{
  uint16_t ui16_adc_motor_temperatured_filtered_10b;

  // low pass filter to avoid possible fast spikes/noise
  ui16_adc_motor_temperatured_accumulated -= ui16_adc_motor_temperatured_accumulated >> READ_MOTOR_TEMPERATURE_FILTER_COEFFICIENT;
  ui16_adc_motor_temperatured_accumulated += ui16_adc_read_throttle_10b();
  ui16_adc_motor_temperatured_filtered_10b = ui16_adc_motor_temperatured_accumulated >> READ_MOTOR_TEMPERATURE_FILTER_COEFFICIENT;

  configuration_variables.ui16_motor_temperature_x2 = (uint16_t) ((float) ui16_adc_motor_temperatured_filtered_10b / 1.024);
  configuration_variables.ui8_motor_temperature = (uint8_t) (configuration_variables.ui16_motor_temperature_x2 >> 1);
}

static uint16_t calc_filtered_battery_voltage(void)
{
  uint16_t ui16_batt_voltage_filtered = (uint16_t) motor_get_adc_battery_voltage_filtered_10b() * ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X512;
  return (ui16_batt_voltage_filtered >> 9);
}

static void apply_street_mode(uint16_t ui16_battery_voltage, uint8_t *ui8_max_speed, uint8_t *ui8_target_current)
{
  if(!configuration_variables.ui8_offroad_mode)
  {
  	// limit speed if street mode is active
    *ui8_max_speed = configuration_variables.ui8_street_speed_limit;

    if((configuration_variables.ui8_street_power_limit_enabled)&&(configuration_variables.ui8_street_power_limit_div25 > 0))
    {
      uint8_t ui8_street_mode_max_current = (uint8_t) (((((uint32_t) configuration_variables.ui8_street_power_limit_div25) * 160) / ((uint32_t) ui16_battery_voltage)) >> 2);
      *ui8_target_current = ui8_min(ui8_street_mode_max_current, *ui8_target_current);
    }
  }
}

static void apply_speed_limit(uint16_t ui16_speed_x10, uint8_t ui8_max_speed, uint8_t *ui8_target_current)
{
  *ui8_target_current = (uint8_t) (map((uint32_t) ui16_speed_x10,
                                       (uint32_t) ((ui8_max_speed * 10) - 20),
                                       (uint32_t) ((ui8_max_speed * 10) + 20),
                                       (uint32_t) *ui8_target_current,
                                       (uint32_t) 0));
}

#if ENABLE_THROTTLE
static void apply_throttle(uint8_t ui8_throttle_value, uint8_t *ui8_motor_enable, uint8_t *ui8_target_current)
{
  uint8_t ui8_temp = (uint8_t) (map((uint32_t) ui8_throttle_value,
                                    (uint32_t) 0,
                                    (uint32_t) 255,
                                    (uint32_t) 0,
                                    (uint32_t) ui8_adc_battery_current_max));

  // set target current
  *ui8_target_current = ui8_max(*ui8_target_current, ui8_temp);

  // enable motor assistance because user is using throttle
  if(*ui8_target_current){*ui8_motor_enable = 1;}
}
#endif

static void apply_walk_assist(uint8_t ui8_walk_assist_value, uint8_t *ui8_motor_enable, uint8_t *ui8_target_current)
{
	uint8_t ui8_temp = (uint8_t) (map((uint32_t) ui8_walk_assist_value,
                                    (uint32_t) 0,
                                    (uint32_t) 255,
                                    (uint32_t) 0,
                                    (uint32_t) ui8_adc_battery_current_max));

	*ui8_target_current = ui8_max(*ui8_target_current, ui8_temp);

	// flag that motor assistance should happen because we may be running with walk assist
	if(*ui8_target_current){*ui8_motor_enable = 1;}
}

static void apply_temperature_limiting(uint8_t *ui8_target_current)
{
  // min temperature value can't be equal or higher than max temperature value...
  if(configuration_variables.ui8_motor_temperature_min_value_to_limit >= configuration_variables.ui8_motor_temperature_max_value_to_limit)
  {
    *ui8_target_current = 0;
    configuration_variables.ui8_temperature_current_limiting_value = 0;
  }
  else
  {
    // reduce motor current if over temperature
    *ui8_target_current = 
      (uint8_t) (map((uint32_t) configuration_variables.ui16_motor_temperature_x2,
                     (uint32_t) (((uint16_t) configuration_variables.ui8_motor_temperature_min_value_to_limit) << 1),
                     (uint32_t) (((uint16_t) configuration_variables.ui8_motor_temperature_max_value_to_limit) << 1),
                     (uint32_t) *ui8_target_current,
                     (uint32_t) 0));

    // get a value linear to the current limitation, just to show to user
    configuration_variables.ui8_temperature_current_limiting_value = 
      (uint8_t) (map((uint32_t) configuration_variables.ui16_motor_temperature_x2,
                     (uint32_t) (((uint16_t) configuration_variables.ui8_motor_temperature_min_value_to_limit) << 1),
                     (uint32_t) (((uint16_t) configuration_variables.ui8_motor_temperature_max_value_to_limit) << 1),
                     (uint32_t) 255,
                     (uint32_t) 0));
  }
}

#if ENABLE_LAST_BETA_RELEASE || ENABLE_LAST_APP_BOOST_CODE ////////////////////////////////////////////
static void boost_run_statemachine (void)
{
  if(configuration_variables.ui8_startup_motor_power_boost_time > 0)
  {
    switch(ui8_startup_boost_state_machine)
    {
      // ebike is stopped, wait for throttle signal to startup boost
      case BOOST_STATE_BOOST_DISABLED:
        if((ui8_torque_sensor > 12)&&(!brake_is_set()))
        {
          ui8_startup_boost_enable = 1;
          ui8_startup_boost_timer = configuration_variables.ui8_startup_motor_power_boost_time;
          ui8_startup_boost_state_machine = BOOST_STATE_BOOST;
        }
				break;

      case BOOST_STATE_BOOST:
        // braking means reseting
        if(brake_is_set())
        {
          ui8_startup_boost_enable = 0;
          ui8_startup_boost_state_machine = BOOST_STATE_BOOST_DISABLED;
        }

        // end boost if
        if(ui8_torque_sensor < 12)
        {
          ui8_startup_boost_enable = 0;
          ui8_startup_boost_state_machine = BOOST_STATE_BOOST_WAIT_TO_RESTART;
        }

        // decrement timer
        if(ui8_startup_boost_timer > 0){ui8_startup_boost_timer--;}

        // end boost and start fade if
        if(ui8_startup_boost_timer == 0)
        {
          ui8_startup_boost_state_machine = BOOST_STATE_FADE;
          ui8_startup_boost_enable = 0;

          // setup variables for fade
          ui8_startup_boost_fade_steps = configuration_variables.ui8_startup_motor_power_boost_fade_time;
          ui16_startup_boost_fade_variable_x256 = ((uint16_t) ui8_adc_battery_target_current << 8);
          ui16_startup_boost_fade_variable_step_amount_x256 = (ui16_startup_boost_fade_variable_x256 / ((uint16_t) ui8_startup_boost_fade_steps));
          ui8_startup_boost_fade_enable = 1;
        }
				break;

      case BOOST_STATE_FADE:
        // braking means reseting
        if(brake_is_set())
        {
          ui8_startup_boost_fade_enable = 0;
          ui8_startup_boost_fade_steps = 0;
          ui8_startup_boost_state_machine = BOOST_STATE_BOOST_DISABLED;
        }

        if(ui8_startup_boost_fade_steps > 0){ui8_startup_boost_fade_steps--;}

        // disable fade if
        if((ui8_torque_sensor < 12)||(ui8_startup_boost_fade_steps == 0))
        {
          ui8_startup_boost_fade_enable = 0;
          ui8_startup_boost_fade_steps = 0;
          ui8_startup_boost_state_machine = BOOST_STATE_BOOST_WAIT_TO_RESTART;
        }
				break;

      // restart when user is not pressing the pedals AND/OR wheel speed = 0
      case BOOST_STATE_BOOST_WAIT_TO_RESTART:
        // wheel speed must be 0 as also torque sensor
        if((configuration_variables.ui8_startup_motor_power_boost_state & 0x01) == 0)
        {
          if((ui16_wheel_speed_x10 == 0)&&(ui8_torque_sensor < 12))
          {
            ui8_startup_boost_state_machine = BOOST_STATE_BOOST_DISABLED;
          }
        }
        // torque sensor must be 0
        if((configuration_variables.ui8_startup_motor_power_boost_state & 0x01) > 0)
        {
          if((ui8_torque_sensor < 12)||(ui8_pas_cadence_rpm == 0))
          {
            ui8_startup_boost_state_machine = BOOST_STATE_BOOST_DISABLED;
          }
        }
				break;

      default:
				break;
    }
  }
}
#else /////////////////////////////////////////////////////////////////////////////////////////////
static void boost_run_statemachine(void)
{
  if(configuration_variables.ui8_startup_motor_power_boost_time > 0)
  {
    switch(ui8_startup_boost_state_machine)
    {
      // ebike is stopped, wait for throttle signal to startup boost
      case BOOST_STATE_BOOST_DISABLED:
        if((ui8_torque_sensor > 0)&&(!brake_is_set()))
        {
          ui8_startup_boost_state_machine = BOOST_STATE_START_BOOST;
        }
        break;

      case BOOST_STATE_START_BOOST:
        ui8_startup_boost_enable = 1;
        ui8_startup_boost_timer = configuration_variables.ui8_startup_motor_power_boost_time;
        ui8_startup_boost_state_machine = BOOST_STATE_BOOST;
        break;

      case BOOST_STATE_BOOST:
        // decrement timer
        if(ui8_startup_boost_timer > 0)
        {
        	ui8_startup_boost_timer--;
        }

        // disable boost if
        if((ui8_torque_sensor == 0)||(ui8_startup_boost_timer == 0))
        {
          ui8_startup_boost_state_machine = BOOST_STATE_END_BOOST;
        }
        break;

      case BOOST_STATE_END_BOOST:
        ui8_startup_boost_enable = 0;

        // setup variables for fade
        ui8_startup_boost_fade_steps = configuration_variables.ui8_startup_motor_power_boost_fade_time;
        ui16_startup_boost_fade_variable_x256 = ((uint16_t) ui8_adc_battery_target_current << 8);
        ui16_startup_boost_fade_variable_step_amount_x256 = (ui16_startup_boost_fade_variable_x256 / ((uint16_t) ui8_startup_boost_fade_steps));
        ui8_startup_boost_fade_enable = 1;

        ui8_startup_boost_state_machine = BOOST_STATE_FADE;
        break;

      case BOOST_STATE_FADE:
        if(ui8_startup_boost_fade_steps > 0)
        {
        	ui8_startup_boost_fade_steps--;
        }

        // disable fade if
        if((ui8_torque_sensor_raw == 0)||(ui8_pas_cadence_rpm == 0)||(ui8_startup_boost_fade_steps == 0))
        {
          ui8_startup_boost_fade_enable = 0;
          ui8_startup_boost_fade_steps = 0;
          ui8_startup_boost_state_machine = BOOST_STATE_BOOST_WAIT_TO_RESTART;
        }
        break;

      // restart when user is not pressing the pedals AND/OR wheel speed = 0
      case BOOST_STATE_BOOST_WAIT_TO_RESTART:
        // wheel speed must be 0 as also torque sensor
        if((configuration_variables.ui8_startup_motor_power_boost_state & 0x01) == 0)
        {
          if((ui16_wheel_speed_x10 == 0)&&(ui8_torque_sensor_raw == 0))
          {
            ui8_startup_boost_state_machine = BOOST_STATE_BOOST_DISABLED;
          }
        }
        // torque sensor must be 0
        if((configuration_variables.ui8_startup_motor_power_boost_state & 0x01) > 0)
        {
          if((ui8_torque_sensor_raw == 0)||(ui8_pas_cadence_rpm == 0))
          {
            ui8_startup_boost_state_machine = BOOST_STATE_BOOST_DISABLED;
          }
        }
        break;

      default:
      	break;
    }
  }
}
#endif

static uint8_t apply_boost(uint8_t ui8_pas_cadence, uint8_t ui8_max_current_boost_state, uint8_t *ui8_target_current)
{
  uint8_t ui8_boost_enable = (ui8_startup_boost_enable && configuration_variables.ui8_assist_level_factor_x10 && ui8_pas_cadence > 0) ? 1 : 0;

  if(ui8_boost_enable)
  {
		*ui8_target_current = ui8_max_current_boost_state;
  }

  return ui8_boost_enable;
}

static void apply_boost_fade_out(uint8_t *ui8_target_current)
{
  if(ui8_startup_boost_fade_enable)
  {
    // here we try to converge to the regular value, ramping down or up step by step
    uint16_t ui16_adc_battery_target_current_x256 = ((uint16_t) ui8_adc_battery_target_current) << 8;
    if(ui16_startup_boost_fade_variable_x256 > ui16_adc_battery_target_current_x256)
    {
      ui16_startup_boost_fade_variable_x256 -= ui16_startup_boost_fade_variable_step_amount_x256;
    }
    else if(ui16_startup_boost_fade_variable_x256 < ui16_adc_battery_target_current_x256)
    {
      ui16_startup_boost_fade_variable_x256 += ui16_startup_boost_fade_variable_step_amount_x256;
    }

    *ui8_target_current = (uint8_t) (ui16_startup_boost_fade_variable_x256 >> 8);
  }
}

static void read_pas_cadence(void)
{
  // cadence in RPM =  60 / (ui16_pas_timer2_ticks * PAS_NUMBER_MAGNETS * 0.000064)
	#if ENABLE_LAST_BETA_RELEASE || ENABLE_LAST_APP_PAS_CADENCE_RPM_CODE
  if(ui16_pas_pwm_cycles_ticks >= ((uint16_t) PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS)||(ui8_pedaling_direction != 1))
	#else
	if(ui16_pas_pwm_cycles_ticks >= ((uint16_t) PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS))
	#endif
  {
  	ui8_pas_cadence_rpm = 0;
  }
  else
  {
    ui8_pas_cadence_rpm = (uint8_t) (60 / (((float) ui16_pas_pwm_cycles_ticks) * ((float) PAS_NUMBER_MAGNETS) * 0.000064));
	}
}

static void torque_sensor_read(void)
{
  // map value from 0 up to 255
  ui8_torque_sensor_raw = (uint8_t) (map(
      UI8_ADC_TORQUE_SENSOR,
      (uint8_t) ui8_adc_torque_sensor_min_value,
      (uint8_t) ui8_adc_torque_sensor_max_value,
      (uint8_t) 0,
      (uint8_t) 255));
			
	#if ENABLE_LAST_BETA_RELEASE || ENABLE_LAST_APP_PEDALLING_CODE //////////////////////////////////////
  // next state machine is used to filter out the torque sensor signal
  // when user is resting on the pedals
  switch(ui8_tstr_state_machine)
  {
    // ebike is stopped
    case STATE_NO_PEDALLING:
			if((ui8_torque_sensor_raw > 0)&&
         (ui16_wheel_speed_x10))
			{
				ui8_tstr_state_machine = STATE_PEDALLING;
			}
			break;

    // wait on this state and reset when ebike stops
    case STATE_PEDALLING:
			if((ui16_wheel_speed_x10 == 0)&&
         (ui8_torque_sensor_raw == 0))
			{
				ui8_tstr_state_machine = STATE_NO_PEDALLING;
			}
			break;

    default:
			break;
  }			
	#else ///////////////////////////////////////////////////////////////////////////////////////////
  switch(ui8_tstr_state_machine)
  {
  	// ebike is stopped, wait for torque sensor signal
    case STATE_NO_PEDALLING:
			if((ui8_torque_sensor_raw > 0)&&(!brake_is_set()))
			{
				ui8_tstr_state_machine = STATE_STARTUP_PEDALLING;
			}
			break;

    // now count 2 seconds
    case STATE_STARTUP_PEDALLING:
			if(ui8_rtst_counter++ > 20) // 2 seconds
			{
				ui8_rtst_counter = 0;
				ui8_tstr_state_machine = STATE_PEDALLING;
			}

			// ebike is not moving, let's return to begin
			if(ui16_wheel_speed_x10 == 0)
			{
				ui8_rtst_counter = 0;
				ui8_tstr_state_machine = STATE_NO_PEDALLING;
			}
			break;

    // wait on this state and reset when ebike stops
    case STATE_PEDALLING:
			if((ui16_wheel_speed_x10 == 0)&&(ui8_torque_sensor_raw == 0))
			{
				ui8_tstr_state_machine = STATE_NO_PEDALLING;
			}
			break;

    default:
			break;
  }
	#endif

  // bike is moving but user doesn't pedal, disable torque sensor signal because user can be resting the feet on the pedals
  if((ui8_tstr_state_machine == STATE_PEDALLING)&&(ui8_pas_cadence_rpm == 0))
  {
    ui8_torque_sensor = 0;
  }
  else
  {
    ui8_torque_sensor = ui8_torque_sensor_raw;
  }
}

static void throttle_read(void)
{
#if ENABLE_THROTTLE
  // map value from 0 up to 255
  ui8_throttle = (uint8_t) (map(
      UI8_ADC_THROTTLE,
      (uint8_t) ADC_THROTTLE_MIN_VALUE,
      (uint8_t) ADC_THROTTLE_MAX_VALUE,
      (uint8_t) 0,
      (uint8_t) 255));
#else
  ui8_throttle = 0;
#endif
}

static void walk_assist_read(void)
{
	// map value from 0 up to 255
	ui8_walk_assist = (uint8_t) (map(
      ui8_walk_assist_current_per_cent,
      (uint8_t) WALK_ASSIST_MIN_VALUE,
      (uint8_t) WALK_ASSIST_MAX_VALUE,
      (uint8_t) 0,
      (uint8_t) 255));
}

// This is the interrupt that happens when UART2 receives data. We need it to be the fastest possible and so
// we do: receive every byte and assembly as a package, finally, signal that we have a package to process (on main slow loop)
// and disable the interrupt. The interrupt should be enable again on main loop, after the package being processed
void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER)
{
	if(UART2_GetFlagStatus(UART2_FLAG_RXNE) == SET)
  {
    UART2->SR &= (uint8_t)~(UART2_FLAG_RXNE); // this may be redundant

    ui8_byte_received = UART2_ReceiveData8();

    switch(ui8_state_machine)
    {
      case 0:
      if(ui8_byte_received == 0x59) // see if we get start package byte
      {
        ui8_rx_buffer[ui8_rx_counter] = ui8_byte_received;
        ui8_rx_counter++;
        ui8_state_machine = 1;
      }
      else
      {
        ui8_rx_counter = 0;
        ui8_state_machine = 0;
      }
      break;

      case 1:
      ui8_rx_buffer[ui8_rx_counter] = ui8_byte_received;
      ui8_rx_counter++;

      // see if is the last byte of the package
      if(ui8_rx_counter > 8)
      {
        ui8_rx_counter = 0;
        ui8_state_machine = 0;
        ui8_received_package_flag = 1; // signal that we have a full package to be processed
        UART2->CR2 &= ~(1 << 5); // disable UART2 receive interrupt
      }			
      break;

      default:
      break;
    }
  }
}

struct_configuration_variables* get_configuration_variables(void)
{
  return &configuration_variables;
}

static void safe_tests(void)
{
	#if ENABLE_LAST_BETA_RELEASE || ENABLE_LAST_APP_SAFE_TESTS_CODE
	// enabe only next state machine if user has startup without pedal rotation
	if((configuration_variables.ui8_motor_assistance_startup_without_pedal_rotation)||
     (!brake_is_set())||
     (configuration_variables.ui8_assist_level_factor_x10)||
     (!ui8_walk_assist))
	{
	#else
	// enabe only next state machine if user has startup without pedal rotation
	if(configuration_variables.ui8_motor_assistance_startup_without_pedal_rotation)
	{
		// the state machine should restart if:
		if((brake_is_set())|| // we hit brakes
			(configuration_variables.ui8_assist_level_factor_x10 == 0)) // we choose assist power assist level = 0
		{
			configuration_variables.ui8_error_states &= ~ERROR_STATE_EBIKE_WHEEL_BLOCKED; // disable error state in case it was enable
			safe_tests_state_machine = 0;
		}
	#endif
		switch(safe_tests_state_machine)
    {
      // start when we have torque sensor or throttle or walk_assist
      case 0:
				#if ENABLE_LAST_BETA_RELEASE || ENABLE_LAST_APP_SAFE_TESTS_CODE
				if((ui8_torque_sensor > 12)||(ui8_throttle))
				#else
				if((ui8_torque_sensor_raw)||(ui8_throttle)||(ui8_walk_assist))
				#endif
				{
					safe_tests_state_machine_counter = 0;
					safe_tests_state_machine = 1;
					break;
				}
				break;

      // wait during 5 seconds for bicyle wheel speed > 4km/h, if not we have an error
      case 1:
				safe_tests_state_machine_counter++;

				// timeout of 5 seconds, not less to be higher than value on torque_sensor_read()
				// hopefully, 5 seconds is safe enough value, mosfets may not burn in 5 seconds if ebike wheel is blocked
				if(safe_tests_state_machine_counter > 50)
				{
					configuration_variables.ui8_error_states |= ERROR_STATE_EBIKE_WHEEL_BLOCKED;
					safe_tests_state_machine_counter = 0;
					safe_tests_state_machine = 2;
					break;
				}

				// bicycle wheel is rotating so we are safe
				if(ui16_wheel_speed_x10 > 40) // seems that 4km/h may be the min value we can measure for the bicycle wheel speed
				{
					safe_tests_state_machine_counter = 0;
					safe_tests_state_machine = 3;
					break;
				}

				// release of throttle and walk_assist and torque sensor, restart
				#if ENABLE_LAST_BETA_RELEASE || ENABLE_LAST_APP_SAFE_TESTS_CODE
				if((ui8_torque_sensor < 12)&&(ui8_throttle == 0))
				#else
				if((ui8_torque_sensor_raw == 0)&&(ui8_throttle == 0)&&(ui8_walk_assist == 0))
				#endif
				{
					safe_tests_state_machine = 0;
				}
				break;

      // wait 3 consecutive seconds for torque sensor and throttle and walk_assist = 0, then we can restart
      case 2:
				#if ENABLE_LAST_BETA_RELEASE || ENABLE_LAST_APP_SAFE_TESTS_CODE
				if((ui8_torque_sensor < 12)&&(ui8_throttle == 0))
				#else
				if((ui8_torque_sensor_raw == 0)&&(ui8_throttle == 0)&&(ui8_walk_assist == 0))
				#endif
				{
					safe_tests_state_machine_counter++;

					if(safe_tests_state_machine_counter > 30)
					{
						configuration_variables.ui8_error_states &= ~ERROR_STATE_EBIKE_WHEEL_BLOCKED;
						safe_tests_state_machine = 0;
						break;
					}
				}
				// keep reseting the counter so we keep on this state
				else
				{
					safe_tests_state_machine_counter = 0;
				}
				break;

      // wait for bicycle wheel to be stopped so we can start again our state machine
      case 3:
				if(ui16_wheel_speed_x10 == 0)
				{
					safe_tests_state_machine = 0;
					break;
				}
				break;

      default:
      	safe_tests_state_machine = 0;
      	break;
    }
  }
  else
  {
    // keep reseting state machine
    configuration_variables.ui8_error_states &= ~ERROR_STATE_EBIKE_WHEEL_BLOCKED; // disable error state in case it was enable
    safe_tests_state_machine = 0;
  }
}
