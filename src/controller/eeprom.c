/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>

#include "eeprom.h"
#include "stm8s.h"
#include "stm8s_flash.h"
#include "ebike_app.h"

static uint8_t array_default_values[EEPROM_BYTES_STORED] =
{
	KEY,
	ASSIST_LEVEL_FACTOR_X10,
	CONFIG_0,
	BATTERY_MAX_CURRENT,
	MOTOR_MAX_POWER_DIV10,
	BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0,
	BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1,
	WHEEL_PERIMETER_0,
	WHEEL_PERIMETER_1,
	WHEEL_MAX_SPEED,
	CONFIG_1,
	STREET_CONFIG,
	STREET_SPEED_LIMIT,
	STREET_POWER_LIMIT_DIV25,
	BATTERY_CELLS_NUMBER,
	BATTERY_PACK_RESISTANCE_0,
	BATTERY_PACK_RESISTANCE_1,
	OEM_WHEEL_SPEED_DIVISOR_0,
	OEM_WHEEL_SPEED_DIVISOR_1,
	ASSIST_LEVEL_FACTOR_1,
	ASSIST_LEVEL_FACTOR_2,
	ASSIST_LEVEL_FACTOR_3,
	ASSIST_LEVEL_FACTOR_4,
	STARTUP_MOTOR_POWER_BOOST_STATE,
	STARTUP_MOTOR_POWER_BOOST_FEATURE_ENABLED,
	STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_1,
	STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_2,
	STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_3,
	STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_4,
	STARTUP_MOTOR_POWER_BOOST_TIME,
	STARTUP_MOTOR_POWER_BOOST_FADE_TIME,
	STARTUP_MOTOR_POWER_BOOST_LIMIT_MAX_POWER,
	TARGET_MAX_BATTERY_POWER_DIV25,
	TEMPERATURE_LIMIT_FEATURE_ENABLED,
	MOTOR_TEMPERATURE_MIN_VALUE_LIMIT,
	MOTOR_TEMPERATURE_MAX_VALUE_LIMIT,
	WALK_ASSIST_PERCENTAGE_CURRENT,
	WALK_ASSIST_PWM_DUTY_CYCLE_LEVEL_0,
	WALK_ASSIST_PWM_DUTY_CYCLE_LEVEL_1,
	WALK_ASSIST_PWM_DUTY_CYCLE_LEVEL_2,
	WALK_ASSIST_PWM_DUTY_CYCLE_LEVEL_3,
	WALK_ASSIST_PWM_DUTY_CYCLE_LEVEL_4,
	WALK_ASSIST_MAX_RAMP_TIME,
	WALK_ASSIST_OFF_DELAY_PWM,
	WALK_ASSIST_OFF_DELAY_TIME_0,
	WALK_ASSIST_OFF_DELAY_TIME_1,
	KEY2
};

static void eeprom_read_values_to_variables(void);
static void eeprom_write_array(uint8_t *array_values);
static void variables_to_array(uint8_t *ui8_array);

//=================================================================================================
//
//=================================================================================================
void eeprom_init(void)
{
	uint8_t ui8_key1;
	uint8_t ui8_key2;

  // start by reading address 0x4000 and address 0x43FF... after see if values are different from our keys,
  // if so mean that eeprom memory is clean and we need to populate: should happen after erasing the microcontroller
  ui8_key1 = FLASH_ReadByte(ADDR_KEY);
  ui8_key2 = FLASH_ReadByte(ADDR_KEY2);
  if((ui8_key1 != KEY)||(ui8_key2 != KEY2)) // verify if our keys exist
  {
  	eeprom_write_array(array_default_values);
  }
}

//=================================================================================================
//
//=================================================================================================
void eeprom_init_variables(void)
{
  struct_configuration_variables *p_configuration_variables;
  p_configuration_variables = get_configuration_variables();

  eeprom_read_values_to_variables();

  // now verify if any EEPROM saved value is out of valid range and if so,
  // write correct ones and read again
  if((p_configuration_variables->ui8_battery_max_current > 100)||
		 (p_configuration_variables->ui8_motor_power_div10 > 195)||
		 (p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 > 630)||
		 (p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 < 160)||
		 (p_configuration_variables->ui16_wheel_perimeter > 3000)||
		 (p_configuration_variables->ui16_wheel_perimeter < 750)||
		 (p_configuration_variables->ui8_battery_cells_number > 15)||
		 (p_configuration_variables->ui8_battery_cells_number < 6)||
		 (p_configuration_variables->ui8_assist_level_factor_x10 > 50)||
		 (p_configuration_variables->ui8_assist_level_power[0] > 50)||
		 (p_configuration_variables->ui8_assist_level_power[1] > 50)||
		 (p_configuration_variables->ui8_assist_level_power[2] > 50)||
		 (p_configuration_variables->ui8_assist_level_power[3] > 50)||
		 (p_configuration_variables->ui8_wheel_max_speed > 99))
  {
    eeprom_write_array(array_default_values);
    eeprom_read_values_to_variables();
  }
}

//=================================================================================================
//
//=================================================================================================
static void eeprom_read_values_to_variables(void)
{
  static uint8_t ui8_temp;
  static uint16_t ui16_temp;

  struct_configuration_variables *p_configuration_variables;
  p_configuration_variables = get_configuration_variables();

	p_configuration_variables->ui8_assist_level_factor_x10 = FLASH_ReadByte(ADDR_ASSIST_LEVEL_FACTOR_X10);

  ui8_temp = FLASH_ReadByte(ADDR_CONFIG_0);
  p_configuration_variables->ui8_lights = ui8_temp & 1 ? 1 : 0;
  p_configuration_variables->ui8_walk_assist = ui8_temp & (1 << 1) ? 1 : 0;
  p_configuration_variables->ui8_offroad_mode = ui8_temp & (1 << 2) ? 1 : 0;
  p_configuration_variables->ui8_emtb_mode = ui8_temp & (1 << 3) ? 1 : 0;
  p_configuration_variables->ui8_emtb_enabled_on_startup = ui8_temp & (1 << 4) ? 1 : 0;

  p_configuration_variables->ui8_battery_max_current = FLASH_ReadByte(ADDR_BATTERY_MAX_CURRENT);
  p_configuration_variables->ui8_motor_power_div10 = FLASH_ReadByte(MOTOR_MAX_POWER_DIV10);

  ui16_temp = FLASH_ReadByte(ADDR_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0);
  ui8_temp = FLASH_ReadByte(ADDR_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1);
  ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
  p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 = ui16_temp;

  ui16_temp = FLASH_ReadByte(ADDR_WHEEL_PERIMETER_0);
  ui8_temp = FLASH_ReadByte(ADDR_WHEEL_PERIMETER_1);
  ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
  p_configuration_variables->ui16_wheel_perimeter = ui16_temp;

  p_configuration_variables->ui8_wheel_max_speed = FLASH_ReadByte(ADDR_WHEEL_MAX_SPEED);

  ui8_temp = FLASH_ReadByte(ADDR_CONFIG_1);
  p_configuration_variables->ui8_motor_type = ui8_temp & 3;
  p_configuration_variables->ui8_motor_assistance_startup_without_pedal_rotation = (ui8_temp & 4) >> 2;

  ui8_temp = FLASH_ReadByte(ADDR_STREET_CONFIG);
  p_configuration_variables->ui8_street_feature_enabled = ui8_temp & 1;
  p_configuration_variables->ui8_street_enabled_on_startup = ui8_temp & (1 << 1);
  p_configuration_variables->ui8_street_power_limit_enabled = ui8_temp & (1 << 2);

  p_configuration_variables->ui8_street_speed_limit = FLASH_ReadByte(ADDR_STREET_SPEED_LIMIT);
  p_configuration_variables->ui8_street_power_limit_div25 = FLASH_ReadByte(ADDR_STREET_POWER_LIMIT_DIV25);
	
	p_configuration_variables->ui8_battery_cells_number = FLASH_ReadByte(ADDR_BATTERY_CELLS_NUMBER);
	
	ui16_temp = FLASH_ReadByte(ADDR_BATTERY_PACK_RESISTANCE_0);
	ui8_temp = FLASH_ReadByte(ADDR_BATTERY_PACK_RESISTANCE_1);
	ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
	p_configuration_variables->ui16_battery_pack_resistance_x1000 = ui16_temp;
	
	ui16_temp = FLASH_ReadByte(ADDR_OEM_WHEEL_SPEED_DIVISOR_0);
	ui8_temp = FLASH_ReadByte(ADDR_OEM_WHEEL_SPEED_DIVISOR_1);
	ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
	p_configuration_variables->ui16_oem_wheel_speed_factor = ui16_temp;

	p_configuration_variables->ui8_assist_level_power[0] = FLASH_ReadByte(ADDR_ASSIST_LEVEL_FACTOR_1);
	p_configuration_variables->ui8_assist_level_power[1] = FLASH_ReadByte(ADDR_ASSIST_LEVEL_FACTOR_2);
	p_configuration_variables->ui8_assist_level_power[2] = FLASH_ReadByte(ADDR_ASSIST_LEVEL_FACTOR_3);
	p_configuration_variables->ui8_assist_level_power[3] = FLASH_ReadByte(ADDR_ASSIST_LEVEL_FACTOR_4);

	p_configuration_variables->ui8_startup_motor_power_boost_state = FLASH_ReadByte(ADDR_STARTUP_MOTOR_POWER_BOOST_STATE);
	p_configuration_variables->ui8_startup_motor_power_boost_feature_enabled = FLASH_ReadByte(ADDR_STARTUP_MOTOR_POWER_BOOST_FEATURE_ENABLED);
	p_configuration_variables->ui8_startup_motor_power_boost[0] = FLASH_ReadByte(ADDR_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_1);
	p_configuration_variables->ui8_startup_motor_power_boost[1] = FLASH_ReadByte(ADDR_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_2);
	p_configuration_variables->ui8_startup_motor_power_boost[2] = FLASH_ReadByte(ADDR_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_3);
	p_configuration_variables->ui8_startup_motor_power_boost[3] = FLASH_ReadByte(ADDR_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_4);
	p_configuration_variables->ui8_startup_motor_power_boost_time = FLASH_ReadByte(ADDR_STARTUP_MOTOR_POWER_BOOST_TIME);
	p_configuration_variables->ui8_startup_motor_power_boost_fade_time = FLASH_ReadByte(ADDR_STARTUP_MOTOR_POWER_BOOST_FADE_TIME);
	p_configuration_variables->ui8_startup_motor_power_boost_limit_to_max_power = FLASH_ReadByte(ADDR_STARTUP_MOTOR_POWER_BOOST_LIMIT_MAX_POWER);

	p_configuration_variables->ui8_target_battery_max_power_div25 = FLASH_ReadByte(ADDR_TARGET_MAX_BATTERY_POWER_DIV25);
	
	p_configuration_variables->ui8_temperature_limit_feature_enabled = FLASH_ReadByte(ADDR_TEMPERATURE_LIMIT_FEATURE_ENABLED);
	p_configuration_variables->ui8_motor_temperature_min_value_to_limit = FLASH_ReadByte(ADDR_MOTOR_TEMPERATURE_MIN_VALUE_LIMIT);
	p_configuration_variables->ui8_motor_temperature_max_value_to_limit = FLASH_ReadByte(ADDR_MOTOR_TEMPERATURE_MAX_VALUE_LIMIT);

	p_configuration_variables->ui8_walk_assist_percentage_current = FLASH_ReadByte(ADDR_WALK_ASSIST_PERCENTAGE_CURRENT);
	p_configuration_variables->ui8_walk_assist_pwm_duty_cycle_level[0] = FLASH_ReadByte(ADDR_WALK_ASSIST_PWM_DUTY_CYCLE_LEVEL_0);
	p_configuration_variables->ui8_walk_assist_pwm_duty_cycle_level[1] = FLASH_ReadByte(ADDR_WALK_ASSIST_PWM_DUTY_CYCLE_LEVEL_1);
	p_configuration_variables->ui8_walk_assist_pwm_duty_cycle_level[2] = FLASH_ReadByte(ADDR_WALK_ASSIST_PWM_DUTY_CYCLE_LEVEL_2);
	p_configuration_variables->ui8_walk_assist_pwm_duty_cycle_level[3] = FLASH_ReadByte(ADDR_WALK_ASSIST_PWM_DUTY_CYCLE_LEVEL_3);
	p_configuration_variables->ui8_walk_assist_pwm_duty_cycle_level[4] = FLASH_ReadByte(ADDR_WALK_ASSIST_PWM_DUTY_CYCLE_LEVEL_4);
	p_configuration_variables->ui8_walk_assist_ramp_time = FLASH_ReadByte(ADDR_WALK_ASSIST_MAX_RAMP_TIME);
	p_configuration_variables->ui8_walk_assist_off_delay_pwm = FLASH_ReadByte(ADDR_WALK_ASSIST_OFF_DELAY_PWM);
	ui16_temp = FLASH_ReadByte(ADDR_WALK_ASSIST_OFF_DELAY_TIME_0);
	ui8_temp = FLASH_ReadByte(ADDR_WALK_ASSIST_OFF_DELAY_TIME_1);
	ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
	p_configuration_variables->ui16_walk_assist_off_delay_time = ui16_temp;
	p_configuration_variables->ui8_walk_assist_pwm_duty_cycle = p_configuration_variables->ui8_walk_assist_pwm_duty_cycle_level[0];
}

//=================================================================================================
//
//=================================================================================================
void eeprom_write_variables(void)
{
  uint8_t array_variables[EEPROM_BYTES_STORED];
  variables_to_array(array_variables);
  eeprom_write_array(array_variables);
}

//=================================================================================================
//
//=================================================================================================
static void variables_to_array(uint8_t *ui8_array)
{
  struct_configuration_variables *p_configuration_variables;
  p_configuration_variables = get_configuration_variables();

  ui8_array[0] = KEY;
  ui8_array[1] = p_configuration_variables->ui8_assist_level_factor_x10;
  ui8_array[2] = (p_configuration_variables->ui8_lights & 1) |
                 ((p_configuration_variables->ui8_walk_assist & 1) << 1) |
                 ((p_configuration_variables->ui8_offroad_mode & 1) << 2) |
								 ((p_configuration_variables->ui8_emtb_mode & 1) << 3) |
								 ((p_configuration_variables->ui8_emtb_enabled_on_startup) << 4);
  ui8_array[3] = p_configuration_variables->ui8_battery_max_current;
  ui8_array[4] = p_configuration_variables->ui8_motor_power_div10;
  ui8_array[5] = p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 & 255;
  ui8_array[6] = (p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 >> 8) & 255;
  ui8_array[7] = p_configuration_variables->ui16_wheel_perimeter & 255;
  ui8_array[8] = (p_configuration_variables->ui16_wheel_perimeter >> 8) & 255;
  ui8_array[9] = p_configuration_variables->ui8_wheel_max_speed;
  ui8_array[10] = (p_configuration_variables->ui8_motor_type & 3) |
                  ((p_configuration_variables->ui8_motor_assistance_startup_without_pedal_rotation & 1) << 2);
  ui8_array[11] = (p_configuration_variables->ui8_street_feature_enabled & 1) |
                  ((p_configuration_variables->ui8_street_enabled_on_startup & 1) << 1) |
                  ((p_configuration_variables->ui8_street_power_limit_enabled & 1) << 2);
  ui8_array[12] = p_configuration_variables->ui8_street_speed_limit;
  ui8_array[13] = p_configuration_variables->ui8_street_power_limit_div25;
	ui8_array[14] = p_configuration_variables->ui8_battery_cells_number;
	ui8_array[15] = p_configuration_variables->ui16_battery_pack_resistance_x1000 & 255;
  ui8_array[16] = (p_configuration_variables->ui16_battery_pack_resistance_x1000 >> 8) & 255;
	ui8_array[17] = p_configuration_variables->ui16_oem_wheel_speed_factor & 255;
  ui8_array[18] = (p_configuration_variables->ui16_oem_wheel_speed_factor >> 8) & 255;
	
	ui8_array[19] = p_configuration_variables->ui8_assist_level_power[0];
	ui8_array[20] = p_configuration_variables->ui8_assist_level_power[1];
	ui8_array[21] = p_configuration_variables->ui8_assist_level_power[2];
	ui8_array[22] = p_configuration_variables->ui8_assist_level_power[3];
	
	ui8_array[23] = p_configuration_variables->ui8_startup_motor_power_boost_state;
	ui8_array[24] = p_configuration_variables->ui8_startup_motor_power_boost_feature_enabled;
	ui8_array[25] = p_configuration_variables->ui8_startup_motor_power_boost[0];
	ui8_array[26] = p_configuration_variables->ui8_startup_motor_power_boost[1];
	ui8_array[27] = p_configuration_variables->ui8_startup_motor_power_boost[2];
	ui8_array[28] = p_configuration_variables->ui8_startup_motor_power_boost[3];
	ui8_array[29] = p_configuration_variables->ui8_startup_motor_power_boost_time;
	ui8_array[30] = p_configuration_variables->ui8_startup_motor_power_boost_fade_time;
	ui8_array[31] = p_configuration_variables->ui8_startup_motor_power_boost_limit_to_max_power;

	ui8_array[32] = p_configuration_variables->ui8_target_battery_max_power_div25;

	ui8_array[33] = p_configuration_variables->ui8_temperature_limit_feature_enabled;
	ui8_array[34] = p_configuration_variables->ui8_motor_temperature_min_value_to_limit;
	ui8_array[35] = p_configuration_variables->ui8_motor_temperature_max_value_to_limit;

	ui8_array[36] = p_configuration_variables->ui8_walk_assist_percentage_current;
	ui8_array[37] = p_configuration_variables->ui8_walk_assist_pwm_duty_cycle_level[0];
	ui8_array[38] = p_configuration_variables->ui8_walk_assist_pwm_duty_cycle_level[1];
	ui8_array[39] = p_configuration_variables->ui8_walk_assist_pwm_duty_cycle_level[2];
	ui8_array[40] = p_configuration_variables->ui8_walk_assist_pwm_duty_cycle_level[3];
	ui8_array[41] = p_configuration_variables->ui8_walk_assist_pwm_duty_cycle_level[4];
	ui8_array[42] = p_configuration_variables->ui8_walk_assist_ramp_time;
	ui8_array[43] = p_configuration_variables->ui8_walk_assist_off_delay_pwm;
	ui8_array[44] = p_configuration_variables->ui16_walk_assist_off_delay_time & 255;
	ui8_array[45] = (p_configuration_variables->ui16_walk_assist_off_delay_time >> 8) & 255;
	ui8_array[46] = KEY2;
}

//=================================================================================================
//
//=================================================================================================
static void eeprom_write_array(uint8_t *array)
{
  uint8_t ui8_i;

  FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);
  
  FLASH_Unlock(FLASH_MEMTYPE_DATA); // Unlock Data memory  
  while(FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET) { } // Wait until Data EEPROM area unlocked flag is set

  for(ui8_i = 0; ui8_i < EEPROM_BYTES_STORED; ui8_i++)
  {
    FLASH_ProgramByte(EEPROM_BASE_ADDR + ui8_i, *array++);
  }

  FLASH_Lock(FLASH_MEMTYPE_DATA);
}

#if ENABLE_EEPROM_WRITE_IF_CHANGED
//=================================================================================================
//
//=================================================================================================
void eeprom_write_if_values_changed(void)
{
	// 2018.08.29:
	// NOTE: the next code gives a problem with motor, when we exchange assist level on LCD3 and
	// all the variables all written to EEPROM. As per datasheet, seems each byte takes ~6ms to be written
	// I am not sure the issue is the amount of time...

	uint8_t ui8_index;

	uint8_t array_variables[EEPROM_BYTES_STORED];
	variables_to_array(array_variables);

	ui8_index = 1; // do not verify the first byte: ADDR_KEY
	while(ui8_index < EEPROM_BYTES_STORED)
	{
		if(array_variables[ui8_index] != FLASH_ReadByte(EEPROM_BASE_ADDR + ui8_index))
		{
			eeprom_write_array(array_variables);
			break; // exit the while loop
		}
		
			ui8_index++;
	}
}
#endif
