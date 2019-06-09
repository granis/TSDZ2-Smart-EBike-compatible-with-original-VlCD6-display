/*
 * config.h
 *
 *  Automatically created by TSDZ2 Controller's Parameters Configurator
 *  Author: marcoq
 */

#ifndef CONFIG_H_
#define CONFIG_H_

//=================================================================================================
// ENABLES
//=================================================================================================
#define ENABLE_VLCD6 1
#define ENABLE_VLCD5 0
#define ENABLE_SELECTION3 0
#define ENABLE_LIGHTS_FROM_OEM 1
#define ENABLE_BACKWARDS_RESISTANCE_OFF 0
#define ENABLE_STREET_MODE_ON_STARTUP 1
#define ENABLE_WALK_ASSIST_FROM_OEM 1
#define ENABLE_BRAKE_SENSOR 0
#define ENABLE_THROTTLE 0
#define ENABLE_WHEEL_PERIMETER_FROM_OEM	1
#define ENABLE_WHEEL_MAX_SPEED_FROM_OEM 1
#define ENABLE_LAST_BETA_RELEASE 0
#define ENABLE_DISPLAY_WORKING_FLAG 1
#define ENABLE_DISPLAY_ALWAYS_ON 0
#define ENABLE_WALK_ASSIST_OFF_DELAY 0
#define ENABLE_STREET_POWER_LIMIT 1
#define TEMPERATURE_LIMIT_FEATURE_ENABLED 0
#define STARTUP_BOOST_WHEN_SPEED_IS_ZERO 1
#define STARTUP_BOOST_WHEN_CADENCE_IS_ZERO 0
//=================================================================================================
// BATTERY
//=================================================================================================
#define ADC_BATTERY_CURRENT_MAX_LIMIT 18.0
#define TARGET_MAX_BATTERY_POWER (uint16_t) 625
#define BATTERY_MAX_CURRENT_FLOAT 17.0
#define BATTERY_CELLS_NUMBER (uint8_t) 10
#define BATTERY_LOW_VOLTAGE_CUT_OFF_DIV10 29.0
#define BATTERY_PACK_RESISTANCE (uint16_t) 196
#define DIVISOR_FOR_CUTOFF_CALC (uint16_t) 44
#define LI_ION_CELL_OVERVOLT 4.25
#define LI_ION_CELL_VOLTS_100 3.96
#define LI_ION_CELL_VOLTS_75 3.70
#define LI_ION_CELL_VOLTS_50 3.44
#define LI_ION_CELL_VOLTS_25 3.30
#define LI_ION_CELL_EMPTY 3.00
#define CHECKBOX15 0
#define SOC_BATTERY_VOLTAGE_FILTER_COEFFICIENT (uint8_t) 4
#define SOC_BATTERY_CURRENT_FILTER_COEFFICIENT (uint8_t) 4
#define SOC_ADC_BATTERY_VOLTAGE_PER_ADC_STEP 0.0866
#define READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT	(uint8_t) 2
#define READ_BATTERY_CURRENT_FILTER_COEFFICIENT (uint8_t) 2
//=================================================================================================
// MOTOR
//=================================================================================================
#define MOTOR_TYPE_36V 1
#define MOTOR_TYPE_48V 0
#define EXPERIMENTAL_HIGH_CADENCE_MODE 0
#define MOTOR_ASSISTANCE_WITHOUT_PEDAL_ROTATION 0
#define MOTOR_PHASE_CURRENT_MAX_AMP 30.0
#define MOTOR_ROTOR_OFFSET_ANGLE (uint8_t) 10
#define MOTOR_ROTOR_ERPS_START_INTERPOLATION_60_DEGREES	(uint8_t) 10
#define MOTOR_OVER_SPEED_ERPS (uint16_t) 520
#define MOTOR_OVER_SPEED_ERPS_EXPERIMENTAL (uint16_t) 700
#define TXTFIELD25 0
#define TXTFIELD26 0
//=================================================================================================
// PWM DUTY CYCLE
//=================================================================================================
#define PWM_CYCLES_SECOND (uint16_t) 15625
#define PWM_DUTY_CYCLE_MIN (uint8_t) 20
#define PWM_DUTY_CYCLE_MAX (uint8_t) 254
#define PWM_DUTY_CYCLE_RAMP_UP_DIV1000 1.3
#define PWM_DUTY_CYCLE_RAMP_DOWN_DIV1000 1.3
#define CURRENT_RAMP 5.0
#define TXTFIELD34 0
//=================================================================================================
// WHEEL
//=================================================================================================
#define WHEEL_PERIMETER (uint16_t) 2083
#define WHEEL_MAX_SPEED (uint8_t) 45
#define OEM_WHEEL_SPEED_DIVISOR (uint16_t) 315
#define WHEEL_SPEED_SENSOR_MAX_PWM_CYCLE_TICKS (uint16_t) 1166
#define WHEEL_SPEED_SENSOR_MIN_PWM_CYCLE_TICKS (uint16_t) 65534
#define WHEEL_SPEED_PI_CONTROLLER_KP_DIVIDEND (uint8_t) 100
#define WHEEL_SPEED_PI_CONTROLLER_KP_DIVISOR (uint8_t) 4
#define WHEEL_SPEED_PI_CONTROLLER_KI_DIVIDEND (uint8_t) 40
#define WHEEL_SPEED_PI_CONTROLLER_KI_DIVISOR (uint8_t) 6
#define TXTFIELD44 0
#define TXTFIELD45 0
//=================================================================================================
// PAS
//=================================================================================================
#define PAS_NUMBER_MAGNETS (uint8_t) 20
#define TXTFIELD48 0
#define TXTFIELD49 0
//=================================================================================================
// PEDAL ASSIST
//=================================================================================================
#define MASTER_LEVEL_FACTOR 1.0
#define ASSIST_LEVEL_FACTOR_X10_1 (uint16_t) 50
#define ASSIST_LEVEL_FACTOR_X10_2 (uint16_t) 120
#define ASSIST_LEVEL_FACTOR_X10_3 (uint16_t) 210
#define ASSIST_LEVEL_FACTOR_X10_4 (uint16_t) 300
//=================================================================================================
// THROTTLE
//=================================================================================================
#define ADC_THROTTLE_THRESHOLD (uint8_t) 10
#define THROTTLE_FILTER_COEFFICIENT (uint8_t) 1
#define ADC_THROTTLE_MIN_VALUE (uint8_t) 47
#define ADC_THROTTLE_MAX_VALUE (uint8_t) 176
//=================================================================================================
// TORQUE SENSOR
//=================================================================================================
#define ADC_TORQUE_SENSOR_THRESHOLD (uint8_t) 6
#define PEDAL_TORQUE_SENSOR_UNIT 0.55
#define AVERAGE_TORQUE_FACTOR 0.637
//=================================================================================================
// WALK ASSIST
//=================================================================================================
#define WALK_ASSIST_PWM_LEVEL_0 (uint8_t) 10
#define WALK_ASSIST_PWM_LEVEL_1 (uint8_t) 13
#define WALK_ASSIST_PWM_LEVEL_2 (uint8_t) 16
#define WALK_ASSIST_PWM_LEVEL_3 (uint8_t) 19
#define WALK_ASSIST_PWM_LEVEL_4 (uint8_t) 22
#define WALK_ASSIST_PERCENTAGE_CURRENT (uint8_t) 20
#define WALK_ASSIST_MAX_RAMP_TIME_DIV10 1.0
#define WALK_ASSIST_OFF_DELAY_PWM_DIV10 (uint8_t) 20
#define WALK_ASSIST_OFF_DELAY_TIME_DIV10 2.0
//=================================================================================================
// STREET MODE
//=================================================================================================
#define STREET_SPEED_LIMIT (uint8_t) 25
#define STREET_POWER_LIMIT (uint16_t) 250
//=================================================================================================
// MOTOR POWER BOOST
//=================================================================================================
#define STARTUP_MOTOR_POWER_BOOST_LIMIT_MAX_POWER 1
#define STARTUP_MOTOR_BOOST_ASSIST_LEVEL_PERCENT_1 (uint8_t) 100
#define STARTUP_MOTOR_BOOST_ASSIST_LEVEL_PERCENT_2 (uint8_t) 72
#define STARTUP_MOTOR_BOOST_ASSIST_LEVEL_PERCENT_3 (uint8_t) 43
#define STARTUP_MOTOR_BOOST_ASSIST_LEVEL_PERCENT_4 (uint8_t) 15
#define STARTUP_MOTOR_POWER_BOOST_TIME_DIV10 2.0
#define STARTUP_MOTOR_POWER_BOOST_FADE_TIME_DIV10 3.5
//=================================================================================================
// CRUISE CONTROL
//=================================================================================================
#define TXTFIELD83 0
#define TXTFIELD84 0
//=================================================================================================
// MOTOR TEMPERATURE
//=================================================================================================
#define MOTOR_TEMPERATURE_MIN_VALUE_LIMIT (uint8_t) 75
#define MOTOR_TEMPERATURE_MAX_VALUE_LIMIT (uint8_t) 85
#define READ_MOTOR_TEMPERATURE_FILTER_COEFFICIENT (uint8_t) 4
//=================================================================================================
// MAGIC BYTE
//=================================================================================================
#define MAGIC_BYTE (uint8_t) 170
//=================================================================================================
// STVP DIRECTORY
//=================================================================================================
#define STVP_DIRECTORY C:/STMicroelectronics/st_toolset/stvp

#endif /* CONFIG_H_ */
