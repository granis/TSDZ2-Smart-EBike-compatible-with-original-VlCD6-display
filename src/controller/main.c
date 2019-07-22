/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>

#include "main.h"
#include "interrupts.h"
#include "stm8s.h"
#include "pins.h"
#include "uart.h"
#include "pwm.h"
#include "motor.h"
#include "wheel_speed_sensor.h"
#include "brake.h"
#include "pas.h"
#include "adc.h"
#include "timers.h"
#include "ebike_app.h"
#include "torque_sensor.h"
#include "eeprom.h"
#include "lights.h"

/////////////////////////////////////////////////////////////////////////////////////////////
//// Functions prototypes

// main -- start of firmware and main loop
int main(void);

// With SDCC, interrupt service routine function prototypes must be placed in the file that contains main ()
// in order for an vector for the interrupt to be placed in the the interrupt vector space.  It's acceptable
// to place the function prototype in a header file as long as the header file is included in the file that
// contains main ().  SDCC will not generate any warnings or errors if this is not done, but the vector will
// not be in place so the ISR will not be executed when the interrupt occurs.

// Calling a function from interrupt not always works, SDCC manual says to avoid it. Maybe the best is to put
// all the code inside the interrupt

// Local VS global variables
// Sometimes I got the following error when compiling the firmware: motor.asm:750: Error: <r> relocation error
// when I have this code inside a function: "static uint8_t ui8_cruise_counter = 0;"
// and the solution was define the variable as global instead
// Another error example:
// *** buffer overflow detected ***: sdcc terminated
// Caught signal 6: SIGABRT

// PWM cycle interrupt
void TIM1_CAP_COM_IRQHandler(void) __interrupt(TIM1_CAP_COM_IRQHANDLER);
void EXTI_PORTC_IRQHandler(void) __interrupt(EXTI_PORTC_IRQHANDLER);
void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER);

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
#if ENABLE_DEBUG_UART
extern volatile struct_configuration_variables configuration_variables;
#endif

//=================================================================================================
//
//=================================================================================================
int main(void)
{
  uint16_t ui16_TIM3_counter = 0;
  uint16_t ui16_ebike_app_controller_counter = 0;
  uint16_t ui16_motor_controller_counter = 0;
  uint16_t ui16_debug_uart_counter = 0;
  uint16_t ui16_temp = 0;
  uint16_t ui16_throttle_value_filtered = 0;
  uint32_t ui32_delay = 0;

  //set clock at the max 16MHz
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);

  // startup wait (1000ms)...
  for(ui32_delay=0; ui32_delay<1000000; ui32_delay++);

	#if ENABLE_DEBUG_FIRMWARE
	buttons_init();
	leds_init();
	#endif

  // initialize EEPROM
  eeprom_init();

	// init variables with the stored value on EEPROM
  eeprom_init_variables();

  // initialize brake
  brake_init();

	#if ENABLE_BRAKE_SENSOR
  while(brake_is_set()); // hold here while brake is pressed -- this is a protection for development
	#endif

  // initialize lights
  lights_init();

  // initialize uart2
  uart2_init();

  // initialize timer2
  timer2_init();

  // initialize timer3
  timer3_init();

  // initialize adc
  adc_init();

  // initialize torque sensor
  torque_sensor_init();

  // initialize pas
  pas_init();

  // initialize wheel speed sensor
  wheel_speed_sensor_init();

  // initialize hall sensor
  hall_sensor_init();

  // initialize pwm
  pwm_init_bipolar_4q();

  // initialize motor
  motor_init();

  // initialize ebike application
  ebike_app_init();

  // initialize interrupts
  enableInterrupts();

	// main loop
  for(;;)
  {
		// because of continue; at the end of each if code block that will stop the while(1) loop there,
		// the first if block code will have the higher priority over any others
		ui16_TIM3_counter = TIM3_GetCounter();
		if((ui16_TIM3_counter - ui16_motor_controller_counter) > 4) // every 4ms
		{
			ui16_motor_controller_counter = ui16_TIM3_counter;
			motor_controller();
			continue;
		}

		ui16_TIM3_counter = TIM3_GetCounter();
		if((ui16_TIM3_counter - ui16_ebike_app_controller_counter) > 100) // every 100ms
		{
			ui16_ebike_app_controller_counter = ui16_TIM3_counter;
			ebike_app_controller();
			continue;
		}

		#if ENABLE_DEBUG_UART
		ui16_TIM3_counter = TIM3_GetCounter();
		if((ui16_TIM3_counter - ui16_debug_uart_counter) > 1000)
		{
			ui16_debug_uart_counter = ui16_TIM3_counter;

			#if 1
			// sugestion: no more than 6-8 variables printed (takes about 3ms to printf 6 variables)
			printf ("%d,%d,%d,%d,%d,%d,%d,%d\n",
				(int)(PEDAL_POWER_DIVISOR),
				(int)(PEDAL_TORQUE_X100),
				(int)(ADC_BATTERY_CURRENT_RAMP_UP_INVERSE_STEP),
				(int)(PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP),
				(int)(ADC_MOTOR_PHASE_CURRENT_MAX),
				(int)(ADC_BATTERY_CURRENT_MAX),
				(int)(0),
				(int)(0)
				);
			#else
			// sugestion: no more than 6-8 variables printed (takes about 3ms to printf 6 variables)
			printf ("%d,%d,%d,%d\n",
				ui16_motor_get_motor_speed_erps(),
				ui8_duty_cycle,
				ui8_adc_battery_current,
				ui8_foc_angle
				);
			#endif
		}
		#endif
  }
}

