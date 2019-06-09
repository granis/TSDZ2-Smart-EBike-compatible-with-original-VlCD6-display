/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include "stm8s.h"
#include "stm8s_it.h"
#include "pins.h"
#include "main.h"
#include "interrupts.h"
#include "brake.h"
#include "motor.h"

//=================================================================================================
//
//=================================================================================================
void EXTI_PORTC_IRQHandler(void) __interrupt(EXTI_PORTC_IRQHANDLER) // Brake signal
{
  if(brake_is_set())
  {
    motor_controller_set_state(MOTOR_CONTROLLER_STATE_BRAKE);
  }
  else
  {
    motor_controller_reset_state(MOTOR_CONTROLLER_STATE_BRAKE);
  }
}

//=================================================================================================
//
//=================================================================================================
void brake_init(void)
{
	#if ENABLE_BRAKE_SENSOR
	//brake pin as external input pin interrupt
  GPIO_Init(BRAKE__PORT,
	    BRAKE__PIN,
	    GPIO_MODE_IN_FL_IT); // floating with external interrupt
	#else
	//brake pin as input with pull-up
  GPIO_Init(BRAKE__PORT,
	    BRAKE__PIN,
	    GPIO_MODE_IN_PU_NO_IT); // pull-up without external interrupt
	#endif

  //initialize the Interrupt sensitivity
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOC,
			    EXTI_SENSITIVITY_RISE_FALL);
}

//=================================================================================================
//
//=================================================================================================
BitStatus brake_is_set(void)
{
  if(GPIO_ReadInputPin(BRAKE__PORT, BRAKE__PIN) == 0)
    return 1;
  else
    return 0;
}

