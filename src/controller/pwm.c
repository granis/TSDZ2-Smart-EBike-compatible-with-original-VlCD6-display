/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>

#include "stm8s_tim1.h"
#include "stm8s_flash.h"
#include "interrupts.h"
#include "pwm.h"
#include "pins.h"

//=================================================================================================
//
//=================================================================================================
void pwm_init_bipolar_4q(void)
{
	// verify if PWM N channels are active on option bytes, if not, enable
  FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);
  if(FLASH_ReadOptionByte(0x4803) != 0x20)
  {
    FLASH_Unlock(FLASH_MEMTYPE_DATA);
    FLASH_EraseOptionByte(0x4803);
    FLASH_ProgramOptionByte(0x4803, 0x20);
    FLASH_Lock(FLASH_MEMTYPE_DATA);
  }

#if ENABLE_BACKWARDS_RESISTANCE_OFF
  TIM1_TimeBaseInit(0, // TIM1_Prescaler = 0
        TIM1_COUNTERMODE_CENTERALIGNED1,
        (512 - 1), // clock = 16MHz; counter period = 1024; PWM freq = 16MHz / 1024 = 15.625kHz;
        //(BUT PWM center aligned mode needs twice the frequency)
        1); // will fire the TIM1_IT_UPDATE at every PWM period cycle

  TIM1_OC1Init(TIM1_OCMODE_PWM1,
				TIM1_OUTPUTSTATE_DISABLE,
				TIM1_OUTPUTNSTATE_DISABLE,
				255, // initial duty_cycle value
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCIDLESTATE_RESET,
				TIM1_OCNIDLESTATE_SET);

  TIM1_OC2Init(TIM1_OCMODE_PWM1,
				TIM1_OUTPUTSTATE_DISABLE,
				TIM1_OUTPUTNSTATE_DISABLE,
				255, // initial duty_cycle value
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCIDLESTATE_RESET,
				TIM1_OCIDLESTATE_SET);

  TIM1_OC3Init(TIM1_OCMODE_PWM1,
				TIM1_OUTPUTSTATE_DISABLE,
				TIM1_OUTPUTNSTATE_DISABLE,
				255, // initial duty_cycle value
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCIDLESTATE_RESET,
				TIM1_OCNIDLESTATE_SET);

  // OC4 is being used only to fire interrupt at a specific time (middle of DC link current pulses)
  // OC4 is always syncronized with PWM
  TIM1_OC4Init(TIM1_OCMODE_PWM1,
				TIM1_OUTPUTSTATE_DISABLE,
				285, // timming for interrupt firing (hand adjusted)
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCIDLESTATE_RESET);

  // break, dead time and lock configuration
  TIM1_BDTRConfig(TIM1_OSSISTATE_ENABLE,
				TIM1_LOCKLEVEL_OFF,
				// hardware needs a dead time of 1us
				16, // DTG = 0; dead time in 62.5 ns steps; 1us/62.5ns = 16
				TIM1_BREAK_DISABLE,
				TIM1_BREAKPOLARITY_LOW,
				TIM1_AUTOMATICOUTPUT_DISABLE);

  TIM1_ITConfig(TIM1_IT_CC4, ENABLE);
  TIM1_Cmd(ENABLE); // TIM1 counter enable
  TIM1_CtrlPWMOutputs(ENABLE);
#else
  TIM1_TimeBaseInit(0, // TIM1_Prescaler = 0
        TIM1_COUNTERMODE_CENTERALIGNED1,
        (512 - 1), // clock = 16MHz; counter period = 1024; PWM freq = 16MHz / 1024 = 15.625kHz;
        //(BUT PWM center aligned mode needs twice the frequency)
        1); // will fire the TIM1_IT_UPDATE at every PWM period cycle

	//#define DISABLE_PWM_CHANNELS_1_3

  TIM1_OC1Init(TIM1_OCMODE_PWM1,
				#ifdef DISABLE_PWM_CHANNELS_1_3
				TIM1_OUTPUTSTATE_DISABLE,
				TIM1_OUTPUTNSTATE_DISABLE,
				#else
				TIM1_OUTPUTSTATE_ENABLE,
				TIM1_OUTPUTNSTATE_ENABLE,
				#endif
				255, // initial duty_cycle value
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCIDLESTATE_RESET,
				TIM1_OCNIDLESTATE_SET);

  TIM1_OC2Init(TIM1_OCMODE_PWM1,
				TIM1_OUTPUTSTATE_ENABLE,
				TIM1_OUTPUTNSTATE_ENABLE,
				255, // initial duty_cycle value
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCIDLESTATE_RESET,
				TIM1_OCIDLESTATE_SET);

  TIM1_OC3Init(TIM1_OCMODE_PWM1,
				#ifdef DISABLE_PWM_CHANNELS_1_3
				TIM1_OUTPUTSTATE_DISABLE,
				TIM1_OUTPUTNSTATE_DISABLE,
				#else
				TIM1_OUTPUTSTATE_ENABLE,
				TIM1_OUTPUTNSTATE_ENABLE,
				#endif
				255, // initial duty_cycle value
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCIDLESTATE_RESET,
				TIM1_OCNIDLESTATE_SET);

  // OC4 is being used only to fire interrupt at a specific time (middle of DC link current pulses)
  // OC4 is always syncronized with PWM
  TIM1_OC4Init(TIM1_OCMODE_PWM1,
				TIM1_OUTPUTSTATE_DISABLE,
				285, // timming for interrupt firing (hand adjusted)
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCIDLESTATE_RESET);

  // break, dead time and lock configuration
  TIM1_BDTRConfig(TIM1_OSSISTATE_ENABLE,
				TIM1_LOCKLEVEL_OFF,
				// hardware needs a dead time of 1us
				16, // DTG = 0; dead time in 62.5 ns steps; 1us/62.5ns = 16
				TIM1_BREAK_DISABLE,
				TIM1_BREAKPOLARITY_LOW,
				TIM1_AUTOMATICOUTPUT_DISABLE);

	TIM1_ITConfig(TIM1_IT_CC4, ENABLE);
	TIM1_Cmd(ENABLE); // TIM1 counter enable
	TIM1_CtrlPWMOutputs(ENABLE);
#endif
}

#if ENABLE_BACKWARDS_RESISTANCE_OFF	
//=================================================================================================
//
//=================================================================================================
void disable_pwm(void)
{
	TIM1_CtrlPWMOutputs(DISABLE);

	TIM1_OC1Init(TIM1_OCMODE_PWM1,
				TIM1_OUTPUTSTATE_DISABLE,
				TIM1_OUTPUTNSTATE_DISABLE,
				255, // initial duty_cycle value
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCIDLESTATE_RESET,
				TIM1_OCNIDLESTATE_SET);

	TIM1_OC2Init(TIM1_OCMODE_PWM1,
				TIM1_OUTPUTSTATE_DISABLE,
				TIM1_OUTPUTNSTATE_DISABLE,
				255, // initial duty_cycle value
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCIDLESTATE_RESET,
				TIM1_OCIDLESTATE_SET);

	TIM1_OC3Init(TIM1_OCMODE_PWM1,
				TIM1_OUTPUTSTATE_DISABLE,
				TIM1_OUTPUTNSTATE_DISABLE,
				255, // initial duty_cycle value
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCIDLESTATE_RESET,
				TIM1_OCNIDLESTATE_SET);

	#if 0
	GPIO_WriteLow(PMW_PHASE_A_LOW__PORT, PMW_PHASE_A_LOW__PIN);
	GPIO_WriteLow(PMW_PHASE_B_LOW__PORT, PMW_PHASE_B_LOW__PIN);
	GPIO_WriteLow(PMW_PHASE_C_LOW__PORT, PMW_PHASE_C_LOW__PIN);

	GPIO_WriteLow(PMW_PHASE_A_HIGH__PORT, PMW_PHASE_A_HIGH__PIN);
	GPIO_WriteLow(PMW_PHASE_B_HIGH__PORT, PMW_PHASE_B_HIGH__PIN);
	GPIO_WriteLow(PMW_PHASE_C_HIGH__PORT, PMW_PHASE_C_HIGH__PIN);
	#endif
}
#endif

#if ENABLE_BACKWARDS_RESISTANCE_OFF
//=================================================================================================
//
//=================================================================================================
void enable_pwm(void)
{
	TIM1_OC1Init(TIM1_OCMODE_PWM1,
				TIM1_OUTPUTSTATE_ENABLE,
				TIM1_OUTPUTNSTATE_ENABLE,
				255, // initial duty_cycle value
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCIDLESTATE_RESET,
				TIM1_OCNIDLESTATE_SET);

  TIM1_OC2Init(TIM1_OCMODE_PWM1,
				TIM1_OUTPUTSTATE_ENABLE,
				TIM1_OUTPUTNSTATE_ENABLE,
				255, // initial duty_cycle value
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCIDLESTATE_RESET,
				TIM1_OCIDLESTATE_SET);

  TIM1_OC3Init(TIM1_OCMODE_PWM1,
				TIM1_OUTPUTSTATE_ENABLE,
				TIM1_OUTPUTNSTATE_ENABLE,
				255, // initial duty_cycle value
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCIDLESTATE_RESET,
				TIM1_OCNIDLESTATE_SET);

	#if 1
  // OC4 is being used only to fire interrupt at a specific time (middle of DC link current pulses)
  // OC4 is always syncronized with PWM
  TIM1_OC4Init(TIM1_OCMODE_PWM1,
				TIM1_OUTPUTSTATE_DISABLE,
				285, // timming for interrupt firing (hand adjusted)
				TIM1_OCPOLARITY_HIGH,
				TIM1_OCIDLESTATE_RESET);

  // break, dead time and lock configuration
  TIM1_BDTRConfig(TIM1_OSSISTATE_ENABLE,
				TIM1_LOCKLEVEL_OFF,
				// hardware nees a dead time of 1us
				16, // DTG = 0; dead time in 62.5 ns steps; 1us/62.5ns = 16
				TIM1_BREAK_DISABLE,
				TIM1_BREAKPOLARITY_LOW,
				TIM1_AUTOMATICOUTPUT_DISABLE);

  TIM1_ITConfig(TIM1_IT_CC4, ENABLE);
  TIM1_Cmd(ENABLE); // TIM1 counter enable
  TIM1_CtrlPWMOutputs(ENABLE);
	#endif
}
#endif
