/*
 * ebike-fw
 *
 * Copyright (C) Daniel Nilsson, 2020.
 *
 * Released under the GPL License, Version 3
 */

#include "system.h"

#include <stm8s.h>
#include <stm8s_tim3.h>

#define CPU_FREQ			16000000		// 16MHz

volatile uint32_t	_ms = 0;

void system_init()
{
	// Setup system clock
	CLK_DeInit();

	CLK_HSECmd(DISABLE);
	CLK_LSICmd(DISABLE);
	CLK_HSICmd(ENABLE);
	while (CLK_GetFlagStatus(CLK_FLAG_HSIRDY) == FALSE);

	CLK_ClockSwitchCmd(ENABLE);
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
	CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);

	CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSI,
		DISABLE, CLK_CURRENTCLOCKSTATE_ENABLE);


	// Setup timer3 as millisecond counter
	CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER3, ENABLE);

	TIM3_DeInit();
	TIM3_TimeBaseInit(TIM3_PRESCALER_1, 15999);
	TIM3_SetCounter(0);
	TIM3_ITConfig(TIM3_IT_UPDATE, ENABLE);
	TIM3_ClearITPendingBit(TIM3_IT_UPDATE);
	TIM3_Cmd(ENABLE);

	enableInterrupts();
}


uint32_t system_ms()
{
	uint32_t val;
	TIM3_ITConfig(TIM3_IT_UPDATE, DISABLE);
	val = _ms;
	TIM3_ITConfig(TIM3_IT_UPDATE, ENABLE);
	return val;
}

void system_delay_ms(uint16_t ms)
{
	uint32_t end = system_ms() + ms;
	while (system_ms() != end);
}


void TIM3_OVF_IRQHandler(void) __interrupt(TIM3_OVF_IRQ)
{
	_ms++;
	TIM3_ClearITPendingBit(TIM3_IT_UPDATE);
}

