/*
 * ebike-fw
 *
 * Copyright (C) Daniel Nilsson, 2020.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include <stdint.h>
#include <stm8s.h>

#include "interrupts.h"

void system_init();

uint32_t system_ms();
void system_delay_ms(uint16_t ms);


void TIM3_OVF_IRQHandler(void) __interrupt(TIM3_OVF_IRQ);

#endif

