/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include "pas.h"
#include "pins.h"
#include "motor.h"

#include <stdint.h>
#include <stm8s.h>

void pas_init (void)
{
	GPIO_Init(PAS1__PORT, PAS1__PIN, GPIO_MODE_IN_PU_NO_IT);
	GPIO_Init(PAS2__PORT, PAS2__PIN, GPIO_MODE_IN_PU_NO_IT);

	// Actual reading of pas sensors is done in periodic timer interrupt in motor.c
	// :TODO: Use external intterupt and move here instead for better code separation?
}

uint32_t pas_get_cadence_rpm()
{
	return motor_get_pas_cadence_rpm();
}

