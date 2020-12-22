/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */
#include "brake.h"
#include "stm8s.h"

void brake_init(void)
{
	GPIO_Init(BRAKE__PORT, BRAKE__PIN, GPIO_MODE_IN_PU_NO_IT);
}

uint8_t brake_is_active(void)
{
	return (GPIO_ReadInputPin(BRAKE__PORT, BRAKE__PIN) == 0);
}
