/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */
#include "utils.h"
#include <stm8s.h>

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
	// if input is smaller/bigger than expected return the min/max out ranges value
	if (x < in_min)
	{
		return out_min;
	}	
	else if (x > in_max)
	{
		return out_max;
	}
	
	// map the input to the output range.
	// round up if mapping bigger ranges to smaller ranges
	else if ((in_max - in_min) > (out_max - out_min))
	{
		return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
	}	
	// round down if mapping smaller ranges to bigger ranges
	else
	{
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}
}

int32_t map_inverse(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
	// if input is smaller/bigger than expected return the min/max out ranges value
	if (x < in_min)
	{
		return out_min;
	}
	
	else if (x > in_max)
	{
		return out_max;
	}

	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint8_t ui8_min(uint8_t value_a, uint8_t value_b)
{
	if (value_a < value_b)
	{
		return value_a;
	}
	else
	{
		return value_b;
	}
}

uint16_t ui16_min(uint16_t value_a, uint16_t value_b)
{
	if (value_a < value_b)
	{
		return value_a;
	}
	else
	{
		return value_b;
	}
}

uint8_t ui8_max(uint8_t value_a, uint8_t value_b)
{
	if (value_a > value_b)
	{
		return value_a;
	}
	else
	{
		return value_b;
	}
}

void ui8_limit_max(uint8_t *ui8_p_value, uint8_t ui8_max_value)
{
	if (*ui8_p_value > ui8_max_value)
	{ 
		*ui8_p_value = ui8_max_value;
	}
}

void ui16_limit_max(uint16_t *ui16_p_value, uint16_t ui16_max_value)
{
	if (*ui16_p_value > ui16_max_value)
	{
		*ui16_p_value = ui16_max_value;
	}
}

