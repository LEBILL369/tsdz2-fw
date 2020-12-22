/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */
#include "torque_sensor.h"
#include "pins.h"
#include "config.h"
#include "adc.h"

#include <stm8s.h>

#include <string.h>


#define STATE_NO_PEDALLING		0
#define STATE_PEDALLING			1

static uint16_t torque_sensor_adc_raw = 0;
static uint16_t torque_sensor_adc_steps = 0;
static uint8_t torque_sensor_startup_threshold_ok = 0;
static uint8_t torque_sensor_state = STATE_NO_PEDALLING;

static uint16_t torque_sensor_weight_raw_x10 = 0;
static uint16_t torque_sensor_weight_x10 = 0;
static uint16_t torque_sensor_weight_offset_x10 = 0;
static uint8_t torque_sensor_first_weight = 1;

static uint16_t pedal_torque_x100 = 0;
static uint16_t pedal_power_x10 = 0;

// Hard coded default torque sensor calibration tables for now
// :TODO: add calibration mode to get these values
#define TORQUE_SENSOR_LINEARIZE_NR_POINTS 8
static uint16_t torque_sensor_linearize_table[TORQUE_SENSOR_LINEARIZE_NR_POINTS][2] =
{
	// (adc value), (adc / kg * 100) 
	{ 192, 17 },
	{ 252, 17},
	{ 280, 18 },
	{ 310, 33 },
	{ 344, 88 },
	{ 352, 313 },
	{ 357, 480 },
	{ 360, 500 }
};

static void linearize_torque_sensor_to_kgs(uint16_t adc_steps, uint16_t* out_weight_x10)
{
#define TS_ADC_VALUE				0
#define TS_ADC_INTERVAL_STEPS		1

	uint16_t array_sum[TORQUE_SENSOR_LINEARIZE_NR_POINTS];
	uint8_t i;
	uint16_t adc_absolute;

	uint32_t tmp = 0;

	memset(array_sum, 0, sizeof(array_sum));

	if (adc_steps > 0)
	{
		adc_absolute = adc_steps + adc_get_troque_sensor_offset();

		for (i = 0; i < (TORQUE_SENSOR_LINEARIZE_NR_POINTS - 1); i++)
		{
			// value is under interval max value
			if (adc_absolute < torque_sensor_linearize_table[i + 1][TS_ADC_VALUE])
			{
				// first
				if (i == 0)
				{
					array_sum[i] = adc_steps;
				}
				else
				{
					array_sum[i] = adc_absolute - torque_sensor_linearize_table[i][TS_ADC_VALUE];
				}

				// exit the for loop as this was the last interval
				break;
			}
			// current value is over current interval
			else
			{
				array_sum[i] = torque_sensor_linearize_table[i + 1][TS_ADC_VALUE] - torque_sensor_linearize_table[i][TS_ADC_VALUE];
			}
		}

		// count values under min value of array linear
		if (adc_get_troque_sensor_offset() < torque_sensor_linearize_table[0][TS_ADC_VALUE])
		{
			array_sum[0] += (torque_sensor_linearize_table[0][TS_ADC_VALUE] - adc_get_troque_sensor_offset());
		}


		// count with the values over max value of array linear
		if (adc_absolute > torque_sensor_linearize_table[7][TS_ADC_VALUE])
		{
			array_sum[7] = adc_absolute - torque_sensor_linearize_table[7][TS_ADC_VALUE];
		}

		// sum the total parcels
		for (i = 0; i < TORQUE_SENSOR_LINEARIZE_NR_POINTS - 1; i++)
		{
			tmp += ((uint32_t)array_sum[i] * (uint32_t)torque_sensor_linearize_table[i + 1][TS_ADC_INTERVAL_STEPS]);
		}

		// sum the last parcel
		tmp += ((uint32_t)array_sum[7] * (uint32_t)torque_sensor_linearize_table[7][TS_ADC_INTERVAL_STEPS]);

		*out_weight_x10 = (uint16_t)(tmp / 10);
	}
	// no torque_sensor_adc_steps
	else
	{
		*out_weight_x10 = 0;
	}
}

static void calc_pedal_torque(uint8_t pas_cadence_rpm)
{
	// calculate power on pedals
	// formula for angular velocity in degrees: power  =  force  *  rotations per second  *  2  *  pi
	// formula for angular velocity in degrees: power  =  (force  *  rotations per minute  *  2  *  pi) / 60

	// ui16_pedal_power_x10 = (ui16_pedal_torque_x100 * ui8_pas_cadence_rpm  *  2  *  pi) / (60 * 10)
	// (2 * pi) / (60 * 10) = 0.010466667
	// 1 / 0.010466667 = 96
	// ui16_pedal_power_x10 = (ui16_pedal_torque_x100 * ui8_pas_cadence_rpm) / 96

	// NOTE
	/*
	Users did report that pedal human power is about 2x more.

	@casainho had the idea to evaluate the torque sensor peak signal (measuring peak signal every pedal rotation) as being a sinewave and so the average would be:

	> [Average value = 0.637 × maximum or peak value, Vpk](https://www.electronics-tutorials.ws/accircuits/average-voltage.html)

	For a quick hack, we can just reduce actual value to 0.637.

	*/

	// linearize and calculate weight on pedals
	linearize_torque_sensor_to_kgs(torque_sensor_adc_raw, &torque_sensor_weight_raw_x10);

	// let´s save the initial weight offset
	if (torque_sensor_first_weight && torque_sensor_adc_raw)
	{
		torque_sensor_first_weight = 0;
		torque_sensor_weight_offset_x10 = torque_sensor_weight_raw_x10;
	}

	// linearize and calculate weight on pedals
	linearize_torque_sensor_to_kgs(torque_sensor_adc_steps, &torque_sensor_weight_x10);

	// remove the weight offset
	if (!torque_sensor_first_weight)
	{
		if (torque_sensor_weight_x10 > torque_sensor_weight_offset_x10)
		{
			torque_sensor_weight_x10 -= torque_sensor_weight_offset_x10;
		}
		else
		{
			torque_sensor_weight_x10 = 0;
		}

		if (torque_sensor_weight_raw_x10 > torque_sensor_weight_offset_x10)
		{
			torque_sensor_weight_raw_x10 -= torque_sensor_weight_offset_x10;
		}

		else
		{
			torque_sensor_weight_raw_x10 = 0;
		}
	}

	pedal_torque_x100 = torque_sensor_weight_x10 * (uint16_t)TORQUE_SENSOR_WEIGHT_TO_FORCE_X10;
	pedal_power_x10 = (uint16_t)(((uint32_t)pedal_torque_x100 * (uint32_t)pas_cadence_rpm) / (uint32_t)96);
}


void torque_sensor_init(void)
{
	GPIO_Init(TORQUE_SENSOR_EXCITATION__PORT, TORQUE_SENSOR_EXCITATION__PIN, GPIO_MODE_OUT_OD_HIZ_FAST);

	// Actual reading of torque sensors is done in periodic timer interrupt in motor.c.
}

void torque_sensor_read(uint8_t pas_cadence_rpm)
{
	uint16_t raw_value;

	raw_value = ADC_10_BIT_TORQUE_SENSOR;

	// should be no problem to use cadence value calculated on the previous cycle
	if ((raw_value >= TORQUE_SENSOR_ADC_THRESHOLD) || (pas_cadence_rpm > 30))
	{
		torque_sensor_startup_threshold_ok = 1;
	}
	else
	{
		torque_sensor_startup_threshold_ok = 0;
	}

	torque_sensor_adc_raw = raw_value;

	// remove the offset
	// make sure readed value is higher than the offset
	if (torque_sensor_adc_raw >= adc_get_troque_sensor_offset())
	{
		torque_sensor_adc_raw = torque_sensor_adc_raw - adc_get_troque_sensor_offset();
	}
	// offset is higher, something is wrong so just keep ui8_torque_sensor_raw at 0 value
	else
	{
		torque_sensor_adc_raw = 0;
	}

	// next state machine is used to filter out the torque sensor signal
	// when user is resting on the pedals
	switch (torque_sensor_state)
	{
		// ebike is stopped
	case STATE_NO_PEDALLING:
		if (torque_sensor_startup_threshold_ok && pas_cadence_rpm > 10)
		{
			torque_sensor_state = STATE_PEDALLING;
		}
		break;

		// wait on this state and reset when ebike stops
	case STATE_PEDALLING:
		if (pas_cadence_rpm == 0 && torque_sensor_adc_raw == 0)
		{
			torque_sensor_state = STATE_NO_PEDALLING;
		}
		break;

	default:
		break;
	}

	// bike is moving but user doesn't pedal, disable torque sensor signal because user can be resting the feet on the pedals
	if (torque_sensor_state == STATE_PEDALLING && pas_cadence_rpm == 0)
	{
		torque_sensor_adc_steps = 0;
	}
	else
	{
		torque_sensor_adc_steps = torque_sensor_adc_raw;
	}

	calc_pedal_torque(pas_cadence_rpm);
}

uint8_t torque_sensor_get_pedal_power_watts_x10()
{
	return pedal_power_x10;
}

