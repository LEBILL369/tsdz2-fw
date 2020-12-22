/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include "adc.h"
#include "system.h"

#include <stm8s.h>
#include <stm8s_adc1.h>

static uint16_t adc_battery_current_offset = 0;
static uint16_t adc_troque_sensor_offset = 0;


static void adc_trigger(void)
{
	// trigger ADC conversion of all channels (scan conversion, buffered)
	ADC1->CSR &= 0x07; // clear EOC flag first (selected also channel 7)
	ADC1->CR1 |= ADC1_CR1_ADON; // Start ADC1 conversion
}

static void adc_calibrate()
{
	uint8_t i;

	// 6s delay to wait for voltages stabilize (maybe beause capacitors on the circuit)
	// this was tested on 27.12.2019 by Casainho and lower values like 5s would not work.
	system_delay_ms(6000);

	// read and discard few samples of ADC, to make sure the next samples are ok
	for (i = 0; i < 64; i++)
	{
		system_delay_ms(2);
		adc_trigger();
		while (!ADC1_GetFlagStatus(ADC1_FLAG_EOC)); // wait for end of conversion
	}

	// read and average a few values of ADC battery current
	adc_battery_current_offset = 0;
	for (i = 0; i < 16; ++i)
	{
		system_delay_ms(2);
		adc_trigger();
		while (!ADC1_GetFlagStatus(ADC1_FLAG_EOC)); // wait for end of conversion
		adc_battery_current_offset += ADC_10_BIT_BATTERY_CURRENT;
	}
	adc_battery_current_offset >>= 4;

	// read and average a few values of ADC torque sensor
	adc_troque_sensor_offset = 0;
	for (i = 0; i < 16; ++i)
	{
		system_delay_ms(2);
		adc_trigger();
		while (!ADC1_GetFlagStatus(ADC1_FLAG_EOC)); // wait for end of conversion
		adc_troque_sensor_offset += ADC_10_BIT_TORQUE_SENSOR;
	}
	adc_troque_sensor_offset >>= 4;
}


void adc_init(void)
{
	//init GPIO for the used ADC pins
	GPIO_Init(GPIOB, (GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_3), GPIO_MODE_IN_FL_NO_IT);

	//init ADC1 peripheral
	ADC1_Init(ADC1_CONVERSIONMODE_SINGLE,
		ADC1_CHANNEL_7,
		ADC1_PRESSEL_FCPU_D2,
		ADC1_EXTTRIG_TIM,
		DISABLE,
		ADC1_ALIGN_LEFT,
		(ADC1_SCHMITTTRIG_CHANNEL3 | ADC1_SCHMITTTRIG_CHANNEL5 | ADC1_SCHMITTTRIG_CHANNEL6 | ADC1_SCHMITTTRIG_CHANNEL7),
		DISABLE);

	ADC1_ScanModeCmd(ENABLE);
	ADC1_Cmd(ENABLE);

	adc_calibrate();
}

uint16_t adc_get_battery_current_offset(void)
{
	return adc_battery_current_offset;
}

uint16_t adc_get_troque_sensor_offset(void)
{
	return adc_troque_sensor_offset;
}
