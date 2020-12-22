/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _ADC_H
#define _ADC_H

#include <stdint.h>

#define ADC_BATTERY_VOLTAGE 				(*(uint8_t*)(0x53EC)) // AIN6
#define ADC_BATTERY_CURRENT					(*(uint8_t*)(0x53EA)) // AIN5
#define ADC_THROTTLE						(*(uint8_t*)(0x53EE)) // AIN7
#define ADC_TORQUE_SENSOR					(*(uint8_t*)(0x53E8)) // AIN4

#define ADC_10_BIT_BATTERY_VOLTAGE			(((*(uint8_t*)(0x53EC)) << 2) | (*(uint8_t*)(0x53ED)))
#define ADC_10_BIT_BATTERY_CURRENT			(((*(uint8_t*)(0x53EA)) << 2) | (*(uint8_t*)(0x53EB)))
#define ADC_10_BIT_THROTTLE					(((*(uint8_t*)(0x53EE)) << 2) | (*(uint8_t*)(0x53EF)))
#define ADC_10_BIT_TORQUE_SENSOR			(((*(uint8_t*)(0x53E8)) << 2) | (*(uint8_t*)(0x53E9)))

void adc_init(void);

uint16_t adc_get_battery_current_offset(void);
uint16_t adc_get_troque_sensor_offset(void);

#endif /* _ADC_H */
