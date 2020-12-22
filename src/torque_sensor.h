/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _TORQUE_SENSOR_H_
#define _TORQUE_SENSOR_H_

#include <stdint.h>

void torque_sensor_init(void);

void torque_sensor_read(uint8_t pas_cadence_rpm);

uint8_t torque_sensor_get_pedal_power_watts_x10();


#endif /* _TORQUE_SENSOR_H_ */
