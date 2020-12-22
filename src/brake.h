/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _BRAKE_H
#define _BRAKE_H

#include "pins.h"
#include <stdint.h>

void brake_init(void);
uint8_t brake_is_active(void);

#endif /* _BRAKE_H */
