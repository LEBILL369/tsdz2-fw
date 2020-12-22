/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _PAS_H_
#define _PAS_H_

void pas_init(void);

uint32_t pas_get_cadence_rpm();

#endif /* _PAS_H_ */
