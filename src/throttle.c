#include "throttle.h"
#include "utils.h"
#include "adc.h"
#include "config.h"

void throttle_init()
{
	// Nothing to do, done in adc_init().
}

uint8_t throttle_read()
{
	// map value from 0 up to 255
	return (uint8_t)(map(
		ADC_THROTTLE,
		(uint8_t)ADC_THROTTLE_MIN_VALUE,
		(uint8_t)ADC_THROTTLE_MAX_VALUE,
		(uint8_t)0,
		(uint8_t)255)
	);
}

