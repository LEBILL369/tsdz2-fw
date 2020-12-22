#include <stdint.h>

#include "system.h"
#include "pins.h"
#include "brake.h"
#include "adc.h"
#include "motor.h"
#include "torque_sensor.h"
#include "throttle.h"
#include "pas.h"
#include "app.h"


#define MOTOR_PROC_INTERVAL_MS		4
#define APP_PROC_INTERVAL_MS		10


int main(void)
{
	system_init();
	adc_init();
	brake_init();
	pas_init();
	torque_sensor_init();
	throttle_init();
	motor_init();
	app_init();

	uint32_t next_motor_proc_ms = 0;
	uint32_t next_app_proc_ms = 0;

	while (1)
	{
		uint32_t now = system_ms();
		if (now >= next_motor_proc_ms)
		{
			next_motor_proc_ms = now + MOTOR_PROC_INTERVAL_MS;
			motor_process();
		}
		else if (now >= next_app_proc_ms)
		{
			next_app_proc_ms = now + APP_PROC_INTERVAL_MS;
			app_process();
		}
	}
}
