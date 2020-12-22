#include "app.h"
#include "motor.h"
#include "utils.h"
#include "pins.h"
#include "adc.h"
#include "config.h"
#include "brake.h"
#include "throttle.h"
#include "torque_sensor.h"

#include <stm8s.h>
#include <string.h>

#define MODE_PAS				0
#define MODE_CRUISE				1

static uint8_t active_mode = MODE_PAS;


static uint8_t motor_enabled = 0;
static uint8_t throttle_enabled = 0;

static uint8_t throttle = 0;
static uint8_t pas_cadence_rpm = 0;
static uint32_t pedal_power_x10 = 0;


static uint8_t increase_pressed = 0;
static uint8_t decrease_pressed = 0;

static uint8_t cruise_paused = 0;



#define PAS_LEVELS_COUNT 6
uint16_t pas_level_factors_x1000[PAS_LEVELS_COUNT] =
{
	PAS_LEVEL_0_FACTOR_X1000,
	PAS_LEVEL_1_FACTOR_X1000,
	PAS_LEVEL_2_FACTOR_X1000,
	PAS_LEVEL_3_FACTOR_X1000,
	PAS_LEVEL_4_FACTOR_X1000,
	PAS_LEVEL_5_FACTOR_X1000,
};

uint8_t pas_level = 0;

#define CRUISE_LEVELS_COUNT 8
uint16_t cruise_level_amps_x10[CRUISE_LEVELS_COUNT] =
{
	CRUISE_LEVEL_0_AMPS_X10,
	CRUISE_LEVEL_1_AMPS_X10,
	CRUISE_LEVEL_2_AMPS_X10,
	CRUISE_LEVEL_3_AMPS_X10,
	CRUISE_LEVEL_4_AMPS_X10,
	CRUISE_LEVEL_5_AMPS_X10,
	CRUISE_LEVEL_6_AMPS_X10,
	CRUISE_LEVEL_7_AMPS_X10,
};

uint8_t cruise_level = 0;



static void read_sensors(void)
{
	throttle = throttle_read();
	pas_cadence_rpm = motor_get_pas_cadence_rpm();

	torque_sensor_read(pas_cadence_rpm);
	pedal_power_x10 = torque_sensor_get_pedal_power_watts_x10();
}


static void switch_mode(uint8_t mode)
{
	if (mode == MODE_PAS)
	{
		// throttle enabled after actively switching to pas for the first time
		// (i.e. not enabled at startup by default)
		throttle_enabled = 1;
		pas_level = 1;
		cruise_level = 0;
		active_mode = MODE_PAS;
	}
	else if (mode == MODE_CRUISE)
	{
		throttle_enabled = 0;
		cruise_level = 0;
		cruise_paused = 0;
		pas_level = 0;
		active_mode = MODE_CRUISE;
	}
}

static void increase_pas_level()
{
	if (pas_level < PAS_LEVELS_COUNT - 1)
	{
		++pas_level;
	}
}

static void decrease_pas_level()
{
	if (pas_level > 0)
	{
		--pas_level;
	}
}

static void increase_cruise_level()
{
	if (cruise_level < CRUISE_LEVELS_COUNT - 1)
	{
		++cruise_level;
	}
}

static void decrease_cruise_level()
{
	if (cruise_level > 0)
	{
		--cruise_level;
	}
}

static void process_input()
{
	if (!GPIO_ReadInputPin(INCREASE_ASSIST__PORT, INCREASE_ASSIST__PIN))
	{
		if (cruise_paused)
		{
			cruise_paused = 0;
			increase_pressed = 1;
		}
		else if (!increase_pressed)
		{
			increase_pressed = 1;

			switch (active_mode)
			{
			case MODE_PAS:
				increase_pas_level();
				break;
			case MODE_CRUISE:
				if (cruise_paused)
				{
					// unpause and keep level
					cruise_paused = 0;
				}
				else
				{
					increase_cruise_level();
				}
			
				break;
			}
		}
	}
	else
	{
		increase_pressed = 0;
	}

	if (!GPIO_ReadInputPin(DECREASE_ASSIST__PORT, DECREASE_ASSIST__PIN))
	{
		if (cruise_paused)
		{
			decrease_pressed = 1;
		}
		else if (!decrease_pressed)
		{
			decrease_pressed = 1;

			switch (active_mode)
			{
			case MODE_PAS:
				decrease_pas_level();
				break;
			case MODE_CRUISE:
				if (cruise_paused)
				{
					// unpause and reset level
					cruise_paused = 0;
					cruise_level = 0;
				}
				else
				{
					decrease_cruise_level();
				}			
				break;
			}
		}
	}
	else
	{
		decrease_pressed = 0;
	}

	// Detect long press (~2s) on brake to allow switch to other mode
	//if (brake_is_active())
	//{
		// :TODO:
	//}

	// pause cruise if brake or throttle is touched
	if (active_mode == MODE_CRUISE && (brake_is_active() || throttle > 0))
	{
		cruise_paused = 1;
	}
}


static void apply_pas(uint32_t* target_current_amps_x10)
{
	uint32_t factor_x1000 = (uint32_t)pas_level_factors_x1000[pas_level];

	if (pas_cadence_rpm)
	{
		*target_current_amps_x10 = ((uint32_t)pedal_power_x10 * factor_x1000) / 1000;;
	}
	else
	{
		*target_current_amps_x10 = 0;
	}

	// if user is rotating the pedals, force use min current of 0.5 amps even if not troque
	if (pas_cadence_rpm && *target_current_amps_x10 < 5)
	{
		*target_current_amps_x10 = 5;
	}
}

static void apply_cruise(uint32_t* target_current_amps_x10)
{
	uint16_t amps_x10 = cruise_level_amps_x10[cruise_level];

	if (!cruise_paused && amps_x10 > *target_current_amps_x10)
	{
		*target_current_amps_x10 = amps_x10;
	}
}

static void apply_throttle(uint32_t* target_current_amps_x10)
{
	uint16_t tmp;
	if (throttle_enabled)
	{
		tmp = (uint16_t)(map((uint32_t)throttle,
			(uint32_t)0,
			(uint32_t)255,
			(uint32_t)0,
			(uint32_t)MAX_MOTOR_CURRENT_X10));

		if (tmp > *target_current_amps_x10)
		{
			*target_current_amps_x10 = tmp;
		}
	}
}

static void adjust_power()
{
	uint32_t target_current_amps_x10 = 0;

	// 15V is the lower hard limit for controller
	if (motor_get_supply_voltage() > 15)
	{
		apply_pas(&target_current_amps_x10);
		apply_cruise(&target_current_amps_x10);
		apply_throttle(&target_current_amps_x10);
	}

	if (brake_is_active())
	{
		target_current_amps_x10 = 0;
	}

	if (!motor_enabled && motor_get_motor_speed_erps() == 0 && target_current_amps_x10 > 0)
	{
		motor_enabled = 1;
		motor_set_pwm_duty_cycle_target(255);
		motor_enable();
	}

	if (motor_enabled && motor_get_motor_speed_erps() == 0 && target_current_amps_x10 == 0)
	{
		motor_enabled = 0;
		motor_set_pwm_duty_cycle_target(0);
		motor_disable();
	}

	if (motor_enabled)
	{
		motor_set_target_max_motor_current(target_current_amps_x10);
	}
	else
	{
		motor_set_target_max_motor_current(0);
	}
}


void app_init()
{
	motor_set_target_max_battery_current(MAX_BATTERY_CURRENT_X10);
	motor_enable();

	GPIO_Init(INCREASE_ASSIST__PORT, INCREASE_ASSIST__PIN, GPIO_MODE_IN_PU_NO_IT);
	GPIO_Init(DECREASE_ASSIST__PORT, DECREASE_ASSIST__PIN, GPIO_MODE_IN_PU_NO_IT);
}

void app_process()
{
	read_sensors();
	process_input();
	adjust_power();
}
