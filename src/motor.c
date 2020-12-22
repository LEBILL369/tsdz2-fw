/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include "motor.h"
#include "pins.h"
#include "brake.h"
#include "config.h"
#include "adc.h"
#include "utils.h"
#include "watchdog.h"

#include <stm8s_adc1.h>
#include <stm8s_gpio.h>
#include <stm8s_tim1.h>
#include <stm8s_wwdg.h>

#include <math.h>
#include <stdint.h>


#define SVM_TABLE_LEN							256
#define SIN_TABLE_LEN							60

 // motor states
#define BLOCK_COMMUTATION						1
#define SINEWAVE_INTERPOLATION_60_DEGREES		2

uint8_t ui8_svm_table[SVM_TABLE_LEN] =
{
	239 ,
	241 ,
	242 ,
	243 ,
	245 ,
	246 ,
	247 ,
	248 ,
	249 ,
	250 ,
	251 ,
	251 ,
	252 ,
	253 ,
	253 ,
	254 ,
	254 ,
	254 ,
	255 ,
	255 ,
	255 ,
	255 ,
	255 ,
	255 ,
	254 ,
	254 ,
	254 ,
	253 ,
	253 ,
	252 ,
	251 ,
	250 ,
	250 ,
	249 ,
	248 ,
	247 ,
	245 ,
	244 ,
	243 ,
	242 ,
	240 ,
	239 ,
	236 ,
	231 ,
	227 ,
	222 ,
	217 ,
	212 ,
	207 ,
	202 ,
	197 ,
	191 ,
	186 ,
	181 ,
	176 ,
	170 ,
	165 ,
	160 ,
	154 ,
	149 ,
	144 ,
	138 ,
	133 ,
	127 ,
	122 ,
	116 ,
	111 ,
	106 ,
	100 ,
	95  ,
	89  ,
	84  ,
	79  ,
	74  ,
	68  ,
	63  ,
	58  ,
	53  ,
	48  ,
	43  ,
	38  ,
	33  ,
	28  ,
	23  ,
	18  ,
	16  ,
	14  ,
	13  ,
	12  ,
	10  ,
	9 ,
	8 ,
	7 ,
	6 ,
	5 ,
	4 ,
	3 ,
	3 ,
	2 ,
	1 ,
	1 ,
	1 ,
	0 ,
	0 ,
	0 ,
	0 ,
	0 ,
	0 ,
	0 ,
	0 ,
	0 ,
	1 ,
	1 ,
	2 ,
	2 ,
	3 ,
	4 ,
	5 ,
	6 ,
	6 ,
	8 ,
	9 ,
	10  ,
	11  ,
	12  ,
	14  ,
	15  ,
	17  ,
	15  ,
	14  ,
	12  ,
	11  ,
	10  ,
	9 ,
	8 ,
	6 ,
	6 ,
	5 ,
	4 ,
	3 ,
	2 ,
	2 ,
	1 ,
	1 ,
	0 ,
	0 ,
	0 ,
	0 ,
	0 ,
	0 ,
	0 ,
	0 ,
	0 ,
	1 ,
	1 ,
	1 ,
	2 ,
	3 ,
	3 ,
	4 ,
	5 ,
	6 ,
	7 ,
	8 ,
	9 ,
	10  ,
	12  ,
	13  ,
	14  ,
	16  ,
	18  ,
	23  ,
	28  ,
	33  ,
	38  ,
	43  ,
	48  ,
	53  ,
	58  ,
	63  ,
	68  ,
	74  ,
	79  ,
	84  ,
	89  ,
	95  ,
	100 ,
	106 ,
	111 ,
	116 ,
	122 ,
	127 ,
	133 ,
	138 ,
	144 ,
	149 ,
	154 ,
	160 ,
	165 ,
	170 ,
	176 ,
	181 ,
	186 ,
	191 ,
	197 ,
	202 ,
	207 ,
	212 ,
	217 ,
	222 ,
	227 ,
	231 ,
	236 ,
	239 ,
	240 ,
	242 ,
	243 ,
	244 ,
	245 ,
	247 ,
	248 ,
	249 ,
	250 ,
	250 ,
	251 ,
	252 ,
	253 ,
	253 ,
	254 ,
	254 ,
	254 ,
	255 ,
	255 ,
	255 ,
	255 ,
	255 ,
	255 ,
	254 ,
	254 ,
	254 ,
	253 ,
	253 ,
	252 ,
	251 ,
	251 ,
	250 ,
	249 ,
	248 ,
	247 ,
	246 ,
	245 ,
	243 ,
	242 ,
	241 ,
	239 ,
	238 ,
};

uint8_t ui8_sin_table[SIN_TABLE_LEN] =
{
	0 ,
	3 ,
	6 ,
	9 ,
	12  ,
	16  ,
	19  ,
	22  ,
	25  ,
	28  ,
	31  ,
	34  ,
	37  ,
	40  ,
	43  ,
	46  ,
	49  ,
	52  ,
	54  ,
	57  ,
	60  ,
	63  ,
	66  ,
	68  ,
	71  ,
	73  ,
	76  ,
	78  ,
	81  ,
	83  ,
	86  ,
	88  ,
	90  ,
	92  ,
	95  ,
	97  ,
	99  ,
	101 ,
	102 ,
	104 ,
	106 ,
	108 ,
	109 ,
	111 ,
	113 ,
	114 ,
	115 ,
	117 ,
	118 ,
	119 ,
	120 ,
	121 ,
	122 ,
	123 ,
	124 ,
	125 ,
	125 ,
	126 ,
	126 ,
	127
};


// Motor control state
static volatile uint16_t motor_adc_battery_current = 0;
static volatile uint16_t motor_adc_current = 0;

static uint16_t motor_pwm_cycles_counter = 1;
static uint16_t motor_pwm_cycles_counter_6 = 1;
static uint16_t motor_pwm_cycles_counter_total = 0xffff;

static uint16_t motor_max_speed_erps = (uint16_t)MOTOR_OVER_SPEED_ERPS;
static volatile uint16_t motor_speed_erps = 0;

static uint8_t motor_svm_table_index = 0;
static uint8_t motor_rotor_absolute_angle = 0;
static uint8_t motor_rotor_angle = 0;

static volatile uint8_t motor_foc_angle = 0;
static uint8_t motor_interpolation_angle = 0;
static uint16_t motor_foc_angle_accumulated = 0;

static uint8_t motor_commutation_type = BLOCK_COMMUTATION;

volatile uint8_t motor_hall_sensors_state = 0;
static uint8_t motor_hall_sensors_state_last = 0;

static uint8_t motor_half_erps_flag = 0;

// PWM duty cycle controller
static volatile uint8_t motor_pwm_duty_cycle = 0;
static volatile uint8_t motor_pwm_duty_cycle_target = 0;
static uint16_t motor_pwm_duty_cycle_ramp_up_inverse_step = 0;
static uint16_t motor_pwm_duty_cycle_ramp_down_inverse_step = 0;
static uint16_t motor_pwm_duty_cycle_ramp_up_counter = 0;
static uint16_t motor_pwm_duty_cycle_ramp_down_counter = 0;

static volatile uint8_t motor_field_weakening_angle = 0;
static volatile uint8_t motor_field_weakening_enable = 0;
static volatile uint8_t motor_field_weakening_enable_state = 0;
static uint16_t motor_field_weakening_ramp_up_counter = 0;
static uint16_t motor_field_weakening_ramp_down_counter = 0;

static uint8_t motor_phase_a_voltage = 0;
static uint8_t motor_phase_b_voltage = 0;
static uint8_t motor_phase_c_voltage = 0;

// Current controller
static uint16_t motor_adc_current_ramp_up_counter = 0;
static uint16_t motor_controller_adc_max_current = 0;
static uint8_t motor_current_controller_counter = 0;
static uint16_t motor_speed_controller_counter = 0;
static uint16_t motor_current_ramp_up_inverse_step = 0;


static volatile uint16_t motor_adc_target_battery_max_current = 0;
static volatile uint16_t motor_adc_target_battery_max_current_fw = 0;
static volatile uint16_t motor_adc_target_motor_max_current = 0;
static volatile uint16_t motor_adc_target_motor_max_current_fw = 0;

static volatile uint8_t motor_brake_is_active = 0;


static uint16_t motor_adc_battery_voltage_accumulated = 0;
static uint16_t motor_adc_battery_voltage_filtered = 0;

static uint16_t motor_adc_battery_current_accumulated = 0;
static volatile uint16_t motor_adc_battery_current_filtered = 0;

static uint16_t motor_adc_current_accumulated = 0;
static volatile uint16_t motor_adc_current_filtered = 0;


// PAS
static uint8_t motor_pas_state;
static uint8_t motor_pas_state_prev;
static uint8_t motor_pas_after_first_pulse = 0;
static uint16_t motor_pas_counter = (uint16_t)PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS;
static volatile uint8_t motor_pas_tick_counter = 0;


static volatile uint16_t motor_pas_pwm_cycle_ticks = 0;
static volatile uint8_t motor_pas_pedal_right = 0;
uint8_t motor_pas_pedaling_direction = 0;

static uint8_t motor_pas_min_cadence_flag = 0;
static uint16_t motor_pas_min_cadence_pwm_cycles_ticks = 0;

uint8_t ui8_first_time_run_flag = 1;
volatile uint16_t ui16_main_loop_wdt_cnt_1 = 0;



static void motor_hall_sensor_init(void)
{
	GPIO_Init(HALL_SENSOR_A__PORT, HALL_SENSOR_A__PIN, GPIO_MODE_IN_FL_NO_IT);
	GPIO_Init(HALL_SENSOR_B__PORT, HALL_SENSOR_B__PIN, GPIO_MODE_IN_FL_NO_IT);
	GPIO_Init(HALL_SENSOR_C__PORT, HALL_SENSOR_C__PIN, GPIO_MODE_IN_FL_NO_IT);

	// Timer2 clock = 16MHz; target: 20us period --> 50khz
	// counter period = (1 / (16000000 / prescaler)) * (159 + 1) = 20us
	TIM2_TimeBaseInit(TIM2_PRESCALER_2, 159);

	// pulse of 2us
	TIM2_OC2Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE, 16, TIM2_OCPOLARITY_HIGH);
	TIM2_OC2PreloadConfig(ENABLE);

	TIM2_ARRPreloadConfig(ENABLE);

	TIM2_Cmd(ENABLE);
}

static void motor_pwm_init(void)
{
	// verify if PWM N channels are active on option bytes, if not, enable
	FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);
	if (FLASH_ReadOptionByte(0x4803) != 0x20)
	{
		FLASH_Unlock(FLASH_MEMTYPE_DATA);
		FLASH_EraseOptionByte(0x4803);
		FLASH_ProgramOptionByte(0x4803, 0x20);
		FLASH_Lock(FLASH_MEMTYPE_DATA);
	}

	TIM1_TimeBaseInit(0, // TIM1_Prescaler = 0
		TIM1_COUNTERMODE_CENTERALIGNED1,
		(512 - 1),	// clock = 16MHz, counter period = 1024, PWM freq = 16MHz / 1024 = 15.625MHz
		//(BUT PWM center aligned mode needs twice the frequency)
		1);

	TIM1_OC1Init(TIM1_OCMODE_PWM1,
		TIM1_OUTPUTSTATE_ENABLE,
		TIM1_OUTPUTNSTATE_ENABLE,
		255, // initial duty_cycle value
		TIM1_OCPOLARITY_HIGH,
		TIM1_OCPOLARITY_HIGH,
		TIM1_OCIDLESTATE_RESET,
		TIM1_OCNIDLESTATE_SET);

	TIM1_OC2Init(TIM1_OCMODE_PWM1,
		TIM1_OUTPUTSTATE_ENABLE,
		TIM1_OUTPUTNSTATE_ENABLE,
		255, // initial duty_cycle value
		TIM1_OCPOLARITY_HIGH,
		TIM1_OCPOLARITY_HIGH,
		TIM1_OCIDLESTATE_RESET,
		TIM1_OCIDLESTATE_SET);

	TIM1_OC3Init(TIM1_OCMODE_PWM1,
		TIM1_OUTPUTSTATE_ENABLE,
		TIM1_OUTPUTNSTATE_ENABLE,
		255, // initial duty_cycle value
		TIM1_OCPOLARITY_HIGH,
		TIM1_OCPOLARITY_HIGH,
		TIM1_OCIDLESTATE_RESET,
		TIM1_OCNIDLESTATE_SET);

	// OC4 is being used only to fire interrupt at a specific time (middle of DC link current pulses)
	// OC4 is always syncronized with PWM
	TIM1_OC4Init(TIM1_OCMODE_PWM1,
		TIM1_OUTPUTSTATE_DISABLE,
		285, // timming for interrupt firing (hand adjusted)
		TIM1_OCPOLARITY_HIGH,
		TIM1_OCIDLESTATE_RESET);

	// break, dead time and lock configuration
	TIM1_BDTRConfig(TIM1_OSSISTATE_ENABLE,
		TIM1_LOCKLEVEL_OFF,
		// hardware needs a dead time of 1us
		16, // DTG = 0; dead time in 62.5 ns steps; 1us/62.5ns = 16
		TIM1_BREAK_DISABLE,
		TIM1_BREAKPOLARITY_LOW,
		TIM1_AUTOMATICOUTPUT_DISABLE);

	TIM1_ITConfig(TIM1_IT_CC4, ENABLE);
	TIM1_Cmd(ENABLE);
	TIM1_CtrlPWMOutputs(ENABLE);
}

static void motor_read_battery_voltage(void)
{
	// low pass filter the voltage readed value, to avoid possible fast spikes/noise
	motor_adc_battery_voltage_accumulated -= motor_adc_battery_voltage_accumulated >> READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT;
	motor_adc_battery_voltage_accumulated += ADC_10_BIT_BATTERY_VOLTAGE;
	motor_adc_battery_voltage_filtered = motor_adc_battery_voltage_accumulated >> READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT;
}

static void motor_read_battery_current(void)
{
	// low pass filter the positive battery readed value (no regen current), to avoid possible fast spikes/noise
	motor_adc_battery_current_accumulated -= motor_adc_battery_current_accumulated >> READ_BATTERY_CURRENT_FILTER_COEFFICIENT;
	motor_adc_battery_current_accumulated += motor_adc_battery_current;
	motor_adc_battery_current_filtered = motor_adc_battery_current_accumulated >> READ_BATTERY_CURRENT_FILTER_COEFFICIENT;
}

static void motor_read_current(void)
{
	// low pass filter the positive motor readed value (no regen current), to avoid possible fast spikes/noise
	motor_adc_current_accumulated -= motor_adc_current_accumulated >> READ_MOTOR_CURRENT_FILTER_COEFFICIENT;
	motor_adc_current_accumulated += motor_adc_current;
	motor_adc_current_filtered = motor_adc_current_accumulated >> READ_MOTOR_CURRENT_FILTER_COEFFICIENT;
}

static void motor_read_brake(void)
{
	motor_brake_is_active = brake_is_active();
}

static uint8_t motor_asin_table(uint8_t inverted_angle_x128)
{
	// calc asin also converts the final result to degrees

	uint8_t index = 0;

	while (index < SIN_TABLE_LEN)
	{
		if (inverted_angle_x128 < ui8_sin_table[index])
		{
			break;
		}

		index++;
	}

	// first value of table is 0 so ui8_index will always increment to at least 1 and return 0
	return index--;
}

static void motor_calc_foc_angle(void)
{
	uint16_t ui16_temp;
	uint32_t ui32_temp;
	uint16_t e_phase_voltage;
	uint32_t i_phase_current_x2;
	uint32_t l_x1048576;
	uint32_t w_angular_velocity_x16;
	uint16_t iwl_128;

	// FOC implementation by calculating the angle between phase current and rotor magnetic flux (BEMF)
	// 1. phase voltage is calculate
	// 2. I*w*L is calculated, where I is the phase current. L was a measured value for 48V motor.
	// 3. inverse sin is calculated of (I*w*L) / phase voltage, were we obtain the angle
	// 4. previous calculated angle is applied to phase voltage vector angle and so the
	// angle between phase current and rotor magnetic flux (BEMF) is kept at 0 (max torque per amp)

	// calc E phase voltage
	ui16_temp = motor_adc_battery_voltage_filtered * ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X512;
	ui16_temp = (ui16_temp >> 8) * motor_pwm_duty_cycle;
	e_phase_voltage = ui16_temp >> 9;

	// calc I phase current
	if (motor_pwm_duty_cycle > 10)
	{
		ui16_temp = ((uint16_t)motor_adc_battery_current_filtered) * ADC10BITS_BATTERY_CURRENT_PER_ADC_STEP_X512;
		i_phase_current_x2 = ui16_temp / motor_pwm_duty_cycle;
	}
	else
	{
		i_phase_current_x2 = 0;
	}

	// calc W angular velocity: erps * 6.3
	// 101 = 6.3 * 16
	w_angular_velocity_x16 = motor_speed_erps * 101;

	// ---------------------------------------------------------------------------------------------------------------------
	// 36 V motor: L = 76uH
	// 48 V motor: L = 135uH
	// ui32_l_x1048576 = 142; // 1048576 = 2^20 | 48V
	// ui32_l_x1048576 = 84; // 1048576 = 2^20 | 36V
	//
	// ui32_l_x1048576 = 142 <--- THIS VALUE WAS verified experimentaly on 2018.07 to be near the best value for a 48V motor
	// Test done with a fixed mechanical load, duty_cycle = 200 and 100 and measured battery current was 16 and 6 (10 and 4 amps)
	// ---------------------------------------------------------------------------------------------------------------------


#if (MOTOR_TYPE == MOTOR_TYPE_36V)
	ui32_l_x1048576 = 84; // 36 V motor

#elif (MOTOR_TYPE == MOTOR_TYPE_48V)
	l_x1048576 = 142; // 48 V motor
#endif

	// calc IwL
	ui32_temp = i_phase_current_x2 * l_x1048576;
	ui32_temp *= w_angular_velocity_x16;
	iwl_128 = ui32_temp >> 18;

	// calc FOC angle
	motor_foc_angle = motor_asin_table(iwl_128 / e_phase_voltage);

	// low pass filter FOC angle
	motor_foc_angle_accumulated -= motor_foc_angle_accumulated >> 4;
	motor_foc_angle_accumulated += motor_foc_angle;
	motor_foc_angle = motor_foc_angle_accumulated >> 4;
}



void motor_init()
{
	motor_set_pwm_duty_cycle_ramp_up_inverse_step(PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP); // each step = 64us
	motor_set_pwm_duty_cycle_ramp_down_inverse_step(PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP); // each step = 64us

	motor_set_current_ramp_up_amps_s(RAMP_UP_AMPS_PER_SECOND_X10);

	motor_set_target_max_battery_current(MAX_BATTERY_CURRENT_X10);
	motor_set_target_max_motor_current(MAX_MOTOR_CURRENT_X10);

	motor_set_field_weakening_enabled(0);

	motor_hall_sensor_init();
	motor_pwm_init();
}

void motor_process(void)
{
	motor_read_battery_voltage();
	motor_read_battery_current();
	motor_read_current();
	motor_read_brake();
	motor_calc_foc_angle();
}


void motor_enable(void)
{
	TIM1_OC1Init(TIM1_OCMODE_PWM1,
		TIM1_OUTPUTSTATE_ENABLE,
		TIM1_OUTPUTNSTATE_ENABLE,
		255, // initial duty_cycle value
		TIM1_OCPOLARITY_HIGH,
		TIM1_OCPOLARITY_HIGH,
		TIM1_OCIDLESTATE_RESET,
		TIM1_OCIDLESTATE_SET);

	TIM1_OC2Init(TIM1_OCMODE_PWM1,
		TIM1_OUTPUTSTATE_ENABLE,
		TIM1_OUTPUTNSTATE_ENABLE,
		255, // initial duty_cycle value
		TIM1_OCPOLARITY_HIGH,
		TIM1_OCPOLARITY_HIGH,
		TIM1_OCIDLESTATE_RESET,
		TIM1_OCIDLESTATE_SET);

	TIM1_OC3Init(TIM1_OCMODE_PWM1,
		TIM1_OUTPUTSTATE_ENABLE,
		TIM1_OUTPUTNSTATE_ENABLE,
		255, // initial duty_cycle value
		TIM1_OCPOLARITY_HIGH,
		TIM1_OCPOLARITY_HIGH,
		TIM1_OCIDLESTATE_RESET,
		TIM1_OCIDLESTATE_SET);
}

void motor_disable(void)
{
	TIM1_OC1Init(TIM1_OCMODE_PWM1,
		TIM1_OUTPUTSTATE_DISABLE,
		TIM1_OUTPUTNSTATE_DISABLE,
		255, // initial duty_cycle value
		TIM1_OCPOLARITY_HIGH,
		TIM1_OCPOLARITY_HIGH,
		TIM1_OCIDLESTATE_RESET,
		TIM1_OCIDLESTATE_SET);

	TIM1_OC2Init(TIM1_OCMODE_PWM1,
		TIM1_OUTPUTSTATE_DISABLE,
		TIM1_OUTPUTNSTATE_DISABLE,
		255, // initial duty_cycle value
		TIM1_OCPOLARITY_HIGH,
		TIM1_OCPOLARITY_HIGH,
		TIM1_OCIDLESTATE_RESET,
		TIM1_OCIDLESTATE_SET);

	TIM1_OC3Init(TIM1_OCMODE_PWM1,
		TIM1_OUTPUTSTATE_DISABLE,
		TIM1_OUTPUTNSTATE_DISABLE,
		255, // initial duty_cycle value
		TIM1_OCPOLARITY_HIGH,
		TIM1_OCPOLARITY_HIGH,
		TIM1_OCIDLESTATE_RESET,
		TIM1_OCIDLESTATE_SET);
}


uint16_t motor_get_motor_speed_erps(void)
{
	return motor_speed_erps;
}

uint8_t motor_get_pas_cadence_rpm(void)
{
	if (motor_pas_pwm_cycle_ticks >= ((uint16_t)PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS))
	{
		return 0;
	}
	else
	{
		// Cadence in RPM = 60 / (motor_pas_pwm_cycle_ticks * PAS_NUMBER_MAGNETS * 0.000064)
		// = (60 * PWM_CYCLES_SECOND / PAS_NUMBER_MAGNETS) / motor_pas_pwm_cycle_ticks
		return (uint8_t)((60U * ((uint16_t)PWM_CYCLES_SECOND / (uint16_t)PAS_NUMBER_MAGNETS)) / motor_pas_pwm_cycle_ticks);
	}
}

uint8_t motor_get_supply_voltage(void)
{
	uint16_t ui16_batt_voltage_filtered = (uint16_t)motor_adc_battery_voltage_filtered * ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X512;
	return (ui16_batt_voltage_filtered >> 9);
}


void motor_set_field_weakening_enabled(uint8_t enable)
{
	motor_field_weakening_enable = enable;
}


void motor_set_target_max_battery_current(uint16_t amps_x10)
{
	uint16_t offset = adc_get_battery_current_offset();

	uint16_t target_max_current_amps_x10 = amps_x10;
	uint16_t target_max_current_amps_x10_fw = amps_x10 + (amps_x10 >> 2);
	ui16_limit_max(&target_max_current_amps_x10, (uint16_t)MAX_BATTERY_CURRENT_X10);
	ui16_limit_max(&target_max_current_amps_x10_fw, (uint16_t)MAX_BATTERY_CURRENT_X10);

	// 6.410 = 1 / 0.156 (each ADC step for current)
	// 6.410 * 8 = ~51
	motor_adc_target_battery_max_current = (uint16_t)((target_max_current_amps_x10 * 51) / 80);
	motor_adc_target_battery_max_current_fw = (uint16_t)((target_max_current_amps_x10_fw * 51) / 80);

	motor_adc_target_battery_max_current += offset;
	motor_adc_target_battery_max_current_fw += offset;
}

void motor_set_target_max_motor_current(uint16_t amps_x10)
{
	uint16_t offset = adc_get_battery_current_offset();

	uint16_t target_max_current_amps_x10 = amps_x10;
	uint16_t target_max_current_amps_x10_fw = amps_x10 + (amps_x10 >> 2);
	ui16_limit_max(&target_max_current_amps_x10, (uint16_t)MAX_MOTOR_CURRENT_X10);
	ui16_limit_max(&target_max_current_amps_x10_fw, (uint16_t)MAX_MOTOR_CURRENT_X10);

	// 6.410 = 1 / 0.156 (each ADC step for current)
	// 6.410 * 8 = ~51
	motor_adc_target_motor_max_current = (uint16_t)((target_max_current_amps_x10 * 51) / 80);
	motor_adc_target_motor_max_current_fw = (uint16_t)((target_max_current_amps_x10_fw * 51) / 80);

	motor_adc_target_motor_max_current += offset;
	motor_adc_target_motor_max_current_fw += offset;

}

void motor_set_current_ramp_up_amps_s(uint8_t value_x10)
{
	/*---------------------------------------------------------
	NOTE:

	Example of calculation:

	Target ramp up: 5 amps per second

	Every second has 15625 PWM cycles interrupts,
	one ADC battery current step --> 0.156 amps:

	5 / 0.156 = 32 (we need to do 32 steps ramp up per second)

	Therefore:

	15625 / 32 = 488

	15625 * 0.156 = 2437.5; 2437.5 * 10 = 24375
	-----------------------------------------------------------*/

	// calculate current step for ramp up
	uint32_t tmp = ((uint32_t)24375) / ((uint32_t)value_x10);
	motor_current_ramp_up_inverse_step = (uint16_t)tmp;
}



void motor_set_pwm_duty_cycle_target(uint8_t value)
{
	if (value > PWM_DUTY_CYCLE_MAX)
	{
		value = PWM_DUTY_CYCLE_MAX;
	}
	
	// if brake is active, keep duty_cycle target at 0
	if (brake_is_active())
	{
		value = 0;
	}
	
	motor_pwm_duty_cycle_target = value;
}

void motor_set_pwm_duty_cycle_ramp_up_inverse_step(uint16_t value)
{
	motor_pwm_duty_cycle_ramp_up_inverse_step = value;
}

void motor_set_pwm_duty_cycle_ramp_down_inverse_step(uint16_t value)
{
	motor_pwm_duty_cycle_ramp_down_inverse_step = value;
}




// Measures did with a 24V Q85 328 RPM motor, rotating motor backwards by hand:
// Hall sensor A positivie to negative transition | BEMF phase B at max value / top of sinewave
// Hall sensor B positivie to negative transition | BEMF phase A at max value / top of sinewave
// Hall sensor C positive to negative transition | BEMF phase C at max value / top of sinewave

// runs every 64us (PWM frequency)
// Measured on 2020.01.02 by Casainho, the interrupt code takes about 42us which is about 66% of the total 64us
void TIM1_CMP_IRQHandler(void) __interrupt(TIM1_CMP_IRQ)
{
	uint8_t tmp8;
	uint16_t tmp16;

	uint16_t adc_target_motor_max_current;

	/****************************************************************************/
	// read battery current ADC value | should happen at middle of the PWM duty_cycle
	// disable scan mode
	ADC1->CR2 &= (uint8_t)(~ADC1_CR2_SCAN);

	// clear EOC flag first (selected also channel 5)
	ADC1->CSR = 0x05;

	// start ADC1 conversion
	ADC1->CR1 |= ADC1_CR1_ADON;
	while (!(ADC1->CSR & ADC1_FLAG_EOC));
	motor_adc_battery_current = ADC_10_BIT_BATTERY_CURRENT;

	// calculate motor current ADC value
	if (motor_pwm_duty_cycle > 0)
	{
		motor_adc_current = (uint16_t)(motor_adc_battery_current << 8) / motor_pwm_duty_cycle;
	}
	else
	{
		motor_adc_current = 0;
	}

	/****************************************************************************/
	// trigger ADC conversion of all channels (scan conversion, buffered)
	ADC1->CR2 |= ADC1_CR2_SCAN; // enable scan mode
	ADC1->CSR = 0x07; // clear EOC flag first (selected also channel 7)
	ADC1->CR1 |= ADC1_CR1_ADON; // start ADC1 conversion

	/****************************************************************************/
	// read hall sensor signals and:
	// - find the motor rotor absolute angle
	// - calc motor speed in erps (ui16_motor_speed_erps)

	// read hall sensors signal pins and mask other pins
	// hall sensors sequence with motor forward rotation: 4, 6, 2, 3, 1, 5
	motor_hall_sensors_state =
		((HALL_SENSOR_A__PORT->IDR & HALL_SENSOR_A__PIN) >> 5) |
		((HALL_SENSOR_B__PORT->IDR & HALL_SENSOR_B__PIN) >> 1) |
		((HALL_SENSOR_C__PORT->IDR & HALL_SENSOR_C__PIN) >> 3);

	// make sure we run next code only when there is a change on the hall sensors signal
	if (motor_hall_sensors_state != motor_hall_sensors_state_last)
	{
		motor_hall_sensors_state_last = motor_hall_sensors_state;

		switch (motor_hall_sensors_state)
		{
		case 3:
			motor_rotor_absolute_angle = (uint8_t)MOTOR_ROTOR_ANGLE_150;
			break;

		case 1:
			if (motor_half_erps_flag == 1)
			{
				motor_half_erps_flag = 0;
				motor_pwm_cycles_counter_total = motor_pwm_cycles_counter;
				motor_pwm_cycles_counter = 1;

				if (motor_pwm_cycles_counter_total > 0)
				{
					// This division takes 4.4us
					motor_speed_erps = PWM_CYCLES_SECOND / motor_pwm_cycles_counter_total;
				}
				else
				{
					motor_speed_erps = PWM_CYCLES_SECOND;
				}
			
				// update motor commutation state based on motor speed
				if (motor_speed_erps > MOTOR_ROTOR_ERPS_START_INTERPOLATION_60_DEGREES)
				{
					if (motor_commutation_type == BLOCK_COMMUTATION)
					{
						motor_commutation_type = SINEWAVE_INTERPOLATION_60_DEGREES;
					}
				}
				else
				{
					if (motor_commutation_type == SINEWAVE_INTERPOLATION_60_DEGREES)
					{
						motor_commutation_type = BLOCK_COMMUTATION;
						motor_foc_angle = 0;
					}
				}
			}

			motor_rotor_absolute_angle = (uint8_t)MOTOR_ROTOR_ANGLE_210;
			break;

		case 5:
			motor_rotor_absolute_angle = (uint8_t)MOTOR_ROTOR_ANGLE_270;
			break;

		case 4:
			motor_rotor_absolute_angle = (uint8_t)MOTOR_ROTOR_ANGLE_330;
			break;

		case 6:
			motor_half_erps_flag = 1;

			motor_rotor_absolute_angle = (uint8_t)MOTOR_ROTOR_ANGLE_30;
			break;

			// BEMF is always 90 degrees advanced over motor rotor position degree zero
			// and here (hall sensor C blue wire, signal transition from positive to negative),
			// phase B BEMF is at max value (measured on osciloscope by rotating the motor)
		case 2:
			motor_rotor_absolute_angle = (uint8_t)MOTOR_ROTOR_ANGLE_90;
			break;

		default:
			return;
		}

		motor_pwm_cycles_counter_6 = 1;
	}


	/****************************************************************************/
	// count number of fast loops / PWM cycles and reset some states when motor is near zero speed
	if (motor_pwm_cycles_counter < PWM_CYCLES_COUNTER_MAX)
	{
		motor_pwm_cycles_counter++;
		motor_pwm_cycles_counter_6++;
	}
	else // happens when motor is stopped or near zero speed
	{
		motor_pwm_cycles_counter = 1; // don't put to 0 to avoid 0 divisions
		motor_pwm_cycles_counter_6 = 1;
		motor_half_erps_flag = 0;
		motor_speed_erps = 0;
		motor_pwm_cycles_counter_total = 0xffff;
		motor_foc_angle = 0;
		motor_commutation_type = BLOCK_COMMUTATION;
		motor_hall_sensors_state_last = 0; // this way we force execution of hall sensors code next time
	}
	/****************************************************************************/


	// - calc interpolation angle and sinewave table index
#define DO_INTERPOLATION 1 // may be useful to disable interpolation when debugging
#if DO_INTERPOLATION == 1
  // calculate the interpolation angle (and it doesn't work when motor starts and at very low speeds)
	if (motor_commutation_type == SINEWAVE_INTERPOLATION_60_DEGREES)
	{
		// division by 0: motor_pwm_cycles_counter_total should never be 0
		// TODO: verifiy if (motor_pwm_cycles_counter_6 << 8) do not overflow
		motor_interpolation_angle = (motor_pwm_cycles_counter_6 << 8) / motor_pwm_cycles_counter_total; // this operations take 4.4us
		motor_rotor_angle = motor_rotor_absolute_angle + motor_interpolation_angle;
		motor_svm_table_index = motor_rotor_angle;
	}
	else
#endif
	{
		motor_svm_table_index = motor_rotor_absolute_angle;
	}

	motor_svm_table_index += motor_foc_angle;

	// we need to put phase voltage 90 degrees ahead of rotor position, to get current 90 degrees ahead and have max torque per amp
	motor_svm_table_index -= 63;


	/****************************************************************************/
	// PWM duty_cycle controller:
	// - brakes are active
	// - limit battery undervoltage
	// - limit battery max current
	// - limit motor max ERPS
	// - ramp up/down PWM duty_cycle value

	// check to enable field weakening state
	// do not enable at low motor speed / low cadence
	if (motor_field_weakening_enable && (motor_speed_erps > MOTOR_SPEED_FIELD_WEAKEANING_MIN))
	{
		motor_field_weakening_enable_state = 1;
	}
		

	++motor_current_controller_counter;
	++motor_speed_controller_counter;

	if 
	(
		motor_brake_is_active ||
//		(ui8_m_pas_min_cadence_flag && (ui8_g_throttle == 0)) ||
		(ADC_BATTERY_VOLTAGE < ADC8BITS_BATTERY_MOTOR_CUT_OFF)
	)
	{
		if (motor_field_weakening_angle)
		{
			--motor_field_weakening_angle;
		}
		else if (motor_pwm_duty_cycle)
		{
			--motor_pwm_duty_cycle;
		}
	}
	// do not control current at every PWM cycle, that will measure and control too fast. Use counter to limit
	else if 
	(
		motor_current_controller_counter > 14 &&
		(motor_adc_battery_current > motor_adc_target_battery_max_current || motor_adc_current > motor_controller_adc_max_current)
	)
	{
		if (motor_field_weakening_angle)
		{
			--motor_field_weakening_angle;
		}
		else if (motor_pwm_duty_cycle)
		{
			--motor_pwm_duty_cycle;
		}
	}
	else if 
	(
		motor_speed_controller_counter > 2000 && // test about every 100ms
		motor_speed_erps > motor_max_speed_erps)
	{
		if (motor_field_weakening_angle)
		{
			--motor_field_weakening_angle;
		}
		else if (motor_pwm_duty_cycle)
		{
			--motor_pwm_duty_cycle;
		}
	}
	else // nothing to limit, so adjust duty_cycle to duty_cycle_target, including ramping or adjust field weakening
	{
		if (
			motor_pwm_duty_cycle >= PWM_DUTY_CYCLE_MAX && // max voltage already applied to motor windings, enter or keep in field weakening state
			motor_field_weakening_enable_state
		)
		{
			if (motor_adc_current < motor_controller_adc_max_current)
			{
				if (motor_field_weakening_ramp_up_counter++ >= FIELD_WEAKENING_RAMP_UP_INVERSE_STEP)
				{
					motor_field_weakening_ramp_up_counter = 0;

					if (motor_field_weakening_angle < FIELD_WEAKENING_ANGLE_MAX)
					{
						++motor_field_weakening_angle;
					}					
				}
			}
			else if (motor_adc_current > motor_controller_adc_max_current)
			{
				if (motor_field_weakening_ramp_down_counter++ >= FIELD_WEAKENING_RAMP_DOWN_INVERSE_STEP)
				{
					motor_field_weakening_ramp_down_counter = 0;

					if (motor_field_weakening_angle)
					{
						--motor_field_weakening_angle;
					}
					else
					{
						--motor_pwm_duty_cycle; // exit from field weakening state
					}
				}
			}
		}
		else
		{
			if (motor_pwm_duty_cycle_target > motor_pwm_duty_cycle)
			{
				if (motor_pwm_duty_cycle_ramp_up_counter++ >= motor_pwm_duty_cycle_ramp_up_inverse_step)
				{
					motor_pwm_duty_cycle_ramp_up_counter = 0;
					++motor_pwm_duty_cycle;
				}
			}
			else if (motor_pwm_duty_cycle_target < motor_pwm_duty_cycle)
			{
				if (motor_pwm_duty_cycle_ramp_down_counter++ >= motor_pwm_duty_cycle_ramp_down_inverse_step)
				{
					motor_pwm_duty_cycle_ramp_down_counter = 0;
					--motor_pwm_duty_cycle;
				}
			}
		}
	}

	motor_svm_table_index += motor_field_weakening_angle;

	// disable field weakening only after leaving the field weakening state
	if (motor_field_weakening_enable == 0 && motor_pwm_duty_cycle < (uint8_t)PWM_DUTY_CYCLE_MAX)
	{
		motor_field_weakening_enable_state = 0;
	}
		
	if (motor_current_controller_counter > 14)
	{
		motor_current_controller_counter = 0;
	}
		

	/****************************************************************************/
	// calculate final PWM duty_cycle values to be applied to TIMER1
	// scale and apply PWM duty_cycle for the 3 phases
	// phase A is advanced 240 degrees over phase B
	tmp8 = ui8_svm_table[(uint8_t)(motor_svm_table_index + 171 /* 240º */)];
	if (tmp8 > MIDDLE_PWM_DUTY_CYCLE_MAX)
	{
		tmp16 = ((uint16_t)(tmp8 - MIDDLE_PWM_DUTY_CYCLE_MAX)) * motor_pwm_duty_cycle;
		tmp8 = (uint8_t)(tmp16 >> 8);
		motor_phase_a_voltage = MIDDLE_PWM_DUTY_CYCLE_MAX + tmp8;
	}
	else
	{
		tmp16 = ((uint16_t)(MIDDLE_PWM_DUTY_CYCLE_MAX - tmp8)) * motor_pwm_duty_cycle;
		tmp8 = (uint8_t)(tmp16 >> 8);
		motor_phase_a_voltage = MIDDLE_PWM_DUTY_CYCLE_MAX - tmp8;
	}

	// phase B as reference phase
	tmp8 = ui8_svm_table[motor_svm_table_index];
	if (tmp8 > MIDDLE_PWM_DUTY_CYCLE_MAX)
	{
		tmp16 = ((uint16_t)(tmp8 - MIDDLE_PWM_DUTY_CYCLE_MAX)) * motor_pwm_duty_cycle;
		tmp8 = (uint8_t)(tmp16 >> 8);
		motor_phase_b_voltage = MIDDLE_PWM_DUTY_CYCLE_MAX + tmp8;
	}
	else
	{
		tmp16 = ((uint16_t)(MIDDLE_PWM_DUTY_CYCLE_MAX - tmp8)) * motor_pwm_duty_cycle;
		tmp8 = (uint8_t)(tmp16 >> 8);
		motor_phase_b_voltage = MIDDLE_PWM_DUTY_CYCLE_MAX - tmp8;
	}

	// phase C is advanced 120 degrees over phase B
	tmp8 = ui8_svm_table[(uint8_t)(motor_svm_table_index + 85 /* 120º */)];
	if (tmp8 > MIDDLE_PWM_DUTY_CYCLE_MAX)
	{
		tmp16 = ((uint16_t)(tmp8 - MIDDLE_PWM_DUTY_CYCLE_MAX)) * motor_pwm_duty_cycle;
		tmp8 = (uint8_t)(tmp16 >> 8);
		motor_phase_c_voltage = MIDDLE_PWM_DUTY_CYCLE_MAX + tmp8;
	}
	else
	{
		tmp16 = ((uint16_t)(MIDDLE_PWM_DUTY_CYCLE_MAX - tmp8)) * motor_pwm_duty_cycle;
		tmp8 = (uint8_t)(tmp16 >> 8);
		motor_phase_c_voltage = MIDDLE_PWM_DUTY_CYCLE_MAX - tmp8;
	}

	// set final duty_cycle value
	// phase B
	TIM1->CCR3H = (uint8_t)(motor_phase_b_voltage >> 7);
	TIM1->CCR3L = (uint8_t)(motor_phase_b_voltage << 1);
	// phase C
	TIM1->CCR2H = (uint8_t)(motor_phase_c_voltage >> 7);
	TIM1->CCR2L = (uint8_t)(motor_phase_c_voltage << 1);
	// phase A
	TIM1->CCR1H = (uint8_t)(motor_phase_a_voltage >> 7);
	TIM1->CCR1L = (uint8_t)(motor_phase_a_voltage << 1);


	/****************************************************************************/
	// Ramp up ADC battery current

	// field weakening has a higher current value to provide the same torque
	if (motor_field_weakening_enable_state)
	{
		adc_target_motor_max_current = motor_adc_target_motor_max_current_fw;
	}		
	else
	{
		adc_target_motor_max_current = motor_adc_target_motor_max_current;
	}

	//// now ramp up
	if (adc_target_motor_max_current > motor_controller_adc_max_current)
	{
		if (motor_adc_current_ramp_up_counter++ >= motor_current_ramp_up_inverse_step)
		{
			motor_adc_current_ramp_up_counter = 0;
			motor_controller_adc_max_current++;
		}
	}
	else if (adc_target_motor_max_current < motor_controller_adc_max_current)
	{
		// we are not doing a ramp down here, just directly setting to the target value
		motor_controller_adc_max_current = adc_target_motor_max_current;
	}
	/****************************************************************************/

	// Compute PAS timming between each positive pulses, in PWM cycles ticks
	// Compute PAS on and off timming of each pulse, in PWM cycles ticks

	// :TODO: move away from here, simplify logic and use external interrupts?

	motor_pas_counter++;

	// detect PAS signal changes
	if ((PAS1__PORT->IDR & PAS1__PIN) == 0)
	{
		motor_pas_state = 0;
	}
	else
	{
		motor_pas_state = 1;
	}

	// PAS signal did change
	if (motor_pas_state != motor_pas_state_prev)
	{
		motor_pas_state_prev = motor_pas_state;

		// consider only when PAS signal transition from 0 to 1
		if (motor_pas_state == 1)
		{
			// keep track of first pulse
			if (!motor_pas_after_first_pulse)
			{
				motor_pas_after_first_pulse = 1;
				motor_pas_pwm_cycle_ticks = (uint16_t)PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS;
			}
			else
			{
				// limit PAS cadence to be less than PAS_ABSOLUTE_MAX_CADENCE_PWM_CYCLE_TICKS
				if (motor_pas_counter < ((uint16_t)PAS_ABSOLUTE_MAX_CADENCE_PWM_CYCLE_TICKS))
				{
					motor_pas_pwm_cycle_ticks = (uint16_t)PAS_ABSOLUTE_MAX_CADENCE_PWM_CYCLE_TICKS;
				}
				else
				{
					motor_pas_pwm_cycle_ticks = motor_pas_counter;
				}

				motor_pas_min_cadence_pwm_cycles_ticks = (motor_pas_pwm_cycle_ticks + (motor_pas_pwm_cycle_ticks >> 2));

				motor_pas_counter = 0;

				// save direction
				if ((PAS2__PORT->IDR & PAS2__PIN) == 0)
				{
					motor_pas_pedaling_direction = 2;
				}
				else
				{
					motor_pas_pedaling_direction = 1;
				}
			}

			// lef/right
			if ((PAS2__PORT->IDR & PAS2__PIN) == 0)
			{
				motor_pas_tick_counter++;
				if (motor_pas_tick_counter > PAS_NUMBER_MAGNETS_X2)
				{
					motor_pas_tick_counter = 1;
				}
			}
			else
			{
				if (motor_pas_tick_counter <= 1)
				{
					motor_pas_tick_counter = PAS_NUMBER_MAGNETS_X2;
				}
				else
				{
					motor_pas_tick_counter--;
				}
			}
		}
		else
		{
			// keep track of first pulse
			if (motor_pas_after_first_pulse)
			{
				// save direction
				if ((PAS2__PORT->IDR & PAS2__PIN) != 0)
				{
					motor_pas_pedaling_direction = 2;
				}
				else
				{
					motor_pas_pedaling_direction = 1;
				}
			}

			// lef/right
			if ((PAS2__PORT->IDR & PAS2__PIN) != 0)
			{
				motor_pas_tick_counter++;
				if (motor_pas_tick_counter > PAS_NUMBER_MAGNETS_X2)
				{
					motor_pas_tick_counter = 1;
				}
			}
			else
			{
				if (motor_pas_tick_counter <= 1)
				{
					motor_pas_tick_counter = PAS_NUMBER_MAGNETS_X2;
				}
				else
				{
					motor_pas_tick_counter--;
				}
			}
		}

		// define if pedal is right or left
		if (motor_pas_tick_counter > PAS_NUMBER_MAGNETS)
		{
			motor_pas_pedal_right = 0;
		}		
		else
		{
			motor_pas_pedal_right = 1;
		}
	}

	// check for permitted relative min cadence value
	if ((motor_pas_pedaling_direction == 2) || // if rotating pedals backwards
		(motor_pas_counter > motor_pas_min_cadence_pwm_cycles_ticks))
	{
		motor_pas_min_cadence_flag = 1;
	}
	
	else
	{
		motor_pas_min_cadence_flag = 0;
	}

	// limit min PAS cadence
	if (motor_pas_min_cadence_flag || motor_pas_counter > ((uint16_t)PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS))
	{
		motor_pas_pwm_cycle_ticks = (uint16_t)PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS;
		motor_pas_min_cadence_pwm_cycles_ticks = (uint16_t)PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS;
		motor_pas_min_cadence_flag = 0;
		motor_pas_after_first_pulse = 0;
		motor_pas_pedaling_direction = 0;
		motor_pas_counter = 0;
	}
	
	/****************************************************************************/

	///****************************************************************************/
	//// reload watchdog timer, every PWM cycle to avoid automatic reset of the microcontroller
	//if (ui8_first_time_run_flag)
	//{ // from the init of watchdog up to first reset on PWM cycle interrupt,
	//  // it can take up to 250ms and so we need to init here inside the PWM cycle
	//	ui8_first_time_run_flag = 0;
	//	watchdog_init();
	//}
	//else
	//{
	//	IWDG->KR = IWDG_KEY_REFRESH; // reload watch dog timer counter

	//	// if the main loop counteris not reset that it is blocked, so, reset the system
	//	++ui16_main_loop_wdt_cnt_1;
	//	if (ui16_main_loop_wdt_cnt_1 > PWM_CYCLES_SECOND) // 1 second
	//	{
	//		// reset system
	//		//  resets a STM8 microcontroller.
	//		//  It activates the Window Watchdog, which resets all because its seventh bit is null.
	//		//  See page 127 of  RM0016 (STM8S and STM8AF microcontroller family) for more details.
	//		WWDG->CR = 0x80;
	//	}
	//}
	///****************************************************************************/


	// clears the TIM1 interrupt TIM1_IT_UPDATE pending bit
	TIM1->SR1 = (uint8_t)(~(uint8_t)TIM1_IT_CC4);
}
