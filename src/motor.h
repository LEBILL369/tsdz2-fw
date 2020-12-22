/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "interrupts.h"

#include <stdint.h>


void motor_init(void);
void motor_process(void);

void motor_enable(void);
void motor_disable(void);

uint16_t motor_get_motor_speed_erps(void);
uint8_t motor_get_pas_cadence_rpm(void);
uint8_t motor_get_supply_voltage(void);

void motor_set_field_weakening_enabled(uint8_t enable);

void motor_set_target_max_battery_current(uint16_t amps_x10);
void motor_set_target_max_motor_current(uint16_t amps_x10);
void motor_set_current_ramp_up_amps_s(uint8_t value_x10);

void motor_set_pwm_duty_cycle_target(uint8_t value);
void motor_set_pwm_duty_cycle_ramp_up_inverse_step(uint16_t value); // each step = 64us
void motor_set_pwm_duty_cycle_ramp_down_inverse_step(uint16_t value); // each step = 64us

void TIM1_CMP_IRQHandler(void) __interrupt(TIM1_CMP_IRQ);


#endif /* _MOTOR_H_ */
