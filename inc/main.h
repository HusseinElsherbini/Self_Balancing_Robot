/*
 * main.h
 *
 *  Created on: Jan 21, 2024
 *      Author: Hae14
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>
#include "led.h"
#include "gpio.h"
#include "rotary_encoder.h"
#include "task_scheduler.h"
#include "platform.h"
#include "cortex.h"
#include <stdio.h>
#include "i2c.h"
#include "imu.h"
#include "pwm_common.h"

void sw_hw_init(void);

#endif /* MAIN_H_ */
