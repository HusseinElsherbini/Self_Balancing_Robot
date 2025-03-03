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
#include "platform.h"
#include "cortex.h"
#include <stdio.h>
#include "i2c.h"
#include "imu.h"
#include "pwm_common.h"
#include "kalman_filter.h"
#include "spi.h"
#include "aux_board_communication.h"
#include "FreeRTOSConfig.h"
#include "FreeRtos.h"
#include "task.h"
#include "task_macros.h"
#include "SEGGER_RTT.h"
#include "motor.h"
#include "pid.h"
#include "smart_port.h"
#include "sbus_receiver.h"
#include "logger.h"
#include "adc.h"
#include "balancing.h"

void sw_hw_init(void);
void Error_Handler(void);

#endif /* MAIN_H_ */
