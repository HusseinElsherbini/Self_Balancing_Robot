#ifndef HALL_EFFECT_H_
#define HALL_EFFECT_H_

#include "stdint.h"
#include "timer_common.h"
#include "gpio.h"

typedef struct {
    TIMER_TIM_T timer_instance;
    GPIO_CONFIG_t channelA;
    GPIO_CONFIG_t channelB;
    uint8_t current_state;  // Current hall sensor reading
    uint8_t last_state;     // Previous reading for direction detection
    uint32_t timestamp;     // Time of last state change
    int32_t position;       // Accumulated position
    float speed;            // Calculated speed in RPM
} HallSensor_t;


void hall_effect_sensor_init(HallSensor_t *sensor);

#endif /* HALL_EFFECT_H_ */