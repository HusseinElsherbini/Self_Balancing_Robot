#ifndef ROTARY_ENCODER_H_
#define ROTARY_ENCODER_H_

#include "stdint.h"
#include "gpio.h"
#include "platform.h"

#define CCW 0U
#define CW  1U
typedef struct
{
    uint8_t current_state;
    uint8_t previous_state;
    uint8_t counter;
    uint8_t direction;
    bool stateChanged;
    GPIO_CONFIG_t clk_pin;
    GPIO_CONFIG_t dt_pin;
    
}RE_T;

extern RE_T rEncoder;

void reEncoderInit(RE_T *re);
void checkEncoderStateChange(RE_T *re);

#endif /* ROTARY_ENCODER_H_ */