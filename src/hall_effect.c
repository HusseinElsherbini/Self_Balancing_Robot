#include "hall_effect.h"



void hall_effect_sensor_init(HallSensor_t *sensor){

    // enable bus to gpio ports of both channels 
    enableBusToGpioPort(sensor->channelA.gpioPort);
    enableBusToGpioPort(sensor->channelB.gpioPort);

    // set mode to alternate function
    setGpioMode(&sensor->channelA.port_base_addr, sensor->channelA.pin, sensor->channelA.mode);
    setGpioMode(&sensor->channelB.port_base_addr, sensor->channelB.pin, sensor->channelB.mode);

    // set no pull up or pull down
    setGpioPupDR(&sensor->channelA.port_base_addr,sensor->channelA.pin, sensor->channelA.pupDr);
    setGpioPupDR(&sensor->channelB.port_base_addr,sensor->channelB.pin, sensor->channelA.pupDr);

    // set alternate function number
    setGpioAlternateFunction(&sensor->channelA.port_base_addr, sensor->channelA.pin, sensor->channelA.alternateFunction);
    setGpioSpeed(&sensor->channelA.port_base_addr, sensor->channelA.pin, sensor->channelA.speed);
    
    setGpioAlternateFunction(&sensor->channelB.port_base_addr, sensor->channelB.pin, sensor->channelB.alternateFunction);
    setGpioSpeed(&sensor->channelB.port_base_addr, sensor->channelB.pin, sensor->channelB.speed);

    
    // enable to timer instance 
    enableBustToTimerPeriph(&sensor->timer_instance);

    // stop timer 
    sensor->timer_instance.timer_config.timer_base_address->CR1 &= ~TIM_CR1_CEN;

    // set prescalar 
    timer_set_prescaler(&sensor->timer_instance.timer_config);

    // max period
    sensor->timer_instance.timer_config.timer_base_address->ARR = 0xFFFF;

    // User standard TI1 input 
    sensor->timer_instance.timer_config.timer_base_address->CR2 &= ~TIM_CR2_TI1S;

    // Configure Channel 1 (PA6) for input capture
    sensor->timer_instance.timer_config.timer_base_address->CCMR1 &= ~TIM_CCMR1_CC1S_Msk;
    sensor->timer_instance.timer_config.timer_base_address->CCMR1 |= TIM_CCMR1_CC1S_0;    // CC1 channel configured as input, mapped to TI1

    // Set input capture filter to reduce noise (0xF = maximum filtering)
    sensor->timer_instance.timer_config.timer_base_address->CCMR1 &= ~TIM_CCMR1_IC1F_Msk;
    sensor->timer_instance.timer_config.timer_base_address->CCMR1 |= (0xF << TIM_CCMR1_IC1F_Pos);

    // Configure Channel 2 (PA7) similarly
    sensor->timer_instance.timer_config.timer_base_address->CCMR1 &= ~TIM_CCMR1_CC2S_Msk;
    sensor->timer_instance.timer_config.timer_base_address->CCMR1 |= TIM_CCMR1_CC2S_0;    // CC2 channel configured as input, mapped to TI2

    sensor->timer_instance.timer_config.timer_base_address->CCMR1 &= ~TIM_CCMR1_IC2F_Msk;
    sensor->timer_instance.timer_config.timer_base_address->CCMR1 |= (0xF << TIM_CCMR1_IC2F_Pos);

    // Enable capture on both channels
    sensor->timer_instance.timer_config.timer_base_address->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;

    // Configure for rising edge detection on both channels
    sensor->timer_instance.timer_config.timer_base_address->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP | 
                                                                        TIM_CCER_CC2P | TIM_CCER_CC2NP);

    // Set up Timer 3 for encoder-like operation
    // SMS = 011 for encoder mode 3 (counts on both TI1 and TI2)
    sensor->timer_instance.timer_config.timer_base_address->SMCR &= ~TIM_SMCR_SMS_Msk;
    sensor->timer_instance.timer_config.timer_base_address->SMCR |= (3 << TIM_SMCR_SMS_Pos);

    // Finally, enable the timer
    sensor->timer_instance.timer_config.timer_base_address->CR1 |= TIM_CR1_CEN;   

}