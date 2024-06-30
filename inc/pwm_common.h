#ifndef PWM_COMMON_H
#define PWM_COMMON_H

#include <stdint.h>
#include "gpio.h"
#include "timer_common.h"

#define PWMA_FREQUENCY                              (10000U)
#define PWMB_FREQUENCY                              (10000U)        

#define PWM_RESOLUTION                              (1000U)
#define PWM_INPUT_CLOCK_PRESCALER                   0U

#define CALCULATE_ARR_VALUE                         (uint32_t)(((MCU_CLK)/((PWM_INPUT_CLOCK_PRESCALER + 1U)*(PWMA_FREQUENCY)) - 1U))


typedef struct
{
    GPIO_CONFIG_t pwm_pin;

}PWM_CONFIG_T;

typedef struct
{
    PWM_CONFIG_T pwm_ch_config;
    TIMER_TIM_T timerX;
    float dutyCycle;
    uint32_t freq;

}PWM_CH_T;

extern PWM_CH_T pwmA;
extern PWM_CH_T pwmB;

void pwmSetDutyCycle(PWM_CH_T *pwm_ch, float duty_cycle);
void pwmInit(PWM_CH_T *pwm_ch);

#endif /* PWM_COMMON_H*/