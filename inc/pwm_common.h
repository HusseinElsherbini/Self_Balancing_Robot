#ifndef PWM_COMMON_H
#define PWM_COMMON_H

#include <stdint.h>
#include "gpio.h"
#include "timer_common.h"

#define PWMA_FREQUENCY                              (20000U)
#define PWMB_FREQUENCY                              (20000U)        

#define PWM_RESOLUTION                              (4199)
#define PWM_INPUT_CLOCK_PRESCALER                   0U

#define CALCULATE_ARR_VALUE                         (uint32_t)(((MCU_CLK)/((PWM_INPUT_CLOCK_PRESCALER + 1U)*(PWMA_FREQUENCY)) - 1U))


typedef struct
{
    GPIO_CONFIG_t pwm_pin;

}PWM_CONFIG_T;

typedef enum 
{
    PWM_OK = 0,
    PWM_INVALID_DUTY_CYCLE,
    PWM_INVALID_FREQUENCY,

}PWM_ERROR_T;
typedef struct
{
    PWM_CONFIG_T pwm_ch_config;
    TIMER_TIM_T timerX;
    float dutyCycle;
    uint32_t freq;
    uint32_t resolution;

}PWM_CH_T;

extern PWM_CH_T pwmA;
extern PWM_CH_T pwmB;

PWM_ERROR_T pwmSetDutyCycle(PWM_CH_T *pwm_ch, uint32_t duty_cycle);
void pwmInit(PWM_CH_T *pwm_ch);

#endif /* PWM_COMMON_H*/