﻿#include "pwm_common.h"

PWM_CH_T pwmA = {
    
    .pwm_ch_config.pwm_pin.port_base_addr        = (GPIO_CONFIG_t *)GPIOA_PERIPH_BASE,
    .pwm_ch_config.pwm_pin.pin                   = 10U,
    .pwm_ch_config.pwm_pin.gpioPort              = GPIOA,
    .timerX.timer_config.captureCompareSelect    = OUTPUT_MODE,
    .timerX.timer_config.timer_oc_mode           = PWM1,
    .timerX.timer_config.channel                 = CH3,
    .timerX.timer_config.timer_base_address      = TIM1,
    .timerX.timer_config.div_clk                 = TIM_CR1_CKD_CK_INT,
    .timerX.timer_config.alignment               = TIM_CR1_CMS_EDGE,
    .timerX.timer_config.timxRccEn               = TIM1_RCC_EN,
    .timerX.timer_config.direction               = TIM1_CR1_DIR_UP,
    .timerX.timer_config.timer_prescaler         = PWM_INPUT_CLOCK_PRESCALER,
    .timerX.ccrReg                               = (uint32_t *)TIMx_CCREG_CHANNEL_OFFSET_ADDRESS(TIM1_PERIPH_BASE, CH3),
    .dutyCycle                                   = 0U,
    .freq                                        = PWMA_FREQUENCY,   // frequency is 20kHz
    .timerX.autoreloadVal                        = CALCULATE_ARR_VALUE,
    .resolution                                  = PWM_RESOLUTION
}; 

PWM_CH_T pwmB = {
    
    .pwm_ch_config.pwm_pin.port_base_addr        = (GPIO_CONFIG_t *)GPIOA_PERIPH_BASE,
    .pwm_ch_config.pwm_pin.pin                   = 9U,
    .pwm_ch_config.pwm_pin.gpioPort              = GPIOA,
    .timerX.timer_config.captureCompareSelect    = OUTPUT_MODE,
    .timerX.timer_config.timer_oc_mode           = PWM1,
    .timerX.timer_config.channel                 = CH2,
    .timerX.timer_config.timer_base_address      = TIM1,
    .timerX.timer_config.div_clk                 = TIM_CR1_CKD_CK_INT,
    .timerX.timer_config.alignment               = TIM_CR1_CMS_EDGE,
    .timerX.timer_config.timxRccEn               = TIM1_RCC_EN,
    .timerX.timer_config.direction               = TIM1_CR1_DIR_UP,
    .timerX.timer_config.timer_prescaler         = PWM_INPUT_CLOCK_PRESCALER,
    .timerX.ccrReg                               = (uint32_t *)TIMx_CCREG_CHANNEL_OFFSET_ADDRESS(TIM1_PERIPH_BASE, CH2),
    .dutyCycle                                   = 0U,
    .freq                                        = PWMA_FREQUENCY,   // frequency is 20kHz
    .timerX.autoreloadVal                        = CALCULATE_ARR_VALUE,
    .resolution                                  = PWM_RESOLUTION
    
}; 

void pwmInit(PWM_CH_T *pwm_ch){

    // enable clock to TIMx peripheral 
    enableBustToTimerPeriph(&pwm_ch->timerX);

    // enable clock to pwm output gpio pin
    enableBusToGpioPort(pwm_ch->pwm_ch_config.pwm_pin.gpioPort);

    // set gpio mode to output
    setGpioMode(&pwm_ch->pwm_ch_config.pwm_pin.port_base_addr, pwm_ch->pwm_ch_config.pwm_pin.pin, GPIO_AF_MODE);

    // set no pull down or pull up
    setGpioPupDR(&pwm_ch->pwm_ch_config.pwm_pin.port_base_addr, pwm_ch->pwm_ch_config.pwm_pin.pin, NOPULLUP_NOPULLDOWN);

    // set gpio to alternate function 
    setGpioAlternateFunction(&pwm_ch->pwm_ch_config.pwm_pin.port_base_addr, pwm_ch->pwm_ch_config.pwm_pin.pin, AF1);

    // setup timer configuration
    timer_setup(&pwm_ch->timerX);

}

PWM_ERROR_T pwmSetDutyCycle(PWM_CH_T *pwm_ch, uint32_t duty_cycle){
    pwm_ch->dutyCycle = duty_cycle;
    // Check for valid duty cycle
    if (duty_cycle < 0 || duty_cycle > pwm_ch->resolution) {
        return PWM_INVALID_DUTY_CYCLE;
    }

    *pwm_ch->timerX.ccrReg = (uint32_t)duty_cycle;
    return PWM_OK;
}

void pwmSetFreq(PWM_CH_T *pwm_ch){
    
}