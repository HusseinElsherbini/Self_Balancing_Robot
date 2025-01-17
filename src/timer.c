#include "timer_common.h"
#include "FreeRTOSConfig.h"
#include "cortex.h"

//#include "main.h"

TIMER_TIM_T htim5 = {0};

void initTimer5(void)
{

    /* Initialize TIM5 */
    htim5.timer_config.timer_base_address = TIM5;
    htim5.timer_config.div_clk = TIM_CR1_CKD_CK_INT;
    htim5.timer_config.timer_prescaler = (configCPU_CLOCK_HZ / 1000000) - 1; // For 1MHz timer clock
    htim5.timer_config.alignment = TIM_CR1_CMS_EDGE;
    htim5.timer_config.direction = TIM1_CR1_DIR_UP;
    htim5.timer_config.timxRccEn = TIM5_RCC_EN;

    /* Enable TIM5 clock */
    enableBustToTimerPeriph(&htim5);
    
    /* Configure Timer */
    // Reset timer registers
    htim5.timer_config.timer_base_address->CR1 = 0;

    // Setup clock division
    SET_BIT(htim5.timer_config.timer_base_address->CR1, htim5.timer_config.div_clk);
    
    // Set prescaler
    htim5.timer_config.timer_base_address->PSC = htim5.timer_config.timer_prescaler;
    
    // Set auto-reload value for 1kHz (1ms) interrupt
    htim5.timer_config.timer_base_address->ARR = (1000000 / configTICK_RATE_HZ) - 1;

       

    // Set counter direction (up counting)
    MODIFY_REG(htim5.timer_config.timer_base_address->CR1, 
               TIM_CR1_DIR_MASK, 
               htim5.timer_config.direction);
    
    // Set alignment mode
    MODIFY_REG(htim5.timer_config.timer_base_address->CR1, 
               TIM_CR1_CMS_MASK, 
               htim5.timer_config.alignment);
    
    // Enable update interrupt
    SET_BIT(htim5.timer_config.timer_base_address->DIER, TIM_DIER_UIE);
   
    /* Enable timer interrupt in NVIC */
    __NVIC_SetPriority(TIM5_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    __NVIC_EnableIRQ(TIM5_IRQn);
   
    /* Start timer */
    // Enable counter
    SET_BIT(htim5.timer_config.timer_base_address->CR1, TIM_CR1_CEN);
    
    // Generate update event to load prescaler and ARR values
    SET_BIT(htim5.timer_config.timer_base_address->EGR, TIM_EGR_UG);
}

void setup_timer_mode(TIMER_CONFIG_T * timerConfig){

    // setup clock division 
    SET_BIT(timerConfig->timer_base_address->CR1, timerConfig->div_clk);

    // alignment edge mode select 
    MODIFY_REG(timerConfig->timer_base_address->CR1, (3U << 5U) ,timerConfig->alignment);

    // set direction of counting
    MODIFY_REG(timerConfig->timer_base_address->CR1, (1U << 4U), timerConfig->direction);

    // set auto reload/preload enable
    SET_BIT(timerConfig->timer_base_address->CR1, (1U << 7U));
    
}

void setup_timer_oc_mode(TIMER_CONFIG_T * timerConfig){

    uint32_t *CRx = &timerConfig->timer_base_address->CR1;
    uint32_t *CCMRx = 0;
    if(timerConfig->channel == CH1 || timerConfig->channel == CH2){
        CCMRx = &timerConfig->timer_base_address->CCMR1;
    }
    else{
        CCMRx = &timerConfig->timer_base_address->CCMR2;
    }
    // how much to shift value in CCMRx register for output mode selection
    uint8_t shiftAmt = 4U;
    if(timerConfig->channel == CH2 || timerConfig->channel == CH4){
        shiftAmt = 12U;
    }
    
    // set output compare mode to PWM
    MODIFY_REG(*CCMRx, (7U << shiftAmt), (timerConfig->timer_oc_mode << shiftAmt));

    // set output compare polarity
    MODIFY_REG(timerConfig->timer_base_address->CCER, (1U << 1U), (0U << 1U));

    // set preload enable, set channel as output in CCMRx reg
    if(timerConfig->channel == CH1 || timerConfig->channel == CH3){
        SET_BIT(*CCMRx, (1U << 3U));
        MODIFY_REG(*CCMRx, (3U << 0U), (timerConfig->captureCompareSelect << 0U));
    }
    else{
        SET_BIT(*CCMRx, (1U << 11U));
        MODIFY_REG(*CCMRx, (3U << 8U), (timerConfig->captureCompareSelect << 8U));
    }
}

// freq = inputClock / ((prescalar - 1) * (arr - 1))
void setup_timer_freq(TIMER_CONFIG_T *timerConfig, uint16_t arr_val){
    WRITE_REG(timerConfig->timer_base_address->ARR, arr_val);
}

void timer_enable_counter(TIMER_CONFIG_T * timerConfig){

    SET_BIT(timerConfig->timer_base_address->CR1, (1U << 0U));
}

void timer_enable_oc_channel(TIMER_CONFIG_T * timerConfig){

    SET_BIT(timerConfig->timer_base_address->CCER, (1U << 4U*(timerConfig->channel - 1)));
}

void timer_set_prescaler(TIMER_CONFIG_T * timerConfig){

    WRITE_REG(timerConfig->timer_base_address->PSC, timerConfig->timer_prescaler);
}

// setup timer, Clock input is 84Mhz
void timer_setup(TIMER_TIM_T * timer){

    setup_timer_mode(&timer->timer_config);
    setup_timer_oc_mode(&timer->timer_config);
    timer_set_prescaler(&timer->timer_config);
    setup_timer_freq(&timer->timer_config, timer->autoreloadVal);
    timer_enable_counter(&timer->timer_config);
    timer_enable_oc_channel(&timer->timer_config);
    if(timer->timer_config.timer_base_address == TIM1){
        // set MOE bits for advanced timer 
        SET_BIT(timer->timer_config.timer_base_address->BDTR, (1U << 15U));        
    }
}

void enableBustToTimerPeriph(TIMER_TIM_T * timer){

    if(timer->timer_config.timer_base_address == TIM1){
        SET_BIT(RCC_REGS->RCC_APB2ENR, (1 << timer->timer_config.timxRccEn));
    }
    else if(timer->timer_config.timer_base_address == TIM5){
        SET_BIT(RCC_REGS->RCC_APB1ENR, (1 << timer->timer_config.timxRccEn));
    }
    
}