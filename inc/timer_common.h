#ifndef TIMER_COMMON_H_
#define TIMER_COMMON_H_

#include "stdint.h"
#include "platform.h"

#define TIM_CR1_CKD_CK_INT      (0x0 << 8U)
#define TIM_CR1_CMS_EDGE        (0x0 << 5U)
#define TIM1_CR1_DIR_UP         (0x0 << 4U)
#define TIM1_CR1_DIR_DOWN       (0x1 << 4U)


typedef struct 
{
    volatile uint32_t CR1;           
    volatile uint32_t CR2;       
    volatile uint32_t SMCR;       
    volatile uint32_t DIER;        
    volatile uint32_t SR;             
    volatile uint32_t EGR;            
    volatile uint32_t CCMR1;          
    volatile uint32_t CCMR2;            
    volatile uint32_t CCER;          
    volatile uint32_t CNT;     
    volatile uint32_t PSC;          
    volatile uint32_t ARR;     
    volatile uint32_t RCR;          
    volatile uint32_t CCR1;  
    volatile uint32_t CCR2;          
    volatile uint32_t CCR3; 
    volatile uint32_t CCR4;  
    volatile uint32_t BDTR;          
    volatile uint32_t DCR; 
    volatile uint32_t DMAR;  

}TIM_REGS_T;

#define TIM1                ((TIM_REGS_T *) TIM1_PERIPH_BASE)   

#define CH1                1U
#define CH2                2U
#define CH3                3U
#define CH4                4U

#define CCRx_CHANNEL_PERIPH_OFFSET(CHANNEL)      (0x30 + 4U*CHANNEL)
#define TIMx_CCREG_CHANNEL_OFFSET_ADDRESS(TIMx_PERIPH_BASE, CHANNEL)     (TIMx_PERIPH_BASE + CCRx_CHANNEL_PERIPH_OFFSET(CHANNEL))

enum timer_oc_modes {

    FROZEN,
    ACTIVE,
    INACTIVE,
    TOGGLE,
    FORCE_LOW,
    FORCE_HIGH,
    PWM1,
    PWM2
};

enum timer_mode {

    OUTPUT_MODE,
    INPUT_MAPPED_TO_TI1,
    INPUT_MAPPED_TO_T12,
    INPUT_MAPPED_TO_TRC
};

typedef struct 
{
    TIM_REGS_T *timer_base_address;        /* timer port base address */
    uint32_t div_clk;
    uint8_t timxRccEn;
    uint32_t direction;
    uint8_t channel;
    uint8_t captureCompareSelect;
    uint32_t alignment;
    uint8_t timer_oc_mode;
    uint8_t timer_prescaler;

}TIMER_CONFIG_T;

typedef struct 
{
    TIMER_CONFIG_T timer_config;
    uint32_t autoreloadVal;
    uint32_t captureCompareVal;
    uint32_t *ccrReg;


}TIMER_TIM_T;

void setup_timer_mode(TIMER_CONFIG_T * timerConfig);
void setup_timer_freq(TIMER_CONFIG_T *timerConfig, uint16_t arr_val);
void setup_timer_oc_mode(TIMER_CONFIG_T * timerConfig);
void timer_setup(TIMER_TIM_T * timer);
void enableBustToTimerPeriph(TIMER_TIM_T * timer);
void timer_enable_counter(TIMER_CONFIG_T * timerConfig);
void timer_enable_oc_channel(TIMER_CONFIG_T * timerConfig);
void timer_set_prescaler(TIMER_CONFIG_T * timerConfig);

#endif /*TIMER_COMMON_H_*/
