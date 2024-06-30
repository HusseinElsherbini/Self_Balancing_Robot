#include "platform.h"
#include "main.h"
#include "imu.h"

void sys_clock_init(void){

    uint32_t temp = 0;

    // define clock configuration structs 
    RccOscConfig_t RccOscConfig = {0};
    RCC_ClkInit_t RccClkInit    = {0};

    // enable power to apb1 bus 
    SET_BIT(RCC_REGS->RCC_APB1ENR, RCC_APB1ENR_PWREN);
    READ_BIT(RCC_REGS->RCC_APB1ENR, RCC_APB1ENR_PWREN);

    /* Configure oscillator */
    RccOscConfig.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    RccOscConfig.HSEState            = RCC_HSE_ON;
    RccOscConfig.pllConfig.PLLState  = RCC_PLL_ON;
    RccOscConfig.pllConfig.PLLSource = RCC_PLLSOURCE_HSE;
    RccOscConfig.pllConfig.PLLM      = 25;
    RccOscConfig.pllConfig.PLLN      = 336;
    RccOscConfig.pllConfig.PLLP      = RCC_PLLP_DIV4;
    RccOscConfig.pllConfig.PLLQ      = 7;

    /* set new HSE state */
    SET_BIT(RCC_REGS->RCC_CR, RCC_HSE_ON);

    /* check if HSE is stable */
    while(((RCC_REGS->RCC_CR) & (1UL << 17)) != (1 << 17)){
        __asm("nop");
    }

    /* disable PLL before configuration */
    CLEAR_BIT(RCC_REGS->RCC_CR, (1U << 24));

    /* check if PLL is unlocked */
    while(((RCC_REGS->RCC_CR) & (1UL << 25)) == (1 << 25)){
        __asm("nop");
    }    

    /* Write PLL configurations to PLLCFGR */
    WRITE_REG(RCC_REGS->RCC_PLLCFGR, RccOscConfig.pllConfig.PLLSource  | \
              RccOscConfig.pllConfig.PLLM                              | \
              (RccOscConfig.pllConfig.PLLN << 6U)                      | \
              (((RccOscConfig.pllConfig.PLLP >> 1U) - 1U) << 16U)      | \
              (RccOscConfig.pllConfig.PLLQ << 24U));

    /* Enable PLL */
    SET_BIT(RCC_REGS->RCC_CR, (1U << 24));

    /* check if PLL is stable */
    while(((RCC_REGS->RCC_CR) & (1UL << 25)) != (1 << 25)){
        __asm("nop");
    }  

    /* initialize core, AHB, and APB bus clocks */
    RccClkInit.ClockType = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_PCLK1 |
                           RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_SYSCLK;
    RccClkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RccClkInit.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RccClkInit.APB1CLKDivider = RCC_HCLK_DIV2;
    RccClkInit.APB2CLKDivider = RCC_HCLK_DIV1;

    /* set flash access latency */
    (*(volatile uint8_t*)0x40023C00U) = (uint8_t)FLASH_ACR_LATENCY_3WS;

    /* HCLK config */
    MODIFY_REG(RCC_REGS->RCC_CFGR, (0x7UL << 10U), (0x00001C00U));   
    MODIFY_REG(RCC_REGS->RCC_CFGR, (0x7UL << 13U), (0x00001C00U << 3));
    MODIFY_REG(RCC_REGS->RCC_CFGR, (0xFUL << 4U), RccClkInit.AHBCLKDivider);

    /* SYSClk config */
    MODIFY_REG(RCC_REGS->RCC_CFGR, (0x3UL << 0U), (RccClkInit.SYSCLKSource));

    /* PCLK1 config */
    MODIFY_REG(RCC_REGS->RCC_CFGR, (0x7UL << 10U), (RccClkInit.APB1CLKDivider)); 
    MODIFY_REG(RCC_REGS->RCC_CFGR, (0x7UL << 13U), (RccClkInit.APB2CLKDivider << 3U)); 

    
}

void delay(uint32_t amount, bool blocking){

        // delay for 100ms
    start_timer(DELAY_TIMER, DELAY_COUNT_1MS*amount);

    if(blocking){
        while (global_timers.timer_flags.delay_timer != EXPIRED);
    }
    
    reset_timer(DELAY_TIMER);
}