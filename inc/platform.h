#ifndef PLATFORM_H_
#define PLATFORM_H_

#include <stdint.h>
#include <stdbool.h>


#define HSE_CLOCK             84000000U
#define MCU_CLK               HSE_CLOCK
#define PLLCK1                42U
#define DUMMY_XPSR            0x01000000

#define SYSTICK_TIM_CLK       HSE_CLOCK

#define GPIOA       0
#define GPIOB       1
#define GPIOC       2
#define GPIOD       3
#define GPIOE       4
#define GPIOH       5


/* Pheripheral base address and AHB1 peripheral base address */
#define PERIPHBASE            0x40000000UL
#define APB1_PERIPH_OFFSET    0x0UL
#define APB2_PERIPH_OFFSET    0x10000UL
#define AHB1_PERIPH_OFFSET    0x20000UL
#define AHB2_PERIPH_OFFSET    0x10000000UL

#define APB1PERIPH_BASE       (PERIPHBASE + APB1_PERIPH_OFFSET)
#define APB2PERIPH_BASE       (PERIPHBASE + APB2_PERIPH_OFFSET)
#define AHB1PERIPHBASE        (PERIPHBASE + AHB1_PERIPH_OFFSET)
#define AHB2PERIPH_BASE       (PERIPHBASE + AHB2_PERIPH_OFFSET)

/* Power control peripheral regs */
#define PWR_PERIPH_OFFSET     0x7000UL
#define PWR_PERIPH_BASE      (APB1PERIPH_BASE + PWR_PERIPH_OFFSET)
#define PWR_CR_VOS_1          0x00008000UL  

typedef struct 
{
    uint32_t PWR_CR;            /* PWR power control register */
    uint32_t PWR_CSR;           /* PWR power control/status register */

}PWR_REGS_t;

#define PWR_REGS               ((PWR_REGS_t*)PWR_PERIPH_BASE)

/* offsets relative to AHB1 peripheral base address*/
#define GPIOA_AHB1_OFFSET     0x0UL
#define GPIOB_AHB1_OFFSET     0x400UL
#define GPIOC_AHB1_OFFSET     0x800UL
#define GPIOD_AHB1_OFFSET     0xC00UL
#define GPIOE_AHB1_OFFSET     0x1000UL
#define GPIOH_AHB1_OFFSET     0x1C00UL

/* GPIOx peripheral base addresses */
#define GPIOA_PERIPH_BASE     (AHB1PERIPHBASE + GPIOA_AHB1_OFFSET)
#define GPIOB_PERIPH_BASE     (AHB1PERIPHBASE + GPIOB_AHB1_OFFSET)
#define GPIOC_PERIPH_BASE     (AHB1PERIPHBASE + GPIOC_AHB1_OFFSET)
#define GPIOD_PERIPH_BASE     (AHB1PERIPHBASE + GPIOD_AHB1_OFFSET)
#define GPIOE_PERIPH_BASE     (AHB1PERIPHBASE + GPIOE_AHB1_OFFSET)
#define GPIOH_PERIPH_BASE     (AHB1PERIPHBASE + GPIOH_AHB1_OFFSET)

#define GPIOx_MODER_OFFSET            (0x0UL)
#define GPIOx_IDR_OFFSET              (0x10UL)
#define GPIOx_ODR_OFFSET              (0x14UL)
 
        
/* GPIOx ENABLE */
#define GPIOA_EN              (1U << 0)
#define GPIOB_EN              (1U << 1)
#define GPIOC_EN              (1U << 2)
#define GPIOD_EN              (1U << 3)
#define GPIOE_EN              (1U << 4)
#define GPIOH_EN              (1U << 7)

#define GPIOx_ENABLE(gpioPort) (1U << gpioPort)

typedef struct 
{
    volatile uint32_t MODER;           /* GPIO port mode register */
    volatile uint32_t OTYPER;          /* GPIO port output type register */
    volatile uint32_t OSPEEDR;         /* GPIO port output speed register  */
    volatile uint32_t PUPDR;           /* GPIO port pull-up/pull-down register */
    volatile uint32_t IDR;             /* GPIO port input data register  */
    volatile uint32_t ODR;             /* GPIO port output data register */
    volatile uint32_t BSRR;            /* GPIO port bit set/reset register  */
    volatile uint32_t LCKR;            /* GPIO port configuration lock register */
    volatile uint32_t AFRL;            /* GPIO alternate function low register  */
    volatile uint32_t AFRH;            /* GPIO alternate function high register */

}GPIO_REGS_t;

typedef struct 
{
    GPIO_REGS_t *port_base_addr;        /* GPIO port base address */
    uint8_t gpioPort;                   /* GPIO Port #*/
    uint8_t pin;                        /* GPIO pin # */

}GPIO_CONFIG_t;

/* timer peripheral registers and base address */
#define TIM1_APB2_OFFSET               0x0UL
#define TIM1_PERIPH_BASE               (APB2PERIPH_BASE + TIM1_APB2_OFFSET)  
#define TIM1_RCC_EN                    0U

/* Reset and clock control register offset and base address*/
#define RCC_OFFSET            0x3800UL
#define RCC_BASE              (AHB1PERIPHBASE + RCC_OFFSET)
#define RCC_AHB1EN_OFFSET     (0x30UL)
#define RCC_AHB1EN            (RCC_BASE + RCC_AHB1EN_OFFSET) 

/* clock configuration structs */

#define RCC_APB1ENR_PWREN                 (0x1UL << 28U)
#define RCC_OSCILLATORTYPE_HSE             0x00000001U
#define RCC_HSE_ON                        (0x1UL << 16U) 
#define RCC_PLL_ON                        ((uint8_t)0x02)
#define RCC_PLLSOURCE_HSE                 (0x1UL << 22U)
#define RCC_PLLP_DIV4                      0x00000004U
#define RCC_CLOCKTYPE_SYSCLK               0x00000001U
#define RCC_CLOCKTYPE_HCLK                 0x00000002U
#define RCC_CLOCKTYPE_PCLK1                0x00000004U
#define RCC_CLOCKTYPE_PCLK2                0x00000008U
#define RCC_SYSCLKSOURCE_PLLCLK            0x00000002U 
#define RCC_HCLK_DIV2                      0x00001000U 
#define RCC_HCLK_DIV2                      0x00001000U 
#define RCC_HCLK_DIV1                      0x00000000U
#define RCC_SYSCLK_DIV1                    0x00000000U
#define FLASH_ACR_LATENCY_3WS              0x00000003U


typedef struct 
{
    volatile uint32_t RCC_CR;                   /* clock control register*/
    volatile uint32_t RCC_PLLCFGR;              /* RCC PLL configuration register */
    volatile uint32_t RCC_CFGR;                 /* clock configuration register */
    volatile uint32_t RCC_CIR;                  /* RCC clock interrupt register */
    volatile uint32_t RCC_AHB1RSTR;             /* RCC AHB1 peripheral reset register  */
    volatile uint32_t RCC_AHB2RSTR;             /* RCC AHB2 peripheral reset register  */
    uint32_t RESERVED0[2];
    volatile uint32_t RCC_APB1RSTR;             /* RCC APB1 peripheral reset register */
    volatile uint32_t RCC_APB2RSTR;             /* RCC APB2 peripheral reset register */
    uint32_t RESERVED1[2];
    volatile uint32_t RCC_AHB1ENR;              /* RCC AHB1 peripheral clock enable register */
    volatile uint32_t RCC_AHB2ENR;              /* RCC AHB2 peripheral clock enable register  */
    uint32_t RESERVED2[2];
    volatile uint32_t RCC_APB1ENR;              /* RCC APB1 peripheral clock enable register  */
    volatile uint32_t RCC_APB2ENR;              /* RCC APB2 peripheral clock enable register */


}RCC_REGS_t;

#define RCC_REGS               ((RCC_REGS_t*)RCC_BASE)

/* clock configuration structs */
typedef struct 
{
    uint32_t PLLState;
    uint32_t PLLSource;     /* clock source of PLL */
    uint32_t PLLM;          /* division factor  */
    uint32_t PLLN;          /* multiplication factor */
    uint32_t PLLP;          /* second division factor  */
    uint32_t PLLQ;          /* division factor for OTG fs, SDIO and RNG clocks  */

}PLLConfig_t;

typedef struct 
{
    uint32_t OscillatorType;
    uint32_t HSEState;          /* state of high speed external oscillator */
    uint32_t LSEState;          /* state of low speed external oscillator  */
    uint32_t HSIstate;          /* state of high speed internal oscillator */
    uint32_t LSIstate;          /* state of low speed internal oscillator  */
    uint32_t HSICalibrationVal; /* HSI trimming value */
    PLLConfig_t pllConfig;

}RccOscConfig_t;

typedef struct
{
  uint32_t ClockType;             /*!< The clock to be configured. */
  uint32_t SYSCLKSource;          /*!< The clock source (SYSCLKS) used as system clock. */                                       
  uint32_t AHBCLKDivider;         /*!< The AHB clock (HCLK) divider. This clock is derived from the system clock (SYSCLK). */                                      
  uint32_t APB1CLKDivider;        /*!< The APB1 clock (PCLK1) divider. This clock is derived from the AHB clock (HCLK). */                                       
  uint32_t APB2CLKDivider;        /*!< The APB2 clock (PCLK2) divider. This clock is derived from the AHB clock (HCLK). */
                                       
}RCC_ClkInit_t;

/* I2C configuration structs */
#define I2C1_PERIPH_OFFSET               (0x5400UL)           
#define I2C1_BASE_ADDRESS                (APB1PERIPH_BASE + I2C1_PERIPH_OFFSET)
#define I2C1_REGS                        ((I2C_REGS_t *)I2C1_BASE_ADDRESS)
#define I2C1_APB1EN                      (1U << 21U)
#define GPIO_I2C1_AF_SDA                 (4U << 24)
#define I2C_CCR_VAL                      (0xD2UL)
#define I2C1_PLCK1_PERIOD                (1.0/((float)PLLCK1))
#define I2C_SM_TRISE_time                ((uint32_t)(1000.0/(I2C1_PLCK1_PERIOD*1000.0)))


typedef struct 
{
    volatile uint32_t I2C_CR1;                /* I2C Control register 1*/
    volatile uint32_t I2C_CR2;                /* I2C Control register 2 */
    volatile uint32_t I2C_OAR1;               /* I2C Own address register 1 */
    volatile uint32_t I2C_OAR2;               /* I2C Own address register 2 */
    volatile uint32_t I2C_DR;                 /* I2C Data register  */
    volatile uint32_t I2C_SR1;                /* I2C Status register 1  */
    volatile uint32_t I2C_SR2;                /* I2C Status register 2 */
    volatile uint32_t I2C_CCR;                /* I2C Clock control register */
    volatile uint32_t I2C_TRISE;              /* I2C TRISE register */
    volatile uint32_t I2C_FLTR;               /* I2C FLTR register  */

}I2C_REGS_t;

typedef struct 
{
    I2C_REGS_t* i2c_regs_base_addr;
    GPIO_CONFIG_t scl_pin_config;
    GPIO_CONFIG_t sda_pin_config;
    
}I2C_CONFIG_t;

extern I2C_CONFIG_t i2c_config;

void sys_clock_init(void);
void i2cInit(void);
void delay(uint32_t amount, bool blocking);

#endif /* PLATFORM_H_ */