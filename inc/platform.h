#ifndef PLATFORM_H_
#define PLATFORM_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum{
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Stream0_IRQn           = 11,     /*!< DMA1 Stream 0 global Interrupt                                    */
  DMA1_Stream1_IRQn           = 12,     /*!< DMA1 Stream 1 global Interrupt                                    */
  DMA1_Stream2_IRQn           = 13,     /*!< DMA1 Stream 2 global Interrupt                                    */
  DMA1_Stream3_IRQn           = 14,     /*!< DMA1 Stream 3 global Interrupt                                    */
  DMA1_Stream4_IRQn           = 15,     /*!< DMA1 Stream 4 global Interrupt                                    */
  DMA1_Stream5_IRQn           = 16,     /*!< DMA1 Stream 5 global Interrupt                                    */
  DMA1_Stream6_IRQn           = 17,     /*!< DMA1 Stream 6 global Interrupt                                    */
  ADC_IRQn                    = 18,     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
  TIM1_UP_TIM10_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
  TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  SPI4_IRQn                   = 84      /*!< SPI4 global Interrupt                                              */
} IRQn_Type;

// BIT_ masks
#define BIT_0                   (1 << 0)
#define BIT_1                   (1 << 1)
#define BIT_2                   (1 << 2)
#define BIT_3                   (1 << 3)
#define BIT_4                   (1 << 4)
#define BIT_5                   (1 << 5)
#define BIT_6                   (1 << 6)
#define BIT_7                   (1 << 7)

#define bit_mask(start_bit, end_bit)      (((1 << (end_bit - start_bit + 1)) - 1) << start_bit)    
#define SET_BIT(REG, BIT)      ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)    ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)     ((REG) & (BIT))
#define CLEAR_REG(REG)         ((REG) = (0x0))
#define WRITE_REG(REG, VAL)    ((REG) = (VAL))
#define READ_REG(REG)          ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
#define GENERIC_SET_MSK(val, shift_amount)        (val << shift_amount)

#define HSE_CLOCK             84000000U
#define MCU_CLK               HSE_CLOCK
#define PLLCK1                42U
#define DUMMY_XPSR            0x01000000

#define NUMBER_OF_GLOBAL_TIMERS   1U
#define DELAY_COUNT_1S  	    (1000U)
#define DELAY_COUNT_500MS  		(500U )
#define DELAY_COUNT_250MS 		(250U )
#define DELAY_COUNT_125MS 		(125U )
#define DELAY_COUNT_1MS         (1U   )


#define start_timer(timer_name, duration)         global_timers.timers[timer_name] = duration
#define reset_timer(x)            		          global_timers.timer_flags.timer[x] = RDY; \
										          global_timers.timers[x]            = 0;
typedef union{

	uint8_t timer[NUMBER_OF_GLOBAL_TIMERS];
	struct{
		uint8_t delay_timer;
	};
}flags;

typedef struct{

  flags timer_flags;
  volatile uint32_t timers[NUMBER_OF_GLOBAL_TIMERS];

}global_timers_t;

extern global_timers_t global_timers;

#define	DELAY_TIMER			           	  0

#define RDY				                  0x1
#define IN_PROGRESS		                  0x2
#define EXPIRED                           0x0

#define SYSTICK_TIM_CLK       HSE_CLOCK

#define GPIOA       0
#define GPIOB       1
#define GPIOC       2
#define GPIOD       3
#define GPIOE       4
#define GPIOH       5

typedef enum 
{
 UNLOCKED = 0x00U,
 LOCKED   = 0x01U  
} LockTypeDef;

typedef enum 
{
  SYS_OK       = 0x00U,
  SYS_ERROR    = 0x01U,
  SYS_BUSY     = 0x02U,
  SYS_TIMEOUT  = 0x03U
} StatusTypeDef;

#define UNUSED(X) (void)X  

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

/*!< Peripheral memory map */
#define APB1PERIPH_BASE       PERIPHBASE
#define APB2PERIPH_BASE       (PERIPHBASE + 0x00010000UL)
#define AHB1PERIPH_BASE       (PERIPHBASE + 0x00020000UL)
#define AHB2PERIPH_BASE       (PERIPHBASE + 0x10000000UL)

/*!< APB1 peripherals */
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000UL)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400UL)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800UL)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00UL)
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800UL)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00UL)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x3000UL)
#define I2S2ext_BASE          (APB1PERIPH_BASE + 0x3400UL)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800UL)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x3C00UL)
#define I2S3ext_BASE          (APB1PERIPH_BASE + 0x4000UL)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400UL)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400UL)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800UL)
#define I2C3_BASE             (APB1PERIPH_BASE + 0x5C00UL)
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000UL)

/*!< APB2 peripherals */
#define TIM1_BASE             (APB2PERIPH_BASE + 0x0000UL)
#define USART1_BASE           (APB2PERIPH_BASE + 0x1000UL)
#define USART6_BASE           (APB2PERIPH_BASE + 0x1400UL)
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2000UL)
#define ADC1_COMMON_BASE      (APB2PERIPH_BASE + 0x2300UL)
/* Legacy define */
#define ADC_BASE               ADC1_COMMON_BASE
#define SDIO_BASE             (APB2PERIPH_BASE + 0x2C00UL)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000UL)
#define SPI4_BASE             (APB2PERIPH_BASE + 0x3400UL)
#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x3800UL)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x3C00UL)
#define TIM9_BASE             (APB2PERIPH_BASE + 0x4000UL)
#define TIM10_BASE            (APB2PERIPH_BASE + 0x4400UL)
#define TIM11_BASE            (APB2PERIPH_BASE + 0x4800UL)

/*!< AHB1 peripherals */
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00UL)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000UL)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00UL)
#define CRC_BASE              (AHB1PERIPH_BASE + 0x3000UL)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800UL)
#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x3C00UL)
#define DMA1_BASE             (AHB1PERIPH_BASE + 0x6000UL)
#define DMA1_Stream0_BASE     (DMA1_BASE + 0x010UL)
#define DMA1_Stream1_BASE     (DMA1_BASE + 0x028UL)
#define DMA1_Stream2_BASE     (DMA1_BASE + 0x040UL)
#define DMA1_Stream3_BASE     (DMA1_BASE + 0x058UL)
#define DMA1_Stream4_BASE     (DMA1_BASE + 0x070UL)
#define DMA1_Stream5_BASE     (DMA1_BASE + 0x088UL)
#define DMA1_Stream6_BASE     (DMA1_BASE + 0x0A0UL)
#define DMA1_Stream7_BASE     (DMA1_BASE + 0x0B8UL)
#define DMA2_BASE             (AHB1PERIPH_BASE + 0x6400UL)
#define DMA2_Stream0_BASE     (DMA2_BASE + 0x010UL)
#define DMA2_Stream1_BASE     (DMA2_BASE + 0x028UL)
#define DMA2_Stream2_BASE     (DMA2_BASE + 0x040UL)
#define DMA2_Stream3_BASE     (DMA2_BASE + 0x058UL)
#define DMA2_Stream4_BASE     (DMA2_BASE + 0x070UL)
#define DMA2_Stream5_BASE     (DMA2_BASE + 0x088UL)
#define DMA2_Stream6_BASE     (DMA2_BASE + 0x0A0UL)
#define DMA2_Stream7_BASE     (DMA2_BASE + 0x0B8UL)
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

/* Power control peripheral regs */
#define SPI1_PERIPH_OFFSET     0x3000UL
#define SPI1_PERIPH_BASE      (APB2PERIPH_BASE + SPI1_PERIPH_OFFSET)

#define RCC_APB2ENR_SPI1EN_Pos             (12U)                               
#define RCC_APB2ENR_SPI1EN_Msk             (0x1UL << RCC_APB2ENR_SPI1EN_Pos)    /*!< 0x00001000 */
#define RCC_APB2ENR_SPI1EN                 RCC_APB2ENR_SPI1EN_Msk      

void sys_clock_init(void);
void i2cInit(void);
void delay(uint32_t amount, bool blocking);

#endif /* PLATFORM_H_ */