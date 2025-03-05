#include "adc.h"
#include "gpio.h"
#include "dma.h"
#include "FreeRTOSConfig.h"
#include "logger.h"
#include "cortex.h"
#include "timer_common.h"

uint16_t adc_dma_buffer[NUM_OF_SENSORS] = {0};

// Example initialization for your specific needs
ADC_CHANNEL_CONFIG_t system_sensors_configs[NUM_OF_SENSORS] = {
    {
        // VIN Sensing on PA5
        .gpio = {
            .port_base_addr = (GPIO_REGS_t*)GPIOA_BASE,
            .gpioPort = GPIOA,
            .pin = 5,
            .mode = GPIO_ANALOG_MODE,
            .pupDr = NOPULLUP_NOPULLDOWN
        },
        .channel = 5,  // ADC1_IN5 for PA5
        .sampling_time = ADC_SAMPLETIME_84CYCLES,
        .rank = 1
    },
    {
        // Motor A Current Sense on PB0
        .gpio = {
            .port_base_addr = (GPIO_REGS_t*)GPIOB_BASE,
            .gpioPort = GPIOB,
            .pin = 0,
            .mode = GPIO_ANALOG_MODE,
            .pupDr = NOPULLUP_NOPULLDOWN
        },
        .channel = 8,  // ADC1_IN8 for PB0
        .sampling_time = ADC_SAMPLETIME_84CYCLES,
        .rank = 2
    },
    {
        // Motor B Current Sense on PB1
        .gpio = {
            .port_base_addr = (GPIO_REGS_t*)GPIOB_BASE,
            .gpioPort = GPIOB,
            .pin = 1,
            .mode = GPIO_ANALOG_MODE,
            .pupDr = NOPULLUP_NOPULLDOWN
        },
        .channel = 9,  // ADC1_IN9 for PB1
        .sampling_time = ADC_SAMPLETIME_84CYCLES,
        .rank = 3
    }
};

ADC_CONFIG_t adc_port_config = {

    .channels = &system_sensors_configs,
    .Instance = (ADC_TypeDef *)ADC1_BASE,
    .num_channels = NUM_OF_SENSORS,
    .dma_mode = 1,
    .continuous = 1,
    .align = ADC_ALIGN_RIGHT,
    .resolution = ADC_RESOLUTION_12B,
    .ext_trigger = ADC_TRIGGER_NONE,
    .trigger_edge = ADC_TRIGGER_NONE,
};

DMA_HandleTypeDef hdma_adc1  = {

        .Instance = DMA2_Stream0,
        .Init.Channel = DMA_CHANNEL_0,
        .Init.Direction = DMA_PERIPH_TO_MEMORY,
        .Parent = NULL,
        .Init.PeriphInc = DMA_PINC_DISABLE,
        .XferHalfCpltCallback = NULL,
        .Init.MemInc = DMA_MINC_ENABLE,
        .Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD,
        .Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD,
        .Init.Mode = DMA_NORMAL,
        .Init.Priority = DMA_PRIORITY_HIGH,
        .Init.FIFOMode = DMA_FIFOMODE_DISABLE,
        .Init.FIFOThreshold = 0,

};

ADC_PORT_t system_sensors = {

    .port_config = &adc_port_config,
    .dma_channel = &hdma_adc1,
};

// Initialize ADC peripheral and GPIO pins
void ADC_Init(ADC_CONFIG_t *config) {
    // Enable ADC and GPIO clocks
    RCC_REGS->RCC_APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC_REGS->RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    
    // Configure GPIO pins for all channels
    for(int i = 0; i < config->num_channels; i++) {
        GPIO_CONFIG_t *gpio = &config->channels[i].gpio;
        // Set GPIO pin to analog mode
        gpio->port_base_addr->MODER |= (3U << (gpio->pin * 2));
    }
    
    // Configure ADC
    config->Instance->CR1 = 0;  // Reset CR1
    config->Instance->CR2 = 0;  // Reset CR2
    
    // Set ADC resolution
    config->Instance->CR1 |= (config->resolution << ADC_CR1_RES_Pos);
    
    // Configure for regular sequence
    config->Instance->SQR1 = ((config->num_channels - 1) << ADC_SQR1_L_Pos);
    
    // Configure channel sequence and sampling time
    for(int i = 0; i < config->num_channels; i++) {
        // Set conversion sequence
        if(i < 6) {
            config->Instance->SQR3 |= (config->channels[i].channel << (5 * i));
        } else if(i < 12) {
            config->Instance->SQR2 |= (config->channels[i].channel << (5 * (i - 6)));
        }
        
        // Set sampling time
        if(config->channels[i].channel < 10) {
            config->Instance->SMPR2 |= (config->channels[i].sampling_time << 
                                      (3 * config->channels[i].channel));
        } else {
            config->Instance->SMPR1 |= (config->channels[i].sampling_time << 
                                      (3 * (config->channels[i].channel - 10)));
        }
    }
    
    // Enable ADC
    config->Instance->CR2 |= ADC_CR2_ADON;
}
/*
void ADC_Init_With_Timer_DMA(ADC_CONFIG_t *config, TIM_REGS_T *timer) {

    // asign the timer to the ADC config
    config->timer = timer;
    
    // Enable all required peripheral clocks
    RCC_REGS->RCC_APB2ENR |= RCC_APB2ENR_ADC1EN;    // ADC1 clock (ADC1)

    RCC_REGS->RCC_APB1ENR |= RCC_APB1ENR_TIM4EN;     // Timer clock (TIM4)

    RCC_REGS->RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN |   // GPIO clocks
                             RCC_AHB1ENR_GPIOBEN |
                             RCC_AHB1ENR_DMA2EN;      // DMA2 clock

    // Configure GPIO pins for analog mode
    for(int i = 0; i < config->num_channels; i++) {
        GPIO_CONFIG_t *gpio = &config->channels[i].gpio;
        // Set GPIO pin to analog mode (11 in MODER)
        gpio->port_base_addr->MODER |= (3U << (gpio->pin * 2));
    }

    // Configure Timer 4 for precise 10ms triggering
    timer->CR1 = 0;             // Reset CR1
    timer->CR2 = 0;             // Reset CR2 (no need for update event trigger)
    timer->PSC = 84 - 1;        // 1MHz timer clock
    timer->ARR = 10000 - 1;     // 10ms period
    timer->CR1 = TIM_CR1_ARPE;  // Enable auto-reload preload

    // Configure Channel 4 for PWM
    timer->CCMR2 &= ~(TIM_CCMR2_CC4S | TIM_CCMR2_OC4M | 
                    TIM_CCMR2_OC4PE | TIM_CCMR2_OC4FE);
    timer->CCMR2 |= (6U << 12);           // PWM Mode 1
    timer->CCMR2 |= TIM_CCMR2_OC4PE;      // Enable preload
    timer->CCR4 = 10;                      // 10µs pulse
    timer->CCER |= TIM_CCER_CC4E;         // Enable output
                      
    // Configure ADC for timer-triggered operation
    config->Instance->CR1 = ADC_CR1_SCAN |
                            ADC_CR1_EOCIE;  // Enable scan mode for multiple channels
    
    // Set sampling time for all channels (480 cycles for stability)
    // This gives us plenty of time for accurate readings
    for(int i = 0; i < config->num_channels; i++) {
        uint32_t channel = config->channels[i].channel;
        if(channel < 10) {
            config->Instance->SMPR2 |= (7U << (3 * channel));
        } else {
            config->Instance->SMPR1 |= (7U << (3 * (channel - 10)));
        }
    }

    // Set conversion sequence
    config->Instance->SQR1 = ((ADC_BUFFER_SIZE - 1) << ADC_SQR1_L_Pos);  // Sequence length
    config->Instance->SQR3 = (5 << (5 * 0)) |    // Channel 2 as first conversion
                 (8 << (5 * 1)) |    // Channel 8 as second conversion
                 (9 << (5 * 2));     // Channel 9 as third conversion

    config->Instance->CR2 &= ~(0xF << 24);    // Clear EXTSEL bits
    // Configure ADC trigger source
    config->Instance->CR2 = ADC_CR2_EXTEN_0 |  // Rising edge trigger
                            (0x9 << 24) |          // Set EXTSEL to 1001 for TIM4_CH4
                            ADC_CR2_DMA |          // Enable DMA
                            ADC_CR2_DDS |       
                            ADC_CR2_EOCS;  // End of conversion selection
    // Configure DMA
    DMA2_Stream0->CR = 0;
    while(DMA2_Stream0->CR & DMA_SxCR_EN);    // Wait for stream to disable

    DMA2_Stream0->PAR = (uint32_t)&config->Instance->DR;
    DMA2_Stream0->M0AR = (uint32_t)adc_dma_buffer;
    DMA2_Stream0->NDTR = config->num_channels;

    DMA2_Stream0->CR = (0 << DMA_SxCR_CHSEL_Pos) |  // Channel 0
                    DMA_SxCR_PL_1 |                 // High priority
                    DMA_SxCR_MSIZE_0 |              // 16-bit memory
                    DMA_SxCR_PSIZE_0 |              // 16-bit peripheral
                    DMA_SxCR_MINC |                 // Increment memory
                    DMA_SxCR_CIRC |                 // Circular mode
                    DMA_SxCR_TCIE;                 // Transfer complete interrupt

    // Disable FIFO (enable direct mode)
    DMA2_Stream0->FCR |= DMA_SxFCR_DMDIS;

    // Clear any pending DMA flags
    DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | 
                  DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | 
                  DMA_LIFCR_CFEIF0;

    // Enable DMA interrupt
    __NVIC_SetPriority(DMA2_Stream0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);
    __NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    __NVIC_SetPriority(ADC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);
    __NVIC_EnableIRQ(ADC_IRQn);

    // Enable DMA first
    DMA2_Stream0->CR |= DMA_SxCR_EN;

    // enable ADC
    config->Instance->CR2 |= ADC_CR2_ADON;

}

void ADC_Init_With_Timer_DMA(ADC_CONFIG_t *config, TIM_REGS_T *timer) {
    // Enable all required peripheral clocks
    RCC_REGS->RCC_APB2ENR |= RCC_APB2ENR_ADC1EN;    // ADC1 clock
    RCC_REGS->RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN |  // GPIO clocks
                             RCC_AHB1ENR_GPIOBEN |
                             RCC_AHB1ENR_DMA2EN;     // DMA2 clock

    // Configure GPIO pins for analog mode
    for(int i = 0; i < config->num_channels; i++) {
        GPIO_CONFIG_t *gpio = &config->channels[i].gpio;
        gpio->port_base_addr->MODER |= (3U << (gpio->pin * 2));  // Set to analog mode (11)
    }

    // Configure ADC for software triggering
    config->Instance->CR1 = ADC_CR1_SCAN |    // Enable scan mode for multiple channels
                           ADC_CR1_EOCIE;     // Enable end of conversion interrupt
    
    // Set sampling time for all channels (480 cycles for stability)
    for(int i = 0; i < config->num_channels; i++) {
        uint32_t channel = config->channels[i].channel;
        if(channel < 10) {
            config->Instance->SMPR2 |= (7U << (3 * channel));
        } else {
            config->Instance->SMPR1 |= (7U << (3 * (channel - 10)));
        }
    }

    // Configure conversion sequence - 3 channels
    config->Instance->SQR1 = (2 << ADC_SQR1_L_Pos);  // 3 conversions (N-1)
    config->Instance->SQR3 = (5 << (5 * 0)) |        // Channel 5 as first conversion
                            (8 << (5 * 1)) |        // Channel 8 as second conversion
                            (9 << (5 * 2));         // Channel 9 as third conversion

    config->Instance->CR2 &= ~(ADC_CR2_EXTSEL); // Clear EXTSEL bits
    // Configure ADC control - software trigger mode
    config->Instance->CR2 = ADC_CR2_DMA |    // Enable DMA mode
                           ADC_CR2_DDS ;    // DMA requests continue

    // Configure DMA
    DMA2_Stream0->CR = 0;  // Reset control register
    while(DMA2_Stream0->CR & DMA_SxCR_EN);   // Wait for stream to disable

    // Set up DMA transfer parameters
    DMA2_Stream0->PAR = (uint32_t)&config->Instance->DR;
    DMA2_Stream0->M0AR = (uint32_t)adc_dma_buffer;
    DMA2_Stream0->NDTR = 3;  // Three channels to transfer

    // Configure DMA control register
    DMA2_Stream0->CR = (0 << DMA_SxCR_CHSEL_Pos) |  // Channel 0
                       DMA_SxCR_PL_1 |              // High priority
                       DMA_SxCR_MSIZE_0 |           // 16-bit memory
                       DMA_SxCR_PSIZE_0 |           // 16-bit peripheral
                       DMA_SxCR_MINC |              // Increment memory
                       DMA_SxCR_CIRC |              // Circular mode
                       DMA_SxCR_TCIE;               // Transfer complete interrupt

    
   // Disable FIFO (enable direct mode)
    DMA2_Stream0->FCR &= ~DMA_SxFCR_DMDIS;

    // Clear any pending DMA flags
    DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | 
                  DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | 
                  DMA_LIFCR_CFEIF0;

    // Configure interrupts
    __NVIC_SetPriority(DMA2_Stream0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);
    __NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    //__NVIC_SetPriority(ADC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);
    //__NVIC_EnableIRQ(ADC_IRQn);

    // Enable DMA
    DMA2_Stream0->CR |= DMA_SxCR_EN;

    // Enable ADC
    config->Instance->CR2 |= ADC_CR2_ADON;
}*/

void ADC_Init_With_Timer_DMA(ADC_CONFIG_t *config, TIM_REGS_T *timer) {
    // Define buffer size - make it larger for statistics
    #define ADC_BUFFER_SIZE 30  // 10 complete sets of 3 channels
    
    // Enable all required peripheral clocks
    RCC_REGS->RCC_APB2ENR |= RCC_APB2ENR_ADC1EN;    // ADC1 clock
    RCC_REGS->RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN |  // GPIO clocks
                             RCC_AHB1ENR_GPIOBEN |
                             RCC_AHB1ENR_DMA2EN;     // DMA2 clock
    
    // Enable Timer 2 clock
    RCC_REGS->RCC_APB1ENR |= RCC_APB1ENR_TIM2EN;    // Timer 2 clock
    
    // Configure GPIO pins for analog mode
    for(int i = 0; i < config->num_channels; i++) {
        GPIO_CONFIG_t *gpio = &config->channels[i].gpio;
        gpio->port_base_addr->MODER |= (3U << (gpio->pin * 2));  // Set to analog mode (11)
    }
    
    // Reset ADC configuration
    config->Instance->CR1 = 0;
    config->Instance->CR2 = 0;
    
    // Configure ADC for timer triggering
    config->Instance->CR1 = ADC_CR1_SCAN;    // Enable scan mode for multiple channels
    
    // Set sampling time for all channels (slower sampling for stability)
    for(int i = 0; i < config->num_channels; i++) {
        uint32_t channel = config->channels[i].channel;
        if(channel < 10) {
            config->Instance->SMPR2 |= (7U << (3 * channel));  // 480 cycles
        } else {
            config->Instance->SMPR1 |= (7U << (3 * (channel - 10)));
        }
    }
    
    // Configure conversion sequence - 3 channels
    config->Instance->SQR1 = (2 << ADC_SQR1_L_Pos);  // 3 conversions (N-1)
    config->Instance->SQR3 = (5 << (5 * 0)) |        // Channel 5 as first conversion
                            (8 << (5 * 1)) |         // Channel 8 as second conversion
                            (9 << (5 * 2));          // Channel 9 as third conversion
    
    // First ensure we're working with Timer 2
    TIM_REGS_T *timer2 = TIM2;
    
    // Check if Timer 2 is already configured and running
    if ((timer2->CR1 & TIM_CR1_CEN) && (timer2->CR2 & (2 << 4))) {
        DEBUG_PRINT("Timer 2 already configured and running with TRGO");
    } else {
        // Reset Timer 2 configuration
        timer2->CR1 = 0;
        timer2->CR2 = 0;
        
        // Wait a moment after reset
        for(volatile int i = 0; i < 100; i++);
        
        // Calculate timer parameters for 10kHz
        // STM32F401RC: System clock is 84MHz, APB1 timer clock = 42MHz
        // For 10kHz: 42,000,000 / 10,000 = 4200
        timer2->PSC = 41;            // Prescaler = 42 (counting from 0)
        timer2->ARR = 99;            // Auto-reload = 100 (counting from 0): 42MHz/42/100 = 10kHz
        
        // Configure Timer 2 Master Mode to use Update event as TRGO
        // Set MMS bits in CR2 to 010 (Update event as trigger output)
        timer2->CR2 &= ~(7 << 4);    // Clear MMS bits (bits 6-4)
        timer2->CR2 |= (2 << 4);     // Set MMS = 010 (Update event)
        
        // Enable Timer 2
        timer2->CR1 |= TIM_CR1_CEN;
    }
    
    // Configure ADC external trigger - THIS IS THE CRITICAL CHANGE
    config->Instance->CR2 &= ~(0xF << ADC_CR2_EXTSEL_Pos);  // Clear EXTSEL bits
    
    // Set trigger source for Timer 2 TRGO event (0x6 = 0110)
    config->Instance->CR2 |= (0x6 << ADC_CR2_EXTSEL_Pos);   // 0x6 = Timer 2 TRGO event
    config->Instance->CR2 |= ADC_CR2_EXTEN_0;               // Trigger on rising edge
    
    // Verify that EXTSEL value is set correctly - crucial for debugging
    uint32_t extsel_check = (config->Instance->CR2 & (0xF << ADC_CR2_EXTSEL_Pos)) >> ADC_CR2_EXTSEL_Pos;

    
    // Configure ADC control
    config->Instance->CR2 |= ADC_CR2_DMA |    // Enable DMA mode
                           ADC_CR2_DDS;     // DMA requests continue
    
    // Stop DMA if it's currently running
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;  // Disable DMA stream
    while(DMA2_Stream0->CR & DMA_SxCR_EN);   // Wait for stream to disable
    
    // Clear any pending DMA flags BEFORE configuration
    DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 |
                 DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 |
                 DMA_LIFCR_CFEIF0;
    
    // Configure DMA
    DMA2_Stream0->CR = 0;  // Reset control register
    
    // Set up DMA transfer parameters
    DMA2_Stream0->PAR = (uint32_t)&config->Instance->DR;
    DMA2_Stream0->M0AR = (uint32_t)adc_dma_buffer;
    DMA2_Stream0->NDTR = ADC_BUFFER_SIZE;  // Larger buffer for statistics
    
    // Configure DMA control register
    DMA2_Stream0->CR = (0 << DMA_SxCR_CHSEL_Pos) |  // Channel 0
                      DMA_SxCR_PL_1 |              // High priority
                      DMA_SxCR_MSIZE_0 |           // 16-bit memory
                      DMA_SxCR_PSIZE_0 |           // 16-bit peripheral
                      DMA_SxCR_MINC |              // Increment memory
                      DMA_SxCR_CIRC |              // Circular mode
                      DMA_SxCR_TCIE;               // Transfer complete interrupt
   
    // Disable FIFO (enable direct mode)
    DMA2_Stream0->FCR &= ~DMA_SxFCR_DMDIS;
    
    // Configure DMA interrupt with higher priority
    uint32_t dma_priority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2;
    __NVIC_SetPriority(DMA2_Stream0_IRQn, dma_priority);
    __NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    
    // Enable DMA - must be before ADC enable
    DMA2_Stream0->CR |= DMA_SxCR_EN;
    
    // Add a small delay to ensure DMA is ready
    for(volatile int i = 0; i < 1000; i++);
    
    // Enable ADC and ADON must be the last step
    config->Instance->CR2 |= ADC_CR2_ADON;
    
    // Add debugging check to verify ADC is enabled
    if(!(config->Instance->CR2 & ADC_CR2_ADON)) {
        DEBUG_ERROR("ADC not enabled properly");
    }
    
    // Add a small delay after ADC enable
    for(volatile int i = 0; i < 1000; i++);
    
}