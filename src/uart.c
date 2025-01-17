#include "uart.h"
#include "sbus_receiver.h"
#include "stdint.h"
#include "platform.h"


void UART_Init(UART_HandleTypeDef *xUartHandle) {

    // Enable GPIOA and USART6 clocks
    SET_BIT(RCC_REGS->RCC_AHB1ENR, GPIOA_EN);    // Enable GPIOA clock
    SET_BIT(RCC_REGS->RCC_APB2ENR, RCC_APB2ENR_USART6EN);   // USART6 is on APB2
    
    // Configure PA12 (USART6_RX) as alternate function

    xUartHandle->uart_config->rx->MODER &= ~GPIO_MODER_MODER12;     // Clear bits
    
    xUartHandle->uart_config->rx->MODER &= ~GPIO_MODER_MODER12;     // Clear bits
    GPIOA->MODER |= GPIO_MODER_MODER12_1;    // Set to alternate function
    
    // Set pull-up on RX pin (optional, can help with noise)
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR12;     // Clear bits
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR12_0;    // Set pull-up
    
    // Set alternate function 8 (USART6) for PA12
    GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL12;    // Clear AF bits for PA12
    GPIOA->AFR[1] |= (8U << GPIO_AFRH_AFSEL12_Pos);  // Set AF8
    
    // Configure UART
    // For 100K baud with 84MHz clock:
    // 84000000/(16*100000) = 52.5
    // Use mantissa = 52, fraction = 8 (0.5 * 16)
    USART6->BRR = (52 << 4) | 8;
    
    // Configure for SBUS with complete signal inversion
    USART6->CR2 = USART_CR2_STOP_1 |    // 2 stop bits
                  USART_CR2_RXINV |      // RX signal inversion

    
    USART6->CR1 = USART_CR1_PCE |       // Parity control enable
                  USART_CR1_M |          // 9 data bits (8 + parity)
                  USART_CR1_PS |         // Even parity
                  USART_CR1_RE |         // Receiver enable
                  USART_CR1_RXNEIE;      // Enable RX interrupt
    
    // Enable UART
    USART6->CR1 |= USART_CR1_UE;
    
    // Enable USART6 interrupt in NVIC
    NVIC_EnableIRQ(USART6_IRQn);
    NVIC_SetPriority(USART6_IRQn, 0);    // High priority
}

