#include "smart_port.h"
#include "gpio.h"
#include "platform.h"
#include "FreeRTOSConfig.h"

UART_HandleTypeDef smartPortUart = {
    .uart_config.rx.gpioPort = GPIOA,
    .uart_config.rx.pin = 2U,
    .uart_config.rx.port_base_addr = GPIOA_PERIPH_BASE,
    .uart_config.tx.gpioPort = GPIOA,
    .uart_config.tx.pin = 3U,
    .uart_config.tx.port_base_addr = GPIOA_PERIPH_BASE,
    .uart_config.baudRate = 56700,
    .uart_config.dataBits = UART_WORDLENGTH_8B,
    .uart_config.Instance = USART2,
    .uart_config.stopBits = UART_STOPBITS_1,
    .uart_config.mode = UART_MODE_TX_RX,
};
SmartPort_HandleTypeDef smart_port_handle  = {

    .uartHandle = &smartPortUart,

};
// Initialize SmartPort on specified UART
void SmartPort_Init(SmartPort_HandleTypeDef *smartportHandle, UART_HandleTypeDef *uartHandle) {
    // Enable GPIOA and USART2 clocks
    enableBusToGpioPort(uartHandle->uart_config.tx.gpioPort);  // For PA2
    enableBusToGpioPort(uartHandle->uart_config.rx.gpioPort);  // For PA3
    SET_BIT(RCC_REGS->RCC_APB1ENR, RCC_APB1ENR_USART2EN);
    
    // Configure PA2 (TX) and PA3 (RX) as alternate function
    setGpioMode(&uartHandle->uart_config.tx, uartHandle->uart_config.tx.pin, GPIO_AF_MODE);
    setGpioMode(&uartHandle->uart_config.rx, uartHandle->uart_config.rx.pin, GPIO_AF_MODE);
    
    // Set alternate function 7 (USART2)
    setGpioAlternateFunction(&uartHandle->uart_config.tx, uartHandle->uart_config.tx.pin, 7);
    setGpioAlternateFunction(&uartHandle->uart_config.rx, uartHandle->uart_config.rx.pin, 7);
    
    // Configure UART for SmartPort
    // 57600 baud with 42MHz APB1 clock
    // 42000000/(16*57600) = 729.16667
    // Mantissa = 729, Fraction = 3 (0.16667 * 16)
    uartHandle->uart_config.Instance->BRR = (729 << 4) | 3;
    
    // Enable TX, RX, and RX interrupt
    uartHandle->uart_config.Instance->CR1 = USART_CR1_TE |      // Transmitter enable
                                           USART_CR1_RE |      // Receiver enable
                                           USART_CR1_RXNEIE;   // RX interrupt enable
    
    // Standard UART settings (8N1)
    uartHandle->uart_config.Instance->CR2 = 0;  // 1 stop bit
    
    // Enable UART
    uartHandle->uart_config.Instance->CR1 |= USART_CR1_UE;
    
    // Enable USART2 interrupt
    __NVIC_EnableIRQ(USART2_IRQn);
    __NVIC_SetPriority(USART2_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);

}

// Helper function to check if enough time has passed between transmissions
bool SmartPort_CanTransmit(SmartPort_HandleTypeDef *smartportHandle, uint32_t current_time) {
    // SmartPort requires at least 1ms between packets
    return (current_time - smartportHandle->last_tx_time) >= 1;
}

// Function to send telemetry data
SmartPort_StatusTypeDef SmartPort_SendData(SmartPort_HandleTypeDef *smartportHandle, 
                                          uint16_t sensorId, 
                                          uint32_t value) {
    if (smartportHandle == NULL) {
        return SMARTPORT_ERROR_NULL_HANDLE;
    }
    
    // Check if UART is ready for transmission
    if (!(smartportHandle->uartHandle->uart_config.Instance->SR & USART_SR_TC)) {
        return SMARTPORT_ERROR_UART;
    }
    
    // Frame format:
    // 0x7E (start), sensor ID (2 bytes), value (4 bytes), CRC (1 byte)
    smartportHandle->tx_buffer[0] = 0x7E;
    smartportHandle->tx_buffer[1] = sensorId & 0xFF;
    smartportHandle->tx_buffer[2] = (sensorId >> 8) & 0xFF;
    smartportHandle->tx_buffer[3] = value & 0xFF;
    smartportHandle->tx_buffer[4] = (value >> 8) & 0xFF;
    smartportHandle->tx_buffer[5] = (value >> 16) & 0xFF;
    smartportHandle->tx_buffer[6] = (value >> 24) & 0xFF;
    
    // Calculate CRC
    uint8_t crc = 0;
    for(int i = 1; i < 7; i++) {
        crc ^= smartportHandle->tx_buffer[i];
    }
    smartportHandle->tx_buffer[7] = crc;
    
    // Start transmission
    smartportHandle->tx_index = 0;
    smartportHandle->uartHandle->uart_config.Instance->DR = smartportHandle->tx_buffer[0];
    
    return SMARTPORT_OK;
}
