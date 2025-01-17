#include "sbus_receiver.h"
#include "stdbool.h"
#include "uart.h"
#include "FreeRTOSConfig.h"
/*
 *  FrSky Taranis X9D Radio Controller
 *  SBUS Protocol Implementation
 *
 *                      .------------------.
 *                     /  .------------.   \
 *                    /  /     LCD     \   \
 *                   /  /              \   \
 *    SA  SB  SC   |  |   [MENU]      |   |   SD  SE  SF
 *    ||  ||  ||   |  |     ^         |   |   ||  ||  ||
 *    ||  ||  ||   |  |  <  +  >      |   |   ||  ||  ||
 *    ||  ||  ||   |  |     v         |   |   ||  ||  ||
 *                  |  |   [PAGE]      |   |
 *                  |  |              |   |
 *     < T1 >      |   \            /    |     < T2 >     
 *    .-==-.       |    \__________/     |    .-==-.
 *    |    |       |                     |    |    |
 *    |    |       |                     |    |    |
 *    |    |       |        (+)         |     |    |
 *    |    |       |     <       >      |     |    |
 *    |    |       |        (-)         |     |    |
 *    '====.      /                      \    .====='
 *          \    /     .---------,       \  /
 *           \  /     /           \       \/
 *            \/     /             \     /\
 *            /\    /               \   /  \
 *           /  \   '---------------'  /    \
 *          /    \                   /      \
 *         '------'                 '--------'
 *           Left                    Right
 *           Stick                   Stick
 *
 * SBUS Protocol:
 * - 100K baud
 * - 8 data bits
 * - Even parity
 * - 2 stop bits
 * - Inverted serial signal
 * 
 * - Right stick (Chennels: "left/right" = 1 "Up/Down" = 2)
 * - Left stick  (Chennels: "left/right" = 3 "Up/Down" = 0)
 */
SBUS_HandleTypeDef sbusHandle = {

    .packet = {
        .rawData = {0},
        .channels = {0},
        .flags = {0},
        .lastUpdate = 0
    },
    .packetBuffer = {0},
    .bufferIndex = 0,
    .state = SBUS_READY
};

UART_HandleTypeDef sbusUart = {
    .uart_config.rx.gpioPort = GPIOA,
    .uart_config.rx.pin = 12U,
    .uart_config.rx.port_base_addr = GPIOA_PERIPH_BASE,
    .uart_config.baudRate = 100000,
    .uart_config.dataBits = UART_WORDLENGTH_9B,
    .uart_config.Instance = USART6_BASE,
    .uart_config.stopBits = UART_STOPBITS_2,
    .uart_config.mode = UART_MODE_RX,
    .uart_config.tx = NULL,
};

void SBUS_init(SBUS_HandleTypeDef *sbusHandle , UART_HandleTypeDef *uartHandle) {
    
    // Save UART handle
    sbusHandle->uartHandle = uartHandle;

    // Enable GPIOA and USART6 clocks
    enableBusToGpioPort(uartHandle->uart_config.rx.gpioPort); // Enable GPIO clock 
    SET_BIT(RCC_REGS->RCC_APB2ENR, RCC_APB2ENR_USART6EN);   // USART6 is on APB2
    
    // Configure gpio (USART6_RX) as alternate function
    setGpioMode(&uartHandle->uart_config.rx, uartHandle->uart_config.rx.pin, GPIO_AF_MODE);
    
    // Set pull-up on RX pin (optional, can help with noise)
    setGpioPupDR(&uartHandle->uart_config.rx, uartHandle->uart_config.rx.pin, PULL_UP);    
    
    // Set alternate function 8 (USART6) for PA12
    setGpioAlternateFunction(&uartHandle->uart_config.rx, uartHandle->uart_config.rx.pin, 8);
    
    // Configure UART
    // For 100K baud with 84MHz clock:
    // 84000000/(16*100000) = 52.5
    // Use mantissa = 52, fraction = 8 (0.5 * 16)

    uartHandle->uart_config.Instance->BRR = (52 << 4) | 8;

    // Configure for SBUS with complete signal inversion

    uartHandle->uart_config.Instance->CR2 = UART_STOPBITS_2;       // RX signal inversion 2 stop bits

    uartHandle->uart_config.Instance->CR1 = USART_CR1_PCE |       // Parity control enable
                  USART_CR1_M |          // 9 data bits (8 + parity)
                  USART_CR1_PS |         // Even parity
                  USART_CR1_RE |         // Receiver enable
                  USART_CR1_RXNEIE;      // Enable RX interrupt

    
    // Enable UART
    uartHandle->uart_config.Instance->CR1 |= USART_CR1_UE;
    
    // Enable USART6 interrupt in NVIC
    __NVIC_EnableIRQ(USART6_IRQn);
    __NVIC_SetPriority(USART6_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);    // Priortiy of 6 (lower than FreeRTOS and IMU)
}


bool SBUS_decode(SBUS_Packet_t* packet) {
    // Verify header (0x0F) and footer (0x00)
    if (packet->rawData[0] != SBUS_HEADER || packet->rawData[24] != SBUS_FOOTER) {
        return false;
    }
   
    // First 8 channels - these are correct and include main controls
    packet->channels[0]  = ((packet->rawData[1]    |packet->rawData[2]<<8)                     & 0x07FF);       // Aileron Left stick UP/DOWN
    packet->channels[1]  = ((packet->rawData[2]>>3 |packet->rawData[3]<<5)                     & 0x07FF);       // Elevator Right stick Left/Right
    packet->channels[2]  = ((packet->rawData[3]>>6 |packet->rawData[4]<<2 |packet->rawData[5]<<10)  & 0x07FF);  // Elevator Right stick UP/DOWN
    packet->channels[3]  = ((packet->rawData[5]>>1 |packet->rawData[6]<<7)                     & 0x07FF);       // Rudder Left stick Left/Right
    packet->channels[4]  = ((packet->rawData[6]>>4 |packet->rawData[7]<<4)                     & 0x07FF);       // SA
    packet->channels[5]  = ((packet->rawData[7]>>7 |packet->rawData[8]<<1 |packet->rawData[9]<<9)   & 0x07FF);  // SB  (PID SWITCH (P = 172 I = 992 D = 1811))
    packet->channels[6]  = ((packet->rawData[9]>>2 |packet->rawData[10]<<6)                    & 0x07FF);       // SC
    packet->channels[7]  = ((packet->rawData[10]>>5|packet->rawData[11]<<3)                    & 0x07FF);       // SD
    
    // Corrected channels 8-15 with proper bit shifting
    packet->channels[8]  = ((packet->rawData[12]   |packet->rawData[13]<<8)                    & 0x07FF);       // SE
    packet->channels[9]  = ((packet->rawData[13]>>3|packet->rawData[14]<<5)                    & 0x07FF);       // SF
    packet->channels[10] = ((packet->rawData[14]>>6|packet->rawData[15]<<2|packet->rawData[16]<<10) & 0x07FF);  // S1 (PID GAIN 172 - 1811)
    packet->channels[11] = ((packet->rawData[16]>>1|packet->rawData[17]<<7)                    & 0x07FF);       // S2 
    packet->channels[12] = ((packet->rawData[17]>>4|packet->rawData[18]<<4)                    & 0x07FF);       // Auxiliary channel
    packet->channels[13] = ((packet->rawData[18]>>7|packet->rawData[19]<<1|packet->rawData[20]<<9)  & 0x07FF);  // Auxiliary channel
    packet->channels[14] = ((packet->rawData[20]>>2|packet->rawData[21]<<6)                    & 0x07FF);       // Auxiliary channel
    packet->channels[15] = ((packet->rawData[21]>>5|packet->rawData[22]<<3)                    & 0x07FF);       // Auxiliary channel
   
    // Decode flags from byte 23 - these indicate special states
    uint8_t flagByte = packet->rawData[23];
    packet->flags.ch17     = (flagByte & (1 << 7)) != 0;
    packet->flags.ch18     = (flagByte & (1 << 6)) != 0;
    packet->flags.frameLost = (flagByte & (1 << 5)) != 0;
    packet->flags.failsafe  = (flagByte & (1 << 4)) != 0;
   
    return true;
}

// Helper function to map SBUS values (0-2047) to a custom range
int16_t SBUS_scaleChannel(uint16_t channelValue, int16_t minOutput, int16_t maxOutput) {
    // SBUS typical range is 172-1811
    const uint16_t SBUS_MIN = 172;
    const uint16_t SBUS_MAX = 1811;
    
    // Constrain input
    if (channelValue < SBUS_MIN) channelValue = SBUS_MIN;
    if (channelValue > SBUS_MAX) channelValue = SBUS_MAX;
    
    // Map to output range
    return (int16_t)(((int32_t)(channelValue - SBUS_MIN) * (maxOutput - minOutput)) / 
                     (SBUS_MAX - SBUS_MIN) + minOutput);
}

void print_sbus_data(SBUS_Packet_t* packet)
{
    SEGGER_RTT_printf(0, "\n=== SBUS Data ===\n");
    
    // Print stick channels (1-4)
    SEGGER_RTT_printf(0, "Sticks:\n");
    SEGGER_RTT_printf(0, "  Aileron  (CH1): %d\n", packet->channels[0]);
    SEGGER_RTT_printf(0, "  Elevator (CH2): %d\n", packet->channels[1]);
    SEGGER_RTT_printf(0, "  Throttle (CH3): %d\n", packet->channels[2]);
    SEGGER_RTT_printf(0, "  Rudder   (CH4): %d\n", packet->channels[3]);
    
    // Print switch channels (5-10)
    SEGGER_RTT_printf(0, "Switches:\n");
    SEGGER_RTT_printf(0, "  SA (CH5): %d\n", packet->channels[4]);
    SEGGER_RTT_printf(0, "  SB (CH6): %d\n", packet->channels[5]);
    SEGGER_RTT_printf(0, "  SC (CH7): %d\n", packet->channels[6]);
    SEGGER_RTT_printf(0, "  SD (CH8): %d\n", packet->channels[7]);
    SEGGER_RTT_printf(0, "  SE (CH9): %d\n", packet->channels[8]);
    SEGGER_RTT_printf(0, "  SF (CH10): %d\n", packet->channels[9]);
    
    // Print auxiliary channels (11-16)
    SEGGER_RTT_printf(0, "Auxiliary:\n");
    SEGGER_RTT_printf(0, "  CH11: %d\n", packet->channels[10]);
    SEGGER_RTT_printf(0, "  CH12: %d\n", packet->channels[11]);
    SEGGER_RTT_printf(0, "  CH13: %d\n", packet->channels[12]);
    SEGGER_RTT_printf(0, "  CH14: %d\n", packet->channels[13]);
    SEGGER_RTT_printf(0, "  CH15: %d\n", packet->channels[14]);
    SEGGER_RTT_printf(0, "  CH16: %d\n", packet->channels[15]);
    
    // Print digital channels and flags
    SEGGER_RTT_printf(0, "Flags:\n");
    SEGGER_RTT_printf(0, "  CH17: %d\n", packet->flags.ch17);
    SEGGER_RTT_printf(0, "  CH18: %d\n", packet->flags.ch18);
    SEGGER_RTT_printf(0, "  Frame Lost: %d\n", packet->flags.frameLost);
    SEGGER_RTT_printf(0, "  Failsafe: %d\n", packet->flags.failsafe);
}