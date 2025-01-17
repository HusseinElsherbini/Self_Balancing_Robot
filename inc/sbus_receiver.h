#ifndef SBUS_RECEIVER_H
#define SBUS_RECEIVER_H

#include "stdint.h"
#include "stdbool.h"
#include "uart.h"

// SBUS Frame Format
// Total length: 25 bytes
// 
// Byte[0]    = SBUS Header (0x0F)
// Byte[1-22] = 16 channels of 11 bit data
// Byte[23]   = Flags byte
//              bit7 = CH17 (digital)
//              bit6 = CH18 (digital)
//              bit5 = Frame lost
//              bit4 = Failsafe activated
// Byte[24]   = SBUS Footer (0x00)

// Channel data bit arrangement (22 bytes contain 16 11-bit values):
// Each channel uses 11 bits. Channels are packed across byte boundaries.
// Example first three channels layout:
// Byte1: 11111111  Byte2: 000CCCCC  Byte3: CCBBBBBB  Byte4: BBAAAAAAA
// Where A = channel 1, B = channel 2, C = channel 3
/*
SBUS Values:     172 -------- 992 -------- 1811
                 ↓            ↓             ↓
Motor Values: -65535 ------- 0 -------- +65535
*/
#define SBUS_PACKET_SIZE 25
#define SBUS_NUM_CHANNELS 16
#define SBUS_HEADER 0x0F
#define SBUS_FOOTER 0x00

#define SIGNAL_TIMEOUT_MS 50 // Timeout for signal loss

typedef struct {
    // Raw packet data
    uint8_t rawData[SBUS_PACKET_SIZE];
    
    // Decoded channel values (11-bit values: 0-2047)
    uint16_t channels[SBUS_NUM_CHANNELS];
    
    // Status flags
    struct {
        bool ch17;            // Digital channel 17
        bool ch18;            // Digital channel 18
        bool frameLost;       // Frame lost flag
        bool failsafe;        // Failsafe activated
    } flags;
    
    // Timestamp of last update (if needed)
    uint32_t lastUpdate;
} SBUS_Packet_t;

typedef enum
{
  SBUS_READY               = 0U,
  SBUS_START_RECEIVED      = 1U,   

} SBUS_RECEIVER_STATE;


typedef struct {
    // UART handle for SBUS
    UART_HandleTypeDef *uartHandle;
    
    // SBUS packet
    SBUS_Packet_t packet;
    
    // SBUS packet buffer
    uint8_t packetBuffer[SBUS_PACKET_SIZE];
    
    // SBUS packet buffer index
    uint8_t bufferIndex;

    SBUS_RECEIVER_STATE state;
    
} SBUS_HandleTypeDef;

extern SBUS_HandleTypeDef sbusHandle;
extern UART_HandleTypeDef sbusUart;


void SBUS_init(SBUS_HandleTypeDef *sbusHandle , UART_HandleTypeDef *uartHandle);
int16_t SBUS_scaleChannel(uint16_t channelValue, int16_t minOutput, int16_t maxOutput);
bool SBUS_decode(SBUS_Packet_t* packet);
void print_sbus_data(SBUS_Packet_t* packet);

#endif // SBUS_RECEIVER_H