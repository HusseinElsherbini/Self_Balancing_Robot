#ifndef AUX_B_COMMUNICATION
#define AUX_B_COMMUNICATION
#include "spi.h"


#define MAX_PAYLOAD_SIZE 255
#define RADIO_CONTROL_DATA_SIZE  6
#define START_BYTE 0x7E
#define PACKET_HEADER_SIZE 3


typedef enum
{
    SENSOR_DATA          = 0x00U,    
    PID_DATA             = 0x01U,   
    SYSTEM_STATUS        = 0x02U,    
    COMMAND              = 0x03U,    
    ACK                  = 0x04U,    
    ERROR                = 0x05U,   
    REQUEST              = 0x06U, 
    RC_DATA              = 0x07U,

} MESSAGE_TYPE;

typedef enum
{
    RADIO_CONTROL_DATA   = 0x01U,
    
}REQUEST_TYPE;

typedef struct {
    int16_t throttle;
    int16_t steering;
    uint8_t aux1;
    uint8_t aux2;
} RadioControlData;

typedef struct {
    uint8_t message_type;
    uint8_t length;
    uint8_t payload[MAX_PAYLOAD_SIZE];
} COMM_Packet;

void end_tx_rx(SPI_HandleTypeDef * spiHandle);
void requestRadioControlData(SPI_HandleTypeDef * spiHandle);
uint16_t calculateCRC16(const uint8_t* data, size_t length) ;
uint16_t calculateChecksum(const COMM_Packet* packet);
void sendPacket(SPI_HandleTypeDef * spiHandle, uint8_t* buffer, uint8_t size);
void serialize_packet(COMM_Packet *packet, uint8_t *buffer);
COMM_Packet* interpret_received_data(uint8_t* rxBuffer, uint16_t rxSize);
void process_received_packet(SPI_HandleTypeDef * spiHandle);
void comErrorCallback(SPI_HandleTypeDef * spiHandle);
#endif /* AUX_B_COMMUNICATION */