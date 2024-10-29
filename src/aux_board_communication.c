#include "spi.h"
#include "gpio.h"
#include "aux_board_communication.h"
#include "platform.h"

void serialize_packet(COMM_Packet *packet, uint8_t *buffer){

    // assert correct params
    if(buffer == NULL || packet == NULL){

        return 0;
    }
    uint16_t index = 0;

    buffer[index++] = START_BYTE;
    buffer[index++] = packet->message_type;
    buffer[index++] = packet->length;

    for(int i = 0; i < packet->length; i++){

        buffer[index] = packet->payload[i];
        index++;
    }


}

uint16_t calculateCRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    while (length--) {
        uint8_t x = crc >> 8 ^ *data++;
        x ^= x >> 4;
        crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x << 5)) ^ ((uint16_t)x);
    }
    return crc;
}

uint16_t calculateChecksum(const COMM_Packet* packet) {

    uint16_t crc = calculateCRC16((uint8_t*)packet, 2 + packet->length);
    return crc;
}

COMM_Packet* interpret_received_data(uint8_t* rxBuffer, uint16_t rxSize) {
    static COMM_Packet packet;  // Static to ensure it persists after function returns

    if (rxSize < sizeof(uint8_t) * 2 + sizeof(uint16_t)) {
        // Not enough data for a valid packet
        return NULL;
    }

    packet.message_type = rxBuffer[0];
    packet.length = rxBuffer[1];

    if (rxSize < packet.length + sizeof(uint8_t) * 2 + sizeof(uint16_t)) {
        // Not enough data for the specified payload length
        return NULL;
    }

    // Copy payload
    memcpy(packet.payload, &rxBuffer[2], packet.length);

    return &packet;
}

void sendPacket(SPI_HandleTypeDef * spiHandle, uint8_t *buffer, uint8_t size){


    /* pull CS low */
    clrGpioOut(&spiHandle->pins[MB_CS]);

    /* send data */
    SPI_Transmit_DMA(spiHandle, buffer, size);
    SPI_TransmitReceive_DMA(spiHandle, spiHandle->pTxBuffPtr, spiHandle->pRxBuffPtr, size);
    //SPI_WRITE_B(spiHandle, &command, 1, 10);

    /* pull CS high
    setGpioOut(&spiHandle->pins[MB_CS]); */
}

void requestRadioControlData(SPI_HandleTypeDef * spiHandle){

    
    COMM_Packet packet = {
        .message_type = REQUEST,
        .length = 1,
        .payload = RADIO_CONTROL_DATA,
    };

    uint8_t transferSize = PACKET_HEADER_SIZE + packet.length + RADIO_CONTROL_DATA_SIZE;

    // allocate memory for dynamic buffer start_byte (1 byte) + message_type (1 byte) + length (1 byte) + dummy Byte = 4 bytes  + payload (0 - 255 bytes)
    uint8_t *txBuffer = (uint8_t *)malloc(sizeof(uint8_t)*(transferSize + 1));
    uint8_t *rxBuffer = (uint8_t *)malloc(sizeof(uint8_t)*(RADIO_CONTROL_DATA_SIZE));

    serialize_packet(&packet, txBuffer);

    // set the last 7 bytes (1 dummy byte + 6 expected bytes) of the txBuffer to 0xFF + 0x00's
    txBuffer[PACKET_HEADER_SIZE + packet.length] = 0xFF;
    memset(&txBuffer[PACKET_HEADER_SIZE + packet.length + 1], 0x00, RADIO_CONTROL_DATA_SIZE);

        /* pull CS low */
    clrGpioOut(&spiHandle->pins[MB_CS]);

    SPI_TransmitReceive_DMA(spiHandle, txBuffer, rxBuffer, transferSize + 1);
    
    //SPI_Transmit_DMA(spiHandle, txBuffer, transferSize);
}

void process_received_packet(SPI_HandleTypeDef * spiHandle){


    //COMM_Packet *packet = interpret_received_data(spiHandle->pRxBuffPtr, spiHandle->RxXferSize);

    asm("nop");
}

void end_tx_rx(SPI_HandleTypeDef * spiHandle){

      /* free dynamically allocated tx buffer */
    free(spiHandle->pTxBuffPtr);
    free(spiHandle->pRxBuffPtr);
    /*
    if(spiHandle->com_state == SPI_REQUEST_RC_DATA){

        delay(1, true);
        uint8_t expectedResponseSize = RADIO_CONTROL_DATA_SIZE;
        uint8_t *txBuffer = (uint8_t *)malloc(sizeof(uint8_t)*(expectedResponseSize));
        uint8_t *rxBuffer = (uint8_t *)malloc(sizeof(uint8_t)*(expectedResponseSize));
        memset(txBuffer, 0x00, expectedResponseSize);

        spiHandle->com_state = WAITING_FOR_RC_DATA;

        clrGpioOut(&spiHandle->pins[MB_CS]);

        SPI_TransmitReceive_DMA(spiHandle, txBuffer, rxBuffer, expectedResponseSize);
        
    }
    else if(spiHandle->com_state == WAITING_FOR_RC_DATA){

        process_received_packet(spiHandle);
        spiHandle->com_state = SPI_COMM_IDLE;
    }
    else{

        spiHandle->com_state = SPI_COMM_IDLE;
    }*/
}
void comErrorCallback(SPI_HandleTypeDef * spiHandle){

    free(spiHandle->pTxBuffPtr);
    free(spiHandle->pRxBuffPtr);
    __asm("nop");
}