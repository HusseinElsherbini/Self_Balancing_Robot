#include "platform.h"
#include "spi.h"
#include "sbus_receiver.h"
#include "uart.h"
#include "task_macros.h"
#include "FreeRTOS.h"
#include "task.h"
#include "smart_port.h"
#include "adc.h"
#include "logger.h"


void ADC_IRQHandler(void)
{
    DEBUG_ISR_ENTER();

    if(system_sensors.port_config->Instance->SR & ADC_SR_EOC) {
        // Clear the EOC flag
        __asm("nop");
    }
    DEBUG_ISR_EXIT();
}
void SPI1_IRQHandler(void){
    
  uint32_t itsource = spiHandle1.Instance->CR2;
  uint32_t itflag   = spiHandle1.Instance->SR;

  /* SPI in mode Receiver ----------------------------------------------------*/
  if ((SPI_CHECK_FLAG(itflag, SPI_FLAG_OVR) == RESET) &&
      (SPI_CHECK_FLAG(itflag, SPI_FLAG_RXNE) != RESET) && (SPI_CHECK_IT_SOURCE(itsource, SPI_IT_RXNE) != RESET))
  {
    spiHandle1.RxISR(&spiHandle1);
    return;
  }

  /* SPI in mode Transmitter -------------------------------------------------*/
  if ((SPI_CHECK_FLAG(itflag, SPI_FLAG_TXE) != RESET) && (SPI_CHECK_IT_SOURCE(itsource, SPI_IT_TXE) != RESET))
  {
    spiHandle1.TxISR(&spiHandle1);
    return;
  }

}			     
void DMA2_Stream0_IRQHandler(void)
{
    DEBUG_ISR_ENTER();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (DMA2->LISR & DMA_LISR_TCIF0) {
        // Clear transfer complete flag
        DMA2->LIFCR = DMA_LIFCR_CTCIF0;

        // Notify task that new ADC data is ready
        xTaskNotifyFromISR(xLogTaskHandle,
                          NOTIFY_LOGGER_TASK_ADC,
                          eSetBits,
                          &xHigherPriorityTaskWoken);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    // Handle any errors
    if (DMA2->LISR & (DMA_LISR_TEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_FEIF0)) {


        // Debugging log
        if (DMA2->LISR & DMA_LISR_TEIF0) {
            DEBUG_ERROR("DMA transfer error detected\n");
        }
        if (DMA2->LISR & DMA_LISR_DMEIF0) {
            DEBUG_ERROR("DMA direct mode error detected\n");
        }
        if (DMA2->LISR & DMA_LISR_FEIF0) {
            DEBUG_ERROR("DMA FIFO error detected\n");
        }        
        // Clear error flags
        DMA2->LIFCR = DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;
    }
    system_sensors.dma_channel->XferCpltCallback(system_sensors.dma_channel);
    DEBUG_ISR_EXIT();
}


/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler(void)
{

  DMA_IRQHandler(&hdma_spi1_tx);

}

// USART6 Interrupt Handler
void USART6_IRQHandler(void) {

    static uint32_t lastPacketTime = 0;  // Time when last valid packet started
    static uint32_t lastByteTime = 0;    // Time of last received byte
    uint32_t currentTime = xTaskGetTickCount();
    volatile BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if(sbusHandle.uartHandle->uart_config.Instance->SR & USART_SR_RXNE) {
        uint8_t data = sbusHandle.uartHandle->uart_config.Instance->DR;
        
        // Check for timeout between bytes (3ms threshold)
        if((currentTime - lastByteTime) > pdMS_TO_TICKS(3)) {
            sbusHandle.state = SBUS_READY;
            sbusHandle.bufferIndex = 0;
        }
        lastByteTime = currentTime;
        
        // If we see a start byte, check if enough time has passed since last packet
        if(data == 0x0F) {
            uint32_t timeSinceLastPacket = currentTime - lastPacketTime;
            
            // If we're close to the expected 6ms packet timing (allowing some variance)
            // OR if it's been too long since last packet (timeout condition)
            if(timeSinceLastPacket > pdMS_TO_TICKS(5) || 
               sbusHandle.state == SBUS_READY) {
                sbusHandle.state = SBUS_START_RECEIVED;
                sbusHandle.bufferIndex = 0;
                sbusHandle.packetBuffer[sbusHandle.bufferIndex++] = data;
                lastPacketTime = currentTime;
                return;  // Exit early as we've handled this byte
            }
            // Otherwise, treat 0x0F as regular data
        }
        
        // Normal packet collection continues here
        if(sbusHandle.state == SBUS_START_RECEIVED) {
            if(sbusHandle.bufferIndex < SBUS_PACKET_SIZE) {
                sbusHandle.packetBuffer[sbusHandle.bufferIndex++] = data;
                
                if(sbusHandle.bufferIndex == SBUS_PACKET_SIZE) {
                    if(sbusHandle.packetBuffer[24] == 0x00) {

                      /*
                        vTaskNotifyGiveIndexedFromISR(xRcTaskHandle, NOTIFY_SBUS_DATA,
                                             &xHigherPriorityTaskWoken);*/
                        xTaskNotifyFromISR(xRcTaskHandle,
                                        NOTIFY_SBUS_DATA,
                                        eSetBits,
                                        &xHigherPriorityTaskWoken);
                        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                    }
                    sbusHandle.state = SBUS_READY;
                }
            } 
            else {
                sbusHandle.state = SBUS_READY;
            }
        }
    }
    
    
}

// USART2 (SmartPort) interrupt handler
void USART2_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    

    if(smart_port_handle.uartHandle->uart_config.Instance->SR & USART_SR_RXNE) {
        uint8_t received = smart_port_handle.uartHandle->uart_config.Instance->DR;
        
        // State machine for receiving SmartPort poll request
        if(received == SPORT_START_POLL) {
            // Start of new poll request
            smart_port_handle.rx_buffer[0] = received;
            smart_port_handle.rx_index = 1;
        }
        else if(smart_port_handle.rx_index == 1) {
            // Got sensor ID after poll byte
            smart_port_handle.rx_buffer[1] = received;
            
            // Only now do we wake the task - we have a complete poll request
            xTaskNotifyFromISR(xRcTaskHandle,
                              NOTIFY_SPORT_POLL,
                              eSetBits,
                              &xHigherPriorityTaskWoken);
            
            smart_port_handle.rx_index = 0;  // Reset for next poll
            
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    
    
}