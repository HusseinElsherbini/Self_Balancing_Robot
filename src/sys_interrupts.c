#include "platform.h"
#include "spi.h"



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

  DMA_IRQHandler(&hdma_spi1_rx);

}

/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler(void)
{

  DMA_IRQHandler(&hdma_spi1_tx);

}
