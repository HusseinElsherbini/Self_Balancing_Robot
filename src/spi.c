#include "spi.h"
#include "gpio.h"
#include "platform.h"
#include "dma.h"
#include "main.h"
#include "aux_board_communication.h"
#include "FreeRTOS.h"
#include "task.h"
/*
MB_SPI_SCK ----> PA5
MB_SPI_MISO ---> PA6
MB_SPI_MOSI ---> PA7
MB_SPI_CS -----> PC5
*/


DMA_HandleTypeDef hdma_spi1_rx = {

    .Instance = DMA2_Stream0,
    .Init.Channel = DMA_CHANNEL_3,
    .Init.Direction = DMA_PERIPH_TO_MEMORY,
    .Parent = &spiHandle1,
    .Init.PeriphInc = DMA_PINC_DISABLE,
    .XferHalfCpltCallback = NULL,
    .Init.MemInc = DMA_MINC_ENABLE,
    .Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
    .Init.MemDataAlignment = DMA_MDATAALIGN_BYTE,
    .Init.Mode = DMA_NORMAL,
    .Init.Priority = DMA_PRIORITY_HIGH,
    .Init.FIFOMode = DMA_FIFOMODE_DISABLE

};

DMA_HandleTypeDef hdma_spi1_tx  = {

    .Instance = DMA2_Stream3,
    .Init.Channel = DMA_CHANNEL_3,
    .Init.Direction = DMA_MEMORY_TO_PERIPH,
    .Parent = &spiHandle1,
    .Init.PeriphInc = DMA_PINC_DISABLE,
    .Init.MemInc = DMA_MINC_ENABLE,
    .XferHalfCpltCallback = NULL,
    .Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
    .Init.MemDataAlignment = DMA_MDATAALIGN_BYTE,
    .Init.Mode = DMA_NORMAL,
    .Init.Priority = DMA_PRIORITY_MEDIUM,
    .Init.FIFOMode = DMA_FIFOMODE_DISABLE

};
SPI_HandleTypeDef spiHandle1 = {
    .Instance = SPI1,
    .chip_select_present = true,
    .Init.Mode = SPI_MODE_MASTER,
    .Init.Direction = SPI_DIRECTION_2LINES,
    .Init.DataSize = SPI_DATASIZE_8BIT,
    .Init.CLKPolarity = SPI_POLARITY_LOW,
    .Init.CLKPhase = SPI_PHASE_1EDGE,
    .Init.NSS = SPI_NSS_SOFT,
    .Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64,
    .Init.FirstBit = SPI_FIRSTBIT_MSB,
    .Init.TIMode = SPI_TIMODE_DISABLE,
    .Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE,
    .Init.CRCPolynomial = 10,
    .hdmarx = &hdma_spi1_rx,
    .hdmatx = &hdma_spi1_tx,
    .ErrorCallback =  comErrorCallback,
    .TxRxCpltCallback = end_tx_rx,
    .TxCpltCallback = end_tx_rx,
    .pins[MB_MOSI].port_base_addr = (GPIO_REGS_t *)GPIOA_PERIPH_BASE,
    .pins[MB_MOSI].gpioPort = GPIOA,
    .pins[MB_MOSI].pin = 7U,
    .pins[MB_MISO].port_base_addr = (GPIO_REGS_t *)GPIOA_PERIPH_BASE,
    .pins[MB_MISO].gpioPort = GPIOA,
    .pins[MB_MISO].pin = 6U,
    .pins[MB_CLK].port_base_addr = (GPIO_REGS_t *)GPIOA_PERIPH_BASE,
    .pins[MB_CLK].gpioPort  = GPIOA,
    .pins[MB_CLK].pin  = 5U,
    .pins[MB_CS].port_base_addr = (GPIO_REGS_t *)GPIOC_PERIPH_BASE,
    .pins[MB_CS].gpioPort   = GPIOC,
    .pins[MB_CS].pin   = 5U,
};

void spi_init(SPI_HandleTypeDef * spiHandle){


    /* initi low level hardware */
    // MOSI
    enableBusToGpioPort(GPIOA);
    setGpioMode(&spiHandle->pins[MB_MOSI], spiHandle->pins[MB_MOSI].pin, GPIO_AF_MODE);
    setGpioAlternateFunction(&spiHandle->pins[MB_MOSI], spiHandle->pins[MB_MOSI].pin, AF5);
    setGpioPupDR(&spiHandle->pins[MB_MOSI], spiHandle->pins[MB_MOSI].pin, NOPULLUP_NOPULLDOWN);
    setGpioSpeed(&spiHandle->pins[MB_MOSI], spiHandle->pins[MB_MOSI].pin, MAX_SPEED);

    // MISO
    enableBusToGpioPort(GPIOA);
    setGpioMode(&spiHandle->pins[MB_MISO], spiHandle->pins[MB_MISO].pin, GPIO_AF_MODE);
    setGpioAlternateFunction(&spiHandle->pins[MB_MISO], spiHandle->pins[MB_MISO].pin, AF5);
    setGpioPupDR(&spiHandle->pins[MB_MISO], spiHandle->pins[MB_MISO].pin, NOPULLUP_NOPULLDOWN);
    setGpioSpeed(&spiHandle->pins[MB_MISO], spiHandle->pins[MB_MISO].pin, MAX_SPEED);

    // CLK
    enableBusToGpioPort(GPIOA);
    setGpioMode(&spiHandle->pins[MB_CLK], spiHandle->pins[MB_CLK].pin, GPIO_AF_MODE);
    setGpioAlternateFunction(&spiHandle->pins[MB_CLK], spiHandle->pins[MB_CLK].pin, AF5);
    setGpioPupDR(&spiHandle->pins[MB_CLK], spiHandle->pins[MB_CLK].pin, NOPULLUP_NOPULLDOWN);
    setGpioSpeed(&spiHandle->pins[MB_CLK], spiHandle->pins[MB_CLK].pin, MAX_SPEED);

    // CS
    enableBusToGpioPort(GPIOC);
    setGpioMode(&spiHandle->pins[MB_CS], spiHandle->pins[MB_CS].pin, GPIO_OUTPUT_MODE);
    setGpioOutput(&spiHandle->pins[MB_CS], spiHandle->pins[MB_CS].pin, GPIO_OUTPUT_PUSH_PULL);
    setGpioSpeed(&spiHandle->pins[MB_CS], spiHandle->pins[MB_CS].pin, MAX_SPEED);
    setGpioOut(&spiHandle->pins[MB_CS]);

    /* enable APB2 bus to SPI1*/
    if(spiHandle->Instance == SPI1){
        SET_BIT(RCC_REGS->RCC_APB2ENR, RCC_APB2ENR_SPI1EN);
    }
    
    uint8_t direction = spiHandle->Init.Direction & (SPI_CR1_RXONLY | SPI_CR1_BIDIMODE);
    WRITE_REG(spiHandle->Instance->CR1, ((spiHandle->Init.Mode & (SPI_CR1_MSTR | SPI_CR1_SSI)) |
                                    (spiHandle->Init.Direction & (SPI_CR1_RXONLY | SPI_CR1_BIDIMODE)) |
                                    (spiHandle->Init.DataSize & SPI_CR1_DFF) |
                                    (spiHandle->Init.CLKPolarity & SPI_CR1_CPOL) |
                                    (spiHandle->Init.CLKPhase & SPI_CR1_CPHA) |
                                    (spiHandle->Init.NSS & SPI_CR1_SSM) |
                                    (spiHandle->Init.BaudRatePrescaler & SPI_CR1_BR_Msk) |
                                    (spiHandle->Init.FirstBit  & SPI_CR1_LSBFIRST) |
                                    (spiHandle->Init.CRCCalculation & SPI_CR1_CRCEN)));

    spiHandle->State = SPI_STATE_READY;

    RCC_DMA2_CLK_ENABLE();

    __NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    __NVIC_EnableIRQ(DMA2_Stream3_IRQn);   

    if(DMA_Init(spiHandle->hdmarx) != SYS_OK){
        Error_Handler();
    }

    if(DMA_Init(spiHandle->hdmatx) != SYS_OK){
        Error_Handler();
    }
    /* Enable interrupt sources */
    //WRITE_REG(spiHandle->Instance->CR2, SPI_CR2_RXNEIE | SPI_CR2_TXEIE | SPI_CR2_ERRIE);  

    /* enable NVIC interrupt */
    //NVIC_ENABLE_IRQ(SPI1_IRQn);

    SET_BIT(spiHandle->Instance->CR1, SPI_CR1_SPE);

}

void SPI_WRITE_B(SPI_HandleTypeDef * spiHandle, uint8_t *buff, uint8_t size, uint8_t timeout){


    uint8_t errorCode = SYS_OK;
    uint16_t initialTxferCount = size;
    TickType_t tickstart = xTaskGetTickCount();

    /* Set the transaction information */
    spiHandle->State       = SPI_STATE_BUSY_TX;
    spiHandle->ErrorCode   = SPI_ERROR_NONE;
    spiHandle->pTxBuffPtr  = (uint8_t *)buff;
    spiHandle->TxXferSize  = size;
    spiHandle->TxXferCount = size;

    /*Init field not used in handle to zero */
    spiHandle->pRxBuffPtr  = (uint8_t *)NULL;
    spiHandle->RxXferSize  = 0U;
    spiHandle->RxXferCount = 0U;
    spiHandle->TxISR       = NULL;
    spiHandle->RxISR       = NULL;

    if(spiHandle->chip_select_present){
            /* pull CS low  if present */
        clrGpioOut(&spiHandle->pins[MB_CS]); 
    }


    if ((spiHandle->Init.Mode == SPI_MODE_SLAVE) || (initialTxferCount == 0x01U))
    {
        *((__IO uint8_t *)&spiHandle->Instance->DR) = (*spiHandle->pTxBuffPtr);
        spiHandle->pTxBuffPtr += sizeof(uint8_t);
        spiHandle->TxXferCount--;
    }
    while (spiHandle->TxXferCount > 0U)
    {
        /* Wait until TXE flag is set to send data */
        if (SPI_GET_FLAG(spiHandle, SPI_FLAG_TXE))
        {
            *((__IO uint8_t *)&spiHandle->Instance->DR) = (*spiHandle->pTxBuffPtr);
            spiHandle->pTxBuffPtr += sizeof(uint8_t);
            spiHandle->TxXferCount--;
        }
        else
        {    
            /* Timeout management */
            if (((xTaskGetTickCount() - tickstart) >=  timeout) || (timeout == 0U))
            {
                errorCode = SYS_TIMEOUT;

                goto error;
            }
        }
    }
    /* wait until busy flag is reset and end transaction */
    while ((SPI_GET_FLAG(spiHandle, SPI_FLAG_BSY) ? SET : RESET) != RESET){

        if (((xTaskGetTickCount() - tickstart) >=  timeout) || (timeout == 0U)){

            SET_BIT(spiHandle->ErrorCode, SPI_ERROR_FLAG);
            errorCode = SYS_TIMEOUT;
            goto error;     
        }

    }
    error:
    /* clear overrun flag */
    SPI_CLEAR_OVRFLAG(spiHandle);
    spiHandle->State  = SPI_STATE_READY;
    _UNLOCK(spiHandle);

    if(spiHandle->chip_select_present){
          
    /* pull CS high if cs present */
        setGpioOut(&spiHandle->pins[MB_CS]);
    }
    return errorCode;
}

StatusTypeDef SPI_TransmitReceive_DMA(SPI_HandleTypeDef *spiHandle, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
  uint32_t             tmp_mode;
  SPI_StateTypeDef tmp_state;
  StatusTypeDef errorcode = SYS_OK;


  /* Process locked */
  _LOCK(spiHandle);

  /* Init temporary variables */
  tmp_state           = spiHandle->State;
  tmp_mode            = spiHandle->Init.Mode;

  if (!((tmp_state == SPI_STATE_READY) ||
        ((tmp_mode == SPI_MODE_MASTER) && (spiHandle->Init.Direction == SPI_DIRECTION_2LINES) && (tmp_state == SPI_STATE_BUSY_RX))))
  {
    errorcode = SYS_BUSY;
    goto error;
  }

  if ((pTxData == NULL) || (pRxData == NULL) || (Size == 0U))
  {
    errorcode = SYS_ERROR;
    goto error;
  }

  /* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
  if (spiHandle->State != SPI_STATE_BUSY_RX)
  {
    spiHandle->State = SPI_STATE_BUSY_TX_RX;
  }

  /* Set the transaction information */
  spiHandle->ErrorCode   = SPI_ERROR_NONE;
  spiHandle->pTxBuffPtr  = (uint8_t *)pTxData;
  spiHandle->TxXferSize  = Size;
  spiHandle->TxXferCount = Size;
  spiHandle->pRxBuffPtr  = (uint8_t *)pRxData;
  spiHandle->RxXferSize  = Size;
  spiHandle->RxXferCount = Size;

  /* Init field not used in handle to zero */
  spiHandle->RxISR       = NULL;
  spiHandle->TxISR       = NULL;

#if (USE_SPI_CRC != 0U)
  /* Reset CRC Calculation */
  if (spiHandle->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
  {
    SPI_RESET_CRC(spiHandle);
  }
#endif /* USE_SPI_CRC */

  /* Check if we are in Rx only or in Rx/Tx Mode and configure the DMA transfer complete callback */
  if (spiHandle->State == SPI_STATE_BUSY_RX)
  {
    /* Set the SPI Rx DMA Half transfer complete callback */
    spiHandle->hdmarx->XferHalfCpltCallback = SPI_DMAHalfReceiveCplt;
    spiHandle->hdmarx->XferCpltCallback     = SPI_DMAReceiveCplt;
  }
  else
  {
    /* Set the SPI Tx/Rx DMA Half transfer complete callback */
    //spiHandle->hdmarx->XferHalfCpltCallback = SPI_DMAHalfTransmitReceiveCplt;
    spiHandle->hdmarx->XferCpltCallback     = SPI_DMATransmitReceiveCplt;
  }

  /* Set the DMA error callback */
  spiHandle->hdmarx->XferErrorCallback = SPI_DMAError;

  /* Set the DMA AbortCpltCallback */
  spiHandle->hdmarx->XferAbortCallback = NULL;

  /* Enable the Rx DMA Stream/Channel  */
  if (SYS_OK != DMA_Start_IT(spiHandle->hdmarx, (uint32_t)&spiHandle->Instance->DR, (uint32_t)spiHandle->pRxBuffPtr,
                                 spiHandle->RxXferCount))
  {
    /* Update SPI error code */
    SET_BIT(spiHandle->ErrorCode, SPI_ERROR_DMA);
    errorcode = SYS_ERROR;

    spiHandle->State = SPI_STATE_READY;
    goto error;
  }

  /* Enable Rx DMA Request */
  SET_BIT(spiHandle->Instance->CR2, SPI_CR2_RXDMAEN);

  /* Set the SPI Tx DMA transfer complete callback as NULL because the communication closing
  is performed in DMA reception complete callback  */
  spiHandle->hdmatx->XferHalfCpltCallback = NULL;
  spiHandle->hdmatx->XferCpltCallback     = NULL;
  spiHandle->hdmatx->XferErrorCallback    = NULL;
  spiHandle->hdmatx->XferAbortCallback    = NULL;

  /* Enable the Tx DMA Stream/Channel  */
  if (SYS_OK != DMA_Start_IT(spiHandle->hdmatx, (uint32_t)spiHandle->pTxBuffPtr, (uint32_t)&spiHandle->Instance->DR,
                                 spiHandle->TxXferCount))
  {
    /* Update SPI error code */
    SET_BIT(spiHandle->ErrorCode, SPI_ERROR_DMA);
    errorcode = SYS_ERROR;

    spiHandle->State = SPI_STATE_READY;
    goto error;
  }

  /* Check if the SPI is already enabled */
  if ((spiHandle->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
  {
    /* Enable SPI peripheral */
    SPI_ENABLE(spiHandle);
  }
  /* Enable the SPI Error Interrupt Bit */
  SPI_ENABLE_IT(spiHandle, (SPI_IT_ERR));

  /* Enable Tx DMA Request */
  SET_BIT(spiHandle->Instance->CR2, SPI_CR2_TXDMAEN);

error :
  /* Process Unlocked */
  _UNLOCK(spiHandle);
  return errorcode;
}

StatusTypeDef SPI_Transmit_DMA(SPI_HandleTypeDef *spiHandle, uint8_t *pData, uint16_t Size)
{
  StatusTypeDef errorcode = SYS_OK;


  /* Process Locked */
  _LOCK(spiHandle);

  if (spiHandle->State != SPI_STATE_READY)
  {
    errorcode = SYS_BUSY;
    goto error;
  }

  if ((pData == NULL) || (Size == 0U))
  {
    errorcode = SYS_ERROR;
    goto error;
  }

  /* Set the transaction information */
  spiHandle->State       = SPI_STATE_BUSY_TX;
  spiHandle->ErrorCode   = SPI_ERROR_NONE;
  spiHandle->pTxBuffPtr  = (uint8_t *)pData;
  spiHandle->TxXferSize  = Size;
  spiHandle->TxXferCount = Size;

  /* Init field not used in handle to zero */
  spiHandle->pRxBuffPtr  = (uint8_t *)NULL;
  spiHandle->TxISR       = NULL;
  spiHandle->RxISR       = NULL;
  spiHandle->RxXferSize  = 0U;
  spiHandle->RxXferCount = 0U;

  /* Configure communication direction : 1Line */
  if (spiHandle->Init.Direction == SPI_DIRECTION_1LINE)
  {
    /* Disable SPI Peripheral before set 1Line direction (BIDIOE bit) */
    SPI_DISABLE(spiHandle);
    SPI_1LINE_TX(spiHandle);
  }

#if (USE_SPI_CRC != 0U)
  /* Reset CRC Calculation */
  if (spiHandle->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
  {
    SPI_RESET_CRC(spiHandle);
  }
#endif /* USE_SPI_CRC */

  /* Set the SPI TxDMA Half transfer complete callback */
  //spiHandle->hdmatx->XferHalfCpltCallback = SPI_DMAHalfTransmitCplt;

  /* Set the SPI TxDMA transfer complete callback */
  spiHandle->hdmatx->XferCpltCallback = SPI_DMATransmitCplt;

  /* Set the DMA error callback */
  spiHandle->hdmatx->XferErrorCallback = SPI_DMAError;

  /* Set the DMA AbortCpltCallback */
  spiHandle->hdmatx->XferAbortCallback = NULL;

  /* Enable the Tx DMA Stream/Channel */
  if (SYS_OK != DMA_Start_IT(spiHandle->hdmatx, (uint32_t)spiHandle->pTxBuffPtr, (uint32_t)&spiHandle->Instance->DR,
                                 spiHandle->TxXferCount))
  {
    /* Update SPI error code */
    SET_BIT(spiHandle->ErrorCode, SPI_ERROR_DMA);
    errorcode = SYS_ERROR;

    spiHandle->State = SPI_STATE_READY;
    goto error;
  }

  /* Check if the SPI is already enabled */
  if ((spiHandle->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
  {
    /* Enable SPI peripheral */
    SPI_ENABLE(spiHandle);
  }

  /* Enable the SPI Error Interrupt Bit */
  SPI_ENABLE_IT(spiHandle, (SPI_IT_ERR));

  /* Enable Tx DMA Request */
  SET_BIT(spiHandle->Instance->CR2, SPI_CR2_TXDMAEN);

error :
  /* Process Unlocked */
  _UNLOCK(spiHandle);
  return errorcode;
}

static void SPI_DMATransmitCplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *spiHandle = (SPI_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); 
  TickType_t tickstart;
  uint32_t errorcode;

  /* Init tickstart for timeout management*/
  tickstart = xTaskGetTickCount();

  /* DMA Normal Mode */
  if ((hdma->Instance->CR & DMA_SxCR_CIRC) != DMA_SxCR_CIRC)
  {
    /* Disable ERR interrupt */
    SPI_DISABLE_IT(spiHandle, SPI_IT_ERR);

    /* Disable Tx DMA Request */
    CLEAR_BIT(spiHandle->Instance->CR2, SPI_CR2_TXDMAEN);

    /* Check the end of the transaction */
    while ((SPI_GET_FLAG(spiHandle, SPI_FLAG_BSY) ? SET : RESET) != RESET){

        if (((xTaskGetTickCount() - tickstart) >=  SPI_DEFAULT_TIMEOUT)){

            SET_BIT(spiHandle->ErrorCode, SPI_ERROR_FLAG);   
        }

    }
    
    /* Clear overrun flag in 2 Lines communication mode because received data is not read */
    if (spiHandle->Init.Direction == SPI_DIRECTION_2LINES)
    {
      SPI_CLEAR_OVRFLAG(spiHandle);
    }

    spiHandle->TxXferCount = 0U;
    spiHandle->State = SPI_STATE_READY;

    if (spiHandle->ErrorCode != SPI_ERROR_NONE)
    {
      /* Call user error callback */
      spiHandle->ErrorCallback(spiHandle);

      return;
    }
  }  
  if(spiHandle->chip_select_present){
        
  /* pull CS high if cs present */
      setGpioOut(&spiHandle->pins[MB_CS]);
  }
  if(spiHandle->TxCpltCallback != NULL){

    /* Call user Tx complete callback */
    spiHandle->TxCpltCallback(spiHandle);
  }


}

/**
  * @brief  DMA SPI receive process complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMAReceiveCplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *spiHandle = (SPI_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); 
  TickType_t tickstart;
#if (USE_SPI_CRC != 0U)
  __IO uint32_t tmpreg = 0U;
#endif /* USE_SPI_CRC */

  /* Init tickstart for timeout management*/
  tickstart = xTaskGetTickCount();

  /* DMA Normal Mode */
  if ((hdma->Instance->CR & DMA_SxCR_CIRC) != DMA_SxCR_CIRC)
  {
    /* Disable ERR interrupt */
    SPI_DISABLE_IT(spiHandle, SPI_IT_ERR);

#if (USE_SPI_CRC != 0U)
    /* CRC handling */
    if (spiHandle->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      /* Wait until RXNE flag */
      if (SPI_WaitFlagStateUntilTimeout(spiHandle, SPI_FLAG_RXNE, SET, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
      {
        /* Error on the CRC reception */
        SET_BIT(spiHandle->ErrorCode, HAL_SPI_ERROR_CRC);
      }
      /* Read CRC */
      tmpreg = READ_REG(spiHandle->Instance->DR);
      /* To avoid GCC warning */
      UNUSED(tmpreg);
    }
#endif /* USE_SPI_CRC */

    /* Check if we are in Master RX 2 line mode */
    if ((spiHandle->Init.Direction == SPI_DIRECTION_2LINES) && (spiHandle->Init.Mode == SPI_MODE_MASTER))
    {
      /* Disable Rx/Tx DMA Request (done by default to handle the case master rx direction 2 lines) */
      CLEAR_BIT(spiHandle->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
    }
    else
    {
      /* Normal case */
      CLEAR_BIT(spiHandle->Instance->CR2, SPI_CR2_RXDMAEN);
    }

    /* Check the end of the transaction */
    while ((SPI_GET_FLAG(spiHandle, SPI_FLAG_BSY) ? SET : RESET) != RESET){

        if (((xTaskGetTickCount() - tickstart) >=  SPI_DEFAULT_TIMEOUT)){

            SET_BIT(spiHandle->ErrorCode, SPI_ERROR_FLAG);   
        }

    }

    spiHandle->RxXferCount = 0U;
    spiHandle->State = SPI_STATE_READY;

#if (USE_SPI_CRC != 0U)
    /* Check if CRC error occurred */
    if (__HAL_SPI_GET_FLAG(spiHandle, SPI_FLAG_CRCERR))
    {
      SET_BIT(spiHandle->ErrorCode, HAL_SPI_ERROR_CRC);
      __HAL_SPI_CLEAR_CRCERRFLAG(spiHandle);
    }
#endif /* USE_SPI_CRC */

    if (spiHandle->ErrorCode != SPI_ERROR_NONE)
    {
      /* Call user error callback */
      spiHandle->ErrorCallback(spiHandle);
      return;
    }
  }
  /* Call user Rx complete callback */

  spiHandle->RxCpltCallback(spiHandle);

}

/**
  * @brief  DMA SPI transmit receive process complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMATransmitReceiveCplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *spiHandle = (SPI_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); 
  TickType_t tickstart;
#if (USE_SPI_CRC != 0U)
  __IO uint32_t tmpreg = 0U;
#endif /* USE_SPI_CRC */

  /* Init tickstart for timeout management*/
  tickstart = xTaskGetTickCount();

  /* DMA Normal Mode */
  if ((hdma->Instance->CR & DMA_SxCR_CIRC) != DMA_SxCR_CIRC)
  {
    /* Disable ERR interrupt */
    SPI_DISABLE_IT(spiHandle, SPI_IT_ERR);

#if (USE_SPI_CRC != 0U)
    /* CRC handling */
    if (spiHandle->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      /* Wait the CRC data */
      if (SPI_WaitFlagStateUntilTimeout(spiHandle, SPI_FLAG_RXNE, SET, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
      {
        SET_BIT(spiHandle->ErrorCode, HAL_SPI_ERROR_CRC);
      }
      /* Read CRC to Flush DR and RXNE flag */
      tmpreg = READ_REG(spiHandle->Instance->DR);
      /* To avoid GCC warning */
      UNUSED(tmpreg);
    }
#endif /* USE_SPI_CRC */

    /* Check the end of the transaction */
    while ((SPI_GET_FLAG(spiHandle, SPI_FLAG_BSY) ? SET : RESET) != RESET){

        if (((xTaskGetTickCount() - tickstart) >=  SPI_DEFAULT_TIMEOUT)){

            SET_BIT(spiHandle->ErrorCode, SPI_ERROR_FLAG);   
        }

    }

    /* Disable Rx/Tx DMA Request */
    CLEAR_BIT(spiHandle->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

    spiHandle->TxXferCount = 0U;
    spiHandle->RxXferCount = 0U;
    spiHandle->State = SPI_STATE_READY;

#if (USE_SPI_CRC != 0U)
    /* Check if CRC error occurred */
    if (__HAL_SPI_GET_FLAG(spiHandle, SPI_FLAG_CRCERR))
    {
      SET_BIT(spiHandle->ErrorCode, HAL_SPI_ERROR_CRC);
      __HAL_SPI_CLEAR_CRCERRFLAG(spiHandle);
    }
#endif /* USE_SPI_CRC */

    if (spiHandle->ErrorCode != SPI_ERROR_NONE)
    {
      /* Call user error callback */
      spiHandle->ErrorCallback(spiHandle);

      return;
    }
  }
  if(spiHandle->chip_select_present){
        
  /* pull CS high if cs present */
      setGpioOut(&spiHandle->pins[MB_CS]);
  }
  if(spiHandle->TxRxCpltCallback != NULL){

    /* Call user TxRx complete callback */

    spiHandle->TxRxCpltCallback(spiHandle);
  }
}

/**
  * @brief  DMA SPI half transmit process complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMAHalfTransmitCplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *spiHandle = (SPI_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); 

  if(spiHandle->TxHalfCpltCallback != NULL){
  /* Call user Tx half complete callback */
  spiHandle->TxHalfCpltCallback(spiHandle);
  }
}

/**
  * @brief  DMA SPI half receive process complete callback
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMAHalfReceiveCplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *spiHandle = (SPI_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); 

  if(spiHandle->RxHalfCpltCallback != NULL){
    spiHandle->RxHalfCpltCallback(spiHandle);
  }
}

/**
  * @brief  DMA SPI half transmit receive process complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMAHalfTransmitReceiveCplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *spiHandle = (SPI_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); 

  if(spiHandle->TxRxHalfCpltCallback != NULL){
    /* Call user TxRx half complete callback */
    spiHandle->TxRxHalfCpltCallback(spiHandle);
  }
}

/**
  * @brief  DMA SPI communication error callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMAError(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *spiHandle = (SPI_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent);

  /* Stop the disable DMA transfer on SPI side */
  CLEAR_BIT(spiHandle->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

  SET_BIT(spiHandle->ErrorCode, SPI_ERROR_DMA);
  spiHandle->State = SPI_STATE_READY;

  spiHandle->ErrorCallback(spiHandle);

}