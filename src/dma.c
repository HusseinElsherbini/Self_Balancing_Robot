#include "dma.h"
#include "platform.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "task.h"

static uint32_t DMA_CalcBaseAndBitshift(DMA_HandleTypeDef *hdma)
{
  uint32_t stream_number = (((uint32_t)hdma->Instance & 0xFFU) - 16U) / 24U;
  
  /* lookup table for necessary bitshift of flags within status registers */
  static const uint8_t flagBitshiftOffset[8U] = {0U, 6U, 16U, 22U, 0U, 6U, 16U, 22U};
  hdma->StreamIndex = flagBitshiftOffset[stream_number];
  
  if (stream_number > 3U)
  {
    /* return pointer to HISR and HIFCR */
    hdma->StreamBaseAddress = (((uint32_t)hdma->Instance & (uint32_t)(~0x3FFU)) + 4U);
  }
  else
  {
    /* return pointer to LISR and LIFCR */
    hdma->StreamBaseAddress = ((uint32_t)hdma->Instance & (uint32_t)(~0x3FFU));
  }
  
  return hdma->StreamBaseAddress;
}

StatusTypeDef DMA_Init(DMA_HandleTypeDef *hdma)
{
  uint32_t tmp = 0U;
  TickType_t tickstart = xTaskGetTickCount();

  DMA_Base_Registers *regs;

  /* Check the DMA peripheral state */
  if(hdma == NULL)
  {
    return SYS_ERROR;
  }

  /* Change DMA peripheral state */
  hdma->State = DMA_STATE_BUSY;

  /* Allocate lock resource */
   _UNLOCK(hdma);
  
  /* Disable the peripheral */
  DMA_DISABLE(hdma);
  
  /* Check if the DMA Stream is effectively disabled */
  while((hdma->Instance->CR & DMA_SxCR_EN) != RESET)
  {
    /* Check for the Timeout */
    if (((xTaskGetTickCount() - tickstart) >  TIMEOUT_DMA_ABORT))
    {
      /* Update error code */
      hdma->ErrorCode = SYS_TIMEOUT;
      
      /* Change the DMA state */
      hdma->State = DMA_STATE_TIMEOUT;
      
      return SYS_TIMEOUT;
    }
  }
  
  /* Get the CR register value */
  tmp = hdma->Instance->CR;

  /* Clear CHSEL, MBURST, PBURST, PL, MSIZE, PSIZE, MINC, PINC, CIRC, DIR, CT and DBM bits */
  tmp &= ((uint32_t)~(DMA_SxCR_CHSEL | DMA_SxCR_MBURST | DMA_SxCR_PBURST | \
                      DMA_SxCR_PL    | DMA_SxCR_MSIZE  | DMA_SxCR_PSIZE  | \
                      DMA_SxCR_MINC  | DMA_SxCR_PINC   | DMA_SxCR_CIRC   | \
                      DMA_SxCR_DIR   | DMA_SxCR_CT     | DMA_SxCR_DBM));

  /* Prepare the DMA Stream configuration */
  tmp |=  hdma->Init.Channel             | hdma->Init.Direction        |
          hdma->Init.PeriphInc           | hdma->Init.MemInc           |
          hdma->Init.PeriphDataAlignment | hdma->Init.MemDataAlignment |
          hdma->Init.Mode                | hdma->Init.Priority;

  /* the Memory burst and peripheral burst are not used when the FIFO is disabled */
  if(hdma->Init.FIFOMode == DMA_FIFOMODE_ENABLE)
  {
    /* Get memory burst and peripheral burst */
    tmp |=  hdma->Init.MemBurst | hdma->Init.PeriphBurst;
  }
  
  /* Write to DMA Stream CR register */
  hdma->Instance->CR = tmp;  

  /* Get the FCR register value */
  tmp = hdma->Instance->FCR;

  /* Clear Direct mode and FIFO threshold bits */
  tmp &= (uint32_t)~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);

  /* Prepare the DMA Stream FIFO configuration */
  tmp |= hdma->Init.FIFOMode;

  /* The FIFO threshold is not used when the FIFO mode is disabled */
  if(hdma->Init.FIFOMode == DMA_FIFOMODE_ENABLE)
  {
    /* Get the FIFO threshold */
    tmp |= hdma->Init.FIFOThreshold;
    
    /* Check compatibility between FIFO threshold level and size of the memory burst */
    /* for INCR4, INCR8, INCR16 bursts */
    if (hdma->Init.MemBurst != DMA_MBURST_SINGLE)
    {
      if (DMA_CheckFifoParam(hdma) != SYS_OK)
      {
        /* Update error code */
        hdma->ErrorCode = DMA_ERROR_PARAM;
        
        /* Change the DMA state */
        hdma->State = DMA_STATE_READY;
        
        return SYS_ERROR; 
      }
    }
  }
  
  /* Write to DMA Stream FCR */
  hdma->Instance->FCR = tmp;

  /* Initialize StreamBaseAddress and StreamIndex parameters to be used to calculate
     DMA steam Base Address needed by HAL_DMA_IRQHandler() and HAL_DMA_PollForTransfer() */
  regs = (DMA_Base_Registers *)DMA_CalcBaseAndBitshift(hdma);
  
  /* Clear all interrupt flags */
  regs->IFCR = 0x3FU << hdma->StreamIndex;

  /* Initialize the error code */
  hdma->ErrorCode = DMA_ERROR_NONE;
                                                                                     
  /* Initialize the DMA state */
  hdma->State = DMA_STATE_READY;

  return SYS_OK;
}

void DMA_IRQHandler(DMA_HandleTypeDef *hdma) 
{
  uint32_t tmpisr;
  __IO uint32_t count = 0U;
  uint32_t timeout = MCU_CLK / 9600U;

  /* calculate DMA base and stream number */
  DMA_Base_Registers *regs = (DMA_Base_Registers *)hdma->StreamBaseAddress;

  tmpisr = regs->ISR;

  /* Transfer Error Interrupt management ***************************************/
  if ((tmpisr & (DMA_FLAG_TEIF0_4 << hdma->StreamIndex)) != RESET)
  {
    if(DMA_GET_IT_SOURCE(hdma, DMA_IT_TE) != RESET)
    {
      /* Disable the transfer error interrupt */
      hdma->Instance->CR  &= ~(DMA_IT_TE);
      
      /* Clear the transfer error flag */
      regs->IFCR = DMA_FLAG_TEIF0_4 << hdma->StreamIndex;
      
      /* Update error code */
      hdma->ErrorCode |= DMA_ERROR_TE;
    }
  }
  /* FIFO Error Interrupt management ******************************************/
  if ((tmpisr & (DMA_FLAG_FEIF0_4 << hdma->StreamIndex)) != RESET)
  {
    if(DMA_GET_IT_SOURCE(hdma, DMA_IT_FE) != RESET)
    {
      /* Clear the FIFO error flag */
      regs->IFCR = DMA_FLAG_FEIF0_4 << hdma->StreamIndex;

      /* Update error code */
      hdma->ErrorCode |= DMA_ERROR_FE;
    }
  }
  /* Direct Mode Error Interrupt management ***********************************/
  if ((tmpisr & (DMA_FLAG_DMEIF0_4 << hdma->StreamIndex)) != RESET)
  {
    if(DMA_GET_IT_SOURCE(hdma, DMA_IT_DME) != RESET)
    {
      /* Clear the direct mode error flag */
      regs->IFCR = DMA_FLAG_DMEIF0_4 << hdma->StreamIndex;

      /* Update error code */
      hdma->ErrorCode |= DMA_ERROR_DME;
    }
  }
  /* Half Transfer Complete Interrupt management ******************************/
  if ((tmpisr & (DMA_FLAG_HTIF0_4 << hdma->StreamIndex)) != RESET)
  {
    if(DMA_GET_IT_SOURCE(hdma, DMA_IT_HT) != RESET)
    {
      /* Clear the half transfer complete flag */
      regs->IFCR = DMA_FLAG_HTIF0_4 << hdma->StreamIndex;
      
      /* Multi_Buffering mode enabled */
      if(((hdma->Instance->CR) & (uint32_t)(DMA_SxCR_DBM)) != RESET)
      {
        /* Current memory buffer used is Memory 0 */
        if((hdma->Instance->CR & DMA_SxCR_CT) == RESET)
        {
          if(hdma->XferHalfCpltCallback != NULL)
          {
            /* Half transfer callback */
            hdma->XferHalfCpltCallback(hdma);
          }
        }
        /* Current memory buffer used is Memory 1 */
        else
        {
          if(hdma->XferM1HalfCpltCallback != NULL)
          {
            /* Half transfer callback */
            hdma->XferM1HalfCpltCallback(hdma);
          }
        }
      }
      else
      {
        /* Disable the half transfer interrupt if the DMA mode is not CIRCULAR */
        if((hdma->Instance->CR & DMA_SxCR_CIRC) == RESET)
        {
          /* Disable the half transfer interrupt */
          hdma->Instance->CR  &= ~(DMA_IT_HT);
        }
        
        if(hdma->XferHalfCpltCallback != NULL)
        {
          /* Half transfer callback */
          hdma->XferHalfCpltCallback(hdma);
        }
      }
    }
  }
  /* Transfer Complete Interrupt management ***********************************/
  if ((tmpisr & (DMA_FLAG_TCIF0_4 << hdma->StreamIndex)) != RESET)
  {
    if(DMA_GET_IT_SOURCE(hdma, DMA_IT_TC) != RESET)
    {
      /* Clear the transfer complete flag */
      regs->IFCR = DMA_FLAG_TCIF0_4 << hdma->StreamIndex;
      
      if(DMA_STATE_ABORT == hdma->State)
      {
        /* Disable all the transfer interrupts */
        hdma->Instance->CR  &= ~(DMA_IT_TC | DMA_IT_TE | DMA_IT_DME);
        hdma->Instance->FCR &= ~(DMA_IT_FE);
        
        if((hdma->XferHalfCpltCallback != NULL) || (hdma->XferM1HalfCpltCallback != NULL))
        {
          hdma->Instance->CR  &= ~(DMA_IT_HT);
        }

        /* Clear all interrupt flags at correct offset within the register */
        regs->IFCR = 0x3FU << hdma->StreamIndex;

        /* Change the DMA state */
        hdma->State = DMA_STATE_READY;

        /* Process Unlocked */
        _UNLOCK(hdma);

        if(hdma->XferAbortCallback != NULL)
        {
          hdma->XferAbortCallback(hdma);
        }
        return;
      }

      if(((hdma->Instance->CR) & (uint32_t)(DMA_SxCR_DBM)) != RESET)
      {
        /* Current memory buffer used is Memory 0 */
        if((hdma->Instance->CR & DMA_SxCR_CT) == RESET)
        {
          if(hdma->XferM1CpltCallback != NULL)
          {
            /* Transfer complete Callback for memory1 */
            hdma->XferM1CpltCallback(hdma);
          }
        }
        /* Current memory buffer used is Memory 1 */
        else
        {
          if(hdma->XferCpltCallback != NULL)
          {
            /* Transfer complete Callback for memory0 */
            hdma->XferCpltCallback(hdma);
          }
        }
      }
      /* Disable the transfer complete interrupt if the DMA mode is not CIRCULAR */
      else
      {
        if((hdma->Instance->CR & DMA_SxCR_CIRC) == RESET)
        {
          /* Disable the transfer complete interrupt */
          hdma->Instance->CR  &= ~(DMA_IT_TC);

          /* Change the DMA state */
          hdma->State = DMA_STATE_READY;

          /* Process Unlocked */
          _UNLOCK(hdma);
        }

        if(hdma->XferCpltCallback != NULL)
        {
          /* Transfer complete callback */
          hdma->XferCpltCallback(hdma);
        }
      }
    }
  }
  
  /* manage error case */
  if(hdma->ErrorCode != DMA_ERROR_NONE)
  {
    if((hdma->ErrorCode & DMA_ERROR_TE) != RESET)
    {
      hdma->State = DMA_STATE_ABORT;

      /* Disable the stream */
      DMA_DISABLE(hdma);

      do
      {
        if (++count > timeout)
        {
          break;
        }
      }
      while((hdma->Instance->CR & DMA_SxCR_EN) != RESET);

      /* Change the DMA state */
      hdma->State = DMA_STATE_READY;

      /* Process Unlocked */
      _UNLOCK(hdma);
    }

    if(hdma->XferErrorCallback != NULL)
    {
      /* Transfer error callback */
      hdma->XferErrorCallback(hdma);
    }
  }
}

StatusTypeDef DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  StatusTypeDef status = SYS_OK;

  /* calculate DMA base and stream number */
  DMA_Base_Registers *regs = (DMA_Base_Registers *)hdma->StreamBaseAddress;
  
 
  /* Process locked */
  _LOCK(hdma);
  
  if(DMA_STATE_READY == hdma->State)
  {
    /* Change DMA peripheral state */
    hdma->State = DMA_STATE_BUSY;
    
    /* Initialize the error code */
    hdma->ErrorCode = DMA_ERROR_NONE;
    
    /* Configure the source, destination address and the data length */
    DMA_SetConfig(hdma, SrcAddress, DstAddress, DataLength);
    
    /* Clear all interrupt flags at correct offset within the register */
    regs->IFCR = 0x3FU << hdma->StreamIndex;
    
    /* Enable Common interrupts*/
    hdma->Instance->CR  |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME;
    
    if(hdma->XferHalfCpltCallback != NULL)
    {
      hdma->Instance->CR  |= DMA_IT_HT;
    }
    
    /* Enable the Peripheral */
    DMA_ENABLE(hdma);
  }
  else
  {
    /* Process unlocked */
    _UNLOCK(hdma);	  
    
    /* Return error status */
    status = SYS_BUSY;
  }
  
  return status;
}

static void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  /* Clear DBM bit */
  hdma->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);

  /* Configure DMA Stream data length */
  hdma->Instance->NDTR = DataLength;

  /* Memory to Peripheral */
  if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
  {
    /* Configure DMA Stream destination address */
    hdma->Instance->PAR = DstAddress;

    /* Configure DMA Stream source address */
    hdma->Instance->M0AR = SrcAddress;
  }
  /* Peripheral to Memory */
  else
  {
    /* Configure DMA Stream source address */
    hdma->Instance->PAR = SrcAddress;

    /* Configure DMA Stream destination address */
    hdma->Instance->M0AR = DstAddress;
  }
}

static StatusTypeDef DMA_CheckFifoParam(DMA_HandleTypeDef *hdma)
{
  StatusTypeDef status = SYS_OK;
  uint32_t tmp = hdma->Init.FIFOThreshold;
  
  /* Memory Data size equal to Byte */
  if(hdma->Init.MemDataAlignment == DMA_MDATAALIGN_BYTE)
  {
    switch (tmp)
    {
    case DMA_FIFO_THRESHOLD_1QUARTERFULL:
    case DMA_FIFO_THRESHOLD_3QUARTERSFULL:
      if ((hdma->Init.MemBurst & DMA_SxCR_MBURST_1) == DMA_SxCR_MBURST_1)
      {
        status = SYS_ERROR;
      }
      break;
    case DMA_FIFO_THRESHOLD_HALFFULL:
      if (hdma->Init.MemBurst == DMA_MBURST_INC16)
      {
        status = SYS_ERROR;
      }
      break;
    case DMA_FIFO_THRESHOLD_FULL:
      break;
    default:
      break;
    }
  }
  
  /* Memory Data size equal to Half-Word */
  else if (hdma->Init.MemDataAlignment == DMA_MDATAALIGN_HALFWORD)
  {
    switch (tmp)
    {
    case DMA_FIFO_THRESHOLD_1QUARTERFULL:
    case DMA_FIFO_THRESHOLD_3QUARTERSFULL:
      status = SYS_ERROR;
      break;
    case DMA_FIFO_THRESHOLD_HALFFULL:
      if ((hdma->Init.MemBurst & DMA_SxCR_MBURST_1) == DMA_SxCR_MBURST_1)
      {
        status = SYS_ERROR;
      }
      break;
    case DMA_FIFO_THRESHOLD_FULL:
      if (hdma->Init.MemBurst == DMA_MBURST_INC16)
      {
        status = SYS_ERROR;
      }
      break;   
    default:
      break;
    }
  }
  
  /* Memory Data size equal to Word */
  else
  {
    switch (tmp)
    {
    case DMA_FIFO_THRESHOLD_1QUARTERFULL:
    case DMA_FIFO_THRESHOLD_HALFFULL:
    case DMA_FIFO_THRESHOLD_3QUARTERSFULL:
      status = SYS_ERROR;
      break;
    case DMA_FIFO_THRESHOLD_FULL:
      if ((hdma->Init.MemBurst & DMA_SxCR_MBURST_1) == DMA_SxCR_MBURST_1)
      {
        status = SYS_ERROR;
      }
      break;
    default:
      break;
    }
  } 
  
  return status; 
}