#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>
#include "platform.h"
#include "cortex.h"
#include "dma.h"
#include "stm32f4_basetypes.h"
/*
MB_SPI_SCK ----> PA5
MB_SPI_MISO ---> PA6
MB_SPI_MOSI ---> PA7
MB_SPI_CS -----> PC5
*/
/*******************  Bit definition for SPI_CR1 register  ********************/
#define SPI_CR1_CPHA_Pos            (0U)                                       
#define SPI_CR1_CPHA_Msk            (0x1UL << SPI_CR1_CPHA_Pos)                 /*!< 0x00000001 */
#define SPI_CR1_CPHA                SPI_CR1_CPHA_Msk                           /*!<Clock Phase      */
#define SPI_CR1_CPOL_Pos            (1U)                                       
#define SPI_CR1_CPOL_Msk            (0x1UL << SPI_CR1_CPOL_Pos)                 /*!< 0x00000002 */
#define SPI_CR1_CPOL                SPI_CR1_CPOL_Msk                           /*!<Clock Polarity   */
#define SPI_CR1_MSTR_Pos            (2U)                                       
#define SPI_CR1_MSTR_Msk            (0x1UL << SPI_CR1_MSTR_Pos)                 /*!< 0x00000004 */
#define SPI_CR1_MSTR                SPI_CR1_MSTR_Msk                           /*!<Master Selection */

#define SPI_CR1_BR_Pos              (3U)                                       
#define SPI_CR1_BR_Msk              (0x7UL << SPI_CR1_BR_Pos)                   /*!< 0x00000038 */
#define SPI_CR1_BR                  SPI_CR1_BR_Msk                             /*!<BR[2:0] bits (Baud Rate Control) */
#define SPI_CR1_BR_0                (0x1UL << SPI_CR1_BR_Pos)                   /*!< 0x00000008 */
#define SPI_CR1_BR_1                (0x2UL << SPI_CR1_BR_Pos)                   /*!< 0x00000010 */
#define SPI_CR1_BR_2                (0x4UL << SPI_CR1_BR_Pos)                   /*!< 0x00000020 */

#define SPI_CR1_SPE_Pos             (6U)                                       
#define SPI_CR1_SPE_Msk             (0x1UL << SPI_CR1_SPE_Pos)                  /*!< 0x00000040 */
#define SPI_CR1_SPE                 SPI_CR1_SPE_Msk                            /*!<SPI Enable                          */
#define SPI_CR1_LSBFIRST_Pos        (7U)                                       
#define SPI_CR1_LSBFIRST_Msk        (0x1UL << SPI_CR1_LSBFIRST_Pos)             /*!< 0x00000080 */
#define SPI_CR1_LSBFIRST            SPI_CR1_LSBFIRST_Msk                       /*!<Frame Format                        */
#define SPI_CR1_SSI_Pos             (8U)                                       
#define SPI_CR1_SSI_Msk             (0x1UL << SPI_CR1_SSI_Pos)                  /*!< 0x00000100 */
#define SPI_CR1_SSI                 SPI_CR1_SSI_Msk                            /*!<Internal slave select               */
#define SPI_CR1_SSM_Pos             (9U)                                       
#define SPI_CR1_SSM_Msk             (0x1UL << SPI_CR1_SSM_Pos)                  /*!< 0x00000200 */
#define SPI_CR1_SSM                 SPI_CR1_SSM_Msk                            /*!<Software slave management           */
#define SPI_CR1_RXONLY_Pos          (10U)                                      
#define SPI_CR1_RXONLY_Msk          (0x1UL << SPI_CR1_RXONLY_Pos)               /*!< 0x00000400 */
#define SPI_CR1_RXONLY              SPI_CR1_RXONLY_Msk                         /*!<Receive only                        */
#define SPI_CR1_DFF_Pos             (11U)                                      
#define SPI_CR1_DFF_Msk             (0x1UL << SPI_CR1_DFF_Pos)                  /*!< 0x00000800 */
#define SPI_CR1_DFF                 SPI_CR1_DFF_Msk                            /*!<Data Frame Format                   */
#define SPI_CR1_CRCNEXT_Pos         (12U)                                      
#define SPI_CR1_CRCNEXT_Msk         (0x1UL << SPI_CR1_CRCNEXT_Pos)              /*!< 0x00001000 */
#define SPI_CR1_CRCNEXT             SPI_CR1_CRCNEXT_Msk                        /*!<Transmit CRC next                   */
#define SPI_CR1_CRCEN_Pos           (13U)                                      
#define SPI_CR1_CRCEN_Msk           (0x1UL << SPI_CR1_CRCEN_Pos)                /*!< 0x00002000 */
#define SPI_CR1_CRCEN               SPI_CR1_CRCEN_Msk                          /*!<Hardware CRC calculation enable     */
#define SPI_CR1_BIDIOE_Pos          (14U)                                      
#define SPI_CR1_BIDIOE_Msk          (0x1UL << SPI_CR1_BIDIOE_Pos)               /*!< 0x00004000 */
#define SPI_CR1_BIDIOE              SPI_CR1_BIDIOE_Msk                         /*!<Output enable in bidirectional mode */
#define SPI_CR1_BIDIMODE_Pos        (15U)                                      
#define SPI_CR1_BIDIMODE_Msk        (0x1UL << SPI_CR1_BIDIMODE_Pos)             /*!< 0x00008000 */
#define SPI_CR1_BIDIMODE            SPI_CR1_BIDIMODE_Msk                       /*!<Bidirectional data mode enable      */

/*******************  Bit definition for SPI_CR2 register  ********************/
#define SPI_CR2_RXDMAEN_Pos         (0U)                                       
#define SPI_CR2_RXDMAEN_Msk         (0x1UL << SPI_CR2_RXDMAEN_Pos)              /*!< 0x00000001 */
#define SPI_CR2_RXDMAEN             SPI_CR2_RXDMAEN_Msk                        /*!<Rx Buffer DMA Enable                 */
#define SPI_CR2_TXDMAEN_Pos         (1U)                                       
#define SPI_CR2_TXDMAEN_Msk         (0x1UL << SPI_CR2_TXDMAEN_Pos)              /*!< 0x00000002 */
#define SPI_CR2_TXDMAEN             SPI_CR2_TXDMAEN_Msk                        /*!<Tx Buffer DMA Enable                 */
#define SPI_CR2_SSOE_Pos            (2U)                                       
#define SPI_CR2_SSOE_Msk            (0x1UL << SPI_CR2_SSOE_Pos)                 /*!< 0x00000004 */
#define SPI_CR2_SSOE                SPI_CR2_SSOE_Msk                           /*!<SS Output Enable                     */
#define SPI_CR2_FRF_Pos             (4U)                                       
#define SPI_CR2_FRF_Msk             (0x1UL << SPI_CR2_FRF_Pos)                  /*!< 0x00000010 */
#define SPI_CR2_FRF                 SPI_CR2_FRF_Msk                            /*!<Frame Format                         */
#define SPI_CR2_ERRIE_Pos           (5U)                                       
#define SPI_CR2_ERRIE_Msk           (0x1UL << SPI_CR2_ERRIE_Pos)                /*!< 0x00000020 */
#define SPI_CR2_ERRIE               SPI_CR2_ERRIE_Msk                          /*!<Error Interrupt Enable               */
#define SPI_CR2_RXNEIE_Pos          (6U)                                       
#define SPI_CR2_RXNEIE_Msk          (0x1UL << SPI_CR2_RXNEIE_Pos)               /*!< 0x00000040 */
#define SPI_CR2_RXNEIE              SPI_CR2_RXNEIE_Msk                         /*!<RX buffer Not Empty Interrupt Enable */
#define SPI_CR2_TXEIE_Pos           (7U)                                       
#define SPI_CR2_TXEIE_Msk           (0x1UL << SPI_CR2_TXEIE_Pos)                /*!< 0x00000080 */
#define SPI_CR2_TXEIE               SPI_CR2_TXEIE_Msk                          /*!<Tx buffer Empty Interrupt Enable     */

/********************  Bit definition for SPI_SR register  ********************/
#define SPI_SR_RXNE_Pos             (0U)                                       
#define SPI_SR_RXNE_Msk             (0x1UL << SPI_SR_RXNE_Pos)                  /*!< 0x00000001 */
#define SPI_SR_RXNE                 SPI_SR_RXNE_Msk                            /*!<Receive buffer Not Empty */
#define SPI_SR_TXE_Pos              (1U)                                       
#define SPI_SR_TXE_Msk              (0x1UL << SPI_SR_TXE_Pos)                   /*!< 0x00000002 */
#define SPI_SR_TXE                  SPI_SR_TXE_Msk                             /*!<Transmit buffer Empty    */
#define SPI_SR_CHSIDE_Pos           (2U)                                       
#define SPI_SR_CHSIDE_Msk           (0x1UL << SPI_SR_CHSIDE_Pos)                /*!< 0x00000004 */
#define SPI_SR_CHSIDE               SPI_SR_CHSIDE_Msk                          /*!<Channel side             */
#define SPI_SR_UDR_Pos              (3U)                                       
#define SPI_SR_UDR_Msk              (0x1UL << SPI_SR_UDR_Pos)                   /*!< 0x00000008 */
#define SPI_SR_UDR                  SPI_SR_UDR_Msk                             /*!<Underrun flag            */
#define SPI_SR_CRCERR_Pos           (4U)                                       
#define SPI_SR_CRCERR_Msk           (0x1UL << SPI_SR_CRCERR_Pos)                /*!< 0x00000010 */
#define SPI_SR_CRCERR               SPI_SR_CRCERR_Msk                          /*!<CRC Error flag           */
#define SPI_SR_MODF_Pos             (5U)                                       
#define SPI_SR_MODF_Msk             (0x1UL << SPI_SR_MODF_Pos)                  /*!< 0x00000020 */
#define SPI_SR_MODF                 SPI_SR_MODF_Msk                            /*!<Mode fault               */
#define SPI_SR_OVR_Pos              (6U)                                       
#define SPI_SR_OVR_Msk              (0x1UL << SPI_SR_OVR_Pos)                   /*!< 0x00000040 */
#define SPI_SR_OVR                  SPI_SR_OVR_Msk                             /*!<Overrun flag             */
#define SPI_SR_BSY_Pos              (7U)                                       
#define SPI_SR_BSY_Msk              (0x1UL << SPI_SR_BSY_Pos)                   /*!< 0x00000080 */
#define SPI_SR_BSY                  SPI_SR_BSY_Msk                             /*!<Busy flag                */
#define SPI_SR_FRE_Pos              (8U)                                       
#define SPI_SR_FRE_Msk              (0x1UL << SPI_SR_FRE_Pos)                   /*!< 0x00000100 */
#define SPI_SR_FRE                  SPI_SR_FRE_Msk                             /*!<Frame format error flag  */

/********************  Bit definition for SPI_DR register  ********************/
#define SPI_DR_DR_Pos               (0U)                                       
#define SPI_DR_DR_Msk               (0xFFFFUL << SPI_DR_DR_Pos)                 /*!< 0x0000FFFF */
#define SPI_DR_DR                   SPI_DR_DR_Msk                              /*!<Data Register           */

/*******************  Bit definition for SPI_CRCPR register  ******************/
#define SPI_CRCPR_CRCPOLY_Pos       (0U)                                       
#define SPI_CRCPR_CRCPOLY_Msk       (0xFFFFUL << SPI_CRCPR_CRCPOLY_Pos)         /*!< 0x0000FFFF */
#define SPI_CRCPR_CRCPOLY           SPI_CRCPR_CRCPOLY_Msk                      /*!<CRC polynomial register */

/******************  Bit definition for SPI_RXCRCR register  ******************/
#define SPI_RXCRCR_RXCRC_Pos        (0U)                                       
#define SPI_RXCRCR_RXCRC_Msk        (0xFFFFUL << SPI_RXCRCR_RXCRC_Pos)          /*!< 0x0000FFFF */
#define SPI_RXCRCR_RXCRC            SPI_RXCRCR_RXCRC_Msk                       /*!<Rx CRC Register         */

/******************  Bit definition for SPI_TXCRCR register  ******************/
#define SPI_TXCRCR_TXCRC_Pos        (0U)                                       
#define SPI_TXCRCR_TXCRC_Msk        (0xFFFFUL << SPI_TXCRCR_TXCRC_Pos)          /*!< 0x0000FFFF */
#define SPI_TXCRCR_TXCRC            SPI_TXCRCR_TXCRC_Msk                       /*!<Tx CRC Register */

#define SPI_MODE_SLAVE                  (0x00000000U)
#define SPI_MODE_MASTER                 (SPI_CR1_MSTR | SPI_CR1_SSI)
/**
  * @}
  */

/** @defgroup SPI_Direction SPI Direction Mode
  * @{
  */
#define SPI_DIRECTION_2LINES            (0x00000000U)
#define SPI_DIRECTION_2LINES_RXONLY     SPI_CR1_RXONLY
#define SPI_DIRECTION_1LINE             SPI_CR1_BIDIMODE
/**
  * @}
  */

/** @defgroup SPI_Data_Size SPI Data Size
  * @{
  */
#define SPI_DATASIZE_8BIT               (0x00000000U)
#define SPI_DATASIZE_16BIT              SPI_CR1_DFF
/**
  * @}
  */

/** @defgroup SPI_Clock_Polarity SPI Clock Polarity
  * @{
  */
#define SPI_POLARITY_LOW                (0x00000000U)
#define SPI_POLARITY_HIGH               SPI_CR1_CPOL
/**
  * @}
  */

/** @defgroup SPI_Clock_Phase SPI Clock Phase
  * @{
  */
#define SPI_PHASE_1EDGE                 (0x00000000U)
#define SPI_PHASE_2EDGE                 SPI_CR1_CPHA
/**
  * @}
  */

/** @defgroup SPI_Slave_Select_management SPI Slave Select Management
  * @{
  */
#define SPI_NSS_SOFT                    SPI_CR1_SSM
#define SPI_NSS_HARD_INPUT              (0x00000000U)
#define SPI_NSS_HARD_OUTPUT             (SPI_CR2_SSOE << 16U)
/**
  * @}
  */

/** @defgroup SPI_BaudRate_Prescaler SPI BaudRate Prescaler
  * @{
  */
#define SPI_BAUDRATEPRESCALER_2         (0x00000000U)
#define SPI_BAUDRATEPRESCALER_4         (SPI_CR1_BR_0)
#define SPI_BAUDRATEPRESCALER_8         (SPI_CR1_BR_1)
#define SPI_BAUDRATEPRESCALER_16        (SPI_CR1_BR_1 | SPI_CR1_BR_0)
#define SPI_BAUDRATEPRESCALER_32        (SPI_CR1_BR_2)
#define SPI_BAUDRATEPRESCALER_64        (SPI_CR1_BR_2 | SPI_CR1_BR_0)
#define SPI_BAUDRATEPRESCALER_128       (SPI_CR1_BR_2 | SPI_CR1_BR_1)
#define SPI_BAUDRATEPRESCALER_256       (SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0)

#define SPI_ERROR_NONE              (0x00000000U)   /*!< No error                               */
#define SPI_ERROR_MODF              (0x00000001U)   /*!< MODF error                             */
#define SPI_ERROR_CRC               (0x00000002U)   /*!< CRC error                              */
#define SPI_ERROR_OVR               (0x00000004U)   /*!< OVR error                              */
#define SPI_ERROR_FRE               (0x00000008U)   /*!< FRE error                              */
#define SPI_ERROR_DMA               (0x00000010U)   /*!< DMA transfer error                     */
#define SPI_ERROR_FLAG              (0x00000020U)   /*!< Error on RXNE/TXE/BSY Flag             */
#define SPI_ERROR_ABORT             (0x00000040U)   /*!< Error during SPI Abort procedure       */

/** @defgroup SPI_MSB_LSB_transmission SPI MSB LSB Transmission
  * @{
  */
#define SPI_FIRSTBIT_MSB                (0x00000000U)
#define SPI_FIRSTBIT_LSB                SPI_CR1_LSBFIRST
/**
  * @}
  */

/** @defgroup SPI_TI_mode SPI TI Mode
  * @{
  */
#define SPI_TIMODE_DISABLE              (0x00000000U)
#define SPI_TIMODE_ENABLE               SPI_CR2_FRF
/**
  * @}
  */

/** @defgroup SPI_CRC_Calculation SPI CRC Calculation
  * @{
  */
#define SPI_CRCCALCULATION_DISABLE      (0x00000000U)
#define SPI_CRCCALCULATION_ENABLE       SPI_CR1_CRCEN
/**
  * @}
  */

/** @defgroup SPI_Interrupt_definition SPI Interrupt Definition
  * @{
  */
#define SPI_IT_TXE                      SPI_CR2_TXEIE
#define SPI_IT_RXNE                     SPI_CR2_RXNEIE
#define SPI_IT_ERR                      SPI_CR2_ERRIE
/**
  * @}
  */
#define SPI_CHECK_FLAG(__SR__, __FLAG__) ((((__SR__) & ((__FLAG__) & SPI_FLAG_MASK)) == \
                                          ((__FLAG__) & SPI_FLAG_MASK)) ? SET : RESET)


#define SPI_CHECK_IT_SOURCE(__CR2__, __INTERRUPT__) ((((__CR2__) & (__INTERRUPT__)) == \
                                                     (__INTERRUPT__)) ? SET : RESET)
typedef enum 
{
  RESET = 0U, 
  SET = !RESET
} FlagStatus, ITStatus;

#define SPI_FLAG_RXNE                   SPI_SR_RXNE   /* SPI status flag: Rx buffer not empty flag       */
#define SPI_FLAG_TXE                    SPI_SR_TXE    /* SPI status flag: Tx buffer empty flag           */
#define SPI_FLAG_BSY                    SPI_SR_BSY    /* SPI status flag: Busy flag                      */
#define SPI_FLAG_CRCERR                 SPI_SR_CRCERR /* SPI Error flag: CRC error flag                  */
#define SPI_FLAG_MODF                   SPI_SR_MODF   /* SPI Error flag: Mode fault flag                 */
#define SPI_FLAG_OVR                    SPI_SR_OVR    /* SPI Error flag: Overrun flag                    */
#define SPI_FLAG_FRE                    SPI_SR_FRE    /* SPI Error flag: TI mode frame format error flag */
#define SPI_FLAG_MASK                   (SPI_SR_RXNE | SPI_SR_TXE | SPI_SR_BSY | SPI_SR_CRCERR\
                                         | SPI_SR_MODF | SPI_SR_OVR | SPI_SR_FRE)
typedef struct
{
  uint32_t Mode;                /*!< Specifies the SPI operating mode.
                                     This parameter can be a value of @ref SPI_Mode */

  uint32_t Direction;           /*!< Specifies the SPI bidirectional mode state.
                                     This parameter can be a value of @ref SPI_Direction */

  uint32_t DataSize;            /*!< Specifies the SPI data size.
                                     This parameter can be a value of @ref SPI_Data_Size */

  uint32_t CLKPolarity;         /*!< Specifies the serial clock steady state.
                                     This parameter can be a value of @ref SPI_Clock_Polarity */

  uint32_t CLKPhase;            /*!< Specifies the clock active edge for the bit capture.
                                     This parameter can be a value of @ref SPI_Clock_Phase */

  uint32_t NSS;                 /*!< Specifies whether the NSS signal is managed by
                                     hardware (NSS pin) or by software using the SSI bit.
                                     This parameter can be a value of @ref SPI_Slave_Select_management */

  uint32_t BaudRatePrescaler;   /*!< Specifies the Baud Rate prescaler value which will be
                                     used to configure the transmit and receive SCK clock.
                                     This parameter can be a value of @ref SPI_BaudRate_Prescaler
                                     @note The communication clock is derived from the master
                                     clock. The slave clock does not need to be set. */

  uint32_t FirstBit;            /*!< Specifies whether data transfers start from MSB or LSB bit.
                                     This parameter can be a value of @ref SPI_MSB_LSB_transmission */

  uint32_t TIMode;              /*!< Specifies if the TI mode is enabled or not.
                                     This parameter can be a value of @ref SPI_TI_mode */

  uint32_t CRCCalculation;      /*!< Specifies if the CRC calculation is enabled or not.
                                     This parameter can be a value of @ref SPI_CRC_Calculation */

  uint32_t CRCPolynomial;       /*!< Specifies the polynomial used for the CRC calculation.
                                     This parameter must be an odd number between Min_Data = 1 and Max_Data = 65535 */
} SPI_InitTypeDef;


typedef struct
{
  __IO uint32_t CR1;        /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
  __IO uint32_t CR2;        /*!< SPI control register 2,                             Address offset: 0x04 */
  __IO uint32_t SR;         /*!< SPI status register,                                Address offset: 0x08 */
  __IO uint32_t DR;         /*!< SPI data register,                                  Address offset: 0x0C */
  __IO uint32_t CRCPR;      /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
  __IO uint32_t RXCRCR;     /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
  __IO uint32_t TXCRCR;     /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
  __IO uint32_t I2SCFGR;    /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
  __IO uint32_t I2SPR;      /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
} SPI_TypeDef;

typedef enum
{
  SPI_STATE_RESET      = 0x00U,    /*!< Peripheral not Initialized                         */
  SPI_STATE_READY      = 0x01U,    /*!< Peripheral Initialized and ready for use           */
  SPI_STATE_BUSY       = 0x02U,    /*!< an internal process is ongoing                     */
  SPI_STATE_BUSY_TX    = 0x03U,    /*!< Data Transmission process is ongoing               */
  SPI_STATE_BUSY_RX    = 0x04U,    /*!< Data Reception process is ongoing                  */
  SPI_STATE_BUSY_TX_RX = 0x05U,    /*!< Data Transmission and Reception process is ongoing */
  SPI_STATE_ERROR      = 0x06U,    /*!< SPI error state                                    */
  SPI_STATE_ABORT      = 0x07U     /*!< SPI abort is ongoing                               */
} SPI_StateTypeDef;


/**
  * @brief  SPI handle Structure definition
  */
typedef struct __SPI_HandleTypeDef
{
  SPI_TypeDef                *Instance;      /*!< SPI registers base address               */

  GPIO_CONFIG_t              pins[4];    

  bool                       chip_select_present;  

  SPI_InitTypeDef            Init;           /*!< SPI communication parameters             */

  uint8_t                    *pTxBuffPtr;    /*!< Pointer to SPI Tx transfer Buffer        */

  uint16_t                   TxXferSize;     /*!< SPI Tx Transfer size                     */

  __IO uint16_t              TxXferCount;    /*!< SPI Tx Transfer Counter                  */

  uint8_t                    *pRxBuffPtr;    /*!< Pointer to SPI Rx transfer Buffer        */

  uint16_t                   RxXferSize;     /*!< SPI Rx Transfer size                     */

  __IO uint16_t              RxXferCount;    /*!< SPI Rx Transfer Counter                  */

  void (*RxISR)(struct __SPI_HandleTypeDef *hspi);   /*!< function pointer on Rx ISR       */

  void (*TxISR)(struct __SPI_HandleTypeDef *hspi);   /*!< function pointer on Tx ISR       */

  void (*ErrorCallback)(struct __SPI_HandleTypeDef *hspi); /*!< Error callback */

  void (*TxCpltCallback)(struct __SPI_HandleTypeDef *hspi); /*!< Transmit complete callback */

  void (*RxCpltCallback)(struct __SPI_HandleTypeDef *hspi); /*!< Receive complete callback */

  void(*TxRxCpltCallback)(struct __SPI_HandleTypeDef *hspi); /*!< Transmit Receive complete callback */

  void(*TxHalfCpltCallback)(struct __SPI_HandleTypeDef *hspi); /*!< Transmit Half complete callback */

  void(*RxHalfCpltCallback)(struct __SPI_HandleTypeDef *hspi); /*!< Receive Half complete callback */

  void(*TxRxHalfCpltCallback)(struct __SPI_HandleTypeDef *hspi); /*!< Transmit Receive Half complete callback */

  DMA_HandleTypeDef          *hdmatx;        /*!< SPI Tx DMA Handle parameters             */

  DMA_HandleTypeDef          *hdmarx;        /*!< SPI Rx DMA Handle parameters             */

  uint32_t                   Lock;           /*!< Locking object                           */

  __IO SPI_StateTypeDef  State;          /*!< SPI communication state                  */

  __IO uint32_t              ErrorCode;      /*!< SPI Error code                           */

} SPI_HandleTypeDef;

#define SPI_1LINE_TX(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_BIDIOE)

#define SPI_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->Instance->SR) & (__FLAG__)) == (__FLAG__))
#define SPI_ENABLE_IT(__HANDLE__, __INTERRUPT__)   SET_BIT((__HANDLE__)->Instance->CR2, (__INTERRUPT__))


#define SPI_DISABLE_IT(__HANDLE__, __INTERRUPT__)  CLEAR_BIT((__HANDLE__)->Instance->CR2, (__INTERRUPT__))



#define SPI_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) ((((__HANDLE__)->Instance->CR2\
                                                              & (__INTERRUPT__)) == (__INTERRUPT__)) ? SET : RESET)

#define SPI_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->Instance->SR) & (__FLAG__)) == (__FLAG__))


#define SPI_CLEAR_CRCERRFLAG(__HANDLE__) ((__HANDLE__)->Instance->SR = (uint16_t)(~SPI_FLAG_CRCERR))

#define SPI_DEFAULT_TIMEOUT 100U

#define SPI_CLEAR_MODFFLAG(__HANDLE__)             \
  do{                                                    \
    __IO uint32_t tmpreg_modf = 0x00U;                   \
    tmpreg_modf = (__HANDLE__)->Instance->SR;            \
    CLEAR_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_SPE); \
    UNUSED(tmpreg_modf);                                 \
  } while(0U)


#define SPI_CLEAR_OVRFLAG(__HANDLE__)        \
  do{                                              \
    __IO uint32_t tmpreg_ovr = 0x00U;              \
    tmpreg_ovr = (__HANDLE__)->Instance->DR;       \
    tmpreg_ovr = (__HANDLE__)->Instance->SR;       \
    UNUSED(tmpreg_ovr);                            \
  } while(0U)


#define SPI_CLEAR_FREFLAG(__HANDLE__)        \
  do{                                              \
    __IO uint32_t tmpreg_fre = 0x00U;              \
    tmpreg_fre = (__HANDLE__)->Instance->SR;       \
    UNUSED(tmpreg_fre);                            \
  }while(0U)


#define SPI_ENABLE(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_SPE)

#define SPI_DISABLE(__HANDLE__) CLEAR_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_SPE)

#define SPI_TIMEOUT_MANAGEMENT(timeout, tickstart)
#define SPI_MASTER_MODE 0x0
#define SPI_SLAVE_MODE 0x1

#define MB_MOSI           0x0
#define MB_MISO           0x1
#define MB_CLK            0x2
#define MB_CS             0x3

#define SPI1   ((SPI_TypeDef *)SPI1_PERIPH_BASE)

extern SPI_HandleTypeDef spiHandle1;

void spi_init(SPI_HandleTypeDef * spiHandle);
void SPI_WRITE_B(SPI_HandleTypeDef * spiHandle, uint8_t *buffer, uint8_t size, uint8_t timeout);
StatusTypeDef SPI_TransmitReceive_DMA(SPI_HandleTypeDef *spiHandle, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
StatusTypeDef SPI_Transmit_DMA(SPI_HandleTypeDef *spiHandle, uint8_t *pData, uint16_t Size);
static void SPI_DMATransmitCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMAReceiveCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMATransmitReceiveCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMAHalfTransmitCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMAHalfReceiveCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMAHalfTransmitReceiveCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMAError(DMA_HandleTypeDef *hdma);

#endif /*SPI_H_*/