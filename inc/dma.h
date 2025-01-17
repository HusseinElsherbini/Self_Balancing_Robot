#ifndef DMA_H_
#define DMA_H_

#include "cortex.h"
#include "stm32f4_basetypes.h"
#include "platform.h"

 #define TIMEOUT_DMA_ABORT    5U
/********************  Bits definition for DMA_SxCR register  *****************/
#define DMA_SxCR_CHSEL_Pos       (25U)                                         
#define DMA_SxCR_CHSEL_Msk       (0x7UL << DMA_SxCR_CHSEL_Pos)                  /*!< 0x0E000000 */
#define DMA_SxCR_CHSEL           DMA_SxCR_CHSEL_Msk                            
#define DMA_SxCR_CHSEL_0         0x02000000U                                   
#define DMA_SxCR_CHSEL_1         0x04000000U                                   
#define DMA_SxCR_CHSEL_2         0x08000000U                                   
#define DMA_SxCR_MBURST_Pos      (23U)                                         
#define DMA_SxCR_MBURST_Msk      (0x3UL << DMA_SxCR_MBURST_Pos)                 /*!< 0x01800000 */
#define DMA_SxCR_MBURST          DMA_SxCR_MBURST_Msk                           
#define DMA_SxCR_MBURST_0        (0x1UL << DMA_SxCR_MBURST_Pos)                 /*!< 0x00800000 */
#define DMA_SxCR_MBURST_1        (0x2UL << DMA_SxCR_MBURST_Pos)                 /*!< 0x01000000 */
#define DMA_SxCR_PBURST_Pos      (21U)                                         
#define DMA_SxCR_PBURST_Msk      (0x3UL << DMA_SxCR_PBURST_Pos)                 /*!< 0x00600000 */
#define DMA_SxCR_PBURST          DMA_SxCR_PBURST_Msk                           
#define DMA_SxCR_PBURST_0        (0x1UL << DMA_SxCR_PBURST_Pos)                 /*!< 0x00200000 */
#define DMA_SxCR_PBURST_1        (0x2UL << DMA_SxCR_PBURST_Pos)                 /*!< 0x00400000 */
#define DMA_SxCR_CT_Pos          (19U)                                         
#define DMA_SxCR_CT_Msk          (0x1UL << DMA_SxCR_CT_Pos)                     /*!< 0x00080000 */
#define DMA_SxCR_CT              DMA_SxCR_CT_Msk                               
#define DMA_SxCR_DBM_Pos         (18U)                                         
#define DMA_SxCR_DBM_Msk         (0x1UL << DMA_SxCR_DBM_Pos)                    /*!< 0x00040000 */
#define DMA_SxCR_DBM             DMA_SxCR_DBM_Msk                              
#define DMA_SxCR_PL_Pos          (16U)                                         
#define DMA_SxCR_PL_Msk          (0x3UL << DMA_SxCR_PL_Pos)                     /*!< 0x00030000 */
#define DMA_SxCR_PL              DMA_SxCR_PL_Msk                               
#define DMA_SxCR_PL_0            (0x1UL << DMA_SxCR_PL_Pos)                     /*!< 0x00010000 */
#define DMA_SxCR_PL_1            (0x2UL << DMA_SxCR_PL_Pos)                     /*!< 0x00020000 */
#define DMA_SxCR_PINCOS_Pos      (15U)                                         
#define DMA_SxCR_PINCOS_Msk      (0x1UL << DMA_SxCR_PINCOS_Pos)                 /*!< 0x00008000 */
#define DMA_SxCR_PINCOS          DMA_SxCR_PINCOS_Msk                           
#define DMA_SxCR_MSIZE_Pos       (13U)                                         
#define DMA_SxCR_MSIZE_Msk       (0x3UL << DMA_SxCR_MSIZE_Pos)                  /*!< 0x00006000 */
#define DMA_SxCR_MSIZE           DMA_SxCR_MSIZE_Msk                            
#define DMA_SxCR_MSIZE_0         (0x1UL << DMA_SxCR_MSIZE_Pos)                  /*!< 0x00002000 */
#define DMA_SxCR_MSIZE_1         (0x2UL << DMA_SxCR_MSIZE_Pos)                  /*!< 0x00004000 */
#define DMA_SxCR_PSIZE_Pos       (11U)                                         
#define DMA_SxCR_PSIZE_Msk       (0x3UL << DMA_SxCR_PSIZE_Pos)                  /*!< 0x00001800 */
#define DMA_SxCR_PSIZE           DMA_SxCR_PSIZE_Msk                            
#define DMA_SxCR_PSIZE_0         (0x1UL << DMA_SxCR_PSIZE_Pos)                  /*!< 0x00000800 */
#define DMA_SxCR_PSIZE_1         (0x2UL << DMA_SxCR_PSIZE_Pos)                  /*!< 0x00001000 */
#define DMA_SxCR_MINC_Pos        (10U)                                         
#define DMA_SxCR_MINC_Msk        (0x1UL << DMA_SxCR_MINC_Pos)                   /*!< 0x00000400 */
#define DMA_SxCR_MINC            DMA_SxCR_MINC_Msk                             
#define DMA_SxCR_PINC_Pos        (9U)                                          
#define DMA_SxCR_PINC_Msk        (0x1UL << DMA_SxCR_PINC_Pos)                   /*!< 0x00000200 */
#define DMA_SxCR_PINC            DMA_SxCR_PINC_Msk                             
#define DMA_SxCR_CIRC_Pos        (8U)                                          
#define DMA_SxCR_CIRC_Msk        (0x1UL << DMA_SxCR_CIRC_Pos)                   /*!< 0x00000100 */
#define DMA_SxCR_CIRC            DMA_SxCR_CIRC_Msk                             
#define DMA_SxCR_DIR_Pos         (6U)                                          
#define DMA_SxCR_DIR_Msk         (0x3UL << DMA_SxCR_DIR_Pos)                    /*!< 0x000000C0 */
#define DMA_SxCR_DIR             DMA_SxCR_DIR_Msk                              
#define DMA_SxCR_DIR_0           (0x1UL << DMA_SxCR_DIR_Pos)                    /*!< 0x00000040 */
#define DMA_SxCR_DIR_1           (0x2UL << DMA_SxCR_DIR_Pos)                    /*!< 0x00000080 */
#define DMA_SxCR_PFCTRL_Pos      (5U)                                          
#define DMA_SxCR_PFCTRL_Msk      (0x1UL << DMA_SxCR_PFCTRL_Pos)                 /*!< 0x00000020 */
#define DMA_SxCR_PFCTRL          DMA_SxCR_PFCTRL_Msk                           
#define DMA_SxCR_TCIE_Pos        (4U)                                          
#define DMA_SxCR_TCIE_Msk        (0x1UL << DMA_SxCR_TCIE_Pos)                   /*!< 0x00000010 */
#define DMA_SxCR_TCIE            DMA_SxCR_TCIE_Msk                             
#define DMA_SxCR_HTIE_Pos        (3U)                                          
#define DMA_SxCR_HTIE_Msk        (0x1UL << DMA_SxCR_HTIE_Pos)                   /*!< 0x00000008 */
#define DMA_SxCR_HTIE            DMA_SxCR_HTIE_Msk                             
#define DMA_SxCR_TEIE_Pos        (2U)                                          
#define DMA_SxCR_TEIE_Msk        (0x1UL << DMA_SxCR_TEIE_Pos)                   /*!< 0x00000004 */
#define DMA_SxCR_TEIE            DMA_SxCR_TEIE_Msk                             
#define DMA_SxCR_DMEIE_Pos       (1U)                                          
#define DMA_SxCR_DMEIE_Msk       (0x1UL << DMA_SxCR_DMEIE_Pos)                  /*!< 0x00000002 */
#define DMA_SxCR_DMEIE           DMA_SxCR_DMEIE_Msk                            
#define DMA_SxCR_EN_Pos          (0U)                                          
#define DMA_SxCR_EN_Msk          (0x1UL << DMA_SxCR_EN_Pos)                     /*!< 0x00000001 */
#define DMA_SxCR_EN              DMA_SxCR_EN_Msk                               

/* Legacy defines */
#define DMA_SxCR_ACK_Pos         (20U)                                         
#define DMA_SxCR_ACK_Msk         (0x1UL << DMA_SxCR_ACK_Pos)                    /*!< 0x00100000 */
#define DMA_SxCR_ACK             DMA_SxCR_ACK_Msk                              

/********************  Bits definition for DMA_SxCNDTR register  **************/
#define DMA_SxNDT_Pos            (0U)                                          
#define DMA_SxNDT_Msk            (0xFFFFUL << DMA_SxNDT_Pos)                    /*!< 0x0000FFFF */
#define DMA_SxNDT                DMA_SxNDT_Msk                                 
#define DMA_SxNDT_0              (0x0001UL << DMA_SxNDT_Pos)                    /*!< 0x00000001 */
#define DMA_SxNDT_1              (0x0002UL << DMA_SxNDT_Pos)                    /*!< 0x00000002 */
#define DMA_SxNDT_2              (0x0004UL << DMA_SxNDT_Pos)                    /*!< 0x00000004 */
#define DMA_SxNDT_3              (0x0008UL << DMA_SxNDT_Pos)                    /*!< 0x00000008 */
#define DMA_SxNDT_4              (0x0010UL << DMA_SxNDT_Pos)                    /*!< 0x00000010 */
#define DMA_SxNDT_5              (0x0020UL << DMA_SxNDT_Pos)                    /*!< 0x00000020 */
#define DMA_SxNDT_6              (0x0040UL << DMA_SxNDT_Pos)                    /*!< 0x00000040 */
#define DMA_SxNDT_7              (0x0080UL << DMA_SxNDT_Pos)                    /*!< 0x00000080 */
#define DMA_SxNDT_8              (0x0100UL << DMA_SxNDT_Pos)                    /*!< 0x00000100 */
#define DMA_SxNDT_9              (0x0200UL << DMA_SxNDT_Pos)                    /*!< 0x00000200 */
#define DMA_SxNDT_10             (0x0400UL << DMA_SxNDT_Pos)                    /*!< 0x00000400 */
#define DMA_SxNDT_11             (0x0800UL << DMA_SxNDT_Pos)                    /*!< 0x00000800 */
#define DMA_SxNDT_12             (0x1000UL << DMA_SxNDT_Pos)                    /*!< 0x00001000 */
#define DMA_SxNDT_13             (0x2000UL << DMA_SxNDT_Pos)                    /*!< 0x00002000 */
#define DMA_SxNDT_14             (0x4000UL << DMA_SxNDT_Pos)                    /*!< 0x00004000 */
#define DMA_SxNDT_15             (0x8000UL << DMA_SxNDT_Pos)                    /*!< 0x00008000 */

/********************  Bits definition for DMA_SxFCR register  ****************/ 
#define DMA_SxFCR_FEIE_Pos       (7U)                                          
#define DMA_SxFCR_FEIE_Msk       (0x1UL << DMA_SxFCR_FEIE_Pos)                  /*!< 0x00000080 */
#define DMA_SxFCR_FEIE           DMA_SxFCR_FEIE_Msk                            
#define DMA_SxFCR_FS_Pos         (3U)                                          
#define DMA_SxFCR_FS_Msk         (0x7UL << DMA_SxFCR_FS_Pos)                    /*!< 0x00000038 */
#define DMA_SxFCR_FS             DMA_SxFCR_FS_Msk                              
#define DMA_SxFCR_FS_0           (0x1UL << DMA_SxFCR_FS_Pos)                    /*!< 0x00000008 */
#define DMA_SxFCR_FS_1           (0x2UL << DMA_SxFCR_FS_Pos)                    /*!< 0x00000010 */
#define DMA_SxFCR_FS_2           (0x4UL << DMA_SxFCR_FS_Pos)                    /*!< 0x00000020 */
#define DMA_SxFCR_DMDIS_Pos      (2U)                                          
#define DMA_SxFCR_DMDIS_Msk      (0x1UL << DMA_SxFCR_DMDIS_Pos)                 /*!< 0x00000004 */
#define DMA_SxFCR_DMDIS          DMA_SxFCR_DMDIS_Msk                           
#define DMA_SxFCR_FTH_Pos        (0U)                                          
#define DMA_SxFCR_FTH_Msk        (0x3UL << DMA_SxFCR_FTH_Pos)                   /*!< 0x00000003 */
#define DMA_SxFCR_FTH            DMA_SxFCR_FTH_Msk                             
#define DMA_SxFCR_FTH_0          (0x1UL << DMA_SxFCR_FTH_Pos)                   /*!< 0x00000001 */
#define DMA_SxFCR_FTH_1          (0x2UL << DMA_SxFCR_FTH_Pos)                   /*!< 0x00000002 */

/********************  Bits definition for DMA_LISR register  *****************/ 
#define DMA_LISR_TCIF3_Pos       (27U)                                         
#define DMA_LISR_TCIF3_Msk       (0x1UL << DMA_LISR_TCIF3_Pos)                  /*!< 0x08000000 */
#define DMA_LISR_TCIF3           DMA_LISR_TCIF3_Msk                            
#define DMA_LISR_HTIF3_Pos       (26U)                                         
#define DMA_LISR_HTIF3_Msk       (0x1UL << DMA_LISR_HTIF3_Pos)                  /*!< 0x04000000 */
#define DMA_LISR_HTIF3           DMA_LISR_HTIF3_Msk                            
#define DMA_LISR_TEIF3_Pos       (25U)                                         
#define DMA_LISR_TEIF3_Msk       (0x1UL << DMA_LISR_TEIF3_Pos)                  /*!< 0x02000000 */
#define DMA_LISR_TEIF3           DMA_LISR_TEIF3_Msk                            
#define DMA_LISR_DMEIF3_Pos      (24U)                                         
#define DMA_LISR_DMEIF3_Msk      (0x1UL << DMA_LISR_DMEIF3_Pos)                 /*!< 0x01000000 */
#define DMA_LISR_DMEIF3          DMA_LISR_DMEIF3_Msk                           
#define DMA_LISR_FEIF3_Pos       (22U)                                         
#define DMA_LISR_FEIF3_Msk       (0x1UL << DMA_LISR_FEIF3_Pos)                  /*!< 0x00400000 */
#define DMA_LISR_FEIF3           DMA_LISR_FEIF3_Msk                            
#define DMA_LISR_TCIF2_Pos       (21U)                                         
#define DMA_LISR_TCIF2_Msk       (0x1UL << DMA_LISR_TCIF2_Pos)                  /*!< 0x00200000 */
#define DMA_LISR_TCIF2           DMA_LISR_TCIF2_Msk                            
#define DMA_LISR_HTIF2_Pos       (20U)                                         
#define DMA_LISR_HTIF2_Msk       (0x1UL << DMA_LISR_HTIF2_Pos)                  /*!< 0x00100000 */
#define DMA_LISR_HTIF2           DMA_LISR_HTIF2_Msk                            
#define DMA_LISR_TEIF2_Pos       (19U)                                         
#define DMA_LISR_TEIF2_Msk       (0x1UL << DMA_LISR_TEIF2_Pos)                  /*!< 0x00080000 */
#define DMA_LISR_TEIF2           DMA_LISR_TEIF2_Msk                            
#define DMA_LISR_DMEIF2_Pos      (18U)                                         
#define DMA_LISR_DMEIF2_Msk      (0x1UL << DMA_LISR_DMEIF2_Pos)                 /*!< 0x00040000 */
#define DMA_LISR_DMEIF2          DMA_LISR_DMEIF2_Msk                           
#define DMA_LISR_FEIF2_Pos       (16U)                                         
#define DMA_LISR_FEIF2_Msk       (0x1UL << DMA_LISR_FEIF2_Pos)                  /*!< 0x00010000 */
#define DMA_LISR_FEIF2           DMA_LISR_FEIF2_Msk                            
#define DMA_LISR_TCIF1_Pos       (11U)                                         
#define DMA_LISR_TCIF1_Msk       (0x1UL << DMA_LISR_TCIF1_Pos)                  /*!< 0x00000800 */
#define DMA_LISR_TCIF1           DMA_LISR_TCIF1_Msk                            
#define DMA_LISR_HTIF1_Pos       (10U)                                         
#define DMA_LISR_HTIF1_Msk       (0x1UL << DMA_LISR_HTIF1_Pos)                  /*!< 0x00000400 */
#define DMA_LISR_HTIF1           DMA_LISR_HTIF1_Msk                            
#define DMA_LISR_TEIF1_Pos       (9U)                                          
#define DMA_LISR_TEIF1_Msk       (0x1UL << DMA_LISR_TEIF1_Pos)                  /*!< 0x00000200 */
#define DMA_LISR_TEIF1           DMA_LISR_TEIF1_Msk                            
#define DMA_LISR_DMEIF1_Pos      (8U)                                          
#define DMA_LISR_DMEIF1_Msk      (0x1UL << DMA_LISR_DMEIF1_Pos)                 /*!< 0x00000100 */
#define DMA_LISR_DMEIF1          DMA_LISR_DMEIF1_Msk                           
#define DMA_LISR_FEIF1_Pos       (6U)                                          
#define DMA_LISR_FEIF1_Msk       (0x1UL << DMA_LISR_FEIF1_Pos)                  /*!< 0x00000040 */
#define DMA_LISR_FEIF1           DMA_LISR_FEIF1_Msk                            
#define DMA_LISR_TCIF0_Pos       (5U)                                          
#define DMA_LISR_TCIF0_Msk       (0x1UL << DMA_LISR_TCIF0_Pos)                  /*!< 0x00000020 */
#define DMA_LISR_TCIF0           DMA_LISR_TCIF0_Msk                            
#define DMA_LISR_HTIF0_Pos       (4U)                                          
#define DMA_LISR_HTIF0_Msk       (0x1UL << DMA_LISR_HTIF0_Pos)                  /*!< 0x00000010 */
#define DMA_LISR_HTIF0           DMA_LISR_HTIF0_Msk                            
#define DMA_LISR_TEIF0_Pos       (3U)                                          
#define DMA_LISR_TEIF0_Msk       (0x1UL << DMA_LISR_TEIF0_Pos)                  /*!< 0x00000008 */
#define DMA_LISR_TEIF0           DMA_LISR_TEIF0_Msk                            
#define DMA_LISR_DMEIF0_Pos      (2U)                                          
#define DMA_LISR_DMEIF0_Msk      (0x1UL << DMA_LISR_DMEIF0_Pos)                 /*!< 0x00000004 */
#define DMA_LISR_DMEIF0          DMA_LISR_DMEIF0_Msk                           
#define DMA_LISR_FEIF0_Pos       (0U)                                          
#define DMA_LISR_FEIF0_Msk       (0x1UL << DMA_LISR_FEIF0_Pos)                  /*!< 0x00000001 */
#define DMA_LISR_FEIF0           DMA_LISR_FEIF0_Msk                            

/********************  Bits definition for DMA_HISR register  *****************/ 
#define DMA_HISR_TCIF7_Pos       (27U)                                         
#define DMA_HISR_TCIF7_Msk       (0x1UL << DMA_HISR_TCIF7_Pos)                  /*!< 0x08000000 */
#define DMA_HISR_TCIF7           DMA_HISR_TCIF7_Msk                            
#define DMA_HISR_HTIF7_Pos       (26U)                                         
#define DMA_HISR_HTIF7_Msk       (0x1UL << DMA_HISR_HTIF7_Pos)                  /*!< 0x04000000 */
#define DMA_HISR_HTIF7           DMA_HISR_HTIF7_Msk                            
#define DMA_HISR_TEIF7_Pos       (25U)                                         
#define DMA_HISR_TEIF7_Msk       (0x1UL << DMA_HISR_TEIF7_Pos)                  /*!< 0x02000000 */
#define DMA_HISR_TEIF7           DMA_HISR_TEIF7_Msk                            
#define DMA_HISR_DMEIF7_Pos      (24U)                                         
#define DMA_HISR_DMEIF7_Msk      (0x1UL << DMA_HISR_DMEIF7_Pos)                 /*!< 0x01000000 */
#define DMA_HISR_DMEIF7          DMA_HISR_DMEIF7_Msk                           
#define DMA_HISR_FEIF7_Pos       (22U)                                         
#define DMA_HISR_FEIF7_Msk       (0x1UL << DMA_HISR_FEIF7_Pos)                  /*!< 0x00400000 */
#define DMA_HISR_FEIF7           DMA_HISR_FEIF7_Msk                            
#define DMA_HISR_TCIF6_Pos       (21U)                                         
#define DMA_HISR_TCIF6_Msk       (0x1UL << DMA_HISR_TCIF6_Pos)                  /*!< 0x00200000 */
#define DMA_HISR_TCIF6           DMA_HISR_TCIF6_Msk                            
#define DMA_HISR_HTIF6_Pos       (20U)                                         
#define DMA_HISR_HTIF6_Msk       (0x1UL << DMA_HISR_HTIF6_Pos)                  /*!< 0x00100000 */
#define DMA_HISR_HTIF6           DMA_HISR_HTIF6_Msk                            
#define DMA_HISR_TEIF6_Pos       (19U)                                         
#define DMA_HISR_TEIF6_Msk       (0x1UL << DMA_HISR_TEIF6_Pos)                  /*!< 0x00080000 */
#define DMA_HISR_TEIF6           DMA_HISR_TEIF6_Msk                            
#define DMA_HISR_DMEIF6_Pos      (18U)                                         
#define DMA_HISR_DMEIF6_Msk      (0x1UL << DMA_HISR_DMEIF6_Pos)                 /*!< 0x00040000 */
#define DMA_HISR_DMEIF6          DMA_HISR_DMEIF6_Msk                           
#define DMA_HISR_FEIF6_Pos       (16U)                                         
#define DMA_HISR_FEIF6_Msk       (0x1UL << DMA_HISR_FEIF6_Pos)                  /*!< 0x00010000 */
#define DMA_HISR_FEIF6           DMA_HISR_FEIF6_Msk                            
#define DMA_HISR_TCIF5_Pos       (11U)                                         
#define DMA_HISR_TCIF5_Msk       (0x1UL << DMA_HISR_TCIF5_Pos)                  /*!< 0x00000800 */
#define DMA_HISR_TCIF5           DMA_HISR_TCIF5_Msk                            
#define DMA_HISR_HTIF5_Pos       (10U)                                         
#define DMA_HISR_HTIF5_Msk       (0x1UL << DMA_HISR_HTIF5_Pos)                  /*!< 0x00000400 */
#define DMA_HISR_HTIF5           DMA_HISR_HTIF5_Msk                            
#define DMA_HISR_TEIF5_Pos       (9U)                                          
#define DMA_HISR_TEIF5_Msk       (0x1UL << DMA_HISR_TEIF5_Pos)                  /*!< 0x00000200 */
#define DMA_HISR_TEIF5           DMA_HISR_TEIF5_Msk                            
#define DMA_HISR_DMEIF5_Pos      (8U)                                          
#define DMA_HISR_DMEIF5_Msk      (0x1UL << DMA_HISR_DMEIF5_Pos)                 /*!< 0x00000100 */
#define DMA_HISR_DMEIF5          DMA_HISR_DMEIF5_Msk                           
#define DMA_HISR_FEIF5_Pos       (6U)                                          
#define DMA_HISR_FEIF5_Msk       (0x1UL << DMA_HISR_FEIF5_Pos)                  /*!< 0x00000040 */
#define DMA_HISR_FEIF5           DMA_HISR_FEIF5_Msk                            
#define DMA_HISR_TCIF4_Pos       (5U)                                          
#define DMA_HISR_TCIF4_Msk       (0x1UL << DMA_HISR_TCIF4_Pos)                  /*!< 0x00000020 */
#define DMA_HISR_TCIF4           DMA_HISR_TCIF4_Msk                            
#define DMA_HISR_HTIF4_Pos       (4U)                                          
#define DMA_HISR_HTIF4_Msk       (0x1UL << DMA_HISR_HTIF4_Pos)                  /*!< 0x00000010 */
#define DMA_HISR_HTIF4           DMA_HISR_HTIF4_Msk                            
#define DMA_HISR_TEIF4_Pos       (3U)                                          
#define DMA_HISR_TEIF4_Msk       (0x1UL << DMA_HISR_TEIF4_Pos)                  /*!< 0x00000008 */
#define DMA_HISR_TEIF4           DMA_HISR_TEIF4_Msk                            
#define DMA_HISR_DMEIF4_Pos      (2U)                                          
#define DMA_HISR_DMEIF4_Msk      (0x1UL << DMA_HISR_DMEIF4_Pos)                 /*!< 0x00000004 */
#define DMA_HISR_DMEIF4          DMA_HISR_DMEIF4_Msk                           
#define DMA_HISR_FEIF4_Pos       (0U)                                          
#define DMA_HISR_FEIF4_Msk       (0x1UL << DMA_HISR_FEIF4_Pos)                  /*!< 0x00000001 */
#define DMA_HISR_FEIF4           DMA_HISR_FEIF4_Msk                            

/********************  Bits definition for DMA_LIFCR register  ****************/ 
#define DMA_LIFCR_CTCIF3_Pos     (27U)                                         
#define DMA_LIFCR_CTCIF3_Msk     (0x1UL << DMA_LIFCR_CTCIF3_Pos)                /*!< 0x08000000 */
#define DMA_LIFCR_CTCIF3         DMA_LIFCR_CTCIF3_Msk                          
#define DMA_LIFCR_CHTIF3_Pos     (26U)                                         
#define DMA_LIFCR_CHTIF3_Msk     (0x1UL << DMA_LIFCR_CHTIF3_Pos)                /*!< 0x04000000 */
#define DMA_LIFCR_CHTIF3         DMA_LIFCR_CHTIF3_Msk                          
#define DMA_LIFCR_CTEIF3_Pos     (25U)                                         
#define DMA_LIFCR_CTEIF3_Msk     (0x1UL << DMA_LIFCR_CTEIF3_Pos)                /*!< 0x02000000 */
#define DMA_LIFCR_CTEIF3         DMA_LIFCR_CTEIF3_Msk                          
#define DMA_LIFCR_CDMEIF3_Pos    (24U)                                         
#define DMA_LIFCR_CDMEIF3_Msk    (0x1UL << DMA_LIFCR_CDMEIF3_Pos)               /*!< 0x01000000 */
#define DMA_LIFCR_CDMEIF3        DMA_LIFCR_CDMEIF3_Msk                         
#define DMA_LIFCR_CFEIF3_Pos     (22U)                                         
#define DMA_LIFCR_CFEIF3_Msk     (0x1UL << DMA_LIFCR_CFEIF3_Pos)                /*!< 0x00400000 */
#define DMA_LIFCR_CFEIF3         DMA_LIFCR_CFEIF3_Msk                          
#define DMA_LIFCR_CTCIF2_Pos     (21U)                                         
#define DMA_LIFCR_CTCIF2_Msk     (0x1UL << DMA_LIFCR_CTCIF2_Pos)                /*!< 0x00200000 */
#define DMA_LIFCR_CTCIF2         DMA_LIFCR_CTCIF2_Msk                          
#define DMA_LIFCR_CHTIF2_Pos     (20U)                                         
#define DMA_LIFCR_CHTIF2_Msk     (0x1UL << DMA_LIFCR_CHTIF2_Pos)                /*!< 0x00100000 */
#define DMA_LIFCR_CHTIF2         DMA_LIFCR_CHTIF2_Msk                          
#define DMA_LIFCR_CTEIF2_Pos     (19U)                                         
#define DMA_LIFCR_CTEIF2_Msk     (0x1UL << DMA_LIFCR_CTEIF2_Pos)                /*!< 0x00080000 */
#define DMA_LIFCR_CTEIF2         DMA_LIFCR_CTEIF2_Msk                          
#define DMA_LIFCR_CDMEIF2_Pos    (18U)                                         
#define DMA_LIFCR_CDMEIF2_Msk    (0x1UL << DMA_LIFCR_CDMEIF2_Pos)               /*!< 0x00040000 */
#define DMA_LIFCR_CDMEIF2        DMA_LIFCR_CDMEIF2_Msk                         
#define DMA_LIFCR_CFEIF2_Pos     (16U)                                         
#define DMA_LIFCR_CFEIF2_Msk     (0x1UL << DMA_LIFCR_CFEIF2_Pos)                /*!< 0x00010000 */
#define DMA_LIFCR_CFEIF2         DMA_LIFCR_CFEIF2_Msk                          
#define DMA_LIFCR_CTCIF1_Pos     (11U)                                         
#define DMA_LIFCR_CTCIF1_Msk     (0x1UL << DMA_LIFCR_CTCIF1_Pos)                /*!< 0x00000800 */
#define DMA_LIFCR_CTCIF1         DMA_LIFCR_CTCIF1_Msk                          
#define DMA_LIFCR_CHTIF1_Pos     (10U)                                         
#define DMA_LIFCR_CHTIF1_Msk     (0x1UL << DMA_LIFCR_CHTIF1_Pos)                /*!< 0x00000400 */
#define DMA_LIFCR_CHTIF1         DMA_LIFCR_CHTIF1_Msk                          
#define DMA_LIFCR_CTEIF1_Pos     (9U)                                          
#define DMA_LIFCR_CTEIF1_Msk     (0x1UL << DMA_LIFCR_CTEIF1_Pos)                /*!< 0x00000200 */
#define DMA_LIFCR_CTEIF1         DMA_LIFCR_CTEIF1_Msk                          
#define DMA_LIFCR_CDMEIF1_Pos    (8U)                                          
#define DMA_LIFCR_CDMEIF1_Msk    (0x1UL << DMA_LIFCR_CDMEIF1_Pos)               /*!< 0x00000100 */
#define DMA_LIFCR_CDMEIF1        DMA_LIFCR_CDMEIF1_Msk                         
#define DMA_LIFCR_CFEIF1_Pos     (6U)                                          
#define DMA_LIFCR_CFEIF1_Msk     (0x1UL << DMA_LIFCR_CFEIF1_Pos)                /*!< 0x00000040 */
#define DMA_LIFCR_CFEIF1         DMA_LIFCR_CFEIF1_Msk                          
#define DMA_LIFCR_CTCIF0_Pos     (5U)                                          
#define DMA_LIFCR_CTCIF0_Msk     (0x1UL << DMA_LIFCR_CTCIF0_Pos)                /*!< 0x00000020 */
#define DMA_LIFCR_CTCIF0         DMA_LIFCR_CTCIF0_Msk                          
#define DMA_LIFCR_CHTIF0_Pos     (4U)                                          
#define DMA_LIFCR_CHTIF0_Msk     (0x1UL << DMA_LIFCR_CHTIF0_Pos)                /*!< 0x00000010 */
#define DMA_LIFCR_CHTIF0         DMA_LIFCR_CHTIF0_Msk                          
#define DMA_LIFCR_CTEIF0_Pos     (3U)                                          
#define DMA_LIFCR_CTEIF0_Msk     (0x1UL << DMA_LIFCR_CTEIF0_Pos)                /*!< 0x00000008 */
#define DMA_LIFCR_CTEIF0         DMA_LIFCR_CTEIF0_Msk                          
#define DMA_LIFCR_CDMEIF0_Pos    (2U)                                          
#define DMA_LIFCR_CDMEIF0_Msk    (0x1UL << DMA_LIFCR_CDMEIF0_Pos)               /*!< 0x00000004 */
#define DMA_LIFCR_CDMEIF0        DMA_LIFCR_CDMEIF0_Msk                         
#define DMA_LIFCR_CFEIF0_Pos     (0U)                                          
#define DMA_LIFCR_CFEIF0_Msk     (0x1UL << DMA_LIFCR_CFEIF0_Pos)                /*!< 0x00000001 */
#define DMA_LIFCR_CFEIF0         DMA_LIFCR_CFEIF0_Msk                          

/********************  Bits definition for DMA_HIFCR  register  ****************/ 
#define DMA_HIFCR_CTCIF7_Pos     (27U)                                         
#define DMA_HIFCR_CTCIF7_Msk     (0x1UL << DMA_HIFCR_CTCIF7_Pos)                /*!< 0x08000000 */
#define DMA_HIFCR_CTCIF7         DMA_HIFCR_CTCIF7_Msk                          
#define DMA_HIFCR_CHTIF7_Pos     (26U)                                         
#define DMA_HIFCR_CHTIF7_Msk     (0x1UL << DMA_HIFCR_CHTIF7_Pos)                /*!< 0x04000000 */
#define DMA_HIFCR_CHTIF7         DMA_HIFCR_CHTIF7_Msk                          
#define DMA_HIFCR_CTEIF7_Pos     (25U)                                         
#define DMA_HIFCR_CTEIF7_Msk     (0x1UL << DMA_HIFCR_CTEIF7_Pos)                /*!< 0x02000000 */
#define DMA_HIFCR_CTEIF7         DMA_HIFCR_CTEIF7_Msk                          
#define DMA_HIFCR_CDMEIF7_Pos    (24U)                                         
#define DMA_HIFCR_CDMEIF7_Msk    (0x1UL << DMA_HIFCR_CDMEIF7_Pos)               /*!< 0x01000000 */
#define DMA_HIFCR_CDMEIF7        DMA_HIFCR_CDMEIF7_Msk                         
#define DMA_HIFCR_CFEIF7_Pos     (22U)                                         
#define DMA_HIFCR_CFEIF7_Msk     (0x1UL << DMA_HIFCR_CFEIF7_Pos)                /*!< 0x00400000 */
#define DMA_HIFCR_CFEIF7         DMA_HIFCR_CFEIF7_Msk                          
#define DMA_HIFCR_CTCIF6_Pos     (21U)                                         
#define DMA_HIFCR_CTCIF6_Msk     (0x1UL << DMA_HIFCR_CTCIF6_Pos)                /*!< 0x00200000 */
#define DMA_HIFCR_CTCIF6         DMA_HIFCR_CTCIF6_Msk                          
#define DMA_HIFCR_CHTIF6_Pos     (20U)                                         
#define DMA_HIFCR_CHTIF6_Msk     (0x1UL << DMA_HIFCR_CHTIF6_Pos)                /*!< 0x00100000 */
#define DMA_HIFCR_CHTIF6         DMA_HIFCR_CHTIF6_Msk                          
#define DMA_HIFCR_CTEIF6_Pos     (19U)                                         
#define DMA_HIFCR_CTEIF6_Msk     (0x1UL << DMA_HIFCR_CTEIF6_Pos)                /*!< 0x00080000 */
#define DMA_HIFCR_CTEIF6         DMA_HIFCR_CTEIF6_Msk                          
#define DMA_HIFCR_CDMEIF6_Pos    (18U)                                         
#define DMA_HIFCR_CDMEIF6_Msk    (0x1UL << DMA_HIFCR_CDMEIF6_Pos)               /*!< 0x00040000 */
#define DMA_HIFCR_CDMEIF6        DMA_HIFCR_CDMEIF6_Msk                         
#define DMA_HIFCR_CFEIF6_Pos     (16U)                                         
#define DMA_HIFCR_CFEIF6_Msk     (0x1UL << DMA_HIFCR_CFEIF6_Pos)                /*!< 0x00010000 */
#define DMA_HIFCR_CFEIF6         DMA_HIFCR_CFEIF6_Msk                          
#define DMA_HIFCR_CTCIF5_Pos     (11U)                                         
#define DMA_HIFCR_CTCIF5_Msk     (0x1UL << DMA_HIFCR_CTCIF5_Pos)                /*!< 0x00000800 */
#define DMA_HIFCR_CTCIF5         DMA_HIFCR_CTCIF5_Msk                          
#define DMA_HIFCR_CHTIF5_Pos     (10U)                                         
#define DMA_HIFCR_CHTIF5_Msk     (0x1UL << DMA_HIFCR_CHTIF5_Pos)                /*!< 0x00000400 */
#define DMA_HIFCR_CHTIF5         DMA_HIFCR_CHTIF5_Msk                          
#define DMA_HIFCR_CTEIF5_Pos     (9U)                                          
#define DMA_HIFCR_CTEIF5_Msk     (0x1UL << DMA_HIFCR_CTEIF5_Pos)                /*!< 0x00000200 */
#define DMA_HIFCR_CTEIF5         DMA_HIFCR_CTEIF5_Msk                          
#define DMA_HIFCR_CDMEIF5_Pos    (8U)                                          
#define DMA_HIFCR_CDMEIF5_Msk    (0x1UL << DMA_HIFCR_CDMEIF5_Pos)               /*!< 0x00000100 */
#define DMA_HIFCR_CDMEIF5        DMA_HIFCR_CDMEIF5_Msk                         
#define DMA_HIFCR_CFEIF5_Pos     (6U)                                          
#define DMA_HIFCR_CFEIF5_Msk     (0x1UL << DMA_HIFCR_CFEIF5_Pos)                /*!< 0x00000040 */
#define DMA_HIFCR_CFEIF5         DMA_HIFCR_CFEIF5_Msk                          
#define DMA_HIFCR_CTCIF4_Pos     (5U)                                          
#define DMA_HIFCR_CTCIF4_Msk     (0x1UL << DMA_HIFCR_CTCIF4_Pos)                /*!< 0x00000020 */
#define DMA_HIFCR_CTCIF4         DMA_HIFCR_CTCIF4_Msk                          
#define DMA_HIFCR_CHTIF4_Pos     (4U)                                          
#define DMA_HIFCR_CHTIF4_Msk     (0x1UL << DMA_HIFCR_CHTIF4_Pos)                /*!< 0x00000010 */
#define DMA_HIFCR_CHTIF4         DMA_HIFCR_CHTIF4_Msk                          
#define DMA_HIFCR_CTEIF4_Pos     (3U)                                          
#define DMA_HIFCR_CTEIF4_Msk     (0x1UL << DMA_HIFCR_CTEIF4_Pos)                /*!< 0x00000008 */
#define DMA_HIFCR_CTEIF4         DMA_HIFCR_CTEIF4_Msk                          
#define DMA_HIFCR_CDMEIF4_Pos    (2U)                                          
#define DMA_HIFCR_CDMEIF4_Msk    (0x1UL << DMA_HIFCR_CDMEIF4_Pos)               /*!< 0x00000004 */
#define DMA_HIFCR_CDMEIF4        DMA_HIFCR_CDMEIF4_Msk                         
#define DMA_HIFCR_CFEIF4_Pos     (0U)                                          
#define DMA_HIFCR_CFEIF4_Msk     (0x1UL << DMA_HIFCR_CFEIF4_Pos)                /*!< 0x00000001 */
#define DMA_HIFCR_CFEIF4         DMA_HIFCR_CFEIF4_Msk                          

/******************  Bit definition for DMA_SxPAR register  ********************/
#define DMA_SxPAR_PA_Pos         (0U)                                          
#define DMA_SxPAR_PA_Msk         (0xFFFFFFFFUL << DMA_SxPAR_PA_Pos)             /*!< 0xFFFFFFFF */
#define DMA_SxPAR_PA             DMA_SxPAR_PA_Msk                              /*!< Peripheral Address */

/******************  Bit definition for DMA_SxM0AR register  ********************/
#define DMA_SxM0AR_M0A_Pos       (0U)                                          
#define DMA_SxM0AR_M0A_Msk       (0xFFFFFFFFUL << DMA_SxM0AR_M0A_Pos)           /*!< 0xFFFFFFFF */
#define DMA_SxM0AR_M0A           DMA_SxM0AR_M0A_Msk                            /*!< Memory Address */

/******************  Bit definition for DMA_SxM1AR register  ********************/
#define DMA_SxM1AR_M1A_Pos       (0U)                                          
#define DMA_SxM1AR_M1A_Msk       (0xFFFFFFFFUL << DMA_SxM1AR_M1A_Pos)           /*!< 0xFFFFFFFF */
#define DMA_SxM1AR_M1A           DMA_SxM1AR_M1A_Msk                            /*!< Memory Address */

#define DMA_PERIPH_TO_MEMORY          0x00000000U                 /*!< Peripheral to memory direction */
#define DMA_MEMORY_TO_PERIPH          ((uint32_t)DMA_SxCR_DIR_0)  /*!< Memory to peripheral direction */
#define DMA_MEMORY_TO_MEMORY          ((uint32_t)DMA_SxCR_DIR_1)  /*!< Memory to memory direction     */
/**
  * @}
  */
        
/** @defgroup DMA_Peripheral_incremented_mode DMA Peripheral incremented mode
  * @brief    DMA peripheral incremented mode 
  * @{
  */ 
#define DMA_PINC_ENABLE               ((uint32_t)DMA_SxCR_PINC)   /*!< Peripheral increment mode enable  */
#define DMA_PINC_DISABLE              0x00000000U                 /*!< Peripheral increment mode disable */
/**
  * @}
  */ 

/** @defgroup DMA_Memory_incremented_mode DMA Memory incremented mode
  * @brief    DMA memory incremented mode 
  * @{
  */ 
#define DMA_MINC_ENABLE               ((uint32_t)DMA_SxCR_MINC)   /*!< Memory increment mode enable  */
#define DMA_MINC_DISABLE              0x00000000U                 /*!< Memory increment mode disable */
/**
  * @}
  */

/** @defgroup DMA_Peripheral_data_size DMA Peripheral data size
  * @brief    DMA peripheral data size 
  * @{
  */ 
#define DMA_PDATAALIGN_BYTE           0x00000000U                  /*!< Peripheral data alignment: Byte     */
#define DMA_PDATAALIGN_HALFWORD       ((uint32_t)DMA_SxCR_PSIZE_0) /*!< Peripheral data alignment: HalfWord */
#define DMA_PDATAALIGN_WORD           ((uint32_t)DMA_SxCR_PSIZE_1) /*!< Peripheral data alignment: Word     */
/**
  * @}
  */ 

/** @defgroup DMA_Memory_data_size DMA Memory data size
  * @brief    DMA memory data size 
  * @{ 
  */
#define DMA_MDATAALIGN_BYTE           0x00000000U                  /*!< Memory data alignment: Byte     */
#define DMA_MDATAALIGN_HALFWORD       ((uint32_t)DMA_SxCR_MSIZE_0) /*!< Memory data alignment: HalfWord */
#define DMA_MDATAALIGN_WORD           ((uint32_t)DMA_SxCR_MSIZE_1) /*!< Memory data alignment: Word     */
/**
  * @}
  */

/** @defgroup DMA_mode DMA mode
  * @brief    DMA mode 
  * @{
  */ 
#define DMA_NORMAL                    0x00000000U                  /*!< Normal mode                  */
#define DMA_CIRCULAR                  ((uint32_t)DMA_SxCR_CIRC)    /*!< Circular mode                */
#define DMA_PFCTRL                    ((uint32_t)DMA_SxCR_PFCTRL)  /*!< Peripheral flow control mode */
/**
  * @}
  */

/** @defgroup DMA_Priority_level DMA Priority level
  * @brief    DMA priority levels 
  * @{
  */
#define DMA_PRIORITY_LOW              0x00000000U                 /*!< Priority level: Low       */
#define DMA_PRIORITY_MEDIUM           ((uint32_t)DMA_SxCR_PL_0)   /*!< Priority level: Medium    */
#define DMA_PRIORITY_HIGH             ((uint32_t)DMA_SxCR_PL_1)   /*!< Priority level: High      */
#define DMA_PRIORITY_VERY_HIGH        ((uint32_t)DMA_SxCR_PL)     /*!< Priority level: Very High */
/**
  * @}
  */ 

/** @defgroup DMA_FIFO_direct_mode DMA FIFO direct mode
  * @brief    DMA FIFO direct mode
  * @{
  */
#define DMA_FIFOMODE_DISABLE          0x00000000U                 /*!< FIFO mode disable */
#define DMA_FIFOMODE_ENABLE           ((uint32_t)DMA_SxFCR_DMDIS) /*!< FIFO mode enable  */
/**
  * @}
  */ 

/** @defgroup DMA_FIFO_threshold_level DMA FIFO threshold level
  * @brief    DMA FIFO level 
  * @{
  */
#define DMA_FIFO_THRESHOLD_1QUARTERFULL       0x00000000U                  /*!< FIFO threshold 1 quart full configuration  */
#define DMA_FIFO_THRESHOLD_HALFFULL           ((uint32_t)DMA_SxFCR_FTH_0)  /*!< FIFO threshold half full configuration     */
#define DMA_FIFO_THRESHOLD_3QUARTERSFULL      ((uint32_t)DMA_SxFCR_FTH_1)  /*!< FIFO threshold 3 quarts full configuration */
#define DMA_FIFO_THRESHOLD_FULL               ((uint32_t)DMA_SxFCR_FTH)    /*!< FIFO threshold full configuration          */
/**
  * @}
  */ 

/** @defgroup DMA_Memory_burst DMA Memory burst
  * @brief    DMA memory burst 
  * @{
  */ 
#define DMA_MBURST_SINGLE             0x00000000U
#define DMA_MBURST_INC4               ((uint32_t)DMA_SxCR_MBURST_0)  
#define DMA_MBURST_INC8               ((uint32_t)DMA_SxCR_MBURST_1)  
#define DMA_MBURST_INC16              ((uint32_t)DMA_SxCR_MBURST)  
/**
  * @}
  */ 

/** @defgroup DMA_Peripheral_burst DMA Peripheral burst
  * @brief    DMA peripheral burst 
  * @{
  */ 
#define DMA_PBURST_SINGLE             0x00000000U
#define DMA_PBURST_INC4               ((uint32_t)DMA_SxCR_PBURST_0)
#define DMA_PBURST_INC8               ((uint32_t)DMA_SxCR_PBURST_1)
#define DMA_PBURST_INC16              ((uint32_t)DMA_SxCR_PBURST)
/**
  * @}
  */

/** @defgroup DMA_interrupt_enable_definitions DMA interrupt enable definitions
  * @brief    DMA interrupts definition 
  * @{
  */
#define DMA_IT_TC                     ((uint32_t)DMA_SxCR_TCIE)
#define DMA_IT_HT                     ((uint32_t)DMA_SxCR_HTIE)
#define DMA_IT_TE                     ((uint32_t)DMA_SxCR_TEIE)
#define DMA_IT_DME                    ((uint32_t)DMA_SxCR_DMEIE)
#define DMA_IT_FE                     0x00000080U
/**
  * @}
  */

/** @defgroup DMA_flag_definitions DMA flag definitions
  * @brief    DMA flag definitions 
  * @{
  */ 
#define DMA_FLAG_FEIF0_4              0x00000001U
#define DMA_FLAG_DMEIF0_4             0x00000004U
#define DMA_FLAG_TEIF0_4              0x00000008U
#define DMA_FLAG_HTIF0_4              0x00000010U
#define DMA_FLAG_TCIF0_4              0x00000020U
#define DMA_FLAG_FEIF1_5              0x00000040U
#define DMA_FLAG_DMEIF1_5             0x00000100U
#define DMA_FLAG_TEIF1_5              0x00000200U
#define DMA_FLAG_HTIF1_5              0x00000400U
#define DMA_FLAG_TCIF1_5              0x00000800U
#define DMA_FLAG_FEIF2_6              0x00010000U
#define DMA_FLAG_DMEIF2_6             0x00040000U
#define DMA_FLAG_TEIF2_6              0x00080000U
#define DMA_FLAG_HTIF2_6              0x00100000U
#define DMA_FLAG_TCIF2_6              0x00200000U
#define DMA_FLAG_FEIF3_7              0x00400000U
#define DMA_FLAG_DMEIF3_7             0x01000000U
#define DMA_FLAG_TEIF3_7              0x02000000U
#define DMA_FLAG_HTIF3_7              0x04000000U
#define DMA_FLAG_TCIF3_7              0x08000000U

typedef struct
{
  __IO uint32_t CR;     /*!< DMA stream x configuration register      */
  __IO uint32_t NDTR;   /*!< DMA stream x number of data register     */
  __IO uint32_t PAR;    /*!< DMA stream x peripheral address register */
  __IO uint32_t M0AR;   /*!< DMA stream x memory 0 address register   */
  __IO uint32_t M1AR;   /*!< DMA stream x memory 1 address register   */
  __IO uint32_t FCR;    /*!< DMA stream x FIFO control register       */
} DMA_Stream_TypeDef;

typedef struct
{
  __IO uint32_t LISR;   /*!< DMA low interrupt status register,      Address offset: 0x00 */
  __IO uint32_t HISR;   /*!< DMA high interrupt status register,     Address offset: 0x04 */
  __IO uint32_t LIFCR;  /*!< DMA low interrupt flag clear register,  Address offset: 0x08 */
  __IO uint32_t HIFCR;  /*!< DMA high interrupt flag clear register, Address offset: 0x0C */
} DMA_TypeDef;

typedef struct
{
  uint32_t Channel;              /*!< Specifies the channel used for the specified stream. 
                                      This parameter can be a value of @ref DMA_Channel_selection                    */

  uint32_t Direction;            /*!< Specifies if the data will be transferred from memory to peripheral, 
                                      from memory to memory or from peripheral to memory.
                                      This parameter can be a value of @ref DMA_Data_transfer_direction              */

  uint32_t PeriphInc;            /*!< Specifies whether the Peripheral address register should be incremented or not.
                                      This parameter can be a value of @ref DMA_Peripheral_incremented_mode          */

  uint32_t MemInc;               /*!< Specifies whether the memory address register should be incremented or not.
                                      This parameter can be a value of @ref DMA_Memory_incremented_mode              */

  uint32_t PeriphDataAlignment;  /*!< Specifies the Peripheral data width.
                                      This parameter can be a value of @ref DMA_Peripheral_data_size                 */

  uint32_t MemDataAlignment;     /*!< Specifies the Memory data width.
                                      This parameter can be a value of @ref DMA_Memory_data_size                     */

  uint32_t Mode;                 /*!< Specifies the operation mode of the DMAy Streamx.
                                      This parameter can be a value of @ref DMA_mode
                                      @note The circular buffer mode cannot be used if the memory-to-memory
                                            data transfer is configured on the selected Stream                        */

  uint32_t Priority;             /*!< Specifies the software priority for the DMAy Streamx.
                                      This parameter can be a value of @ref DMA_Priority_level                       */

  uint32_t FIFOMode;             /*!< Specifies if the FIFO mode or Direct mode will be used for the specified stream.
                                      This parameter can be a value of @ref DMA_FIFO_direct_mode
                                      @note The Direct mode (FIFO mode disabled) cannot be used if the 
                                            memory-to-memory data transfer is configured on the selected stream       */

  uint32_t FIFOThreshold;        /*!< Specifies the FIFO threshold level.
                                      This parameter can be a value of @ref DMA_FIFO_threshold_level                  */

  uint32_t MemBurst;             /*!< Specifies the Burst transfer configuration for the memory transfers. 
                                      It specifies the amount of data to be transferred in a single non interruptible
                                      transaction.
                                      This parameter can be a value of @ref DMA_Memory_burst 
                                      @note The burst mode is possible only if the address Increment mode is enabled. */

  uint32_t PeriphBurst;          /*!< Specifies the Burst transfer configuration for the peripheral transfers. 
                                      It specifies the amount of data to be transferred in a single non interruptible 
                                      transaction. 
                                      This parameter can be a value of @ref DMA_Peripheral_burst
                                      @note The burst mode is possible only if the address Increment mode is enabled. */
}DMA_InitTypeDef;


/** 
  * @brief  HAL DMA State structures definition
  */
typedef enum
{
  DMA_STATE_RESET             = 0x00U,  /*!< DMA not yet initialized or disabled */
  DMA_STATE_READY             = 0x01U,  /*!< DMA initialized and ready for use   */
  DMA_STATE_BUSY              = 0x02U,  /*!< DMA process is ongoing              */
  DMA_STATE_TIMEOUT           = 0x03U,  /*!< DMA timeout state                   */
  DMA_STATE_ERROR             = 0x04U,  /*!< DMA error state                     */
  DMA_STATE_ABORT             = 0x05U,  /*!< DMA Abort state                     */
}DMA_StateTypeDef;

/** 
  * @brief  HAL DMA Error Code structure definition
  */
typedef enum
{
  DMA_FULL_TRANSFER           = 0x00U,  /*!< Full transfer     */
  DMA_HALF_TRANSFER           = 0x01U   /*!< Half Transfer     */
}DMA_LevelCompleteTypeDef;

typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;

/** 
  * @brief  HAL DMA Error Code structure definition
  */
typedef enum
{
  DMA_XFER_CPLT_CB_ID         = 0x00U,  /*!< Full transfer     */
  DMA_XFER_HALFCPLT_CB_ID     = 0x01U,  /*!< Half Transfer     */
  DMA_XFER_M1CPLT_CB_ID       = 0x02U,  /*!< M1 Full Transfer  */
  DMA_XFER_M1HALFCPLT_CB_ID   = 0x03U,  /*!< M1 Half Transfer  */
  DMA_XFER_ERROR_CB_ID        = 0x04U,  /*!< Error             */
  DMA_XFER_ABORT_CB_ID        = 0x05U,  /*!< Abort             */
  DMA_XFER_ALL_CB_ID          = 0x06U   /*!< All               */
}DMA_CallbackIDTypeDef;

#define DMA_ERROR_NONE            0x00000000U    /*!< No error                               */
#define DMA_ERROR_TE              0x00000001U    /*!< Transfer error                         */
#define DMA_ERROR_FE              0x00000002U    /*!< FIFO error                             */
#define DMA_ERROR_DME             0x00000004U    /*!< Direct Mode error                      */
#define DMA_ERROR_TIMEOUT         0x00000020U    /*!< Timeout error                          */
#define DMA_ERROR_PARAM           0x00000040U    /*!< Parameter error                        */
#define DMA_ERROR_NO_XFER         0x00000080U    /*!< Abort requested with no Xfer ongoing   */
#define DMA_ERROR_NOT_SUPPORTED   0x00000100U    /*!< Not supported mode                     */

/** 
  * @brief  DMA handle Structure definition
  */
typedef struct DMA_HandleTypeDef
{
  DMA_Stream_TypeDef         *Instance;                                                        /*!< Register base address                  */

  DMA_InitTypeDef            Init;                                                             /*!< DMA communication parameters           */ 

  uint32_t            Lock;                                                                    /*!< DMA locking object                     */  

  __IO DMA_StateTypeDef  State;                                                            /*!< DMA transfer state                     */

  void                       *Parent;                                                          /*!< Parent object state                    */ 

  void                       (* XferCpltCallback)( struct __DMA_HandleTypeDef * hdma);         /*!< DMA transfer complete callback         */

  void                       (* XferHalfCpltCallback)( struct __DMA_HandleTypeDef * hdma);     /*!< DMA Half transfer complete callback    */

  void                       (* XferM1CpltCallback)( struct __DMA_HandleTypeDef * hdma);       /*!< DMA transfer complete Memory1 callback */
  
  void                       (* XferM1HalfCpltCallback)( struct __DMA_HandleTypeDef * hdma);   /*!< DMA transfer Half complete Memory1 callback */
  
  void                       (* XferErrorCallback)( struct __DMA_HandleTypeDef * hdma);        /*!< DMA transfer error callback            */
  
  void                       (* XferAbortCallback)( struct __DMA_HandleTypeDef * hdma);        /*!< DMA transfer Abort callback            */  

  __IO uint32_t              ErrorCode;                                                        /*!< DMA Error code                          */
  
  uint32_t                   StreamBaseAddress;                                                /*!< DMA Stream Base Address                */

  uint32_t                   StreamIndex;                                                      /*!< DMA Stream Index                       */
 
}DMA_HandleTypeDef;

#define DMA_PERIPH_TO_MEMORY          0x00000000U                 /*!< Peripheral to memory direction */
#define DMA_MEMORY_TO_PERIPH          ((uint32_t)DMA_SxCR_DIR_0)  /*!< Memory to peripheral direction */
#define DMA_MEMORY_TO_MEMORY          ((uint32_t)DMA_SxCR_DIR_1)  /*!< Memory to memory direction     */

#define DMA1                ((DMA_TypeDef *) DMA1_BASE)
#define DMA1_Stream0        ((DMA_Stream_TypeDef *) DMA1_Stream0_BASE)
#define DMA1_Stream1        ((DMA_Stream_TypeDef *) DMA1_Stream1_BASE)
#define DMA1_Stream2        ((DMA_Stream_TypeDef *) DMA1_Stream2_BASE)
#define DMA1_Stream3        ((DMA_Stream_TypeDef *) DMA1_Stream3_BASE)
#define DMA1_Stream4        ((DMA_Stream_TypeDef *) DMA1_Stream4_BASE)
#define DMA1_Stream5        ((DMA_Stream_TypeDef *) DMA1_Stream5_BASE)
#define DMA1_Stream6        ((DMA_Stream_TypeDef *) DMA1_Stream6_BASE)
#define DMA1_Stream7        ((DMA_Stream_TypeDef *) DMA1_Stream7_BASE)
#define DMA2                ((DMA_TypeDef *) DMA2_BASE)
#define DMA2_Stream0        ((DMA_Stream_TypeDef *) DMA2_Stream0_BASE)
#define DMA2_Stream1        ((DMA_Stream_TypeDef *) DMA2_Stream1_BASE)
#define DMA2_Stream2        ((DMA_Stream_TypeDef *) DMA2_Stream2_BASE)
#define DMA2_Stream3        ((DMA_Stream_TypeDef *) DMA2_Stream3_BASE)
#define DMA2_Stream4        ((DMA_Stream_TypeDef *) DMA2_Stream4_BASE)
#define DMA2_Stream5        ((DMA_Stream_TypeDef *) DMA2_Stream5_BASE)
#define DMA2_Stream6        ((DMA_Stream_TypeDef *) DMA2_Stream6_BASE)

#define DMA_CHANNEL_0                 0x00000000U    /*!< DMA Channel 0 */
#define DMA_CHANNEL_1                 0x02000000U    /*!< DMA Channel 1 */
#define DMA_CHANNEL_2                 0x04000000U    /*!< DMA Channel 2 */
#define DMA_CHANNEL_3                 0x06000000U    /*!< DMA Channel 3 */
#define DMA_CHANNEL_4                 0x08000000U    /*!< DMA Channel 4 */
#define DMA_CHANNEL_5                 0x0A000000U    /*!< DMA Channel 5 */
#define DMA_CHANNEL_6                 0x0C000000U    /*!< DMA Channel 6 */
#define DMA_CHANNEL_7                 0x0E000000U    /*!< DMA Channel 7 */

#define RCC_DMA2_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCC_REGS->RCC_AHB1ENR, RCC_AHB1ENR_DMA2EN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC_REGS->RCC_AHB1ENR, RCC_AHB1ENR_DMA2EN);\
                                        UNUSED(tmpreg); \
                                          } while(0U)

#define RCC_AHB1ENR_DMA2EN_Pos             (22U)                               
#define RCC_AHB1ENR_DMA2EN_Msk             (0x1UL << RCC_AHB1ENR_DMA2EN_Pos)    /*!< 0x00400000 */
#define RCC_AHB1ENR_DMA2EN                 RCC_AHB1ENR_DMA2EN_Msk         

#define DMA_ENABLE(__HANDLE__)      ((__HANDLE__)->Instance->CR |=  DMA_SxCR_EN)

#define DMA_DISABLE(__HANDLE__)     ((__HANDLE__)->Instance->CR &=  ~DMA_SxCR_EN)

#define DMA_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)  (((__INTERRUPT__) != DMA_IT_FE)? \
                                                        ((__HANDLE__)->Instance->CR & (__INTERRUPT__)) : \
                                                        ((__HANDLE__)->Instance->FCR & (__INTERRUPT__)))
extern DMA_HandleTypeDef hdma_spi1_rx;

extern DMA_HandleTypeDef hdma_spi1_tx;

StatusTypeDef DMA_Init(DMA_HandleTypeDef *hdma);
static uint32_t DMA_CalcBaseAndBitshift(DMA_HandleTypeDef *hdma);
void DMA_IRQHandler(DMA_HandleTypeDef *hdma);
StatusTypeDef DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
static void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
static StatusTypeDef DMA_CheckFifoParam(DMA_HandleTypeDef *hdma);

#endif /*DMA_H_*/