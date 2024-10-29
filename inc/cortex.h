#ifndef CORTEX_H_
#define CORTEX_H_
#include <stdint.h>
#include "math.h"
#include "stdbool.h"
#include "platform.h"
// processor fault macros
#define USGFAULTENABLE 18
#define BUSFAULTENABLE 17
#define MEMFAULTENABLE 16

#define INTERRUPT_DISABLE() do{__asm volatile ("MOV R0,#0x1"); __asm volatile("MSR PRIMASK, R0"); } while(0);
#define INTERRUPT_ENABLE()  do{__asm volatile ("MOV R0,#0x0"); __asm volatile("MSR PRIMASK, R0"); } while(0);

#ifdef __cplusplus
  #define   __I     volatile             /*!< Defines 'read only' permissions */
#else
  #define   __I     volatile const       /*!< Defines 'read only' permissions */
#endif
#define     __O     volatile             /*!< Defines 'write only' permissions */
#define     __IO    volatile             /*!< Defines 'read / write' permissions */

/* following defines should be used for structure members */
#define     __IM     volatile const      /*! Defines 'read only' structure member permissions */
#define     __OM     volatile            /*! Defines 'write only' structure member permissions */
#define     __IOM    volatile            /*! Defines 'read / write' structure member permissions */

#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address */
#define ITM_BASE            (0xE0000000UL)                            /*!< ITM Base Address */
#define DWT_BASE            (0xE0001000UL)                            /*!< DWT Base Address */
#define TPI_BASE            (0xE0040000UL)                            /*!< TPI Base Address */
#define CoreDebug_BASE      (0xE000EDF0UL)                            /*!< Core Debug Base Address */
#define SysTick_BASE        (SCS_BASE +  0x0010UL)                    /*!< SysTick Base Address */
#define NVIC_BASE           (SCS_BASE +  0x0100UL)                    /*!< NVIC Base Address */
#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address */

struct cortex_pp{
    volatile uint32_t* pSTCSR; // SysTick control and status register
    volatile uint32_t* pSTRVR; // SysTick reload value register
    volatile uint32_t* pSTCVR; // Systick current value register
    volatile uint32_t* pICSR;  // Interrupt Control and State Register
    volatile uint32_t* pSHCSR;  // System Handler Control and State Register
    volatile uint32_t* pISER[7];
    volatile uint32_t* pACTLR;  // Auxilliary control register 
    volatile uint32_t* pSCCR; 
};

typedef struct{

    volatile bool startScheduler;

}sysFlags_t;

extern sysFlags_t sysFlags;
extern struct cortex_pp cortexPperiphs; 

#define DISABLE_WB() *(cortexPperiphs.pACTLR) |= (1U << 1U) // disables write buffer use for access to default memory 

typedef union{
    volatile uint32_t sf[8];
    struct{
        volatile uint32_t  R0; 
        volatile uint32_t  R1; 
        volatile uint32_t  R2; 
        volatile uint32_t  R3; 
        volatile uint32_t  R12;
        volatile uint32_t  LR; 
        volatile uint32_t  PC;
        volatile uint32_t  xPSR;
    };
}stack_frame_t;

typedef struct
{
  __IOM uint32_t ISER[8U];               /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
        uint32_t RESERVED0[24U];
  __IOM uint32_t ICER[8U];               /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
        uint32_t RSERVED1[24U];
  __IOM uint32_t ISPR[8U];               /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
        uint32_t RESERVED2[24U];
  __IOM uint32_t ICPR[8U];               /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
        uint32_t RESERVED3[24U];
  __IOM uint32_t IABR[8U];               /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */
        uint32_t RESERVED4[56U];
  __IOM uint8_t  IP[240U];               /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
        uint32_t RESERVED5[644U];
  __OM  uint32_t STIR;                   /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */
}  NVIC_Type;

#define NVIC                ((NVIC_Type      *)     NVIC_BASE     )   /*!< NVIC configuration struct */

static inline void __disable_irq(void)
{
  __asm volatile ("cpsid i" : : : "memory");
}

extern stack_frame_t stack_frame;
void __NVIC_EnableIRQ(IRQn_Type IRQn);
void enable_configurable_sys_faults(void);
void init_systick_timer(uint32_t tick_hz);



#endif /*CORTEX_H_*/