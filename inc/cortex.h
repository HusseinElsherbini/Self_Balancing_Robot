#ifndef CORTEX_H_
#define CORTEX_H_
#include <stdint.h>
#include "math.h"
#include "stdbool.h"
#include "task_scheduler.h"

// processor fault macros
#define USGFAULTENABLE 18
#define BUSFAULTENABLE 17
#define MEMFAULTENABLE 16

#define INTERRUPT_DISABLE() do{__asm volatile ("MOV R0,#0x1"); __asm volatile("MSR PRIMASK, R0"); } while(0);
#define INTERRUPT_ENABLE()  do{__asm volatile ("MOV R0,#0x0"); __asm volatile("MSR PRIMASK, R0"); } while(0);
     
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

extern stack_frame_t stack_frame;
#define NVIC_ENABLE_IRQ(IRQn)        (*(cortexPperiphs.pISER[(int)(ceil((double)(IRQn/32)))]) |= ((uint32_t)(1 << (IRQn % 32))))
void enable_configurable_sys_faults(void);
void init_systick_timer(uint32_t tick_hz);
void taskStackTraceDepth(TCB_t * userTask);


#endif /*CORTEX_H_*/