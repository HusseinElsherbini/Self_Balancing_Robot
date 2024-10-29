#include "cortex.h"
#include "platform.h"
#include "task_scheduler.h"

sysFlags_t sysFlags = {
	.startScheduler = false,
};

struct cortex_pp cortexPperiphs = {

    .pSTCSR = (uint32_t*)0xE000E010,
    .pSTRVR = (uint32_t*)0xE000E014,
    .pSTCVR = (uint32_t*)0xE000E018,
    .pICSR  = (uint32_t*)0xE000ED04,
    .pSHCSR = (uint32_t*)0xE000ED24,
	.pISER  = (uint32_t*)0xE000E100,
	.pACTLR = (uint32_t*)0xE000E008,
	.pSCCR  = (uint32_t*)0xE000ED14,

};

stack_frame_t stack_frame = {0};

void init_systick_timer(uint32_t tick_hz){


	uint32_t count_value = ((SYSTICK_TIM_CLK/tick_hz) - 1);

	// clear the value of SVR
	*(cortexPperiphs.pSTRVR) &= ~(0x00FFFFFF);

	// load SVR with desired value
	*(cortexPperiphs.pSTRVR) |= count_value;

	// enable systick exception request
	*(cortexPperiphs.pSTCSR) |= (1 << 1);

	// select processor clock as clock source
	*(cortexPperiphs.pSTCSR) |= (1 << 2);

	// enable counter
	*(cortexPperiphs.pSTCSR) |= (1 << 0);

}

void enable_configurable_sys_faults(void){

	// enable USAGE FAULT, BUS FAULT and MEM MANAGE FAULT exceptions
	*(cortexPperiphs.pSHCSR) |= (1 << USGFAULTENABLE) | (1 << BUSFAULTENABLE) | (1 << MEMFAULTENABLE);

	// enable div by zero trap 

	*(cortexPperiphs.pSCCR) |= (1 << 4U);

}

void __NVIC_EnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}