#include "led.h"
#include "main.h"
#include "task_scheduler.h"
#include "platform.h"
#include "imu.h"

uint8_t current_task = 0;
uint32_t gTickCount = 0;

TCB_t user_tasks[MAX_TASKS] = {0};

void (*taskHandlers[MAX_TASKS])(void) = {
	
	task1_handler,
	task2_handler,
	task3_handler,
	task4_handler,
	task5_handler,
	task6_handler

};

void unblock_tasks(void){

	for(int i = 0; i < MAX_TASKS - 1; i++){

		if(user_tasks[i].current_state != TASK_READY_STATE){
			if((user_tasks[i].block_count == gTickCount && user_tasks[i].lock == UNLOCKED) || 
		               (user_tasks[i].block_count == 0 && user_tasks[i].lock == UNLOCKED))
			{
				if(i == 0){
					asm("nop");
				}
				user_tasks[i].current_state = TASK_READY_STATE;
			}
		}
	}
}
__attribute__((naked)) void init_schedulaer_stack(uint32_t sched_stack_start){

	// load MSP register with address of top of stack
	__asm volatile("MSR MSP, %0": : "r" (sched_stack_start) :);
	__asm volatile("BX LR");
}

void init_tasks_stack(void){

	// initialize task parameters
	for(int i = 0; i < MAX_TASKS; i++){

		user_tasks[i].current_state 		= TASK_READY_STATE;          // set initial state to READY
		user_tasks[i].lock          		= UNLOCKED;					 // set task lock to UNLOCKED
		user_tasks[i].psp_value     		= TASK_STACK_START(i);   // set task's stack pointer to it's start address
		user_tasks[i].taskStackStartAddress = TASK_STACK_START(i);   // save stack start address
		user_tasks[i].task_handler 			= taskHandlers[i];           // set task handler pointer to proper handler 

	}

	// initialize private stack area of each task with a DUMMY stack frame
	uint32_t *psp;

	for(int i=0; i < MAX_TASKS; i++){

		psp = (uint32_t*)user_tasks[i].psp_value;

		// push XPSR onto stack
		psp--;
		*psp = DUMMY_XPSR; // 0x01000000 T bit has to be set

		// push PC onto stack and load it with value of it's respective task handler address
		psp--;
		*psp = (uint32_t)user_tasks[i].task_handler;

		// push LR register onto stack and load it with value of EXC_RETURN CODE 0xFFFFFFFD
		psp--;
		*psp = (uint32_t)0xFFFFFFFD;

		// push R0-R12 onto stack and intitialize them with zeros
		for(int j = 0; j < 13; j++){

			psp--;
			*psp = (uint32_t)0x0;
		}
		user_tasks[i].psp_value = (uint32_t)psp;

		// set rest of task stack space with unique pattern for task depth tracing
		while(psp != (user_tasks[i].taskStackStartAddress - TASK_STACK_SIZE)){
			psp--;
			*psp = 0xF3F4F5F6;
		}
		
	}

}

void update_global_tick_count(void){
	gTickCount++;
}

uint32_t get_global_tick_count(void){

	return gTickCount;
}
void schedule(void){
	// pend the pendsv exception
	uint32_t *pICSR = (uint32_t *)0xE000ED04;
	*pICSR |= ( 1 << 28 );
}

uint32_t get_psp_value(void){

	return user_tasks[current_task].psp_value;
}

void save_psp_value(uint32_t current_psp_value){
	user_tasks[current_task].psp_value = current_psp_value;
}

void update_next_task(void){

	int state = TASK_BLOCKED_STATE;
	// check if
	for(int i = 0; i < MAX_TASKS; i++){
		current_task++;
		current_task %= MAX_TASKS;
		state = user_tasks[current_task].current_state;
		if( (state == TASK_READY_STATE) && (current_task != MAX_TASKS - 1) ){
			if(i == 0){
				asm("nop");
			}
			break;
		}
	}
	if(state != TASK_READY_STATE){
		current_task = MAX_TASKS - 1;
	}

}

__attribute__((naked)) void switch_sp_to_psp(void){

	// initialize psp with TASK1 stack start
	__asm volatile ("PUSH {LR}"); // the next instruction will branch to a seperate function and corrupt the current LR register, save to stack
	__asm volatile ("BL get_psp_value"); // branch with link to the get psp function to retrieve the task psp stack val
	__asm volatile ("MSR PSP, R0"); // when the previous branch function ran, it's return was stored in R0 according to cortex standard
	__asm volatile ("POP {LR}"); // POP the LR value previously stored on the stack back

	//change sp to psp using CONTROL register
	__asm volatile ("MOV R0, #0X02");   // push the immediate value of 2 into R0
	__asm volatile ("MSR CONTROL, R0"); // move content of R0 into special register CONTROL

	// return to main
	__asm volatile ("BX LR");
}

void lock_task(uint8_t task_num){

	INTERRUPT_DISABLE();
	user_tasks[current_task].block_count = 0U;
	user_tasks[task_num].lock = LOCKED;
	user_tasks[task_num].current_state = TASK_BLOCKED_STATE;
	schedule();
	INTERRUPT_ENABLE();

}
void task_delay(uint32_t tick_no, uint8_t lock_state){

	// disable interrupt to avoid race condition
	INTERRUPT_DISABLE();
	if(current_task != 3){
		user_tasks[current_task].block_count = gTickCount + tick_no;
		user_tasks[current_task].lock = lock_state;
		user_tasks[current_task].current_state = TASK_BLOCKED_STATE;
		schedule();
	}
	// reenable interrupts
	INTERRUPT_ENABLE();
}

void taskStackTraceDepth(TCB_t * userTask){

	uint32_t *psp = userTask->taskStackStartAddress;
	uint32_t byteCount = 0;
	// calculate maximum number of bytes task used 
	while(psp != (userTask->taskStackStartAddress - TASK_STACK_SIZE)){
		psp--;
		if(*psp == 0xF3F4F5F6 ){
			if(byteCount > userTask->maxStackSpaceUsed){
				userTask->maxStackSpaceUsed = byteCount;
			}
			break;
		}
		else{
			byteCount += 4;
		}

	}
}