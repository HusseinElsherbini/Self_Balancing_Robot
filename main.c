#include "main.h"

//semihosting init function
extern void initialise_monitor_handles(void);

// initialize global timers 
global_timers_t global_timers = {
	.timer_flags.timer = {RDY},
	.timers            = {0}
};

int main(void)
{
	initialise_monitor_handles();

	printf("Implementation of a task scheduler\n");

	//DISABLE_WB();

	init_schedulaer_stack(SCHED_STACK_START);

	init_tasks_stack();
	
	sw_hw_init();

	pwmSetDutyCycle(&pwmA, (float)50.0);
	pwmSetDutyCycle(&pwmB, (float)50.0);
	
	switch_sp_to_psp();	

	// start scheduler context switching 
	sysFlags.startScheduler = true;

	task1_handler();

	for(;;);
	
	
}

void task1_handler(void)	
{
	// retrieve raw data from imu 
	while(1){

		led_on(&yellow_led);
		// check if new packet received
		while(!getDMPpacket(&i2c1, &mpu6050_raw_data.raw_data, DMP_PACKET_SIZE, false));
		//get_raw_measurements(&i2c1, false);
		lock_task(IMU_RETRIEVE_RAW_DATA);
		//process_raw_measurements();
		led_off(&yellow_led);
		task_delay(DELAY_COUNT_1MS*5U, (uint8_t)UNLOCKED);
		
	}
}
void task2_handler(void)
{
	while(1){
		led_on(&blue_led);
		task_delay(DELAY_COUNT_500MS, (uint8_t)UNLOCKED);
		led_off(&blue_led);
		task_delay(DELAY_COUNT_500MS, (uint8_t)UNLOCKED);
	}
}
void task3_handler(void)
{
	while(1){
		led_on(&red_led);
		task_delay(DELAY_COUNT_250MS, (uint8_t)UNLOCKED);
		led_off(&red_led);
		task_delay(DELAY_COUNT_250MS, (uint8_t)UNLOCKED);
	}
}

void task4_handler(void)
{
	while(1){
		checkEncoderStateChange(&rEncoder);
		if(rEncoder.stateChanged){
		}
	}

}

void task5_handler(void)
{
	while(1){
		for(int i = 0; i < MAX_TASKS - 2; i++){
			taskStackTraceDepth(&user_tasks[i]);
			task_delay(DELAY_COUNT_1S, (uint8_t)UNLOCKED);
		}
	}

}
void task6_handler(void)
{
	while(1){
		__asm("nop");
	}
}
__attribute__((naked)) void PendSV_Handler(void){

	// when a function is given the attribute "naked" the user must use BX instruction to return back
	__asm("nop");
	/* Save the context of the running task
	 * 1. Get current running task's PSP value
	 * 2. Using that PSP value store stack frame 2 (R4-R11)
	 * 3. Save the current value of the PSP
	 */
	__asm volatile ("PUSH {LR}");
	__asm volatile ("MRS R0, PSP");
	__asm volatile ("STMDB R0!, {R4-R11}");
	__asm volatile ("BL save_psp_value");

	/* Retrieve the contect of the current task
	 * 1. Decide next task to run
	 * 2. get its past PSP value
	 * 3. Using that PSP value retrieve it's SF2 (R4-R11)
	 * 4. Update PSP and exit
	 */
	__asm volatile ("BL update_next_task");
	__asm volatile ("BL get_psp_value");
	__asm volatile ("LDMIA R0!, {R4-R11}");
	__asm volatile ("MSR PSP, R0");
	__asm volatile ("POP {LR}");
	__asm volatile ("BX LR");

}
void SysTick_Handler(void){


	// manage global timers
	for(int i = 0; i < sizeof(global_timers.timers)/sizeof(global_timers.timers[0]); i++){

		if (global_timers.timers[i] > 0){
			  global_timers.timers[i]--;
			  global_timers.timer_flags.timer[i] = IN_PROGRESS;
		}
		else if(global_timers.timers[i] == 0 && global_timers.timer_flags.timer[i] == IN_PROGRESS){

			global_timers.timer_flags.timer[i] = EXPIRED;	
		}
	}
	// only run schedular if flag is set
	if(sysFlags.startScheduler == true){
		// update global tick count
		update_global_tick_count();

		// unblock tasks that can run
		unblock_tasks();

		// pend the pendsv exception
		*(cortexPperiphs.pICSR) |= ( 1 << 28 );
	}

}

__attribute__((naked)) void MemManage_Handler(void){
	// capture the stack pointer value by using a naked function, this function removes epilogue/prologue which manipulate the stack pointer value
	__asm ("MRS R0, MSP");
	__asm ("B MemManage_Handler_c");

}

void MemManage_Handler_c(uint32_t *pMSP){

	// read the MEM MANAGE FAULT STATUS REGISTER
	uint32_t *pMMSR = (uint32_t*)0xE000ED28;
	for(int i =0; i < 8; i++){
		stack_frame.sf[i] = pMSP[i];
	}
	// read the MEM MANAGE FAULT ADDRESS REGISTER
	uint32_t *pMMAR = (uint32_t*)0xE000ED34;
	__asm("nop");;

}
__attribute__((naked)) void BusFault_Handler(void){
	// capture the stack pointer value by using a naked function, this function removes epilogue/prologue which manipulate the stack pointer value
	__asm ("MRS R0, MSP");
	__asm ("B BusFault_Handler_c");

}

void BusFault_Handler_c(uint32_t *pMSP){

	// read the BUS FAULT STATUS REGISTER
	uint32_t pBFSR = *((uint32_t*)0xE000ED29);
	for(int i =0; i < 8; i++){
		stack_frame.sf[i] = pMSP[i];
	}
	// read the BUS FAULT ADDRESS REGISTER
	uint32_t pBFAR = *((uint32_t*)0xE000ED38);
	__asm("nop");

}
__attribute__((naked)) void UsageFault_Handler(void){
	// capture the stack pointer value by using a naked function, this function removes epilogue/prologue which manipulate the stack pointer value
	__asm ("MRS R0, MSP");
	__asm ("B UsageFault_Handler_c");

}
void UsageFault_Handler_c(uint32_t *pMSP){

	// read the USAGE FAULT STATUS REGISTER
	uint32_t *pUFSR = (uint32_t*)0xE00ED2A;
	for(int i =0; i < 8; i++){
		stack_frame.sf[i] = pMSP[i];
	}
	__asm("nop");

}

__attribute__((naked)) void HardFault_Handler(void){
	// capture the stack pointer value by using a naked function, this function removes epilogue/prologue which manipulate the stack pointer value
	__asm ("MRS R0, MSP");
	__asm ("B HardFault_Handler_c");

}

void HardFault_Handler_c(uint32_t *pMSP){

	for(int i =0; i < 8; i++){
		stack_frame.sf[i] = pMSP[i];
	}

	__asm("nop");

}