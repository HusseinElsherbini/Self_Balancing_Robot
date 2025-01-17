#include "main.h"

// initialize global timers 
global_timers_t global_timers = {
	.timer_flags.timer = {RDY},
	.timers            = {0}
};

volatile TaskHandle_t xBalancingTaskHandle = NULL;
volatile TaskHandle_t xRcTaskHandle = NULL;
volatile TaskHandle_t xMonitorTaskHandle = NULL;
volatile TaskHandle_t xLogTaskHandle = NULL;

QueueHandle_t xImuDataQueue = NULL;
QueueHandle_t xRcCommandsQueue = NULL;
QueueHandle_t xMotorCommandsQueue = NULL;
QueueHandle_t xLogQueue = NULL;


int main(void)
{

	SEGGER_RTT_printf(0, "Robot Coming to life! \n");
	// disable write buffer use for access to default memory 
	DISABLE_WB();

	// enable fpu
	enableFPU();

	sw_hw_init();
	
	// stop motors
	actuateMotor(&xMotorBHandle, MOTOR_STOP, (uint32_t)0);
	actuateMotor(&xMotorAHandle, MOTOR_STOP , (uint32_t)0);
    
	// moving to TIM5 as main timer
	disable_systick_timer();

	// start scheduler context switching 
    xTaskCreate(vBalancingTask,                // Task function
                "Balancing",                   // Task name
                BALANCING_TASK_STACK_SIZE,     // Stack size
                NULL,                          // Parameters
                TASK_PRIORITY_BALANCE,         // Priority
                NULL);                         // Task handle
	/*
	xTaskCreate(vRcTask,              	   // Task function
				"RC",                      // Task name
				RC_TASK_STACK_SIZE,  	   // Stack size
				NULL,                 	   // Parameters
				TASK_PRIORITY_RC_RECEIVE,  // Priority
				NULL);                	   // Task handle
*/
#if USE_TASK_ANALYSIS
	xTaskCreate(vMonitorTask,          // Task function
				"Monitor",             // Task name
				MONITOR_STACK_SIZE,    // Stack size
				NULL,                  // Parameters
				TASK_PRIORITY_LOGGING, // Priority
				NULL);                 // Task handle
#endif
#if DEBUG_LEVEL
    DEBUG_INIT();
    xLogQueue = xQueueCreate(1, sizeof(balance_log_data_t));
    
    // Create logging task with lower priority than balance task
    xTaskCreate(vLogTask,
                "Log",
                LOG_CRITICAL_DATA_TASK_STACK_SIZE,
                NULL,
                TASK_PRIORITY_LOGGING,  // Lower priority than balance task
                NULL);
#endif
    // Start the scheduler
    vTaskStartScheduler();
	for(;;);
	
	
}

void TIM5_IRQHandler(void)
{
    // Clear the interrupt flag
    CLEAR_BIT(htim5.timer_config.timer_base_address->SR, TIM_SR_UIF);
	

    // Call FreeRTOS tick handler if scheduler has started
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        xPortSysTickHandler();
    }
}

void vBalancingTask(void *pvParameters)
{
	// set task handle
	xBalancingTaskHandle = xTaskGetCurrentTaskHandle();
	balancingTaskData.targetAngle = 0.0f;  // Fixed target angle for upright position
	balance_log_data_t log_data;

    balancingTaskData.BALANCING_TASK_STATE = BALANCING_TASK_RUNNING;
	// initialize last wake time
	balancingTaskData.xLastWakeTime = xTaskGetTickCount();

	while(1){
        
        switch(balancingTaskData.BALANCING_TASK_STATE){

            case BALANCING_TASK_RUNNING:
                // handle balancing
                robotBalance(&balancingTaskData);
                break;

            case BALANCING_TASK_STANDBY:
                // handle standby
                robotStandby(&balancingTaskData);
                break;

            case BALANCING_TASK_ERROR:
                // handle error
                robotError(&balancingTaskData);
                //BALANCING_TASK_STATE = BALANCING_TASK_READY;
                break;

            default:
                // do something else
                break;
        }
        if(balancingTaskData.BALANCING_TASK_STATE != BALANCING_TASK_ERROR){
            // Prepare log data
            log_data.angle = balancingTaskData.mpu6050_data->processedData.processedData.angle;   // Current angle
            log_data.angular_vel = balancingTaskData.mpu6050_data->processedData.processedData.angular_velocity;  // Angular velocity 
            log_data.pid_output = balancingTaskData.pidOutput;  // PID output   
            xQueueOverwrite(xLogQueue, &log_data);
        }
        vTaskDelayUntil(&balancingTaskData.xLastWakeTime, pdMS_TO_TICKS(BALANCING_TASK_PERIOD_MS));    
	}
}
void vRcTask(void *pvParameters)
{

    xRcTaskHandle = xTaskGetCurrentTaskHandle();
    TickType_t xLastWakeTime;
    SBUS_Packet_t sbusPacket;
    sport_data_frame_t telemetryFrame = {
        .header = SPORT_START_DATA
    };
   
    xLastWakeTime = xTaskGetTickCount();
   
    while(1) {
        // Wait for either SBUS or SmartPort events
        uint32_t notificationValue = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(20));
        
        // Check which event occurred
        if(notificationValue & NOTIFY_SBUS_DATA) {
            // Process SBUS data
            sbusPacket.lastUpdate = xTaskGetTickCount();
            memcpy(sbusPacket.rawData, sbusHandle.packetBuffer, sizeof(sbusHandle.packetBuffer));
            SBUS_decode(&sbusPacket);
            print_sbus_data(&sbusPacket);
        }
        
        if(notificationValue & NOTIFY_SPORT_POLL) {
			/*
            // Immediately respond to telemetry poll
            updateTelemetryData(&telemetryFrame);
            telemetryFrame.checksum = calculateSportChecksum(&telemetryFrame);
            sendSportFrame(&telemetryFrame);*/
        }
        
        // Check for SBUS timeout regardless of which event woke us
        if((xTaskGetTickCount() - sbusPacket.lastUpdate) > pdMS_TO_TICKS(SIGNAL_TIMEOUT_MS)) {
            sbusPacket.flags.frameLost = true;
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(RC_TASK_PERIOD_MS));
    }
	
}
void vMonitorTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while(1)
    {

        SEGGER_RTT_printf(0, "\n=== System Analysis ===\n");
        // Check heap status
        checkHeapStatus();
        
        // Check stack usage
        checkTaskStacks();
        
        // Print task timing
        printTaskTiming();
        
        // Run every 5 seconds
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5000));
    }
}
void vLogTask(void *pvParameters) {

    xLogTaskHandle = xTaskGetCurrentTaskHandle();
    balance_log_data_t log_data = {0};
    BalanceLogger_t logger = {0};
    uint32_t notificationValue;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t ADC_TIMEOUT_TICKS = pdMS_TO_TICKS(20);

    // Initialize our logging system
    Logger_Init(&logger, &log_data);

    while(1) {
        // start conversions on ADC by enabling the timer
        system_sensors.port_config->Instance->CR2 |= ADC_CR2_SWSTART;
        notificationValue = ulTaskNotifyTake(pdTRUE, ADC_TIMEOUT_TICKS);
       
        if (notificationValue == NOTIFY_LOGGER_TASK_ADC) {
            // Check for new balance data without blocking

            if(xQueueReceive(xLogQueue, &log_data, 0) == pdTRUE) {
                __asm("nop");
            }
            if (!process_balance_telemetry(&log_data, &logger)) {
                DEBUG_ERROR("Failed to process telemetry");
            }
        } else {
            DEBUG_WARN("ADC timeout at %u", xTaskGetTickCount());
        }
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(LOG_TASK_PERIOD_MS));
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

void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {
  }

}
void vPortSetupTimerInterrupt(void)
{
    /* Override the default implementation of vPortSetupTimerInterrupt() */
    initTimer5();
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

}