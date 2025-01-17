
#include "task_macros.h"
#include "timer_common.h"
#include "SEGGER_RTT.h"
#include "platform.h"
#include "logger.h"

static uint64_t previous_runtimes[4] = {0};  // Assuming max 4 tasks
// Task analysis system
#define BUFFER_SIZE_UP 1024  // Size for each buffer

// Define buffers for up (target to host) channels
static char UpBuffer0[BUFFER_SIZE_UP];
static char UpBuffer1[BUFFER_SIZE_UP];

void configureStatsTimer(void)
{

    // Enable TIM2 clock
    RCC_REGS->RCC_APB1ENR |= RCC_APB1ENR_TIM2EN;
    
    // Reset timer
    TIM2->CR1 = 0;
    TIM2->PSC = 0;          // No prescaling - max resolution
    TIM2->ARR = 0xFFFFFFFF; // Maximum period (32-bit)
    
    // Start timer - just enable counter
    TIM2->CR1 |= TIM_CR1_CEN;

}

void init_rtt_channels(void)
{
    // Configure buffers with explicit sizes
    int ret1 = SEGGER_RTT_ConfigUpBuffer(MONITOR_TERMINAL, "Monitor", UpBuffer0, BUFFER_SIZE_UP, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    int ret2 = SEGGER_RTT_ConfigUpBuffer(IMU_DATA_TERMINAL, "IMU Data", UpBuffer1, BUFFER_SIZE_UP, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    
    SEGGER_RTT_printf(0, "RTT Config returns: %d, %d\n", ret1, ret2);
    
    // Check available space
    int space0 = SEGGER_RTT_GetAvailWriteSpace(MONITOR_TERMINAL);
    int space1 = SEGGER_RTT_GetAvailWriteSpace(IMU_DATA_TERMINAL);
    SEGGER_RTT_printf(0, "Channel 0 space: %d\n", space0);
    SEGGER_RTT_printf(0, "Channel 1 space: %d\n", space1);
}

uint32_t getStatsTimerCount(void)
{
    return TIM2->CNT;
}

// Function to analyze all tasks
TaskAnalysis_t* getTaskAnalysis(void)
{
    static TaskAnalysis_t analysis;
    static TaskStatus_t taskStatusArray[15];  // Adjust size based on max tasks
    
    analysis.arraySize = uxTaskGetNumberOfTasks();
    analysis.taskStatusArray = taskStatusArray;
    
    // Get task information
    uxTaskGetSystemState(analysis.taskStatusArray, 
                        analysis.arraySize, 
                        &analysis.totalRunTime);
                        
    return &analysis;
}

// Stack high water mark monitoring
void checkTaskStacks(void)
{
    TaskAnalysis_t* analysis = getTaskAnalysis();
    
    for(uint32_t i = 0; i < analysis->arraySize; i++)
    {
        TaskStatus_t* task = &analysis->taskStatusArray[i];
        uint32_t stackHighWaterMark = uxTaskGetStackHighWaterMark(task->xHandle);
        
        // Convert words to bytes (1 word = 4 bytes)
        uint32_t stackFreeBytes = stackHighWaterMark * 4;

        SEGGER_RTT_printf(0, "Task: %s, Minimum Free Stack: %lu bytes\n", 
               task->pcTaskName, 
               stackFreeBytes);
    }
}

void checkHeapStatus(void)
{
    size_t freeHeap = xPortGetFreeHeapSize();
    size_t minEverFree = xPortGetMinimumEverFreeHeapSize();
    

    // Convert to KB if > 1024 bytes
    if(freeHeap >= 1024) {
        SEGGER_RTT_printf(0, "Current Free Heap: %lu.%lu KB\n", 
                         freeHeap/1024, (freeHeap%1024)*100/1024);
    } else {
        SEGGER_RTT_printf(0, "Current Free Heap: %lu bytes\n", freeHeap);
    }
    
    if(minEverFree >= 1024) {
        SEGGER_RTT_printf(0, "Minimum Ever Free Heap: %lu.%lu KB\n", 
                         minEverFree/1024, (minEverFree%1024)*100/1024);
    } else {
        SEGGER_RTT_printf(0, "Minimum Ever Free Heap: %lu bytes\n", minEverFree);
    }
}

// Task timing analysis
void printTaskTiming(void)
{
   TaskAnalysis_t* analysis = getTaskAnalysis();
   static uint64_t previous_runtimes[4] = {0};
   uint64_t total_delta = 0;
   uint64_t current_runtimes[4] = {0};
   uint64_t deltas[4] = {0};
   static uint32_t first_run = 1;

   // Get current values and calculate deltas over 5 second period
    for(uint32_t i = 0; i < analysis->arraySize; i++)
    {
        current_runtimes[i] = analysis->taskStatusArray[i].ulRunTimeCounter;
        
        // Handle counter overflow or first run
        if(first_run || current_runtimes[i] < previous_runtimes[i]) {
            // Reset all previous runtimes and return
            for(uint32_t j = 0; j < analysis->arraySize; j++) {
                previous_runtimes[j] = current_runtimes[j];
            }
            first_run = 0;
            SEGGER_RTT_printf(0, "Counter overflow detected, resetting measurements\n");
            return;
        }
        
        deltas[i] = current_runtimes[i] - previous_runtimes[i];
        total_delta += deltas[i];
        previous_runtimes[i] = current_runtimes[i];
    }
   SEGGER_RTT_printf(0, "\nTask Timing Analysis (Last 5 seconds):\n");
   for(uint32_t i = 0; i < analysis->arraySize; i++)
   {
       TaskStatus_t* task = &analysis->taskStatusArray[i];
       uint32_t percentage = 0;
       
       if(total_delta > 0)
       {
           percentage = (uint32_t)((deltas[i] * 10000ULL) / total_delta);
       }

       // Convert timer ticks to microseconds
       // SystemCoreClock ticks per second, we want microseconds over 5 seconds
       uint32_t runtime_us = (uint32_t)((deltas[i] * 1000000ULL) / (HSE_CLOCK));
       
       SEGGER_RTT_printf(0, "Task: %s\n", task->pcTaskName);
       if(runtime_us < 1000) {
           SEGGER_RTT_printf(0, "  Total Runtime: %lu us (%lu.%02lu%%)\n", 
                  runtime_us,
                  percentage / 100,
                  percentage % 100);
       } else {
           SEGGER_RTT_printf(0, "  Total Runtime: %lu.%03lu ms (%lu.%02lu%%)\n", 
                  runtime_us / 1000,
                  runtime_us % 1000,
                  percentage / 100,
                  percentage % 100);
       }
       
       // For IMU task, show average time per iteration
       if(strcmp(task->pcTaskName, "IMU") == 0) {
           // Number of iterations in 5 seconds
           uint32_t iterations = 5000 / BALANCING_TASK_PERIOD_MS;  // Should be 1000
           uint32_t avg_us = runtime_us / iterations;
           SEGGER_RTT_printf(0, "  Avg per iteration: %lu us\n", avg_us);
       }

       SEGGER_RTT_printf(0, "  Priority: %lu\n", task->uxCurrentPriority);
       SEGGER_RTT_printf(0, "  State: %d\n", task->eCurrentState);
       
       previous_runtimes[i] = current_runtimes[i];
   }

   // Add total measured time for validation
   uint32_t total_us = (uint32_t)((total_delta * 1000000ULL) / (HSE_CLOCK));
   SEGGER_RTT_printf(0, "\nTotal measured time: %lu.%03lu ms\n", 
                    total_us / 1000, 
                    total_us % 1000);
}

// Stack overflow hook
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{

    DEBUG_WARN("Stack overflow in task: %s\n", pcTaskName);
    // Maybe blink LED or take other action
    while(1);
}

// Malloc failed hook
void vApplicationMallocFailedHook(void)
{

    SEGGER_RTT_printf(0, "Malloc failed!\n");
    // Maybe blink LED or take other action
    while(1);
}

void vApplicationIdleHook(void)
{
    /* Called every iteration of idle task */
    
    #if (configUSE_IDLE_HOOK == 1)
        /* Feed the watchdog if you have one */
        
        /* Can be used for low power modes */
        
        /* Let RTT flush its buffer */

        //SEGGER_RTT_printf(0, "");


    #endif
}