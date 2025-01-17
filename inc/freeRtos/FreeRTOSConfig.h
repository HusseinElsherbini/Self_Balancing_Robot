#ifndef FREERTOS_CONFIG_H

#define FREERTOS_CONFIG_H

/* Here is a good place to include header files that are required across

   your application. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
    #include <stdint.h>
    #define configENABLE_FPU                  1    // If using FPU
    extern uint32_t SystemCoreClock;
#endif


#define configPRIO_BITS                         4    // STM32F4 uses 4 bits for priority
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15   // Lowest priority 
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5    // Highest priority that can use FreeRTOS API


#define configUSE_PREEMPTION                                        1

#define configUSE_PORT_OPTIMISED_TASK_SELECTION                     0

#define configUSE_TICKLESS_IDLE                                     0

#define configCPU_CLOCK_HZ                                          84000000

#define configSYSTICK_CLOCK_HZ                                      1000000

#define configTICK_RATE_HZ                                          1000

#define configMAX_PRIORITIES                                        5

#define configMINIMAL_STACK_SIZE                                    128

#define configMAX_TASK_NAME_LEN                                     16

#define configIDLE_SHOULD_YIELD                                     1

#define configUSE_TASK_NOTIFICATIONS                                1

#define configTASK_NOTIFICATION_ARRAY_ENTRIES                       3

#define configUSE_MUTEXES                                           0

#define configUSE_RECURSIVE_MUTEXES                                 0

#define configUSE_COUNTING_SEMAPHORES                               0

#define configUSE_ALTERNATIVE_API                                   0 /* Deprecated! */

#define configQUEUE_REGISTRY_SIZE                                   10

#define configUSE_QUEUE_SETS                                        0

#define configUSE_TIME_SLICING                                      0

#define configUSE_NEWLIB_REENTRANT                                  0

#define configENABLE_BACKWARD_COMPATIBILITY                         0

#define configNUM_THREAD_LOCAL_STORAGE_POINTERS                     5

#define configUSE_MINI_LIST_ITEM                                    1

#define configSTACK_DEPTH_TYPE                                      uint16_t

#define configMESSAGE_BUFFER_LENGTH_TYPE                            size_t

#define configHEAP_CLEAR_MEMORY_ON_FREE                             1


/* Memory allocation related definitions. */

#define configSUPPORT_STATIC_ALLOCATION                             1

#define configSUPPORT_DYNAMIC_ALLOCATION                            1

#define configTOTAL_HEAP_SIZE                                       ( ( size_t ) ( 10 * 1024 ) )

#define configAPPLICATION_ALLOCATED_HEAP                            0

#define configSTACK_ALLOCATION_FROM_SEPARATE_HEAP                   0

#define configKERNEL_PROVIDED_STATIC_MEMORY                         1

/* Hook function related definitions. */

#define configUSE_TICK_HOOK                                 0

#define configUSE_DAEMON_TASK_STARTUP_HOOK                  0

#define configUSE_SB_COMPLETED_CALLBACK                     0


/* Co-routine related definitions. */

#define configUSE_CO_ROUTINES                               0

#define configMAX_CO_ROUTINE_PRIORITIES                     1


/* Software timer related definitions. */

#define configUSE_TIMERS                                    1

#define configTIMER_TASK_PRIORITY                           3

#define configTIMER_QUEUE_LENGTH                            10

#define configTIMER_TASK_STACK_DEPTH                        configMINIMAL_STACK_SIZE

#define configTICK_TYPE_WIDTH_IN_BITS                       TICK_TYPE_WIDTH_32_BITS

/* Interrupt nesting behaviour configuration. */

#define configKERNEL_INTERRUPT_PRIORITY         ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

#define configMAX_SYSCALL_INTERRUPT_PRIORITY    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* Define to trap errors during development. */

#define configASSERT( x )         \
    if( ( x ) == 0 )              \
    {                             \
        taskDISABLE_INTERRUPTS(); \
        for( ; ; )                \
        ;                         \
    }


/* FreeRTOS MPU specific definitions. */

#define configINCLUDE_APPLICATION_DEFINED_PRIVILEGED_FUNCTIONS 0

#define configTOTAL_MPU_REGIONS                                8 /* Default value */

#define configTEX_S_C_B_FLASH                                  0x07UL /* Default value */

#define configTEX_S_C_B_SRAM                                   0x07UL /* Default value */

#define configENFORCE_SYSTEM_CALLS_FROM_KERNEL_ONLY            1

#define configALLOW_UNPRIVILEGED_CRITICAL_SECTIONS             1

#define configENABLE_ERRATA_837070_WORKAROUND                  1


/* ARMv8-M secure side port related definitions. */

#define secureconfigMAX_SECURE_CONTEXTS         5


/* Optional functions - most linkers will remove unused functions anyway. */

#define INCLUDE_vTaskPrioritySet                1

#define INCLUDE_uxTaskPriorityGet               1

#define INCLUDE_vTaskDelete                     1

#define INCLUDE_vTaskSuspend                    1

#define INCLUDE_xResumeFromISR                  1

#define INCLUDE_vTaskDelayUntil                 1

#define INCLUDE_vTaskDelay                      1

#define INCLUDE_xTaskGetSchedulerState          1

#define INCLUDE_xTaskGetCurrentTaskHandle       1

#define INCLUDE_uxTaskGetStackHighWaterMark     1

#define INCLUDE_uxTaskGetStackHighWaterMark2    0

#define INCLUDE_eTaskGetState                   0

#define INCLUDE_xEventGroupSetBitFromISR        1

#define INCLUDE_xTimerPendFunctionCall          0

#define INCLUDE_xTaskAbortDelay                 0

#define INCLUDE_xTaskGetHandle                  0

#define INCLUDE_xTaskResumeFromISR              1

#define configOVERRIDE_DEFAULT_TICK_CONFIGURATION 1

// In FreeRTOSConfig.h
#define configUSE_TRACE_FACILITY                1    // Enable trace facility
#define configUSE_STATS_FORMATTING_FUNCTIONS    1    // Enable stats formatting
#define configGENERATE_RUN_TIME_STATS           1    // Enable runtime stats
#define configUSE_MALLOC_FAILED_HOOK            1    // Monitor heap allocation failures
#define configCHECK_FOR_STACK_OVERFLOW          2    // Enable stack overflow checking (mode 2 is more thorough)
#define configUSE_IDLE_HOOK                     1    // Use idle hook for stats

// Define how to get high resolution time for runtime stats
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()  configureStatsTimer()
#define portGET_RUN_TIME_COUNTER_VALUE()          getStatsTimerCount()


// In FreeRTOSConfig.h
#define SEGGER_SYSVIEW_ENABLE 1

// Enable FreeRTOS trace hooks
#define INCLUDE_xTaskGetIdleTaskHandle 1
#define INCLUDE_pxTaskGetStackStart    1

#include "SEGGER_SYSVIEW_FreeRTOS.h"
#endif /* FREERTOS_CONFIG_H */