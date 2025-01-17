#ifndef TASK_MACROS_H_
#define TASK_MACROS_H_

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// Task handles
extern volatile TaskHandle_t xBalancingTaskHandle;
extern volatile TaskHandle_t xRcTaskHandle;
extern volatile TaskHandle_t xMonitorTaskHandle;
extern volatile TaskHandle_t xLogTaskHandle;
// Queues
extern QueueHandle_t xImuDataQueue;
extern QueueHandle_t xRcCommandsQueue;
extern QueueHandle_t xMotorCommandsQueue;
extern QueueHandle_t xLogQueue;


#define NOTIFY_IMU_DATA      (1 << 3)
#define NOTIFY_SBUS_DATA     (1 << 0)    // Bit 0 for SBUS data
#define NOTIFY_SPORT_POLL    (1 << 1)    // Bit 1 for SmartPort poll

#define NOTIFY_LOGGER_TASK_ADC  (1 << 0)
// Task priorities (higher number = higher priority)
#define TASK_PRIORITY_BALANCE      4  // High - needs consistent timing
#define TASK_PRIORITY_RC_RECEIVE   3  // Medium - user input
#define TASK_PRIORITY_MGM_COMM     1  // Low - non-critical display data
#define TASK_PRIORITY_LOGGING      2  // non - critical
// Task periods
#define BALANCING_TASK_PERIOD_MS        5    // 200Hz for IMU sampling
#define RC_TASK_PERIOD_MS               5    // 200Hz RC reading
#define MGM_TASK_PERIOD_MS              100  // 10Hz display updates
#define LOG_TASK_PERIOD_MS              10   // 100Hz Logging task

// Stack sizes
#define BALANCING_TASK_STACK_SIZE            (configMINIMAL_STACK_SIZE * 2)  // Larger for filter calculations
#define RC_TASK_STACK_SIZE                    configMINIMAL_STACK_SIZE
#define MGM_TASK_STACK_SIZE                   configMINIMAL_STACK_SIZE
#define MONITOR_TASK_STACK_SIZE              (configMINIMAL_STACK_SIZE*2)
#define LOG_CRITICAL_DATA_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE*8)

// IMU task speicial defines
// Constants
#define IMU_I2C_TIMEOUT_TICKS     pdMS_TO_TICKS(3)    // 1ms timeout max
#define MPU6050_TRANSFER_TIME_US  300                 // Typical I2C transfer time for MPU6050
#define LOGGING_TIMEOUT_TICKS     pdMS_TO_TICKS(10)
// Task timing analysis and monitoring
#define MONITOR_TERMINAL            0U
#define IMU_DATA_TERMINAL           1U

typedef struct {
    TaskStatus_t *taskStatusArray;
    uint32_t arraySize;
    uint32_t totalRunTime;
} TaskAnalysis_t;


// Timer configuration
void configureStatsTimer(void);
uint32_t getStatsTimerCount(void);

// Analysis functions
TaskAnalysis_t* getTaskAnalysis(void);
void checkTaskStacks(void);
void checkHeapStatus(void);
void printTaskTiming(void);

// FreeRTOS hooks (these are usually declared in FreeRTOSConfig.h)
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
void vApplicationMallocFailedHook(void);

// Monitor task
void vMonitorTask(void *pvParameters);

void vBalancingTask(void *pvParameters);
void vRcTask(void *pvParameters);
void vLogTask(void *pvParameters);

#endif /* TASK_MACROS_H_ */