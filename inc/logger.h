#ifndef LOGGER_H
#define LOGGER_H

#include "stdint.h"
#include "dma.h"
#include "SEGGER_SYSVIEW.h"
#include "balancing.h"
#include "FreeRTOS.h"
#include "queue.h"

// Master debug switch from Makefile
#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL 0
#endif

// Individual feature switches
#define DEBUG_LEVEL_NONE      0
#define DEBUG_LEVEL_MINIMAL   1  // Critical events only
#define DEBUG_LEVEL_NORMAL    2  // Regular debugging
#define DEBUG_LEVEL_VERBOSE   3  // Full system analysis

// Debug macros that compile to nothing when disabled
#if (DEBUG_LEVEL > DEBUG_LEVEL_NONE)
    #define DEBUG_INIT() do { \
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; \
        DWT->CYCCNT = 0; \
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; \
        SEGGER_RTT_Init(); \
        SEGGER_SYSVIEW_Conf(); \
        SEGGER_SYSVIEW_Start(); \
    } while(0)

    #define DEBUG_ISR_ENTER()  SEGGER_SYSVIEW_RecordEnterISR()
    #define DEBUG_ISR_EXIT()   SEGGER_SYSVIEW_RecordExitISR()
#else
    #define DEBUG_INIT()           do {} while(0)
    #define DEBUG_ISR_ENTER()      do {} while(0)
    #define DEBUG_ISR_EXIT()       do {} while(0)
#endif

// More detailed debugging only in higher levels
#if (DEBUG_LEVEL >= DEBUG_LEVEL_NORMAL)
    #define DEBUG_PRINT(msg)             SEGGER_SYSVIEW_PrintfTarget(msg)
    #define DEBUG_ERROR(msg)             SEGGER_SYSVIEW_Error(msg)
    #define DEBUG_WARN(msg, paramList)   SEGGER_SYSVIEW_WarnfTarget(msg, paramList)
#else
    #define DEBUG_PRINT(msg)       do {} while(0)
    #define DEBUG_ERROR(msg)       do {} while(0)
#endif

// Define indices for our data channels to make the code more maintainable
typedef enum {
    LOGGER_DATA_TIMESTAMP = 0,   // First data channel
    LOGGER_DATA_ANGLE,           // First float in struct
    LOGGER_DATA_ANGULAR_VEL,     // Second float in struct

    LOGGER_DATA_PID_OUTPUT,  // PID output 
    LOGGER_DATA_MOTORA_CURRENT,  // First motor current
    LOGGER_DATA_MOTORB_CURRENT,  // Second motor current

    LOGGER_DATA_MOTORA_PWM, // First motor PWM      
    LOGGER_DATA_MOTORB_PWM, // Second motor PWM
    LOGGER_DATA_BATTERY_VOLTAGE, // Battery voltage
    LOGGER_DATA_MOTOR_VOLTAGE,   // Motor voltage
    LOGGER_DATA_TOTAL_CURRENT,      // Total current draw
    LOGGER_DATA_POWER,       // Calculated power consumption
    LOGGER_DATA_COUNT      // Number of data channels
} LoggerDataIndex_t;


// Structure to hold all the data we want to log
typedef struct {
    // Timing information
    float timestamp;    // System time when data was captured
    
    // IMU and balance data
    float angle;          // Processed angle from MPU6050
    float angular_vel;    // Angular velocity
    
    float pid_output;    // Combined PID output
    
    // Motor data
    float motorA_current;  // Left motor current from VNH5019
    float motorB_current; // Right motor current from VNH5019
    float motorA_pwm;    // Left motor PWM value
    float motorB_pwm;    // Right motor PWM value
    
    // Power system data
    float battery_voltage;    // Main power supply voltage (up to 16.8V)
    float motor_voltage;      // Voltage at VNH5019 (might be different from battery)
    float total_current;      // Combined current draw (if measuring)
    float power_consumption;  // Calculated power in watts
} balance_log_data_t;

// Structure to hold all logging configuration and state
typedef struct {
    // Array of data channel configurations
    SEGGER_SYSVIEW_DATA_REGISTER dataConfigs[LOGGER_DATA_COUNT];
    
    // Array of data samples that we'll reuse for each logging cycle
    SEGGER_SYSVIEW_DATA_SAMPLE dataSamples[LOGGER_DATA_COUNT];
    
    // Flag to track if logger is initialized
    bool isInitialized;
    
    // Reference to the data structure we're logging
    balance_log_data_t* pLogData;
} BalanceLogger_t;


extern SEGGER_SYSVIEW_DATA_REGISTER PlotConfig;

// Function prototypes for our logger interface
void Logger_Init(BalanceLogger_t* logger, balance_log_data_t* pLogData);
bool process_balance_telemetry(balance_log_data_t* log_data, BalanceLogger_t* logger);

#endif /*LOGGER_H*/