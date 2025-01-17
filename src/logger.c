#include "logger.h"
#include "stdint.h"
#include "adc.h"
#include "dma.h"
#include "freeRtos.h"
#include "task_macros.h"
#include "SEGGER_SYSVIEW.h"
#include "motor.h"

void Logger_Init(BalanceLogger_t* logger, balance_log_data_t* pLogData) {
    // Prevent double initialization
    if (logger->isInitialized) {
        return;
    }
    
    // Store reference to the data structure we'll be logging
    logger->pLogData = pLogData;
    
    // Initialize all data channel configurations    
     

    // Timestamp configuration
    logger->dataConfigs[LOGGER_DATA_TIMESTAMP] = (SEGGER_SYSVIEW_DATA_REGISTER){
        .ID = LOGGER_DATA_TIMESTAMP,
        .sName = "Time Stamp",
        .DataType = SEGGER_SYSVIEW_TYPE_FLOAT,
        .ScalingFactor = 1.0f, // No scaling
        .RangeMin = 0,         // 0ms
        .RangeMax = INT32_MAX, // Max value 
        .sUnit = "ms"
    };
    // Angle configuration   
    logger->dataConfigs[LOGGER_DATA_ANGLE] = (SEGGER_SYSVIEW_DATA_REGISTER){
        .ID = LOGGER_DATA_ANGLE,
        .sName = "Balance Angle",
        .DataType = SEGGER_SYSVIEW_TYPE_FLOAT,
        .ScalingFactor = 1.0f,    // Multiply by 100 to preserve 2 decimal places
        .RangeMin = -90,       // -90 degrees
        .RangeMax = 90,        // +90 degrees
        .sUnit = "deg"
    };

    // Angular velocity configuration
    logger->dataConfigs[LOGGER_DATA_ANGULAR_VEL] = (SEGGER_SYSVIEW_DATA_REGISTER){
        .ID = LOGGER_DATA_ANGULAR_VEL,
        .sName = "Angular Velocity",
        .DataType = SEGGER_SYSVIEW_TYPE_FLOAT,
        .ScalingFactor = 1.0f,
        .RangeMin = -300,      // -300 deg/s
        .RangeMax = 300,       // +300 deg/s
        .sUnit = "deg/s"
    };
    
    // Motor voltage configuration
    logger->dataConfigs[LOGGER_DATA_MOTOR_VOLTAGE] = (SEGGER_SYSVIEW_DATA_REGISTER){
        .ID = LOGGER_DATA_MOTOR_VOLTAGE,
        .sName = "Motor Voltage",
        .DataType = SEGGER_SYSVIEW_TYPE_FLOAT,
        .ScalingFactor = 1.0f,
        .RangeMin = 0,
        .RangeMax = 30,        // 30V
        .sUnit = "V"
    };
    
    // Power supply voltage configuration
    logger->dataConfigs[LOGGER_DATA_BATTERY_VOLTAGE] = (SEGGER_SYSVIEW_DATA_REGISTER){
        .ID = LOGGER_DATA_BATTERY_VOLTAGE,
        .sName = "Battery Voltage",
        .DataType = SEGGER_SYSVIEW_TYPE_FLOAT,
        .ScalingFactor = 1.0f,
        .RangeMin = 0,
        .RangeMax = 30,        // 30V
        .sUnit = "V"
    };

    // Motor A current configuration
    logger->dataConfigs[LOGGER_DATA_MOTORA_CURRENT] = (SEGGER_SYSVIEW_DATA_REGISTER){
        .ID = LOGGER_DATA_MOTORA_CURRENT,
        .sName = "Motor A Current",
        .DataType = SEGGER_SYSVIEW_TYPE_FLOAT,
        .ScalingFactor = 1.0f,
        .RangeMin = 0,
        .RangeMax = 100,        // 100A
        .sUnit = "A"
    };

    // Motor B current configuration
    logger->dataConfigs[LOGGER_DATA_MOTORB_CURRENT] = (SEGGER_SYSVIEW_DATA_REGISTER){
        .ID = LOGGER_DATA_MOTORB_CURRENT,
        .sName = "Motor B Current",
        .DataType = SEGGER_SYSVIEW_TYPE_FLOAT,
        .ScalingFactor = 1.0f,
        .RangeMin = 0,
        .RangeMax = 100,        // 100A
        .sUnit = "A"
    };
    // Motor A current configuration
    logger->dataConfigs[LOGGER_DATA_MOTORA_PWM] = (SEGGER_SYSVIEW_DATA_REGISTER){
        .ID = LOGGER_DATA_MOTORA_PWM,
        .sName = "Motor A PWM",
        .DataType = SEGGER_SYSVIEW_TYPE_FLOAT,
        .ScalingFactor = 1.0f,
        .RangeMin = 0,
        .RangeMax = 4199,        // Max PWM value
    };

    // Motor B current configuration
    logger->dataConfigs[LOGGER_DATA_MOTORB_PWM] = (SEGGER_SYSVIEW_DATA_REGISTER){
        .ID = LOGGER_DATA_MOTORB_PWM,
        .sName = "Motor B PWM",
        .DataType = SEGGER_SYSVIEW_TYPE_FLOAT,
        .ScalingFactor = 1.0f,
        .RangeMin = 0,
        .RangeMax = 4199,        // Max PWM value
    };

    // PID output configuration
    logger->dataConfigs[LOGGER_DATA_PID_OUTPUT] = (SEGGER_SYSVIEW_DATA_REGISTER){
        .ID = LOGGER_DATA_PID_OUTPUT,
        .sName = "PID Output",
        .DataType = SEGGER_SYSVIEW_TYPE_FLOAT,
        .ScalingFactor = 1.0f,
        .RangeMin = -1000,
        .RangeMax = 1000,
    };

    logger->dataConfigs[LOGGER_DATA_POWER] = (SEGGER_SYSVIEW_DATA_REGISTER){
        .ID = LOGGER_DATA_POWER,
        .sName = "Power Consumption",
        .DataType = SEGGER_SYSVIEW_TYPE_FLOAT,
        .ScalingFactor = 1.0f,
        .RangeMin = 0,
        .RangeMax = 1000,
        .sUnit = "W"
    };

    // PID output configuration
    logger->dataConfigs[LOGGER_DATA_TOTAL_CURRENT] = (SEGGER_SYSVIEW_DATA_REGISTER){
        .ID = LOGGER_DATA_TOTAL_CURRENT,
        .sName = "Current Draw",
        .DataType = SEGGER_SYSVIEW_TYPE_FLOAT,
        .ScalingFactor = 1.0f,
        .RangeMin = 0,
        .RangeMax = 100,
        .sUnit = "A"
    };

    // Register all data channels with SystemView
    for (int i = 0; i < LOGGER_DATA_COUNT; i++) {

        SEGGER_SYSVIEW_RegisterData(&logger->dataConfigs[i]);
        
        // Pre-configure the data samples with their IDs
        logger->dataSamples[i].ID = i;
    }
    
    logger->isInitialized = true;
}

void Logger_Process(BalanceLogger_t* logger) {
    if (!logger->isInitialized || !logger->pLogData) {
        return;
    }
    
    // Create a float pointer that points to the start of our data
    float *data = (float*)&logger->pLogData->timestamp;
    
    // Now we can iterate safely through the struct's float members
    for (int i = 0; i < LOGGER_DATA_COUNT; i++) {
        logger->dataSamples[i].pValue.pFloat = &data[i];
        SEGGER_SYSVIEW_SampleData(&logger->dataSamples[i]);
    }
}

/**
 * Processes and logs balance robot telemetry data
 * 
 * This function:
 * 1. Processes ADC readings for motor voltage and current
 * 2. Updates power system measurements (battery, current, power)
 * 4. Logs all telemetry to SystemView for visualization
 * 
 * @param log_data Pointer to structure holding all robot telemetry
 * @param logger   Pointer to SystemView logger configuration
 * @return true if processing succeeded, false if any critical error occurred
 */
bool process_balance_telemetry(balance_log_data_t* log_data, 
                                    BalanceLogger_t* logger) {
    if (!log_data || !logger) {
        return false;
    }

    // Update timestamp for this data point
    log_data->timestamp = pdTICKS_TO_MS(xTaskGetTickCount());
    
    // Process motor measurements from ADC data
    calculate_motor_voltage(adc_dma_buffer[0], &xMotorAHandle);
    calculate_motor_current(adc_dma_buffer[1], &xMotorAHandle);
    calculate_motor_voltage(adc_dma_buffer[0], &xMotorBHandle);
    calculate_motor_current(adc_dma_buffer[2], &xMotorBHandle);
    
    // Update motor measurements
    log_data->motor_voltage = xMotorAHandle.voltage;
    log_data->motorA_current = xMotorAHandle.current;
    log_data->motorB_current = xMotorBHandle.current;
    log_data->total_current = log_data->motorA_current + log_data->motorB_current;
    log_data->battery_voltage = calculate_power_supply_voltage((uint16_t)adc_dma_buffer[0],log_data->total_current);
    
    // Update PWM values
    log_data->motorA_pwm = xMotorAHandle.pwm_ch.dutyCycle;
    log_data->motorB_pwm = xMotorBHandle.pwm_ch.dutyCycle;

    // Calculate power consumption
    log_data->power_consumption = log_data->battery_voltage * log_data->total_current;

    // Send all data points to SystemView
    Logger_Process(logger);
    
    return true;
}