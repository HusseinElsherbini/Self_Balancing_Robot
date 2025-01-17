#include "task_macros.h"
#include "motor.h"
#include "led.h"
#include "imu.h"
#include "pid.h"
#include "i2c.h"
#include "balancing.h"
#include "logger.h"

BalancingTaskData_t balancingTaskData = {
    .mpu6050_data = &mpu6050_data,
    .BALANCING_TASK_STATE = BALANCING_TASK_READY,
    .xI2cTransferTime = 0,
    .xLastWakeTime = 0,
    .xTransferStart = 0,
    .xLastAngleCalc = 0,
    .xLastLedToggle = 0,
    .targetAngle = 0,
    .pidOutput = 0,
    .motorCommands = {0},
    .motorA = &xMotorAHandle,
    .motorB = &xMotorBHandle
};
BALANCING_TASK_STATE_t robotBalance(BalancingTaskData_t *balancingTaskData){
 
    balancingTaskData->BALANCING_TASK_STATE = BALANCING_TASK_RUNNING;
    balancingTaskData->xTransferStart = xTaskGetTickCount();
    get_raw_measurements(&i2c1, false);
    uint32_t notificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(IMU_I2C_TIMEOUT_TICKS));

    if(notificationValue == NOTIFY_IMU_DATA){
                
                
        balancingTaskData->xI2cTransferTime = xTaskGetTickCount() - balancingTaskData->xTransferStart;

        // If transfer took too long, log it
        if(balancingTaskData->xI2cTransferTime > IMU_I2C_TIMEOUT_TICKS) {  // More than 2ms
            //log_timing_error(xTimeElapsed); TODO: Implement this function
            asm("nop");
        }

        // process raw data
        process_raw_measurements(&mpu6050_data);

        // calculate time since last angle calculation
        float dt = (float)(xTaskGetTickCount() - balancingTaskData->xLastAngleCalc) / (float)configTICK_RATE_HZ;

        // convert raw data to angle
        convert_raw_data_to_angle(&mpu6050_data, dt);
        
        if(abs(mpu6050_data.processedData.processedData.angle) > 55.0f){
            // handle error
            actuateMotor(&xMotorAHandle, MOTOR_STOP, 0);
            actuateMotor(&xMotorBHandle, MOTOR_STOP, 0);
            balancingTaskData->BALANCING_TASK_STATE = BALANCING_TASK_STANDBY;
            return BALANCING_TASK_STANDBY;
        }
        if(xTaskGetTickCount() - balancingTaskData->xLastLedToggle > pdMS_TO_TICKS(500)){
            led_toggle(&red_led);
            balancingTaskData->xLastLedToggle = xTaskGetTickCount();
        }

        // set last angle calculation time
        balancingTaskData->xLastAngleCalc = xTaskGetTickCount();			

#if PRINT_IMU_DATA

        // print imu data
        print_imu_data(&mpu6050_data);

#endif

        // calculate PID output
        dt = calculateDeltaTime(xTaskGetTickCount(), &anglePID.lastTick);

        // Calculate PID output (-4199 to +4199 range)
        balancingTaskData->pidOutput = calculatePID(&anglePID, 
                            balancingTaskData->targetAngle, 
                            mpu6050_data.processedData.processedData.angle, 
                            dt);
        // Determine motor direction
        MotorDirection_t motorDirection;
        uint16_t pwmValue;

        if(abs(balancingTaskData->pidOutput) < MOTOR_DEADBAND) {
            // If PID output is very small, stop the motors
            motorDirection = MOTOR_STOP;
            pwmValue = 0;
        } else {
            // For larger corrections, determine direction and PWM
            if(balancingTaskData->pidOutput > 0) {
                motorDirection = MOTOR_REVERSE;
            } else {
                motorDirection = MOTOR_FORWARD;
            }
            
            // Convert PID output to PWM, accounting for deadband
            float adjustedOutput = abs(balancingTaskData->pidOutput) - MOTOR_DEADBAND;
            pwmValue = fabs(balancingTaskData->pidOutput);
        }
        // For debugging - direct view of values
        #if DEBUG_PRINT_PID
        SEGGER_RTT_printf("Angle: %.2f, PID: %.2f, PWM: %u\n", 
            imuData.pitch, pidOutput, pwmValue);
        #endif
        
        // Assign motor commands
        balancingTaskData->motorA.pwm_ch.dutyCycle = pwmValue;
        balancingTaskData->motorB.pwm_ch.dutyCycle = pwmValue;

        // Update motor PWM values
        //actuateMotor(&xMotorAHandle, motorDirection, balancingTaskData->motorA.pwm_ch.dutyCycle);
        //actuateMotor(&xMotorBHandle, motorDirection, balancingTaskData->motorB.pwm_ch.dutyCycle);
        
    }
    else{
        // handle timeout
        asm("nop");
        actuateMotor(&xMotorAHandle, MOTOR_STOP, 0);
        actuateMotor(&xMotorBHandle, MOTOR_STOP, 0);
        balancingTaskData->BALANCING_TASK_STATE = BALANCING_TASK_ERROR;
        return BALANCING_TASK_ERROR;
        //handle_i2c_timeout(); TODO: Implement this function
    }
    return balancingTaskData->BALANCING_TASK_STATE;
}


BALANCING_TASK_STATE_t robotStandby(BalancingTaskData_t *balancingTaskData){

        // handle standby
    actuateMotor(&xMotorAHandle, MOTOR_STOP, 0);
    actuateMotor(&xMotorBHandle, MOTOR_STOP, 0);
    asm("nop");
    balancingTaskData->xTransferStart = xTaskGetTickCount();
    get_raw_measurements(&i2c1, false);

    uint32_t notificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(IMU_I2C_TIMEOUT_TICKS));

    if(notificationValue == NOTIFY_IMU_DATA){
        
        balancingTaskData->xI2cTransferTime = xTaskGetTickCount() - balancingTaskData->xTransferStart;
        // process raw data
        process_raw_measurements(&mpu6050_data);

        // calculate time since last angle calculation 
        float dt = (float)(xTaskGetTickCount() - balancingTaskData->xLastAngleCalc) / (float)configTICK_RATE_HZ;

        // convert raw data to angle
        convert_raw_data_to_angle(&mpu6050_data, dt);
        
        balancingTaskData->xLastAngleCalc = xTaskGetTickCount();
        if(abs(mpu6050_data.processedData.processedData.angle) < 35.0f){
            // handle error
            balancingTaskData->BALANCING_TASK_STATE = BALANCING_TASK_RUNNING;
        }
    }
    else{
        // handle timeout
        asm("nop");
        balancingTaskData->BALANCING_TASK_STATE = BALANCING_TASK_ERROR;
        //handle_i2c_timeout(); TODO: Implement this function
    }
    return balancingTaskData->BALANCING_TASK_STATE;

}

void robotError(BalancingTaskData_t *balancingTaskData){

    // handle error
    actuateMotor(&xMotorAHandle, MOTOR_STOP, 0);
    actuateMotor(&xMotorBHandle, MOTOR_STOP, 0);
    asm("nop");
    if(xTaskGetTickCount() - balancingTaskData->xLastLedToggle > pdMS_TO_TICKS(250)){
        led_toggle(&red_led);
        DEBUG_WARN("Balancing task error", 0);
        balancingTaskData->xLastLedToggle = xTaskGetTickCount();
    }

    //BALANCING_TASK_STATE = BALANCING_TASK_READY;
}