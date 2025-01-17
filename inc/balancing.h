#ifndef BALANCING_H_
#define BALANCING_H_

#include "FreeRTOS.h"
#include "motor.h"
#include "imu.h"

typedef enum {
    BALANCING_TASK_READY = 0,
    BALANCING_TASK_RUNNING,
    BALANCING_TASK_ERROR,
    BALANCING_TASK_STANDBY

}BALANCING_TASK_STATE_t;

typedef struct {
    mpu6050_data_t *mpu6050_data;
    BALANCING_TASK_STATE_t BALANCING_TASK_STATE;
    TickType_t xLastWakeTime;
    TickType_t xTransferStart;
    TickType_t xLastAngleCalc;
    TickType_t xLastLedToggle;
    TickType_t xI2cTransferTime;
    float targetAngle;
    float pidOutput;
    MotorCommands_t motorCommands;
    MotorHandle_t motorA;
    MotorHandle_t motorB;

}BalancingTaskData_t;

extern BalancingTaskData_t balancingTaskData;

BALANCING_TASK_STATE_t robotBalance(BalancingTaskData_t *balancingTaskData);
BALANCING_TASK_STATE_t robotStandby(BalancingTaskData_t *balancingTaskData);
void robotError(BalancingTaskData_t *balancingTaskData);

#endif /* BALANCING_H_ */