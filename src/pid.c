#include "pid.h"
#include "stdint.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"

// Initialize angle PID parameters - start with just P control
PidParameters_t anglePID = {
    .Kp = 15.0f,      // Start with a conservative value
    .Ki = 50.0f,       // Start with no integral term
    .Kd = 0.1f,       // Start with no derivative term
    .integral = 0.0f,
    .prevError = 0.0f,
    .outputMin = -4199.0f,
    .outputMax = 4199.0f,
    .currentPidOutput = 0.0f,
    .lastPidOutput = 0.0f,
    .rateLimit = 10.0f,
    .integralWindupLimit = INTEGRAL_LIMIT,
};


float applyRateLimit(float new_output, float last_output, float dt, float rate_limit) {
    float max_change = rate_limit * dt;
    float output_change = new_output - last_output;
    
    if(output_change > max_change) {
        return last_output + max_change;
    } else if(output_change < -max_change) {
        return last_output - max_change;
    }
    return new_output;
}

// PID calculation function
float calculatePID(PidParameters_t *pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;
   
    // Calculate integral
    pid->integral += error * dt;
    pid->integral = fminf(fmaxf(pid->integral, pid->outputMin), pid->outputMax);

    // Apply integral limit
    if (pid->integral > INTEGRAL_LIMIT) {
        pid->integral = INTEGRAL_LIMIT;
    } else if (pid->integral < -INTEGRAL_LIMIT) {
        pid->integral = -INTEGRAL_LIMIT;
    }
    // Calculate derivative
    float derivative = (error - pid->prevError) / dt;
    pid->prevError = error;
   
    // Calculate output
    float output = (pid->Kp * error) +
                  (pid->Ki * pid->integral) +
                  (pid->Kd * derivative);
   
    // Apply output limits
    output = fminf(fmaxf(output, pid->outputMin), pid->outputMax);
    
    return output;
}

float calculateDeltaTime(uint32_t currentTick, uint32_t *lastTick) {
    uint32_t tickDiff;
    
    // Handle tick counter overflow
    if (currentTick < *lastTick) {
        tickDiff = (portMAX_DELAY - *lastTick) + currentTick;
    } else {
        tickDiff = currentTick - *lastTick;
    }
    
    // Update last tick
    *lastTick = currentTick;
    
    // Convert ticks to seconds
    return (float)tickDiff / configTICK_RATE_HZ;
}