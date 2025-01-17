#ifndef PID_H
#define PID_H
#include "stdint.h"


#define INTEGRAL_LIMIT (4199.0f * 0.3f)  // About 1260 units
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prevError;
    float outputMin;
    float outputMax;
    float deadband;
    float integralWindupLimit;
    float setpoint;
    float currentPidOutput;
    float lastPidOutput;
    float rateLimit;
    uint32_t lastTick;

} PidParameters_t;

extern PidParameters_t anglePID;

#define MOTOR_DEADBAND 20                // PWM deadband
// PWM configuration
#define PWM_MAX 4199                // Maximum PWM value
#define PWM_HALF (PWM_MAX / 2)      // Center point (2099)


float calculatePID(PidParameters_t *pid, float setpoint, float measurement, float dt);
float calculateDeltaTime(uint32_t currentTick, uint32_t *lastTick);
float applyRateLimit(float new_output, float last_output, float dt, float rate_limit);

#endif // PID_H