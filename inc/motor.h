#ifndef MOTOR_H
#define MOTOR_H

#include "pwm_common.h"
#include "hall_effect.h"
/*

Frequency = 20kHz
Period = 1/Frequency = 1/20,000 = 50 microseconds (μs)

Total steps = 2^16 = 65,536 steps
Time per step = 50μs ÷ 65,536 = 0.76 nanoseconds (ns)

PWM Period (50μs)
|<-------------- 50 microseconds ------------->|
|                                             |
High _________                                |
           |                                  |
Low        |__________________________________|
          
With 16-bit:
Each step = 0.76ns
                                             
Different bit depths:
8-bit  (256 steps)    = 195ns per step
10-bit (1024 steps)   = 49ns per step
12-bit (4096 steps)   = 12ns per step
16-bit (65536 steps)  = 0.76ns per step

TB6612FNG MOSFETs:

Turn-on time: ~100ns
Turn-off time: ~100ns
Total switching time: ~200ns

can't create any pulse shorter than 200ns

minimum pulse width = 212ns if 12 bit PWM resolution is used

PWM      ____      ____      ____
    ____|    |____|    |____|    |____
    
IN1   ________
    __|        |___________________
    
IN2   ________________
    __|                |___________

Motor
Current  →→→   ←←←     →→→    ←←←

*/

typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_REVERSE

} MotorDirection_t;

typedef struct {
    float left_motor;  // -100 to +100%
    float right_motor; // -100 to +100%
} MotorCommands_t;

#define MOTORA_CURRENT_SENSE_RES1   9950.0f  // 9.95k 
#define MOTORA_CURRENT_SENSE_RES2   1010.0f   // 1.01k

#define MOTORB_CURRENT_SENSE_RES1   9930.0f  // 9.93k 
#define MOTORB_CURRENT_SENSE_RES2   994.0f   // .994k

/*
MOTOR A CURRENT SENSE RESISTOR VALUES 
R1 = 9.95k
R2 = 1.01k

MOTOR B CURRENT SENSE RESISTOR VALUES 
R1 = 9.93k
R2 = .994

*/

#define VIN_VOLTAGE_DIVIDER_RES1   9720.0f  // 9.72k 
#define VIN_VOLTAGE_DIVIDER_RES2   982.0f // 0.982

typedef struct {
    PWM_CH_T pwm_ch;
    GPIO_CONFIG_t in1;
    GPIO_CONFIG_t in2;
    HallSensor_t hall_effect_sensor;
    float voltage;
    float current;
    float peak_current;
    float min_current;
    float current_sense_res1;
    float current_sense_res2;
    float voltage_sense_res1;
    float voltage_sense_res2;

} MotorHandle_t;

extern MotorHandle_t xMotorAHandle;
extern MotorHandle_t xMotorBHandle;


void motorSetDutyCycle(MotorHandle_t *motor, uint32_t duty_cycle);
void motorInit(MotorHandle_t *motor, PWM_CH_T *pwm_ch);
void motorSetDirection(MotorHandle_t *motor, MotorDirection_t direction);
void actuateMotor(MotorHandle_t *motor, MotorDirection_t direction, uint32_t duty_cycle);
void calculate_motor_current(float adc_value, MotorHandle_t *motor);
void calculate_motor_voltage(uint16_t adc_value, MotorHandle_t *motor);
float calculate_power_supply_voltage(uint16_t adc_value, float total_current);

#endif // MOTOR_H