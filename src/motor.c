#include "motor.h"
#include "pwm_common.h"
#include "gpio.h"
#include "stddef.h"

MotorHandle_t xMotorAHandle = {
    .in1.gpioPort = GPIOC,
    .in1.pin = 8,
    .in1.port_base_addr = (GPIO_REGS_t *)GPIOC_PERIPH_BASE,
    .in2.gpioPort = GPIOC,
    .in2.pin = 7,
    .in2.port_base_addr = (GPIO_REGS_t *)GPIOC_PERIPH_BASE,
    .hall_effect_sensor.channelA.gpioPort = GPIOA,
    .hall_effect_sensor.channelA.pin = 0,
    .hall_effect_sensor.channelB.gpioPort = GPIOA,
    .hall_effect_sensor.channelB.pin = 1,
    .hall_effect_sensor.channelA.port_base_addr = (GPIO_REGS_t *)GPIOA_PERIPH_BASE,
    .hall_effect_sensor.channelB.port_base_addr = (GPIO_REGS_t *)GPIOA_PERIPH_BASE,
    .hall_effect_sensor.channelA.alternateFunction = AF2,
    .hall_effect_sensor.channelA.pupDr = NOPULLUP_NOPULLDOWN,
    .hall_effect_sensor.channelA.speed = MAX_SPEED,
    .hall_effect_sensor.channelA.mode = GPIO_AF_MODE,
    .hall_effect_sensor.channelB.alternateFunction = AF2,
    .hall_effect_sensor.channelB.pupDr = NOPULLUP_NOPULLDOWN,
    .hall_effect_sensor.channelB.speed = MAX_SPEED,
    .hall_effect_sensor.channelB.mode = GPIO_AF_MODE,
    .current = 0.0f,
    .voltage = 0.0f,
    .peak_current = 0.0f,
    .min_current = 0.0f,
    .current_sense_res1 = MOTORA_CURRENT_SENSE_RES1,
    .current_sense_res2 = MOTORA_CURRENT_SENSE_RES2,
    .voltage_sense_res1 = VIN_VOLTAGE_DIVIDER_RES1,
    .voltage_sense_res2 = VIN_VOLTAGE_DIVIDER_RES2,

};

MotorHandle_t xMotorBHandle = {
    .in1.gpioPort = GPIOC,
    .in1.pin = 6,
    .in1.port_base_addr = (GPIO_REGS_t *)GPIOC_PERIPH_BASE,
    .in2.gpioPort = GPIOC,
    .in2.pin = 9,
    .in2.port_base_addr = (GPIO_REGS_t *)GPIOC_PERIPH_BASE,
    .hall_effect_sensor.channelA.gpioPort = GPIOA,
    .hall_effect_sensor.channelA.pin = 6,
    .hall_effect_sensor.channelA.gpioPort = GPIOA,
    .hall_effect_sensor.channelA.pin = 7,
    .hall_effect_sensor.channelA.port_base_addr = (GPIO_REGS_t *)GPIOA_PERIPH_BASE,
    .hall_effect_sensor.channelB.port_base_addr = (GPIO_REGS_t *)GPIOA_PERIPH_BASE,
    .hall_effect_sensor.channelA.alternateFunction = AF2,
    .hall_effect_sensor.channelA.pupDr = NOPULLUP_NOPULLDOWN,
    .hall_effect_sensor.channelA.speed = MAX_SPEED,
    .hall_effect_sensor.channelA.mode = GPIO_AF_MODE,
    .hall_effect_sensor.channelB.alternateFunction = AF2,
    .hall_effect_sensor.channelB.pupDr = NOPULLUP_NOPULLDOWN,
    .hall_effect_sensor.channelB.speed = MAX_SPEED,
    .hall_effect_sensor.channelB.mode = GPIO_AF_MODE,
    .current = 0.0f,
    .voltage = 0.0f,
    .peak_current = 0.0f,
    .min_current = 0.0f,
    .current_sense_res1 = MOTORB_CURRENT_SENSE_RES1,
    .current_sense_res2 = MOTORB_CURRENT_SENSE_RES2,
    .voltage_sense_res1 = VIN_VOLTAGE_DIVIDER_RES1,
    .voltage_sense_res2 = VIN_VOLTAGE_DIVIDER_RES2,
};



void motorInit(MotorHandle_t *motor, PWM_CH_T *pwm_ch)
{

    motor->pwm_ch = *pwm_ch;

    enableBusToGpioPort(motor->in1.gpioPort);
    enableBusToGpioPort(motor->in2.gpioPort);

    // Initialize IN1 and IN2 pins
    setGpioMode(&motor->in1, motor->in1.pin, GPIO_OUTPUT_MODE);
    setGpioOutput(&motor->in1, motor->in1.pin, GPIO_OUTPUT_PUSH_PULL);
    clrGpioOut(&motor->in1);

    setGpioMode(&motor->in2, motor->in2.pin, GPIO_OUTPUT_MODE);
    setGpioOutput(&motor->in2, motor->in2.pin, GPIO_OUTPUT_PUSH_PULL);
    clrGpioOut(&motor->in2);

    // Initialize PWM channel
    pwmInit(&motor->pwm_ch);
    
    // Initialize Hall Effect sensor
    hall_effect_sensor_init(&motor->hall_effect_sensor);
}

void motorSetDutyCycle(MotorHandle_t *motor, uint32_t duty_cycle)
{
    // Check for valid motor handle
    if (motor == NULL) {
        return;
    }
    
    // Update motor duty cycle
    motor->pwm_ch.dutyCycle = duty_cycle;
    
    // Set PWM value
    pwmSetDutyCycle(&motor->pwm_ch, motor->pwm_ch.dutyCycle);
}

void motorSetDirection(MotorHandle_t *motor, MotorDirection_t direction){
    switch(direction){
        case MOTOR_STOP:
            clrGpioOut(&motor->in1);
            clrGpioOut(&motor->in2);
            break;
        case MOTOR_FORWARD:
            setGpioOut(&motor->in1);
            clrGpioOut(&motor->in2);
            break;
        case MOTOR_REVERSE:
            clrGpioOut(&motor->in1);
            setGpioOut(&motor->in2);
            break;
        default:
            break;
    }
}

void actuateMotor(MotorHandle_t *motor, MotorDirection_t direction, uint32_t duty_cycle){
    motorSetDirection(motor, direction);
    motorSetDutyCycle(motor, duty_cycle);
}

void calculate_motor_current(float adc_value, MotorHandle_t *motor) {
    const float ADC_REFERENCE = 3.3f;
    const float ADC_MAX_VALUE = 4096.0f;
   
    // Convert ADC reading to voltage (0-3.3V)
    float adc_voltage = (adc_value * ADC_REFERENCE) / ADC_MAX_VALUE;
   
    // Since we have parallel resistors of equal value (1kΩ each)
    // The effective resistance is R1*R2/(R1+R2) = 500Ω
    float parallel_resistance = (motor->current_sense_res1 * motor->current_sense_res2) /
                              (motor->current_sense_res1 + motor->current_sense_res2);
   
    // Calculate sense current using the effective parallel resistance
    float sense_current = adc_voltage / parallel_resistance;
   
    // Select K ratio based on sense voltage
    float K;
    if (adc_voltage < 0.5f) {
        K = 7110.0f;  // For low current range
    } else if (adc_voltage < 1.3f) {
        K = 7030.0f;
    } else if (adc_voltage < 2.4f) {
        K = 6990.0f;
    } else {
        K = 6940.0f;
    }
   
    // Calculate final motor current
    motor->current = sense_current * K;
}

void calculate_motor_voltage(uint16_t adc_value, MotorHandle_t *motor){

    // First, convert ADC reading to voltage after the diode
    float adc_voltage = (adc_value * 3.3f) / 4096.0f;  // For 12-bit ADC
    float voltage_divider_ratio = (motor->voltage_sense_res1 + motor->voltage_sense_res2) / motor->voltage_sense_res2;  

    motor->voltage = (float)(adc_voltage*voltage_divider_ratio);
}

float calculate_power_supply_voltage(uint16_t adc_value, float total_current) {
    // First, convert ADC reading to voltage after the diode
    float adc_voltage = (adc_value * 3.3f) / 4096.0f;  // For 12-bit ADC
    float voltage_divider_ratio = (VIN_VOLTAGE_DIVIDER_RES1 + VIN_VOLTAGE_DIVIDER_RES2) / VIN_VOLTAGE_DIVIDER_RES2;

    float vin_after_diode = adc_voltage * voltage_divider_ratio;
    
    // Calculate diode forward voltage drop based on current
    // The voltage drop increases slightly with current
    // This is a simplified model of the diode's behavior
    float diode_drop = 0.75f;  // Base voltage drop
    if(total_current > 0.1f) {
        // Add about 0.1V per amp of current, up to a reasonable limit
        // This is an approximation - you might want to adjust these values
        float additional_drop = fminf(total_current * 0.1f, 0.3f);
        diode_drop += additional_drop;
    }
    
    // Calculate power supply voltage by adding the diode drop
    float power_supply_voltage = vin_after_diode + diode_drop;
    
    return power_supply_voltage;
}