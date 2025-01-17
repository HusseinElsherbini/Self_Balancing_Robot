#include "led.h"
#include "main.h"
#include "platform.h"
#include "i2c.h"
#include "imu.h"
#include "rotary_encoder.h"
#include "pwm_common.h"
#include "spi.h"
#include "SEGGER_RTT.h"
#include "motor.h"
#include "adc.h"

void sw_hw_init(void){

	sys_clock_init();
    init_systick_timer(1000U);   // initialize systick timer for 1ms periods for initialization purposes, will move over to TIM5 as main timer
    
    enable_configurable_sys_faults();
    led_init(&yellow_led);
    led_init(&red_led);
    led_init(&blue_led);
    //reEncoderInit(&rEncoder);
    i2cInit(&i2c1);
    imu_init(&i2c1);     
    //spi_init(&spiHandle1);

    // Initialize RTT
    SEGGER_RTT_Init();
    //init_rtt_channels();

    // Initialize motor
    motorInit(&xMotorAHandle, &pwmA);
    motorInit(&xMotorBHandle, &pwmB);

    SBUS_init(&sbusHandle , &sbusUart);

    ADC_Init_With_Timer_DMA(system_sensors.port_config, TIM4);
}
