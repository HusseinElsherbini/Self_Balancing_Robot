#include "led.h"
#include "main.h"
#include "platform.h"
#include "task_scheduler.h"
#include "i2c.h"
#include "imu.h"
#include "rotary_encoder.h"
#include "pwm_common.h"

void sw_hw_init(void){

	sys_clock_init();
    init_systick_timer(TICK_HZ);
    enable_configurable_sys_faults();
    led_init(&yellow_led);
    led_init(&red_led);
    led_init(&blue_led);
    reEncoderInit(&rEncoder);
    pwmInit(&pwmA);
    pwmInit(&pwmB);
    i2cInit();
    imu_init(&i2c1);      
    
}
