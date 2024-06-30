#include "rotary_encoder.h"
#include "gpio.h"

RE_T rEncoder = {
    .clk_pin            = (GPIO_CONFIG_t *)GPIOC_PERIPH_BASE,
    .dt_pin             = (GPIO_CONFIG_t *)GPIOC_PERIPH_BASE,
    .clk_pin.gpioPort   = GPIOC,
    .clk_pin.pin        = 1U,
    .dt_pin.pin         = 0U,
    .counter            = 0U,
    .dt_pin.gpioPort    = GPIOC,
    .stateChanged       = false
}; 


void reEncoderInit(RE_T *re){

    // enable AHB1 for pins 
    enableBusToGpioPort(re->clk_pin.gpioPort);
    enableBusToGpioPort(re->dt_pin.gpioPort);

    // set GPIO mode to input 
    setGpioMode(&re->clk_pin, re->clk_pin.pin, GPIO_INPUT_MODE);
    setGpioMode(&re->dt_pin, re->dt_pin.pin, GPIO_INPUT_MODE);

    // set PINS to no pull up or pull down
    setGpioPupDR(re->clk_pin.port_base_addr, re->clk_pin.pin, NOPULLUP_NOPULLDOWN);
    setGpioPupDR(re->dt_pin.port_base_addr, re->dt_pin.pin, NOPULLUP_NOPULLDOWN);

    re->previous_state = readGpioInput(&re->clk_pin);

}

void checkEncoderStateChange(RE_T *re){

    // read CLK pin to check it's state
    re->current_state = readGpioInput(&re->clk_pin);

    if(re->current_state != re->previous_state & re->current_state == 1U){
        re->stateChanged = true;
        // check direction 
        if(readGpioInput(&re->dt_pin) == re->current_state){
            re->direction = CW;
            re->counter++;
        }
        else{
            re->direction = CCW;
            re->counter--;
        }
    }   
    re->previous_state = re->current_state;
}