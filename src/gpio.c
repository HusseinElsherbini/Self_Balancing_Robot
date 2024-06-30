#include "platform.h"
#include "gpio.h"
#include "main.h"


void setGpioMode(GPIO_CONFIG_t *gpio_base, uint8_t pin, uint8_t mode){

    gpio_base->port_base_addr->MODER |= GPIO_MODE_MSK(pin, mode);
}

void setGpioOutput(GPIO_CONFIG_t *gpio_base, uint8_t pin, uint8_t outputType){

    gpio_base->port_base_addr->OTYPER |= GENERIC_SET_MSK(outputType, pin);
}

void setGpioAlternateFunction(GPIO_CONFIG_t *gpio_base, uint8_t pin, uint8_t alternateFunction){

    uint16_t shiftAmt = (4U * (pin % 8U));
    if(pin < 7){
        MODIFY_REG( gpio_base->port_base_addr->AFRL, (15U << shiftAmt), (alternateFunction << shiftAmt));
    }
    else{
        MODIFY_REG( gpio_base->port_base_addr->AFRH, (15U << shiftAmt), (alternateFunction << shiftAmt));
    }
    
}

void setGpioPupDR(GPIO_CONFIG_t *gpio_base, uint8_t pin, uint8_t pupMode){

    MODIFY_REG(gpio_base->port_base_addr->PUPDR, GENERIC_SET_MSK(3U, (2U)*pin), GENERIC_SET_MSK(pupMode, (2U)*pin));
}

void enableBusToGpioPort(uint8_t gpioPort){

    SET_BIT(RCC_REGS->RCC_AHB1ENR, GPIOx_ENABLE(gpioPort));
}

uint8_t readGpioInput(GPIO_CONFIG_t *gpio_base){

    return (uint8_t)(READ_BIT(gpio_base->port_base_addr->IDR, (1U << gpio_base->pin)) >> gpio_base->pin);
}