#ifndef GPIO_H_
#define GPIO_H_

#include "platform.h"
#include "gpio.h"
#include "stdint.h"

#define GPIO_INPUT_MODE               (0)
#define GPIO_OUTPUT_MODE              (1)
#define GPIO_AF_MODE                  (2)
#define GPIO_ANALOG_MODE              (3)

#define GPIO_MODE_MSK(pin, mode)     (mode << 2*pin)

#define GPIO_OUTPUT_PUSH_PULL        (0)
#define GPIO_OUTPUT_OPEN_DRAIN       (1)

#define NOPULLUP_NOPULLDOWN          (0)
#define PULL_UP                      (1)
#define PULL_DOWN                    (2)

enum gpio_alternate_funcs {

    AF0,
    AF1,
    AF2,
    AF3,
    AF4,
    AF5,
    AF6,
    AF7,
    AF8,
    AF9,
    AF10,
    AF11,
    AF12,
    AF13,
    AF14,
    AF15
};

void setGpioMode(GPIO_CONFIG_t *gpio_base, uint8_t pin, uint8_t mode);
void setGpioOutput(GPIO_CONFIG_t *gpio_base, uint8_t pin, uint8_t outputType);
void setGpioAlternateFunction(GPIO_CONFIG_t *gpio_base, uint8_t pin, uint8_t alternateFunction);
void setGpioPupDR(GPIO_CONFIG_t *gpio_base, uint8_t pin, uint8_t pupMode);
void enableBusToGpioPort(uint8_t gpioPort);
uint8_t readGpioInput(GPIO_CONFIG_t *gpio_base);

#endif /*GPIO_H_*/