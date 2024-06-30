/*
 * led.h
 *
 *  Created on: Jan 22, 2024
 *      Author: Hae14
 */

#ifndef LED_H_
#define LED_H_

#include <stdint.h>

#define LED_YELLOW  2
#define LED_RED    13
#define LED_BLUE    1


typedef struct {

    uint32_t *gpio_base_address;
    uint32_t *rcc_ahb1en_reg;
    uint32_t gpio_en;
    uint8_t  led_no;
    uint32_t *mode_reg;
    uint32_t *data_reg;
   
}led;

extern led yellow_led;
extern led blue_led;
extern led red_led;

void led_init(led *ledX);
void led_on(led *ledX);
void led_off(led *ledX);

#endif /* LED_H_ */
