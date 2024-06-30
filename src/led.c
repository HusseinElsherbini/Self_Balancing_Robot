/*
 * led.c
 *
 *  Created on: Jan 22, 2024
 *      Author: Hae14
 */


#include<stdint.h>
#include "main.h"
#include "led.h"

// initialize led objects 
led yellow_led =  {

    .gpio_base_address = (uint32_t *)GPIOA_PERIPH_BASE,
    .rcc_ahb1en_reg =    (uint32_t *)RCC_AHB1EN,
    .mode_reg =          (uint32_t *)(GPIOA_PERIPH_BASE + GPIOx_MODER_OFFSET),
    .data_reg =          (uint32_t *)(GPIOA_PERIPH_BASE + GPIOx_ODR_OFFSET),
    .gpio_en =           (uint32_t)GPIOA_EN,
    .led_no  =           (uint8_t)LED_YELLOW
};

led blue_led =  {

    .gpio_base_address = (uint32_t *)GPIOA_PERIPH_BASE,
    .rcc_ahb1en_reg =    (uint32_t *)RCC_AHB1EN,
    .mode_reg =          (uint32_t *)(GPIOA_PERIPH_BASE + GPIOx_MODER_OFFSET),
    .data_reg =          (uint32_t *)(GPIOA_PERIPH_BASE + GPIOx_ODR_OFFSET),
    .gpio_en =           (uint32_t)GPIOA_EN,
    .led_no  =           (uint8_t)LED_BLUE
};

led red_led =  {

    .gpio_base_address = (uint32_t *)GPIOB_PERIPH_BASE,
    .rcc_ahb1en_reg =    (uint32_t *)RCC_AHB1EN,
    .mode_reg =          (uint32_t *)(GPIOB_PERIPH_BASE + GPIOx_MODER_OFFSET),
    .data_reg =          (uint32_t *)(GPIOB_PERIPH_BASE + GPIOx_ODR_OFFSET),
    .gpio_en =           (uint32_t)GPIOB_EN,
    .led_no  =           (uint8_t)LED_RED
};
   
void led_init(led *ledX)
{

	// enable AHB bus to GPIO A and B
	*(ledX->rcc_ahb1en_reg) |= (ledX->gpio_en);

	//configure LED_GREEN
	*(ledX->mode_reg) |= ( 1 << (2 * ledX->led_no));

    led_off(ledX);

}

void led_on(led *ledX){

  *(ledX->data_reg) |= ( 1 << ledX->led_no );

}

void led_off(led *ledX){

  *(ledX->data_reg) &=  ~( 1 << ledX->led_no );

}
