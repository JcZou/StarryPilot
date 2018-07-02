/**
  ******************************************************************************
  * @file    led.h 
  * @author  J Zou
  * @version V1.0
  * @date    21-Feb-2017
  * @brief   LED Function
  ******************************************************************************
*/  

#ifndef  _LED_H_
#define  _LED_H_

#include "stm32f10x.h"

#define LED_PORT		GPIOB
#define LED_BLUE_PIN	GPIO_Pin_14
#define LED_RED_PIN		GPIO_Pin_15
#define LED_RCC			RCC_APB2Periph_GPIOB

typedef enum
{
	LED_RED = 0,
	LED_BLUE
}LED_Type;

void led_on(LED_Type led);
void led_off(LED_Type led);
void led_toggle(LED_Type led);
void led_init(void);

#endif
