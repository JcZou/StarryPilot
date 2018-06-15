/*
 * File      : led.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-6-6    		zoujiachi   	the first version
 */
 
#ifndef __LED_H__
#define __LED_H__

#include "stm32f4xx.h"

typedef enum
{
	LED_RED = 0,
	LED_GREEN,
	LED_BLUE,
	LED_YELLOW,
	LED_WHITE
}LED_COLOR;

void led_entry(void *parameter);
int device_led_init(void);
void led_on(void);
void led_off(void);
uint8_t TCA62724_set_color(LED_COLOR color);
uint8_t TCA62724_set_color_with_bright(LED_COLOR color, uint8_t bright);
uint8_t TCA62724_blink_control(uint8_t on_ff);

#endif
