/**
  ******************************************************************************
  * @file    main.c 
  * @author  J Zou
  * @version V1.0
  * @date    13-Feb-2017
  * @brief   Main program body
  ******************************************************************************
*/  

#include <stdio.h>
#include "usart.h"
#include "ppm_capture.h"
#include "time.h"
#include "protocol.h"
#include "led.h"
#include "lidar_lite.h"

int main(void)
{
	uint8_t ch;
	uint32_t time_led, time_sync;
	uint32_t now;
	
	usart_init();
	time_init();
	ppm_capture_init();
	led_init();
	lidar_lite_init();
	
	led_on(LED_BLUE);
	led_on(LED_RED);
	
	time_led = time_nowMs();
	time_sync = time_led;

	while (1)
	{
		if(read_ch(&ch)){
			protocol_input_byte(ch);
		}
		
		if(sync_finish()){
			if(ppm_ready()){
				ppm_clear_ready();
				send_ppm_value();
			}
			
			if(lidar_lite_ready()){
				lidar_lite_clear_ready();
				send_lidar_distance();
			}

			now = time_nowMs();
			if( now - time_led > 1000 ){
				led_toggle(LED_BLUE);
				time_led  = now;
			}
		}else{
			now = time_nowMs();
			if( now - time_sync > 100 ){
				send_package(CMD_SYNC , NULL , 0);
				time_sync  = now;
			}
			
			now = time_nowMs();
			if( now - time_led > 1000 ){
				led_toggle(LED_RED);
				time_led  = now;
			}
		}
	}
}
