/**
  ******************************************************************************
  * @file    ppm_capture.c 
  * @author  J Zou
  * @version V1.0
  * @date    16-Feb-2017
  * @brief   PPM Decoder
  ******************************************************************************
*/  

#include <stdio.h>
#include "ppm_capture.h"
#include "time.h"
#include "protocol.h"
#include "usart.h"

#define GET_GAP(x,y)	x > y ? (x-y) : (0xFFFF-y+x)

//we just need accuracy of 0.01ms
#define ENCODER_FREQ 100000
static uint32_t Scale_US = 1000000/ENCODER_FREQ;
static uint8_t PPM_Send_Freq = 0;
static uint8_t ppm_recv = 0;

ppm_status_machine_param machine_param;

void ppm_next_status(ppm_status_machine_param* param)
{
	static uint8_t max_ch = STA_EDGE_9+1;
	
	param->status = (param->status+1)%max_ch;
}

void reset_machine_state(ppm_status_machine_param* param)
{
	param->status = STA_EDGE_1;
	param->bad_frame = 0;
	param->last_ic = 0;
}	

void ppm_status_machine(uint16_t IC_Val)
{
	static uint16_t temp_val[8];
	uint16_t gap;
	
	gap = GET_GAP(IC_Val, machine_param.last_ic);

	/* each ppm frame has a 5.5ms start puls */
	if(gap > 500){	//each tick is 0.01ms
		reset_machine_state(&machine_param);
	}
	else if(gap < 80 || gap > 220){
		/* bad signal, drop frame */
		machine_param.bad_frame = 1;
	}
	
	if(machine_param.status == STA_EDGE_1){
		/* do nothing */
	}else if(machine_param.status == STA_EDGE_9){
		uint8_t i;
		
		temp_val[STA_EDGE_9-1] = gap;
		/* we know next we have a 5.5ms gap, so we can do some time-cost task here */
		/* If it's bad frame, we drop it */
		if( !machine_param.bad_frame ){
			for(i = 0 ; i<8 ; i++){
				machine_param.ppm_val[i] = Scale_US * temp_val[i];
			}
			ppm_recv = 1;
		}
		
	}else{
		temp_val[machine_param.status-1] = gap;
	}

	machine_param.last_ic = IC_Val;
	ppm_next_status(&machine_param);
}

void TIM1_irq_event_handler(void)
{
	volatile uint16_t IC1ReadValue = 0;
	
	if(TIM_GetITStatus(TIM1, TIM_IT_CC1) == SET) {
		/* Clear TIM1 Capture compare interrupt pending bit */
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
		
		IC1ReadValue = TIM_GetCapture1(TIM1);
		ppm_status_machine(IC1ReadValue);
	}
}

void get_ppm_value(uint32_t val[8])
{
	uint8_t i;
	
	for(i = 0 ; i<8 ; i++){
		val[i] = machine_param.ppm_val[i];
	}
}

uint8_t config_ppm_send_freq(uint8_t freq)
{
	if(freq >=0 && freq <= 50){
		PPM_Send_Freq = freq;
		return 1;
	}else
		return 0;
}

uint8_t ppm_ready(void)
{
	static uint32_t prev_time = 0;
	uint32_t time = time_nowMs();
	
	if(!PPM_Send_Freq)
		return 0;
	
	if( time - prev_time > 1000/PPM_Send_Freq && ppm_recv){
		prev_time = time;
		return 1;
	}
	
	return 0;
}

void ppm_clear_ready(void)
{
	ppm_recv = 0;
}

uint8_t send_ppm_value(void)
{
	uint32_t ppm_val[8];
	
	get_ppm_value(ppm_val);
	
	send_package(CMD_CHANNEL_VAL , (uint8_t*)ppm_val , 32);

	return 1;
}

uint8_t ppm_capture_init(void)
{
	TIM_ICInitTypeDef  TIM_ICInitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
	
	reset_machine_state(&machine_param);
	
	/* Clock Enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	/* Enable global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	/* lower priority to lidar */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	/* TIM1 channel 1 pin (PA.08) configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* Time Base configuration */
    TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock/ENCODER_FREQ;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
	
    TIM_ICInit(TIM1, &TIM_ICInitStructure);

	
	/* TIM enable counter */
    TIM_Cmd(TIM1, ENABLE);
	/* Enable the CC1 Interrupt Request */
    TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
	
	return 1;
}
