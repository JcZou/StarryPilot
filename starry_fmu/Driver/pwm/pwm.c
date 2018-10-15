/*
 * File      : pwm.c
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-06-22     zoujiachi   	the first version
 */
 
#include "stm32f4xx.h"
#include "pwm.h"
#include "console.h"
#include "motor.h"

#define TIMER_FREQUENCY	3000000						// Timer frequency: 3M
#define PWM_DEFAULT_FREQUENCY	50					// pwm default frequqncy: 50Hz
#define PWM_ARR(freq) 	(TIMER_FREQUENCY/freq) 		// CCR reload value, Timer frequency = 3M/60K = 50 Hz

static int pwm_freq;
static float _pwm_fmu_duty_cyc[MAX_PWM_AUX_CHAN] = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
static Motor_Chan_Info _fmu_chan_info = {MOTOR_DEV_AUX, MAX_PWM_AUX_CHAN};

void pwm_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* TIM1 clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	/* GPIOE clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/* TIM1 CH1 (PE9,FMU_CH4), TIM1 CH2 (PE11,FMU_CH3), TIM1 CH3 (PE13,FMU_CH2) and TIM1 CH4 (PE14,FMU_CH1) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOE, &GPIO_InitStructure); 
	
	/* Connect TIM1 pins to AF */  
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1); 
	
	/* TIM4 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	/* GPIOE clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	/* TIM4 CH2 (PD13,FMU_CH5), TIM4 CH3 (PD14,FMU_CH6) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	/* Connect TIM4 pins to AF */  
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4); 
}

void pwm_timer_init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	uint16_t PrescalerValue;
	RCC_ClocksTypeDef  rcc_clocks;

	/* TIM1CLK = 2 * PCLK2 */ 
    /* PCLK2 = HCLK / 2 */
    RCC_GetClocksFreq(&rcc_clocks);
	
	/* Compute the prescaler value, TIM1 frequency = 3M Hz */
	PrescalerValue = (uint16_t) ((rcc_clocks.PCLK2_Frequency * 2 / TIMER_FREQUENCY) - 1);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = PWM_ARR(pwm_freq);	//PWM Frequency = 3M/60K = 50 Hz
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	//TIM_Cmd(TIM1, ENABLE);
	
	/* TIM4CLK = 2 * PCLK1 */ 
    /* PCLK1 = HCLK / 4 */
	PrescalerValue = (uint16_t) ((rcc_clocks.PCLK1_Frequency * 2 / TIMER_FREQUENCY) - 1);
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM4, ENABLE);
	//TIM_Cmd(TIM4, ENABLE);
}

void stm32_pwm_write(struct rt_device *device, uint8_t chan_id, float* duty_cyc)
{
	if(chan_id & MOTOR_CH1){
		TIM_SetCompare4(TIM1, PWM_ARR(pwm_freq)*duty_cyc[0]);
		_pwm_fmu_duty_cyc[0] = duty_cyc[0];
	}
	if(chan_id & MOTOR_CH2){
		TIM_SetCompare3(TIM1, PWM_ARR(pwm_freq)*duty_cyc[1]);
		_pwm_fmu_duty_cyc[1] = duty_cyc[1];
	}
	if(chan_id & MOTOR_CH3){
		TIM_SetCompare2(TIM1, PWM_ARR(pwm_freq)*duty_cyc[2]);
		_pwm_fmu_duty_cyc[2] = duty_cyc[2];
	}
	if(chan_id & MOTOR_CH4){
		TIM_SetCompare1(TIM1, PWM_ARR(pwm_freq)*duty_cyc[3]);
		_pwm_fmu_duty_cyc[3] = duty_cyc[3];
	}
	if(chan_id & MOTOR_CH5){
		TIM_SetCompare2(TIM4, PWM_ARR(pwm_freq)*duty_cyc[4]);
		_pwm_fmu_duty_cyc[4] = duty_cyc[4];
	}
	if(chan_id & MOTOR_CH6){
		TIM_SetCompare3(TIM4, PWM_ARR(pwm_freq)*duty_cyc[5]);
		_pwm_fmu_duty_cyc[5] = duty_cyc[5];
	}
}

int stm32_pwm_read(struct rt_device *device, uint8_t chan_id, float* buffer)
{
	for(uint8_t i = 0 ; i < MAX_PWM_AUX_CHAN ; i++){
		buffer[i] = _pwm_fmu_duty_cyc[i];
	}

	return 0;
}

void stm32_pwm_configure(rt_device_t dev, rt_uint8_t cmd, void *args)
{
	if(cmd == PWM_CMD_SET_FREQ){
		pwm_freq = *((int*)args);
		pwm_timer_init();
		// after changing frequency, the timer compare value should be re-configured
		stm32_pwm_write(dev, MOTOR_CH_ALL, _pwm_fmu_duty_cyc);
	}else if(cmd == PWM_CMD_ENABLE){
		int on_off = *((int*)args);
		if(on_off){
			TIM_Cmd(TIM1, ENABLE);
			TIM_CtrlPWMOutputs(TIM1, ENABLE);
			TIM_Cmd(TIM4, ENABLE);
		}
		else{
			TIM_Cmd(TIM1, DISABLE);
			TIM_CtrlPWMOutputs(TIM1, DISABLE);
			TIM_Cmd(TIM4, DISABLE);
		}
	}
}

const static struct rt_pwm_ops _stm32_pwm_ops =
{
    stm32_pwm_configure,
    stm32_pwm_write,
    stm32_pwm_read,
};

int stm32_pwm_init(void)
{
	pwm_freq = PWM_DEFAULT_FREQUENCY;
	
	pwm_gpio_init();
	pwm_timer_init();
	/* TIM1 Main Output Enable, Need Enable later */
	TIM_Cmd(TIM1, DISABLE);
	TIM_CtrlPWMOutputs(TIM1, DISABLE);
	TIM_Cmd(TIM4, DISABLE);
	
	rt_device_motor_register("motor_aux", &_stm32_pwm_ops, &_fmu_chan_info);
	
	return 0;
}
