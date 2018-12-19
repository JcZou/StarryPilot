/*****************************************************************************
Copyright (c) 2018, StarryPilot Development Team. All rights reserved.

Author: Jiachi Zou

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of StarryPilot nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "stm32f4xx.h"
#include "board.h"
#include "usart.h"
#include "gpio.h"
#include "pwm.h"
#include "pwm_io.h"
#include "systime.h"
//#include <spi.h>
#include "stm32f4_spi.h"
#include "console.h"
#include "cdcacm.h"
//#include "incapture.h"
//#include "balance_car_motor.h"

/**
 * @addtogroup STM32
 */

/*@{*/

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
#ifdef  VECT_TAB_RAM
	/* Set the Vector Table base location at 0x20000000 */
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
	/* Set the Vector Table base location at 0x08000000 */
	/* app start from 0x8004000,so need configure vectortab offset to 0x4000 */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x4000);
#endif

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}

/*******************************************************************************
 * Function Name  : SysTick_Configuration
 * Description    : Configures the SysTick for OS tick.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void  SysTick_Configuration(void)
{
	RCC_ClocksTypeDef  rcc_clocks;
	rt_uint32_t         cnts;

	RCC_GetClocksFreq(&rcc_clocks);

	cnts = (rt_uint32_t)rcc_clocks.HCLK_Frequency / RT_TICK_PER_SECOND;
	cnts = cnts / 8;

	SysTick_Config(cnts);
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
}

/**
 * This is the timer interrupt service routine.
 *
 */
void SysTick_Handler(void)
{
	/* enter interrupt */
	rt_interrupt_enter();

	rt_tick_increase();

	_systime_t.msPeriod += _systime_t.msPerPeriod;

	/* leave interrupt */
	rt_interrupt_leave();
}

/*** SPI1 BUS and device
SPI1_MOSI: PA7
SPI1_MISO: PA6
SPI1_SCK : PA5
*/
void stm32_hw_spi_init(void)
{
	/* register SPI bus */
	static struct stm32_spi_bus stm32_spi1;

	/* SPI1 configure */
	{
		GPIO_InitTypeDef GPIO_InitStructure;

		/* Enable SPI1 Periph clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

		GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //for emi
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL; //NO PULL, pull up will let sdcard unsteady

		/* Configure SPI1 pins */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	} /* SPI1 configuration */

	/* register SPI1 */
	stm32_spi_register(SPI1, &stm32_spi1, "spi1");

	/* attach spi_device_1 to spi1 */
	{
		static struct rt_spi_device rt_spi_device_1;
		static struct stm32_spi_cs  stm32_spi_cs_1;

		GPIO_InitTypeDef GPIO_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

		stm32_spi_cs_1.GPIOx = GPIOC;
		stm32_spi_cs_1.GPIO_Pin = GPIO_Pin_15;

		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		GPIO_SetBits(stm32_spi_cs_1.GPIOx, stm32_spi_cs_1.GPIO_Pin);

		/* rt name max is 8 (RT_NAME_MAX	   8) */
		rt_spi_bus_attach_device(&rt_spi_device_1, "spi_d1", "spi1", (void*)&stm32_spi_cs_1);
	}

	/* attach spi_device_2 to spi1 */
	{
		static struct rt_spi_device rt_spi_device_2;
		static struct stm32_spi_cs  stm32_spi_cs_2;

		GPIO_InitTypeDef GPIO_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

		stm32_spi_cs_2.GPIOx = GPIOC;
		stm32_spi_cs_2.GPIO_Pin = GPIO_Pin_13;

		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		GPIO_SetBits(stm32_spi_cs_2.GPIOx, stm32_spi_cs_2.GPIO_Pin);

		/* rt name max is 8 (RT_NAME_MAX	   8) */
		rt_spi_bus_attach_device(&rt_spi_device_2, "spi_d2", "spi1", (void*)&stm32_spi_cs_2);
	}

	/* attach spi_device_3 to spi1 */
	{
		static struct rt_spi_device rt_spi_device_3;
		static struct stm32_spi_cs  stm32_spi_cs_3;

		GPIO_InitTypeDef GPIO_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

		stm32_spi_cs_3.GPIOx = GPIOD;
		stm32_spi_cs_3.GPIO_Pin = GPIO_Pin_7;

		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
		GPIO_Init(GPIOD, &GPIO_InitStructure);

		GPIO_SetBits(stm32_spi_cs_3.GPIOx, stm32_spi_cs_3.GPIO_Pin);

		/* rt name max is 8 (RT_NAME_MAX	   8) */
		rt_spi_bus_attach_device(&rt_spi_device_3, "spi_d3", "spi1", (void*)&stm32_spi_cs_3);
	}

	/* attach spi_device_4 to spi1 */
	{
		static struct rt_spi_device rt_spi_device_4;
		static struct stm32_spi_cs  stm32_spi_cs_4;

		GPIO_InitTypeDef GPIO_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

		stm32_spi_cs_4.GPIOx = GPIOC;
		stm32_spi_cs_4.GPIO_Pin = GPIO_Pin_2;

		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		GPIO_SetBits(stm32_spi_cs_4.GPIOx, stm32_spi_cs_4.GPIO_Pin);

		/* rt name max is 8 (RT_NAME_MAX	   8) */
		rt_spi_bus_attach_device(&rt_spi_device_4, "spi_d4", "spi1", (void*)&stm32_spi_cs_4);
	}
}

/**
 * This function will initial STM32 board.
 */
void rt_hw_board_init()
{
	/* NVIC Configuration */
	NVIC_Configuration();

	/* Config system time module */
	device_systime_init();

	/* Configure the SysTick */
	SysTick_Configuration();

	stm32_hw_usart_init();
	stm32_hw_pin_init();
	stm32_hw_spi_init();
	stm32_hw_pwm_init();
}

/*@}*/
