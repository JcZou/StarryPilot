/*
 * File      : board.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      first implementation
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "stm32f4xx.h"
#include "board.h"
#include "usart.h"
#include "gpio.h"
#include "pwm.h"
#include "pwm_io.h"
#include "delay.h"
//#include <spi.h>
#include "stm32f4_spi.h"
#include "console.h"
#include "cdcacm.h"

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
	
	_delay_t.msPeriod += _delay_t.msPerPeriod;

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
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);

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
		
//		{
//			struct rt_spi_configuration cfg;
//			cfg.data_width = 8;
//			cfg.mode = RT_SPI_MODE_3 | RT_SPI_MSB; /* SPI Compatible Modes 3 */	
//			cfg.max_hz = 3000000;
//			
//			rt_spi_device_1.config.data_width = cfg.data_width;
//			rt_spi_device_1.config.mode       = cfg.mode & RT_SPI_MODE_MASK ;
//			rt_spi_device_1.config.max_hz     = cfg.max_hz;
//			rt_spi_configure(&rt_spi_device_1, &cfg);
//		}
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
		
//		/* config spi_device2 */
//		{
//			struct rt_spi_configuration cfg;
//			cfg.data_width = 8;
//			cfg.mode = RT_SPI_MODE_3 | RT_SPI_MSB; /* SPI Compatible Modes 3 */	
//			cfg.max_hz = 3000000;	/* max speed 8M Hz */
//			
//			rt_spi_device_2.config.data_width = cfg.data_width;
//			rt_spi_device_2.config.mode       = cfg.mode & RT_SPI_MODE_MASK ;
//			rt_spi_device_2.config.max_hz     = cfg.max_hz;
//			rt_spi_configure(&rt_spi_device_2, &cfg);
//		}
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
		
//		/* config spi_device2 */
//		{
//			struct rt_spi_configuration cfg;
//			cfg.data_width = 8;
//			cfg.mode = RT_SPI_MODE_3 | RT_SPI_MSB; /* SPI Compatible Modes 3 */	
//			cfg.max_hz = 3000000;	/* max speed 8M Hz */
//			
//			rt_spi_device_2.config.data_width = cfg.data_width;
//			rt_spi_device_2.config.mode       = cfg.mode & RT_SPI_MODE_MASK ;
//			rt_spi_device_2.config.max_hz     = cfg.max_hz;
//			rt_spi_configure(&rt_spi_device_2, &cfg);
//		}
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
	
	device_delay_init();

    /* Configure the SysTick */
    SysTick_Configuration();

	stm32_hw_usart_init();
	//usb_cdc_init();
//	/* do not use INT_TX or INT_RX flag here, because rt_mem is not init yet */
//	log_init(LOG_INTERFACE_SERIAL);
	
    stm32_hw_pin_init();
	stm32_hw_spi_init();
	stm32_pwm_init();
	pwm_io_init();
    
//#ifdef RT_USING_CONSOLE
//    rt_console_set_device(CONSOLE_DEVICE);
//#endif
}

/*@}*/
