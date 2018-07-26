/*
 * File      : board.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-09-22     Bernard      add board.h to this bsp
 */

// <<< Use Configuration Wizard in Context Menu >>>
#ifndef __BOARD_H__
#define __BOARD_H__

#include <stm32f4xx.h>

/* board configuration */
// <o> SDCard Driver <1=>SDIO sdcard <0=>SPI MMC card
// 	<i>Default: 1
#define STM32_USE_SDIO			0

/* whether use board external SRAM memory */
// <e>Use external SRAM memory on the board
// 	<i>Enable External SRAM memory
#define STM32_EXT_SRAM          0
//	<o>Begin Address of External SRAM
//		<i>Default: 0x68000000
#define STM32_EXT_SRAM_BEGIN    0x68000000 /* the begining address of external SRAM */
//	<o>End Address of External SRAM
//		<i>Default: 0x68080000
#define STM32_EXT_SRAM_END      0x68080000 /* the end address of external SRAM */
// </e>

// <o> Internal SRAM memory size[Kbytes] <8-64>
//	<i>Default: 64
#ifdef __ICCARM__
// Use *.icf ram symbal, to avoid hardcode.
extern char __ICFEDIT_region_RAM_end__;
#define STM32_SRAM_END          &__ICFEDIT_region_RAM_end__
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)&& (__ARMCC_VERSION >= 6010050)
//#define STM32_SRAM_SIZE         128
/* the size of heap is defined in startup.s, the address can be found in .map file */
extern int __heap_base;
extern int __heap_limit;
#define STM32_SRAM_BEGIN		(&__heap_base)	
#define STM32_SRAM_END          (&__heap_limit)
#else
extern int __bss_end;
#define STM32_SRAM_SIZE         (128 * 1024) /* 112KB(SRAM1) + 16KB(SRAM2) */
#define STM32_SRAM_BEGIN	(&__bss_end)	
#define STM32_SRAM_END          (0x20000000 + STM32_SRAM_SIZE) 

#endif

// <o> Console on USART: <0=> no console <1=>USART 1 <2=>USART 2 <3=> USART 3
// 	<i>Default: 1
#define STM32_CONSOLE_USART		3

void rt_hw_board_init(void);
void stm32_hw_spi_init(void);

//#if STM32_CONSOLE_USART == 0
//#define CONSOLE_DEVICE "no"
//#elif STM32_CONSOLE_USART == 1
//#define CONSOLE_DEVICE "uart1"
//#elif STM32_CONSOLE_USART == 2
//#define CONSOLE_DEVICE "uart2"
//#elif STM32_CONSOLE_USART == 3
//#define CONSOLE_DEVICE "uart3"
//#endif

//#define CONSOLE_DEVICE	"usb"
#define CONSOLE_DEVICE "uart3"

#define FINSH_DEVICE_NAME   CONSOLE_DEVICE

#endif

// <<< Use Configuration Wizard in Context Menu >>>
