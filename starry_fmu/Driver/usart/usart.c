/*
 * File      : usart.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2010-03-29     Bernard      remove interrupt Tx and DMA Rx mode
 * 2012-02-08     aozima       update for F4.
 * 2012-07-28     aozima       update for ART board.
 */

#include "stm32f4xx.h"
#include "usart.h"
#include "board.h"

#include <rtdevice.h>

/* UART GPIO define. */
#define UART1_GPIO_TX       GPIO_Pin_9
#define UART1_TX_PIN_SOURCE GPIO_PinSource9
#define UART1_GPIO_RX       GPIO_Pin_10
#define UART1_RX_PIN_SOURCE GPIO_PinSource10
#define UART1_GPIO          GPIOA
#define UART1_GPIO_RCC      RCC_AHB1Periph_GPIOA
#define RCC_APBPeriph_UART1 RCC_APB2Periph_USART1
#define UART1_TX_DMA        DMA1_Channel4
#define UART1_RX_DMA        DMA1_Channel5

#define UART2_GPIO_TX       GPIO_Pin_2
#define UART2_TX_PIN_SOURCE GPIO_PinSource2
#define UART2_GPIO_RX       GPIO_Pin_3
#define UART2_RX_PIN_SOURCE GPIO_PinSource3
#define UART2_GPIO          GPIOA
#define UART2_GPIO_RCC      RCC_AHB1Periph_GPIOA
#define RCC_APBPeriph_UART2 RCC_APB1Periph_USART2
#define UART2_TX_DMA        DMA1_Channel4
#define UART2_RX_DMA        DMA1_Channel5

#define UART3_GPIO_TX       GPIO_Pin_8
#define UART3_TX_PIN_SOURCE GPIO_PinSource8
#define UART3_GPIO_RX       GPIO_Pin_9
#define UART3_RX_PIN_SOURCE GPIO_PinSource9
#define UART3_GPIO          GPIOD
#define UART3_GPIO_RCC      RCC_AHB1Periph_GPIOD
#define RCC_APBPeriph_UART3 RCC_APB1Periph_USART3
#define UART3_TX_DMA        DMA1_Stream1
#define UART3_RX_DMA        DMA1_Stream3

#define UART4_GPIO_TX       GPIO_Pin_0
#define UART4_TX_PIN_SOURCE GPIO_PinSource0
#define UART4_GPIO_RX       GPIO_Pin_1
#define UART4_RX_PIN_SOURCE GPIO_PinSource1
#define UART4_GPIO          GPIOA
#define UART4_GPIO_RCC      RCC_AHB1Periph_GPIOA
#define RCC_APBPeriph_UART4 RCC_APB1Periph_UART4

#define UART6_GPIO_TX       GPIO_Pin_6
#define UART6_TX_PIN_SOURCE GPIO_PinSource6
#define UART6_GPIO_RX       GPIO_Pin_7
#define UART6_RX_PIN_SOURCE GPIO_PinSource7
#define UART6_GPIO          GPIOC
#define UART6_GPIO_RCC      RCC_AHB1Periph_GPIOC
#define RCC_APBPeriph_UART6 RCC_APB2Periph_USART6

/* STM32 uart driver */
struct stm32_uart
{
    USART_TypeDef *uart_device;
    IRQn_Type irq;
};

static rt_err_t stm32_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct stm32_uart *uart;
    USART_InitTypeDef USART_InitStructure;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    uart = (struct stm32_uart *)serial->parent.user_data;
	
	if(cfg->baud_rate!=BAUD_RATE_2400 && cfg->baud_rate!=BAUD_RATE_4800 && cfg->baud_rate!=BAUD_RATE_9600 && cfg->baud_rate!=BAUD_RATE_19200
		 && cfg->baud_rate!=BAUD_RATE_38400 && cfg->baud_rate!=BAUD_RATE_57600 && cfg->baud_rate!=BAUD_RATE_115200
		&& cfg->baud_rate!=BAUD_RATE_230400 && cfg->baud_rate!=BAUD_RATE_460800 && cfg->baud_rate!=BAUD_RATE_921600)
	{
		USART_InitStructure.USART_BaudRate = serial->config.baud_rate;	//dont change baudrate
	}else{
		USART_InitStructure.USART_BaudRate = cfg->baud_rate;
	}

    if (cfg->data_bits == DATA_BITS_8)
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	else if(cfg->data_bits == DATA_BITS_9)
		USART_InitStructure.USART_WordLength = USART_WordLength_9b;

    if (cfg->stop_bits == STOP_BITS_1)
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
    else if (cfg->stop_bits == STOP_BITS_2)
        USART_InitStructure.USART_StopBits = USART_StopBits_2;

    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(uart->uart_device, &USART_InitStructure);

    /* Enable USART */
    USART_Cmd(uart->uart_device, ENABLE);

    return RT_EOK;
}

static rt_err_t stm32_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct stm32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        UART_DISABLE_IRQ(uart->irq);
        /* disable interrupt */
        USART_ITConfig(uart->uart_device, USART_IT_RXNE, DISABLE);
        break;
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        UART_ENABLE_IRQ(uart->irq);
        /* enable interrupt */
        USART_ITConfig(uart->uart_device, USART_IT_RXNE, ENABLE);
        break;
    }

    return RT_EOK;
}

static int stm32_putc(struct rt_serial_device *serial, char c)
{
    struct stm32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    //while (!(uart->uart_device->SR & USART_FLAG_TXE));
	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(uart->uart_device, USART_FLAG_TC) == RESET)
	{}

    uart->uart_device->DR = c;

    return 1;
}

static int stm32_getc(struct rt_serial_device *serial)
{
    int ch;
    struct stm32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    ch = -1;
    if (uart->uart_device->SR & USART_FLAG_RXNE)
    {
        ch = uart->uart_device->DR & 0xff;
    }

    return ch;
}

static const struct rt_uart_ops stm32_uart_ops =
{
    stm32_configure,
    stm32_control,
    stm32_putc,
    stm32_getc,
};

#if defined(RT_USING_UART1)
/* UART1 device driver structure */
struct stm32_uart uart1 =
{
    USART1,
    USART1_IRQn,
};
struct rt_serial_device serial1;

void USART1_IRQHandler(void)
{
    struct stm32_uart *uart;

    uart = &uart1;

    /* enter interrupt */
    rt_interrupt_enter();
    if (USART_GetITStatus(uart->uart_device, USART_IT_RXNE) != RESET)
    {
        rt_hw_serial_isr(&serial1, RT_SERIAL_EVENT_RX_IND);
    }
    if (USART_GetITStatus(uart->uart_device, USART_IT_TC) != RESET)
    {
        /* clear interrupt */
        USART_ClearITPendingBit(uart->uart_device, USART_IT_TC);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_UART1 */

#if defined(RT_USING_UART2)
/* UART2 device driver structure */
struct stm32_uart uart2 =
{
    USART2,
    USART2_IRQn,
};
struct rt_serial_device serial2;

void USART2_IRQHandler(void)
{
    struct stm32_uart *uart;

    uart = &uart2;

    /* enter interrupt */
    rt_interrupt_enter();
    if (USART_GetITStatus(uart->uart_device, USART_IT_RXNE) != RESET)
    {
        rt_hw_serial_isr(&serial2, RT_SERIAL_EVENT_RX_IND);
    }
    if (USART_GetITStatus(uart->uart_device, USART_IT_TC) != RESET)
    {
        /* clear interrupt */
        USART_ClearITPendingBit(uart->uart_device, USART_IT_TC);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_UART2 */

#if defined(RT_USING_UART3)
/* UART3 device driver structure */
struct stm32_uart uart3 =
{
    USART3,
    USART3_IRQn,
};
struct rt_serial_device serial3;

void USART3_IRQHandler(void)
{
    struct stm32_uart *uart;

    uart = &uart3;

    /* enter interrupt */
    rt_interrupt_enter();
    if (USART_GetITStatus(uart->uart_device, USART_IT_RXNE) != RESET)
    {
        rt_hw_serial_isr(&serial3, RT_SERIAL_EVENT_RX_IND);
    }
    if (USART_GetITStatus(uart->uart_device, USART_IT_TC) != RESET)
    {
        /* clear interrupt */
        USART_ClearITPendingBit(uart->uart_device, USART_IT_TC);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_UART3 */

#if defined(RT_USING_UART4)
/* UART4 device driver structure */
struct stm32_uart uart4 =
{
    UART4,
    UART4_IRQn,
};
struct rt_serial_device serial4;

void UART4_IRQHandler(void)
{
    struct stm32_uart *uart;

    uart = &uart4;

    /* enter interrupt */
    rt_interrupt_enter();
    if (USART_GetITStatus(uart->uart_device, USART_IT_RXNE) != RESET)
    {
        rt_hw_serial_isr(&serial4, RT_SERIAL_EVENT_RX_IND);
    }
    if (USART_GetITStatus(uart->uart_device, USART_IT_TC) != RESET)
    {
        /* clear interrupt */
        USART_ClearITPendingBit(uart->uart_device, USART_IT_TC);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_UART4 */

#if defined(RT_USING_UART6)
/* UART6 device driver structure */
struct stm32_uart uart6 =
{
    USART6,
    USART6_IRQn,
};
struct rt_serial_device serial6;

void USART6_IRQHandler(void)
{
    struct stm32_uart *uart;

    uart = &uart6;
	
	//printf("usart6\r\n");

    /* enter interrupt */
    rt_interrupt_enter();
    if (USART_GetITStatus(uart->uart_device, USART_IT_RXNE) != RESET)
    {
        rt_hw_serial_isr(&serial6, RT_SERIAL_EVENT_RX_IND);
    }
    if (USART_GetITStatus(uart->uart_device, USART_IT_TC) != RESET)
    {
        /* clear interrupt */
        USART_ClearITPendingBit(uart->uart_device, USART_IT_TC);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_UART6 */

static void RCC_Configuration(void)
{
#ifdef RT_USING_UART1
    /* Enable UART1 GPIO clocks */
    RCC_AHB1PeriphClockCmd(UART1_GPIO_RCC, ENABLE);
    /* Enable UART1 clock */
    RCC_APB2PeriphClockCmd(RCC_APBPeriph_UART1, ENABLE);
#endif /* RT_USING_UART1 */

#ifdef RT_USING_UART2
    /* Enable UART2 GPIO clocks */
    RCC_AHB1PeriphClockCmd(UART2_GPIO_RCC, ENABLE);
    /* Enable UART2 clock */
    RCC_APB1PeriphClockCmd(RCC_APBPeriph_UART2, ENABLE);
#endif /* RT_USING_UART1 */

#ifdef RT_USING_UART3
    /* Enable UART3 GPIO clocks */
    RCC_AHB1PeriphClockCmd(UART3_GPIO_RCC, ENABLE);
    /* Enable UART3 clock */
    RCC_APB1PeriphClockCmd(RCC_APBPeriph_UART3, ENABLE);
#endif /* RT_USING_UART3 */
	
#ifdef RT_USING_UART4
    /* Enable UART4 GPIO clocks */
    RCC_AHB1PeriphClockCmd(UART4_GPIO_RCC, ENABLE);
    /* Enable UART4 clock */
    RCC_APB1PeriphClockCmd(RCC_APBPeriph_UART4, ENABLE);
#endif /* RT_USING_UART4 */

#ifdef RT_USING_UART6
    /* Enable UART6 GPIO clocks */
    RCC_AHB1PeriphClockCmd(UART6_GPIO_RCC, ENABLE);
    /* Enable UART6 clock */
    RCC_APB2PeriphClockCmd(RCC_APBPeriph_UART6, ENABLE);
#endif /* RT_USING_UART6 */
}

static void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

#ifdef RT_USING_UART1
    /* Configure USART1 Rx/tx PIN */
    GPIO_InitStructure.GPIO_Pin = UART1_GPIO_RX | UART1_GPIO_TX;
    GPIO_Init(UART1_GPIO, &GPIO_InitStructure);

    /* Connect alternate function */
    GPIO_PinAFConfig(UART1_GPIO, UART1_TX_PIN_SOURCE, GPIO_AF_USART1);
    GPIO_PinAFConfig(UART1_GPIO, UART1_RX_PIN_SOURCE, GPIO_AF_USART1);
#endif /* RT_USING_UART1 */

#ifdef RT_USING_UART2
    /* Configure USART2 Rx/tx PIN */
    GPIO_InitStructure.GPIO_Pin = UART2_GPIO_RX | UART2_GPIO_TX;
    GPIO_Init(UART2_GPIO, &GPIO_InitStructure);

    /* Connect alternate function */
    GPIO_PinAFConfig(UART2_GPIO, UART2_TX_PIN_SOURCE, GPIO_AF_USART2);
    GPIO_PinAFConfig(UART2_GPIO, UART2_RX_PIN_SOURCE, GPIO_AF_USART2);
#endif /* RT_USING_UART2 */

#ifdef RT_USING_UART3
    /* Configure USART3 Rx/tx PIN */
    GPIO_InitStructure.GPIO_Pin = UART3_GPIO_TX | UART3_GPIO_RX;
    GPIO_Init(UART3_GPIO, &GPIO_InitStructure);

    /* Connect alternate function */
    GPIO_PinAFConfig(UART3_GPIO, UART3_TX_PIN_SOURCE, GPIO_AF_USART3);
    GPIO_PinAFConfig(UART3_GPIO, UART3_RX_PIN_SOURCE, GPIO_AF_USART3);
#endif /* RT_USING_UART3 */

#ifdef RT_USING_UART4
    /* Configure USART4 Rx/tx PIN */
    GPIO_InitStructure.GPIO_Pin = UART4_GPIO_TX | UART4_GPIO_RX;
    GPIO_Init(UART4_GPIO, &GPIO_InitStructure);

    /* Connect alternate function */
    GPIO_PinAFConfig(UART4_GPIO, UART4_TX_PIN_SOURCE, GPIO_AF_UART4);
    GPIO_PinAFConfig(UART4_GPIO, UART4_RX_PIN_SOURCE, GPIO_AF_UART4);
#endif /* RT_USING_UART4 */

#ifdef RT_USING_UART6
    /* Configure USART6 Rx/tx PIN */
    GPIO_InitStructure.GPIO_Pin = UART6_GPIO_RX | UART6_GPIO_TX;
    GPIO_Init(UART6_GPIO, &GPIO_InitStructure);

    /* Connect alternate function */
    GPIO_PinAFConfig(UART6_GPIO, UART6_TX_PIN_SOURCE, GPIO_AF_USART6);
    GPIO_PinAFConfig(UART6_GPIO, UART6_RX_PIN_SOURCE, GPIO_AF_USART6);
#endif /* RT_USING_UART6 */
}

static void NVIC_Configuration(struct stm32_uart *uart)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = uart->irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

int stm32_hw_usart_init(void)
{
    struct stm32_uart *uart;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    RCC_Configuration();
    GPIO_Configuration();

#ifdef RT_USING_UART1
    uart = &uart1;

    serial1.ops    = &stm32_uart_ops;
    serial1.config = config;

    NVIC_Configuration(&uart1);

    /* register UART1 device */
    rt_hw_serial_register(&serial1,
                          "uart1",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          uart);
#endif /* RT_USING_UART1 */

#ifdef RT_USING_UART2
    uart = &uart2;

    serial2.ops    = &stm32_uart_ops;
    serial2.config = config;

    NVIC_Configuration(&uart2);

    /* register UART1 device */
    rt_hw_serial_register(&serial2,
                          "uart2",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          uart);
#endif /* RT_USING_UART2 */

#ifdef RT_USING_UART3
    uart = &uart3;

    serial3.ops    = &stm32_uart_ops;
    serial3.config = config;

    NVIC_Configuration(&uart3);

    /* register UART3 device */
    rt_hw_serial_register(&serial3,
                          "uart3",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          uart);
#endif /* RT_USING_UART3 */

#ifdef RT_USING_UART4
    uart = &uart4;
	struct serial_configure serial4_config = RT_SERIAL4_CONFIG;

    serial4.ops    = &stm32_uart_ops;
    serial4.config = serial4_config;

    NVIC_Configuration(&uart4);

    /* register UART4 device */
    rt_hw_serial_register(&serial4,
                          "uart4",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          uart);
#endif /* RT_USING_UART4 */

#ifdef RT_USING_UART6
    uart = &uart6;

//    serial6.ops    = &stm32_uart_ops;
//    serial6.config = config;
	
	struct serial_configure serial6_config = RT_SERIAL6_CONFIG;

    serial6.ops    = &stm32_uart_ops;
    serial6.config = serial6_config;

    NVIC_Configuration(&uart6);

    /* register UART6 device */
    rt_hw_serial_register(&serial6,
                          "uart6",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          uart);
#endif /* RT_USING_UART6 */

    return 0;
}
INIT_BOARD_EXPORT(stm32_hw_usart_init);
