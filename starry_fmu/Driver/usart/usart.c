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

#define UART2_GPIO_TX       GPIO_Pin_5
#define UART2_TX_PIN_SOURCE GPIO_PinSource5
#define UART2_GPIO_RX       GPIO_Pin_6
#define UART2_RX_PIN_SOURCE GPIO_PinSource6
#define UART2_GPIO          GPIOD
#define UART2_GPIO_RCC      RCC_AHB1Periph_GPIOD
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
    struct stm32_uart_dma
    {
        /* rx dma stream */
        DMA_Stream_TypeDef *rx_stream;
        /* dma channel */
        uint32_t rx_ch;
        /* dma flag */
        uint32_t rx_flag;
        /* dma irq channel */
        uint8_t rx_irq_ch;
        /* setting receive len */
        rt_size_t setting_recv_len;
        /* last receive index */
        rt_size_t last_recv_index;
        /* tx dma stream */
        DMA_Stream_TypeDef *tx_stream;
        /* dma channel */
        uint32_t tx_ch;
        /* dma flag */
        uint32_t tx_flag;
        /* dma irq channel */
        uint8_t tx_irq_ch;
    } dma;
};

static void dma_uart_tx_config(struct rt_serial_device *serial, rt_uint8_t *buf, rt_size_t size);

static void DMA_Configuration(struct rt_serial_device *serial);

static rt_err_t stm32_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct stm32_uart* uart;
    USART_InitTypeDef USART_InitStructure;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    uart = (struct stm32_uart *)serial->parent.user_data;

    USART_InitStructure.USART_BaudRate = cfg->baud_rate;

    if (cfg->data_bits == DATA_BITS_8){
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    } else if (cfg->data_bits == DATA_BITS_9) {
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    }

    if (cfg->stop_bits == STOP_BITS_1){
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
    } else if (cfg->stop_bits == STOP_BITS_2){
        USART_InitStructure.USART_StopBits = USART_StopBits_2;
    }

    if (cfg->parity == PARITY_NONE){
        USART_InitStructure.USART_Parity = USART_Parity_No;
    } else if (cfg->parity == PARITY_ODD) {
        USART_InitStructure.USART_Parity = USART_Parity_Odd;
    } else if (cfg->parity == PARITY_EVEN) {
        USART_InitStructure.USART_Parity = USART_Parity_Even;
    }

    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(uart->uart_device, &USART_InitStructure);

    /* Enable USART */
    USART_Cmd(uart->uart_device, ENABLE);

    return RT_EOK;
}

static rt_err_t stm32_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct stm32_uart* uart;
    rt_uint32_t ctrl_arg = (rt_uint32_t)(arg);

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        if (ctrl_arg == RT_DEVICE_FLAG_INT_RX)
        {
            /* disable rx irq */
            UART_DISABLE_IRQ(uart->irq);
            /* disable interrupt */
            USART_ITConfig(uart->uart_device, USART_IT_RXNE, DISABLE);
        }
        else if (ctrl_arg == RT_DEVICE_FLAG_DMA_RX)
        {
            DMA_ITConfig(uart->dma.rx_stream, DMA_IT_TC, DISABLE);
            DMA_ClearFlag(uart->dma.rx_stream, uart->dma.rx_flag);
            USART_DMACmd(uart->uart_device, USART_DMAReq_Rx, DISABLE);
            DMA_Cmd(uart->dma.rx_stream, DISABLE);
        }
        break;
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        UART_ENABLE_IRQ(uart->irq);
        /* enable interrupt */
        USART_ITConfig(uart->uart_device, USART_IT_RXNE, ENABLE);
        break;
        /* USART config */
    case RT_DEVICE_CTRL_CONFIG :
        if (ctrl_arg == RT_DEVICE_FLAG_DMA_RX) {
            DMA_Configuration(serial);
        }
    }

    return RT_EOK;
}

static int stm32_putc(struct rt_serial_device *serial, char c)
{
    struct stm32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    while (!(uart->uart_device->SR & USART_FLAG_TXE));
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

rt_size_t stm32_dma_transmit(struct rt_serial_device *serial, rt_uint8_t *buf, rt_size_t size, int direction)
{
    if (direction == RT_SERIAL_DMA_TX) {
        dma_uart_tx_config(serial, buf, size);
    }
}


/**
 * DMA initialize by DMA_InitStruct structure
 *
 * @param serial serial device
 * @param setting_recv_len setting receive length
 * @param mem_base_addr memory 0 base address for DMA stream
 */
static void dma_uart_config(struct rt_serial_device *serial, uint32_t setting_recv_len,
        void *mem_base_addr)
{
    struct stm32_uart *uart = (struct stm32_uart *) serial->parent.user_data;
    DMA_InitTypeDef DMA_InitStructure;

    /* rx dma config */
    uart->dma.setting_recv_len = setting_recv_len;
    DMA_DeInit(uart->dma.rx_stream);
    while (DMA_GetCmdStatus(uart->dma.rx_stream) != DISABLE);
    DMA_InitStructure.DMA_Channel = uart->dma.rx_ch;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(uart->uart_device->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)mem_base_addr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = uart->dma.setting_recv_len;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(uart->dma.rx_stream, &DMA_InitStructure);
}

/**
 * DMA initialize by DMA_InitStruct structure
 *
 * @param serial serial device
 * @param setting_recv_len setting receive length
 * @param mem_base_addr memory 0 base address for DMA stream
 */
static void dma_uart_tx_config(struct rt_serial_device *serial, rt_uint8_t *buf, rt_size_t size)
{
    struct stm32_uart *uart = (struct stm32_uart *) serial->parent.user_data;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    static int dma_clk_init = 0;

    /* DMA clock enable */
    if (!dma_clk_init) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
        dma_clk_init = 1;
    }

    /* tx dma config */
    DMA_DeInit(uart->dma.tx_stream);
    while (DMA_GetCmdStatus(uart->dma.tx_stream) != DISABLE);
    DMA_InitStructure.DMA_Channel = uart->dma.tx_ch;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(uart->uart_device->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = size;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(uart->dma.tx_stream, &DMA_InitStructure);

    /* tx dma interrupt config */
    NVIC_InitStructure.NVIC_IRQChannel = uart->dma.tx_irq_ch;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DMA_ClearFlag(uart->dma.tx_stream, uart->dma.tx_flag);
    DMA_ITConfig(uart->dma.tx_stream, DMA_IT_TC, ENABLE);
    USART_DMACmd(uart->uart_device, USART_DMAReq_Tx, ENABLE);
    DMA_Cmd(uart->dma.tx_stream, ENABLE);

}

/**
 * Serial port receive idle process. This need add to uart idle ISR.
 *
 * @param serial serial device
 */
static void dma_uart_rx_idle_isr(struct rt_serial_device *serial) {
    struct stm32_uart *uart = (struct stm32_uart *) serial->parent.user_data;
    rt_size_t recv_total_index, recv_len;
    rt_base_t level;

    /* disable interrupt */
    level = rt_hw_interrupt_disable();

    recv_total_index = uart->dma.setting_recv_len - DMA_GetCurrDataCounter(uart->dma.rx_stream);
    recv_len = recv_total_index - uart->dma.last_recv_index;
    uart->dma.last_recv_index = recv_total_index;
    /* enable interrupt */
    rt_hw_interrupt_enable(level);

    if (recv_len) rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_DMADONE | (recv_len << 8));

    /* read a data for clear receive idle interrupt flag */
    USART_ReceiveData(uart->uart_device);
}

/**
 * DMA receive done process. This need add to DMA receive done ISR.
 *
 * @param serial serial device
 */
static void dma_rx_done_isr(struct rt_serial_device *serial)
{
    struct stm32_uart *uart = (struct stm32_uart *) serial->parent.user_data;
    rt_size_t recv_len;
    rt_base_t level;

    if (DMA_GetFlagStatus(uart->dma.rx_stream, uart->dma.rx_flag) != RESET)
    {
        /* disable interrupt */
        level = rt_hw_interrupt_disable();

        recv_len = uart->dma.setting_recv_len - uart->dma.last_recv_index;
        /* reset last recv index */
        uart->dma.last_recv_index = 0;
        /* enable interrupt */
        rt_hw_interrupt_enable(level);

        if (recv_len) rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_DMADONE | (recv_len << 8));

        /* start receive data */
        DMA_ClearFlag(uart->dma.rx_stream, uart->dma.rx_flag);
    }
}

/**
 * DMA receive done process. This need add to DMA receive done ISR.
 *
 * @param serial serial device
 */
static void dma_tx_done_isr(struct rt_serial_device *serial)
{
    struct stm32_uart *uart = (struct stm32_uart *) serial->parent.user_data;
    rt_base_t level;

    if (DMA_GetFlagStatus(uart->dma.tx_stream, uart->dma.tx_flag) != RESET)
    {
        rt_hw_serial_isr(serial, RT_SERIAL_EVENT_TX_DMADONE);

        /* start receive data */
        DMA_ClearFlag(uart->dma.tx_stream, uart->dma.tx_flag);
    }
}


/**
 * Uart common interrupt process. This need add to uart ISR.
 *
 * @param serial serial device
 */
static void uart_isr(struct rt_serial_device *serial)
{
    struct stm32_uart *uart = (struct stm32_uart *) serial->parent.user_data;

    RT_ASSERT(uart != RT_NULL);

    if(USART_GetITStatus(uart->uart_device, USART_IT_RXNE) != RESET)
    {
        rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);
        /* clear interrupt */
        USART_ClearITPendingBit(uart->uart_device, USART_IT_RXNE);
    }
    if(USART_GetITStatus(uart->uart_device, USART_IT_IDLE) != RESET)
    {
        dma_uart_rx_idle_isr(serial);
    }
    if (USART_GetITStatus(uart->uart_device, USART_IT_TC) != RESET)
    {
        /* clear interrupt */
        USART_ClearITPendingBit(uart->uart_device, USART_IT_TC);
    }
    if (USART_GetFlagStatus(uart->uart_device, USART_FLAG_ORE) == SET)
    {
        stm32_getc(serial);
    }
}

static const struct rt_uart_ops stm32_uart_ops =
{
    stm32_configure,
    stm32_control,
    stm32_putc,
    stm32_getc,
    stm32_dma_transmit
};

#if defined(RT_USING_UART1)
/* UART1 device driver structure */
struct stm32_uart uart1 =
{
    USART1,
    USART1_IRQn,
    {
        DMA2_Stream5,
        DMA_Channel_4,
        DMA_FLAG_TCIF5,
        DMA2_Stream5_IRQn,
        0,
        0,
        DMA2_Stream7,
        DMA_Channel_4,
        DMA_FLAG_TCIF7,
        DMA2_Stream7_IRQn,
    },
};
struct rt_serial_device serial1;

void USART1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&serial1);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA2_Stream5_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    dma_rx_done_isr(&serial1);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA2_Stream7_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    dma_tx_done_isr(&serial1);

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
    {
        DMA1_Stream5,
        DMA_Channel_4,
        DMA_FLAG_TCIF5,
        DMA1_Stream5_IRQn,
        0,
        0,
        DMA1_Stream6,
        DMA_Channel_4,
        DMA_FLAG_TCIF6,
        DMA1_Stream6_IRQn,
    },
};
struct rt_serial_device serial2;

void USART2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&serial2);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Stream5_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    dma_rx_done_isr(&serial2);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Stream6_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    dma_tx_done_isr(&serial2);

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
    {
        DMA1_Stream1,
        DMA_Channel_4,
        DMA_FLAG_TCIF1,
        DMA1_Stream1_IRQn,
        0,
        0,
        DMA1_Stream3,
        DMA_Channel_4,
        DMA_FLAG_TCIF3,
        DMA1_Stream3_IRQn,
    },
};
struct rt_serial_device serial3;

void USART3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&serial3);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Stream1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    dma_rx_done_isr(&serial3);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Stream3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    dma_tx_done_isr(&serial3);

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
    {
        DMA1_Stream2,
        DMA_Channel_4,
        DMA_FLAG_TCIF2,
        DMA1_Stream2_IRQn,
        0,
        0,
        DMA1_Stream4,
        DMA_Channel_4,
        DMA_FLAG_TCIF4,
        DMA1_Stream4_IRQn,
    },
};
struct rt_serial_device serial4;

void UART4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&serial4);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Stream2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    dma_rx_done_isr(&serial4);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Stream4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    dma_tx_done_isr(&serial4);

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
    {
        DMA2_Stream1,
        DMA_Channel_5,
        DMA_FLAG_TCIF1,
        DMA2_Stream1_IRQn,
        0,
        0,
        DMA2_Stream6,
        DMA_Channel_5,
        DMA_FLAG_TCIF6,
        DMA2_Stream6_IRQn,
    },
};
struct rt_serial_device serial6;

void USART6_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&serial6);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA2_Stream1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    dma_rx_done_isr(&serial6);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA2_Stream6_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    dma_tx_done_isr(&serial6);

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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static void DMA_Configuration(struct rt_serial_device *serial) {
    struct stm32_uart *uart = (struct stm32_uart *) serial->parent.user_data;
    struct rt_serial_rx_fifo *rx_fifo = (struct rt_serial_rx_fifo *)serial->serial_rx;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* enable transmit idle interrupt */
    USART_ITConfig(uart->uart_device, USART_IT_IDLE , ENABLE);

    /* DMA clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    /* rx dma config */
    dma_uart_config(serial, serial->config.bufsz, rx_fifo->buffer);

    DMA_ClearFlag(uart->dma.rx_stream, uart->dma.rx_flag);
    DMA_ITConfig(uart->dma.rx_stream, DMA_IT_TC, ENABLE);
    USART_DMACmd(uart->uart_device, USART_DMAReq_Rx, ENABLE);
    DMA_Cmd(uart->dma.rx_stream, ENABLE);

    /* rx dma interrupt config */
    NVIC_InitStructure.NVIC_IRQChannel = uart->dma.rx_irq_ch;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* config for serial_configure structure */
#define RT_SERIAL_CONFIG                  \
{                                         \
    BAUD_RATE_57600, /* 57600 bits/s */  \
    DATA_BITS_8,      /* 8 databits */     \
    STOP_BITS_1,      /* 1 stopbit */      \
    PARITY_NONE,      /* No parity  */     \
    BIT_ORDER_LSB,    /* LSB first sent */ \
    NRZ_NORMAL,       /* Normal mode */    \
    RT_SERIAL_RB_BUFSZ * 2, /* Buffer size */  \
    0                                      \
}

#define RT_SERIAL4_CONFIG           	   \
{                                          \
    BAUD_RATE_9600, /* 115200 bits/s */  \
    DATA_BITS_8,      /* 8 databits */     \
    STOP_BITS_1,      /* 1 stopbit */      \
    PARITY_NONE,      /* No parity  */     \
    BIT_ORDER_LSB,    /* LSB first sent */ \
    NRZ_NORMAL,       /* Normal mode */    \
    RT_SERIAL_RB_BUFSZ, /* Buffer size */  \
    0                                      \
}

#define RT_SERIAL6_CONFIG           	   \
{                                          \
    BAUD_RATE_115200, /* 115200 bits/s */  \
    DATA_BITS_8,      /* 8 databits */     \
    STOP_BITS_1,      /* 1 stopbit */      \
    PARITY_NONE,      /* No parity  */     \
    BIT_ORDER_LSB,    /* LSB first sent */ \
    NRZ_NORMAL,       /* Normal mode */    \
    RT_SERIAL_RB_BUFSZ, /* Buffer size */  \
    0                                      \
}

int stm32_hw_usart_init(void)
{
    struct stm32_uart *uart;
    struct serial_configure config = RT_SERIAL_CONFIG;

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
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | 
                          RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX,
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
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | 
                          RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX,
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
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | 
                          RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX,
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
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | 
                          RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX,
                          uart);
#endif /* RT_USING_UART4 */

#ifdef RT_USING_UART6
    uart = &uart6;
    struct serial_configure serial6_config = RT_SERIAL6_CONFIG;

    serial6.ops    = &stm32_uart_ops;
    serial6.config = serial6_config;

    NVIC_Configuration(&uart6);

    /* register UART6 device */
    rt_hw_serial_register(&serial6,
                          "uart6",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | 
                          RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX,
                          uart);
#endif /* RT_USING_UART6 */

    return 0;
}
INIT_BOARD_EXPORT(stm32_hw_usart_init);
