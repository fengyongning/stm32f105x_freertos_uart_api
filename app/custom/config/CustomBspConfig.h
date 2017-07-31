#ifndef __CUSTOM_BSP_CONFIG_H__
#define __CUSTOM_BSP_CONFIG_H__


#include "FreeRTOSConfig.h"
#include "cmsis_os.h"

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "stm32f105x_mnc.h"
#include "core_cm3.h"
#include "ph_uart.h"


/************************************************************************
 * 用户定义功能模块使能/禁止
 ************************************************************************/
#define DEBUG_CMD_MODULE_ENABLED

/************************************************************************
 * ALL PERIPHERAL IRQ PRIORITY
 ************************************************************************/
#define	NORMAL_UART_IRQ_PRIORITY				5
#define TRACE_UART_IRQ_PRIORITY                                 6

/************************************************************************
 * DMA  通道分配有一定的对应关系，不能随意更改，需要更改时查阅手册
 ************************************************************************/
#define UART1_RX_DMA_CHANNEL					DMA1_Channel5
#define UART2_RX_DMA_CHANNEL					DMA1_Channel6
#define UART3_RX_DMA_CHANNEL					DMA1_Channel3
#define UART4_RX_DMA_CHANNEL					DMA2_Channel3
#define UART1_RX_DMA_CHANNEL_IRQ				DMA1_Channel5_IRQn
#define UART2_RX_DMA_CHANNEL_IRQ				DMA1_Channel6_IRQn
#define UART3_RX_DMA_CHANNEL_IRQ				DMA1_Channel3_IRQn
#define UART4_RX_DMA_CHANNEL_IRQ				DMA2_Channel3_IRQn

/************************************************************************
 * UART 分配
 ************************************************************************/
/*
DEBUG_CMD_PORT:   1      //调试命令接口
GNSS_UART_PORT:   2       
BLE_UART_PORT:    3
MODEM_UART_PORT:  4
DEBUG_TRACE_PORT: 5      //调试信息输出口
*/
#define DEBUG_CMD_PORT							PH_UART_PORT1


/************************************************************************
 * TIMER 分配
 ************************************************************************/


/************************************************************************
 * GPRS 
 ************************************************************************/


#endif  //__CUSTOM_BSP_CONFIG_H__