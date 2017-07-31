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
 * �û����幦��ģ��ʹ��/��ֹ
 ************************************************************************/
#define DEBUG_CMD_MODULE_ENABLED

/************************************************************************
 * ALL PERIPHERAL IRQ PRIORITY
 ************************************************************************/
#define	NORMAL_UART_IRQ_PRIORITY				5
#define TRACE_UART_IRQ_PRIORITY                                 6

/************************************************************************
 * DMA  ͨ��������һ���Ķ�Ӧ��ϵ������������ģ���Ҫ����ʱ�����ֲ�
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
 * UART ����
 ************************************************************************/
/*
DEBUG_CMD_PORT:   1      //��������ӿ�
GNSS_UART_PORT:   2       
BLE_UART_PORT:    3
MODEM_UART_PORT:  4
DEBUG_TRACE_PORT: 5      //������Ϣ�����
*/
#define DEBUG_CMD_PORT							PH_UART_PORT1


/************************************************************************
 * TIMER ����
 ************************************************************************/


/************************************************************************
 * GPRS 
 ************************************************************************/


#endif  //__CUSTOM_BSP_CONFIG_H__