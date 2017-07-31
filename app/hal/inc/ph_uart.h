/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of Quectel Co., Ltd. 2013
*
*****************************************************************************/
/*****************************************************************************
 *
 * Filename:
 * ---------
 *   ph_uart.h
 *
 * Project:
 * --------
 *   OpenCPU
 *
 * Description:
 * ------------
 *  Uart APIs defines.
 *
 * Author:
 * -------
 * -------
 *
 *============================================================================
 *             HISTORY
 *----------------------------------------------------------------------------
 *
 ****************************************************************************/
#ifndef __PH_UART_H__
#define __PH_UART_H__

#include "ph_type.h"

/****************************************************************************
 * Micro Definitions
 ***************************************************************************/
#define	DEBUG_UART_TX_BUF_LENGTH					256
#define	PH_UART_TX_BUF_LENGTH						256
#define	PH_UART_RX_BUF_LENGTH						1024


/****************************************************************************
 * Type Definitions
 ***************************************************************************/
/** UART port enum.     
 *  This enumerate all the UART ports!
 */
typedef enum {
    PH_UART_PORT1, 
    PH_UART_PORT2, 
    PH_UART_PORT3,
    PH_UART_PORT4,			
    PH_UART_PORT5,
    PH_UART_MAX_NUM,
    PH_UART_NULL = 99
} PhUart_enum;

/** UART bandrate type. */
typedef enum {
	PH_UART_BAUD_1200          =1200,
	PH_UART_BAUD_2400          =2400,
	PH_UART_BAUD_4800          =4800,
	PH_UART_BAUD_9600          =9600,
	PH_UART_BAUD_19200         =19200,
	PH_UART_BAUD_38400         =38400,
	PH_UART_BAUD_57600         =57600,
	PH_UART_BAUD_115200        =115200,
	PH_UART_BAUD_230400        =230400,
	PH_UART_BAUD_460800        =460800
} PhUartBaudrate;

/** UART data bits mode. */
typedef enum {
    PH_UART_DATA_BITS_5=5,	/*stm32f1xx is not support*/
    PH_UART_DATA_BITS_6,		/*stm32f1xx is not support*/
    PH_UART_DATA_BITS_7,		/*stm32f1xx is not support*/
    PH_UART_DATA_BITS_8,		
    PH_UART_DATA_BITS_9
} PhUartDataBits_enum;

/** UART stop bits mode. */
typedef enum {
    PH_UART_STOP_BITS_1=1,
	PH_UART_STOP_BITS_0_5,	/*stm32f1xx is not support*/
    PH_UART_STOP_BITS_2,
    PH_UART_STOP_BITS_1_5	/*stm32f1xx is not support*/
} PhUartStopBits_enum;

/** UART parity bits mode. */
typedef enum {
    PH_UART_PARITY_NONE=0,
    PH_UART_PARITY_ODD,
    PH_UART_PARITY_EVEN,
} PhUartParity_enum;

/** UART hwFlowCtrl bits mode. */
typedef enum {
    PH_UART_FLOW_CTRL_NONE=0,
    PH_UART_FLOW_CTRL_RTS,
    PH_UART_FLOW_CTRL_CTS,
    PH_UART_FLOW_CTRL_RTS_CTS,
} PhUartFlowCtrl_enum;

/** UART configuration structure. */
typedef struct {
   	PhUartBaudrate  baudrate; 
    PhUartDataBits_enum  dataBits;
    PhUartStopBits_enum  stopBits;
    PhUartParity_enum  parity;
	PhUartFlowCtrl_enum flowCtrl;

	uart_tx_comp_callback txCompletedCb;
	uart_rx_comp_callback rxCompletedCb;
	u16	txBuffMaxSize;
	u16 rxBuffMaxSize;
} PhUartConfig_st;

/** PH event parameter type.
 *  This struct block carries the UART port for PH_EVENT_UART_READY_RD and PH_EVENT_UART_READY_WR.
 */
typedef struct {
    PhUart_enum uart;
} PhUart_st;
/****************************************************************************
 * Function Declear
 ***************************************************************************/


/****************************************************************************
 * Function Declear
 ***************************************************************************/
void ph_uart_irq_handle(u8 port);
s32 ph_uart_open(PhUart_enum port, PhUartConfig_st *cfg);
s32 ph_uart_close(PhUart_enum port);
u16 ph_uart_write(PhUart_enum port, u8 *msg, u16 size);
u16 ph_uart_read(PhUart_enum port, u8 *msg, u16 rdLen);
s32 ph_uart_debug_open(PhUart_enum port, PhUartBaudrate bandrate);
void ph_debug_print(u8 level, s8 *fmt, ...);



#endif  // End-of __P_UART_H__

