/**
  ******************************************************************************
  * @file    modem.c
  * @author
  * @version
  * @date
  * @brief
  ******************************************************************************
  */
#include "CustomBspConfig.h"

#ifdef DEBUG_CMD_MODULE_ENABLED
#include <string.h>
#include "debug_cmd.h"
#include "os_interface.h"

/* Private macro -------------------------------------------------------------*/
#define DEBUG_CMD_TX_MAX_SIZE						128
#define DEBUG_CMD_RX_MAX_SIZE						128

/* Private variables ---------------------------------------------------------*/
static u8 debug_cmd_rx[DEBUG_CMD_RX_MAX_SIZE];

/**
  * @brief
  * @param
  * @retval
  */
bool DebugCmdParse(u8 *msg, u16 size)
{
    /*串口接收到的内容，在这里处理，这里只简单的回显*/
    DebugCmdSend(msg, size);

    return TRUE;
}

/**
  * @brief
  * @param
  * @retval
  */
void DebugCmdRxProc(uint8_t port)
{
    u16 len;

    len = ph_uart_read(port, (u8 *)debug_cmd_rx, DEBUG_CMD_RX_MAX_SIZE);
    if(len > 0)
    {
        debug_cmd_rx[len] = 0;

        DebugCmdParse((u8 *)debug_cmd_rx, len);
    }
}

/**
  * @brief
  * @param
  * @retval
  */
u16 DebugCmdSend(u8 *msg, u16 size)
{
    return ph_uart_write(DEBUG_CMD_PORT, (u8 *)msg, size);
}

/**
  * @brief
  * @param
  * @retval
  */
bool DebugCmdSetup(void)
{
	s32 ret;
    PhUartConfig_st config;

    config.baudrate = PH_UART_BAUD_115200;
    config.dataBits = PH_UART_DATA_BITS_8;
    config.stopBits = PH_UART_STOP_BITS_1;
    config.parity = PH_UART_PARITY_NONE;
    config.flowCtrl = PH_UART_FLOW_CTRL_NONE;
    config.rxCompletedCb = DebugCmdRxProc;
    config.txCompletedCb = NULL;
	config.rxBuffMaxSize = DEBUG_CMD_RX_MAX_SIZE;
	config.txBuffMaxSize = DEBUG_CMD_TX_MAX_SIZE;
    ret = ph_uart_open(DEBUG_CMD_PORT, &config);
    if(ret == PH_RET_OK)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}


#endif

