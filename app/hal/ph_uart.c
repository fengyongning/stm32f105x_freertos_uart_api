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
 *   ph_uart.c
 *
 * Project:
 * --------
 *
 *
 * Description:
 * ------------
 *   The module implements UART related APIs.
 *
 * Author:
 * -------
 * -------
 *  Designed by     :
 *  Coded    by     :
 *  Tested   by     :
 *============================================================================
 *             HISTORY
 *----------------------------------------------------------------------------
 *
 ****************************************************************************/
#include <stdarg.h>
#include "ph_common.h"


/***********************************************************************
 * MACRO CONSTANT DEFINITIONS
************************************************************************/

/***********************************************************************
 * VARIABLES DEFINITIONS
************************************************************************/
u8 debugBuffer[DEBUG_UART_TX_BUF_LENGTH];
UART_HandleTypeDef UARTPort[PH_UART_MAX_NUM];
PhUart_enum debug_port;



void ph_uart_irq_enable(PhUart_enum port, u32 irq)
{
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)&UARTPort[port];

    __HAL_UART_ENABLE_IT(huart,irq);
}

void ph_uart_irq_disable(PhUart_enum port, u32 irq)
{
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)&UARTPort[port];

    __HAL_UART_DISABLE_IT(huart,irq);
}


/*****************************************************************
* Function:     ph_uart_irq_handle
*
* Description:
*               This function handle the uart IRQ.
* Return:
*               None
*
*****************************************************************/
void ph_uart_irq_handle(u8 port)
{
    if(port == PH_UART_PORT1)
    {
        HAL_UART_IRQHandler((UART_HandleTypeDef *)&UARTPort[PH_UART_PORT1]);
    }
    else if(port == PH_UART_PORT2)
    {
        HAL_UART_IRQHandler((UART_HandleTypeDef *)&UARTPort[PH_UART_PORT2]);
    }
    else if(port == PH_UART_PORT3)
    {
        HAL_UART_IRQHandler((UART_HandleTypeDef *)&UARTPort[PH_UART_PORT3]);
    }
    else if(port == PH_UART_PORT4)
    {
        HAL_UART_IRQHandler((UART_HandleTypeDef *)&UARTPort[PH_UART_PORT4]);
    }
	else if(port == PH_UART_PORT5)
    {
        HAL_UART_IRQHandler((UART_HandleTypeDef *)&UARTPort[PH_UART_PORT5]);
    }
}

/*****************************************************************
* Function:     ph_uart_open
*
* Description:
*               This function opens a debug UART port with the
*               specified parameters.
* Return:
*               PH_RET_OK indicates success; otherwise failure.
*
*****************************************************************/
s32 ph_uart_open(PhUart_enum port, PhUartConfig_st *cfg)
{
    UART_HandleTypeDef *huart;
    void *buffer1 = NULL;
    void *buffer2 = NULL;
    void *buffer3 = NULL;
    void *buffer4 = NULL;
    IRQn_Type uartIRQ;
    IRQn_Type dmaIRQ;

    if((port >= PH_UART_MAX_NUM) || (port == PH_UART_PORT5))
    {
        return PH_RET_ERR_INVALID_PORT;
    }

    huart = (UART_HandleTypeDef *)&UARTPort[port];

	if(huart->Lock == HAL_LOCKED)
	{
		return PH_RET_ERR_PORT_ALREADY_OPENED;
	}

	if(cfg->txBuffMaxSize == 0)
	{
		cfg->txBuffMaxSize = PH_UART_TX_BUF_LENGTH;    //默认值
	}

	if(cfg->rxBuffMaxSize == 0)
	{
		cfg->rxBuffMaxSize = PH_UART_RX_BUF_LENGTH;    //默认值
	}

    if(huart->TxRingBuff.ptrBuf == NULL)
    {
        /*分配发送数据缓存空间*/
        buffer1 = pvPortMalloc(cfg->txBuffMaxSize);
        if(buffer1 == NULL)
        {
            return PH_RET_ERR_MALLOC_MEM;
        }
        huart->TxRingBuff.read = 0;
        huart->TxRingBuff.write = 0;
        huart->TxRingBuff.length = cfg->txBuffMaxSize;
        huart->TxRingBuff.ptrBuf = (u8 *)buffer1;
        huart->TxXferCount = 0;
    }

    if(huart->RxRingBuff.ptrBuf == NULL)
    {
        /*分配接收数据缓存空间*/
        buffer2 = pvPortMalloc(cfg->rxBuffMaxSize);
        if(buffer2 == NULL)
        {
            vPortFree(buffer1);
            return PH_RET_ERR_MALLOC_MEM;
        }
        huart->RxRingBuff.read = 0;
        huart->RxRingBuff.write = 0;
        huart->RxRingBuff.length = cfg->rxBuffMaxSize;
        huart->RxRingBuff.ptrBuf = (u8 *)buffer2;
        huart->RxXferCount = 0;
    }

    if(huart->hdmarx == NULL)
    {
        /*分配接收DMA缓存，发送无需DMA*/
        buffer3 = pvPortMalloc(sizeof(DMA_HandleTypeDef));
        if(buffer3 == NULL)
        {
            vPortFree(buffer2);
            vPortFree(buffer1);
            return PH_RET_ERR_MALLOC_MEM;
        }
        huart->hdmarx = (DMA_HandleTypeDef *)buffer3;
    }

    if(huart->hdmarxBuff.ptrBuf == NULL)
    {
        /*分配接收数据缓存空间*/
        buffer4 = pvPortMalloc(cfg->rxBuffMaxSize);
        if(buffer4 == NULL)
        {
            vPortFree(buffer3);
            vPortFree(buffer2);
            vPortFree(buffer1);
            return PH_RET_ERR_MALLOC_MEM;
        }
        huart->hdmarxBuff.read = 0;
        huart->hdmarxBuff.write = 0;
        huart->hdmarxBuff.length = cfg->rxBuffMaxSize;
        huart->hdmarxBuff.ptrBuf = (u8 *)buffer4;
    }

    huart->portId = (u8)port;
    huart->TxCompletedCallbak = cfg->txCompletedCb;
	huart->RxCompletedCallbak = cfg->rxCompletedCb;

    /* Enable UART clock */
    switch(port)
    {
        case PH_UART_PORT1:
            huart->Instance = USART1;
            uartIRQ = USART1_IRQn;
            huart->hdmarx->Instance = UART1_RX_DMA_CHANNEL;
            dmaIRQ = UART1_RX_DMA_CHANNEL_IRQ;
            __HAL_RCC_USART1_CLK_ENABLE();
            break;
        case PH_UART_PORT2:
            huart->Instance = USART2;
            uartIRQ = USART2_IRQn;
            huart->hdmarx->Instance = UART2_RX_DMA_CHANNEL;
            dmaIRQ = UART2_RX_DMA_CHANNEL_IRQ;
            __HAL_RCC_USART2_CLK_ENABLE();
            break;
        case PH_UART_PORT3:
            huart->Instance = USART3;
            uartIRQ = USART3_IRQn;
            huart->hdmarx->Instance = UART3_RX_DMA_CHANNEL;
            dmaIRQ = UART3_RX_DMA_CHANNEL_IRQ;
            __HAL_RCC_USART3_CLK_ENABLE();
            break;
        case PH_UART_PORT4:
            huart->Instance = UART4;
            uartIRQ = UART4_IRQn;
			huart->hdmarx->Instance = UART4_RX_DMA_CHANNEL;
            dmaIRQ = UART4_RX_DMA_CHANNEL_IRQ;
            __HAL_RCC_UART4_CLK_ENABLE();
            break;
    }

    HAL_UART_DeInit(huart);

    /* Set DMA for Uart RX */
	if(port != PH_UART_PORT4)
	{
		ph_dma1_clock_enable();
	}
	else
	{
		ph_dma2_clock_enable();
	}
	ph_nvic_irq_disable(dmaIRQ);
    huart->hdmarx->Init.Direction = DMA_PERIPH_TO_MEMORY;
    huart->hdmarx->Init.PeriphInc = DMA_PINC_DISABLE;
    huart->hdmarx->Init.MemInc = DMA_MINC_ENABLE;
    huart->hdmarx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    huart->hdmarx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    huart->hdmarx->Init.Mode = DMA_NORMAL;
    huart->hdmarx->Init.Priority = DMA_PRIORITY_VERY_HIGH;
    huart->hdmarx->State = HAL_DMA_STATE_RESET;
    HAL_DMA_Init(huart->hdmarx);
    HAL_DMA_Start(huart->hdmarx,(u32)&huart->Instance->DR,(u32)&huart->hdmarxBuff.ptrBuf[0],huart->hdmarxBuff.length);
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT);
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    /* Set interrupt for Uart */
    ph_irq_priority_set(uartIRQ, NORMAL_UART_IRQ_PRIORITY);
    ph_uart_irq_disable(port, UART_IT_TXE);
    ph_uart_irq_disable(port, UART_IT_TC);
    ph_uart_irq_disable(port, UART_IT_RXNE);
    ph_uart_irq_enable(port, UART_IT_IDLE);     // For DMA receive
    ph_nvic_irq_enable(uartIRQ);

    /* Configure Uart and enable it */
    huart->Init.BaudRate = (u32)cfg->baudrate;
    if(cfg->dataBits == PH_UART_DATA_BITS_9)
    {
        huart->Init.WordLength = UART_WORDLENGTH_9B;
    }
    else
    {
        huart->Init.WordLength = UART_WORDLENGTH_8B;
    }

    if(cfg->stopBits == PH_UART_STOP_BITS_2)
    {
        huart->Init.StopBits = UART_STOPBITS_2;
    }
    else
    {
        huart->Init.StopBits = UART_STOPBITS_1;
    }

    if(cfg->parity == PH_UART_PARITY_EVEN)
    {
        huart->Init.Parity = UART_PARITY_EVEN;
    }
    else if(cfg->parity == PH_UART_PARITY_ODD)
    {
        huart->Init.Parity = UART_PARITY_ODD;
    }
    else
    {
        huart->Init.Parity = UART_PARITY_NONE;
    }

    if(cfg->flowCtrl == PH_UART_FLOW_CTRL_RTS)
    {
        huart->Init.HwFlowCtl = UART_HWCONTROL_RTS;
    }
    else if(cfg->flowCtrl == PH_UART_FLOW_CTRL_CTS)
    {
        huart->Init.HwFlowCtl = UART_HWCONTROL_CTS;
    }
    else if(cfg->flowCtrl == PH_UART_FLOW_CTRL_RTS_CTS)
    {
        huart->Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
    }
    else
    {
        huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    }
    huart->Init.Mode = UART_MODE_TX_RX;
    huart->State = HAL_UART_STATE_RESET;
    HAL_UART_Init(huart);

	__HAL_LOCK(huart);

    return PH_RET_OK;
}


s32 ph_uart_close(PhUart_enum port)
{
	UART_HandleTypeDef *huart;

	if(port >= PH_UART_MAX_NUM)
    {
        return PH_RET_ERR_INVALID_PORT;
    }

	huart = (UART_HandleTypeDef *)&UARTPort[port];
	
	HAL_DMA_DeInit(huart->hdmarx);
	HAL_UART_DeInit(huart);	

	switch(port)
    {
        case PH_UART_PORT1:
            __HAL_RCC_USART1_CLK_DISABLE();
            break;
        case PH_UART_PORT2:
            __HAL_RCC_USART2_CLK_DISABLE();
            break;
        case PH_UART_PORT3:
            __HAL_RCC_USART3_CLK_DISABLE();
            break;
        case PH_UART_PORT4:
            __HAL_RCC_UART4_CLK_DISABLE();
            break;
    }

	if(port != PH_UART_PORT4)
	{
		ph_dma1_clock_disable();
	}
	else
	{
		ph_dma2_clock_disable();
	}

	if(huart->hdmarxBuff.ptrBuf != NULL)
	{
		vPortFree(huart->hdmarxBuff.ptrBuf);
		huart->hdmarxBuff.ptrBuf = NULL;
	}

	if(huart->hdmarx != NULL)
	{
		vPortFree(huart->hdmarx);
		huart->hdmarx = NULL;
	}

	if(huart->RxRingBuff.ptrBuf != NULL)
	{
		vPortFree(huart->RxRingBuff.ptrBuf);
		huart->RxRingBuff.ptrBuf = NULL;
	}

	if(huart->TxRingBuff.ptrBuf != NULL)
	{
		vPortFree(huart->TxRingBuff.ptrBuf);
		huart->TxRingBuff.ptrBuf = NULL;
	}
	
	__HAL_UNLOCK(huart);

	return PH_RET_OK;
}

/*****************************************************************
* Function:     ph_uart_write
*
* Description:
*               This function opens a UART port with the
*               specified parameters.
* Return:
*               PH_RET_OK indicates success; otherwise failure.
*
*****************************************************************/
u16 ph_uart_write(PhUart_enum port, u8 *msg, u16 size)
{
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)&UARTPort[port];
    u32 tmp_it_source = 0;
    u16 actual_len = size;
    u16 length = 0;
    u16 write_idx = 0;
    u16 read_idx = 0;
    u16 buf_depth = 0;

    if(huart->State == HAL_UART_STATE_RESET)
    {
        return 0;
    }

    write_idx = huart->TxRingBuff.write;
    read_idx = huart->TxRingBuff.read;
    buf_depth = huart->TxRingBuff.length;

    /*如果buf已写满则不能再写入*/
    if((write_idx+1)%buf_depth == read_idx)
    {
        return 0;
    }

    /*根据buf可用空间大小调整发送数据长度*/
    if(write_idx > read_idx)
    {
        if((write_idx+size) > buf_depth)
        {
            if(((write_idx+size)%buf_depth) > read_idx)
            {
                actual_len = buf_depth - (write_idx - read_idx);
            }
        }
    }
    else if(write_idx < read_idx)
    {
        if((read_idx - write_idx) < size)
        {
            actual_len = read_idx - write_idx;
        }
    }
    else
    {
        if(size > buf_depth)
        {
            actual_len = buf_depth;
        }
    }

    if((write_idx+actual_len) <= buf_depth)
    {
        /*write没有越界*/
        memcpy((u8 *)&huart->TxRingBuff.ptrBuf[write_idx], msg, actual_len);
    }
    else
    {
        /*write有越界*/
        length = buf_depth-write_idx;
        memcpy((u8 *)&huart->TxRingBuff.ptrBuf[write_idx], msg, length);
        memcpy((u8 *)&huart->TxRingBuff.ptrBuf[0], msg+length, actual_len-length);
    }

    huart->TxRingBuff.write = (write_idx+actual_len)%buf_depth;

    /*启动发送*/
    tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_TXE) | __HAL_UART_GET_IT_SOURCE(huart, UART_IT_TC);
    if(tmp_it_source == RESET)
    {
        huart->TxXferCount = actual_len;

        ph_uart_irq_enable(port, UART_IT_TXE);
    }

    return actual_len;
}


/*****************************************************************
* Function:     ph_uart_read
*
* Description:
*               This function opens a UART port with the
*               specified parameters.
* Return:
*               PH_RET_OK indicates success; otherwise failure.
*
*****************************************************************/
u16 ph_uart_read(PhUart_enum port, u8 *msg, u16 rdLen)
{
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)&UARTPort[port];
    u16 recv_len = 0;
    u16 read_id;
    u16 write_id;
    u16 buf_depth;
    u16 length;

    read_id = huart->RxRingBuff.read;
    write_id = huart->RxRingBuff.write;
    buf_depth = huart->RxRingBuff.length;
    /*计算已接收数据长度*/
    if(write_id >= read_id)
    {
        recv_len = write_id-read_id;
    }
    else
    {
        recv_len = buf_depth-(read_id-write_id);
    }

    /*根据实际接收长度*/
    if(recv_len > 0)
    {
        if(rdLen < recv_len)
        {
            recv_len = rdLen;
        }

        if((read_id+recv_len) <= buf_depth)
        {
            /*read没有越界*/
            memcpy(msg, (u8 *)&huart->RxRingBuff.ptrBuf[read_id], recv_len);
        }
        else
        {
            /*read有越界*/
            length = buf_depth-read_id;
            memcpy(msg, (u8 *)&huart->RxRingBuff.ptrBuf[read_id], length);
            memcpy(msg+length, (u8 *)&huart->RxRingBuff.ptrBuf[0], recv_len-length);
        }

        huart->RxRingBuff.read = (read_id+recv_len)%buf_depth;
    }

    huart->RxXferCount = 0;

    return recv_len;
}

/*****************************************************************
* Function:     ph_uart_debug_open
*
* Description:
*               This function opens a debug UART port with the
*               specified parameters.
* Return:
*               PH_RET_OK indicates success; otherwise failure.
*
*****************************************************************/
s32 ph_uart_debug_open(PhUart_enum port, PhUartBaudrate bandrate)
{
    UART_HandleTypeDef *huart;
    IRQn_Type uartIRQ;

    if(port >= PH_UART_MAX_NUM)
    {
        return PH_RET_ERR_INVALID_PORT;
    }

    huart = (UART_HandleTypeDef *)&UARTPort[port];

    if(huart->TxRingBuff.ptrBuf == NULL)
    {
        memset(debugBuffer, 0, DEBUG_UART_TX_BUF_LENGTH);
        /*debug串口静态分配缓存*/
        huart->TxRingBuff.read = 0;
        huart->TxRingBuff.write = 0;
        huart->TxRingBuff.length = DEBUG_UART_TX_BUF_LENGTH;
        huart->TxRingBuff.ptrBuf = debugBuffer;
        huart->TxXferCount = 0;
    }

    /* Enable UART clock */
    switch(port)
    {
        case PH_UART_PORT1:
            huart->Instance = USART1;
            uartIRQ = USART1_IRQn;
            __HAL_RCC_USART1_CLK_ENABLE();
            break;
        case PH_UART_PORT2:
            huart->Instance = USART2;
            uartIRQ = USART2_IRQn;
            __HAL_RCC_USART2_CLK_ENABLE();
            break;
        case PH_UART_PORT3:
            huart->Instance = USART3;
            uartIRQ = USART3_IRQn;
            __HAL_RCC_USART3_CLK_ENABLE();
            break;
        case PH_UART_PORT4:
            huart->Instance = UART4;
            uartIRQ = UART4_IRQn;
            __HAL_RCC_UART4_CLK_ENABLE();
            break;
		case PH_UART_PORT5:
            huart->Instance = UART5;
            uartIRQ = UART5_IRQn;
            __HAL_RCC_UART5_CLK_ENABLE();
            break;
    }

    debug_port = port;
    huart->portId = (u8)port;
	huart->TxCompletedCallbak = NULL;
	huart->RxCompletedCallbak = NULL;
    HAL_UART_DeInit(huart);

    /* Set interrupt for Uart */
    ph_irq_priority_set(uartIRQ, TRACE_UART_IRQ_PRIORITY);
    ph_uart_irq_disable(port, UART_IT_TXE);
    ph_uart_irq_disable(port, UART_IT_TC);
    ph_uart_irq_disable(port, UART_IT_RXNE);
    ph_uart_irq_disable(port, UART_IT_IDLE);
    ph_nvic_irq_enable(uartIRQ);

    __HAL_UART_FLUSH_DRREGISTER(huart);
    /* Configure Uart and enable it */
    huart->Init.BaudRate = (u32)bandrate;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.StopBits = UART_STOPBITS_1;
    huart->Init.Parity = UART_PARITY_NONE;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.Mode = UART_MODE_TX;
    huart->State = HAL_UART_STATE_RESET;
    HAL_UART_Init(huart);

    return PH_RET_OK;
}

/*****************************************************************
* Function:     ph_uart_debug_send
*
* Description:
*               This function opens a debug UART port with the
*               specified parameters.
* Return:
*               PH_RET_OK indicates success; otherwise failure.
*
*****************************************************************/
s32 ph_uart_debug_send(u8 *msg, u16 size)
{
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)&UARTPort[debug_port];
    u32 tmp_it_source = 0;
    u16 actual_len = size;
    u16 length = 0;
    u16 write_idx = 0;
    u16 read_idx = 0;
    u16 buf_depth = 0;

    if(huart->State == HAL_UART_STATE_RESET)
    {
        return PH_RET_ERR_GENERAL;
    }

    write_idx = huart->TxRingBuff.write;
    read_idx = huart->TxRingBuff.read;
    buf_depth = huart->TxRingBuff.length;

    /*如果buf已写满则不能再写入*/
    if((write_idx+1)%buf_depth == read_idx)
    {
        return PH_RET_ERR_GENERAL;
    }

    /*根据buf可用空间大小调整发送数据长度*/
    if(write_idx > read_idx)
    {
        if((write_idx+size) > buf_depth)
        {
            if(((write_idx+size)%buf_depth) > read_idx)
            {
                actual_len = buf_depth - (write_idx - read_idx);
            }
        }
    }
    else if(write_idx < read_idx)
    {
        if((read_idx - write_idx) < size)
        {
            actual_len = read_idx - write_idx;
        }
    }
    else
    {
        if(size > buf_depth)
        {
            actual_len = buf_depth;
        }
    }

    if((write_idx+actual_len) <= buf_depth)
    {
        /*write没有越界*/
        memcpy((u8 *)&huart->TxRingBuff.ptrBuf[write_idx], msg, actual_len);
    }
    else
    {
        /*write有越界*/
        length = buf_depth-write_idx;
        memcpy((u8 *)&huart->TxRingBuff.ptrBuf[write_idx], msg, length);
        memcpy((u8 *)&huart->TxRingBuff.ptrBuf[0], msg+length, actual_len-length);
    }

    huart->TxRingBuff.write = (write_idx+actual_len)%buf_depth;

    /*启动发送*/
    tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_TXE) | __HAL_UART_GET_IT_SOURCE(huart, UART_IT_TC);
    if(tmp_it_source == RESET)
    {
        huart->TxXferCount = actual_len;

        ph_uart_irq_enable(debug_port, UART_IT_TXE);
    }

    return PH_RET_OK;
}


void ph_debug_print(u8 level, s8 *fmt, ...)
{
	int ret;
    va_list ap;
    char buf[128];

    va_start(ap,fmt);
    ret = vsnprintf((char *)buf,128,(char const *)fmt,ap);
    va_end(ap);

	if(ret > 0)
	{
    	ph_uart_debug_send((u8 *)&buf[0], (u16)strlen(buf));
	}
}




