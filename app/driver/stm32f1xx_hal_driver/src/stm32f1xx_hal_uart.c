/**
  ******************************************************************************
  * @file    stm32f1xx_hal_uart.c
  * @author  MCD Application Team
  * @version V1.0.4
  * @date    29-April-2016
  * @brief   UART HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Universal Asynchronous Receiver Transmitter (UART) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions
  *           + Peripheral State and Errors functions
  @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
  [..]
    The UART HAL driver can be used as follows:

    (#) Declare a UART_HandleTypeDef handle structure.

    (#) Initialize the UART low level resources by implementing the HAL_UART_MspInit() API:
        (##) Enable the USARTx interface clock.
        (##) UART pins configuration:
            (+++) Enable the clock for the UART GPIOs.
             (+++) Configure the USART pins (TX as alternate function pull-up, RX as alternate function Input).
        (##) NVIC configuration if you need to use interrupt process (HAL_UART_Transmit_IT()
             and HAL_UART_Receive_IT() APIs):
            (+++) Configure the USARTx interrupt priority.
            (+++) Enable the NVIC USART IRQ handle.
        (##) DMA Configuration if you need to use DMA process (HAL_UART_Transmit_DMA()
             and HAL_UART_Receive_DMA() APIs):
            (+++) Declare a DMA handle structure for the Tx/Rx channel.
            (+++) Enable the DMAx interface clock.
            (+++) Configure the declared DMA handle structure with the required
                  Tx/Rx parameters.
            (+++) Configure the DMA Tx/Rx channel.
            (+++) Associate the initialized DMA handle to the UART DMA Tx/Rx handle.
            (+++) Configure the priority and enable the NVIC for the transfer complete
                  interrupt on the DMA Tx/Rx channel.
            (+++) Configure the USARTx interrupt priority and enable the NVIC USART IRQ handle
                  (used for last byte sending completion detection in DMA non circular mode)

    (#) Program the Baud Rate, Word Length, Stop Bit, Parity, Hardware
        flow control and Mode(Receiver/Transmitter) in the huart Init structure.

    (#) For the UART asynchronous mode, initialize the UART registers by calling
        the HAL_UART_Init() API.

    (#) For the UART Half duplex mode, initialize the UART registers by calling
        the HAL_HalfDuplex_Init() API.

    (#) For the LIN mode, initialize the UART registers by calling the HAL_LIN_Init() API.

    (#) For the Multi-Processor mode, initialize the UART registers by calling
        the HAL_MultiProcessor_Init() API.

     [..]
       (@) The specific UART interrupts (Transmission complete interrupt,
            RXNE interrupt and Error Interrupts) will be managed using the macros
            __HAL_UART_ENABLE_IT() and __HAL_UART_DISABLE_IT() inside the transmit
            and receive process.

     [..]
       (@) These APIs (HAL_UART_Init() and HAL_HalfDuplex_Init()) configure also the
            low level Hardware GPIO, CLOCK, CORTEX...etc) by calling the customed
            HAL_UART_MspInit() API.

     [..]
        Three operation modes are available within this driver :

     *** Polling mode IO operation ***
     =================================
     [..]
       (+) Send an amount of data in blocking mode using HAL_UART_Transmit()
       (+) Receive an amount of data in blocking mode using HAL_UART_Receive()

     *** Interrupt mode IO operation ***
     ===================================
     [..]
       (+) Send an amount of data in non blocking mode using HAL_UART_Transmit_IT()
       (+) At transmission end of transfer HAL_UART_TxCpltCallback is executed and user can
            add his own code by customization of function pointer HAL_UART_TxCpltCallback
       (+) Receive an amount of data in non blocking mode using HAL_UART_Receive_IT()
       (+) At reception end of transfer HAL_UART_RxCpltCallback is executed and user can
            add his own code by customization of function pointer HAL_UART_RxCpltCallback
       (+) In case of transfer Error, HAL_UART_ErrorCallback() function is executed and user can
            add his own code by customization of function pointer HAL_UART_ErrorCallback

     *** DMA mode IO operation ***
     ==============================
     [..]
       (+) Send an amount of data in non blocking mode (DMA) using HAL_UART_Transmit_DMA()
       (+) At transmission end of half transfer HAL_UART_TxHalfCpltCallback is executed and user can
            add his own code by customization of function pointer HAL_UART_TxHalfCpltCallback
       (+) At transmission end of transfer HAL_UART_TxCpltCallback is executed and user can
            add his own code by customization of function pointer HAL_UART_TxCpltCallback
       (+) Receive an amount of data in non blocking mode (DMA) using HAL_UART_Receive_DMA()
       (+) At reception end of half transfer HAL_UART_RxHalfCpltCallback is executed and user can
            add his own code by customization of function pointer HAL_UART_RxHalfCpltCallback
       (+) At reception end of transfer HAL_UART_RxCpltCallback is executed and user can
            add his own code by customization of function pointer HAL_UART_RxCpltCallback
       (+) In case of transfer Error, HAL_UART_ErrorCallback() function is executed and user can
            add his own code by customization of function pointer HAL_UART_ErrorCallback
       (+) Pause the DMA Transfer using HAL_UART_DMAPause()
       (+) Resume the DMA Transfer using HAL_UART_DMAResume()
       (+) Stop the DMA Transfer using HAL_UART_DMAStop()

     *** UART HAL driver macros list ***
     =============================================
     [..]
       Below the list of most used macros in UART HAL driver.

      (+) __HAL_UART_ENABLE: Enable the UART peripheral
      (+) __HAL_UART_DISABLE: Disable the UART peripheral
      (+) __HAL_UART_GET_FLAG : Check whether the specified UART flag is set or not
      (+) __HAL_UART_CLEAR_FLAG : Clear the specified UART pending flag
      (+) __HAL_UART_ENABLE_IT: Enable the specified UART interrupt
      (+) __HAL_UART_DISABLE_IT: Disable the specified UART interrupt
      (+) __HAL_UART_GET_IT_SOURCE: Check whether the specified UART interrupt has occurred or not

     [..]
       (@) You can refer to the UART HAL driver header file for more useful macros

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/** @addtogroup STM32F1xx_HAL_Driver
  * @{
  */
#ifdef HAL_UART_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @addtogroup UART_Private_Functions   UART Private Functions
  * @{
  */
static void UART_SetConfig(UART_HandleTypeDef *huart);
static HAL_StatusTypeDef UART_Transmit_IT(UART_HandleTypeDef *huart);
static HAL_StatusTypeDef UART_EndTransmit_IT(UART_HandleTypeDef *huart);
static HAL_StatusTypeDef UART_Receive_IT(UART_HandleTypeDef *huart);

/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/

/** @defgroup UART_Exported_Functions UART Exported Functions
  * @{
  */

/** @defgroup UART_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions
  *
@verbatim
===============================================================================
            ##### Initialization and Configuration functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to initialize the USARTx or the UARTy
    in asynchronous mode.
      (+) For the asynchronous mode only these parameters can be configured:
        (++) Baud Rate
        (++) Word Length
        (++) Stop Bit
        (++) Parity
        (++) Hardware flow control
        (++) Receiver/transmitter modes
    [..]
    The HAL_UART_Init(), HAL_HalfDuplex_Init(), HAL_LIN_Init() and HAL_MultiProcessor_Init() APIs
    follow respectively the UART asynchronous, UART Half duplex, LIN and Multi-Processor
    configuration procedures (details for the procedures are available in reference manuals
    (RM0008 for STM32F10Xxx MCUs and RM0041 for STM32F100xx MCUs)).


@endverbatim
  * @{
  */

/*
  Additionnal remark: If the parity is enabled, then the MSB bit of the data written
                      in the data register is transmitted but is changed by the parity bit.
                      Depending on the frame length defined by the M bit (8-bits or 9-bits),
                      the possible UART frame formats are as listed in the following table:
    +-------------------------------------------------------------+
    |   M bit |  PCE bit  |            UART frame                 |
    |---------------------|---------------------------------------|
    |    0    |    0      |    | SB | 8 bit data | STB |          |
    |---------|-----------|---------------------------------------|
    |    0    |    1      |    | SB | 7 bit data | PB | STB |     |
    |---------|-----------|---------------------------------------|
    |    1    |    0      |    | SB | 9 bit data | STB |          |
    |---------|-----------|---------------------------------------|
    |    1    |    1      |    | SB | 8 bit data | PB | STB |     |
    +-------------------------------------------------------------+
*/

/**
  * @brief  Initializes the UART mode according to the specified parameters in
  *         the UART_InitTypeDef and create the associated handle.
  * @param  huart: Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart)
{
    /* Check the UART handle allocation */
    if(huart == NULL)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    if(huart->Init.HwFlowCtl != UART_HWCONTROL_NONE)
    {
        /* The hardware flow control is available only for USART1, USART2, USART3 */
        assert_param(IS_UART_HWFLOW_INSTANCE(huart->Instance));
        assert_param(IS_UART_HARDWARE_FLOW_CONTROL(huart->Init.HwFlowCtl));
    }
    else
    {
        assert_param(IS_UART_INSTANCE(huart->Instance));
    }
    assert_param(IS_UART_WORD_LENGTH(huart->Init.WordLength));
    assert_param(IS_UART_OVERSAMPLING(huart->Init.OverSampling));

    if(huart->State == HAL_UART_STATE_RESET)
    {
        /* Init the low level hardware */
        HAL_UART_MspInit(huart);
    }

    huart->State = HAL_UART_STATE_BUSY;

    /* Disable the peripheral */
    __HAL_UART_DISABLE(huart);

    /* Set the UART Communication parameters */
    UART_SetConfig(huart);

    /* In asynchronous mode, the following bits must be kept cleared:
       - LINEN and CLKEN bits in the USART_CR2 register,
       - SCEN, HDSEL and IREN  bits in the USART_CR3 register.*/
    CLEAR_BIT(huart->Instance->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
    CLEAR_BIT(huart->Instance->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN));

    /* Enable the peripheral */
    __HAL_UART_ENABLE(huart);

    /* Initialize the UART state */
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->State= HAL_UART_STATE_READY;

    return HAL_OK;
}


/**
  * @brief  DeInitializes the UART peripheral.
  * @param  huart: Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *huart)
{
    /* Check the UART handle allocation */
    if(huart == NULL)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    assert_param(IS_UART_INSTANCE(huart->Instance));

    huart->State = HAL_UART_STATE_BUSY;

    /* Disable the Peripheral */
    __HAL_UART_DISABLE(huart);

    huart->Instance->CR1 = 0x0;
    huart->Instance->CR2 = 0x0;
    huart->Instance->CR3 = 0x0;

    /* DeInit the low level hardware */
    HAL_UART_MspDeInit(huart);

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->State = HAL_UART_STATE_RESET;

    return HAL_OK;
}

/**
  * @brief  UART MSP Init.
  * @param  huart: Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
__weak void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_UART_MspInit can be implemented in the user file
     */
}

/**
  * @brief  UART MSP DeInit.
  * @param  huart: Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
__weak void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_UART_MspDeInit can be implemented in the user file
     */
}

/**
  * @brief  This function handles UART interrupt request.
  * @param  huart: Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    uint32_t tmp_flag = 0, tmp_it_source = 0;


    tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_PE);
    tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_PE);
    /* UART parity error interrupt occurred ------------------------------------*/
    if((tmp_flag != RESET) && (tmp_it_source != RESET))
    {
        huart->ErrorCode |= HAL_UART_ERROR_PE;
    }

    tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_FE);
    tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR);
    /* UART frame error interrupt occurred -------------------------------------*/
    if((tmp_flag != RESET) && (tmp_it_source != RESET))
    {
        huart->ErrorCode |= HAL_UART_ERROR_FE;
    }

    tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_NE);
    /* UART noise error interrupt occurred -------------------------------------*/
    if((tmp_flag != RESET) && (tmp_it_source != RESET))
    {
        huart->ErrorCode |= HAL_UART_ERROR_NE;
    }

    tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_ORE);
    /* UART Over-Run interrupt occurred ----------------------------------------*/
    if((tmp_flag != RESET) && (tmp_it_source != RESET))
    {
        huart->ErrorCode |= HAL_UART_ERROR_ORE;
    }

    tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE);
    tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_RXNE);
    /* UART in mode Receiver ---------------------------------------------------*/
    if((tmp_flag != RESET) && (tmp_it_source != RESET))
    {

    }

    tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE);
    tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE);
    /* UART in mode Receiver end --------------------------------------------*/
    if((tmp_flag != RESET) && (tmp_it_source != RESET))
    {
        UART_Receive_IT(huart);
    }

    tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_TXE);
    tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_TXE);
    /* UART in mode Transmitter ------------------------------------------------*/
    if((tmp_flag != RESET) && (tmp_it_source != RESET))
    {
        UART_Transmit_IT(huart);
    }

    tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_TC);
    tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_TC);
    /* UART in mode Transmitter end --------------------------------------------*/
    if((tmp_flag != RESET) && (tmp_it_source != RESET))
    {
        UART_EndTransmit_IT(huart);
    }

    if(huart->ErrorCode != HAL_UART_ERROR_NONE)
    {
        /* Clear all the error flag at once */
        __HAL_UART_CLEAR_PEFLAG(huart);

        /* Set the UART state ready to be able to start again the process */
        huart->State = HAL_UART_STATE_READY;

        //HAL_UART_ErrorCallback(huart);
    }
}

/**
  * @}
  */

/** @defgroup UART_Exported_Functions_Group3 Peripheral Control functions
  *  @brief   UART control functions
  *
@verbatim
  ==============================================================================
                      ##### Peripheral Control functions #####
  ==============================================================================
  [..]
    This subsection provides a set of functions allowing to control the UART:
    (+) HAL_LIN_SendBreak() API can be helpful to transmit the break character.
    (+) HAL_MultiProcessor_EnterMuteMode() API can be helpful to enter the UART in mute mode.
    (+) HAL_MultiProcessor_ExitMuteMode() API can be helpful to exit the UART mute mode by software.
    (+) HAL_HalfDuplex_EnableTransmitter() API to enable the UART transmitter and disables the UART receiver in Half Duplex mode
    (+) HAL_HalfDuplex_EnableReceiver() API to enable the UART receiver and disables the UART transmitter in Half Duplex mode

@endverbatim
  * @{
  */

/**
  * @}
  */

/** @defgroup UART_Exported_Functions_Group4 Peripheral State and Errors functions
  *  @brief   UART State and Errors functions
  *
@verbatim
  ==============================================================================
                 ##### Peripheral State and Errors functions #####
  ==============================================================================
 [..]
   This subsection provides a set of functions allowing to return the State of
   UART communication process, return Peripheral Errors occurred during communication
   process
   (+) HAL_UART_GetState() API can be helpful to check in run-time the state of the UART peripheral.
   (+) HAL_UART_GetError() check in run-time errors that could be occurred during communication.

@endverbatim
  * @{
  */


/**
  * @brief  Sends an amount of data in non blocking mode.
  * @param  huart: Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval HAL status
  */
static HAL_StatusTypeDef UART_Transmit_IT(UART_HandleTypeDef *huart)
{
    uint8_t dat;
    uint16_t tmp;
    uint16_t read_idx;
    uint16_t buf_depth;


    read_idx = huart->TxRingBuff.read;
    buf_depth = huart->TxRingBuff.length;

    if(huart->Init.WordLength == UART_WORDLENGTH_9B)
    {
        tmp = (uint16_t)(huart->TxRingBuff.ptrBuf[read_idx] & (uint16_t)0x01FF);
        huart->Instance->DR = (uint16_t)tmp;

        read_idx = (read_idx+1)%buf_depth;  //指向下一个字节
        if(huart->Init.Parity == UART_PARITY_NONE)
        {
            read_idx += 2;
        }
        else
        {
            read_idx += 1;
        }

        huart->TxRingBuff.read = read_idx;
    }
    else
    {
        dat = (uint8_t)(huart->TxRingBuff.ptrBuf[read_idx] & (uint8_t)0x00FF);
        huart->Instance->DR = dat;

        huart->TxRingBuff.read = (read_idx+1)%buf_depth;
    }

    if(--huart->TxXferCount == 0)
    {
        /* Disable the UART Transmit Complete Interrupt */
        __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);

        /* Enable the UART Transmit Complete Interrupt */
        __HAL_UART_ENABLE_IT(huart, UART_IT_TC);
    }

    return HAL_OK;
}


/**
  * @brief  Wraps up transmission in non blocking mode.
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval HAL status
  */
static HAL_StatusTypeDef UART_EndTransmit_IT(UART_HandleTypeDef *huart)
{
    uint16_t write_idx,read_idx;
    uint16_t buf_depth;


    write_idx = huart->TxRingBuff.write;
    read_idx = huart->TxRingBuff.read;
    buf_depth = huart->TxRingBuff.length;

    /*判断是否还有数据需要发送*/
    if(write_idx >= read_idx)
    {
        huart->TxXferCount = (write_idx - read_idx) % buf_depth;
    }
    else
    {
        huart->TxXferCount = buf_depth - ((read_idx - write_idx) % buf_depth);
    }

    if(huart->TxXferCount > 0)
    {
        /* Disable the UART Transmit Complete Interrupt */
        __HAL_UART_DISABLE_IT(huart, UART_IT_TC);
        /* Enable the UART Transmit Complete Interrupt */
        if(__HAL_UART_GET_IT_SOURCE(huart, UART_IT_TXE) == RESET)
        {
            __HAL_UART_ENABLE_IT(huart, UART_IT_TXE);
        }
    }
    else
    {
    	/* Disable the UART Transmit Complete Interrupt */
        __HAL_UART_DISABLE_IT(huart, UART_IT_TC);

		/* Notice user */
        if(huart->TxCompletedCallbak != NULL)
        {
           	huart->TxCompletedCallbak(huart->portId);
        }
    }

    return HAL_OK;
}


static HAL_StatusTypeDef UART_Receive_IT(UART_HandleTypeDef *huart)
{
    uint16_t count = 0;
    uint32_t temp;
    uint16_t read_idx;
    uint16_t write_idx;
    uint16_t buf_depth;

    /* Clear the idle flag */
    temp = huart->Instance->SR;
    temp = huart->Instance->DR;
    temp = temp;

    /* Disable DMA and Get data length */
    __HAL_DMA_DISABLE(huart->hdmarx);
    count = huart->hdmarxBuff.length - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    write_idx = huart->RxRingBuff.write;
    read_idx = huart->RxRingBuff.read;
    buf_depth = huart->RxRingBuff.length;

    /*如果buf已写满则不能再写入*/
    if((write_idx+1)%buf_depth != read_idx)
    {
        if(write_idx > read_idx)
        {
            if((write_idx+count) > buf_depth)
            {     
                if(((write_idx+count)%buf_depth) > read_idx)
                {
                    count = buf_depth - (write_idx - read_idx);
                }
            }
        }
        else if(write_idx < read_idx)
        {
            if((read_idx - write_idx) < count)
            {
                count = read_idx - write_idx;
            }
        }
        else
        {
            if(count > buf_depth)
            {
                count = buf_depth;
            }
        }

		if((write_idx+count) <= buf_depth)
		{
			/*write没有越界*/
        	memcpy((uint8_t *)&huart->RxRingBuff.ptrBuf[write_idx], (uint8_t *)&huart->hdmarxBuff.ptrBuf[0], count);
		}
		else
		{
			/*write有越界*/
        	temp = buf_depth-write_idx;
        	memcpy((uint8_t *)&huart->RxRingBuff.ptrBuf[write_idx], (uint8_t *)&huart->hdmarxBuff.ptrBuf[0], temp);
        	memcpy((uint8_t *)&huart->RxRingBuff.ptrBuf[0], (uint8_t *)&huart->hdmarxBuff.ptrBuf[temp], count-temp);
		}

		huart->RxRingBuff.write = (write_idx+count)%buf_depth;

		huart->RxXferCount = count;
		
		/* Notice user */
    	if(huart->RxCompletedCallbak != NULL)
    	{	
    		huart->RxCompletedCallbak(huart->portId);
    	}
    }

    /* Enable DMA */
    __HAL_UNLOCK(huart->hdmarx);
    HAL_DMA_Start(huart->hdmarx,(uint32_t)&huart->Instance->DR,(uint32_t)&huart->hdmarxBuff.ptrBuf[0],huart->hdmarxBuff.length);
    
    return HAL_OK;
}

/**
  * @brief  Configures the UART peripheral.
  * @param  huart: Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
static void UART_SetConfig(UART_HandleTypeDef *huart)
{
    uint32_t tmpreg = 0x00;

    /* Check the parameters */
    assert_param(IS_UART_BAUDRATE(huart->Init.BaudRate));
    assert_param(IS_UART_STOPBITS(huart->Init.StopBits));
    assert_param(IS_UART_PARITY(huart->Init.Parity));
    assert_param(IS_UART_MODE(huart->Init.Mode));

    /*------- UART-associated USART registers setting : CR2 Configuration ------*/
    /* Configure the UART Stop Bits: Set STOP[13:12] bits according
     * to huart->Init.StopBits value */
    MODIFY_REG(huart->Instance->CR2, USART_CR2_STOP, huart->Init.StopBits);

    /*------- UART-associated USART registers setting : CR1 Configuration ------*/
    /* Configure the UART Word Length, Parity and mode:
       Set the M bits according to huart->Init.WordLength value
       Set PCE and PS bits according to huart->Init.Parity value
       Set TE and RE bits according to huart->Init.Mode value */
    tmpreg = (uint32_t)huart->Init.WordLength | huart->Init.Parity | huart->Init.Mode ;
    MODIFY_REG(huart->Instance->CR1,
               (uint32_t)(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | USART_CR1_RE),
               tmpreg);

    /*------- UART-associated USART registers setting : CR3 Configuration ------*/
    /* Configure the UART HFC: Set CTSE and RTSE bits according to huart->Init.HwFlowCtl value */
    MODIFY_REG(huart->Instance->CR3, (USART_CR3_RTSE | USART_CR3_CTSE), huart->Init.HwFlowCtl);

    /*------- UART-associated USART registers setting : BRR Configuration ------*/
    if((huart->Instance == USART1))
    {
        huart->Instance->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK2Freq(), huart->Init.BaudRate);
    }
    else
    {
        huart->Instance->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK1Freq(), huart->Init.BaudRate);
    }
}
/**
  * @}
  */

#endif /* HAL_UART_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
