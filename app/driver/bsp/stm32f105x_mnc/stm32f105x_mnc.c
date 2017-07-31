/**
  ******************************************************************************
  * @file    stm32f105x_mnc.c
  * @author  MCD Application Team
  * @version V1.0.3
  * @date    29-April-2016
  * @brief   This file provides set of firmware functions to manage:
  *          - LEDs and push-button available on STM32F1XX-Nucleo Kit
  *            from STMicroelectronics
  *          - LCD, joystick and microSD available on Adafruit 1.8" TFT LCD
  *            shield (reference ID 802)
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
#include "stm32f105x_mnc.h"
#include "core_cm3.h"

/**
* @brief STM32F105RB MNC BSP Driver version
*/
uint32_t AHB_Clock;
uint32_t APB1_Clock;
uint32_t APB2_Clock;

/** @defgroup STM32F105X_MNC Private_Functions Private Functions
  * @{
  */ 
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 64000000
  *            HCLK(Hz)                       = 64000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            PLLMUL                         = 16
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void BSP_SysClockConfig( void )
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;
    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
    if( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK )
    {
        while( 1 );
    }
    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_2 ) != HAL_OK )
    {
        while( 1 );
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
    if( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit ) != HAL_OK )
    {
        while( 1 );
    }
    /* Update the BusClock global variable */
    AHB_Clock = SystemCoreClock / 1;
    APB1_Clock = AHB_Clock / 2;
    APB2_Clock = AHB_Clock / 1;
    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config( HAL_RCC_GetHCLKFreq() / 1000 );
    /**Configure the Systick
    */
    HAL_SYSTICK_CLKSourceConfig( SYSTICK_CLKSOURCE_HCLK );
    /**Configure the Systick interrupt time
    */
    __HAL_RCC_PLLI2S_ENABLE();
    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
}

/**
  * @brief  This method returns the STM32F105X_MNC BSP Driver revision
  * @retval version : 0xXYZR (8bits for each decimal, R for RC)
  */
void BSP_GpioConfig(GPIO_TypeDef *port,uint32_t pin,uint32_t mode,uint32_t pull,uint32_t speed)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin =  pin;
	GPIO_InitStructure.Mode = mode;
	GPIO_InitStructure.Pull = pull;
	GPIO_InitStructure.Speed = speed;
	HAL_GPIO_Init(port, &GPIO_InitStructure);
}

void BSP_GpioSet(GPIO_TypeDef *port,uint16_t pin,GPIO_PinState state)
{
	HAL_GPIO_WritePin(port,pin,state);
}

GPIO_PinState BSP_GpioGet(GPIO_TypeDef *port,uint16_t pin)
{
	return HAL_GPIO_ReadPin(port,pin);
}

void BSP_GpioInit(void)
{
	/* Uart IO config */
	BSP_GpioConfig(USART1_TX_GPIO_PORT,USART1_TX_PIN,GPIO_MODE_AF_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);
	BSP_GpioConfig(USART1_RX_GPIO_PORT,USART1_RX_PIN,GPIO_MODE_AF_INPUT,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);

	BSP_GpioConfig(USART2_TX_GPIO_PORT,USART2_TX_PIN,GPIO_MODE_AF_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);
	BSP_GpioConfig(USART2_RX_GPIO_PORT,USART2_RX_PIN,GPIO_MODE_AF_INPUT,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);

	BSP_GpioConfig(USART3_TX_GPIO_PORT,USART3_TX_PIN,GPIO_MODE_AF_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);
	BSP_GpioConfig(USART3_RX_GPIO_PORT,USART3_RX_PIN,GPIO_MODE_AF_INPUT,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);

	BSP_GpioConfig(UART4_TX_GPIO_PORT,UART4_TX_PIN,GPIO_MODE_AF_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);
	BSP_GpioConfig(UART4_RX_GPIO_PORT,UART4_RX_PIN,GPIO_MODE_AF_INPUT,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);

	BSP_GpioConfig(UART5_TX_GPIO_PORT,UART5_TX_PIN,GPIO_MODE_AF_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);
	BSP_GpioConfig(UART5_RX_GPIO_PORT,UART5_RX_PIN,GPIO_MODE_AF_INPUT,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);
}

/**
  * @brief  Initializes the MSP.
  * @retval None
  */
void BSP_HardwareInit(void)
{
  	/* Enable GPIO clocks */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_AFIO_CLK_ENABLE();

	/* IOMUX init */
	BSP_GpioInit();
}


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
