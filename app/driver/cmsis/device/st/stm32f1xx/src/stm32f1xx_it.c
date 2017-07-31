/**
  ******************************************************************************
  * @file    Templates/Src/stm32f1xx.c
  * @author  MCD Application Team
  * @version V1.3.0
  * @date    18-December-2015
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
#include "stm32f1xx_it.h"
#include "ph_common.h"
#ifdef HAL_SPI_MODULE_ENABLED
#include "flash_ext.h"
#endif

/** @addtogroup STM32F1xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  __ASM("BKPT #01");
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	osSystickHandler();
	
  	HAL_IncTick();
}

/******************************************************************************/
/*                 STM32F1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f1xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles Uart1 Handler.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
	ph_uart_irq_handle(PH_UART_PORT1);
}

/**
  * @brief  This function handles Uart2 Handler.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
	ph_uart_irq_handle(PH_UART_PORT2);
}

/**
  * @brief  This function handles Uart3 Handler.
  * @param  None
  * @retval None
  */
void USART3_IRQHandler(void)
{
	ph_uart_irq_handle(PH_UART_PORT3);
}

/**
  * @brief  This function handles Uart4 Handler.
  * @param  None
  * @retval None
  */
void UART4_IRQHandler(void)
{
	ph_uart_irq_handle(PH_UART_PORT4);
}

/**
  * @brief  This function handles Uart4 Handler.
  * @param  None
  * @retval None
  */
void UART5_IRQHandler(void)
{
	ph_uart_irq_handle(PH_UART_PORT5);
}

#ifdef BLE_MODULE_ENABLED 
void EXTI2_IRQHandler(void)
{
	//BleLinkHandle();
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}
#endif

#ifdef HAL_TIM_MODULE_ENABLED
void TIM1_UP_IRQHandler(void)
{
	ph_timer_irq_handle(TIMER_ID_1);
}

void TIM2_IRQHandler(void)
{
	ph_timer_irq_handle(TIMER_ID_2);
}

void TIM3_IRQHandler(void)
{
	ph_timer_irq_handle(TIMER_ID_3);
}

void TIM4_IRQHandler(void)
{
	ph_timer_irq_handle(TIMER_ID_4);
}

void TIM5_IRQHandler(void)
{
	ph_timer_irq_handle(TIMER_ID_5);
}

void TIM6_IRQHandler(void)
{
	ph_timer_irq_handle(TIMER_ID_6);
}

void TIM7_IRQHandler(void)
{
	ph_timer_irq_handle(TIMER_ID_7);
}
#endif

#ifdef HAL_SPI_MODULE_ENABLED
void SPI1_IRQHandler(void)
{
	Flash_SpiInterrupt();
}
#endif
/*
void CAN1_RX0_IRQHandler( void )
{
    ph_can_rx_handle();
}
*/
/**
  * @}
  */ 

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
