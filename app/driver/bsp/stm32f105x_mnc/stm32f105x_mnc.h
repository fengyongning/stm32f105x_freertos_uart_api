/**
  ******************************************************************************
  * @file    stm32f105x_mnc.h
  * @author  MCD Application Team
  * @version V1.0.3
  * @date    29-April-2016
  * @brief   This file contains definitions for:
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F105X_MNC_H
#define __STM32F105X_MNC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <time.h>   
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"


/**
  * @}
  */
/************************************************************************
 * GPIO
 ************************************************************************/
/* UART port */
#define USART1_TX_PIN                    		GPIO_PIN_9
#define USART1_TX_GPIO_PORT              		GPIOA
#define USART1_RX_PIN                    		GPIO_PIN_10
#define USART1_RX_GPIO_PORT              		GPIOA

#define USART2_TX_PIN                    		GPIO_PIN_2
#define USART2_TX_GPIO_PORT              		GPIOA
#define USART2_RX_PIN                    		GPIO_PIN_3
#define USART2_RX_GPIO_PORT              		GPIOA

#define USART3_TX_PIN                    		GPIO_PIN_10
#define USART3_TX_GPIO_PORT              		GPIOB
#define USART3_RX_PIN                    		GPIO_PIN_11
#define USART3_RX_GPIO_PORT              		GPIOB

#define UART4_TX_PIN                     		GPIO_PIN_10
#define UART4_TX_GPIO_PORT               		GPIOC
#define UART4_RX_PIN                     		GPIO_PIN_11
#define UART4_RX_GPIO_PORT               		GPIOC

#define UART5_TX_PIN                     		GPIO_PIN_12
#define UART5_TX_GPIO_PORT               		GPIOC
#define UART5_RX_PIN                     		GPIO_PIN_2
#define UART5_RX_GPIO_PORT               		GPIOD

/** @
  * @{
  */
extern uint32_t AHB_Clock;
extern uint32_t APB1_Clock;
extern uint32_t APB2_Clock;
/** @
  * @{
  */
void BSP_SysClockConfig(void);  
void BSP_HardwareInit(void);  
void BSP_GpioConfig(GPIO_TypeDef *port,uint32_t pin,uint32_t mode,uint32_t pull,uint32_t speed);
void BSP_GpioSet(GPIO_TypeDef *port,uint16_t pin,GPIO_PinState state);
GPIO_PinState BSP_GpioGet(GPIO_TypeDef *port,uint16_t pin);


#ifdef __cplusplus
}
#endif

#endif /* __STM32F105X_MNC_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
