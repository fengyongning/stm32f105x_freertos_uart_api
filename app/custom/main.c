/**
  ******************************************************************************
  * @file    Templates/Src/main.c
  * @author  MCD Application Team
  * @version V1.3.0
  * @date    18-December-2015
  * @brief   Main program body
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
#include <stdio.h>
#include "CustomBspConfig.h"
#include "os_interface.h"

/** @addtogroup STM32F1xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */
/* Private macro -------------------------------------------------------------*/
#define MAIN_TASK                       1

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Extern function prototypes ------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void Main_task( void const* argument );

/* Private variables ---------------------------------------------------------*/
osThreadId MainTaskHandle;


/*
 * Starts all the other tasks, then starts the scheduler.
 */
int main( void )
{
    /* STM32F1xx HAL library initialization:
         - Configure the Flash prefetch
         - Set NVIC Group Priority to 4
       */
    HAL_Init();
    /* Configure the system clock to 72 MHz */
    BSP_SysClockConfig();
    /* Initial Board peripheral */
    BSP_HardwareInit();
	
    /*UART命令测试口配置*/
    DebugCmdSetup();

    /*  Thread  definition */
    osThreadDef( MAIN_TASK, Main_task, osPriorityNormal, 0, 2 * configMINIMAL_STACK_SIZE );
    MainTaskHandle = osThreadCreate( osThread( MAIN_TASK ), NULL );
	
    /* Start scheduler */
    osKernelStart();
    /* We should never get here as control is now taken by the scheduler */
    for( ;; );
}


/**
  * @brief  test
  */
void Main_task( void const* argument )
{
    ( void ) argument;
	
    while( 1 )
    {
        /* Suspend Thread */
        osThreadSuspend(NULL);   // Task什么都不干
    }
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
