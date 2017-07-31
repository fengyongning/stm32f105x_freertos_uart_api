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
 *   ph_common.c
 *
 * Project:
 * --------
 *
 *
 * Description:
 * ------------
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
#include "ph_common.h"


/***********************************************************************
 * MACRO CONSTANT DEFINITIONS
************************************************************************/

/***********************************************************************
 * VARIABLES DEFINITIONS
************************************************************************/
static u8 dma1_clk_op_cnt = 0;
static u8 dma2_clk_op_cnt = 0;


/***********************************************************************
 * FUNCTIONS DEFINITIONS
************************************************************************/

/*****************************************************************
* Function:     ph_irq_priority_set
*
* Description:
*               This function set irq priority at priority group 4
* Return:
*               none
*
*****************************************************************/
void ph_irq_priority_set(IRQn_Type IRQn, u32 priority)
{
	u32 prioritygroup = 0x00;
	
	prioritygroup = NVIC_GetPriorityGrouping();
  	NVIC_SetPriority(IRQn, NVIC_EncodePriority(prioritygroup, priority, 0));
}

void ph_dma1_clock_enable(void)
{
	++dma1_clk_op_cnt;

	if(__HAL_RCC_DMA1_IS_CLK_DISABLED())
	{
		__HAL_RCC_DMA1_CLK_ENABLE();
	}
}

void ph_dma1_clock_disable(void)
{
	if((dma1_clk_op_cnt == 0)||((--dma1_clk_op_cnt) == 0))
	{
		__HAL_RCC_DMA1_CLK_DISABLE();
	}
}

void ph_dma2_clock_enable(void)
{
	++dma2_clk_op_cnt;

	if(__HAL_RCC_DMA2_IS_CLK_DISABLED())
	{
		__HAL_RCC_DMA2_CLK_ENABLE();
	}
}

void ph_dma2_clock_disable(void)
{
	if((dma2_clk_op_cnt == 0)||((--dma2_clk_op_cnt) == 0))
	{
		__HAL_RCC_DMA2_CLK_DISABLE();
	}
}

/*****************************************************************
* Function:     ph_irq_enable
*
* Description:
*               This function set irq priority at priority group 4
* Return:
*               none
*
*****************************************************************/
void ph_nvic_irq_enable(IRQn_Type IRQn)
{
	NVIC_EnableIRQ(IRQn);
}

/*****************************************************************
* Function:     ph_irq_disable
*
* Description:
*               This function set irq priority at priority group 4
* Return:
*               none
*
*****************************************************************/
void ph_nvic_irq_disable(IRQn_Type IRQn)
{
	NVIC_DisableIRQ(IRQn);
}






