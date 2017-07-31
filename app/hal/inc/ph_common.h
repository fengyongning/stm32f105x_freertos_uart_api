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
 *   ph_common.h
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
#ifndef __PH_COMMON_H__
#define __PH_COMMON_H__

#include "CustomBspConfig.h"
#include "ph_type.h"
#include "ph_error.h"
#include "ph_uart.h"

/***********************************************************************
 * MACRO CONSTANT DEFINITIONS
************************************************************************/


/***********************************************************************
 * VARIABLES DEFINITIONS
************************************************************************/


/***********************************************************************
 * FUNCTIONS DEFINITIONS
************************************************************************/
void ph_irq_priority_set(IRQn_Type IRQn, u32 priority);
void ph_nvic_irq_enable(IRQn_Type IRQn);
void ph_nvic_irq_disable(IRQn_Type IRQn);
void ph_dma1_clock_enable(void);
void ph_dma1_clock_disable(void);
void ph_dma2_clock_enable(void);
void ph_dma2_clock_disable(void);


#endif   // End-of __PH_COMMON_H__ 

