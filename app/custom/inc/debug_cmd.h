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
 *   debug_cmd.h
 *
 * Project:
 * --------
 *   OpenCPU
 *
 * Description:
 * ------------
 *   error code  defines.
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
#ifndef __DEBUG_CMD_H__
#define __DEBUG_CMD_H__

#include "ph_common.h"


/* Private macro -------------------------------------------------------------*/
	
/* Private typedef -----------------------------------------------------------*/



/* Export Function -----------------------------------------------------------*/
bool DebugCmdSetup(void);
u16 DebugCmdSend(u8 *msg, u16 size);









#endif 
