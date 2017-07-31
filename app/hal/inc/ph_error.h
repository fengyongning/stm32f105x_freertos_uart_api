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
 *   ph_error.h
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

#ifndef __PH_ERROR_H__
#define __PH_ERROR_H__

/****************************************************************************
 * Error Code Definition
 ***************************************************************************/
typedef enum
{
    PH_RET_OK           						= 0,
	PH_RET_ERR_GENERAL		 					= -1,
	PH_RET_ERR_FATAL 							= -2,
    PH_RET_ERR_INVALID_PARAMETER 				= -3,
	PH_RET_ERR_INVALID_OP 						= -4,
	PH_RET_ERR_NOMATCHVERSION        			= -5,
	PH_RET_ERR_INVALIDFLASHID      				= -6,
	PH_RET_ERR_INVALID_PORT 					= -7,
	PH_RET_ERR_INVALID_BAUDRATE 				= -8,
	PH_RET_ERR_PORT_NOT_OPENED 					= -9,
	PH_RET_ERR_PORT_ALREADY_OPENED 				= -10,
	PH_RET_ERR_PORT_BUSY						= -11,
	PH_RET_ERR_API_NO_RESPONSE 					= -12,
    PH_RET_ERR_API_INVALID_RESPONSE 			= -13,
    PH_RET_ERR_INVALID_TASK_ID 					= -14,
    PH_RET_ERR_MALLOC_MEM 						= -15,
    PH_RET_ERR_CRCERROR							= -16,
} PH_RET_RESULT;

#endif // End-of __PH_ERROR_H__ 

