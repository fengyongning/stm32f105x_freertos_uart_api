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
 *   os_interface.h
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
#ifndef __OS_INTERFACE_H__
#define __OS_INTERFACE_H__

#include "ph_common.h"

/* Private macro -------------------------------------------------------------*/
#define	 USER_MSG_MAX_SIZE				64
#define	 MODEM_TX_DAT_MAX_SIZE			64

#define	 MODEM_IP_ADDR_LEN				24
#define	 MODEM_DOMAIN_LEN				32
#define	 MODEM_APN_NAME_LEN				16
#define	 MODEM_PHONE_NUM_LEN			16
#define	 MODEM_SMS_CONTENT_LEN			128

/* Private macro -------------------------------------------------------------*/
#define	 SIG_BLE_STA_Pos					(0)
#define	 SIG_BLE_STA_Mask					(0x3 << SIG_BLE_STA_Pos)
#define	 SIG_BLE_CMD_Pos					(2)
#define	 SIG_BLE_CMD_Mask					(0x3f << SIG_BLE_CMD_Pos)
#define	 SIG_OBD_STA_Pos					(8)
#define	 SIG_OBD_STA_Mask					(0x7 << SIG_BLE_STA_Pos)

#define	 SIG_MDM_COM_RET_Pos				(0)
#define	 SIG_MDM_COM_RET_Mask				(0xf << SIG_MDM_COM_RET_Pos)
#define	 SIG_MDM_COM_STA_Pos				(4)
#define	 SIG_MDM_COM_STA_Mask				(0xff << SIG_MDM_COM_STA_Pos)
#define	 SIG_MDM_COM_CMD_Pos				(12)
#define	 SIG_MDM_COM_CMD_Mask				(0xf << SIG_MDM_COM_CMD_Pos)
	
/* Private typedef -----------------------------------------------------------*/
typedef enum
{
	SIG_BLE_STA_CONNECTED             = (0x01 << SIG_BLE_STA_Pos),     //0x01
	SIG_BLE_STA_DISCONNECTED          = (0x02 << SIG_BLE_STA_Pos),     //0x02
	
	SIG_BLE_CMD_FIRM_UPDATE           = (0x01 << SIG_BLE_CMD_Pos),     //0x04
	SIG_BLE_CMD_GET_DEV_INFO          = (0x02 << SIG_BLE_CMD_Pos),     //0x08
	SIG_BLE_CMD_GET_OBD_INFO          = (0x03 << SIG_BLE_CMD_Pos),     //0x0C
	SIG_BLE_CMD_SET_OBD_PARAM         = (0x04 << SIG_BLE_CMD_Pos),     //0x10
	SIG_BLE_CMD_SET_OBD_MODE          = (0x05 << SIG_BLE_CMD_Pos),     //0x14
	SIG_BLE_CMD_GET_OBD_PARAM         = (0x06 << SIG_BLE_CMD_Pos),     //0x18
	
	SIG_OBD_STA_FIRE				  = (0x01 << SIG_OBD_STA_Pos),
	SIG_OBD_STA_MISFIRE				  = (0x02 << SIG_OBD_STA_Pos),
	SIG_OBD_STA_TROUBLE				  = (0x03 << SIG_OBD_STA_Pos),
} MainSignalType_en;

typedef enum
{
	SIG_COM_RET_NONE				  = (0x0 << SIG_MDM_COM_RET_Pos),
	SIG_COM_RET_OK				  	  = (0x1 << SIG_MDM_COM_RET_Pos),
	SIG_COM_RET_ERR				  	  = (0x2 << SIG_MDM_COM_RET_Pos),

	SIG_COM_STA_SIM				  	  = (0x1 << SIG_MDM_COM_STA_Pos),
	SIG_COM_STA_REGISTER			  = (0x2 << SIG_MDM_COM_STA_Pos),
	SIG_COM_STA_ATTACH			      = (0x3 << SIG_MDM_COM_STA_Pos),
	SIG_COM_STA_SIGNAL			      = (0x4 << SIG_MDM_COM_STA_Pos),
	SIG_COM_STA_APN_CFG			      = (0x5 << SIG_MDM_COM_STA_Pos),
	SIG_COM_STA_GPRS_ACTIVE			  = (0x6 << SIG_MDM_COM_STA_Pos),
	SIG_COM_STA_IPADDR			      = (0x7 << SIG_MDM_COM_STA_Pos),
	SIG_COM_STA_CONNECT			      = (0x8 << SIG_MDM_COM_STA_Pos),
	SIG_COM_STA_SEND			      = (0x9 << SIG_MDM_COM_STA_Pos),
	SIG_COM_STA_RECV			      = (0xa << SIG_MDM_COM_STA_Pos),
	SIG_COM_STA_CLOSE			      = (0xb << SIG_MDM_COM_STA_Pos),
	SIG_COM_STA_SHUT			      = (0xc << SIG_MDM_COM_STA_Pos),
	SIG_COM_STA_RECV_WAIT			  = (0xd << SIG_MDM_COM_STA_Pos),
	SIG_COM_SMS_SEND				  = (0xe << SIG_MDM_COM_STA_Pos), 

	SIG_COM_CMD_START				  = (0x1 << SIG_MDM_COM_CMD_Pos),
} MdmComSignalType_en;

/***********************************************************************
 * TYPE DEFINITIONS
************************************************************************/
typedef enum
{
	OS_EVENT_NULL = 0,
	OS_EVENT_TIMER, 					/* timer time out*/
	OS_EVENT_KEY, 						/* KEY*/
	OS_EVENT_INT, 						/* GPIO interrupt*/
    OS_EVENT_MDM_REQUEST,
    OS_EVENT_ADC, 
    OS_EVENT_USER_MSG,
    OS_EVENT_BLE_DATA_PUSH,
    OS_EVENT_OBD,
    OS_EVENT_NUM,
} osEvent_enum;

typedef enum
{
	CMD_OBD_SET_MODE,
	CMD_OBD_GET_INFO,
	CMD_OBD_SET_PARAM,
	CMD_OBD_GET_RTD,
} obd_cmd_type_en;

typedef enum
{
	MDM_GPRS,
	MDM_SMS,
	MDM_DIAL,
	MDM_GET_CELL,
} mdm_cmd_type_en;

typedef enum
{
	SOCKET_TCP,
	SOCKET_UDP
} mdm_socket_mode_en;

typedef enum
{
	SMS_SEND,
	SMS_RECEIVE,
} mdm_sms_type_en;

typedef struct
{
	mdm_socket_mode_en mode;		//TCP,UDP
	u8 apn_name[MODEM_APN_NAME_LEN];
	u8 domain[MODEM_DOMAIN_LEN];
	u8 ip[MODEM_IP_ADDR_LEN];
	u16 port;
	u8 keep_link;
	u8 need_resp;
} mdm_socket_st;

typedef struct
{
    mdm_sms_type_en type;           //send or receive
	u8 phone_no[MODEM_PHONE_NUM_LEN];
	u8 content[MODEM_SMS_CONTENT_LEN];
	u32 utc_time;
} mdm_sms_st;

typedef struct
{
	u8 phone_no[MODEM_PHONE_NUM_LEN];
} mdm_dial_st;

typedef void(*mdm_result_func)(u8 *resp,u16 len);		//回调函数，用于通知调用方modem执行结果

typedef union
{
	mdm_socket_st 		socket;
	mdm_sms_st 			sms;
	mdm_dial_st 		dial;
} mdm_config_un;

typedef struct 
{
	mdm_cmd_type_en cmd;		/*命令类型:短信,电话,网络*/
	mdm_config_un config;	
    u16 dat_len;				/*发送数据长度*/
    void *data_p;				/*发送数据指针*/
	mdm_result_func result_cb;	/*调用方指定的结果回调函数*/
} osModemReq_st;

typedef void(*ble_result_func)(u8 result,u8 *resp,u16 len);		//回调函数，用于通知调用方BLE执行结果
typedef struct 
{
    u8 frame_ctrl;				/*帧控制字段*/
	u16 dat_len;                /*发送数据长度*/
    void *data_p;				/*发送数据指针*/
	ble_result_func result_cb;	/*调用方指定的结果回调函数*/
} osBleReq_st;

typedef struct
{
	obd_cmd_type_en cmd;
	u8 dat_len;
	void *data_p;
} osObdReq_st;

typedef struct {
    osThreadId src;
    bool use_point;
    u8 len;
    u8 data[USER_MSG_MAX_SIZE];
    const void * data_p;
} osUserMsg_st;

typedef union {
	PhUart_st uart;
	osUserMsg_st user_msg;
	osModemReq_st  mdm_req;
	osBleReq_st   ble_req;
	osObdReq_st   obd_req;
} osEventData_union;

/* EVENT data */
typedef struct
{
    osEvent_enum event;
    osEventData_union data;
} osEvent_st; 

/****************************************************************************
 * Function export
 ***************************************************************************/
bool get_event_from_mail(osMailQId mailID, osEvent_st *event);
bool set_event_to_mail(osMailQId mailID, osEvent_st *event);
bool send_data_to_modem(osModemReq_st *dat);
bool send_data_to_ble(osBleReq_st *req);
bool send_data_to_obd(osObdReq_st *req);
bool send_signal_to_main(s32 signal);


#endif // End-of __PH_ERROR_H__ 

