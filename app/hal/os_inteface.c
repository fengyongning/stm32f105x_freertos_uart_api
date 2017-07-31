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
 *   os_interface.c
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
#include "os_interface.h"


/***********************************************************************
 * MACRO CONSTANT DEFINITIONS
************************************************************************/

/***********************************************************************
 * VARIABLES DEFINITIONS
************************************************************************/


/***********************************************************************
 * FUNCTIONS DEFINITIONS
************************************************************************/
bool get_event_from_mail(osMailQId mailID, osEvent_st *event)
{
    osEvent OSEvent;

    OSEvent = osMailGet(mailID, portMAX_DELAY);
    if(OSEvent.status == osEventMail)
    {
        memcpy(event, OSEvent.value.p, sizeof(osEvent_st));

        return TRUE;
    }
    else
    {
        event->event = OS_EVENT_NULL;
        return FALSE;
    }
}

bool set_event_to_mail(osMailQId mailID, osEvent_st *event)
{
    osEvent_st *osEvent = NULL;

    osEvent = osMailAlloc(mailID, 0);
    if(osEvent)
    {
        osEvent->event = event->event;
        osEvent->data = event->data;

        osMailPut(mailID, osEvent);
        osMailFree(mailID, osEvent);

        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

#ifdef MODEM_MODULE_ENABLED 
extern osMailQId ModemUserMailQ;
bool send_data_to_modem(osModemReq_st *dat)
{
    osEvent_st event;
	bool ret;

    event.event = OS_EVENT_MDM_REQUEST;
    event.data.mdm_req.cmd = dat->cmd;
    if(event.data.mdm_req.cmd == MDM_GPRS)
    {
        event.data.mdm_req.config.socket.mode = dat->config.socket.mode;
        memcpy(event.data.mdm_req.config.socket.apn_name, dat->config.socket.apn_name, MODEM_APN_NAME_LEN);
		memcpy(event.data.mdm_req.config.socket.domain, dat->config.socket.domain, MODEM_DOMAIN_LEN);
		memcpy(event.data.mdm_req.config.socket.ip, dat->config.socket.ip, MODEM_IP_ADDR_LEN);
		event.data.mdm_req.config.socket.port = dat->config.socket.port;
        event.data.mdm_req.config.socket.keep_link = dat->config.socket.keep_link;
		event.data.mdm_req.config.socket.need_resp = dat->config.socket.need_resp;
    }
    else if(event.data.mdm_req.cmd == MDM_SMS)
    {
    	event.data.mdm_req.config.sms.type = dat->config.sms.type;
		event.data.mdm_req.config.sms.utc_time = dat->config.sms.utc_time;
        memcpy(event.data.mdm_req.config.sms.phone_no, dat->config.sms.phone_no, MODEM_PHONE_NUM_LEN);
		memcpy(event.data.mdm_req.config.sms.content, dat->config.sms.content, MODEM_SMS_CONTENT_LEN);
    }
    else if(event.data.mdm_req.cmd == MDM_DIAL)
    {
        memcpy(event.data.mdm_req.config.dial.phone_no, dat->config.dial.phone_no, MODEM_PHONE_NUM_LEN);
    }
	event.data.mdm_req.dat_len = dat->dat_len;
	event.data.mdm_req.data_p = dat->data_p;
	event.data.mdm_req.result_cb = dat->result_cb;

    ret = set_event_to_mail(ModemUserMailQ, &event);

	return ret;
}
#endif

#ifdef BLE_MODULE_ENABLED
extern osMailQId BleMailQ;
bool send_data_to_ble(osBleReq_st *req)
{
    osEvent_st event;
	bool ret;
    
    event.event = OS_EVENT_BLE_DATA_PUSH;
	event.data.ble_req.frame_ctrl = req->frame_ctrl;
	event.data.ble_req.data_p = req->data_p;
    event.data.ble_req.dat_len = req->dat_len;
	event.data.ble_req.result_cb = req->result_cb;
    
    ret = set_event_to_mail(BleMailQ, &event);

	return ret;
}
#endif

#ifdef OBD_MODULE_ENABLED
extern osMailQId ObdMailQ;
bool send_data_to_obd(osObdReq_st *req)
{
    osEvent_st event;
	bool ret;
    
    event.event = OS_EVENT_OBD;
    event.data.obd_req.cmd = req->cmd;
	event.data.obd_req.data_p = req->data_p;
    event.data.obd_req.dat_len = req->dat_len;
    
    ret = set_event_to_mail(ObdMailQ, &event);

	return ret;
}
#endif

extern osThreadId MainTaskHandle;
bool send_signal_to_main(s32 signal)
{
    s32 ret;

    ret = osSignalSet(MainTaskHandle, signal);
    if(ret == osOK)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}


