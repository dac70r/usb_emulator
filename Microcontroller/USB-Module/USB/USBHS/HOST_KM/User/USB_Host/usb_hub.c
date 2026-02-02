/********************************** (C) COPYRIGHT *******************************
* File Name          : usb_hub.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : This file provides all the USB firmware functions.
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "usb_hub.h"

Link_HUBSaveData* USBHS_Hub_LinkHead;//usbhs port link head
Link_HUBSaveData* USBFS_Hub_LinkHead;//usbfs port link head

USB_HUB_Info USBHS_HubInfo[8];
USB_HUB_Info USBFS_HubInfo[8];
/*******************************************************************************
 * @fn        Hublink_ModifyHubData
 *
 * @briefI    Modify HUB linked list elements
 *
 * @param     p - Original linked list
 *            oldhubdata - Old Elements
 *            newhubdata - New Elements
 *
 * @return    None
 */
UINT8 Hublink_ModifyHubData(Link_HUBSaveData* p, USB_HUB_SaveData oldhubdata, USB_HUB_SaveData newhubdata)
{
    p = p->next;
    while (p)
    {
        if (p->HUB_SaveData.Depth == oldhubdata.Depth
            && p->HUB_SaveData.UpLevelPort == oldhubdata.UpLevelPort
            && p->HUB_SaveData.CurrentPort == oldhubdata.CurrentPort
            && p->HUB_SaveData.HUB_Info.DevAddr == oldhubdata.HUB_Info.DevAddr)
        {
            p->HUB_SaveData = newhubdata;
            return 0;//success
        }
        p = p->next;
    }
    return 1;//fail
}

/*******************************************************************************
 * @fn        Hublink_InitHubLink
 *
 * @briefI    Hublink_SearchHubData
 *
 * @param     None
 *
 * @return    head node
 */
Link_HUBSaveData* Hublink_InitHubLink( void )
{
    Link_HUBSaveData* p = NULL;
    Link_HUBSaveData* temp = (Link_HUBSaveData*)malloc(sizeof(Link_HUBSaveData));
	
	// Initializes all Link_HUBSaveData fields to be 0
    memset(&temp->HUB_SaveData, 0, sizeof(USB_HUB_SaveData)); 
	
    temp->HUB_SaveData.Depth = 0xff;
    temp->HUB_SaveData.CurrentPort = 0xff;
    temp->HUB_SaveData.UpLevelPort = 0xff;

    temp->next = NULL;
    p = temp;

    return p;
}

/*******************************************************************************
 * @fn        Hublink_SearchHubData
 *
 * @briefI    Find the corresponding node
 *
 * @param     p - head node
 *            HUB_SaveData - Node to be searched
 *
 * @return    0 - success 1 - fail
 */
UINT8 Hublink_SearchHubData(Link_HUBSaveData* p, USB_HUB_SaveData *HUB_SaveData)
{
    p = p->next;
    while (p)
    {
        if (p->HUB_SaveData.Depth == (*HUB_SaveData).Depth
            && p->HUB_SaveData.UpLevelPort == (*HUB_SaveData).UpLevelPort
            && p->HUB_SaveData.CurrentPort == (*HUB_SaveData).CurrentPort
            && p->HUB_SaveData.HUB_Info.DevAddr == HUB_SaveData->HUB_Info.DevAddr)
        {
            *HUB_SaveData = p->HUB_SaveData;
            return 0;
        }
        p = p->next;
    }
    return 1;//返回1，表示未找到
}

/*******************************************************************************
 * @fn        Hublink_InsertHubData
 *
 * @briefI    Insert Node
 *
 * @param     p - head node
 *            HUB_SaveData - Node to be insert
 *
 * @return    0 - success 1 - fail
 */
UINT8 Hublink_InsertHubData(Link_HUBSaveData* p, USB_HUB_SaveData HUB_SaveData)
{
    Link_HUBSaveData* c = NULL;
    Link_HUBSaveData* temp = p;

    if(Hublink_SearchHubData(p,&HUB_SaveData) == 0)
    {
        return 1;
    }

    while( temp->next )
    {
        temp = temp->next;
    }

    c = (Link_HUBSaveData*)malloc(sizeof(Link_HUBSaveData));
    c->HUB_SaveData = HUB_SaveData;
    c->next = temp->next;
    temp->next = c;

    return 0;
}

/*******************************************************************************
 * @fn        Hublink_DeleteHubData
 *
 * @briefI    Delete Node
 *
 * @param     p - head node
 *            HUB_SaveData - Node to be delete
 *
 * @return    0 - success 1 - fail
 */
UINT8 Hublink_DeleteHubData(Link_HUBSaveData* p, USB_HUB_SaveData HUB_SaveData)
{
    Link_HUBSaveData* del = NULL, *temp = p;
    UINT8 find = 0;
    while (temp->next)
    {
        if (temp->next->HUB_SaveData.Depth == HUB_SaveData.Depth
            && temp->next->HUB_SaveData.UpLevelPort == HUB_SaveData.UpLevelPort
            && temp->next->HUB_SaveData.CurrentPort == HUB_SaveData.CurrentPort
            && temp->next->HUB_SaveData.HUB_Info.DevAddr == HUB_SaveData.HUB_Info.DevAddr)
        {
            find = 1;
            break;
        }
        temp = temp->next;
    }
    if (find == 0)
    {
        return 1;
    }
    else
    {
        del = temp->next;
        temp->next = temp->next->next;
        free(del);
        return 0;
    }
}

/*******************************************************************************
 * @fn        Hublink_judgingstructures
 *
 * @briefI    Determine and delete sub devices under the device
 *
 * @param     hubdata - Variables used for judgment
 *
 * @return    None
 */
UINT8  Hublink_judgingstructures( USB_HUB_SaveData hubdata )
{
    UINT8 ret = 0;
    UINT8 del_flg = 0;
    Link_HUBSaveData *deletehub = NULL;
    deletehub = USBHS_Hub_LinkHead;

    while(deletehub != NULL)
    {
        if( deletehub->HUB_SaveData.Depth-1 == hubdata.Depth )
        {
                if( deletehub->HUB_SaveData.HUB_Info.DevAddr == hubdata.HUB_Info.portD[hubdata.CurrentPort-1].Addr )
                {
                    if( deletehub->HUB_SaveData.HUB_Info.portD[deletehub->HUB_SaveData.CurrentPort-1].DeviceType == 0x09 )
                    {
                        Hublink_judgingstructures(deletehub->HUB_SaveData);
                        del_flg = 1;
                        break;
                    }
                    else
                    {
                        del_flg = 1;
                        break;
                    }
                }
        }
        deletehub = deletehub->next;
    }

    if( del_flg == 1 )
    {
        USB_DelUSBHS_AddressNumber(deletehub->HUB_SaveData.HUB_Info.portD[deletehub->HUB_SaveData.CurrentPort-1].Addr);
        ret = Hublink_DeleteHubData(USBHS_Hub_LinkHead,deletehub->HUB_SaveData);//Delete linked list nodes
        if( ret == 0 )
        {
        }
        return 0;
    }
    else
    {
        return 1;
    }

}

/*******************************************************************************
 * @fn        Hublink_finesubstructures
 *
 * @briefI    Used to query whether there are sub devices under this variable
 *
 * @param     hubdata - Variables used for judgment
 *
 * @return    None
 */
void Hublink_finesubstructures( USB_HUB_SaveData hubdata )
{
    while( 1 )
    {
        if(Hublink_judgingstructures(hubdata) == 0)//If there are devices, proceed to the next judgment until there are no devices available
        {
            continue;
        }
        else
        {
            break;
        }
    }
}
