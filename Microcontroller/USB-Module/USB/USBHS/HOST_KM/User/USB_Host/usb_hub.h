
#ifndef __HSHUB_H
#define __HSHUB_H

#include "ch32f20x_usbfs_host.h"
#include "ch32f20x_usbhs_host.h"
#include "stdlib.h"


#define    HUB_ERR_SCUESS       0x02
#define    HUB_ERR_CONNECT      0x01
#define    HUB_ERR_DISCONNECT   0x00
#define    HUB_CS_NAK           0x2a
#define    DEVICE_ADDR          0x08                                        /* U30 device address*/

 typedef struct  __attribute__((packed)) _HUB_Endp_Info
 {
     UINT8 Num;          // 端点号
     UINT8 EndpType;     // 端点类型：1-IOS, 2-BULK, 3-INT
     UINT8 tog;
     UINT8 HighTransNum; // 一个微帧内事务数量(<=3)
     UINT16 Endp_Size;   // 端点大小
 } HUB_Endp_Info, *PHUB_Enpd_Info;


 typedef struct  __attribute__((packed)) _HUB_Port_Info
 {
     UINT8 Status;
     UINT8 Speed;        // [8]-S, 0-lowspeed, 1-fullspeed;
     UINT8 Num;          // 端口号
     UINT8 Addr;         //HUB下面设备地址
     UINT8 DeviceType;   //设备类型
     UINT8 DeviceHIDType;//HID类型
     UINT8 EndpNum;      //端点数量
     UINT8 PortpChangeField;//Port state change
     HUB_Endp_Info portEndp[8];          //8个端点
 } HUB_Port_Info, *PHUB_Port_Info;

 typedef struct  __attribute__((packed)) _USB_HUB_Info
 {
     UINT8 Status;      // 0-disconnect  1-connect  2-config
     UINT8 Speed;       // 0-fullspeed, 1-highspeed, 2-lowspeed
     UINT8 IntEndpnum;
     UINT8 DevAddr;
     UINT8 EndpSize;
     UINT8 Numofport;
     HUB_Port_Info portD[4];
 } USB_HUB_Info, *PUSB_HUB_Info;


 typedef struct  __attribute__((packed)) _USB_HUB_SaveData
 {

     UINT8 Depth;
     UINT8 UpLevelPort;
     UINT8 CurrentPort;
     USB_HUB_Info HUB_Info;

 } USB_HUB_SaveData;


 //HUB数据链表中节点的结构
 typedef struct __attribute__((packed)) _USBHS_Link_HUBSaveData
 {
     USB_HUB_SaveData HUB_SaveData;
     struct _USBHS_Link_HUBSaveData* next;
 }Link_HUBSaveData;

Link_HUBSaveData* Hublink_InitHubLink(void);
UINT8  USBHS_HUB_Process( u8 depth,u8 addr,u8 uplevelport );
UINT8 USB_SetUSBHS_AddressNumber(void);
UINT8 USB_DelUSBHS_AddressNumber(UINT8 addr);

extern Link_HUBSaveData* USBHS_Hub_LinkHead;
extern Link_HUBSaveData* USBFS_Hub_LinkHead;

extern USB_HUB_Info USBFS_HubInfo[8];
extern UINT8 USBFS_glable_index ;

extern __attribute__ ((aligned(4)))  UINT8 USBHS_Test_Buf[ 3072 ]  ;    //高速高带宽端点一个微帧内最大可以有3次事务数 1024*3=3072


UINT8  USBFS_HUB_Process( UINT8 depth ,UINT8 addr,UINT8 uplevelport );


UINT8 Hublink_ModifyHubData(Link_HUBSaveData* p, USB_HUB_SaveData oldhubdata, USB_HUB_SaveData newhubdata);
Link_HUBSaveData* Hublink_InitHubLink( void );
UINT8 Hublink_SearchHubData(Link_HUBSaveData* p, USB_HUB_SaveData *HUB_SaveData);
UINT8 Hublink_InsertHubData(Link_HUBSaveData* p, USB_HUB_SaveData HUB_SaveData);
UINT8 Hublink_DeleteHubData(Link_HUBSaveData* p, USB_HUB_SaveData HUB_SaveData);
void Hublink_finesubstructures( USB_HUB_SaveData hubdata );

extern USBDEV_INFO  USBHS_thisUsbDev;
extern USB_HUB_Info USBHS_HubInfo[8];
extern uint8_t USBHS_SSPLITPacket( PHUB_Port_Info portn, uint8_t *pbuf, uint16_t *plen, uint8_t endp_pid, uint8_t tog ,UINT8 hubaddr);
extern uint8_t USBHS_CSPLITPacket( PHUB_Port_Info portn, uint8_t *pbuf, uint16_t *plen, uint8_t endp_pid, uint8_t tog ,UINT8 hubaddr);

extern UINT8 USBHS_BULK_Send_Data( PHUB_Port_Info phub,UINT8 num_port,UINT8 *pdata,UINT16 *len,UINT8 endp,UINT8 *tog ,UINT8 hubaddr);
extern UINT8 USBHS_BULK_In_Data( PHUB_Port_Info phub,UINT8 num_port,UINT8 *pdata,UINT16 *len,UINT8 endp,UINT8 *tog ,UINT8 hubaddr);
extern UINT8 USBHS_CtrlGetConfigDescr( PHUB_Port_Info phub, UINT8 *Databuf );
extern void USBHS_HubUSBHS_Analysis_Descr(PHUB_Port_Info portn,PUINT8 pdesc, UINT16 l);
extern UINT8 USBHS_USBHostTransact(UINT8 endp_pid,UINT8 toggle,UINT32 timeout) ;
extern UINT8 USBHS_ISO_Send_Data( PHUB_Port_Info phub,UINT8 *pdata,UINT16 len,UINT8 endp );
extern UINT8 USBHS_ISO_Rec_Data( PHUB_Port_Info phub,UINT8 *pdata,UINT16 *len,UINT8 endp );
extern UINT8 USBHS_USBHostTransactISO(UINT8 endp_pid, UINT16 *plen, UINT8 tog);

extern u8 USBHS_glable_index ;
#endif

