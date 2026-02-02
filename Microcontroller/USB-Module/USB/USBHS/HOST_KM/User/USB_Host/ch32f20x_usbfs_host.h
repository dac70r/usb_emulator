/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32v30x_usbotg_host.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : USBOTG full speed host header file
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/


#ifndef __CH32V30x_USBFS_HOST_H__
#define __CH32V30x_USBFS_HOST_H__

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************/
/* Header File */
#include "ch32f20x.h"
#include "ch32f20x_usb.h"
#include "ch32f20x_usbhs_host.h"
#include "usb_hub.h"

/*******************************************************************************/
/* Macro Definition */
#ifndef VOID
#define VOID                    void
#endif
#ifndef CONST
#define CONST                   const
#endif
#ifndef BOOL
typedef unsigned char           BOOL;
#endif
#ifndef BOOLEAN
typedef unsigned char           BOOLEAN;
#endif
#ifndef CHAR
typedef char                    CHAR;
#endif
#ifndef INT8
typedef char                    INT8;
#endif
#ifndef INT16
typedef short                   INT16;
#endif
#ifndef INT32
typedef long                    INT32;
#endif
#ifndef UINT8
typedef unsigned char           UINT8;
#endif
#ifndef UINT16
typedef unsigned short          UINT16;
#endif
#ifndef UINT32
typedef unsigned long           UINT32;
#endif
#ifndef UINT8V
typedef unsigned char volatile  UINT8V;
#endif
#ifndef UINT16V
typedef unsigned short volatile UINT16V;
#endif
#ifndef UINT32V
typedef unsigned long volatile  UINT32V;
#endif

#ifndef PVOID
typedef void                    *PVOID;
#endif
#ifndef PCHAR
typedef char                    *PCHAR;
#endif
#ifndef PCHAR
typedef const char              *PCCHAR;
#endif
#ifndef PINT8
typedef char                    *PINT8;
#endif
#ifndef PINT16
typedef short                   *PINT16;
#endif
#ifndef PINT32
typedef long                    *PINT32;
#endif
#ifndef PUINT8
typedef unsigned char           *PUINT8;
#endif
#ifndef PUINT16
typedef unsigned short          *PUINT16;
#endif
#ifndef PUINT32
typedef unsigned long           *PUINT32;
#endif
#ifndef PUINT8V
typedef volatile unsigned char  *PUINT8V;
#endif
#ifndef PUINT16V
typedef volatile unsigned short *PUINT16V;
#endif
#ifndef PUINT32V
typedef volatile unsigned long  *PUINT32V;
#endif



/* USB Buffer Size */
#ifndef USBFS_MAX_PACKET_SIZE
#define USBFS_MAX_PACKET_SIZE       64    
#endif
/*******************************************************************************/
/* Constant Definition */
#ifndef DEF_USB_GEN_ENUM_CMD
#define DEF_USB_GEN_ENUM_CMD
/* Get Device Descriptor Command Packet */
__attribute__((aligned(4))) static const uint8_t  SetupGetDevDesc[ ] =
{
    USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_DEVICE, 0x00, 0x00, sizeof( USB_DEV_DESCR ), 0x00
};

/* Get Configuration Descriptor Command Packet */
__attribute__((aligned(4))) static const uint8_t SetupGetCfgDesc[ ] =
{
    USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_CONFIG, 0x00, 0x00, 0x04, 0x00
};

/* Get String Descriptor Command Packet */
__attribute__((aligned(4))) static const uint8_t SetupGetStrDesc[ ] =
{
    USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_STRING, 0x09, 0x04, 0x04, 0x00
};

/* Set USB Address Command Packet */
__attribute__((aligned(4))) static const uint8_t SetupSetAddr[ ] =
{
    USB_REQ_TYP_OUT, USB_SET_ADDRESS, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* Set USB Configuration Command Packet */
__attribute__((aligned(4))) static const uint8_t SetupSetConfig[ ] =
{
    USB_REQ_TYP_OUT, USB_SET_CONFIGURATION, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* Clear Endpoint STALL Command Packet */
__attribute__((aligned(4))) static const uint8_t SetupClearEndpStall[ ] =
{
    USB_REQ_TYP_OUT | USB_REQ_RECIP_ENDP, USB_CLEAR_FEATURE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* Set Device Interface Command Packet */
__attribute__((aligned(4))) static const uint8_t SetupSetInterface[ ] =
{
    USB_REQ_RECIP_INTERF, USB_SET_INTERFACE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
#endif

//endpoint
#define DEF_ENDP_0     0
#define DEF_ENDP_1     1
#define DEF_ENDP_2     2
#define DEF_ENDP_3     3
#define DEF_ENDP_4     4
#define DEF_ENDP_5     5
#define DEF_ENDP_6     6
#define DEF_ENDP_7     7
#define DEF_ENDP_8     8
#define DEF_ENDP_9     9
#define DEF_ENDP_10    10
#define DEF_ENDP_11    11
#define DEF_ENDP_12    12
#define DEF_ENDP_13    13
#define DEF_ENDP_14    14
#define DEF_ENDP_15    15


#define MAX_PACKET_SIZE         1024      /* maximum packet size */
#define WAIT_USB_TOUT_200US     3000
#define WAIT_TIME               1000000

/*******************************************************************************/


extern __attribute__ ((aligned(4))) UINT8   USBFS_endpTXbuf[ MAX_PACKET_SIZE ];  // OUT, must even address
extern __attribute__ ((aligned(4))) UINT8   USBFS_endpRXbuf[ MAX_PACKET_SIZE ];  // OUT, must even addres

extern __attribute__ ((aligned(4))) UINT8 USBFS_test_buf[3072];// //高速高带宽端点一个微帧内最大可以有3次事务数 1024*3=3072


/* USB Setup Request */
#define pUSBFS_SetupRequest        ( (PUSB_SETUP_REQ)USBFS_endpTXbuf )


/*******************************************************************************/
/* Function Declaration */
extern void USBFS_RCC_Init( void );
extern void USBFS_Host_Init( FunctionalState sta );
extern uint8_t USBFSH_CheckRootHubPortStatus( uint8_t dev_sta );
extern uint8_t USBFSH_CheckRootHubPortEnable( void );
extern uint8_t USBFSH_CheckRootHubPortSpeed( void );
extern void USBFSH_SetSelfAddr( uint8_t addr );
extern void USBFSH_SetSelfSpeed( uint8_t speed );
extern void USBFSH_ResetRootHubPort( uint8_t mode );
extern uint8_t USBFSH_EnableRootHubPort( uint8_t *pspeed );
extern uint8_t USBFSH_Transact( uint8_t endp_pid, uint8_t endp_tog, uint32_t timeout );
extern uint8_t USBFSH_CtrlTransfer( uint8_t ep0_size, uint8_t *pbuf, uint16_t *plen );
extern uint8_t USBFSH_GetDeviceDescr( uint8_t *pep0_size, uint8_t *pbuf );
extern uint8_t USBFSH_GetConfigDescr( uint8_t ep0_size, uint8_t *pbuf, uint16_t buf_len, uint16_t *pcfg_len );
extern uint8_t USBFSH_GetStrDescr( uint8_t ep0_size, uint8_t str_num, uint8_t *pbuf );
extern uint8_t USBFSH_SetUsbAddress( uint8_t ep0_size, uint8_t addr );
extern uint8_t USBFSH_SetUsbConfig( uint8_t ep0_size, uint8_t cfg_val );
extern uint8_t USBFSH_ClearEndpStall( uint8_t ep0_size, uint8_t endp_num );
extern uint8_t USBFSH_GetEndpData( uint8_t endp_num, uint8_t *pendp_tog, uint8_t *pbuf, uint16_t *plen );
extern uint8_t USBFSH_SendEndpData( uint8_t endp_num, uint8_t *pendp_tog, uint8_t *pbuf, uint16_t len );


UINT8 USBFS_HUBHostEnum( UINT8 depth, UINT8 *Databuf,UINT8 port );
UINT8 USBFS_HostEnum( UINT8 depth, UINT8 *Databuf );
UINT8 USBFS_Del_AddressNumber(UINT8 addr);
UINT8 USBFS_Set_AddressNumber(void);

#ifdef __cplusplus
}
#endif

#endif

