/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32v30x_usbhs_host.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : This file provides all the USB firmware functions.
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/ 
#include <usb_hub.h>
#include "ch32f20x_usbhs_host.h"
#include "stdlib.h"
#include "ch32f20x_usb.h"
#include "ch32f20x_usbfs_device.h"
#include "usbd_composite_km.h"
#include "main.h"

UINT8  USBHS_UsbDevEndp0Size = 0;
UINT8  USBHS_gEndp_Num = 0;
UINT8  USBHS_Hid_Report_Len = 0;
UINT16 USBHS_EndpnMaxSize = 0;
USBDEV_INFO  USBHS_thisUsbDev;

uint8_t  MS_Data_Packs[ 4 ] = { 0x00, 0x00, 0x00, 0x00 };                                          // Mouse IN Data Packet
uint8_t  KB_Data_Packs[ 8 ] = { 0x00, 0x00, 0x00, 0x00 ,0x00 ,0x00, 0x00, 0x00};                   // Keyboard IN Data Packet

UINT8 USBHS_AddressNum[127] = {0};
UINT8 USBHS_glable_index = 0;
__attribute__ ((aligned(4))) UINT8 USBHS_Test_Buf[3072];// //高速高带宽端点一个微帧内最大可以有3次事务数 1024*3=3072

/******************************** HOST DEVICE **********************************/
__attribute__ ((aligned(4))) const UINT8  USBHS_GetDevDescrptor[]={USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_DEVICE, 0x00, 0x00, 0x12, 0x00};
__attribute__ ((aligned(4))) const UINT8  USBHS_GetConfigDescrptor[]= {USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_CONFIG, 0x00, 0x00, 0x04, 0x00};
__attribute__ ((aligned(4))) const UINT8  USBHS_SetAddress[]={USB_REQ_TYP_OUT, USB_SET_ADDRESS, USB_DEVICE_ADDR, 0x00, 0x00, 0x00, 0x00, 0x00};
__attribute__ ((aligned(4))) const UINT8  USBHS_SetConfig[]={USB_REQ_TYP_OUT, USB_SET_CONFIGURATION, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
__attribute__ ((aligned(4))) const UINT8  USBHS_Clear_EndpStall[]={USB_REQ_RECIP_INTERF, USB_SET_INTERFACE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
__attribute__ ((aligned(4))) const UINT8  USBHS_SetupSetUsbInterface[] = { USB_REQ_RECIP_INTERF, USB_SET_INTERFACE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
__attribute__ ((aligned(4))) const UINT8  USBHS_SetupClrEndpStall[] = { USB_REQ_TYP_OUT | USB_REQ_RECIP_ENDP, USB_CLEAR_FEATURE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };


/* USB HUB Class Request Types */
#define USB_CLEAR_HUB_FEATURE           0x20
#define USB_CLEAR_PORT_FEATURE          0x23
#define USB_CLEAR_TT_BUFFER             0x23
#define USB_GET_HUB_DESCR               0xA0
#define USB_GET_HUB_STATUS              0xA0
#define USB_GET_PORT_STATUS             0xA3
#define USB_RESET_TT                    0x23
#define USB_SET_HUB_DESCR               0x20
#define USB_SET_HUB_FEATURE             0x20
#define USB_SET_PORT_FEATURE            0x23
#define USB_GET_TT_STATUS               0xA3
#define USB_STOP_TT                     0x23

#define PID_SSPLIT          (0<<0)
#define PID_CSPLIT          (1<<0)
#define SS_ALL              (3<<8)
#define SS_BEGIN            (1<<8)
#define SS_MIDDLE           (0<<8)
#define SS_END              (2<<8)
#define ENDP3_TX_ISO        (1<<3)
#define RB_UH_EP_RX_TYPE    (1<<18)
#define R32_UH_EP_TYPE      (*((PUINT32V)(0x40023414)))
/*********************************************************************
 * @fn      USB20_RCC_Init
 *
 * @brief   USB RCC initialized
 *
 * @return  none
 */
void USB20_RCC_Init( void )
{
    RCC->CFGR2 = USBHS_PLL_SRC_HSE | USBHS_PLL_SRC_PRE_DIV2 | USBHS_PLL_CKREF_4M;//PLL REF = HSE/2 = 4MHz
    RCC->CFGR2 |= USBHS_CLK_SRC_PHY|USBHS_PLLALIVE;
    RCC->AHBPCENR |= RCC_AHBPeriph_USBHS;                  //USB clock enable
    Delay_Us(200);
    USBHSH->HOST_CTRL |= PHY_SUSPENDM;
    Delay_Us(5);
}

/*********************************************************************
 * @fn      USBHS_HostInit
 *
 * @brief   USB host mode initialized.
 *
 * @param   sta - ENABLE or DISABLE
 *
 * @return  none
 */
void USBHS_HostInit (FunctionalState sta)  // USBHS host initial
{
    if(sta==ENABLE)
    {
        USB20_RCC_Init();

        USBHSH->CONTROL =  INT_BUSY_EN | DMA_EN | HIGH_SPEED | HOST_MODE;
        USBHSH->HOST_CTRL = PHY_SUSPENDM | SEND_SOF_EN;
        USBHSH->HOST_EP_CONFIG = HOST_TX_EN | HOST_RX_EN ;                // send enable, receive enable
        USBHSH->HOST_RX_MAX_LEN = 1024;                                    // receive max length
        USBHSH->HOST_RX_DMA = (UINT32)USBHS_endpRXbuf;
        USBHSH->HOST_TX_DMA = (UINT32)USBHS_endpTXbuf;
        R8_USB_INT_EN |= RB_UIE_SOF_ACT;
    }
    else
    {
        USBHSH->CONTROL = USB_FORCE_RST | USB_ALL_CLR;
    }
}

/*********************************************************************
 * @fn      USBHS_SetBusReset
 *
 * @brief   Reset USB bus
 *
 * @return  none
 */
void  USBHS_SetBusReset(void)
{
    USBHSH->HOST_CTRL |= SEND_BUS_RESET;                              //bus reset
    Delay_Ms(15);
    USBHSH->HOST_CTRL &= ~SEND_BUS_RESET;
    while( !(USBHSH->HOST_CTRL & UH_SOFT_FREE) );                     //wait bus idle;
    USBHSH->HOST_CTRL |= SEND_SOF_EN;                                 //sof enable
    if( (USBHSH->SPEED_TYPE & USBSPEED_MASK ) == 1 )   USBHS_thisUsbDev.DeviceSpeed = USB_HIGH_SPEED;

}

/*********************************************************************
 * @fn      USBHS_CopySetupReqPkg
 *
 * @brief   copy the contents of the buffer to send buffer.
 *
 * @param   pReqPkt - target buffer address
 *
 * @return  none
 */
void USBHS_CopySetupReqPkg( const UINT8 *pReqPkt )
{
    UINT8 i;

    for ( i = 0; i != sizeof( USB_SETUP_REQ ); i ++ )
    {
        ((PUINT8)pSetupReq)[ i ] = *pReqPkt;
        pReqPkt++;
    }
}

/*********************************************************************
 * @fn      USBHS_USBHostTransact
 *
 * @brief   General host transaction
 *
 * @param   endp_pid - PID of the transaction  and the number of endpoint
 *          toggle - sync  trigger bit
 *          timeout - value of timeout
 *
 * @return  Error state
 */
UINT8 USBHS_USBHostTransact(UINT8 endp_pid,UINT8 toggle,UINT32 timeout)  //general
{
    UINT8   TransRetry,r;
    UINT8   s;
    UINT32  i;

    USBHSH->HOST_TX_CTRL = USBHSH->HOST_RX_CTRL = toggle;
    TransRetry = 0;
    do
    {
        USBHSH->HOST_EP_PID = endp_pid ;                                      //设置主机发送包的令牌
        USBHSH->INT_FG = USBHS_ACT_FLAG;                                      //清发送完成中断
        for ( i = WAIT_USB_TOUT_200US; i != 0 && ((USBHSH->INT_FG) & USBHS_ACT_FLAG) == 0 ; i -- )//
        {
            Delay_Us( 1 );                                                      //等待发送完成
        }
        USBHSH->HOST_EP_PID = 0x00;
        if ( (USBHSH->INT_FG & USBHS_ACT_FLAG) == 0 )  return( ERR_USB_UNKNOWN );

        s  = USBHSH->INT_FG ;
        if( s & USBHS_DETECT_FLAG )                                             //当前设备被拔除
        {
            USBHSH->INT_FG = USBHS_DETECT_FLAG;
            Delay_Us(200);
            if( USBHSH->MIS_ST & USBHS_ATTCH )
            {
                if(USBHSH->HOST_CTRL & SEND_SOF_EN)    return ( ERR_USB_CONNECT );
            }
            else    return ( ERR_USB_DISCON );
        }
        else if ( s & USBHS_ACT_FLAG )                                                        //数据包传输成功
        {
            r = USBHSH->INT_ST & USBHS_HOST_RES;
            if((endp_pid >> 4) == USB_PID_IN )
            {
                if ( (USBHSH->INT_ST & USBHS_TOGGLE_OK) )
                {
                    return( ERR_SUCCESS);
                }             //数据包令牌匹配
            }
            else
            {
                if ( (r == USB_PID_ACK)||(r == USB_PID_NYET) )
                {
                    return( ERR_SUCCESS );                //setup/out包等待设备响应ACK
                }
            }
            if ( r == USB_PID_STALL )                    return( r | ERR_USB_TRANSFER );       //设备响应STALL

            if ( r == USB_PID_NAK )                              //设备响应NAK 超时
            {
                if ( timeout == 0 )                      return( r | ERR_USB_TRANSFER );
                if ( timeout < 0xFFFF ) timeout --;
                -- TransRetry;
            }
            else switch ( endp_pid >> 4  )
            {
                case USB_PID_SETUP:

                case USB_PID_OUT:
                    if ( r ) return( r | ERR_USB_TRANSFER );           //
                    break;
                case USB_PID_IN:                                        //2b
                    if ( (r == USB_PID_DATA0) || (r == USB_PID_DATA1) ){
                    }
                    else if ( r ) return( r | ERR_USB_TRANSFER );
                    break;
                default:
                    return( ERR_USB_UNKNOWN );
            }
        }
        else {
            USBHSH->INT_FG = 0x3F;
        }
        Delay_Us( 15 );
    } while ( ++ TransRetry < 3 );

    return( ERR_USB_TRANSFER );
}


/*********************************************************************
 * @fn      USBHS_HostCtrlTransfer
 *
 * @brief   Host control transfer.
 *
 * @param   databuf - Receiving or send data buffer.
 *          RetLen - Data length.
 *
 * @return  Error state
 */
UINT8 USBHS_HostCtrlTransfer(PUINT8 databuf,PUINT8 len)
{
    UINT8   ret;
    UINT8   tog = 1;
    PUINT8  pBuf;
    UINT16  relen;
    PUINT8  pLen;
    UINT32  rxlen;

    pBuf = databuf;
    pLen = len;

    if( pLen )  *pLen = 0;
    Delay_Us( 100 );
    USBHSH->HOST_TX_LEN = 8;
    ret = USBHS_USBHostTransact( (USB_PID_SETUP<<4)|DEF_ENDP_0, 0, 200000 );
    if(ret != ERR_SUCCESS)      return ( ret );                     //error

    relen = pSetupReq->wLength;

    if(relen && pBuf)                                           //data stage
    {
       if( (pSetupReq->bRequestType) & USB_REQ_TYP_IN )            //device to host
       {
           while(relen)
           {
               if( USBHS_thisUsbDev.DeviceSpeed != USB_HIGH_SPEED )
                   Delay_Us( 100 );
               USBHSH->HOST_RX_DMA = (UINT32)databuf + *pLen;
               ret = USBHS_USBHostTransact( (USB_PID_IN<<4)| DEF_ENDP_0, tog<<3, 20000 );
               if(ret != ERR_SUCCESS)                return ( ret );
               tog ^=1;
               rxlen = (USBHSH->RX_LEN < relen) ? USBHSH->RX_LEN : relen;
               relen -= rxlen;
               if(pLen)  *pLen += rxlen;
               if( ( USBHSH->RX_LEN == 0 ) || (USBHSH->RX_LEN & ( USBHS_UsbDevEndp0Size - 1 )))  break;
            }
            USBHSH->HOST_TX_LEN = 0 ;
         }
       else
       {                                                           // host to device
          while(relen)
          {
              if( USBHS_thisUsbDev.DeviceSpeed != USB_HIGH_SPEED )
                  Delay_Us( 100 );
               USBHSH->HOST_TX_DMA = (UINT32)databuf + *pLen;
               USBHSH->HOST_TX_LEN = (relen >= USBHS_UsbDevEndp0Size)? USBHS_UsbDevEndp0Size : relen;

               ret = USBHS_USBHostTransact((USB_PID_OUT<<4)|DEF_ENDP_0,  tog<<3,  20000);
               if(ret != ERR_SUCCESS)               return  ( ret );
               tog ^=1;
               relen -= USBHSH->HOST_TX_LEN;
               if( pLen )  *pLen += USBHSH->HOST_TX_LEN;
          }
        }
    }
    if( USBHS_thisUsbDev.DeviceSpeed != USB_HIGH_SPEED )
        Delay_Us( 100 );
    ret = USBHS_USBHostTransact( ((USBHSH->HOST_TX_LEN) ? USB_PID_IN<<4|DEF_ENDP_0 : USB_PID_OUT<<4|DEF_ENDP_0),
            UH_R_TOG_1|UH_T_TOG_1, 20000 );
    if(ret != ERR_SUCCESS)            return( ret );

    if ( USBHSH->HOST_TX_LEN == 0 )   return( ERR_SUCCESS );    //status stage is out, send a zero-length packet.

    if ( USBHSH->RX_LEN == 0 )        return( ERR_SUCCESS );    //status stage is in, a zero-length packet is returned indicating success.

    return ERR_USB_BUF_OVER;
}

/*********************************************************************
 * @fn      USBHS_CtrlGetDevDescr
 *
 * @brief   Get device descrptor
 *
 * @return  Error state
 */
UINT8 USBHS_CtrlGetDevDescr( UINT8 *Databuf )
{
    UINT8 ret;
    UINT8 len;
    USBHS_UsbDevEndp0Size = 8;
    USBHS_CopySetupReqPkg( USBHS_GetDevDescrptor );
    pSetupReq->wLength = USBHS_UsbDevEndp0Size;
    ret = USBHS_HostCtrlTransfer( Databuf,	&len );
	
    if( ret != ERR_SUCCESS )                     return  ( ret );
//    if( len < USBHS_UsbDevEndp0Size )                  return  ERR_USB_BUF_OVER;
    USBHS_UsbDevEndp0Size = ((PUSB_DEV_DESCR)Databuf)->bMaxPacketSize0;

    USBHS_CopySetupReqPkg( USBHS_GetDevDescrptor );                               //获取全部设备描述符
    ret = USBHS_HostCtrlTransfer( Databuf,	&len );
	
    if( ret != ERR_SUCCESS )                     return  ( ret );
		printf("	USBHS_CtrlGetDevDescr: Retrieved Device Description\n");
    return( ERR_SUCCESS );
}

/*********************************************************************
 * @fn      USBHS_CtrlGetConfigDescr
 *
 * @brief   Get configuration descriptor.
 *
 * @return  Error state
 */
UINT8 USBHS_CtrlGetConfigDescr( PHUB_Port_Info phub, UINT8 *Databuf )
{
    UINT8  ret;
    UINT8  len;
    UINT16 reallen;

		// ---- First Transfer: Read header (9 bytes) ----
    USBHS_CopySetupReqPkg( USBHS_GetConfigDescrptor );									
    ret = USBHS_HostCtrlTransfer( Databuf, &len );
    if(ret != ERR_SUCCESS)             return  ( ret );
	
		// Should always get at least wLength (usually 9 bytes)
    if(len < ( pSetupReq->wLength ) )  return  ERR_USB_BUF_OVER;
	
		// Extract full configuration descriptor length
    reallen = ((PUSB_CFG_DESCR)Databuf)-> wTotalLength;             																		// 解析全部配置描述符的长度

		// ---- Second Transfer: Read full configuration descriptor ----
    USBHS_CopySetupReqPkg( USBHS_GetConfigDescrptor );
    pSetupReq->wLength = reallen;
    ret = USBHS_HostCtrlTransfer( (UINT8 *)Databuf, &len );               															// 获取全部配置描述符
    if( ret != ERR_SUCCESS )           return  ( ret );

		// Save Configuration Value
    USBHS_thisUsbDev.DeviceCongValue = ( (PUSB_CFG_DESCR)Databuf )-> bConfigurationValue;								// DeviceCongValue = DeviceConfigValue, we will use 1 Config value at all times to avoid confusion
    printf("	\tUSBHS_CtrlGetConfigDescr: USBHS_thisUsbDev.DeviceType=%02x\n", USBHS_thisUsbDev.DeviceType);													
	
    if( USBHS_thisUsbDev.DeviceType == 0x09 ){        																									// HUB处理
        USBHS_HubUSBHS_Analysis_Descr( phub,(UINT8 *)Databuf, pSetupReq->wLength );
    }
    else{                   																																						// 其他设备处理
        USBHS_Analysis_Descr( &USBHS_thisUsbDev, (UINT8 *)Databuf, pSetupReq->wLength );
    }
		
    return( ERR_SUCCESS );
}

/*********************************************************************
 * @fn      CtrlSetUsbAddress
 *
 * @brief   Set USB device address.
 *
 * @param   addr - Device address.
 *
 * @return  Error state
 */
UINT8 USBHS_CtrlUSBHS_SetAddress(UINT8 addr)
{
    UINT8 ret;

    USBHS_CopySetupReqPkg( USBHS_SetAddress );
    pSetupReq->wValue = addr;
    ret = USBHS_HostCtrlTransfer( NULL, NULL );
    if(ret != ERR_SUCCESS)  return  ( ret );
    USBHS_CurrentAddr( addr );
    Delay_Ms(5);
    return  ERR_SUCCESS;
}

/*********************************************************************
 * @fn      USBHS_CtrlSetUsbConfig
 *
 * @brief   Set usb configration.
 *
 * @param   cfg_val - device configuration value
 *
 * @return  Error state
 */
UINT8 USBHS_CtrlSetUsbConfig( UINT8 cfg_val)
{
    USBHS_CopySetupReqPkg( USBHS_SetConfig );
    pSetupReq->wValue = cfg_val;
    return( USBHS_HostCtrlTransfer( NULL, NULL ));
}

/*********************************************************************
 * @fn      USBHS_CtrlSetIdle
 *
 * @brief   Set Idle.
 *
 * @param   None
 *
 * @return  Error state
 */
UINT8 USBHS_CtrlSetIdle( void )
{
    pSetupReq->bRequestType = 0x21;
    pSetupReq->bRequestType = 0x0a;
    pSetupReq->wValue = 0;
    pSetupReq->wIndex = 0;
    pSetupReq->wLength = 0;
    return( USBHS_HostCtrlTransfer( NULL, NULL ));
}

/*********************************************************************
 * @fn      USBHS_CtrlGetReportDescr
 *
 * @brief   Get hid report
 *
 * @return  Error state
 */
UINT8 USBHS_CtrlGetReportDescr( UINT8 *Databuf )
{
    UINT8 ret;

    pSetupReq->bRequestType = 0x81;
    pSetupReq->bRequest = 0x06;
    pSetupReq->wValue = 0x2200;
    pSetupReq->wIndex = 0x0;
    pSetupReq->wLength = USBHS_Hid_Report_Len;

    ret = USBHS_HostCtrlTransfer( Databuf, &USBHS_Hid_Report_Len );
    if( ret != ERR_SUCCESS )                     return  ( ret );
    return( ERR_SUCCESS );
}

/*********************************************************************
 * @fn      USBHS_CtrlClearEndpStall
 *
 * @brief   clear endpoint stall status.
 *
 * @param   endp - number of endpoint
 *
 * @return  Error state
 */
UINT8 USBHS_CtrlClearEndpStall( UINT8 endp )
{
    USBHS_CopySetupReqPkg( USBHS_SetupClrEndpStall );
    pSetupReq -> wIndex = endp;
    return( USBHS_HostCtrlTransfer( NULL, NULL ) );
}

/*********************************************************************
 * @fn      USBHS_CtrlSetUsbIntercace
 *
 * @brief   Set USB Interface configuration.
 *
 * @param   cfg - configuration value
 *
 * @return  Error state
 */
UINT8 USBHS_CtrlSetUsbIntercace( UINT8 cfg )
{
    USBHS_CopySetupReqPkg( USBHS_SetupSetUsbInterface );
    pSetupReq -> wValue = cfg;
    return( USBHS_HostCtrlTransfer( NULL, NULL ) );
}

/*********************************************************************
 * @fn      USBHS_HubGetPortStatus
 *
 * @brief   Check the port of hub,and return the port's status
 *
 * @param   HubPortIndex - index of the hub port index
 *
 * @return  Error state
 */
UINT8   USBHS_HubGetPortStatus( UINT8 depth ,UINT8 HubPortIndex )
{
    UINT8   s;
    UINT8  len;
    UINT16 Port_Change_Field;

    pSetupReq -> bRequestType = HUB_GET_PORT_STATUS;
    pSetupReq -> bRequest = HUB_GET_STATUS;
    pSetupReq -> wValue = 0x0000;
    pSetupReq -> wIndex = 0x0000|HubPortIndex;
    pSetupReq -> wLength = 0x0004;
    s = USBHS_HostCtrlTransfer( USBHS_endpRXbuf, &len );         // 执行控制传输
    if ( s != ERR_SUCCESS )
    {
        return( s );
    }
    if ( len < 4 )
    {
        return( ERR_USB_BUF_OVER );                  // 描述符长度错误
    }

    Port_Change_Field = USBHS_endpRXbuf[2];
    Port_Change_Field |= ((UINT16)USBHS_endpRXbuf[3]<<8);

    USBHS_HubInfo[depth].portD[HubPortIndex-1].PortpChangeField = Port_Change_Field;

    return( ERR_SUCCESS );
}

/*********************************************************************
 * @fn      USBHS_HubSetPortFeature
 *
 * @brief   set the port feature of hub
 *
 * @param   HubPortIndex - index of the hub port index
 *          FeatureSelt - feature selector
 *
 * @return  Error state
 */
UINT8   USBHS_HubSetPortFeature( UINT8 HubPortIndex, UINT8 FeatureSelt )
{
    pSetupReq -> bRequestType = HUB_SET_PORT_FEATURE;
    pSetupReq -> bRequest = HUB_SET_FEATURE;
    pSetupReq -> wValue = 0x0000|FeatureSelt;
    pSetupReq -> wIndex = 0x0000|HubPortIndex;
    pSetupReq -> wLength = 0x0000;
    return( USBHS_HostCtrlTransfer( NULL, NULL ) );     // 执行控制传输
}

/*********************************************************************
 * @fn      USBHS_HubClearPortFeature
 *
 * @brief   clear the port feature of hub
 *
 * @param   HubPortIndex - index of the hub port index
 *          FeatureSelt - feature selector
 *
 * @return  Error state
 */

UINT8   USBHS_HOST_ClearPortFeature_Process( UINT8 HubPortIndex, UINT8 FeatureSelt )
{
    pSetupReq -> bRequestType = HUB_CLEAR_PORT_FEATURE;
    pSetupReq -> bRequest = HUB_CLEAR_FEATURE;
    pSetupReq -> wValue = 0x0000|FeatureSelt;
    pSetupReq -> wIndex = 0x0000|HubPortIndex;
    pSetupReq -> wLength = 0x0000;
    return( USBHS_HostCtrlTransfer( NULL, NULL ) );     // 执行控制传输
}

/*********************************************************************
 * @fn      USBHS_HubClearPortFeature
 *
 * @brief   clear the port feature of hub
 *
 * @param   depth - hub depth
 *          port - hub port
 *
 * @return  state
 */
UINT8  USBHS_HubClearPortFeature( UINT8 depth,UINT8 port )
{
    UINT8 status = 0;
    if( USBHS_HubInfo[depth].portD[port-1].PortpChangeField&0x01 )
    {
        status = USBHS_HOST_ClearPortFeature_Process( port,16 );
        if( status == ERR_SUCCESS )return status;
    }
    if( USBHS_HubInfo[depth].portD[port-1].PortpChangeField&0x02 )
    {
        status = USBHS_HOST_ClearPortFeature_Process( port,17 );
        if( status == ERR_SUCCESS )return status;
    }
    if( USBHS_HubInfo[depth].portD[port-1].PortpChangeField&0x04 )
    {
        status = USBHS_HOST_ClearPortFeature_Process( port,18 );
        if( status == ERR_SUCCESS )return status;
    }
    if( USBHS_HubInfo[depth].portD[port-1].PortpChangeField&0x08 )
    {
        status = USBHS_HOST_ClearPortFeature_Process( port,19 );
        if( status == ERR_SUCCESS )return status;
    }
    if( USBHS_HubInfo[depth].portD[port-1].PortpChangeField&0x10 )
    {
        status = USBHS_HOST_ClearPortFeature_Process( port,20 );
        if( status == ERR_SUCCESS )return status;
    }
    return status;
}

/*********************************************************************
 * @fn      USBHS_CtrlGet_HUBDevDescr
 *
 * @brief   Get HUB descriptor
 *
 * @param   Databuf - data buffer
 *
 * @return  state
 */
UINT8 USBHS_CtrlGet_HUBDevDescr( UINT8 *Databuf )
{
    UINT8 ret;
    UINT8 len;
    UINT8 buf[8] =
    {
           0xA0,0x06,0x00,0x29,0x00,0x00,0x02,0x00
    };
    USBHS_CopySetupReqPkg( buf );
    pSetupReq->wLength = USBHS_UsbDevEndp0Size;
    ret = USBHS_HostCtrlTransfer( Databuf,&len );
    if( ret != ERR_SUCCESS )                     return  ( ret );
    buf[6] = *Databuf;
    USBHS_CopySetupReqPkg( buf );                               //获取全部设备描述符
    ret = USBHS_HostCtrlTransfer( Databuf,&len );
    if( ret != ERR_SUCCESS )                     return  ( ret );
    return( ERR_SUCCESS );
}

/*********************************************************************
 * @fn      USBHS_CtrlGet_HUBDevDescr
 *
 * @brief   Get HUB descriptor
 *
 * @param   pDataBuf - 8-byte request
 *          pRetLen - Actual number of bytes sent and received
 *
 * @return  state
 */
UINT8 USBHS_HubCtrlTransfer( PHUB_Port_Info portn, uint8_t *pDataBuf, uint16_t *pRetLen, UINT8 hubaddr)
{
    uint16_t  RemLen, len,i;
    uint8_t   s, ctrltog;
    uint8_t   *pBuf;
    uint16_t  *pLen;
    pBuf = pDataBuf;
    pLen = pRetLen;
    if ( pLen ) *pLen = 0;

/**************** SETUP */
    len = sizeof( USB_SETUP_REQ );
    s = USBHS_SSPLITPacket(portn, (uint8_t *)pSetupReq, &len, USB_PID_SETUP << 4 | 0x00, 0 , hubaddr);
    if( s != ERR_SUCCESS )  return s;
    s = USBHS_CSPLITPacket(portn, NULL, NULL, USB_PID_SETUP << 4 | 0x00, 0, hubaddr);
    if( s != ERR_SUCCESS )  return s;

/**************** DATA */
    RemLen = pSetupReq -> wLength;
    ctrltog = 1;
    if ( RemLen && pBuf )
    {
        if ( pSetupReq -> bRequestType & USB_REQ_TYP_IN )
        {
            while ( RemLen )
            {
                Delay_Us( 200 );
                s = USBHS_SSPLITPacket(portn, pBuf, &len, USB_PID_IN << 4 | 0x00, ctrltog ,hubaddr);
                if( s != ERR_SUCCESS )  {return s;}
                s = USBHS_CSPLITPacket(portn, pBuf, &len, USB_PID_IN << 4 | 0x00, ctrltog ,hubaddr);
                if( s == HUB_CS_NAK )continue;
                if( s != ERR_SUCCESS )  {return s;}
                ctrltog ^= 1;
                RemLen -= len;
                pBuf += len;
                if ( pLen ) *pLen += len;
                if ( len == 0 || ( len & ( USBHS_UsbDevEndp0Size - 1 ) ) ) break;
            }
            ctrltog = 0;
        }
        else
        {
            while ( RemLen )
            {
                len = RemLen >= USBHS_UsbDevEndp0Size ? USBHS_UsbDevEndp0Size : RemLen;
                s = USBHS_SSPLITPacket(portn, pBuf, &len, USB_PID_OUT << 4 | 0x00, ctrltog ,hubaddr);
                if( s != ERR_SUCCESS )  return s;
                s = USBHS_CSPLITPacket(portn, pBuf, &len, USB_PID_OUT << 4 | 0x00, ctrltog ,hubaddr);
                if( s == HUB_CS_NAK )continue;
                if( s != ERR_SUCCESS )  return s;
                ctrltog ^= 1;
                RemLen -= len;
                pBuf += len;
                if ( pLen ) *pLen += len;
            }
            ctrltog = 1;
        }
    }
    Delay_Us( 200 );
    len = 0;
    for( i=0;i!=100;i++ )
    {
        s = USBHS_SSPLITPacket(portn, pBuf, &len, (ctrltog ? USB_PID_IN : USB_PID_OUT) << 4 | 0x00, 1 ,hubaddr);
        if( s != ERR_SUCCESS )  return s;
        s = USBHS_CSPLITPacket(portn, pBuf, &len, (ctrltog ? USB_PID_IN : USB_PID_OUT) << 4 | 0x00, 1 ,hubaddr);
        if( s == HUB_CS_NAK )continue;
        if( s != ERR_SUCCESS )  return s;
        if( len != 0 )    return( ERR_USB_BUF_OVER );
        return( ERR_SUCCESS );
    }
}

/*********************************************************************
 * @fn      USBHS_HubUSBGetDevDescr
 *
 * @brief   Get device descriptor for USB HUB 
 *
 * @param   Databuf - data buffer
 *
 * @return  state
 */
UINT8 USBHS_HubUSBGetDevDescr( PHUB_Port_Info phub, UINT8 *Databuf ,UINT8 hubaddr)
{
    UINT8 ret;
    UINT16 len;
	
		/* Manually Setup the standard USB device descriptor request */
    pSetupReq->bRequestType = 0x80;
    pSetupReq->bRequest 		= 0x06;
    pSetupReq->wValue 			= 0x0100;
    pSetupReq->wIndex 			= 0x0000;
    pSetupReq->wLength 			= 0x12;
	
	ret = USBHS_HubCtrlTransfer( phub, Databuf, &len ,hubaddr);											// phub: pointer to HUB_Port_Info, Databuf: data buffer, &len: pointer to uint16 variable, contains actual bytes received
																																									// hubaddr: USB address of the hub  
    USBHS_UsbDevEndp0Size = *(Databuf+7);         //端点0大小
    printf("	\tUSBHS_UsbDevEndp0Size=%02x\n" ,USBHS_UsbDevEndp0Size);
    return ret;
}

/*********************************************************************
 * @fn      USBHS_HubUSBSetAddr
 *
 * @brief   Set Address
 *
 * @param   TBC -- Dennis
 *
 * @return  status
 */
UINT8 USBHS_HubUSBSetAddr( PHUB_Port_Info phub,UINT8 addr,UINT8 *Databuf ,UINT8 hubaddr)
{
    UINT8 ret;
    pSetupReq->bRequestType = 0x00;
    pSetupReq->bRequest = 0x05;
    pSetupReq->wValue = addr;
    pSetupReq->wIndex = 0x0000;
    pSetupReq->wLength = 0x0000;
    ret = USBHS_HubCtrlTransfer( phub,Databuf,NULL , hubaddr);
    if( ret == ERR_SUCCESS ){
        USBHS_CurrentAddr( addr );
        phub->Addr = addr;
    }
    return ret;
}

/*********************************************************************
 * @fn      USBHS_HubUSBHS_Analysis_Descr
 *
 * @brief   Analyse the Descriptor, printf function
 *
 * @param   N/A
 *
 * @return  N/A
 */
void USBHS_HubUSBHS_Analysis_Descr(PHUB_Port_Info portn,PUINT8 pdesc, UINT16 l)
{
    UINT16 i;

    UINT8 endp_num = 0;
    for( i=0; i<l; i++ )                                                //分析描述符
    {
         if((pdesc[i]==0x09)&&(pdesc[i+1]==0x02))
         {
                printf("	bNumInterfaces:%02x \n",pdesc[i+4]);            //配置描述符里的接口数-第5个字节
         }
         if((pdesc[i]==0x09)&&(pdesc[i+1]==0x04))
         {
              printf("	device_type:%02x \n",pdesc[i+5]);                 //接口类型
              portn->DeviceType = pdesc[i+5];
              portn->DeviceHIDType = pdesc[i+7];
         }
         if((pdesc[i]==0x09)&&(pdesc[i+1]==0x21))                       //报表描述符长度
         {
              USBHS_Hid_Report_Len = ((UINT16)pdesc[i+8]<<8)|pdesc[i+7];
              printf("	USBHS_Hid_Report_Len:%02x \n", USBHS_Hid_Report_Len);                 //接口类型
         }
         if((pdesc[i]==0x07)&&(pdesc[i+1]==0x05))
         {
            if((pdesc[i+2])&0x80)
            {
                 printf("	endpIN:%02x \n",pdesc[i+2]&0x0f);              //取in端点号
                 portn->portEndp[ endp_num ].Num = pdesc[i+2];
                 portn->portEndp[ endp_num ].EndpType = pdesc[i+3];
                 if(((pdesc[i+5])&0x18) && (portn->Speed == 0x01) )     //高速高带宽端点
                 {
                     USBHS_EndpnMaxSize = ((UINT16)(pdesc[i+5]&0x07)<<8)|pdesc[i+4]; //取端点大小
                     portn->portEndp[ endp_num ].HighTransNum = ((pdesc[i+5]&0x18)>>3); //  一个微帧内事务数
                 }
                 else {
                     USBHS_EndpnMaxSize = ((UINT16)pdesc[i+5]<<8)|pdesc[i+4];     //取端点大小
                     portn->portEndp[ endp_num ].HighTransNum = 0;
                 }
                 portn->portEndp[ endp_num ].Endp_Size = USBHS_EndpnMaxSize;
                 portn->portEndp[ endp_num ].tog = 0;
                 endp_num++;
                 portn->EndpNum = endp_num;
            }
            else
            {
                printf("	endpOUT:%02x \n",pdesc[i+2]&0x0f);              //取out端点号
                USBHS_EndpnMaxSize = ((UINT16)pdesc[i+5]<<8)|pdesc[i+4];     //取端点大小
                portn->portEndp[ endp_num ].Num = pdesc[i+2];
                portn->portEndp[ endp_num ].EndpType = pdesc[i+3];
                if(((pdesc[i+5])&0x18) && (portn->Speed == 0x01) )     //高速高带宽端点
                {
                    USBHS_EndpnMaxSize = ((UINT16)(pdesc[i+5]&0x07)<<8)|pdesc[i+4]; //取端点大小
                }
                else {
                    USBHS_EndpnMaxSize = ((UINT16)pdesc[i+5]<<8)|pdesc[i+4];     //取端点大小
                }
                portn->portEndp[ endp_num ].Endp_Size = USBHS_EndpnMaxSize;
                portn->portEndp[ endp_num ].tog = 0;
                endp_num++;
                portn->EndpNum = endp_num;
                printf("	Out_endpmaxsize:%02x \n",USBHS_EndpnMaxSize);
            }
        }
  }
}

/*********************************************************************
 * @fn      HubUSBHS_CtrlGetConfigDescr
 *
 * @brief   This function writes a control command to the USB Slave
 *
 * @param   N/A
 *
 * @return  N/A
 */
UINT8 HubUSBHS_CtrlGetConfigDescr( PHUB_Port_Info phub,UINT8 *Databuf ,UINT8 hubaddr)
{
    UINT8  ret;
    UINT16  len;

    pSetupReq->bRequestType = 0x80;
    pSetupReq->bRequest = 0x06;
    pSetupReq->wValue = 0x0200;
    pSetupReq->wIndex = 0x0000;
    pSetupReq->wLength = 8;
    ret = USBHS_HubCtrlTransfer( phub,Databuf,&len , hubaddr);
    if(ret != ERR_SUCCESS)             return  ( ret );
    if(len < ( pSetupReq->wLength ) )  return  ERR_USB_BUF_OVER;
    len = pSetupReq->wLength = ((PUSB_CFG_DESCR)Databuf)-> wTotalLength; //解析全部配置描述符的长度
    printf("\tlen=%02x\n",len);
    ret = USBHS_HubCtrlTransfer( phub,Databuf,&len , hubaddr);
    USBHS_thisUsbDev.DeviceCongValue = ( (PUSB_CFG_DESCR)Databuf )-> bConfigurationValue;
    USBHS_HubUSBHS_Analysis_Descr( phub,(UINT8 *)Databuf, pSetupReq->wLength );
    return ret;
}

/*HID获取报表描述符*/
UINT8 USBHS_HubCtrlGetHidDescr( PHUB_Port_Info phub,UINT8 *Databuf ,UINT8 hubaddr)
{
    UINT8  ret;
    UINT16  len;

    pSetupReq->bRequestType = 0x81;
    pSetupReq->bRequest = 0x06;
    pSetupReq->wValue = 0x2200;
    pSetupReq->wIndex = 0x0001;
    pSetupReq->wLength = USBHS_Hid_Report_Len;
    ret = USBHS_HubCtrlTransfer( phub,Databuf,&len , hubaddr);
    if(ret != ERR_SUCCESS)             return  ( ret );
    if(len < ( pSetupReq->wLength ) )  return  ERR_USB_BUF_OVER;
    return ret;
}

UINT8 USBHS_HubUSBSetIdle( PHUB_Port_Info phub, UINT8 *Databuf ,UINT8 hubaddr)
{
    UINT8 ret;
    pSetupReq->bRequestType = 0x21;
    pSetupReq->bRequest = 0x0a;
    pSetupReq->wValue = 0x00;
    pSetupReq->wIndex = 0x0000;
    pSetupReq->wLength = 0x0000;
    ret = USBHS_HubCtrlTransfer( phub, Databuf, NULL , hubaddr);
    if ( ret != ERR_SUCCESS )         return( 1 );
    return ERR_SUCCESS;
}

UINT8 USBHS_HubUSBSetReport( PHUB_Port_Info phub, UINT8 *Databuf,UINT16 *len ,UINT8 hubaddr)
{
    UINT8 ret;
    pSetupReq->bRequestType = 0x21;
    pSetupReq->bRequest = 0x09;
    pSetupReq->wValue = 0x0200;
    pSetupReq->wIndex = 0x0000;
    pSetupReq->wLength = 0x0001;
    ret = USBHS_HubCtrlTransfer( phub, Databuf, len, hubaddr );
    if ( ret != ERR_SUCCESS )         return( 1 );
    return ERR_SUCCESS;
}


UINT8 USBHS_HubUSBUSBHS_SetConfig( PHUB_Port_Info phub, UINT8 cfg,UINT8 *Databuf ,UINT8 hubaddr)
{
    UINT8 ret;
    pSetupReq->bRequestType = 0x00;
    pSetupReq->bRequest = 0x09;
    pSetupReq->wValue = cfg;
    pSetupReq->wIndex = 0x0000;
    pSetupReq->wLength = 0x0000;
    ret = USBHS_HubCtrlTransfer( phub, Databuf, NULL , hubaddr);
    if ( ret != ERR_SUCCESS )         return( 1 );
    return ERR_SUCCESS;
}

/*********************************************************************
 * @fn      USBOTG_HostEnum
 *
 * @brief   Host enumerated device.
 *
 * @return  Error state
 */
UINT8 USBHS_HostEnum( UINT8 depth, UINT8 *Databuf )
{
	printf("	USBHS_HostEnum: -- Start of Function --\n");
  UINT8 ret,i;

  USBHS_SetBusReset();
  USBHS_CurrentAddr(0x00);
  Delay_Ms(10);
  ret = USBHS_CtrlGetDevDescr( Databuf );						// -- comment by Dennis: device info here
  if( ret != ERR_SUCCESS )													// -- comment by Dennis: program does not reach here
  {
			printf("	USBHS_HostEnum: get device descriptor: %02x\n",ret);
      return( ret );
  }
  //hub_addr = ((PUSB_SETUP_REQ)USBHS_SetAddress)->wValue;

  USBHS_HubInfo[depth].DevAddr = ((PUSB_SETUP_REQ)USBHS_SetAddress)->wValue;           //HUB的地址
	
  ret = USBHS_CtrlUSBHS_SetAddress( ((PUSB_SETUP_REQ)USBHS_SetAddress)->wValue );
  if(ret != ERR_SUCCESS)
  {
			printf("	USBHS_HostEnum: set address:%02x\n",ret);
      return( ret );
  }
  ret = USBHS_CtrlGetConfigDescr(NULL, Databuf );
  if(ret != ERR_SUCCESS)
  {
			printf("	USBHS_HostEnum: get configuration descriptor:%02x\n",ret);
      return( ret );
  }
  ret = USBHS_CtrlSetUsbConfig( USBHS_thisUsbDev.DeviceCongValue );
  if( ret != ERR_SUCCESS )
  {
			printf("	USBHS_HostEnum: set configuration:%02x\n",ret);
      return( ret );
  }

	/* FOR HID */
  if( USBHS_thisUsbDev.DeviceType == 0x03 )
  {
      USBHS_CtrlSetIdle();

      USBHS_CtrlGetReportDescr(Databuf);
  }
	
	/* FOR HUBS */
  else if( USBHS_thisUsbDev.DeviceType == 0x09 )
  {      //HUB
      ret = USBHS_CtrlGet_HUBDevDescr( Databuf );
      if( ret != ERR_SUCCESS )												// -- comment by Dennis: program does not reach here
      {
					printf("	USBHS_HostEnum: get_hub_desc:%02x\n",ret);
          return( ret );
      }
      for( i=0;i!=4;i++ ){														// -- comment by Dennis: This is assuming all hubs are 1-4
          ret = USBHS_HubSetPortFeature( i+1,0x08 );
					printf("	USBHS_HostEnum: hub_power_status=%02x,%d\n",ret,i);
				if( ret != ERR_SUCCESS )return ret;						// -- comment by Dennis: program does not reach here
      }
  }
	printf("	USBHS_HostEnum: -- End of Function --\n");
  return( ERR_SUCCESS );
}

/*********************************************************************
 * @fn      USBHS_CurrentAddr
 *
 * @brief   Current device address.
 *
 * @param   address - Endpoint address.
 *
 * @return  none
 */
void USBHS_CurrentAddr( UINT8 address )
{
    USBHSH->DEV_AD = address;                  // SET ADDRESS
}

/*********************************************************************
 * @fn      Anaylisys_Descr
 *
 * @brief   Descriptor analysis.
 *
 * @param   pusbdev - device information variable.
 *          pdesc - descriptor buffer to analyze
 *          l - length
 *
 * @return  none
 */
void USBHS_Analysis_Descr(pUSBDEV_INFO pusbdev,PUINT8 pdesc, UINT16 l)
{
    UINT16 i,j = 0;
    for( i=0; i<l; i++ )                                                //分析描述符
    {
         if((pdesc[i]==0x09)&&(pdesc[i+1]==0x02))
         {
					 printf("	USBHS_Analysis_Descr: bNumInterfaces:%02x \n",pdesc[i+4]);            //配置描述符里的接口数-第5个字节
         }
         if((pdesc[i]==0x09)&&(pdesc[i+1]==0x04))
         {
					 printf("	USBHS_Analysis_Descr: device_type:%02x \n",pdesc[i+5]);               //接口类型
             pusbdev->DeviceType = pdesc[i+5];
         }
         if((pdesc[i]==0x09)&&(pdesc[i+1]==0x21))                       									//报表描述符长度
         {
              USBHS_Hid_Report_Len = ((UINT16)pdesc[i+8]<<8)|pdesc[i+7];
              printf("	USBHS_Hid_Report_Len: %02x \n",USBHS_Hid_Report_Len);             //接口类型
         }
         if((pdesc[i]==0x07)&&(pdesc[i+1]==0x05))
         {
            if((pdesc[i+2])&0x80)
            {
								 printf("	USBHS_Analysis_Descr: endpIN:%02x \n",pdesc[i+2]&0x0f);         //取in端点号
                 pusbdev->DevEndp.InEndpNum[j++] = pdesc[i+2]&0x0f;
                 pusbdev->DevEndp.InEndpCount++;
                 USBHS_EndpnMaxSize = ((UINT16)pdesc[i+5]<<8)|pdesc[i+4];     						//取端点大小
                 pusbdev->DevEndp.InEndpMaxSize = USBHS_EndpnMaxSize;

                 pusbdev->DevEndp.Ininterval = pdesc[i+6];

								printf("	USBHS_Analysis_Descr: In_endpmaxsize:%02x \n",USBHS_EndpnMaxSize);
            }
            else
            {
                printf("endpOUT:%02x \n",pdesc[i+2]&0x0f);             					 					//取out端点号
                pusbdev->DevEndp.OutEndpNum = pdesc[i+2]&0x0f;
                pusbdev->DevEndp.OutEndpCount++;
                USBHS_EndpnMaxSize =((UINT16)pdesc[i+5]<<8)|pdesc[i+4];        						//取端点大小
                pusbdev->DevEndp.OutEndpMaxSize = USBHS_EndpnMaxSize;
                printf("Out_endpmaxsize:%02x \n",USBHS_EndpnMaxSize);
            }
        }
  }
}

UINT8 USBHS_HUBCheckPortConnect( UINT8 depth, UINT8 port ,UINT8 dev_exist)
{
    UINT8 ret;
    ret = USBHS_HubGetPortStatus( depth,port+1 );
    if( ret != ERR_SUCCESS )return ret;
    /* 判断当前端口连接状态 */
    if( USBHS_endpRXbuf[ 2 ] & 0x01 )                                                      /* 该端口连接状态发生改变 */
    {
        if( USBHS_endpRXbuf[ 0 ] & 0x01 )
        {
            if( USBHS_HubInfo[depth].portD[port].Status < HUB_ERR_SCUESS )
            {
                USBHS_HubInfo[depth].portD[port].Status = HUB_ERR_CONNECT;
            }
            return( 0x18 );                                    /* 该端口：检测到设备连接 */
        }
        else
        {
            USBHS_HubInfo[depth].portD[port].Status = HUB_ERR_DISCONNECT;
            USBHS_HubInfo[depth].portD[port].Status = 0;
            return( 0x19 );                                 /* 该端口：检测到设备断开 */
        }
    }
    else                                                                        /* 该端口连接状态未发生改变 */
    {
        if( ((USBHS_endpRXbuf[ 0 ] & 0x01) && dev_exist == 0) || (USBHS_endpRXbuf[ 0 ] == 0x03) )
        {
            if( USBHS_HubInfo[depth].portD[port].Status < HUB_ERR_SCUESS )
            {
                USBHS_HubInfo[depth].portD[port].Status = HUB_ERR_CONNECT;
            }
            return( 0x02 );                                                     /* 该端口：有设备 */
        }
        else
        {
            USBHS_HubInfo[depth].portD[port].Status = HUB_ERR_DISCONNECT;
            USBHS_HubInfo[depth].portD[port].Status = 0;
            return( 0x01 );                                                     /* 该端口：无设备 */
        }
    }
}

//开始分离处理
uint8_t USBHS_SSPLITPacket( PHUB_Port_Info portn, uint8_t *pbuf, uint16_t *plen, uint8_t endp_pid, uint8_t tog ,UINT8 hubaddr)
{
    uint32_t  i;
    uint16_t   r, type;

    if((endp_pid&0x0f)) type = portn->portEndp[USBHS_gEndp_Num].EndpType;
    else  type = 0;

    R16_UH_SPLIT_DATA = (type<<10)|(portn->Num<<1)|( portn->Speed ? 0x100 : 0 );
    R8_USB_DEV_AD = hubaddr;
    R8_USB_INT_FG = RB_UIF_TRANSFER;
    for ( i = 500; i != 0; i -- ){
        r = ( R16_USB_FRAME_NO>>13 );
        if( r == 1 )    break;
        Delay_Us( 2 );
    }
    if ( i == 0 )    return( ERR_USB_TRANSFER );
    while(!(R8_USB_MIS_ST & RB_UMS_SPLIT_CAN));
    R8_UH_EP_PID = (USB_PID_SPLIT<<4)|(endp_pid&0x0F);      // SSPLIT
    for ( i = WAIT_USB_TOUT_200US; i != 0 && (R8_USB_INT_FG & RB_UIF_TRANSFER) == 0 ; i -- );//  Delay_Us( 1 );
    R8_UH_EP_PID = 0x00;
    if ( (R8_USB_INT_FG & RB_UIF_TRANSFER) == 0 )    return( ERR_USB_TRANSFER );
    /* pid-data包    */
    if( (endp_pid>>4) != USB_PID_IN ){
        R8_UH_TX_CTRL = (tog<<3);           //有数据和应答 OUT/SETUP
        R32_UH_TX_DMA = (UINT32)pbuf;
        R16_UH_TX_LEN = *plen;
    }
    else
    {                                                    // IN
        R8_UH_RX_CTRL = RB_UH_R_NODATA | (tog<<3) | RB_UH_R_NORES;     //没数据,无应答
    }
    R8_USB_DEV_AD = portn->Addr;
    R8_UH_EP_PID = endp_pid;                                // SETUP/OUT/IN
    R8_USB_INT_FG = RB_UIF_TRANSFER;
    for ( i = WAIT_USB_TOUT_200US; i != 0 && (R8_USB_INT_FG & RB_UIF_TRANSFER) == 0 ; i -- )  Delay_Us( 1 );
    R8_UH_EP_PID = 0x00;
    if ( R8_USB_INT_FG & RB_UIF_TRANSFER ){
        r = R8_USB_INT_ST & MASK_UIS_H_RES;
        if ( r == USB_PID_ACK )                    return( ERR_SUCCESS );       // hub no space
        if( type == 3 && r == USB_PID_NULL )       return( ERR_SUCCESS );
    }
    return( ERR_USB_TRANSFER );
}

uint8_t USBHS_CSPLITPacket( PHUB_Port_Info portn, uint8_t *pbuf, uint16_t *plen, uint8_t endp_pid, uint8_t tog ,UINT8 hubaddr)
{
    uint32_t  i;
    uint16_t   r, type;
    uint8_t   TransRetry=0;

    if((endp_pid&0x0f)) type = portn->portEndp[USBHS_gEndp_Num].EndpType;
    else
        type = 0;
    /* CSPLIT+PID   */
    do{
        R16_UH_SPLIT_DATA = (type<<10)|(portn->Num<<1)|( portn->Speed ? 0x101 : 0x001 );

        R8_USB_DEV_AD = hubaddr;

        R8_USB_INT_FG = RB_UIF_TRANSFER;
        while(!(R8_USB_MIS_ST & RB_UMS_SPLIT_CAN));
        R8_UH_EP_PID = (USB_PID_SPLIT<<4)|(endp_pid&0x0F);      // SSPLIT
        for ( i = WAIT_USB_TOUT_200US; i != 0 && (R8_USB_INT_FG & RB_UIF_TRANSFER) == 0 ; i -- ) ;// Delay_Us( 1 );
        R8_UH_EP_PID = 0x00;
        if ( (R8_USB_INT_FG & RB_UIF_TRANSFER) == 0 )          return( ERR_USB_TRANSFER );

        /* pid-data包  */
        if((endp_pid>>4) == USB_PID_IN){
            R8_UH_RX_CTRL = (tog<<3)| RB_UH_R_NORES;  //期待数据 IN 无应答
            R32_UH_RX_DMA = (UINT32)pbuf;
        }
        else{
            R8_UH_TX_CTRL = (tog<<3)|RB_UH_T_NODATA;    //无数据 OUT 期望应答
        }
        R8_USB_DEV_AD = portn->Addr;
        R8_USB_INT_FG = RB_UIF_TRANSFER;
        R8_UH_EP_PID = endp_pid;                                // SETUP/OUT/IN
        for ( i = WAIT_USB_TOUT_200US; i != 0 && (R8_USB_INT_FG & RB_UIF_TRANSFER) == 0 ; i -- )  Delay_Us( 1 );
        R8_UH_EP_PID = 0x00;
        if ( R8_USB_INT_FG & RB_UIF_TRANSFER ){
            r = R8_USB_INT_ST & MASK_UIS_H_RES;
            if( (R8_USB_INT_ST&RB_UIS_TOG_OK) && ((endp_pid>>4)==USB_PID_IN)){
                *plen = R16_USBHS_RX_LEN;
                return( ERR_SUCCESS );
            }
            if ( r == USB_PID_ACK )                    return( ERR_SUCCESS );
            else if ( r == USB_PID_NYET )              ;
            else                                       return( r | ERR_USB_TRANSFER );
        }
        Delay_Us( 20 );
    }while( ++TransRetry<100 );
    return( ERR_USB_TRANSFER );
}

UINT8 USBHS_BULK_Send_Data( PHUB_Port_Info phub,UINT8 num_port,UINT8 *pdata,UINT16 *len,UINT8 endp,UINT8 *tog ,UINT8 hubaddr)
{

    UINT8 s;
    UINT16 temp,time_out;
    time_out = 0;
    while(1){
        if( *len >= phub->portEndp[num_port].Endp_Size ){
            temp =  phub->portEndp[num_port].Endp_Size;
            *len -=  phub->portEndp[num_port].Endp_Size;
        }
        else{
            temp = *len;
            *len = 0;
        }
        if( phub->Speed == 0x01 ){      //高速单独添加
            USBHSH->HOST_TX_DMA = (UINT32)pdata;
            USBHSH->HOST_TX_LEN = temp;

            s = USBHS_USBHostTransact((USB_PID_OUT<<4)|(endp&0x0f),  *tog<<3,  20000);
            if(s != ERR_SUCCESS)  return  ( s );
            *tog ^=1;
            pdata+=temp;
            if( *len==0 )break;
        }
        else{
            USBHS_gEndp_Num = num_port;
            s = USBHS_SSPLITPacket(phub, pdata, &temp, ((USB_PID_OUT << 4) | (endp&0x0f)), *tog , hubaddr);
            if( s != ERR_SUCCESS )  return s;
            s = USBHS_CSPLITPacket(phub, NULL, NULL, ((USB_PID_OUT << 4) | (endp&0x0f)), *tog, hubaddr);
            time_out++;
            if( time_out>=100 )return s;       //重试100次发送失败则退出
            if( s == HUB_CS_NAK )continue;      //NAK需要重试
            if( s != ERR_SUCCESS )  return s;
            *tog^=0x01;
            pdata+=temp;
            if( *len==0 )break;
        }
    }
    return s;
}

UINT8 USBHS_BULK_In_Data( PHUB_Port_Info phub,UINT8 num_port,UINT8 *pdata,UINT16 *len,UINT8 endp,UINT8 *tog ,UINT8 hubaddr)
{

    UINT8 s;
    UINT16 time_out = 0;
    while(1){
        if( phub->Speed == 0x01 ){      //高速单独添加
            USBHS_CurrentAddr(phub->Addr);
            USBHSH->HOST_RX_DMA = (UINT32)pdata;
            s = USBHS_USBHostTransact( USB_PID_IN << 4 | (endp&0x0f) , *tog<<3, 20000 );
            if(s != ERR_SUCCESS) return ( s );
            *tog ^= 0x01;
            *len = USBHSH->RX_LEN ;
            return s;
        }
        else{
            USBHS_gEndp_Num = num_port;
            s = USBHS_SSPLITPacket(phub, pdata, len, USB_PID_IN << 4 | (endp&0x0f), *tog , hubaddr);
            if( s != ERR_SUCCESS )  return s;
            s = USBHS_CSPLITPacket(phub, pdata, len, USB_PID_IN << 4 | (endp&0x0f), *tog, hubaddr);
            time_out++;
            if( time_out>=100 )return s;       //重试100次发送失败则退出
            if( s == HUB_CS_NAK )continue;
            if( s != ERR_SUCCESS )  return s;
            *tog ^= 0x01;
            return s;
        }
    }
}

/*******************************************************************************
 * @fn        USBHS_INT_In_Data
 *
* @briefI    	Performs an Interrupt IN transfer on a USB endpoint
 *
 * @param     PHUB_Port_Info phub      // info about the device on the hub port
							UINT8 num_port           // endpoint index in your endpoint table
							UINT8 *pdata             // buffer where data is stored
							UINT16 *len              // length returned by hardware
							UINT8 endp               // endpoint number (0x81 for IN EP1, 0x01 for OUT EP1)
							UINT8 *tog               // DATA0/DATA1 toggle
							UINT8 hubaddr            // hub address

 *
 * @return    status
 */
UINT8 USBHS_INT_In_Data( PHUB_Port_Info phub, UINT8 num_port, UINT8 *pdata, UINT16 *len, UINT8 endp, UINT8 *tog, UINT8 hubaddr)
{

    UINT8 s;
    UINT16 time_out;
    if( phub->Speed == 0x01 )				// 高速设备
    {                               
				//printf(" High Speed USB Device with %02x of endpoints\n" ,USBHS_gEndp_Num);
        USBHS_CurrentAddr(phub->Addr);
        USBHSH->HOST_RX_DMA = (UINT32)pdata;
        s = USBHS_USBHostTransact( USB_PID_IN << 4 | (endp&0x0f) , *tog<<3, 0 );
        if(s != ERR_SUCCESS) return ( s );
        *len = USBHSH->RX_LEN ;
    }
    else
    {                               // 低全速设备
        USBHS_gEndp_Num = num_port;
				//printf(" Low/Normal Speed USB Device with %02x of endpoints\n" ,USBHS_gEndp_Num);
			
				// Low/full-speed through a hub requires split transactions:
        s = USBHS_SSPLITPacket(phub, pdata, len, USB_PID_IN << 4 | (endp&0x0f), *tog , hubaddr);			// Start Split 
        if( s != ERR_SUCCESS )  return s;
			
        s = USBHS_CSPLITPacket(phub, pdata, len, USB_PID_IN << 4 | (endp&0x0f), *tog, hubaddr);				// Complete Split
        time_out++;
			
        if( s != ERR_SUCCESS )  return s;
    }
    return s;
}

//开始分离处理
uint8_t ISO_USBHS_SSPLITPacket( PHUB_Port_Info portn, uint8_t *pbuf, uint16_t *plen, uint8_t endp_pid, UINT16 iso_type )
{
    uint32_t  i;
    uint8_t   r, type;

    if((endp_pid&0x0f)) type = portn->portEndp[USBHS_gEndp_Num].EndpType;
    else type = 0;
    /* SSPLIT+PID   */
    while(!(R8_USB_MIS_ST & RB_UMS_SPLIT_CAN));
    if( endp_pid>>4 == USB_PID_IN )
    {
        R16_UH_SPLIT_DATA = (type<<10)|(portn->Num<<1) | 0x100 ;
    }
    else
    {
        R16_UH_SPLIT_DATA = (type<<10)|(portn->Num<<1) | iso_type;
    }
    R8_USB_DEV_AD = USBHS_HubInfo[USBHS_glable_index].DevAddr;
    R8_USB_INT_FG = RB_UIF_TRANSFER;
    while(!(R8_USB_MIS_ST & RB_UMS_SPLIT_CAN));
    R8_UH_EP_PID = (USB_PID_SPLIT<<4)|(endp_pid&0x0F);      // SSPLIT
    for ( i = WAIT_USB_TOUT_200US; i != 0 && (R8_USB_INT_FG & RB_UIF_TRANSFER) == 0 ; i -- )  Delay_Us( 1 );
    R8_UH_EP_PID = 0x00;
    if ( (R8_USB_INT_FG & RB_UIF_TRANSFER) == 0 ) { return( ERR_USB_TRANSFER );}

    /* pid-data包    */
    if( (endp_pid>>4) != USB_PID_IN ){
        R32_UH_EP_TYPE |= ENDP3_TX_ISO;
        R8_UH_TX_CTRL = (0<<3) | RB_UH_T_NORES;           //有数据和应答 OUT/SETUP
        R32_UH_TX_DMA = (UINT32)pbuf;
        R16_UH_TX_LEN = *plen;
    }
    else{                                                    // IN
        R8_UH_RX_CTRL = RB_UH_R_NODATA | (0<<3) | RB_UH_R_NORES;     //没数据,无应答
    }
    R8_USB_DEV_AD = portn->Addr;
    R8_USB_INT_FG = RB_UIF_TRANSFER;
    R8_UH_EP_PID = endp_pid;                                // SETUP/OUT/IN
    for ( i = WAIT_USB_TOUT_200US; i != 0 && (R8_USB_INT_FG & RB_UIF_TRANSFER) == 0 ; i -- )  Delay_Us( 1 );
    R8_UH_EP_PID = 0x00;
    if ( R8_USB_INT_FG & RB_UIF_TRANSFER ){
        r = R8_USB_INT_ST & MASK_UIS_H_RES;
        if( ((endp_pid>>4)==USB_PID_OUT) && (r == USB_PID_DATA0) ) return( ERR_SUCCESS );
    }
    return( ERR_USB_TRANSFER );
}

uint8_t ISO_USBHS_CSPLITPacket( PHUB_Port_Info portn, uint8_t *pbuf, uint16_t *plen, uint8_t endp_pid )
{
    uint32_t  i;
    uint8_t   r, type;
    uint8_t   TransRetry=0;

    if((endp_pid&0x0f)) type = portn->portEndp[USBHS_gEndp_Num].EndpType;
    else type = 0;
    /* CSPLIT+PID   */
    do{
        R16_UH_SPLIT_DATA = (type<<10)|(portn->Num<<1)|0x101 ;
        R8_USB_DEV_AD = USBHS_HubInfo[USBHS_glable_index].DevAddr;
        R8_USB_INT_FG = RB_UIF_TRANSFER;
        while(!(R8_USB_MIS_ST & RB_UMS_SOF_PRES));
        R8_UH_EP_PID = (USB_PID_SPLIT<<4)|(endp_pid&0x0F);      // SSPLIT
        for ( i = WAIT_USB_TOUT_200US; i != 0 && (R8_USB_INT_FG & RB_UIF_TRANSFER) == 0 ; i -- )  Delay_Us( 1 );
        R8_UH_EP_PID = 0x00;
        if ( (R8_USB_INT_FG & RB_UIF_TRANSFER) == 0 )          return( ERR_USB_TRANSFER );

        /***********  pid-data包    */
        if((endp_pid>>4) == USB_PID_IN){
            R32_UH_EP_TYPE |= RB_UH_EP_RX_TYPE;
            R8_UH_RX_CTRL = (3<<3)| RB_UH_R_NORES;  //期待数据 IN 无应答
            R32_UH_RX_DMA = (UINT32)pbuf;
        }
        else{
            R8_UH_TX_CTRL = RB_UH_T_NODATA;    //无数据 OUT 期望应答
        }
        R8_USB_DEV_AD = portn->Addr;
        R8_USB_INT_FG = RB_UIF_TRANSFER;
        R8_UH_EP_PID = endp_pid;                                // SETUP/OUT/IN
        for ( i = WAIT_USB_TOUT_200US; i != 0 && (R8_USB_INT_FG & RB_UIF_TRANSFER) == 0 ; i -- )  Delay_Us( 1 );
        R8_UH_EP_PID = 0x00;
        if ( R8_USB_INT_FG & RB_UIF_TRANSFER ){
            r = R8_USB_INT_ST & MASK_UIS_H_RES;
            if( ((endp_pid>>4)==USB_PID_IN) && ( (r == USB_PID_DATA0) || (r == USB_PID_MDATA) ) ){
                *plen = R16_USBHS_RX_LEN;
                if( r == USB_PID_DATA0 )*plen |= 0x8000;        //表示结束
                return( ERR_SUCCESS );
            }
            else if ( r == USB_PID_NYET );
            else      return( r | ERR_USB_TRANSFER );
        }
        Delay_Us( 20 );
    }while( ++TransRetry<30 );
    return( ERR_USB_TRANSFER );
}

UINT8 USBHS_USBHostTransactISO(UINT8 endp_pid, UINT16 *plen, UINT8 tog)
{
    uint32_t  i;
    uint8_t   r;
    uint8_t   TransRetry=0;

    do{
        if( (endp_pid>>4) != USB_PID_IN ){                       // OUT
            R32_UH_EP_TYPE |= ENDP3_TX_ISO;
            R8_UH_TX_CTRL = (tog) | RB_UH_T_NORES;           //有数据,无应答
        }
        else{                                                    // IN
            R32_UH_EP_TYPE |= RB_UH_EP_RX_TYPE;
            R8_UH_RX_CTRL =  (2<<3) | RB_UH_R_NORES;         //有数据,无应答
        }
        R8_USB_INT_FG = RB_UIF_TRANSFER;
        R8_UH_EP_PID = endp_pid;                                // SETUP/OUT/IN
        for ( i = WAIT_USB_TOUT_200US; i != 0 && (R8_USB_INT_FG & RB_UIF_TRANSFER) == 0 ; i -- )  Delay_Us( 1 );
        R8_UH_EP_PID = 0x00;
        if ( R8_USB_INT_FG & RB_UIF_TRANSFER ){
            r = R8_USB_INT_ST & MASK_UIS_H_RES;
            if( ((endp_pid>>4)==USB_PID_IN) && ( (r == USB_PID_DATA2) || (r == USB_PID_DATA1)|| (r == USB_PID_DATA0) ) ){
                *plen = R16_USBHS_RX_LEN;
                if( r == USB_PID_DATA0 )*plen |= 0x8000;        //表示结束
                return( ERR_SUCCESS );
            }
            else if ( (endp_pid>>4)==USB_PID_OUT ) return( ERR_SUCCESS );
            else if ( r == USB_PID_NYET );
            else return( r | ERR_USB_TRANSFER );
        }
        Delay_Us( 20 );
    }while( ++TransRetry<30 );
    return( ERR_USB_TRANSFER );
}

UINT8 USBHS_ISO_Send_Data( PHUB_Port_Info phub,UINT8 *pdata,UINT16 len,UINT8 endp )
{
    UINT8  ret;
    UINT16 count = len;
    UINT16 posoffset = 0;
    UINT16 k;
    UINT16 EndpMaxSize = phub->portEndp[USBHS_gEndp_Num].Endp_Size;

    if( phub->Speed == 0x01 ){          //高速设备  //需要添加
        USBHS_CurrentAddr(phub->Addr);
        USBHSH->HOST_TX_DMA = (UINT32)pdata;

        while( ((R16_USB_FRAME_NO>>13)& 0x7) != 7);        // wait frist sof in frame
        while( !(R8_USB_INT_FG & RB_UIF_HST_SOF));
        if( count <= EndpMaxSize ){               //1个微帧内1个事务数
            USBHSH->HOST_TX_LEN = count;
            ret = USBHS_USBHostTransactISO( endp, NULL, RB_UH_T_TOG0 );   //第1个事务
        }
        else if( count <= EndpMaxSize * 3 ){
            USBHSH->HOST_TX_LEN = EndpMaxSize;
            ret = USBHS_USBHostTransactISO( endp, NULL, RB_UH_T_TOGM );   //第1个事务
            if( ret != ERR_SUCCESS) return( ERR_USB_TRANSFER );

            if( count <= EndpMaxSize * 2 ){       //1个微帧内2个事务数
                USBHSH->HOST_TX_DMA = (UINT32)(pdata+EndpMaxSize);
                USBHSH->HOST_TX_LEN = (count - EndpMaxSize);
                ret = USBHS_USBHostTransactISO( endp, NULL, RB_UH_T_TOG1 ); //第2个事务
            }
            else {                                                          //1个微帧内3个事务数
                USBHSH->HOST_TX_DMA = (UINT32)(pdata+EndpMaxSize);
                USBHSH->HOST_TX_LEN = EndpMaxSize;
                ret = USBHS_USBHostTransactISO( endp, NULL, RB_UH_T_TOGM );   //第2个事务
                if( ret != ERR_SUCCESS) return( ERR_USB_TRANSFER );

                USBHSH->HOST_TX_DMA = (UINT32)(pdata+EndpMaxSize*2);
                USBHSH->HOST_TX_LEN = (count - EndpMaxSize*2);
                ret = USBHS_USBHostTransactISO( endp, NULL, RB_UH_T_TOG2 );   //第3个事务
            }
        }
        else return( ERR_USB_TRANSFER );
    }
    else {
        while( ((R16_USB_FRAME_NO>>13)& 0x7) != 7);        // wait frist sof in frame
        if( count <= 188 ){
            while( !(R8_USB_INT_FG & RB_UIF_HST_SOF));
            R8_USB_INT_FG = RB_UIF_HST_SOF;
            ret = ISO_USBHS_SSPLITPacket( phub,pdata,&count,endp,SS_ALL );
        }
        else{
            while( !(R8_USB_INT_FG & RB_UIF_HST_SOF));
            R8_USB_INT_FG = RB_UIF_HST_SOF;
            k = 188;
            ret = ISO_USBHS_SSPLITPacket( phub,pdata,&k,endp,SS_BEGIN );//begin
            posoffset += k;
            count = count - 188;
            for( ; count > 188; )
            {
                while( !(R8_USB_INT_FG & RB_UIF_HST_SOF));
                R8_USB_INT_FG = RB_UIF_HST_SOF;
                ret = ISO_USBHS_SSPLITPacket( phub,pdata+posoffset,&k,endp,SS_MIDDLE );//middle
                posoffset += k;
                count = count - 188;
            }
            while( !(R8_USB_INT_FG & RB_UIF_HST_SOF));
            R8_USB_INT_FG = RB_UIF_HST_SOF;
            ret = ISO_USBHS_SSPLITPacket( phub,pdata+posoffset,&count,endp,SS_END );//end
        }
    }
    return ret;
}

UINT8 USBHS_ISO_Rec_Data( PHUB_Port_Info phub,UINT8 *pdata,UINT16 *len,UINT8 endp )
{
    UINT16 ret,temp,temp1;
    UINT16 recvlen=0;
    if( phub->Speed == 0x01 ){          //高速设备
        *len = 0;
        USBHS_CurrentAddr(phub->Addr);
        while( ((R16_USB_FRAME_NO>>13)& 0x7) != 7);        // wait frist sof in frame
        while( !(R8_USB_INT_FG & RB_UIF_HST_SOF));
        USBHSH->HOST_RX_DMA = (UINT32)USBHS_endpRXbuf;
        ret = USBHS_USBHostTransactISO( endp, &temp1, 0 );       //一个微帧最多三个事务
        if( ret == ERR_SUCCESS ){
            recvlen = temp1&0x7fff;
            memcpy( pdata, USBHS_endpRXbuf, recvlen );
            *len+=temp1&0x7fff;
            if( temp1&0x8000 )return ERR_SUCCESS;
            if( phub->portEndp[USBHS_gEndp_Num].HighTransNum == 0x00 )return( ERR_USB_TRANSFER ); //如果是1个事务且数据包不为DATA0，返回错误
        }
        else return ret;
        USBHSH->HOST_RX_DMA = (UINT32)USBHS_endpRXbuf;
        ret = USBHS_USBHostTransactISO( endp, &temp1, 0 );
        if( ret == ERR_SUCCESS){
            recvlen = temp1&0x7fff;
            memcpy( pdata+(*len), USBHS_endpRXbuf, recvlen );
            *len+=temp1&0x7fff;
            if( temp1&0x8000 )return ERR_SUCCESS;
            if( phub->portEndp[USBHS_gEndp_Num].HighTransNum == 0x01 )return( ERR_USB_TRANSFER );
        }
        else return ret;
        USBHSH->HOST_RX_DMA = (UINT32)USBHS_endpRXbuf;
        ret = USBHS_USBHostTransactISO( endp, &temp1, 0 );
        if( ret == ERR_SUCCESS ){
            recvlen = temp1&0x7fff;
            memcpy( pdata+(*len), USBHS_endpRXbuf, recvlen );
            *len+=temp1&0x7fff;
            if( temp1&0x8000 )return ERR_SUCCESS;
            if( phub->portEndp[USBHS_gEndp_Num].HighTransNum == 0x01 )return( ERR_USB_TRANSFER );
        }
        else return ret;
    }
    else{
        *len = 0;
        while( ((R16_USB_FRAME_NO>>13)& 0x7) != 7);        // wait frist sof in frame
        while( !(R8_USB_INT_FG & RB_UIF_HST_SOF));
        ret = ISO_USBHS_SSPLITPacket( phub,USBHS_endpRXbuf,NULL,endp,0 );
        while(!(R8_USB_MIS_ST & RB_UMS_SPLIT_CAN));
        ret = ISO_USBHS_CSPLITPacket(phub,USBHS_endpRXbuf,&temp,endp);      //最多IN--6包数据
        if( ret == ERR_SUCCESS ){
            recvlen = temp&0x7fff;
            memcpy( pdata, USBHS_endpRXbuf, recvlen );
            *len+=temp&0x7fff;
            if( temp&0x8000 )return ERR_SUCCESS;
        }
        else return ret;
        ret = ISO_USBHS_CSPLITPacket(phub,USBHS_endpRXbuf,&temp,endp);
        if( ret == ERR_SUCCESS ){
            recvlen = temp&0x7fff;
            memcpy( pdata+(*len), USBHS_endpRXbuf, recvlen );
            *len+=temp&0x7fff;                              //如果为DATA0则表示结束。直接退出
            if( temp&0x8000 )return ERR_SUCCESS;
        }
        else return ret;
        ret = ISO_USBHS_CSPLITPacket(phub,USBHS_endpRXbuf,&temp,endp);
        if( ret == ERR_SUCCESS ){
            recvlen = temp&0x7fff;
            memcpy( pdata+(*len), USBHS_endpRXbuf, recvlen );
            *len+=temp&0x7fff;
            if( temp&0x8000 )return ERR_SUCCESS;
        }
        else return ret;
        ret = ISO_USBHS_CSPLITPacket(phub,USBHS_endpRXbuf,&temp,endp);
        if( ret == ERR_SUCCESS ){
            recvlen = temp&0x7fff;
            memcpy( pdata+(*len), USBHS_endpRXbuf, recvlen );
            *len+=temp&0x7fff;
            if( temp&0x8000 )return ERR_SUCCESS;
        }
        else return ret;
        ret = ISO_USBHS_CSPLITPacket(phub,USBHS_endpRXbuf,&temp,endp);
        if( ret == ERR_SUCCESS ){
            recvlen = temp&0x7fff;
            memcpy( pdata+(*len), USBHS_endpRXbuf, recvlen );
            *len+=temp&0x7fff;
            if( temp&0x8000 )return ERR_SUCCESS;
        }
        else return ret;
        ret = ISO_USBHS_CSPLITPacket(phub,USBHS_endpRXbuf,&temp,endp);
        if( ret == ERR_SUCCESS ){
            recvlen = temp&0x7fff;
            memcpy( pdata+(*len), USBHS_endpRXbuf, recvlen );
            *len+=temp&0x7fff;
            if( temp&0x8000 )return ERR_SUCCESS;
        }
        else return ret;
    }
}

/*********************************************************************
 * @fn      USBHS_HUBHostEnum
 *
 * @brief   Enumerates a USB Device Connected to a HUB port
 *
 * @param   See Parameters
 *
 * @return  Status
 */
UINT8 USBHS_HUBHostEnum( u8 depth, UINT8 *Databuf, UINT8 port , UINT8 uplevelport, UINT8 hubaddr)
{
  UINT8 ret;
  UINT16 i;
  u16 temp16;																																																			// 16-bit unsigned integer
  Delay_Ms( 10 );
	
	/* Reads Port Status */
  ret= USBHS_HubGetPortStatus( depth, port+1 );
  if( ret != ERR_SUCCESS )
  {
      return( ret );
  }
	
	/* Powers the port, 0x04 to PORT_POWER */
  ret = USBHS_HubSetPortFeature( port + 1,0x04 );       																													//复位下面的设备
  if( ret != ERR_SUCCESS )
  {
      return( ret );
  }
	
  /* 查询当前复位端口,直到复位完成,把完成后的状态显示出来 */
  do
  {
      Delay_Ms( 10 );
      ret = USBHS_HubGetPortStatus(  depth, port + 1 );
      if( ret != ERR_SUCCESS )
      {
          return( ret );
      }
  }while( USBHS_endpRXbuf[ 0 ] & 0x10 );																																					// this condition checks if the reset is complete (REFER to PAGE 427 of Universal Serial Bus Config)
  Delay_Ms( 30 );																																																	// BIT4 corresponds to PORTT_RESET, and when 0, the reset is finished and loop exits

  ret = USBHS_HubClearPortFeature( depth, port+1);
  if( ret != ERR_SUCCESS )
  {
      return( ret );
  }
  ret= USBHS_HubGetPortStatus( depth, port+1 );
  if( ret != ERR_SUCCESS )
  {
      return( ret );
  }
	
	/* Displays the Port Status and Port Change Field */
	printf("	USBHS_HUBHostEnum: Display Port Status Field Bits \n");																								// eg. 0x03 0x03 0x10 0x00 -> First 2 bytes 0x03 0x03 = 0b0000_0011_0000_0011 -> bit 0: 1 (Device Connected), bit 1: 1 (Port Enabled)
	printf("	");																																																		// eg. bit 4: 0 (Reset signaling not asserted), bit 8: 1 (Port not in the Powered-off state), bit 9: 1 (Low Speed Device)
  for( ret=0;ret!=4;ret++ ){
      printf("%02x ",USBHS_endpRXbuf[ret]);
  }
  printf("\n");

  if( ( USBHS_endpRXbuf[ 0 ] & 0x01 ) == 0x00 )																																		// Returns 0x18 if No Device is Connected (Port Connection: 0)
  {
      return( 0x18 );
  }
	
	// UINT8 Status;      // 0-disconnect  	1-connect  	2-config
  // UINT8 Speed;       // 0-fullspeed 		1-highspeed 2-lowspeed
  temp16 = *((uint16_t *) &USBHS_endpRXbuf[0]);
  if( temp16 & 0x01 ){              																																						// 低全速
      USBHS_HubInfo[depth].portD[port].Num = port+1;
      USBHS_HubInfo[depth].portD[port].Addr = 0;
      USBHS_HubInfo[depth].portD[port].Status = 1;
      USBHS_HubInfo[depth].portD[port].Speed = (temp16&(1<<9))? 2:((temp16&(1<<10))?1:0);												// 1--高速，2--低速，0--全速 
  }
  else USBHS_HubInfo[depth].portD[port].Status = 0;            																									// 高速
	
	/* Prints Regardless of USB Speed */
  printf("	USBHS_HUBHostEnum: Speed=%02x, %02x\n", USBHS_HubInfo[depth].portD[port].Speed, USBHS_HubInfo[depth].portD[port].Status);
  USBHS_CurrentAddr(0);																																													// 先设置地址0

  if( USBHS_HubInfo[depth].portD[port].Speed == 1){                																						  // 表示是高速 					- For HIGH SPEED 
      ret = USBHS_CtrlGetDevDescr( Databuf );
      if( ret != ERR_SUCCESS )
      {
          printf("get device descriptor:%02x\n",ret);
          return( ret );
      }
      USBHS_HubInfo[depth].portD[port].Addr = USB_SetUSBHS_AddressNumber();
      ret = USBHS_CtrlUSBHS_SetAddress( USBHS_HubInfo[depth].portD[port].Addr );           //设置地址
      if(ret != ERR_SUCCESS)
      {
          printf("set address:%02x,%d\n",ret,port+5);
          return( ret );
      }
      ret = USBHS_CtrlGetConfigDescr( &USBHS_HubInfo[depth].portD[port],Databuf );                          //高速也要进行数据分析
      if(ret != ERR_SUCCESS)
      {
          printf("get configuration descriptor:%02x\n",ret);
          return( ret );
      }
      ret = USBHS_CtrlSetUsbConfig( USBHS_thisUsbDev.DeviceCongValue );
      if( ret != ERR_SUCCESS )
      {
          printf("set configuration:%02x\n",ret);
          return( ret );
      }

      if( USBHS_HubInfo[depth].portD[port].DeviceType == 0x03 )//hid
      {
          USBHS_CtrlSetIdle();

          USBHS_CtrlGetReportDescr(Databuf);
      }
      if( USBHS_HubInfo[depth].portD[port].DeviceType == 0x09 )
      {         //HUB的初始化
          ret = USBHS_CtrlGet_HUBDevDescr( Databuf );
          if( ret != ERR_SUCCESS )
          {
              printf("get_hub_desc:%02x\n",ret);
              return( ret );
          }
          for( i=0;i!=4;i++ )
          {
              ret = USBHS_HubSetPortFeature( i+1,0x08 );
              printf("hub_power_status=%02x,%d\n",ret,i);
              if( ret != ERR_SUCCESS )return ret;
          }
          USBHS_HubInfo[depth+1].DevAddr = USBHS_HubInfo[depth].portD[port].Addr;   //下级HUB的根目录地址
      }
      USBHS_HubInfo[depth].portD[port].Status = HUB_ERR_SCUESS;          //枚举成功
      USBHSH->HOST_RX_DMA = (UINT32)USBHS_endpRXbuf;
      USBHSH->HOST_TX_DMA = (UINT32)USBHS_endpTXbuf;              //回复控制请求地址
      return( ERR_SUCCESS );
  }
	
  
	else{        																																																	// 下面是全速和低速处理 - For LOW SPEED 
			printf("	USBHS_HUBHostEnum: full & low speed emulation \n");
		
      Delay_Ms(300);
      ret = USBHS_HubUSBGetDevDescr( &USBHS_HubInfo[depth].portD[port], Databuf, hubaddr);
      printf("	USBHS_HUBHostEnum: hub_dev_status=%02x\n",ret);
      ret = USBHS_HubUSBSetAddr( &USBHS_HubInfo[depth].portD[port],USB_SetUSBHS_AddressNumber(),Databuf ,hubaddr);

      printf("	USBHS_HUBHostEnum: set_addr=%02x\n",ret);
      ret = HubUSBHS_CtrlGetConfigDescr( &USBHS_HubInfo[depth].portD[port],Databuf ,hubaddr);
      printf("	USBHS_HUBHostEnum: hub_cfg_status=%02x\n",ret);
      ret = USBHS_HubUSBUSBHS_SetConfig(  &USBHS_HubInfo[depth].portD[port],USBHS_thisUsbDev.DeviceCongValue,Databuf ,hubaddr);
      printf("	USBHS_HUBHostEnum: hub_setcfg_status=%02x\n",ret);
      if( USBHS_HubInfo[depth].portD[port].DeviceType == 0x03 ){             													//HID设备类
          ret = USBHS_HubUSBSetIdle( &USBHS_HubInfo[depth].portD[port],Databuf ,hubaddr);  						//SET_IDLE
          printf("	USBHS_HUBHostEnum: set_idle=%02x\n",ret);
          ret = USBHS_HubCtrlGetHidDescr( &USBHS_HubInfo[depth].portD[port],Databuf ,hubaddr); 				//获取报表描述符
          printf("	USBHS_HUBHostEnum: get_report=%02x\n",ret);
      }
      USBHS_HubInfo[depth].portD[port].Status = HUB_ERR_SCUESS;          															//枚举成功
      USBHSH->HOST_RX_DMA = (UINT32)USBHS_endpRXbuf;      																						//恢复控制请求地址
      USBHSH->HOST_TX_DMA = (UINT32)USBHS_endpTXbuf;
      return( ERR_SUCCESS );
  }
}

/*******************************************************************************
 * @fn        Hublink_finesubstructures
 *
 * @briefI    Number of assigned addresses
 *
 * @param     None
 *
 * @return    Number of addresses to be set
 */
UINT8 USB_SetUSBHS_AddressNumber(void)
{
    UINT8 i = 0;
    for (i = DEVICE_ADDR+1; i < 127; i++)
    {
        if( USBHS_AddressNum[i] == 0 )
        {
            USBHS_AddressNum[i] = 1;
            return i;
        }
    }
    return 0xff;
}

/*******************************************************************************
 * @fn        Hublink_finesubstructures
 *
 * @briefI    Number of assigned addresses
 *
 * @param     None
 *
 * @return    0 - success 1 - faild
 */
UINT8 USB_DelUSBHS_AddressNumber(UINT8 addr)
{
    if( USBHS_AddressNum[addr] == 1)
    {
        USBHS_AddressNum[addr] = 0;
        return 0;//success
    }

    return 1;//fail
}

/*******************************************************************************
 * @fn        USBHS_HUB_Process
 *
 * @briefI    USBHS HUB Main Process
 *
 * @param     depth - hub depth
 *            addr  - device address
 *            uplevelport - up level port number
 *
 * @return    status
 */
UINT8 USBHS_HUB_Process( UINT8 depth, UINT8 addr, UINT8 uplevelport )
{
	
  UINT8 i,ret,s,dev_exist = 0;
  UINT16 temp,len;
  USB_HUB_SaveData hubdata,modifyhubdata;
  Link_HUBSaveData *printhub = NULL;																																	//用来打印

      for( i=0;i!=4;i++ )
      {
					// printf("	USBHS_HUB_Process: %02x\n" ,i);
          USBHSH->HOST_RX_DMA = (UINT32)USBHS_endpRXbuf;      																				//恢复控制请求地址
          USBHSH->HOST_TX_DMA = (UINT32)USBHS_endpTXbuf;

          hubdata.Depth = depth;
          hubdata.UpLevelPort = uplevelport;
          hubdata.CurrentPort = i+1;
          hubdata.HUB_Info = USBHS_HubInfo[depth];
          ret = Hublink_SearchHubData(USBHS_Hub_LinkHead , &hubdata);																	//判断当前节点是否有保存的数据
          if( ret == 0 )
          {
              USBHS_HubInfo[depth] = hubdata.HUB_Info;																								//若有保存的数据则替换
              dev_exist = 1;
          }
          else
          {
              memset( &USBHS_HubInfo[depth].portD[i], 0x00, sizeof( USBHS_HubInfo[depth].portD[i] ) );	//没有保存的数据则清除 防止使用上一个HUB数据
          }

          USBHS_CurrentAddr(addr);          																													//设置HUB地址进行操作
          USBHS_HUBCheckPortConnect( depth, i , dev_exist);

					// Main process function: Detects if there are changes at the port 
          if( USBHS_HubInfo[depth].portD[i].PortpChangeField )																				
          {
							// Sub-process 1: Detects that device connected to the port
              if( (USBHS_HubInfo[depth].portD[i].Status == HUB_ERR_CONNECT) )													// HUB is connected 设备连接
              {           
									printf("	-- USBHS_HUB_Process: Entering USBHS_HUBHostEnum -- \r\n");
                  ret = USBHS_HUBHostEnum( depth, USBHS_Test_Buf, i, uplevelport, addr);							// We run this function - so from main, main section 2 hub, then here 	
									printf("	-- USBHS_HUB_Process: Exiting USBHS_HUBHostEnum -- \r\n");
									printf("	USBHS_HUB_Process: Enum successful\r\n");									
                  if( ret == ERR_SUCCESS )
                  {
                      USBHS_HubInfo[depth].portD[i].Status = HUB_ERR_SCUESS;          								//枚举成功

                      hubdata.Depth = depth;
                      hubdata.UpLevelPort = uplevelport;
                      hubdata.CurrentPort = i+1;
                      hubdata.HUB_Info = USBHS_HubInfo[depth];

                      ret = Hublink_InsertHubData(USBHS_Hub_LinkHead,hubdata);
                      if( ret == 0 )
                      {
                          printf("#Save: depth-%x,uplevelport-%x,port-%x,Status-%x,hubaddr-%x#\n",depth,uplevelport,i+1,USBHS_HubInfo[depth].portD[i].Status,USBHS_HubInfo[depth].portD[i].Addr);
#if 1
                          printhub = USBHS_Hub_LinkHead;
                          while(printhub != NULL)//打印当前链表剩余节点
                          {
                                printf("/*******----------------------------------/\n");
                                printf("printhub.HUB_SaveData.Depth=%x\n",printhub->HUB_SaveData.Depth);
                                printf("printhub.HUB_SaveData.UpLevelPort=%x\n",printhub->HUB_SaveData.UpLevelPort);
                                printf("printhub.HUB_SaveData.CurrentPort=%x\n",printhub->HUB_SaveData.CurrentPort);
                                printf("printhub->HUB_SaveData.HUB_Info.portD[printhub->HUB_SaveData.CurrentPort-1].DeviceType=%x\n",printhub->HUB_SaveData.HUB_Info.portD[printhub->HUB_SaveData.CurrentPort-1].DeviceType);
                                printhub = printhub->next;
                          }
#endif
                      }
                  }
                  else
                  {
                      USBHS_HubInfo[depth].portD[i].Status = HUB_ERR_DISCONNECT;          					//枚举失败.重新枚举
                  }
              }
							
              // Sub-process 2: Detects that device disconnected from the port
							else if( USBHS_HubInfo[depth].portD[i].Status == 0x00 ) // HUB_ERR_DISCONNECT
              {
                  hubdata.Depth = depth;
                  hubdata.UpLevelPort = uplevelport;
                  hubdata.CurrentPort = i+1;
                  hubdata.HUB_Info = USBHS_HubInfo[depth];

                  if( hubdata.HUB_Info.portD[i].DeviceType == 0x09 )																//如果是HUB，删除时需要判断是否有下级节点，如果有需要删除
                  {
                      Hublink_finesubstructures(hubdata);
                  }

                  ret = Hublink_DeleteHubData(USBHS_Hub_LinkHead,hubdata);													//删除链表节点
                  if( ret == 0 )
                  {
                      printf("#Delete: depth-%x,uplevelport-%x,port-%x#\n",depth,uplevelport,i+1);
                      USB_DelUSBHS_AddressNumber(USBHS_HubInfo[depth].portD[i].Addr);
#if 1
                      printhub = USBHS_Hub_LinkHead;
                      while(printhub != NULL)//打印当前链表剩余节点
                      {
                            printf("/**********************************/\n");
                            printf("printhub.HUB_SaveData.Depth=%x\n",printhub->HUB_SaveData.Depth);
                            printf("printhub.HUB_SaveData.UpLevelPort=%x\n",printhub->HUB_SaveData.UpLevelPort);
                            printf("printhub.HUB_SaveData.CurrentPort=%x\n",printhub->HUB_SaveData.CurrentPort);
                            printf("printhub->HUB_SaveData.HUB_Info.portD[printhub->HUB_SaveData.CurrentPort-1].DeviceType=%x\n",printhub->HUB_SaveData.HUB_Info.portD[printhub->HUB_SaveData.CurrentPort-1].DeviceType);

                            printhub = printhub->next;
                      }
#endif
                  }

                  ret = USBHS_HubClearPortFeature( depth,i+1);
                  memset( &USBHS_HubInfo[depth].portD[i],0x00,sizeof( USBHS_HubInfo[depth].portD[i] ) );
                  printf("hub_device_disconnect_port=%02x\r\n",i+1);
              }
              // Sub-process 3: Error return function implementation
							else
              {
                  ret = USBHS_HubClearPortFeature( depth,i+1);
                  if( ret != ERR_SUCCESS )return ret;
                  ret = USBHS_HubGetPortStatus( depth,i+1 );
                  if( ret != ERR_SUCCESS )return ret;
              }
          }
					
					// Main process to printf Data from HID device																						
          if( USBHS_HubInfo[depth].portD[i].Status == HUB_ERR_SCUESS )					// 设备端口操作成功 Device Port Operation Success
          {       																																																			
              if(  USBHS_HubInfo[depth].portD[i].DeviceType == 0x03 )						// HID设备类 				HID Device Type
              {       																																																	
                  for( s=0;s!=USBHS_HubInfo[depth].portD[i].EndpNum; s++)				// 几个端点 				Loops through all the endpoints reported by Device
                  {   																																																	
                      if( USBHS_HubInfo[depth].portD[i].portEndp[s].Num &0x80 ) // IN端点						Checks if the endpoint is an IN endpoint
                      {    
													//																										// 									Executes Interrupt In Data transactions to the USB HID Device
                          ret = USBHS_INT_In_Data( &USBHS_HubInfo[depth].portD[i],s,USBHS_Test_Buf,&len,(USB_PID_IN<<4)|USBHS_HubInfo[depth].portD[i].portEndp[s].Num,&USBHS_HubInfo[depth].portD[i].portEndp[s].tog ,addr);
                          if( ret == ERR_SUCCESS )
                          {
															// After a successful IN transfer, you flip the DATA toggle so the next transfer alternates DATA0/DATA1.
															// As required by the USB protocol.
															
                              USBHS_HubInfo[depth].portD[i].portEndp[s].tog ^= 0x01;
#if USB_DEBUG								
															printf(" Device identifier: %x ", USBHS_HubInfo[depth].portD[i].DeviceHIDType);
#endif
															if(USBHS_HubInfo[depth].portD[i].DeviceHIDType == 0x02) // Process mouse applications
															{
#if USB_DEBUG
																printf("Processing mouse applications...");															// Prints Copied stored data points 
#endif													
														
																	for( temp = 0; temp!=len; temp++ )
																	{
																		MS_Data_Packs[temp] = USBHS_Test_Buf[temp] ;      
#if USB_DEBUG   
																	printf("%02x ", USBHS_Test_Buf[temp]);															// Prints Copied stored data points 
#endif
																	}

																// Upload mouse data to host: 
																USBFS_Endp_DataUp( DEF_UEP2, MS_Data_Packs, sizeof( MS_Data_Packs ), DEF_UEP_CPY_LOAD );
#if USB_DEBUG
																printf(" -- \n");
#endif
															}
															
															else if(USBHS_HubInfo[depth].portD[i].DeviceHIDType == 0x00) // Process keyboard applications
															{
#if USB_DEBUG
																	printf("Processing keyboard applications...\n");															// Prints Copied stored data points 
#endif														
																	for( temp = 0; temp!=len; temp++ )
																	{
																		KB_Data_Packs[temp] =       USBHS_Test_Buf[temp] ;
#if USB_DEBUG
																		printf("%02x ", USBHS_Test_Buf[temp]);															// Prints Copied stored data points 
#endif
																	}
											
																	// Upload mouse data to host: 
																	USBFS_Endp_DataUp( DEF_UEP1, KB_Data_Packs, sizeof( KB_Data_Packs ), DEF_UEP_CPY_LOAD );
#if USB_DEBUG
																	printf(" -- \n");	
#endif																	
															}
															
															else
															{
																// placeholder
															}
															
															// Updates the hubdata after modifications
                              modifyhubdata = hubdata;
                              modifyhubdata.HUB_Info.portD[i].portEndp[s].tog = USBHS_HubInfo[depth].portD[i].portEndp[s].tog;
                              Hublink_ModifyHubData(USBHS_Hub_LinkHead,hubdata,modifyhubdata);
                          }
                      }
                  }
              }
							
              else if( USBHS_HubInfo[depth].portD[i].DeviceType == 0x09 )			   																					// If the device type is HUB (0x09)
              {   																																																			// 下面是HUB，进行枚举设备, we need to do enumeration again
                  USBHS_glable_index++;
									printf("USBHS_HUB_Process: Hub Enumeration\n");
                  ret = USBHS_HUB_Process( USBHS_glable_index, USBHS_HubInfo[depth].portD[i].Addr, i+1);		
                  USBHS_glable_index--;
                  if( ret != ERR_SUCCESS )return ret;
              }
          }
          Delay_Ms(2);
      }
			
			//总的设备断开，直接跳走,需要把HUB的所有变了全部清除
      if( (USBHSH->MIS_ST & USBHS_ATTCH) == 0)
      {
				printf("USBHS_HUB_Process: Detect USB HUB Deattach - Initialization \n");
          for( i=0;i!=4;i++ )
          {
              USBHS_HubInfo[depth].portD[i].Status = 0;
              USBHS_HubInfo[depth].portD[i].Addr = 0;
              USBHS_HubInfo[depth].portD[i].Speed = 0;
              USBHS_HubInfo[depth].portD[i].DeviceType = 0;
          }
          memset( USBHS_HubInfo,0x00,sizeof( USBHS_HubInfo ) );
          return ERR_USB_DISCON;
      }
      return ERR_SUCCESS;
}

/*******************************************************************************
 * @fn        USBHS_HUB_Process_Remote
 *
 * @briefI    USBHS HUB Main Process
 *
 * @param     depth - hub depth
 *            addr  - device address
 *            uplevelport - up level port number
 *
 * @return    status
 */
UINT8 USBHS_HUB_Process_Remote( UINT8 depth, UINT8 addr, UINT8 uplevelport )
{
	
  UINT8 i,ret,s,dev_exist = 0;
  UINT16 temp,len;
  USB_HUB_SaveData hubdata,modifyhubdata;
  Link_HUBSaveData *printhub = NULL;																																	//用来打印

      for( i=0;i!=4;i++ )
      {
					// printf("	USBHS_HUB_Process: %02x\n" ,i);
          USBHSH->HOST_RX_DMA = (UINT32)USBHS_endpRXbuf;      																				//恢复控制请求地址
          USBHSH->HOST_TX_DMA = (UINT32)USBHS_endpTXbuf;

          hubdata.Depth = depth;
          hubdata.UpLevelPort = uplevelport;
          hubdata.CurrentPort = i+1;
          hubdata.HUB_Info = USBHS_HubInfo[depth];
          ret = Hublink_SearchHubData(USBHS_Hub_LinkHead , &hubdata);																	//判断当前节点是否有保存的数据
          if( ret == 0 )
          {
              USBHS_HubInfo[depth] = hubdata.HUB_Info;																								//若有保存的数据则替换
              dev_exist = 1;
          }
          else
          {
              memset( &USBHS_HubInfo[depth].portD[i], 0x00, sizeof( USBHS_HubInfo[depth].portD[i] ) );	//没有保存的数据则清除 防止使用上一个HUB数据
          }

          USBHS_CurrentAddr(addr);          																													//设置HUB地址进行操作
          USBHS_HUBCheckPortConnect( depth, i , dev_exist);

					// Main process function: Detects if there are changes at the port 
          if( USBHS_HubInfo[depth].portD[i].PortpChangeField )																				
          {
							// Sub-process 1: Detects that device connected to the port
              if( (USBHS_HubInfo[depth].portD[i].Status == HUB_ERR_CONNECT) )													// HUB is connected 设备连接
              {           
									printf("	-- USBHS_HUB_Process: Entering USBHS_HUBHostEnum -- \r\n");
                  ret = USBHS_HUBHostEnum( depth, USBHS_Test_Buf, i, uplevelport, addr);							// We run this function - so from main, main section 2 hub, then here 	
									printf("	-- USBHS_HUB_Process: Exiting USBHS_HUBHostEnum -- \r\n");
									printf("	USBHS_HUB_Process: Enum successful\r\n");									
                  if( ret == ERR_SUCCESS )
                  {
                      USBHS_HubInfo[depth].portD[i].Status = HUB_ERR_SCUESS;          								//枚举成功

                      hubdata.Depth = depth;
                      hubdata.UpLevelPort = uplevelport;
                      hubdata.CurrentPort = i+1;
                      hubdata.HUB_Info = USBHS_HubInfo[depth];

                      ret = Hublink_InsertHubData(USBHS_Hub_LinkHead,hubdata);
                      if( ret == 0 )
                      {
                          printf("#Save: depth-%x,uplevelport-%x,port-%x,Status-%x,hubaddr-%x#\n",depth,uplevelport,i+1,USBHS_HubInfo[depth].portD[i].Status,USBHS_HubInfo[depth].portD[i].Addr);
#if 1
                          printhub = USBHS_Hub_LinkHead;
                          while(printhub != NULL)//打印当前链表剩余节点
                          {
                                printf("/*******----------------------------------/\n");
                                printf("printhub.HUB_SaveData.Depth=%x\n",printhub->HUB_SaveData.Depth);
                                printf("printhub.HUB_SaveData.UpLevelPort=%x\n",printhub->HUB_SaveData.UpLevelPort);
                                printf("printhub.HUB_SaveData.CurrentPort=%x\n",printhub->HUB_SaveData.CurrentPort);
                                printf("printhub->HUB_SaveData.HUB_Info.portD[printhub->HUB_SaveData.CurrentPort-1].DeviceType=%x\n",printhub->HUB_SaveData.HUB_Info.portD[printhub->HUB_SaveData.CurrentPort-1].DeviceType);
                                printhub = printhub->next;
                          }
#endif
                      }
                  }
                  else
                  {
                      USBHS_HubInfo[depth].portD[i].Status = HUB_ERR_DISCONNECT;          					//枚举失败.重新枚举
                  }
              }
							
              // Sub-process 2: Detects that device disconnected from the port
							else if( USBHS_HubInfo[depth].portD[i].Status == 0x00 ) // HUB_ERR_DISCONNECT
              {
                  hubdata.Depth = depth;
                  hubdata.UpLevelPort = uplevelport;
                  hubdata.CurrentPort = i+1;
                  hubdata.HUB_Info = USBHS_HubInfo[depth];

                  if( hubdata.HUB_Info.portD[i].DeviceType == 0x09 )																//如果是HUB，删除时需要判断是否有下级节点，如果有需要删除
                  {
                      Hublink_finesubstructures(hubdata);
                  }

                  ret = Hublink_DeleteHubData(USBHS_Hub_LinkHead,hubdata);													//删除链表节点
                  if( ret == 0 )
                  {
                      printf("#Delete: depth-%x,uplevelport-%x,port-%x#\n",depth,uplevelport,i+1);
                      USB_DelUSBHS_AddressNumber(USBHS_HubInfo[depth].portD[i].Addr);
#if 1
                      printhub = USBHS_Hub_LinkHead;
                      while(printhub != NULL)//打印当前链表剩余节点
                      {
                            printf("/**********************************/\n");
                            printf("printhub.HUB_SaveData.Depth=%x\n",printhub->HUB_SaveData.Depth);
                            printf("printhub.HUB_SaveData.UpLevelPort=%x\n",printhub->HUB_SaveData.UpLevelPort);
                            printf("printhub.HUB_SaveData.CurrentPort=%x\n",printhub->HUB_SaveData.CurrentPort);
                            printf("printhub->HUB_SaveData.HUB_Info.portD[printhub->HUB_SaveData.CurrentPort-1].DeviceType=%x\n",printhub->HUB_SaveData.HUB_Info.portD[printhub->HUB_SaveData.CurrentPort-1].DeviceType);

                            printhub = printhub->next;
                      }
#endif
                  }

                  ret = USBHS_HubClearPortFeature( depth,i+1);
                  memset( &USBHS_HubInfo[depth].portD[i],0x00,sizeof( USBHS_HubInfo[depth].portD[i] ) );
                  printf("hub_device_disconnect_port=%02x\r\n",i+1);
              }
              // Sub-process 3: Error return function implementation
							else
              {
                  ret = USBHS_HubClearPortFeature( depth,i+1);
                  if( ret != ERR_SUCCESS )return ret;
                  ret = USBHS_HubGetPortStatus( depth,i+1 );
                  if( ret != ERR_SUCCESS )return ret;
              }
          }
					
					// Main process to printf Data from HID device																						
          if( USBHS_HubInfo[depth].portD[i].Status == HUB_ERR_SCUESS )					// 设备端口操作成功 Device Port Operation Success
          {       																																																			
              if(  USBHS_HubInfo[depth].portD[i].DeviceType == 0x03 )						// HID设备类 				HID Device Type
              {       																																																	
                  for( s=0;s!=USBHS_HubInfo[depth].portD[i].EndpNum; s++)				// 几个端点 				Loops through all the endpoints reported by Device
                  {   																																																	
                      if( USBHS_HubInfo[depth].portD[i].portEndp[s].Num &0x80 ) // IN端点						Checks if the endpoint is an IN endpoint
                      {    
													//																										// 									Executes Interrupt In Data transactions to the USB HID Device
															
                              USBHS_HubInfo[depth].portD[i].portEndp[s].tog ^= 0x01;
#if USB_DEBUG								
															printf(" Device identifier: %x ", USBHS_HubInfo[depth].portD[i].DeviceHIDType);
#endif
															if(USBHS_HubInfo[depth].portD[i].DeviceHIDType == 0x02) // Process mouse applications
															{
#if USB_DEBUG
																printf("Processing mouse applications...");															// Prints Copied stored data points 
#endif													
														
																	for( temp = 0; temp!=len; temp++ )
																	{
																		MS_Data_Packs[temp] = USBHS_Test_Buf[temp] ;      
#if USB_DEBUG   
																	printf("%02x ", USBHS_Test_Buf[temp]);															// Prints Copied stored data points 
#endif
																	}

																// Upload mouse data to host: 
																USBFS_Endp_DataUp( DEF_UEP2, MS_Data_Packs, sizeof( MS_Data_Packs ), DEF_UEP_CPY_LOAD );
#if USB_DEBUG
																printf(" -- \n");
#endif
															}
															
															else if(USBHS_HubInfo[depth].portD[i].DeviceHIDType == 0x00) // Process keyboard applications
															{
#if USB_DEBUG
																	printf("Processing keyboard applications...\n");															// Prints Copied stored data points 
#endif														
																	for( temp = 0; temp!=len; temp++ )
																	{
																		KB_Data_Packs[temp] =       USBHS_Test_Buf[temp] ;
#if USB_DEBUG
																		printf("%02x ", USBHS_Test_Buf[temp]);															// Prints Copied stored data points 
#endif
																	}
											
																	// Upload mouse data to host: 
																	USBFS_Endp_DataUp( DEF_UEP1, KB_Data_Packs, sizeof( KB_Data_Packs ), DEF_UEP_CPY_LOAD );
#if USB_DEBUG
																	printf(" -- \n");	
#endif																	
															}
															
															else
															{
																// placeholder
															}
															
															// Updates the hubdata after modifications
                              modifyhubdata = hubdata;
                              modifyhubdata.HUB_Info.portD[i].portEndp[s].tog = USBHS_HubInfo[depth].portD[i].portEndp[s].tog;
                              Hublink_ModifyHubData(USBHS_Hub_LinkHead,hubdata,modifyhubdata);

                      }
                  }
              }
							
              else if( USBHS_HubInfo[depth].portD[i].DeviceType == 0x09 )			   																					// If the device type is HUB (0x09)
              {   																																																			// 下面是HUB，进行枚举设备, we need to do enumeration again
                  USBHS_glable_index++;
									printf("USBHS_HUB_Process: Hub Enumeration\n");
                  ret = USBHS_HUB_Process( USBHS_glable_index, USBHS_HubInfo[depth].portD[i].Addr, i+1);		
                  USBHS_glable_index--;
                  if( ret != ERR_SUCCESS )return ret;
              }
          }
          Delay_Ms(2);
      }
			
			//总的设备断开，直接跳走,需要把HUB的所有变了全部清除
      if( (USBHSH->MIS_ST & USBHS_ATTCH) == 0)
      {
				printf("USBHS_HUB_Process: Detect USB HUB Deattach - Initialization \n");
          for( i=0;i!=4;i++ )
          {
              USBHS_HubInfo[depth].portD[i].Status = 0;
              USBHS_HubInfo[depth].portD[i].Addr = 0;
              USBHS_HubInfo[depth].portD[i].Speed = 0;
              USBHS_HubInfo[depth].portD[i].DeviceType = 0;
          }
          memset( USBHS_HubInfo,0x00,sizeof( USBHS_HubInfo ) );
          return ERR_USB_DISCON;
      }
      return ERR_SUCCESS;
}
