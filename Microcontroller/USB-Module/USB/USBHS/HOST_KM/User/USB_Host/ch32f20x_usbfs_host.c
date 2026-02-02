/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32f20x_usbfs_host.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2022/09/01
* Description        : This file provides the relevant operation functions of the
*                      USB full-speed host port.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/


/*******************************************************************************/
/* Header File */
#include <usbfs_host_config.h>
#include <usb_hub.h>
#include "stdlib.h"
#include "ch32f20x_usbhs_host.h"
#include "ch32f20x_usbfs_device.h"

/*******************************************************************************/
/* Variable Definition */

UINT8  USBFS_UsbDevEndp0Size = 0;
UINT16 USBFS_EndpnMaxSize = 0;
USBDEV_INFO  USBFS_thisUsbDev;
UINT16 Hid_Report_Len;
UINT8 USBFS_AddressNum[127] = {0};
UINT8 USBFS_glable_index = 0;
struct   _ROOT_HUB_DEVICE RootHubDev[ DEF_TOTAL_ROOT_HUB ];
__attribute__ ((aligned(4))) UINT8 USBFS_test_buf[3072];
#define DEF_COM_BUF_LEN                 1024
uint8_t  Com_Buf[ DEF_COM_BUF_LEN ];                                            // General Buffer
uint8_t  DevDesc_Buf[ 18 ];                                                     // Device Descriptor Buffer

/******************************** HOST DEVICE **********************************/
__attribute__ ((aligned(4))) const UINT8  GetDevDescrptor[]={USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_DEVICE, 0x00, 0x00, 0x12, 0x00};
__attribute__ ((aligned(4))) const UINT8  GetConfigDescrptor[]= {USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_CONFIG, 0x00, 0x00, 0x04, 0x00};
__attribute__ ((aligned(4))) const UINT8  SetAddress[]={USB_REQ_TYP_OUT, USB_SET_ADDRESS, USB_DEVICE_ADDR, 0x00, 0x00, 0x00, 0x00, 0x00};
__attribute__ ((aligned(4))) const UINT8  SetConfig[]={USB_REQ_TYP_OUT, USB_SET_CONFIGURATION, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
__attribute__ ((aligned(4))) const UINT8  Clear_EndpStall[]={USB_REQ_RECIP_INTERF, USB_SET_INTERFACE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
__attribute__ ((aligned(4))) const UINT8  SetupSetUsbInterface[] = { USB_REQ_RECIP_INTERF, USB_SET_INTERFACE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
__attribute__ ((aligned(4))) const UINT8  SetupClrEndpStall[] = { USB_REQ_TYP_OUT | USB_REQ_RECIP_ENDP, USB_CLEAR_FEATURE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/*********************************************************************
 * @fn      USBFS_RCC_Init
 *
 * @brief   Set USB port clock.
 *          Note: If the SystemCoreClock is selected as the USB clock source,
 *          only the frequency specified below can be used.
 *
 * @return  none
 */
void USBFS_RCC_Init( void )
{
#ifdef CH32V30x_D8C
    RCC_USBCLK48MConfig( RCC_USBCLK48MCLKSource_USBPHY );
    RCC_USBHSPLLCLKConfig( RCC_HSBHSPLLCLKSource_HSE );
    RCC_USBHSConfig( RCC_USBPLL_Div2 );
    RCC_USBHSPLLCKREFCLKConfig( RCC_USBHSPLLCKREFCLK_4M );
    RCC_USBHSPHYPLLALIVEcmd( ENABLE );
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_USBHS, ENABLE );
#else
    if( SystemCoreClock == 144000000 )
    {
        RCC_OTGFSCLKConfig( RCC_OTGFSCLKSource_PLLCLK_Div3 );
    }
    else if( SystemCoreClock == 96000000 )
    {
        RCC_OTGFSCLKConfig( RCC_OTGFSCLKSource_PLLCLK_Div2 );
    }
    else if( SystemCoreClock == 48000000 )
    {
        RCC_OTGFSCLKConfig( RCC_OTGFSCLKSource_PLLCLK_Div1 );
    }
#endif
    
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_OTG_FS, ENABLE );
}

/*********************************************************************
 * @fn      USBFS_Host_Init
 *
 * @brief   Initialize USB port host configuration.
 *
 * @param   sta - ENABLE or DISABLE
 *
 * @return  none
 */
void USBFS_Host_Init( FunctionalState sta )
{
    if( sta == ENABLE )
    {
        /* Reset USB module */
        USBOTG_H_FS->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
        Delay_Us( 10 );
        USBOTG_H_FS->BASE_CTRL = 0;

        /* Initialize USB host configuration */
        USBOTG_H_FS->BASE_CTRL = USBFS_UC_HOST_MODE | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN;
        USBOTG_H_FS->HOST_EP_MOD = USBFS_UH_EP_TX_EN | USBFS_UH_EP_RX_EN;
        USBOTG_H_FS->HOST_RX_DMA = (uint32_t)USBFS_endpRXbuf;
        USBOTG_H_FS->HOST_TX_DMA = (uint32_t)USBFS_endpTXbuf;
    }
    else
    {
        USBOTG_H_FS->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
        Delay_Us( 10 );
        USBOTG_H_FS->BASE_CTRL = 0;
    }
}

/*********************************************************************
 * @fn      USBFSH_CheckRootHubPortStatus
 *
 * @brief   Check the current status of the USB port in combination with 
 *          the saved status of the root device connected to this port.
 *
 * @para    dev_sta: The saved status of the root device.
 *
 * @return  The current status of the port.
 */
uint8_t USBFSH_CheckRootHubPortStatus( uint8_t dev_sta )
{
    if( USBOTG_H_FS->INT_FG & USBFS_UIF_DETECT ) // Check that there is a device connection or disconnection event on the port
    {
        USBOTG_H_FS->INT_FG = USBFS_UIF_DETECT; // Clear flag
        
        if( USBOTG_H_FS->MIS_ST & USBFS_UMS_DEV_ATTACH ) // Check that there is a device connection to the port
        {
            if( ( dev_sta == ROOT_DEV_DISCONNECT ) || ( ( dev_sta != ROOT_DEV_FAILED ) && ( USBFSH_CheckRootHubPortEnable( ) == 0x00 ) ) )
            {
                return ROOT_DEV_CONNECTED;
            }
            else
            {
                return ROOT_DEV_FAILED;
            }
        }
        else // Check that there is no device connection to the port
        {
            return ROOT_DEV_DISCONNECT;
        }
    }
    else
    {
        return ROOT_DEV_FAILED;
    }
}

/*********************************************************************
 * @fn      USBFSH_CheckRootHubPortEnable
 *
 * @brief   Check the enable status of the USB port.
 *          Note: This bit is automatically cleared when the device is disconnected.    
 *
 * @return  The current enable status of the port.
 */
uint8_t USBFSH_CheckRootHubPortEnable( void )
{
    return ( USBOTG_H_FS->HOST_CTRL & USBFS_UH_PORT_EN );
}

/*********************************************************************
 * @fn      USBFSH_CheckRootHubPortSpeed
 *
 * @brief   Check the speed of the USB port.
 *
 * @return  The current speed of the port.
 */
uint8_t USBFSH_CheckRootHubPortSpeed( void )
{
    return ( USBOTG_H_FS->MIS_ST & USBFS_UMS_DM_LEVEL? USB_LOW_SPEED: USB_FULL_SPEED );
}

/*********************************************************************
 * @fn      USBFSH_SetSelfAddr
 *
 * @brief   Set the USB device address.
 *
 * @para    addr: USB device address.
 *
 * @return  none
 */
void USBFSH_SetSelfAddr( uint8_t addr )
{
    USBOTG_H_FS->DEV_ADDR = ( USBOTG_H_FS->DEV_ADDR & USBFS_UDA_GP_BIT ) | ( addr & USBFS_USB_ADDR_MASK );
}

/*********************************************************************
 * @fn      USBFSH_SetSelfSpeed
 *
 * @brief   Set USB speed.
 *
 * @para    speed: USB speed.
 *
 * @return  none
 */
void USBFSH_SetSelfSpeed( uint8_t speed )
{
    if( speed == USB_FULL_SPEED )
    {
        USBOTG_H_FS->BASE_CTRL &= ~USBFS_UC_LOW_SPEED;
        USBOTG_H_FS->HOST_CTRL &= ~USBFS_UH_LOW_SPEED;
        USBOTG_H_FS->HOST_SETUP &= ~USBFS_UH_PRE_PID_EN;
    }
    else
    {
        USBOTG_H_FS->BASE_CTRL |= USBFS_UC_LOW_SPEED;
        USBOTG_H_FS->HOST_CTRL |= USBFS_UH_LOW_SPEED;
        USBOTG_H_FS->HOST_SETUP |= USBFS_UH_PRE_PID_EN;
    }
}

/*********************************************************************
 * @fn      USBFSH_ResetRootHubPort
 *
 * @brief   Reset USB port.
 *
 * @para    mod: Reset host port operating mode.
 *               0 -> reset and wait end
 *               1 -> begin reset
 *               2 -> end reset
 *
 * @return  none
 */
void USBFSH_ResetRootHubPort( uint8_t mode )
{
    USBFSH_SetSelfAddr( 0x00 );
    USBFSH_SetSelfSpeed( USB_FULL_SPEED );
    
    if( mode <= 1 )
    {
        USBOTG_H_FS->HOST_CTRL |= USBFS_UH_BUS_RESET; // Start reset
    }
    if( mode == 0 )
    {
        Delay_Ms( DEF_BUS_RESET_TIME ); // Reset time from 10mS to 20mS
    }
    if( mode != 1 )
    {
        USBOTG_H_FS->HOST_CTRL &= ~USBFS_UH_BUS_RESET; // End reset
    }
    Delay_Ms( 2 );

    if( USBOTG_H_FS->INT_FG & USBFS_UIF_DETECT )
    {
        if( USBOTG_H_FS->MIS_ST & USBFS_UMS_DEV_ATTACH )
        {
            USBOTG_H_FS->INT_FG = USBFS_UIF_DETECT;
        }
    }
}

/*********************************************************************
 * @fn      USBFSH_EnableRootHubPort
 *
 * @brief   Enable USB host port.
 *
 * @para    *pspeed: USB speed.
 *
 * @return  Operation result of the enabled port.
 */
uint8_t USBFSH_EnableRootHubPort( uint8_t *pspeed )
{
    if( USBOTG_H_FS->MIS_ST & USBFS_UMS_DEV_ATTACH )
    {
        if( USBFSH_CheckRootHubPortEnable( ) == 0x00 )
        { 
            *pspeed = USBFSH_CheckRootHubPortSpeed( );
            if( *pspeed == USB_LOW_SPEED )
            {
                USBFSH_SetSelfSpeed( USB_LOW_SPEED );
            }
        }
        USBOTG_H_FS->HOST_CTRL |= USBFS_UH_PORT_EN;
        USBOTG_H_FS->HOST_SETUP |= USBFS_UH_SOF_EN;

        return ERR_SUCCESS;
    }

    return ERR_USB_DISCON;
}

/*********************************************************************
 * @fn      USBFSH_Transact
 *
 * @brief   Perform USB transaction.
 *
 * @para    endp_pid: Token PID.
 *          endp_tog: Toggle
 *          timeout: Timeout time.
 *
 * @return  USB transfer result.
 */
uint8_t USBFSH_Transact( uint8_t endp_pid, uint8_t endp_tog, uint32_t timeout )
{
    uint8_t  r, trans_retry;
    uint16_t i;

    USBOTG_H_FS->HOST_TX_CTRL = USBOTG_H_FS->HOST_RX_CTRL = endp_tog;
    trans_retry = 0;

    do
    {
        USBOTG_H_FS->HOST_EP_PID = endp_pid; // Specify token PID and endpoint number
        USBOTG_H_FS->INT_FG = USBFS_UIF_TRANSFER; // Allow transfer
        for( i = DEF_WAIT_USB_TRANSFER_CNT; ( i != 0 ) && ( ( USBOTG_H_FS->INT_FG & USBFS_UIF_TRANSFER ) == 0 ); i-- )
        {
            Delay_Us( 1 ); // Delay for USB transfer
        }
        USBOTG_H_FS->HOST_EP_PID = 0x00; // Stop transfer

        if( ( USBOTG_H_FS->INT_FG & USBFS_UIF_TRANSFER ) == 0 )
        {
            return ERR_USB_UNKNOWN;
        }
        else // Complete transfer
        {
            if( USBOTG_H_FS->INT_ST & USBFS_UIS_TOG_OK )
            {
                return ERR_SUCCESS;
            }

            r = USBOTG_H_FS->INT_ST & USBFS_UIS_H_RES_MASK; // Response status of current USB transaction

            if( r == USB_PID_STALL )
            {
                return ( r | ERR_USB_TRANSFER );
            }
            if( r == USB_PID_NAK )
            {
                if( timeout == 0 )
                {
                    return ( r | ERR_USB_TRANSFER );
                }
                if( timeout < 0xFFFFFFFF )
                {
                    timeout--;
                }
                --trans_retry;
            }
            else switch ( endp_pid >> 4 )
            {
                case USB_PID_SETUP:

                case USB_PID_OUT:
                    if( r )
                    {
                        return ( r | ERR_USB_TRANSFER );
                    }
                    break;

                case USB_PID_IN:
                    if( ( r == USB_PID_DATA0 ) || ( r == USB_PID_DATA1 ) )
                    {
                        ;
                    }
                    else if( r )
                    {
                        return ( r | ERR_USB_TRANSFER );
                    }
                    break;

                default:
                    return ERR_USB_UNKNOWN;
            }
        }
        Delay_Us( 2000 );

        if( USBOTG_H_FS->INT_FG & USBFS_UIF_DETECT )
        {
            Delay_Us( 200 );

            if( USBFSH_CheckRootHubPortEnable( ) == 0x00 )
            {
                return ERR_USB_CONNECT;  // USB device connect
            }
            else
            {
                USBOTG_H_FS->INT_FG = USBFS_UIF_DETECT;
            }
        }
    }while( ++trans_retry < 10 );

    return ERR_USB_TRANSFER; // Reply timeout

}

/*********************************************************************
 * @fn      USBFSH_CtrlTransfer
 *
 * @brief   USB host control transfer.
 *
 * @para    ep0_size: Device endpoint 0 size
 *          pbuf: Data buffer
 *          plen: Data length
 *
 * @return  USB control transfer result.
 */
uint8_t USBFSH_CtrlTransfer( uint8_t ep0_size, uint8_t *pbuf, uint16_t *plen )
{
    uint8_t  s;
    uint16_t rem_len, rx_len, rx_cnt, tx_cnt;

    Delay_Us( 100 );
    if( plen )
    {
        *plen = 0;
    }
    USBOTG_H_FS->HOST_TX_LEN = sizeof( USB_SETUP_REQ );

    s = USBFSH_Transact( ( USB_PID_SETUP << 4 ) | 0x00, 0x00, DEF_CTRL_TRANS_TIMEOVER_CNT ); // SETUP stage
    if( s != ERR_SUCCESS )
    {
        return s;
    }
    
    USBOTG_H_FS->HOST_TX_CTRL = USBOTG_H_FS->HOST_RX_CTRL = USBFS_UH_T_TOG | USBFS_UH_R_TOG; // Default DATA1
    rem_len = pUSBFS_SetupRequest->wLength;
    if( rem_len && pbuf )
    {
        if( pUSBFS_SetupRequest->bRequestType & USB_REQ_TYP_IN )
        {
            /* Receive data */
            while( rem_len )
            {
                Delay_Us( 100 );
                s = USBFSH_Transact( ( USB_PID_IN << 4 ) | 0x00, USBOTG_H_FS->HOST_RX_CTRL, DEF_CTRL_TRANS_TIMEOVER_CNT );  // IN
                if( s != ERR_SUCCESS )
                {
                    return s;
                }
                USBOTG_H_FS->HOST_RX_CTRL ^= USBFS_UH_R_TOG;
                
                rx_len = ( USBOTG_H_FS->RX_LEN < rem_len )? USBOTG_H_FS->RX_LEN : rem_len;
                rem_len -= rx_len;
                if( plen )
                {
                    *plen += rx_len; // The total length of the actual successful transmission and reception
                }
                for( rx_cnt = 0; rx_cnt != rx_len; rx_cnt++ )
                {
                    *pbuf = USBFS_endpRXbuf[ rx_cnt ];
                    pbuf++;
                }

                if( ( USBOTG_H_FS->RX_LEN == 0 ) || ( USBOTG_H_FS->RX_LEN & ( ep0_size - 1 ) ) )
                {
                    break; // Short package
                }
            }
            USBOTG_H_FS->HOST_TX_LEN = 0; // Status stage is OUT
        }
        else
        {
            /* Send data */
            while( rem_len )
            {
                Delay_Us( 100 );
                USBOTG_H_FS->HOST_TX_LEN = ( rem_len >= ep0_size )? ep0_size : rem_len;
                for( tx_cnt = 0; tx_cnt != USBOTG_H_FS->HOST_TX_LEN; tx_cnt++ )
                {
                    USBFS_endpTXbuf[ tx_cnt ] = *pbuf;
                    pbuf++;
                }
                s = USBFSH_Transact( USB_PID_OUT << 4 | 0x00, USBOTG_H_FS->HOST_TX_CTRL, DEF_CTRL_TRANS_TIMEOVER_CNT ); // OUT
                if( s != ERR_SUCCESS )
                {
                    return s;
                }
                USBOTG_H_FS->HOST_TX_CTRL ^= USBFS_UH_T_TOG;
                
                rem_len -= USBOTG_H_FS->HOST_TX_LEN;
                if( plen )
                {
                    *plen += USBOTG_H_FS->HOST_TX_LEN; // The total length of the actual successful transmission and reception
                }
            }
        }
    }

    Delay_Us( 100 );
    s = USBFSH_Transact( ( USBOTG_H_FS->HOST_TX_LEN )? ( USB_PID_IN << 4 | 0x00 ) : ( USB_PID_OUT << 4 | 0x00 ), USBFS_UH_R_TOG | USBFS_UH_T_TOG, DEF_CTRL_TRANS_TIMEOVER_CNT ); // STATUS stage
    if( s != ERR_SUCCESS )
    {
        return s;
    }
    if( USBOTG_H_FS->HOST_TX_LEN == 0 )
    {
        return ERR_SUCCESS;
    }
    if( USBOTG_H_FS->RX_LEN == 0 )
    {
        return ERR_SUCCESS;
    }
    return ERR_USB_BUF_OVER;
}

/*********************************************************************
 * @fn      USBFSH_GetDeviceDescr
 *
 * @brief   Get the device descriptor of the USB device.
 *
 * @para    *pep0_size: Device endpoint 0 size.
 *          *pbuf: Data buffer.
 *
 * @return  The result of getting the device descriptor.
 */
uint8_t USBFSH_GetDeviceDescr( uint8_t *pep0_size, uint8_t *pbuf )
{
    uint8_t  s;
    uint16_t len;

    *pep0_size = DEFAULT_ENDP0_SIZE;
    memcpy( pUSBFS_SetupRequest, SetupGetDevDesc, sizeof( USB_SETUP_REQ ) );
    s = USBFSH_CtrlTransfer( *pep0_size, pbuf, &len );
    if( s != ERR_SUCCESS )
    {
        return s;
    }

    *pep0_size = ( (PUSB_DEV_DESCR)pbuf )->bMaxPacketSize0;
    if( len < ( (PUSB_SETUP_REQ)SetupGetDevDesc )->wLength )
    {
        return ERR_USB_BUF_OVER;
    }
    return ERR_SUCCESS;
}

/*********************************************************************
 * @fn      USBFSH_GetConfigDescr
 *
 * @brief   Get the configuration descriptor of the USB device. 
 *
 * @para    ep0_size: Device endpoint 0 size.
 *          *pbuf: Data buffer.
 *          buf_len: Data buffer length.
 *          *pcfg_len: The length of the device configuration descriptor.
 *
 * @return  The result of getting the configuration descriptor.
 */
uint8_t USBFSH_GetConfigDescr( uint8_t ep0_size, uint8_t *pbuf, uint16_t buf_len, uint16_t *pcfg_len )
{
    uint8_t  s;
    
    memcpy( pUSBFS_SetupRequest, SetupGetCfgDesc, sizeof( USB_SETUP_REQ ) );
    s = USBFSH_CtrlTransfer( ep0_size, pbuf, pcfg_len );
    if( s != ERR_SUCCESS )
    {
        return s;
    }
    if( *pcfg_len < ( (PUSB_SETUP_REQ)SetupGetCfgDesc )->wLength )
    {
        return ERR_USB_BUF_OVER;
    }

    *pcfg_len = ( (PUSB_CFG_DESCR)pbuf )->wTotalLength;
    if( *pcfg_len > buf_len  )
    {
        *pcfg_len = buf_len;
    }
    memcpy( pUSBFS_SetupRequest, SetupGetCfgDesc, sizeof( USB_SETUP_REQ ) );
    pUSBFS_SetupRequest->wLength = *pcfg_len;
    s = USBFSH_CtrlTransfer( ep0_size, pbuf, pcfg_len );
    return s;
}

/*********************************************************************
 * @fn      USBFSH_GetStrDescr
 *
 * @brief   Get the string descriptor of the USB device.
 *
 * @para    ep0_size: Device endpoint 0 size.
 *          str_num: Index of string descriptor.  
 *          *pbuf: Data buffer.
 *
 * @return  The result of getting the string descriptor.
 */
uint8_t USBFSH_GetStrDescr( uint8_t ep0_size, uint8_t str_num, uint8_t *pbuf )
{
    uint8_t  s;
    uint16_t len;

    /* Get the string descriptor of the first 4 bytes */
    memcpy( pUSBFS_SetupRequest, SetupGetStrDesc, sizeof( USB_SETUP_REQ ) );
    pUSBFS_SetupRequest->wValue = ( (uint16_t)USB_DESCR_TYP_STRING << 8 ) | str_num;
    s = USBFSH_CtrlTransfer( ep0_size, pbuf, &len );
    if( s != ERR_SUCCESS )
    {
        return s;
    }

    /* Get the complete string descriptor */
    len = pbuf[ 0 ];
    memcpy( pUSBFS_SetupRequest, SetupGetStrDesc, sizeof( USB_SETUP_REQ ) );
    pUSBFS_SetupRequest->wValue = ( (uint16_t)USB_DESCR_TYP_STRING << 8 ) | str_num;
    pUSBFS_SetupRequest->wLength = len;
    s = USBFSH_CtrlTransfer( ep0_size, pbuf, &len );
    if( s != ERR_SUCCESS )
    {
        return s;
    }
    return ERR_SUCCESS;
}

/*********************************************************************
 * @fn      USBFSH_SetUsbAddress
 *
 * @brief   Set USB device address.
 *
 * @para    ep0_size: Device endpoint 0 size.
 *          addr: Device address.
 *
 * @return  The result of setting device address.
 */
uint8_t USBFSH_SetUsbAddress( uint8_t ep0_size, uint8_t addr )
{
    uint8_t  s;

    memcpy( pUSBFS_SetupRequest, SetupSetAddr, sizeof( USB_SETUP_REQ ) );
    pUSBFS_SetupRequest->wValue = (uint16_t)addr;
    s = USBFSH_CtrlTransfer( ep0_size, NULL, NULL );
    if( s != ERR_SUCCESS )
    {
        return s;
    }
    USBFSH_SetSelfAddr( addr );
    Delay_Ms( DEF_BUS_RESET_TIME >> 1 ); // Wait for the USB device to complete its operation.
    return ERR_SUCCESS;
}

/*********************************************************************
 * @fn      USBFSH_SetUsbConfig
 *
 * @brief   Set USB configuration.
 *
 * @para    ep0_size: Device endpoint 0 size
 *          cfg_val: Device configuration value
 *
 * @return  The result of setting device configuration.
 */
uint8_t USBFSH_SetUsbConfig( uint8_t ep0_size, uint8_t cfg_val )
{
    memcpy( pUSBFS_SetupRequest, SetupSetConfig, sizeof( USB_SETUP_REQ ) );
    pUSBFS_SetupRequest->wValue = (uint16_t)cfg_val;
    return USBFSH_CtrlTransfer( ep0_size, NULL, NULL );
}

/*********************************************************************
 * @fn      USBFSH_ClearEndpStall
 *
 * @brief   Clear endpoint stall.
 *
 * @para    ep0_size: Device endpoint 0 size
 *          endp_num: Endpoint number.
 *
 * @return  The result of clearing endpoint stall.
 */
uint8_t USBFSH_ClearEndpStall( uint8_t ep0_size, uint8_t endp_num )
{
    memcpy( pUSBFS_SetupRequest, SetupClearEndpStall, sizeof( USB_SETUP_REQ ) );
    pUSBFS_SetupRequest->wIndex = (uint16_t)endp_num;
    return USBFSH_CtrlTransfer( ep0_size, NULL, NULL );
}

/*********************************************************************
 * @fn      USBFSH_GetEndpData
 *
 * @brief   Get data from USB device input endpoint.
 *
 * @para    endp_num: Endpoint number
 *          pendp_tog: Endpoint toggle
 *          pbuf: Data Buffer
 *          plen: Data length
 *
 * @return  The result of getting data.
 */
uint8_t USBFSH_GetEndpData( uint8_t endp_num, uint8_t *pendp_tog, uint8_t *pbuf, uint16_t *plen )
{
    uint8_t  s;
    
    s = USBFSH_Transact( ( USB_PID_IN << 4 ) | endp_num, *pendp_tog, 0 );
    if( s == ERR_SUCCESS )
    {
        *plen = USBOTG_H_FS->RX_LEN;
        memcpy( pbuf, USBFS_endpRXbuf, *plen );

        *pendp_tog ^= USBFS_UH_R_TOG;
    }
    
    return s;
}

/*********************************************************************
 * @fn      USBFSH_SendEndpData
 *
 * @brief   Send data to the USB device output endpoint.
 *
 * @para    endp_num: Endpoint number
 *          endp_tog: Endpoint toggle
 *          pbuf: Data Buffer
 *          len: Data length
 *
 * @return  The result of sending data.
 */
uint8_t USBFSH_SendEndpData( uint8_t endp_num, uint8_t *pendp_tog, uint8_t *pbuf, uint16_t len )
{
    uint8_t  s;
    
    memcpy( USBFS_endpTXbuf, pbuf, len );
    USBOTG_H_FS->HOST_TX_LEN = len;
    s = USBFSH_Transact( ( USB_PID_OUT << 4 ) | endp_num, *pendp_tog, 0 );
    if( s == ERR_SUCCESS )
    {
        *pendp_tog ^= USBFS_UH_T_TOG;
    }

    return s;
}

/*********************************************************************
 * @fn      SetBusReset
 *
 * @brief   Reset USB bus
 *
 * @return  none
 */
void  SetBusReset(void)
{
    USBOTG_H_FS->HOST_CTRL |= USBFS_UH_BUS_RESET;                              //bus reset
    Delay_Ms(15);
    USBOTG_H_FS->HOST_CTRL &= ~USBFS_UH_BUS_RESET;
}

/*********************************************************************
 * @fn      USBFS_CurrentAddr
 *
 * @brief   set address
 *
 * @return  none
 */
void USBFS_CurrentAddr( UINT8 address )
{
    USBOTG_H_FS->DEV_ADDR = address;                  // SET ADDRESS
}

/*********************************************************************
 * @fn      CopySetupReqPkg
 *
 * @brief   copy the contents of the buffer to send buffer.
 *
 * @param   pReqPkt - target buffer address
 *
 * @return  none
 */
void CopySetupReqPkg( const UINT8 *pReqPkt )
{
    UINT8 i;

    for ( i = 0; i != sizeof( USB_SETUP_REQ ); i ++ )
    {
        ((PUINT8)pUSBFS_SetupRequest)[ i ] = *pReqPkt;
        pReqPkt++;
    }
}

/*********************************************************************
 * @fn      HostCtrlTransfer
 *
 * @brief   Host control transfer.
 *
 * @param   databuf - Receiving or send data buffer.
 *          RetLen - Data length.
 *
 * @return  Error state
 */
UINT8 HostCtrlTransfer(PUINT8 databuf,PUINT8 len)
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
    USBOTG_H_FS->HOST_TX_LEN = 8;
    ret = USBFSH_Transact( (USB_PID_SETUP<<4)|DEF_ENDP_0, 0, 200000 );
    if(ret != ERR_SUCCESS)      return ( ret );                     //error

    relen = pUSBFS_SetupRequest->wLength;

    if(relen && pBuf)                                           //data stage
    {
       if( (pUSBFS_SetupRequest->bRequestType) & USB_REQ_TYP_IN )            //device to host
       {
           while(relen)
           {
               if( USBFS_thisUsbDev.DeviceSpeed != USB_HIGH_SPEED )
                   Delay_Us( 100 );
               USBOTG_H_FS->HOST_RX_DMA = (UINT32)databuf + *pLen;
               ret = USBFSH_Transact( (USB_PID_IN<<4)| DEF_ENDP_0, tog<<3, 20000 );
               if(ret != ERR_SUCCESS)                return ( ret );
               tog ^=1;
               rxlen = (USBOTG_H_FS->RX_LEN < relen) ? USBOTG_H_FS->RX_LEN : relen;
               relen -= rxlen;
               if(pLen)  *pLen += rxlen;
               if( ( USBOTG_H_FS->RX_LEN == 0 ) || (USBOTG_H_FS->RX_LEN & ( USBFS_UsbDevEndp0Size - 1 )))  break;
            }
           USBOTG_H_FS->HOST_TX_LEN = 0 ;
         }
       else
       {                                                           // host to device
          while(relen)
          {
              if( USBFS_thisUsbDev.DeviceSpeed != USB_HIGH_SPEED )
                  Delay_Us( 100 );
              USBOTG_H_FS->HOST_TX_DMA = (UINT32)databuf + *pLen;
               USBOTG_H_FS->HOST_TX_LEN = (relen >= USBFS_UsbDevEndp0Size)? USBFS_UsbDevEndp0Size : relen;

               ret = USBFSH_Transact((USB_PID_OUT<<4)|DEF_ENDP_0,  tog<<3,  20000);
               if(ret != ERR_SUCCESS)               return  ( ret );
               tog ^=1;
               relen -= USBOTG_H_FS->HOST_TX_LEN;
               if( pLen )  *pLen += USBOTG_H_FS->HOST_TX_LEN;
          }
        }
    }
    if( USBFS_thisUsbDev.DeviceSpeed != USB_HIGH_SPEED )
        Delay_Us( 100 );
    ret = USBFSH_Transact( ((USBOTG_H_FS->HOST_TX_LEN) ? USB_PID_IN<<4|DEF_ENDP_0 : USB_PID_OUT<<4|DEF_ENDP_0),
            USBFS_UH_R_TOG | USBFS_UH_T_TOG, 20000 );
    if(ret != ERR_SUCCESS)            return( ret );

    if ( USBOTG_H_FS->HOST_TX_LEN == 0 )   return( ERR_SUCCESS );    //status stage is out, send a zero-length packet.

    if ( USBOTG_H_FS->RX_LEN == 0 )        return( ERR_SUCCESS );    //status stage is in, a zero-length packet is returned indicating success.

    return ERR_USB_BUF_OVER;
}

/*********************************************************************
 * @fn      CtrlGetDevDescr
 *
 * @brief   Get device descrptor
 *
 * @return  Error state
 */
UINT8 CtrlGetDevDescr( UINT8 *Databuf )
{
    UINT8 ret;
    UINT16 len;

    USBFS_UsbDevEndp0Size = 8;
    CopySetupReqPkg( GetDevDescrptor );
    pUSBFS_SetupRequest->wLength = USBFS_UsbDevEndp0Size;
    ret = USBFSH_CtrlTransfer( USBFS_UsbDevEndp0Size , Databuf, &len );

    if( ret != ERR_SUCCESS )                     return  ( ret );
    USBFS_UsbDevEndp0Size = ((PUSB_DEV_DESCR)Databuf)->bMaxPacketSize0;

    CopySetupReqPkg( GetDevDescrptor );                               //获取全部设备描述符
    ret = USBFSH_CtrlTransfer( USBFS_UsbDevEndp0Size , Databuf, &len );
    if( ret != ERR_SUCCESS )                     return  ( ret );
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
UINT8 CtrlSetAddress(UINT8 addr)
{
    UINT8 ret;

    CopySetupReqPkg( SetAddress );
    pUSBFS_SetupRequest->wValue = addr;
    ret = HostCtrlTransfer( NULL, NULL );
    if(ret != ERR_SUCCESS)  return  ( ret );
    USBFS_CurrentAddr( addr );
    Delay_Ms(5);
    return  ERR_SUCCESS;
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
void Analysis_Descr(pUSBDEV_INFO pusbdev,PUINT8 pdesc, UINT16 l)
{
    UINT16 i,j=0;

    for( i=0; i<l; i++ )                                                //分析描述符
    {
         if((pdesc[i]==0x09)&&(pdesc[i+1]==0x02))
         {
                printf("bNumInterfaces:%02x \n",pdesc[i+4]);            //配置描述符里的接口数-第5个字节
         }
         if((pdesc[i]==0x09)&&(pdesc[i+1]==0x04))
         {
             printf("device_type:%02x \n",pdesc[i+5]);                 //接口类型
             pusbdev->DeviceType = pdesc[i+5];
         }
         if((pdesc[i]==0x09)&&(pdesc[i+1]==0x21))                       //报表描述符长度
         {
              Hid_Report_Len = ((UINT16)pdesc[i+8]<<8)|pdesc[i+7];
              printf("Hid_Report_Len:%02x \n",Hid_Report_Len);                 //接口类型
         }
         if((pdesc[i]==0x07)&&(pdesc[i+1]==0x05))
         {
            if((pdesc[i+2])&0x80)
            {
                 printf("endpIN:%02x \n",pdesc[i+2]&0x0f);              //取in端点号
                 pusbdev->DevEndp.InEndpNum[j++] = pdesc[i+2]&0x0f;
                 pusbdev->DevEndp.InEndpCount++;
                 USBFS_EndpnMaxSize = ((UINT16)pdesc[i+5]<<8)|pdesc[i+4];     //取端点大小
                 pusbdev->DevEndp.InEndpMaxSize = USBFS_EndpnMaxSize;

                 pusbdev->DevEndp.Ininterval = pdesc[i+6];

                 printf("In_endpmaxsize:%02x \n",USBFS_EndpnMaxSize);
            }
            else
            {
                printf("endpOUT:%02x \n",pdesc[i+2]&0x0f);              //取out端点号
                pusbdev->DevEndp.OutEndpNum = pdesc[i+2]&0x0f;
                pusbdev->DevEndp.OutEndpCount++;
                USBFS_EndpnMaxSize =((UINT16)pdesc[i+5]<<8)|pdesc[i+4];        //取端点大小
                pusbdev->DevEndp.OutEndpMaxSize = USBFS_EndpnMaxSize;
                printf("Out_endpmaxsize:%02x \n",USBFS_EndpnMaxSize);
            }
        }
  }
}

/*********************************************************************
 * @fn      HubAnalysis_Descr
 *
 * @brief   Descriptor analysis.
 *
 * @return  none
 */
void HubAnalysis_Descr(PHUB_Port_Info portn,PUINT8 pdesc, UINT16 l)
{
    UINT16 i;
    uint8_t endp_num;
    endp_num = 0;
    for( i=0; i<l; i++ )                                                //分析描述符
    {
         if((pdesc[i]==0x09)&&(pdesc[i+1]==0x02))
         {
                printf("bNumInterfaces:%02x \n",pdesc[i+4]);            //配置描述符里的接口数-第5个字节
         }
         if((pdesc[i]==0x09)&&(pdesc[i+1]==0x04))
         {
              printf("device_type:%02x \n",pdesc[i+5]);                 //接口类型
              portn->DeviceType = pdesc[i+5];
              portn->DeviceHIDType = pdesc[i+7];

         }
         if((pdesc[i]==0x09)&&(pdesc[i+1]==0x21))                       //报表描述符长度
         {
              Hid_Report_Len = ((UINT16)pdesc[i+8]<<8)|pdesc[i+7];
              printf("Hid_Report_Len:%02x \n",Hid_Report_Len);                 //接口类型
         }
         if((pdesc[i]==0x07)&&(pdesc[i+1]==0x05))
         {
            if((pdesc[i+2])&0x80)
            {
                 printf("endpIN:%02x \n",pdesc[i+2]&0x0f);              //取in端点号
                 portn->portEndp[ endp_num ].Num = pdesc[i+2];
                 portn->portEndp[ endp_num ].EndpType = pdesc[i+3];
                 if(((pdesc[i+5])&0x18) && (portn->Speed == 0x01) )     //高速高带宽端点
                 {
                     USBFS_EndpnMaxSize = ((UINT16)(pdesc[i+5]&0x07)<<8)|pdesc[i+4]; //取端点大小
                     portn->portEndp[ endp_num ].HighTransNum = ((pdesc[i+5]&0x18)>>3); //  一个微帧内事务数
                 }
                 else
                 {
                     USBFS_EndpnMaxSize = ((UINT16)pdesc[i+5]<<8)|pdesc[i+4];     //取端点大小
                     portn->portEndp[ endp_num ].HighTransNum = 0;
                 }
                 portn->portEndp[ endp_num ].Endp_Size = USBFS_EndpnMaxSize;
                 portn->portEndp[ endp_num ].tog = 0;
                 endp_num++;
                 portn->EndpNum = endp_num;
            }
            else
            {
                printf("endpOUT:%02x \n",pdesc[i+2]&0x0f);              //取out端点号
                USBFS_EndpnMaxSize = ((UINT16)pdesc[i+5]<<8)|pdesc[i+4];     //取端点大小
                portn->portEndp[ endp_num ].Num = pdesc[i+2];
                portn->portEndp[ endp_num ].EndpType = pdesc[i+3];
                if(((pdesc[i+5])&0x18) && (portn->Speed == 0x01) )     //高速高带宽端点
                {
                    USBFS_EndpnMaxSize = ((UINT16)(pdesc[i+5]&0x07)<<8)|pdesc[i+4]; //取端点大小
                }
                else {
                    USBFS_EndpnMaxSize = ((UINT16)pdesc[i+5]<<8)|pdesc[i+4];     //取端点大小
                }
                portn->portEndp[ endp_num ].Endp_Size = USBFS_EndpnMaxSize;
                portn->portEndp[ endp_num ].tog = 0;
                endp_num++;
                portn->EndpNum = endp_num;
                printf("Out_endpmaxsize:%02x \n",USBFS_EndpnMaxSize);
            }
        }
  }
}

/*********************************************************************
 * @fn      CtrlGetConfigDescr
 *
 * @brief   Get configuration descriptor.
 *
 * @return  Error state
 */

UINT8 CtrlGetConfigDescr( PHUB_Port_Info phub, UINT8 *Databuf )
{
    UINT8  ret;
    UINT16  len;
    UINT16 reallen;

    CopySetupReqPkg( GetConfigDescrptor );
    ret =  USBFSH_CtrlTransfer(USBFS_UsbDevEndp0Size , Databuf, &len);
    if(ret != ERR_SUCCESS)             return  ( ret );
    if(len < ( pUSBFS_SetupRequest->wLength ) )  return  ERR_USB_BUF_OVER;

    reallen = ((PUSB_CFG_DESCR)Databuf)-> wTotalLength;             //解析全部配置描述符的长度

    CopySetupReqPkg( GetConfigDescrptor );
    pUSBFS_SetupRequest->wLength = reallen;
    ret =  USBFSH_CtrlTransfer(USBFS_UsbDevEndp0Size , Databuf, &len);    //获取全部配置描述符
    if( ret != ERR_SUCCESS )           return  ( ret );

    USBFS_thisUsbDev.DeviceCongValue = ( (PUSB_CFG_DESCR)Databuf )-> bConfigurationValue;
    printf("USBFS_thisUsbDev.DeviceType=%02x\n",USBFS_thisUsbDev.DeviceType);
    if( USBFS_thisUsbDev.DeviceType == 0x09 )
    {        //HUB处理
        HubAnalysis_Descr( phub,(UINT8 *)Databuf, pUSBFS_SetupRequest->wLength );
    }
    else
    {                   //其他设备处理
        Analysis_Descr( &USBFS_thisUsbDev, (UINT8 *)Databuf, pUSBFS_SetupRequest->wLength );
    }
    return( ERR_SUCCESS );
}

/*********************************************************************
 * @fn      CtrlSetUsbConfig
 *
 * @brief   Set usb configration.
 *
 * @param   cfg_val - device configuration value
 *
 * @return  Error state
 */
UINT8 CtrlSetUsbConfig( UINT8 cfg_val)
{
    CopySetupReqPkg( SetConfig );
    pUSBFS_SetupRequest->wValue = cfg_val;
    return( HostCtrlTransfer( NULL, NULL ));
}

/*********************************************************************
 * @fn      CtrlSetIdle
 *
 * @brief   Set idle
 *
 * @param   None
 *
 * @return  Error state
 */
UINT8 CtrlSetIdle( void )
{
    pUSBFS_SetupRequest->bRequestType = 0x21;
    pUSBFS_SetupRequest->bRequest = 0x0a;
    pUSBFS_SetupRequest->wIndex = 0;
    pUSBFS_SetupRequest->wLength = 0;
    pUSBFS_SetupRequest ->wValue = 0;
    return( HostCtrlTransfer( NULL, NULL ));
}

/*********************************************************************
 * @fn      CtrlSetIdle
 *
 * @brief   Set idle
 *
 * @param   None
 *
 * @return  Error state
 */
uint8_t CtrlGetReportDescr( UINT8 *Databuf )
{
    pUSBFS_SetupRequest->bRequestType = 0x81;
    pUSBFS_SetupRequest->bRequest = 0x06;
    pUSBFS_SetupRequest ->wValue = 0x2200;
    pUSBFS_SetupRequest->wIndex = 0;
    pUSBFS_SetupRequest->wLength = Hid_Report_Len;

    return(USBFSH_CtrlTransfer(USBFS_UsbDevEndp0Size, Databuf, &Hid_Report_Len));
}

/*********************************************************************
 * @fn      Ctrlclearendpstall
 *
 * @brief   clear endpoint stall.
 *
 * @return  Error state
 */
UINT8 Ctrlclearendpstall( )
{
    CopySetupReqPkg( Clear_EndpStall );
    return( HostCtrlTransfer( NULL, NULL ));
}

/*********************************************************************
 * @fn      CtrlSetUsbConfig
 *
 * @brief   get HUB description
 *
 * @param   cfg_val - device configuration value
 *
 * @return  Error state
 */
UINT8 CtrlGet_HUBDevDescr( UINT8 *Databuf )
{
    UINT8 ret;
    UINT16 len;
    UINT8 buf[8] =
    {
       0xA0,0x06,0x00,0x29,0x00,0x00,0x02,0x00
    };
    CopySetupReqPkg( buf );
    pUSBFS_SetupRequest->wLength = USBFS_UsbDevEndp0Size;
    ret = USBFSH_CtrlTransfer(USBFS_UsbDevEndp0Size , Databuf, &len);
    if( ret != ERR_SUCCESS )                     return  ( ret );
    buf[6] = *Databuf;
    CopySetupReqPkg( buf );                               //获取全部设备描述符
    ret = USBFSH_CtrlTransfer(USBFS_UsbDevEndp0Size , Databuf, &len);
    if( ret != ERR_SUCCESS )                     return  ( ret );
    return( ERR_SUCCESS );
}

/*********************************************************************
 * @fn      HubSetPortFeature
 *
 * @brief   set the port feature of hub
 *
 * @param   HubPortIndex - index of the hub port index
 *          FeatureSelt - feature selector
 *
 * @return  Error state
 */
UINT8   HubSetPortFeature( UINT8 HubPortIndex, UINT8 FeatureSelt )
{
    pUSBFS_SetupRequest -> bRequestType = HUB_SET_PORT_FEATURE;
    pUSBFS_SetupRequest -> bRequest = HUB_SET_FEATURE;
    pUSBFS_SetupRequest -> wValue = 0x0000|FeatureSelt;
    pUSBFS_SetupRequest -> wIndex = 0x0000|HubPortIndex;
    pUSBFS_SetupRequest -> wLength = 0x0000;
    return(USBFSH_CtrlTransfer(USBFS_UsbDevEndp0Size , NULL, NULL));
}

/*********************************************************************
 * @fn      USBH_ResetRootHubPort
 *
 * @brief   Reset USB port.
 *
 * @para    index: USB host port
 *          mod: Reset host port operating mode.
 *               0 -> reset and wait end
 *               1 -> begin reset
 *               2 -> end reset
 *
 * @return  none
 */
void USBH_ResetRootHubPort( uint8_t usb_port, uint8_t mode )
{
    if( usb_port == DEF_USBFS_PORT_INDEX )
    {
#if DEF_USBFS_PORT_EN
        USBFSH_ResetRootHubPort( mode );
#endif
    }
    else if( usb_port == DEF_USBHS_PORT_INDEX )
    {
#if DEF_USBHS_PORT_EN
        USBHSH_ResetRootHubPort( mode );
#endif
    }
}

/*********************************************************************
 * @fn      USBH_EnableRootHubPort
 *
 * @brief   Enable USB host port.
 *
 * @para    index: USB host port
 *
 * @return  none
 */
uint8_t USBH_EnableRootHubPort( uint8_t usb_port )
{
    uint8_t s = ERR_USB_UNSUPPORT;

    if( usb_port == DEF_USBFS_PORT_INDEX )
    {
#if DEF_USBFS_PORT_EN
        s = USBFSH_EnableRootHubPort( &RootHubDev[ usb_port ].bSpeed );
#endif
    }
    else if( usb_port == DEF_USBHS_PORT_INDEX )
    {
#if DEF_USBHS_PORT_EN
        s = USBHSH_EnableRootHubPort( &RootHubDev[ usb_port ].bSpeed );
#endif
    }

    return s;
}

/*********************************************************************
 * @fn      USBFS_HostEnum
 *
 * @brief   FS port enumeration device.
 *
 * @para    depth - hub depth
 *          Databuf - data buffer
 *
 * @return  none
 */
UINT8 USBFS_HostEnum( UINT8 depth, UINT8 *Databuf )
{
  UINT8 ret,i;
  uint8_t  s;
  /* Reset the USB device and wait for the USB device to reconnect */
  USBH_ResetRootHubPort( 0, 0 );
  for( i = 0, s = 0; i < DEF_RE_ATTACH_TIMEOUT; i++ )
  {
      if( USBH_EnableRootHubPort( 0 ) == ERR_SUCCESS )
      {
          i = 0;
          s++;
          if( s > 6 )
          {
              break;
          }
      }
      Delay_Ms( 1 );
  }

  USBFS_CurrentAddr(0x00);
  Delay_Ms(10);

  ret = CtrlGetDevDescr( Databuf );
  if( ret != ERR_SUCCESS )
  {
      printf("get device descriptor:%02x\n",ret);
      return( ret );
  }

  USBFS_HubInfo[depth].DevAddr = ((PUSB_SETUP_REQ)SetAddress)->wValue;           //HUB的地址
  ret = CtrlSetAddress( ((PUSB_SETUP_REQ)SetAddress)->wValue );
  if(ret != ERR_SUCCESS)
  {
      printf("set address:%02x\n",ret);
      return( ret );
  }
  ret = CtrlGetConfigDescr(NULL, Databuf );
  if(ret != ERR_SUCCESS)
  {
      printf("get configuration descriptor:%02x\n",ret);
      return( ret );
  }
  ret = CtrlSetUsbConfig( 0x01 );
  if( ret != ERR_SUCCESS )
  {
      printf("set configuration:%02x\n",ret);
      return( ret );
  }

  if( USBFS_thisUsbDev.DeviceType == 0x03 )
  {
      CtrlSetIdle( );

      ret = CtrlGetReportDescr( Databuf );
      if( ret != ERR_SUCCESS )return ret;
  }
  else if( USBFS_thisUsbDev.DeviceType == 0x09 )
  {      //HUB
      ret = CtrlGet_HUBDevDescr( Databuf );
      if( ret != ERR_SUCCESS )
      {
          printf("get_hub_desc:%02x\n",ret);
          return( ret );
      }
      for( i=0;i!=4;i++ ){
          ret = HubSetPortFeature( i+1,0x08 );
          printf("hub_power_status=%02x,%d\n",ret,i);
          if( ret != ERR_SUCCESS )return ret;
      }
  }
  return( ERR_SUCCESS );
}

/*********************************************************************
 * @fn      HubGetPortStatus
 *
 * @brief   Check the port of hub,and return the port's status
 *
 * @param   HubPortIndex - index of the hub port index
 *
 * @return  Error state
 */
UINT8 HubGetPortStatus( UINT8 depth ,UINT8 HubPortIndex )
{
    UINT8   s;
    UINT16  len;
    UINT16 Port_Change_Field;


    pUSBFS_SetupRequest -> bRequestType = HUB_GET_PORT_STATUS;
    pUSBFS_SetupRequest -> bRequest = HUB_GET_STATUS;
    pUSBFS_SetupRequest -> wValue = 0x0000;
    pUSBFS_SetupRequest -> wIndex = 0x0000|HubPortIndex;
    pUSBFS_SetupRequest -> wLength = 0x0004;
    s = USBFSH_CtrlTransfer(USBFS_UsbDevEndp0Size , USBFS_endpRXbuf, &len);


    if ( s != ERR_SUCCESS )
    {
        return( s );
    }
    if ( len < 4 )
    {
        return( ERR_USB_BUF_OVER );                  // 描述符长度错误
    }

    Port_Change_Field = USBFS_endpRXbuf[2];
    Port_Change_Field |= ((UINT16)USBFS_endpRXbuf[3]<<8);

    USBFS_HubInfo[depth].portD[HubPortIndex-1].PortpChangeField = Port_Change_Field;

    return( ERR_SUCCESS );
}

/*********************************************************************
 * @fn      HUB_CheckPortConnect
 *
 * @brief   Check port connect
 *
 * @return  Error state
 */
UINT8 HUB_CheckPortConnect( UINT8 depth,UINT8 port , UINT8 dev_exist)
{
    UINT8 ret;
    ret = HubGetPortStatus( depth,port+1 );
    if( ret != ERR_SUCCESS )return ret;
    /* 判断当前端口连接状态 */
    if( USBFS_endpRXbuf[ 2 ] & 0x01 )                                                      /* 该端口连接状态发生改变 */
    {
        if( USBFS_endpRXbuf[ 0 ] & 0x01 )
        {
            if( USBFS_HubInfo[depth].portD[port].Status < HUB_ERR_SCUESS )
            {
                USBFS_HubInfo[depth].portD[port].Status = HUB_ERR_CONNECT;
            }
            return( 0x18 );                                    /* 该端口：检测到设备连接 */
        }
        else
        {
            USBFS_HubInfo[depth].portD[port].Status = HUB_ERR_DISCONNECT;
            USBFS_HubInfo[depth].portD[port].Status = 0;
            return( 0x19 );                                 /* 该端口：检测到设备断开 */
        }
    }
    else                                                                        /* 该端口连接状态未发生改变 */
    {
        if( ((USBFS_endpRXbuf[ 0 ] & 0x01) && dev_exist == 0) || (USBFS_endpRXbuf[ 0 ] == 0x03) )
        {
            if( USBFS_HubInfo[depth].portD[port].Status < HUB_ERR_SCUESS )
            {
                USBFS_HubInfo[depth].portD[port].Status = HUB_ERR_CONNECT;
            }
            return( 0x02 );                                                     /* 该端口：有设备 */
        }
        else
        {
            USBFS_HubInfo[depth].portD[port].Status = HUB_ERR_DISCONNECT;
            USBFS_HubInfo[depth].portD[port].Status = 0;
            return( 0x01 );                                                     /* 该端口：无设备 */
        }
    }

}

/*********************************************************************
 * @fn      HubClearPortFeature
 *
 * @brief   clear the port feature of hub
 *
 * @param   HubPortIndex - index of the hub port index
 *          FeatureSelt - feature selector
 *
 * @return  Error state
 */
UINT8 HOST_ClearPortFeature_Process( UINT8 HubPortIndex, UINT8 FeatureSelt )
{
    pUSBFS_SetupRequest -> bRequestType = HUB_CLEAR_PORT_FEATURE;
    pUSBFS_SetupRequest -> bRequest = HUB_CLEAR_FEATURE;
    pUSBFS_SetupRequest -> wValue = 0x0000|FeatureSelt;
    pUSBFS_SetupRequest -> wIndex = 0x0000|HubPortIndex;
    pUSBFS_SetupRequest -> wLength = 0x0000;
    return( HostCtrlTransfer( NULL, NULL ) );     // 执行控制传输
}

/*********************************************************************
 * @fn      HubClearPortFeature
 *
 * @brief   Hub Clear Port Feature
 *
 * @return  Error state
 */
UINT8 HubClearPortFeature( UINT8 depth,UINT8 port )
{
    UINT8 status = 0;
    if( USBFS_HubInfo[depth].portD[port-1].PortpChangeField&0x01 )
    {
        status = HOST_ClearPortFeature_Process( port,16 );
        if( status == ERR_SUCCESS )return status;
    }
    if( USBFS_HubInfo[depth].portD[port-1].PortpChangeField&0x02 )
    {
        status = HOST_ClearPortFeature_Process( port,17 );
        if( status == ERR_SUCCESS )return status;
    }
    if( USBFS_HubInfo[depth].portD[port-1].PortpChangeField&0x04 )
    {
        status = HOST_ClearPortFeature_Process( port,18 );
        if( status == ERR_SUCCESS )return status;
    }
    if( USBFS_HubInfo[depth].portD[port-1].PortpChangeField&0x08 )
    {
        status = HOST_ClearPortFeature_Process( port,19 );
        if( status == ERR_SUCCESS )return status;
    }
    if( USBFS_HubInfo[depth].portD[port-1].PortpChangeField&0x10 )
    {
        status = HOST_ClearPortFeature_Process( port,20 );
        if( status == ERR_SUCCESS )return status;
    }
    return status;
}

/*********************************************************************
 * @fn      HubClearPortFeature
 *
 * @brief   HID Get Report Descriptor
 *
 * @return  Error state
 */
UINT8 HubCtrlGetHidDescr( PHUB_Port_Info phub,UINT8 *Databuf )
{
    UINT8  ret;
    UINT16  len;

    pUSBFS_SetupRequest->bRequestType = 0x81;
    pUSBFS_SetupRequest->bRequest = 0x06;
    pUSBFS_SetupRequest->wValue = 0x2200;
    pUSBFS_SetupRequest->wIndex = 0x0000;
    pUSBFS_SetupRequest->wLength = Hid_Report_Len;
    ret = USBFSH_CtrlTransfer( 8,Databuf,&len );
    if(ret != ERR_SUCCESS)             return  ( ret );
    if(len < ( pUSBFS_SetupRequest->wLength ) )  return  ERR_USB_BUF_OVER;
    return ret;
}

/*********************************************************************
 * @fn      HubUSBSetIdle
 *
 * @brief   set idle
 *
 * @return  state
 */
UINT8 HubUSBSetIdle( PHUB_Port_Info phub, UINT8 *Databuf )
{
    UINT8 ret;
    pUSBFS_SetupRequest->bRequestType = 0x21;
    pUSBFS_SetupRequest->bRequest = 0x0a;
    pUSBFS_SetupRequest->wValue = 0x00;
    pUSBFS_SetupRequest->wIndex = 0x0000;
    pUSBFS_SetupRequest->wLength = 0x0000;
    ret = USBFSH_CtrlTransfer( 8, Databuf, NULL );
    if ( ret != ERR_SUCCESS )         return( 1 );
    return ERR_SUCCESS;
}

/*********************************************************************
 * @fn      HubUSBSetReport
 *
 * @brief   set report
 *
 * @return  state
 */
UINT8 HubUSBSetReport( PHUB_Port_Info phub, UINT8 *Databuf,UINT16 *len )
{
    UINT8 ret;
    pUSBFS_SetupRequest->bRequestType = 0x21;
    pUSBFS_SetupRequest->bRequest = 0x09;
    pUSBFS_SetupRequest->wValue = 0x0200;
    pUSBFS_SetupRequest->wIndex = 0x0000;
    pUSBFS_SetupRequest->wLength = 0x0001;
    ret = USBFSH_CtrlTransfer( 8, Databuf, len );
    return ret;
}

/*********************************************************************
 * @fn      USBFS_setdevspeed
 *
 * @brief   set speed
 *
 * @return  state
 */
void USBFS_setdevspeed( UINT8 speed )
{
    if( speed == 0 )
    {
        USBOTG_H_FS->BASE_CTRL &= ~USBFS_UC_LOW_SPEED;
        USBOTG_H_FS->HOST_CTRL &= ~USBFS_UH_LOW_SPEED;
        USBOTG_H_FS->HOST_SETUP &= ~USBFS_UH_PRE_PID_EN;
    }
    else
    {
        USBOTG_H_FS->BASE_CTRL |= USBFS_UC_LOW_SPEED;
        USBOTG_H_FS->HOST_CTRL |= USBFS_UH_LOW_SPEED;
        USBOTG_H_FS->HOST_SETUP |= USBFS_UH_PRE_PID_EN;
    }

}

/*********************************************************************
 * @fn      USBFS_HUBHostEnum
 *
 * @brief   Enumerating devices
 *
 * @return  state
 */
UINT8 USBFS_HUBHostEnum( UINT8 depth, UINT8 *Databuf,UINT8 port )
{
  UINT8 ret;
  UINT16 i;
  UINT16 temp16;
  Delay_Ms( 10 );
  ret= HubGetPortStatus( depth,port+1 );
  if( ret != ERR_SUCCESS )
  {
      return( ret );
  }
  ret = HubSetPortFeature( port + 1,0x04 );       //复位下面的设备
  if( ret != ERR_SUCCESS )
  {
      return( ret );
  }
  /* 查询当前复位端口,直到复位完成,把完成后的状态显示出来 */
  do
  {
      Delay_Ms( 10 );
      ret = HubGetPortStatus( depth,port+1 );
      if( ret != ERR_SUCCESS )
      {
          return( ret );
      }
  }while( USBFS_endpRXbuf[ 0 ] & 0x10 );
  Delay_Ms( 30 );
  ret = HubClearPortFeature( depth,port + 1);
  if( ret != ERR_SUCCESS )
  {
      return( ret );
  }
  ret= HubGetPortStatus( depth,port+1 );
  if( ret != ERR_SUCCESS )
  {
      return( ret );
  }


  for( ret=0;ret!=4;ret++ ){
      printf("%02x ",USBFS_endpRXbuf[ret]);
  }
  printf("\n");
  if( ( USBFS_endpRXbuf[ 0 ] & 0x01 ) == 0x00 )
  {
      return( 0x18 );
  }
  temp16 = *((uint16_t *)&USBFS_endpRXbuf[0]);
  if( temp16 & 0x01 ){              //低全速
      USBFS_HubInfo[depth].portD[port].Num = port+1;
      USBFS_HubInfo[depth].portD[port].Addr = 0;
      USBFS_HubInfo[depth].portD[port].Status = 1;
      USBFS_HubInfo[depth].portD[port].Speed = (temp16&(1<<9))?2:((temp16&(1<<10))?1:0);//1--高速，2--低速，0--全速
  }
  else USBFS_HubInfo[depth].portD[port].Status = 0;            //高速
  printf("Speed=%02x,%02x\n",USBFS_HubInfo[depth].portD[port].Speed,USBFS_HubInfo[depth].portD[port].Status);


    USBFS_CurrentAddr(0);//先设置地址0
    USBFS_setdevspeed(USBFS_HubInfo[depth].portD[port].Speed);//切换全速-低速
    if( USBFS_HubInfo[depth].Speed != 0x02 )
    {
        USBOTG_H_FS->HOST_CTRL &= ~USBFS_UH_LOW_SPEED;
    }

      ret = CtrlGetDevDescr( Databuf );
      if( ret != ERR_SUCCESS )
      {
          printf("get device descriptor:%02x\n",ret);
          return( ret );
      }

      USBFS_HubInfo[depth].portD[port].Addr = USBFS_Set_AddressNumber();
      ret = CtrlSetAddress( USBFS_HubInfo[depth].portD[port].Addr );           //设置地址
      if(ret != ERR_SUCCESS)
      {
          printf("set address:%02x,%d\n",ret,port+5);
          return( ret );
      }
      ret = CtrlGetConfigDescr( &USBFS_HubInfo[depth].portD[port],Databuf );                          //高速也要进行数据分析
      if(ret != ERR_SUCCESS)
      {
          printf("get configuration descriptor:%02x\n",ret);
          return( ret );
      }
      ret = CtrlSetUsbConfig( USBFS_thisUsbDev.DeviceCongValue );
      if( ret != ERR_SUCCESS )
      {
          printf("set configuration:%02x\n",ret);
          return( ret );
      }
      if( USBFS_HubInfo[depth].portD[port].DeviceType == 0x03 ){             //HID设备类
          ret = HubUSBSetIdle( &USBFS_HubInfo[depth].portD[port],Databuf );  //SET_IDLE
          printf("set_idle=%02x\n",ret);
          ret = HubCtrlGetHidDescr( &USBFS_HubInfo[depth].portD[port],Databuf ); //获取报表描述符
          printf("get_report=%02x\n",ret);
      }
      if( USBFS_HubInfo[depth].portD[port].DeviceType == 0x09 ){         //HUB的初始化
          ret = CtrlGet_HUBDevDescr( Databuf );
          if( ret != ERR_SUCCESS )
          {
              printf("get_hub_desc:%02x\n",ret);
              return( ret );
          }
          for( i=0;i!=4;i++ ){
              ret = HubSetPortFeature( i+1,0x08 );
              printf("hub_power_status=%02x,%d\n",ret,i);
              if( ret != ERR_SUCCESS )return ret;
          }
          USBFS_HubInfo[depth+1].DevAddr = USBFS_HubInfo[depth].portD[port].Addr;   //下级HUB的根目录地址23-05-24
      }
      USBFS_HubInfo[depth].portD[port].Status = HUB_ERR_SCUESS;          //枚举成功
      USBOTG_H_FS->HOST_RX_DMA = (UINT32)USBFS_endpRXbuf;
      USBOTG_H_FS->HOST_TX_DMA = (UINT32)USBFS_endpTXbuf;              //回复控制请求地址
      return( ERR_SUCCESS );

}

/*********************************************************************
 * @fn      USBFS_INT_In_Data
 *
 * @brief   To interrupt endpoint IN data
 *
 * @return  state
 */
UINT8 USBFS_INT_In_Data( PHUB_Port_Info phub,UINT8 num_port,UINT8 *pdata,UINT16 *len,UINT8 endp,UINT8 *tog )
{

    UINT8 s;
    USBFS_CurrentAddr(phub->Addr);
    USBOTG_H_FS->HOST_RX_DMA = (UINT32)pdata;
    s = USBFSH_Transact( (USB_PID_IN << 4) | (endp&0x0f) , *tog, 0 );
    if(s != ERR_SUCCESS) return ( s );
    *len = USBOTG_H_FS->RX_LEN ;
    return s;
}

/*******************************************************************************
 * @fn        USBFS_Set_AddressNumber
 *
 * @briefI    Number of assigned addresses
 *
 * @param     None
 *
 * @return    Number of addresses to be set
 */
UINT8 USBFS_Set_AddressNumber(void)
{
    UINT8 i = 0;
    for (i = DEVICE_ADDR+1; i < 127; i++)
    {
        if( USBFS_AddressNum[i] == 0 )
        {
            USBFS_AddressNum[i] = 1;
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
UINT8 USBFS_Del_AddressNumber(UINT8 addr)
{
    if( USBFS_AddressNum[addr] == 1)
    {
        USBFS_AddressNum[addr] = 0;
        return 0;//success
    }

    return 1;//fail
}

/*********************************************************************
 * @fn      USBH_GetConfigDescr
 *
 * @brief   Get the configuration descriptor of the USB device.
 *
 * @para    index: USB host port
 *
 * @return  none
 */
uint8_t USBH_GetConfigDescr( uint8_t usb_port, uint16_t *pcfg_len )
{
    uint8_t s = ERR_USB_UNSUPPORT;

    if( usb_port == DEF_USBFS_PORT_INDEX )
    {
#if DEF_USBFS_PORT_EN
        s = USBFSH_GetConfigDescr( RootHubDev[ usb_port ].bEp0MaxPks, Com_Buf, DEF_COM_BUF_LEN, pcfg_len );
#endif
    }
    else if( usb_port == DEF_USBHS_PORT_INDEX )
    {
#if DEF_USBHS_PORT_EN
        s = USBHSH_GetConfigDescr( RootHubDev[ usb_port ].bEp0MaxPks, Com_Buf, DEF_COM_BUF_LEN, pcfg_len );
#endif
    }

    return s;
}

/*********************************************************************
 * @fn      USBH_SetUsbAddress
 *
 * @brief   Set USB device address.
 *
 * @para    index: USB host port
 *
 * @return  none
 */
uint8_t USBH_SetUsbAddress( uint8_t usb_port )
{
    uint8_t s = ERR_USB_UNSUPPORT;

    if( usb_port == DEF_USBFS_PORT_INDEX )
    {
#if DEF_USBFS_PORT_EN
        RootHubDev[ usb_port ].bAddress = (uint8_t)( DEF_USBFS_PORT_INDEX + USB_DEVICE_ADDR );
        s = USBFSH_SetUsbAddress( RootHubDev[ usb_port ].bEp0MaxPks, RootHubDev[ usb_port ].bAddress );
#endif
    }
    else if( usb_port == DEF_USBHS_PORT_INDEX )
    {
#if DEF_USBHS_PORT_EN
        RootHubDev[ usb_port ].bAddress = (uint8_t)( DEF_USBHS_PORT_INDEX + USB_DEVICE_ADDR );
        s = USBHSH_SetUsbAddress( RootHubDev[ usb_port ].bEp0MaxPks, RootHubDev[ usb_port ].bAddress );
#endif
    }

    return s;
}

/*********************************************************************
 * @fn      USBH_AnalyseType
 *
 * @brief   Simply analyze USB device type.
 *
* @para     pdev_buf: Device descriptor buffer
 *          pcfg_buf: Configuration descriptor buffer
 *          ptype: Device type.
 *
 * @return  none
 */
void USBH_AnalyseType( uint8_t *pdev_buf, uint8_t *pcfg_buf, uint8_t *ptype )
{
    uint8_t  dv_cls, if_cls;

    dv_cls = ( (PUSB_DEV_DESCR)pdev_buf )->bDeviceClass;
    if_cls = ( (PUSB_CFG_DESCR_LONG)pcfg_buf )->itf_descr.bInterfaceClass;
    if( ( dv_cls == USB_DEV_CLASS_STORAGE ) || ( if_cls == USB_DEV_CLASS_STORAGE ) )
    {
        *ptype = USB_DEV_CLASS_STORAGE;
    }
    else if( ( dv_cls == USB_DEV_CLASS_PRINTER ) || ( if_cls == USB_DEV_CLASS_PRINTER ) )
    {
        *ptype = USB_DEV_CLASS_PRINTER;
    }
    else if( ( dv_cls == USB_DEV_CLASS_HID ) || ( if_cls == USB_DEV_CLASS_HID ) )
    {
        *ptype = USB_DEV_CLASS_HID;
    }
    else if( ( dv_cls == USB_DEV_CLASS_HUB ) || ( if_cls == USB_DEV_CLASS_HUB ) )
    {
        *ptype = USB_DEV_CLASS_HUB;
    }
    else
    {
        *ptype = DEF_DEV_TYPE_UNKNOWN;
    }
}

/*********************************************************************
 * @fn      USBFSH_SetUsbConfig
 *
 * @brief   Set USB configuration.
 *
 * @para    index: USB host port
 *
 * @return  none
 */
uint8_t USBH_SetUsbConfig( uint8_t usb_port, uint8_t cfg_val )
{
    uint8_t s = ERR_USB_UNSUPPORT;

    if( usb_port == DEF_USBFS_PORT_INDEX )
    {
#if DEF_USBFS_PORT_EN
        s = USBFSH_SetUsbConfig( RootHubDev[ usb_port ].bEp0MaxPks, cfg_val );
#endif
    }
    else if( usb_port == DEF_USBHS_PORT_INDEX )
    {
#if DEF_USBHS_PORT_EN
        s = USBHSH_SetUsbConfig( RootHubDev[ usb_port ].bEp0MaxPks, cfg_val );
#endif
    }

    return s;
}

/*********************************************************************
 * @fn      USBFS_HUB_Process
 *
 * @brief   usbfs hub main process.
 *
 * @return  none
 */
UINT8  USBFS_HUB_Process( UINT8 depth ,UINT8 addr ,UINT8 uplevelport)
{
  UINT8 i,ret,s,dev_exist = 0;
  UINT16 temp,len;
  USB_HUB_SaveData hubdata,modifyhubdata;
  Link_HUBSaveData *printhub = NULL;//用来打印

      for( i=0;i!=4;i++ )
      {
          USBOTG_H_FS->HOST_RX_DMA = (UINT32)USBFS_endpRXbuf;      //恢复控制请求地址
          USBOTG_H_FS->HOST_TX_DMA = (UINT32)USBFS_endpTXbuf;

          hubdata.Depth = depth;
          hubdata.UpLevelPort = uplevelport;
          hubdata.CurrentPort = i+1;
          hubdata.HUB_Info = USBFS_HubInfo[depth];
          ret = Hublink_SearchHubData(USBFS_Hub_LinkHead , &hubdata);//判断当前节点是否有保存的数据
          if( ret == 0 )
          {
              USBFS_HubInfo[depth] = hubdata.HUB_Info;//若有保存的数据则替换
              dev_exist = 1;
          }
          else
          {
              memset( &USBFS_HubInfo[depth].portD[i],0x00,sizeof( USBFS_HubInfo[depth].portD[i] ) );//没有保存的数据则清除 防止使用上一个HUB数据
          }

          USBFS_CurrentAddr(addr);          //设置HUB地址进行操作
          USBFS_setdevspeed(USBFS_HubInfo[depth].Speed);

          HUB_CheckPortConnect( depth, i, dev_exist);

          if( USBFS_HubInfo[depth].portD[i].PortpChangeField )
          {
              if( (USBFS_HubInfo[depth].portD[i].Status == HUB_ERR_CONNECT) )
              {            //设备连接

                  ret = USBFS_HUBHostEnum( depth,USBFS_test_buf,i );
                  if( ret == ERR_SUCCESS )
                  {
                      USBFS_HubInfo[depth].portD[i].Status = HUB_ERR_SCUESS;          //枚举成功

                      hubdata.Depth = depth;
                      hubdata.UpLevelPort = uplevelport;
                      hubdata.CurrentPort = i+1;
                      hubdata.HUB_Info = USBFS_HubInfo[depth];
                      ret = Hublink_InsertHubData(USBFS_Hub_LinkHead,hubdata);
                      if( ret == 0 )
                      {
                          printf("#Save: depth-%x,uplevelport-%x,port-%x,Status-%x,hubaddr-%x#\n",depth,uplevelport,i+1,USBFS_HubInfo[depth].portD[i].Status,USBFS_HubInfo[depth].portD[i].Addr);
#if 1
                          printhub = USBFS_Hub_LinkHead;
                          while(printhub != NULL)//打印当前链表剩余节点
                          {
                                printf("/----------------------------------/\n");
                                printf("printhub.HUB_SaveData.Depth=%x\n",printhub->HUB_SaveData.Depth);
                                printf("printhub.HUB_SaveData.UpLevelPort=%x\n",printhub->HUB_SaveData.UpLevelPort);
                                printf("printhub.HUB_SaveData.CurrentPort=%x\n",printhub->HUB_SaveData.CurrentPort);
                                printhub = printhub->next;
                          }
#endif
                      }
                  }
                  else
                  {
                      USBFS_HubInfo[depth].portD[i].Status = HUB_ERR_DISCONNECT;          //枚举失败.重新枚举
                  }
              }
              else if( (USBFS_HubInfo[depth].portD[i].Status ) == 0x00 )
              {
                  hubdata.Depth = depth;
                  hubdata.UpLevelPort = uplevelport;
                  hubdata.CurrentPort = i+1;
                  hubdata.HUB_Info = USBFS_HubInfo[depth];

                  if( hubdata.HUB_Info.portD[i].DeviceType == 0x09 )//如果是HUB，删除时需要判断是否有下级节点，如果有需要删除
                  {
                      Hublink_finesubstructures(hubdata);
                  }

                  ret = Hublink_DeleteHubData(USBFS_Hub_LinkHead,hubdata);//删除链表节点
                  if( ret == 0 )
                  {
                      printf("#Delete: depth-%x,uplevelport-%x,port-%x,addr-%x#\n",depth,uplevelport,i+1,USBFS_HubInfo[depth].portD[i].Addr);
                      USBFS_Del_AddressNumber(USBFS_HubInfo[depth].portD[i].Addr);//清除已用的地址
#if 1
                      printhub = USBFS_Hub_LinkHead;
                      while(printhub != NULL)//打印当前链表剩余节点
                      {
                            printf("/**********************************/\n");
                            printf("printhub.HUB_SaveData.Depth=%x\n",printhub->HUB_SaveData.Depth);
                            printf("printhub.HUB_SaveData.UpLevelPort=%x\n",printhub->HUB_SaveData.UpLevelPort);
                            printf("printhub.HUB_SaveData.CurrentPort=%x\n",printhub->HUB_SaveData.CurrentPort);
                            printhub = printhub->next;
                      }
#endif
                  }
                  ret = HubClearPortFeature( depth,i+1);
                  memset( &USBFS_HubInfo[depth].portD[i],0x00,sizeof( USBFS_HubInfo[depth].portD[i] ) );
                  printf("hub_device_disconnect_port=%02x\r\n",i+1);
              }
              else
              {
                  ret = HubClearPortFeature( depth,i+1);
                  if( ret != ERR_SUCCESS )return ret;
                  ret = HubGetPortStatus( depth,i+1 );
                  if( ret != ERR_SUCCESS )return ret;
              }

          }

          if( USBFS_HubInfo[depth].portD[i].Status == HUB_ERR_SCUESS )
          {       //设备端口操作成功
              if(  USBFS_HubInfo[depth].portD[i].DeviceType == 0x03 )
              {       //HID设备类
                  for( s=0;s!=USBFS_HubInfo[depth].portD[i].EndpNum;s++ )
                  {   //几个端点
                      if( USBFS_HubInfo[depth].portD[i].portEndp[s].Num &0x80 )
                      {     //IN端点
                        USBFS_setdevspeed(USBFS_HubInfo[depth].portD[i].Speed);
                        if( USBFS_HubInfo[depth].Speed != 0x02 )
                        {
                            USBOTG_H_FS->HOST_CTRL &= ~USBFS_UH_LOW_SPEED;
                            Delay_Us(100);
                        }

                        ret = USBFS_INT_In_Data( &USBFS_HubInfo[depth].portD[i],s,USBFS_test_buf,&len,(USB_PID_IN<<4)|USBFS_HubInfo[depth].portD[i].portEndp[s].Num,&USBFS_HubInfo[depth].portD[i].portEndp[s].tog );
                        if( ret == ERR_SUCCESS )
                        {
                            USBFS_HubInfo[depth].portD[i].portEndp[s].tog ^= USBFS_UH_R_TOG;
                            for( temp = 0; temp!=len;temp++ )
                            {
                                printf("%02x ",USBFS_test_buf[temp]);
                            }
                            printf("\n");

                            modifyhubdata = hubdata;
                            modifyhubdata.HUB_Info.portD[i].portEndp[s].tog = USBFS_HubInfo[depth].portD[i].portEndp[s].tog;
                            Hublink_ModifyHubData(USBFS_Hub_LinkHead,hubdata,modifyhubdata);
                        }
                      }
                  }
              }
              else if( USBFS_HubInfo[depth].portD[i].DeviceType == 0x08 )
              {   //UDISK,需要通过BULK进行处理UFI命令

              }
              else if( USBFS_HubInfo[depth].portD[i].DeviceType == 0x09 )
              {   //下面是HUB，进行枚举设备
                  USBFS_glable_index++;
                  ret = USBFS_HUB_Process( USBFS_glable_index ,USBFS_HubInfo[depth].portD[i].Addr,i+1);
                  USBFS_glable_index--;
                  if( ret != ERR_SUCCESS )return ret;
              }
              else
              {                                             //其他设备类
                  for( s=0;s!=USBFS_HubInfo[depth].portD[i].EndpNum;s++ )
                  {   //几个端点
                      if( USBFS_HubInfo[depth].portD[i].portEndp[s].EndpType == 0x01 )
                      {  //同步端点

                      }
                  }
              }
          }
          Delay_Ms(2);
      }
      if( (USBOTG_H_FS->MIS_ST & USBFS_UMS_DEV_ATTACH) == 0)
      {                //总的设备断开，直接跳走,需要把HUB的所有变了全部清除
          printf("dis\n");
          for( i=0;i!=4;i++ )
          {
              USBFS_HubInfo[depth].portD[i].Status = 0;
              USBFS_HubInfo[depth].portD[i].Addr = 0;
              USBFS_HubInfo[depth].portD[i].Speed = 0;
              USBFS_HubInfo[depth].portD[i].DeviceType = 0;
          }
          memset( USBFS_HubInfo,0x00,sizeof( USBFS_HubInfo ) );

          return ERR_USB_DISCON;
      }
      return ERR_SUCCESS;
}
