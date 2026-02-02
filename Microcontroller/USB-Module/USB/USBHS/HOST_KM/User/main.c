/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2022/09/01
* Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 * @Note
 * This example demonstrates the process of enumerating the keyboard and mouse 
 * by a USB host and obtaining data based on the polling time of the input endpoints 
 * of the keyboard and mouse. 
 * The USBFS port also supports enumeration of keyboard and mouse attached at tier 
 * level 2(Hub 1).
*/

/*
 * @Note
 * Please select the corresponding macro definition (CH32F20x_D6/CH32F20x_D8/CH32F20x_D8C/CH32F20x_D8W) 
 * and startup_xxx.s file according to the chip model, otherwise the example may be abnormal.
 * In addition, when the system clock is selected as the USBFS clock source, only 144MHz/96MHz/48MHz 
 * are supported.
 */

/*******************************************************************************/
/* Header Files */
#include <usb_hub.h>
#include "debug.h"
#include "ch32f20x_usbhs_host.h"
#include "ch32f20x_usbfs_host.h"
#include "usbfs_host_config.h"
#include "stdlib.h"
#include "ch32f20x_usbfs_device.h"	// - comment added by Dennis, usb device library   
#include "usbd_composite_km.h"			// - comment added by Dennis, usb device library
#include "ch32f20x_usart.h"
#include "main.h"
#include "ch32f20x_usbfs_device.h"
#include "ctype.h"

__attribute__ ((aligned(4))) UINT8   USBHS_endpTXbuf[ MAX_PACKET_SIZE ];  // IN, must even address
__attribute__ ((aligned(4))) UINT8   USBHS_endpRXbuf[ MAX_PACKET_SIZE ];  // OUT, must even address

__attribute__ ((aligned(4))) UINT8   USBFS_endpTXbuf[ MAX_PACKET_SIZE ];  // IN, must even address
__attribute__ ((aligned(4))) UINT8   USBFS_endpRXbuf[ MAX_PACKET_SIZE ];  // OUT, must even address

#define USBHS_HUBCONNECT      (1<<0)
#define USBHS_ROOTDEVCONNECT  (1<<1)
#define USBFS_HUBCONNECT      (1<<2)
#define USBFS_ROOTDEVCONNECT  (1<<3)

uint8_t MS_DataPack[4] = {0x00, 0x00, 0x00, 0xff};
uint8_t KB_DataPack[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t uart4_storage[buffer_length] = {0x00};
uint8_t indexes = 0x00;
uint8_t clear_buffer_flag = 0x00;
	
void UART4_IRQHandler(void)
{
	if(USART_GetITStatus(UART4, USART_IT_RXNE))
	{

		uint8_t byte = (uint8_t) USART_ReceiveData(UART4);
		uart4_storage[indexes] = byte;					// store byte in the buffer
		indexes = (indexes + 1) % buffer_length;	// incrementing the index and ensuring circular buffer wrap-around
		
		if(byte == 0x55){
			//printf("%c", byte);																			// problematic here
			clear_buffer_flag = 1;
		}

	}
}

uint8_t hex_to_byte(char high, char low)
{
    uint8_t h = (high <= '9') ? high - '0'
                              : (high & ~0x20) - 'A' + 10;

    uint8_t l = (low <= '9') ? low - '0'
                              : (low & ~0x20) - 'A' + 10;

    return (h << 4) | l;
}

void parse_hex_report(const char *ascii, uint8_t *out_bytes, int max_len)
{
    int bi = 0;
		int i;
    for (i = 0; ascii[i] != '\0' && bi < max_len; )
    {
        // Skip spaces
        while (ascii[i] == ' ') i++;

        // If not valid hex or string ends, stop
        if (!isxdigit(ascii[i]) || !isxdigit(ascii[i+1]))
            break;

        // Convert two ASCII characters into one byte
        out_bytes[bi++] = hex_to_byte(ascii[i], ascii[i+1]);

        // Move forward by 2 ASCII chars
        i += 2;
    }
}

void decode_frame(uint8_t *frame, int frame_len)
{
    uint8_t start = frame[0];     // 0xAA
    uint8_t cmd   = frame[1];
    uint8_t len   = frame[2];

    // Safety check
    if (start != 0xAA) return;
    if (frame[4 + len] != 0x55) return;   // end byte check

    // Copy payload into a null-terminated string
    char ascii_payload[64];
    memcpy(ascii_payload, &frame[3], len);
    ascii_payload[len] = '\0';

    printf("ASCII payload received: %s\n", ascii_payload);

    // Now convert to HID bytes
    uint8_t hid_report[8] = {0};
    parse_hex_report(ascii_payload, hid_report, sizeof(hid_report));

    printf("Decoded HID bytes: ");
		int i;
    for (i = 0; i < sizeof(hid_report); i++)
        printf("%02X ", hid_report[i]);
    printf("\n");
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main( void )
{
	  UINT8 ret,s,temp,fstog = 0,hstog = 0;

    UINT16 len;
    UINT8 usbconnectflag = 0;
    Link_HUBSaveData *delhub = NULL;							// clear link
	
		// Setup the Switch Pin as Input GPIO
		GPIO_InitTypeDef GPIO_InitStructure;
		/* 1. Enable GPIOB clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
		/* 2. Configure PB15 as output push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;   // Input with Pull Up Resistor
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

		/* 3. Initialize */
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		// Temporary Flag Registers
		uint8_t usb_root_status 		= 0;
		uint8_t device_init_status 	= 0;
	
    /* Initialize Delay and UART Peripherals */
    Delay_Init( );
    USART_Printf_Init( 115200 );
		printf( "/**************************************************/\n");
		printf( "Main: Program Name - USB HOST <-> Device Combo Testing \r\n" );
		printf( "Main: SystemClk:%d\r\n", SystemCoreClock );
    
		/* 	Initialize Host 		*/
		USBHS_HostInit(ENABLE);	
    memset( &RootHubDev[ 0x00 ].bStatus, 0, sizeof( ROOT_HUB_DEVICE ) );
		
		USBFS_RCC_Init( );																															// Clock Initialization via RCC 
		Delay_Ms(5);
		USBFS_Device_Init(ENABLE);																											// USB Device Initialization - Registers, Endpoint, NVIC
		USB_Sleep_Wakeup_CFG( );																												// EXTI (external interrupt) line so the USB peripheral can wake the MCU 

		/* Create HS and FS Objects thru Dynamic Allocation */ 
		USBHS_Hub_LinkHead = Hublink_InitHubLink();
    
		USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);																		// Interrupt receive command from python via UART
		NVIC_EnableIRQ(UART4_IRQn);																											// Enables NVIC interrupt


    while( 1 )
    {
			/*********************** IN THE REMOTE STATE ************************/
			//if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15) == 1){														// We use PB15 to control remote_mode/ local_mode (SWITCH)
			if(communication_mode == remote_control){		
					// in the remote state
					if(clear_buffer_flag == 1){
								if(uart4_storage[0] == 0xaa){																				// checks if the first and last byte 	
									
									char *ascii_payload = (char *)&uart4_storage[3];									// create a pointer to the data stored 
									
									if(uart4_storage[1] == 0x01){		
										char temp[64];
										memcpy(temp, ascii_payload, 8);																	// copy the stored data into a temp variable, the length of mouse HID is 8 bytes
										temp[8] = '\0';
									
										uint8_t hid[4];
										parse_hex_report(temp, hid, 4);																	// parse the temp variables into hid report 
										
										MS_DataPack[0] = hid[0];
										MS_DataPack[1] = hid[1];
										MS_DataPack[2] = hid[2];
										MS_DataPack[3] = hid[3];
										
										USBFS_Endp_DataUp(DEF_UEP2, MS_DataPack, sizeof(MS_DataPack), DEF_UEP_CPY_LOAD);		// manipulate the mouse 
									}
									else if(uart4_storage[1] == 0x02){
										char temp[64];
										memcpy(temp, ascii_payload, 16);																	// copy the stored data into a temp variable, the length of mouse HID is 16 bytes
										temp[16] = '\0';
									
										uint8_t hid[8];
										parse_hex_report(temp, hid, 8);																	// parse the temp variables into hid report 
										
										KB_DataPack[0] = hid[0];
										KB_DataPack[1] = hid[1];
										KB_DataPack[2] = hid[2];
										KB_DataPack[3] = hid[3];
										KB_DataPack[4] = hid[4];
										KB_DataPack[5] = hid[5];
										KB_DataPack[6] = hid[6];
										KB_DataPack[7] = hid[7];
										
										//memset(KB_DataPack, hid, sizeof(KB_DataPack));
										USBFS_Endp_DataUp(DEF_UEP1, KB_DataPack, sizeof(KB_DataPack), DEF_UEP_CPY_LOAD);		// manipulate the keyboard 
										Delay_Ms(10);																																				// add a big enough delay after initial 
										
										memset(KB_DataPack, 0, sizeof(KB_DataPack));
										USBFS_Endp_DataUp(DEF_UEP1, KB_DataPack, sizeof(KB_DataPack), DEF_UEP_CPY_LOAD);		// manipulate the keyboard 
									}
									else{
										// placeholder only
										printf("Error: transfer neither mouse nor keyboard\n");
									}
																	
								}
								
								clear_buffer_flag = 0;																							// must reset the buffer flag
								indexes = 0;																													// must reset the index of the buffer
								printf("%s", uart4_storage);   																			// print return message to pyserial
								memset(uart4_storage, 0, sizeof(uart4_storage[buffer_length]));			// clear the buffer contents
								
					}
					
			}
			/*********************** IN THE LOCAL STATE ************************/
			else{   
				// 1. Detects USB Port Changes 
				if( USBHSH->INT_FG & USBHS_DETECT_FLAG )   																		
        {
            USBHSH->INT_FG = USBHS_DETECT_FLAG;	// Clears the Interrupt Flag
					
            if( USBHSH->MIS_ST & USBHS_ATTCH )  // Checks if the USB Hub is Attached 
            {
								printf("Main: Port Status Change: Device Attached\n");								// -- comment by Dennis 18/11/2025
							
								memset( USBHS_HubInfo, 0x00, sizeof( USBHS_HubInfo ) );
                Delay_Ms(200);
							
                ret = USBHS_HostEnum(0, USBHS_endpRXbuf);														// -- comment by Dennis 14/11/2025, general information
                
								if (ret == ERR_SUCCESS) 
								{
										printf("Main: Hub Enumeration Success\n");
										switch (USBHS_thisUsbDev.DeviceType)    
										{
												case 0x09:  // HUB Device
														USBHS_glable_index = 0;
														usbconnectflag |= USBHS_HUBCONNECT;
														break;

												case 0x03:  // HID Device
														usbconnectflag |= USBHS_ROOTDEVCONNECT;
														break;

												default:
														printf("Main: Unknown Device Type 0x%02X\n", USBHS_thisUsbDev.DeviceType);
														break;
										}
								} 
								else{
										printf("Main: Enumeration error\n");
								}
            }
						
            else
            {
							// Device detachment detected, execute the following: 
								printf("Main: Port Status Change: Device Detached\n");														// -- comment by Dennis 13/11/2025
							
                usbconnectflag &= ~(USBHS_HUBCONNECT | USBHS_ROOTDEVCONNECT);
                delhub = USBHS_Hub_LinkHead->next;
							
								// decontructs the object created and frees memory 
                while(delhub)
                {
                    ret = USB_DelUSBHS_AddressNumber(delhub->HUB_SaveData.HUB_Info.portD[delhub->HUB_SaveData.CurrentPort - 1].Addr);
                    free(USBHS_Hub_LinkHead);
                    USBHS_Hub_LinkHead = delhub;
                    delhub = delhub->next;
                }
                USBHS_HostInit(DISABLE);
                USBHS_HostInit(ENABLE);
                //printf("disconnect\n");
                memset( &USBHS_thisUsbDev,0x00,sizeof( USBHS_thisUsbDev ) );
            }
        }
				// 1. ELSE IF No USB Port Changes Detected - For Debug Purposes Only
				else{
						//printf("Main: No USB Port Changes Detected\n");
						//Delay_Ms(1000);
				}
				
				/* Section 2: Handles the Comms with USB Hub or USB Root Device */
				/* USB Hub */
        if( usbconnectflag & USBHS_HUBCONNECT )							// No Attach/ Detach Event, checks for connection
        {								
            usbconnectflag &= ~USBHS_ROOTDEVCONNECT;
							//printf("Before USB Hub Process \n");
            ret = USBHS_HUB_Process( USBHS_glable_index, USBHS_HubInfo[0].DevAddr ,0);		// -- comment by Dennis 14/11/2025 -- HUB's MAIN PROCESS HERE, ALWAYS RUN
							//printf("After USB Hub Process \n");
						
            if( ret == ERR_USB_DISCON )
            {
                usbconnectflag &= ~USBHS_HUBCONNECT;
            }
        }
				
				
				
				
				/* USB Root Device */ 
				// Since we mainly deal with Hubs, we comment this portion of the code for now
#if 0				
        if( usbconnectflag & USBHS_ROOTDEVCONNECT )
        {
						if(usb_root_status == 0){																										// -- comment by Dennis 14/11/2025
							printf("Main: USB Root Device is detected \n");
							usb_root_status = 1;
						} 	
            usbconnectflag &= ~USBHS_HUBCONNECT;
						
            for( s=0;s!=USBHS_thisUsbDev.DevEndp.InEndpCount;s++ )
            {

                USBHSH->HOST_RX_DMA = (UINT32)USBHS_Test_Buf;
                ret = USBHS_USBHostTransact( USB_PID_IN << 4 | (USBHS_thisUsbDev.DevEndp.InEndpNum[s]&0x0f) , hstog, 0 );
                if( ret == 0 )
                {
                    hstog ^= UH_R_TOG_1;
                    len = USBHSH->RX_LEN ;
                    for( temp = 0; temp!=len;temp++ )
                    {
                        printf("%02x ",USBHS_Test_Buf[temp]); 
                    }
                    printf("\n");
                }
            }
            Delay_Ms(USBHS_thisUsbDev.DevEndp.Ininterval);
						
        }
			
			
#endif	
			}

			


		} // while 
} // main 



