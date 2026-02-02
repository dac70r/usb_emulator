/********************************** (C) COPYRIGHT  *******************************
* File Name          : main.h
* Author             : -
* Version            : V1.0.0
* Date               : 2021/08/08
* Description        : -
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __MAIN_H
#define __MAIN_H 	

#include "stdint.h"

#define buffer_length					256
#define local_mode						0x00
#define remote_mode						0x01
#define test_serial_port			1
#define emulation							!test_serial_port

typedef enum {
	local_control = 0,
	remote_control = 1
}communication_type;

// By Default, we have set our communication mode to be local control
static communication_type communication_mode = remote_control;

extern uint8_t clear_buffer_flag;
extern uint8_t uart4_storage[buffer_length];
extern uint8_t indexes;

#endif /* __MAIN_H */



