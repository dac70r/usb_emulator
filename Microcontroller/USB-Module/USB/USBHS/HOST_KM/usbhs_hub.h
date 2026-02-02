/* usbhs_hub.h */
#ifndef __USBHS_HUB_H__
#define __USBHS_HUB_H__

#include "ch32f20x_usbhs_host.h"
#include <stdint.h>

#define USBHS_HUB_MAX_PORTS    4
#define USBHS_HUB_MAX_EP       4

/**
 * 信息结构: 每个 HUB 下游端口的状态
 */
typedef struct {
    uint8_t  port_num;         // 端口号 (1...N)
    uint8_t  dev_addr;         // 分配的设备地址
    uint8_t  speed;            // USB_SPEED: USB_LOW/FULL/HIGH
    uint8_t  ep0_size;         // 端点0最大包长度
    // 可扩展: Endpoint 描述
    uint8_t  in_ep_num;
    uint8_t  in_ep_addr[USBHS_HUB_MAX_EP];
    uint8_t  in_ep_tgl[USBHS_HUB_MAX_EP];
    uint16_t in_ep_size[USBHS_HUB_MAX_EP];
} USBHS_HubPortInfo;

/**
 * HUB 管理句柄
 */
typedef struct {
    uint8_t             hub_addr;         // HUB 本身的地址
    uint8_t             num_ports;        // HUB 支持端口数
    USBHS_HubPortInfo   ports[USBHS_HUB_MAX_PORTS];
} USBHS_HubHandle;

/**
 * @brief  初始化 USBHS HUB 模块
 * @param  hh  HUB 管理句柄
 */
void USBHS_Hub_Init(USBHS_HubHandle *hh);

/**
 * @brief  读取并解析 HUB 描述符，获取端口数量
 * @param  hh  HUB 管理句柄
 * @return 错误码
 */
uint8_t USBHS_Hub_ParseDescriptor(USBHS_HubHandle *hh);

/**
 * @brief  轮询 HUB 中断端点，检查端口状态变化
 * @param  hh  HUB 管理句柄
 * @param  buf 数据缓冲区
 * @return 错误码
 */
uint8_t USBHS_Hub_PollInterrupt(USBHS_HubHandle *hh, uint8_t *buf);

/**
 * @brief  对指定端口执行预枚举（连接清理、复位）
 * @param  hh       HUB 管理句柄
 * @param  port_idx 端口索引 (0-based)
 * @return 错误码
 */
uint8_t USBHS_Hub_PreEnumPort(USBHS_HubHandle *hh, uint8_t port_idx);

/**
 * @brief  枚举下游设备：GetDesc/SetAddr/SetCfg
 * @param  hh       HUB 管理句柄
 * @param  port_idx 端口索引
 * @return 错误码
 */
uint8_t USBHS_Hub_EnumPortDevice(USBHS_HubHandle *hh, uint8_t port_idx);

#endif /* __USBHS_HUB_H__ */
