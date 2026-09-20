#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

/*
 * 遥控板串口（CRSF 420000 8N1）
 *
 * 遥控板 → FC
 *   DEVICE_PING  0x28          周期心跳
 *   RC CHANNELS  0x16 / 0x17   连接时优先用此通道
 *   GET_RX_UID   COMMAND 0x32 / 0x10 0x0A
 *   RX_BIND      COMMAND 0x32 / 0x10 0x01
 *   MSP_WRITE    0x7C  MSP_SET_VTX_CONFIG(89) 切图传频段
 *
 * FC → 遥控板
 *   DEVICE_INFO  0x29          回应 PING，遥控板据此确认已连接
 *   UID 回复     EE 0C 32 EE EA 10 0A [UID0..5] [CRC]
 */

#ifdef USE_SERIALRX_CRSF
void rcBoardInit(void);
void rcBoardProcess(void);
bool rcBoardIsConnected(void);
uint8_t rcBoardFrameStatus(void);
float rcBoardReadRawRC(uint8_t chan);
timeUs_t rcBoardLastRcFrameTimeUs(void);
#else
static inline void rcBoardInit(void) {}
static inline void rcBoardProcess(void) {}
static inline bool rcBoardIsConnected(void) { return false; }
#endif
