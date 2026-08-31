#pragma once

/*
 * 遥控板串口协议（115200 8N1）
 *
 * 帧格式：AA 55 [CMD] [LEN] [PAYLOAD...] [XOR] 5A
 * XOR = CMD ^ LEN ^ PAYLOAD[0] ^ ... ^ PAYLOAD[LEN-1]
 *
 * 遥控板 → FC
 *   读 UID     AA 55 01 00 01 5A
 *   接收机配对 AA 55 02 00 02 5A
 *
 * FC → 遥控板
 *   UID 回复   AA 55 81 06 [UID0..5] [XOR] 5A
 *   配对应答   AA 55 82 01 [STATUS] [XOR] 5A   STATUS: 0=OK, 1=失败
 */

#ifdef USE_SERIALRX_CRSF
void rcBoardInit(void);
void rcBoardProcess(void);
#else
static inline void rcBoardInit(void) {}
static inline void rcBoardProcess(void) {}
#endif
