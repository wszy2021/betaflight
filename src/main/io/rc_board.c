#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SERIALRX_CRSF

#include "drivers/serial.h"
#include "drivers/time.h"
#include "io/rc_board.h"
#include "io/serial.h"
#include "rx/crsf.h"
#include "rx/crsf_protocol.h"
#include "rx/rx_bind.h"

#define RC_BOARD_BAUDRATE           115200
#define RC_BOARD_FRAME_HEADER0      0xAA
#define RC_BOARD_FRAME_HEADER1      0x55
#define RC_BOARD_FRAME_TAIL         0x5A
#define RC_BOARD_PAYLOAD_MAX        8
#define RC_BOARD_FRAME_TIMEOUT_US   50000

#define RC_BOARD_CMD_GET_UID        0x01
#define RC_BOARD_CMD_BIND           0x02
#define RC_BOARD_CMD_UID_REPLY      0x81
#define RC_BOARD_CMD_BIND_REPLY     0x82

#define RC_BOARD_STATUS_OK          0
#define RC_BOARD_STATUS_FAIL        1

typedef enum {
    RC_BOARD_WAIT_HEADER0 = 0,
    RC_BOARD_WAIT_HEADER1,
    RC_BOARD_WAIT_CMD,
    RC_BOARD_WAIT_LEN,
    RC_BOARD_WAIT_PAYLOAD,
    RC_BOARD_WAIT_XOR,
    RC_BOARD_WAIT_TAIL,
} rcBoardParseState_e;

static serialPort_t *rcBoardPort;

static rcBoardParseState_e parseState = RC_BOARD_WAIT_HEADER0;
static uint8_t parseCmd;
static uint8_t parseLen;
static uint8_t parseIndex;
static uint8_t parsePayload[RC_BOARD_PAYLOAD_MAX];
static uint8_t parseXor;
static timeUs_t parseLastByteUs;

static volatile uint8_t pendingCmd;
static volatile bool cmdPending;

static uint8_t rcBoardChecksum(uint8_t cmd, uint8_t len, const uint8_t *payload)
{
    uint8_t checksum = cmd ^ len;
    for (uint8_t i = 0; i < len; i++) {
        checksum ^= payload[i];
    }
    return checksum;
}

static void rcBoardSendFrame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    if (!rcBoardPort || len > RC_BOARD_PAYLOAD_MAX) {
        return;
    }

    uint8_t frame[6 + RC_BOARD_PAYLOAD_MAX];
    uint8_t index = 0;

    frame[index++] = RC_BOARD_FRAME_HEADER0;
    frame[index++] = RC_BOARD_FRAME_HEADER1;
    frame[index++] = cmd;
    frame[index++] = len;
    for (uint8_t i = 0; i < len; i++) {
        frame[index++] = payload[i];
    }
    frame[index++] = rcBoardChecksum(cmd, len, payload);
    frame[index++] = RC_BOARD_FRAME_TAIL;

    serialWriteBuf(rcBoardPort, frame, index);
}

static void rcBoardResetParser(void)
{
    parseState = RC_BOARD_WAIT_HEADER0;
    parseIndex = 0;
    parseLen = 0;
}

static void rcBoardDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    const timeUs_t nowUs = microsISR();
    if (parseState != RC_BOARD_WAIT_HEADER0 &&
        cmpTimeUs(nowUs, parseLastByteUs) > RC_BOARD_FRAME_TIMEOUT_US) {
        rcBoardResetParser();
    }
    parseLastByteUs = nowUs;

    const uint8_t byte = (uint8_t)c;

    switch (parseState) {
    case RC_BOARD_WAIT_HEADER0:
        if (byte == RC_BOARD_FRAME_HEADER0) {
            parseState = RC_BOARD_WAIT_HEADER1;
        }
        break;
    case RC_BOARD_WAIT_HEADER1:
        parseState = (byte == RC_BOARD_FRAME_HEADER1) ? RC_BOARD_WAIT_CMD : RC_BOARD_WAIT_HEADER0;
        break;
    case RC_BOARD_WAIT_CMD:
        parseCmd = byte;
        parseState = RC_BOARD_WAIT_LEN;
        break;
    case RC_BOARD_WAIT_LEN:
        parseLen = byte;
        parseIndex = 0;
        parseXor = parseCmd ^ parseLen;
        if (parseLen > RC_BOARD_PAYLOAD_MAX) {
            rcBoardResetParser();
        } else if (parseLen == 0) {
            parseState = RC_BOARD_WAIT_XOR;
        } else {
            parseState = RC_BOARD_WAIT_PAYLOAD;
        }
        break;
    case RC_BOARD_WAIT_PAYLOAD:
        parsePayload[parseIndex++] = byte;
        parseXor ^= byte;
        if (parseIndex >= parseLen) {
            parseState = RC_BOARD_WAIT_XOR;
        }
        break;
    case RC_BOARD_WAIT_XOR:
        if (byte == parseXor) {
            parseState = RC_BOARD_WAIT_TAIL;
        } else {
            rcBoardResetParser();
        }
        break;
    case RC_BOARD_WAIT_TAIL:
        if (byte == RC_BOARD_FRAME_TAIL) {
            pendingCmd = parseCmd;
            cmdPending = true;
        }
        rcBoardResetParser();
        break;
    default:
        rcBoardResetParser();
        break;
    }
}

static void rcBoardHandleGetUid(void)
{
    uint8_t uid[CRSF_RX_UID_LENGTH] = { 0 };
    crsfRxGetUid(uid);
    rcBoardSendFrame(RC_BOARD_CMD_UID_REPLY, uid, CRSF_RX_UID_LENGTH);
}

static void rcBoardHandleBind(void)
{
    uint8_t status = RC_BOARD_STATUS_FAIL;

#if defined(USE_RX_BIND)
    if (startRxBind()) {
        status = RC_BOARD_STATUS_OK;
    }
#else
    if (crsfRxIsActive()) {
        crsfRxBind();
        status = RC_BOARD_STATUS_OK;
    }
#endif

    rcBoardSendFrame(RC_BOARD_CMD_BIND_REPLY, &status, 1);
}

void rcBoardInit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RC_BOARD);
    if (!portConfig) {
        return;
    }

    rcBoardPort = openSerialPort(portConfig->identifier,
        FUNCTION_RC_BOARD,
        rcBoardDataReceive,
        NULL,
        RC_BOARD_BAUDRATE,
        MODE_RXTX,
        SERIAL_NOT_INVERTED);
}

void rcBoardProcess(void)
{
    crsfRxProcessUidSave();

    if (!cmdPending) {
        return;
    }

    const uint8_t cmd = pendingCmd;
    cmdPending = false;

    switch (cmd) {
    case RC_BOARD_CMD_GET_UID:
        rcBoardHandleGetUid();
        break;
    case RC_BOARD_CMD_BIND:
        rcBoardHandleBind();
        break;
    default:
        break;
    }
}

#endif
