#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SERIALRX_CRSF

#include "build/version.h"

#include "common/crc.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "follow/follow_bundle.h"

#include "io/rc_board.h"
#include "io/serial.h"

#include "msp/msp_protocol.h"

#include "rx/crsf.h"
#include "rx/crsf_protocol.h"
#include "rx/rx.h"

#if defined(USE_VTX_COMMON)
#include "drivers/vtx_common.h"
#include "io/vtx.h"
#endif

#if defined(USE_MSP_OVER_TELEMETRY)
#include "telemetry/msp_shared.h"
#endif

#define RC_BOARD_PING_TIMEOUT_US    1000000
#define RC_BOARD_FRAME_TIMEOUT_US   1750

static serialPort_t *rcBoardPort;

static crsfFrame_t rcBoardFrame;
static crsfFrame_t rcBoardChannelFrame;
static uint32_t rcBoardChannelData[CRSF_MAX_CHANNEL];
static float rcBoardChannelScale = CRSF_RC_CHANNEL_SCALE_LEGACY;

static volatile bool rcBoardFrameDone;
static volatile bool rcBoardUidReplyPending;
static volatile bool rcBoardBindPending;
static volatile bool rcBoardMspPending;
static volatile bool rcBoardDeviceInfoPending;
static volatile bool rcBoardConnected;
static volatile timeUs_t rcBoardLastPingUs;
static volatile timeUs_t rcBoardLastRcUs;
static volatile uint8_t rcBoardPingOrigin = CRSF_ADDRESS_CRSF_TRANSMITTER;

static uint8_t rcBoardMspPayload[CRSF_PAYLOAD_SIZE_MAX];
static volatile uint8_t rcBoardMspLen;

static void rcBoardExpireConnection(void)
{
    if (rcBoardConnected && rcBoardLastPingUs != 0 &&
        cmpTimeUs(micros(), rcBoardLastPingUs) >= RC_BOARD_PING_TIMEOUT_US) {
        rcBoardConnected = false;
    }
}

static void rcBoardSendUidReply(void)
{
    if (!rcBoardPort) {
        return;
    }

    uint8_t uid[CRSF_RX_UID_LENGTH];
    if (!crsfRxGetUid(uid)) {
        return;
    }

    uint8_t dest = rcBoardPingOrigin;
    if (dest == 0 || dest == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
        dest = CRSF_ADDRESS_CRSF_TRANSMITTER;
    }

    uint8_t frame[2 + CRSF_COMMAND_RX_UID_FRAME_LENGTH];
    frame[0] = dest;
    frame[1] = CRSF_COMMAND_RX_UID_FRAME_LENGTH;
    frame[2] = CRSF_FRAMETYPE_COMMAND;
    frame[3] = dest;
    frame[4] = CRSF_ADDRESS_RADIO_TRANSMITTER;
    frame[5] = CRSF_COMMAND_SUBCMD_RX;
    frame[6] = CRSF_COMMAND_SUBCMD_RX_GET_UID;
    memcpy(&frame[7], uid, CRSF_RX_UID_LENGTH);

    uint8_t crc = 0;
    for (int i = 2; i < 13; i++) {
        crc = crc8_dvb_s2(crc, frame[i]);
    }
    frame[13] = crc;

    serialWriteBuf(rcBoardPort, frame, sizeof(frame));
}

static void rcBoardSendDeviceInfo(void)
{
    if (!rcBoardPort) {
        return;
    }

    const char *name = FC_FIRMWARE_NAME;
    const uint8_t nameLen = strlen(name) + 1; // include NUL
    uint8_t dest = rcBoardPingOrigin;
    if (dest == 0 || dest == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
        dest = CRSF_ADDRESS_CRSF_TRANSMITTER;
    }

    // type + dest + orig + name\0 + 12 nulls + paramCount + version + crc
    const uint8_t frameLength = CRSF_FRAME_LENGTH_TYPE + CRSF_FRAME_ORIGIN_DEST_SIZE + nameLen + 12 + 2 + CRSF_FRAME_LENGTH_CRC;
    uint8_t frame[CRSF_FRAME_SIZE_MAX];
    uint8_t i = 0;

    frame[i++] = CRSF_SYNC_BYTE;
    frame[i++] = frameLength;
    frame[i++] = CRSF_FRAMETYPE_DEVICE_INFO;
    frame[i++] = dest;
    frame[i++] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    memcpy(&frame[i], name, nameLen);
    i += nameLen;
    memset(&frame[i], 0, 12);
    i += 12;
    frame[i++] = 0;    // parameter count
    frame[i++] = 0x01; // parameter version

    uint8_t crc = 0;
    for (int n = 2; n < i; n++) {
        crc = crc8_dvb_s2(crc, frame[n]);
    }
    frame[i++] = crc;

    serialWriteBuf(rcBoardPort, frame, i);
}

static bool rcBoardIsCommand(uint8_t subCmd)
{
    return rcBoardFrame.frame.type == CRSF_FRAMETYPE_COMMAND &&
        rcBoardFrame.frame.frameLength >= 6 &&
        rcBoardFrame.bytes[5] == CRSF_COMMAND_SUBCMD_RX &&
        rcBoardFrame.bytes[6] == subCmd;
}

#if defined(USE_VTX_COMMON) && !defined(USE_MSP_OVER_TELEMETRY)
static void rcBoardApplyVtxConfig(const uint8_t *payload, uint8_t size)
{
    if (size < 2) {
        return;
    }

    vtxDevice_t *vtxDevice = vtxCommonDevice();
    const uint16_t newFrequency = payload[0] | ((uint16_t)payload[1] << 8);

    if (newFrequency <= VTXCOMMON_MSP_BANDCHAN_CHKVAL) {
        const uint8_t newBand = (newFrequency / 8) + 1;
        const uint8_t newChannel = (newFrequency % 8) + 1;
        vtxSettingsConfigMutable()->band = newBand;
        vtxSettingsConfigMutable()->channel = newChannel;
        vtxSettingsConfigMutable()->freq = vtxCommonLookupFrequency(vtxDevice, newBand, newChannel);
    } else if (newFrequency <= VTX_SETTINGS_MAX_FREQUENCY_MHZ) {
        vtxSettingsConfigMutable()->band = 0;
        vtxSettingsConfigMutable()->freq = newFrequency;
    }

    if (size >= 3) {
        vtxSettingsConfigMutable()->power = payload[2];
    }
}
#endif

static void rcBoardHandleMspWrite(uint8_t *msp, uint8_t len)
{
#if defined(USE_MSP_OVER_TELEMETRY)
    handleMspFrame(msp, len, NULL);
#elif defined(USE_VTX_COMMON)
    // MSPv1: status, size, cmd, payload...
    if (len >= 5 && (msp[0] & 0x10) && msp[2] == MSP_SET_VTX_CONFIG) {
        rcBoardApplyVtxConfig(&msp[3], msp[1]);
    }
#else
    UNUSED(msp);
    UNUSED(len);
#endif
}

static void rcBoardQueueMspWrite(void)
{
    const int mspLen = (int)rcBoardFrame.frame.frameLength - 4;
    if (mspLen <= 0 || mspLen > (int)sizeof(rcBoardMspPayload)) {
        return;
    }

    memcpy(rcBoardMspPayload, &rcBoardFrame.bytes[5], mspLen);
    rcBoardMspLen = (uint8_t)mspLen;
    rcBoardMspPending = true;
}

static uint8_t rcBoardFrameCrc(void)
{
    uint8_t crc = crc8_dvb_s2(0, rcBoardFrame.frame.type);
    for (int i = 0; i < rcBoardFrame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC; ++i) {
        crc = crc8_dvb_s2(crc, rcBoardFrame.frame.payload[i]);
    }
    return crc;
}

static void rcBoardDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    static uint8_t framePosition = 0;
    static timeUs_t frameStartUs = 0;
    const timeUs_t nowUs = microsISR();

    if (cmpTimeUs(nowUs, frameStartUs) > RC_BOARD_FRAME_TIMEOUT_US) {
        framePosition = 0;
    }

    if (framePosition == 0) {
        frameStartUs = nowUs;
    }

    const int fullFrameLength = framePosition < 3
        ? 5
        : MIN(rcBoardFrame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH, CRSF_FRAME_SIZE_MAX);

    if (framePosition < fullFrameLength) {
        rcBoardFrame.bytes[framePosition++] = (uint8_t)c;
        if (framePosition >= fullFrameLength) {
            framePosition = 0;
            if (rcBoardFrameCrc() != rcBoardFrame.bytes[fullFrameLength - 1]) {
                return;
            }

            switch (rcBoardFrame.frame.type) {
            case CRSF_FRAMETYPE_DEVICE_PING:
                rcBoardLastPingUs = nowUs;
                rcBoardConnected = true;
                if (rcBoardFrame.frame.frameLength >= 4) {
                    rcBoardPingOrigin = rcBoardFrame.bytes[4];
                }
                rcBoardDeviceInfoPending = true;
                break;

            case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            case CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED:
                memcpy(&rcBoardChannelFrame, &rcBoardFrame, sizeof(rcBoardFrame));
                rcBoardLastRcUs = nowUs;
                rcBoardFrameDone = true;
                if (rcBoardConnected) {
                    followUpdateRcData(&rcBoardChannelFrame, sizeof(rcBoardChannelFrame));
                }
                break;

            case CRSF_FRAMETYPE_COMMAND:
                if (rcBoardIsCommand(CRSF_COMMAND_SUBCMD_RX_GET_UID)) {
                    rcBoardUidReplyPending = true;
                } else if (rcBoardIsCommand(CRSF_COMMAND_SUBCMD_RX_BIND)) {
                    rcBoardBindPending = true;
                }
                break;

            case CRSF_FRAMETYPE_MSP_WRITE:
            case CRSF_FRAMETYPE_MSP_REQ:
                rcBoardQueueMspWrite();
                break;

            default:
                break;
            }
        }
    }
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
        CRSF_BAUDRATE,
        CRSF_PORT_MODE,
        CRSF_PORT_OPTIONS);

    for (int i = 0; i < CRSF_MAX_CHANNEL; ++i) {
        rcBoardChannelData[i] = (16 * rxConfig()->midrc) / 10 - 1408;
    }
}

void rcBoardProcess(void)
{
    rcBoardExpireConnection();

    if (rcBoardDeviceInfoPending) {
        rcBoardDeviceInfoPending = false;
        rcBoardSendDeviceInfo();
    }

    if (rcBoardUidReplyPending) {
        uint8_t uid[CRSF_RX_UID_LENGTH];
        if (crsfRxGetUid(uid)) {
            rcBoardUidReplyPending = false;
            rcBoardSendUidReply();
        }
    }

    if (rcBoardBindPending) {
        rcBoardBindPending = false;
        crsfRxBind();
    }

    if (rcBoardMspPending) {
        uint8_t mspBuf[CRSF_PAYLOAD_SIZE_MAX];
        const uint8_t mspLen = rcBoardMspLen;
        memcpy(mspBuf, rcBoardMspPayload, mspLen);
        rcBoardMspPending = false;
        rcBoardHandleMspWrite(mspBuf, mspLen);
    }
}

bool rcBoardIsConnected(void)
{
    return rcBoardPort && rcBoardConnected;
}

uint8_t rcBoardFrameStatus(void)
{
    rcBoardExpireConnection();

    if (!rcBoardFrameDone) {
        return RX_FRAME_PENDING;
    }

    rcBoardFrameDone = false;
    if (crsfDecodeRcChannels(&rcBoardChannelFrame, rcBoardChannelData, &rcBoardChannelScale)) {
        return RX_FRAME_COMPLETE;
    }

    return RX_FRAME_PENDING;
}

float rcBoardReadRawRC(uint8_t chan)
{
    if (chan >= CRSF_MAX_CHANNEL) {
        return 0;
    }

    return crsfChannelToPwm(rcBoardChannelData[chan], rcBoardChannelScale);
}

timeUs_t rcBoardLastRcFrameTimeUs(void)
{
    return rcBoardLastRcUs;
}

#endif
