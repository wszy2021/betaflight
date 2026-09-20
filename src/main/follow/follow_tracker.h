#pragma once

#include "follow/follow_control.h"

extern serialPort_t *followWorkPort;
extern serialPort_t *followSimPort;
extern const char *followVersion;
extern const char *followDescription;

bool followTrackerInit(void);
void followTrackerTask(timeUs_t currentTimeUs);
void followTrackerSerialInput(uint16_t data, void *rxCallbackData);
void followUpdateTrackerStatus(timeUs_t currentTimeUs);
bool followTrackerStatusUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs);
void followTimerCallback(timerOvrHandlerRec_t *cbRec, captureCompare_t capture);
void followTimerTask(timeUs_t currentTimeUs);
void followAnalyzeSerialInput(void);
void followAnalyzeSimInput(void);
void followPrintDebug(const char *msg, ...);
void followSendCommand(uint8_t *cmd, int32_t size, bool calcSum);
void followSendHeartbeatDefault(void);
void followStartTrackDefault(void);
void followStopTrackDefault(void);
void followSetExpandDefault(bool value);
void followSetTransferDefault(bool value);
void followMoveUpDefault(void);
void followMoveDownDefault(void);
void followMoveLeftDefault(void);
void followMoveRightDefault(void);
void followConfirmTrackDefault(void);
void followSwitchLensDefault(uint8_t value);
void followFuseTriggerHandle(uint32_t startTimeMs);
