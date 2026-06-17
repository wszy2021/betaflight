#pragma once

#include "follow/follow_math.h"

uint32_t *followGetCrsfChannelData(void);

typedef void (*followComputeTargetChannelFn)(int xOffset, int yOffset);
typedef void (*followCommandVoidFn)(void);
typedef void (*followCommandBoolFn)(bool value);
typedef void (*followCommandUint8Fn)(uint8_t value);

typedef struct
{
    followCommandVoidFn sendHeartbeat;
    followCommandVoidFn startTrack;
    followCommandVoidFn stopTrack;
    followCommandBoolFn setExpand;
    followCommandBoolFn setTransfer;
    followCommandVoidFn moveUp;
    followCommandVoidFn moveDown;
    followCommandVoidFn moveLeft;
    followCommandVoidFn moveRight;
    followCommandVoidFn confirmTrack;
    followCommandUint8Fn switchLens;
} followCommandHooks_t;

void followUpdateRcData(void *data, uint8_t size);
void followDealWithRcValues(void);
void followStartTrackProcess(void);
int8_t followAdjustCrsfDataIfNecessary(void);
void followSetCommandHooks(const followCommandHooks_t *hooks);
const followCommandHooks_t *followGetCommandHooks(void);
void followResetCommandHooks(void);
void followSetComputeTargetChannelHook(followComputeTargetChannelFn fn);
followComputeTargetChannelFn followGetComputeTargetChannelHook(void);
void followResetComputeTargetChannelHook(void);
void followComputeTargetChannelDefault(int xOffset, int yOffset);
void followComputeTargetChannel(int xOffset, int yOffset);
void followReceivePidGroup(uint8_t type, uint8_t *data, uint32_t size);
void followSendPidGroup(uint8_t type, uint8_t *data, uint32_t size);
void followParseSimData(uint8_t *data, uint32_t size);
void followSelectPidProfile(void);
void followCalcAttitude(void);
void followHandleGuidance(void);
void followSetErrVelWindowDepth(int depth);
void followResetErrVelWindow(void);
void followSetPitOffRateWindowDepth(int depth);
void followResetPitOffRateWindow(void);
float followFilterPitOffRate(const float raw);
void followUpdateFlightMode(const char *mode);
void followUpdateAttitude(float pitch, float roll, float yaw);
