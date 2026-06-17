#pragma once

#include "follow/follow_config.h"

typedef enum {
    FOLLOW_LAUNCH_STATUS_PRE_ARMED,
    FOLLOW_LAUNCH_STATUS_ARMED_PRE_LAUNCH,
    FOLLOW_LAUNCH_STATUS_TAKEOFF,
    FOLLOW_LAUNCH_STATUS_GUIDANCE
} followLaunchStatus_e;

typedef struct {
    float alpha;
    float beta;
    float x;
    float v;
} followAlphaBetaFilter_t;

typedef struct {
    uint8_t deviceAddress;
    uint8_t frameSize;
    uint8_t type;
    uint8_t *data;
    uint8_t crc;
} PACKED followCrsfHeader_t;

typedef struct {
    unsigned ch0 : 11;
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
} PACKED followCrsfChannels_t;

typedef struct {
    int ch1;
    int ch2;
    int ch3;
    int ch4;
    int ch5;
    int ch6;
    int ch7;
    int ch8;
    int ch9;
    int ch10;
    int ch11;
    int ch12;
    int ch13;
    int ch14;
    int ch15;
    int ch16;
} followRcChannels_t;

typedef struct {
    float rollPRate;
    float rollDRate;
    float rollIRate;
    float rollIMax;
    float rollMinOutput;
    float rollMaxOutput;
    float rollFilterAlpha;
    float pitchPRate;
    float pitchDRate;
    float pitchIRate;
    float pitchIMax;
    float pitchMinOutput;
    float pitchMaxOutput;
    float pitchFilterAlpha;
    float accPRate;
    float accDRate;
    float accIRate;
    float accIMax;
    float accMinOutput;
    float accMaxOutput;
    float accFilterAlpha;
    float yawPRate;
    float yawDRate;
    float yawIRate;
    float yawIMax;
    float yawMinOutput;
    float yawMaxOutput;
    float yawFilterAlpha;
} followPidValues_t;

#define FOLLOW_PID_VALUE_COUNT 28

typedef struct {
    float expected;
    float error;
    float limitedError;
    float integralOutput;
    float integralLimit;
    float lastError;
    float dt;
    float output;
    float outputLimit;
} followPidRuntime_t;

typedef struct {
    followPidValues_t profile1;
    followPidValues_t profile2;
    followPidValues_t profile3;
    followPidValues_t profile4;
    followPidValues_t profile5;
    followPidValues_t profile6;
    uint32_t flashInitFlag;
} followPidTable_t;

typedef struct __attribute__((packed)) {
    unsigned char traceStatus : 8;
    unsigned coordinateX : 16;
    unsigned coordinateY : 16;
    unsigned width : 16;
    unsigned height : 16;
    unsigned char checksum : 8;
} followTraceInfo_t;

typedef struct {
    uint8_t mode;
    uint8_t deviceCpuType;
    uint8_t visibleLensType;
    uint8_t infraredLensType;
    uint8_t traceState;
    uint8_t transferState;
    uint8_t traceStage;
    uint8_t traceLensType;
    uint8_t previousTraceLensType;
    uint8_t traceUpdated;
    uint8_t attitudeUpdated;
    uint32_t trackCommand;
    uint8_t moveTrackBoxFlag;
    uint16_t moveTrackBoxTime;
    uint8_t rcChannelInfoValid;
    uint16_t rcLoseSignalTime;
    uint8_t pcRcChannelInfoValid;
    uint8_t fpvAttitudeInfoValid;
    uint32_t ledTimeCount;
    uint32_t keyCommand;
    int trackErrorX;
    int trackErrorY;
    uint32_t traceErrorTimeMs;
    int lastTrackErrorX;
    int lastTrackErrorY;
    int16_t cameraResolutionX;
    int16_t cameraResolutionY;
    float pixelFocalLength;
    uint8_t inSwitchState;
    float lastView[3];
    uint8_t firstView;
    uint32_t lastPidTick;
    uint32_t lastPidRateTick;
    float targetRollChannel;
    float targetPitchChannel;
    float targetAccChannel;
    float targetYawChannel;
    uint32_t initialRollChannel;
    uint32_t initialPitchChannel;
    uint32_t initialAccChannel;
    uint32_t initialYawChannel;
    uint8_t aiRcChannelInfoValid;
    uint8_t autoControlMode;
    uint8_t previousAutoControlMode;
    uint8_t needStore;
    float temperature;
    float pressure;
    float height;
    float heightH0;
    float yawRad;
    float pitchRad;
    float rollRad;
    uint32_t attitudeDataTimeMs;
    float yawDeg;
    float pitchDeg;
    float rollDeg;
    float startYawDeg;
    float startRollDeg;
    uint16_t calRollLockTime;
    float yawError;
    float lastYawError;
    float diffYawError;
    float trackPitchDeg;
    float pitchError;
    float lastPitchError;
    float desiredRoll;
    float rollError;
    float lastRollError;
    float diffRollError;
    uint32_t testPeriodCount;
    uint32_t testPeriod;
    float trackAttenuationXY;
    uint32_t trackAttenuationStartMs;
    float minTrackPitchDeg;
    uint8_t reduceTrackPitch;
    uint8_t controlLaunchStatus;
    uint32_t controlLaunchTimeMs;
} followSystemData_t;

#define FOLLOW_AUTO_CONTROL_ANGLE 0x00
#define FOLLOW_AUTO_CONTROL_ACRO 0x01

extern followSystemData_t followSystemData;
extern followPidTable_t followPidTable;
extern followPidValues_t followCurrentPid;
extern volatile uint8_t followRcDataUpdated;
extern volatile uint8_t followModeActive;
