#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "follow/follow_bundle.h"

#define FOLLOW_MODE_HANDLER_COUNT 8
#define FOLLOW_MODE_STATE_SIZE 16

typedef enum {
    FOLLOW_CONTROL_MODE_NONE = 0,
    FOLLOW_CONTROL_MODE_TRACK = 1,
    FOLLOW_CONTROL_MODE_FOLLOW = 2,
    FOLLOW_CONTROL_MODE_ORBIT = 3,
} followControlMode_e;

typedef struct {
    bool valid;
    bool targetLocked;
    float dt;
    float targetDistance;
    float targetBearing;
    float targetYawRate;
    int targetErrorX;
    int targetErrorY;
    float bodyRollRad;
    float bodyPitchRad;
    float bodyYawRad;
    float homeYawRad;
    float baseThrottleChannel;
    float desiredRadius;
    float desiredSpeed;
    float yawRateLimit;
} followModeFrame_t;

typedef struct {
    bool enabled;
    float rollChannel;
    float pitchChannel;
    float yawChannel;
    float throttleChannel;
} followModeOutput_t;

typedef struct {
    followControlMode_e mode;
    uint32_t enterTimeMs;
    uint32_t updateCount;
    float state[FOLLOW_MODE_STATE_SIZE];
} followModeRuntime_t;

typedef struct followModeHandler_s {
    followControlMode_e mode;
    const char *name;
    void (*init)(followModeRuntime_t *runtime, const followModeFrame_t *frame, followModeOutput_t *output);
    void (*reset)(followModeRuntime_t *runtime);
    bool (*update)(followModeRuntime_t *runtime, const followModeFrame_t *frame, followModeOutput_t *output);
} followModeHandler_t;

void followModeOutputReset(followModeOutput_t *output);
void followModeOutputHold(followModeOutput_t *output, float throttleChannel);
void followModeOutputApply(const followModeOutput_t *output);
void followModeBuildFrameFromSystem(followModeFrame_t *frame);

void followModeRuntimeReset(followModeRuntime_t *runtime);
bool followModeRegister(const followModeHandler_t *handler);
void followModeResetRegistry(void);
const followModeHandler_t *followModeGetHandler(followControlMode_e mode);
bool followModeSelect(followModeRuntime_t *runtime, followControlMode_e mode, const followModeFrame_t *frame, followModeOutput_t *output);
bool followModeStep(followModeRuntime_t *runtime, const followModeFrame_t *frame, followModeOutput_t *output);
