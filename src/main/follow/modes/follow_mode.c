#include "follow/modes/follow_mode.h"

static const followModeHandler_t *followModeHandlers[FOLLOW_MODE_HANDLER_COUNT];

static float followModeLimit(float value, float minValue, float maxValue)
{
    return followLimit(value, minValue, maxValue);
}

static float followModeResolveThrottle(float throttleChannel)
{
    if (throttleChannel <= 0.0f) {
        if (followSystemData.targetAccChannel > 0.0f) {
            return followSystemData.targetAccChannel;
        }

        if (followSystemData.initialAccChannel > 0U) {
            return (float)followSystemData.initialAccChannel;
        }

        return RC_RANGE_TH_MIN;
    }

    return throttleChannel;
}

void followModeOutputReset(followModeOutput_t *output)
{
    if (!output) {
        return;
    }

    output->enabled = false;
    output->rollChannel = RC_RANGE_TERM;
    output->pitchChannel = RC_RANGE_TERM;
    output->yawChannel = RC_RANGE_TERM;
    output->throttleChannel = followModeResolveThrottle(0.0f);
}

void followModeOutputHold(followModeOutput_t *output, float throttleChannel)
{
    if (!output) {
        return;
    }

    output->enabled = false;
    output->rollChannel = RC_RANGE_TERM;
    output->pitchChannel = RC_RANGE_TERM;
    output->yawChannel = RC_RANGE_TERM;
    output->throttleChannel = followModeLimit(followModeResolveThrottle(throttleChannel), RC_RANGE_TH_MIN, RC_RANGE_TH_MAX);
}

void followModeOutputApply(const followModeOutput_t *output)
{
    if (!output) {
        return;
    }

    followSystemData.targetRollChannel = followModeLimit(output->rollChannel, RC_RANGE_MIN, RC_RANGE_MAX);
    followSystemData.targetPitchChannel = followModeLimit(output->pitchChannel, RC_RANGE_MIN, RC_RANGE_MAX);
    followSystemData.targetYawChannel = followModeLimit(output->yawChannel, RC_RANGE_MIN, RC_RANGE_MAX);
    followSystemData.targetAccChannel = followModeLimit(output->throttleChannel, RC_RANGE_TH_MIN, RC_RANGE_TH_MAX);
}

void followModeBuildFrameFromSystem(followModeFrame_t *frame)
{
    if (!frame) {
        return;
    }

    memset(frame, 0, sizeof(*frame));
    frame->valid = followSystemData.mode == 1;
    frame->targetLocked = followSystemData.traceState == 1;
    frame->dt = 0.01f;
    frame->targetDistance = 0.0f;
    frame->targetBearing = 0.0f;
    frame->targetYawRate = 0.0f;
    frame->targetErrorX = followSystemData.trackErrorX;
    frame->targetErrorY = followSystemData.trackErrorY;
    frame->bodyRollRad = followSystemData.rollRad;
    frame->bodyPitchRad = followSystemData.pitchRad;
    frame->bodyYawRad = followSystemData.yawRad;
    frame->homeYawRad = followSystemData.startYawDeg / 57.29578f;
    frame->baseThrottleChannel = followModeResolveThrottle(followSystemData.targetAccChannel);
    frame->desiredRadius = 30.0f;
    frame->desiredSpeed = 1.0f;
    frame->yawRateLimit = 320.0f;
}

void followModeRuntimeReset(followModeRuntime_t *runtime)
{
    if (!runtime) {
        return;
    }

    memset(runtime, 0, sizeof(*runtime));
    runtime->mode = FOLLOW_CONTROL_MODE_NONE;
}

bool followModeRegister(const followModeHandler_t *handler)
{
    int index;

    if (!handler || handler->mode <= FOLLOW_CONTROL_MODE_NONE || handler->mode >= FOLLOW_MODE_HANDLER_COUNT) {
        return false;
    }

    index = (int)handler->mode;
    followModeHandlers[index] = handler;
    return true;
}

void followModeResetRegistry(void)
{
    memset(followModeHandlers, 0, sizeof(followModeHandlers));
}

const followModeHandler_t *followModeGetHandler(followControlMode_e mode)
{
    int index = (int)mode;

    if (index <= FOLLOW_CONTROL_MODE_NONE || index >= FOLLOW_MODE_HANDLER_COUNT) {
        return NULL;
    }

    return followModeHandlers[index];
}

bool followModeSelect(followModeRuntime_t *runtime, followControlMode_e mode, const followModeFrame_t *frame, followModeOutput_t *output)
{
    const followModeHandler_t *handler;

    if (!runtime) {
        return false;
    }

    handler = followModeGetHandler(mode);
    if (!handler) {
        return false;
    }

    memset(runtime->state, 0, sizeof(runtime->state));
    runtime->mode = mode;
    runtime->enterTimeMs = millis();
    runtime->updateCount = 0U;

    if (handler->init) {
        handler->init(runtime, frame, output);
    } else {
        followModeOutputHold(output, frame ? frame->baseThrottleChannel : 0.0f);
    }

    return true;
}

bool followModeStep(followModeRuntime_t *runtime, const followModeFrame_t *frame, followModeOutput_t *output)
{
    const followModeHandler_t *handler;

    if (!runtime || !output) {
        return false;
    }

    handler = followModeGetHandler(runtime->mode);
    if (!handler || !handler->update) {
        followModeOutputHold(output, frame ? frame->baseThrottleChannel : 0.0f);
        return false;
    }

    runtime->updateCount++;
    return handler->update(runtime, frame, output);
}
