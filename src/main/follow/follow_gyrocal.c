#include "follow/follow_gyrocal.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <strings.h>

#include "platform.h"

#include "common/axis.h"
#include "common/utils.h"

#include "config/config.h"
#include "fc/runtime_config.h"
#include "sensors/gyro.h"

static int32_t gyroCalibrationValueFromZero(const float gyroZero)
{
    return lrintf(gyroZero * GYRO_CALIBRATION_VALUE_SCALE);
}

static float gyroZeroFromCalibrationValue(const int32_t value)
{
    return value / (float)GYRO_CALIBRATION_VALUE_SCALE;
}

static const gyroCalibrationConfig_t *gyroSavedCalibration(uint8_t gyroIndex)
{
    if (gyroIndex >= GYRO_CALIBRATION_CONFIG_COUNT) {
        return NULL;
    }

    return &gyroConfig()->gyroCalibration[gyroIndex];
}

static gyroCalibrationConfig_t *gyroSavedCalibrationMutable(uint8_t gyroIndex)
{
    if (gyroIndex >= GYRO_CALIBRATION_CONFIG_COUNT) {
        return NULL;
    }

    return &gyroConfigMutable()->gyroCalibration[gyroIndex];
}

static void gyroClearSavedCalibration(uint8_t gyroIndex)
{
    gyroCalibrationConfig_t *savedCalibration = gyroSavedCalibrationMutable(gyroIndex);

    if (!savedCalibration) {
        return;
    }

    for (int valueIndex = 0; valueIndex < GYRO_CALIBRATION_CONFIG_VALUE_COUNT; valueIndex++) {
        savedCalibration->raw[valueIndex] = 0;
    }
}

static void gyroStoreSensorCalibration(const gyroSensor_t *gyroSensor, uint8_t gyroIndex)
{
    gyroCalibrationConfig_t *savedCalibration = gyroSavedCalibrationMutable(gyroIndex);

    if (!savedCalibration) {
        return;
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        savedCalibration->raw[axis] = gyroCalibrationValueFromZero(gyroSensor->gyroDev.gyroZero[axis]);
    }

    savedCalibration->values.calibrationCompleted = 1;
}

void gyroApplySavedCalibration(gyroSensor_t *gyroSensor, uint8_t gyroIndex)
{
    const gyroCalibrationConfig_t *savedCalibration = gyroSavedCalibration(gyroIndex);

    gyroSensor->calibration.cyclesRemaining = 0;
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        gyroSensor->gyroDev.gyroZero[axis] = 0.0f;
    }

    if (!savedCalibration || !savedCalibration->values.calibrationCompleted) {
        return;
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        gyroSensor->gyroDev.gyroZero[axis] = gyroZeroFromCalibrationValue(savedCalibration->raw[axis]);
    }
}

bool gyroSaveCalibration(void)
{
    if (!gyroIsCalibrationComplete()) {
        return false;
    }

    switch (gyro.gyroToUse) {
    default:
    case GYRO_CONFIG_USE_GYRO_1:
        gyroStoreSensorCalibration(&gyro.gyroSensor1, GYRO_CONFIG_USE_GYRO_1);
        break;
#ifdef USE_MULTI_GYRO
    case GYRO_CONFIG_USE_GYRO_2:
        gyroStoreSensorCalibration(&gyro.gyroSensor2, GYRO_CONFIG_USE_GYRO_2);
        break;
    case GYRO_CONFIG_USE_GYRO_BOTH:
        gyroStoreSensorCalibration(&gyro.gyroSensor1, GYRO_CONFIG_USE_GYRO_1);
        gyroStoreSensorCalibration(&gyro.gyroSensor2, GYRO_CONFIG_USE_GYRO_2);
        break;
#endif
    }

    saveConfigAndNotify();

    return true;
}

void gyroResetSavedCalibration(void)
{
    for (int gyroIndex = 0; gyroIndex < GYRO_CALIBRATION_CONFIG_COUNT; gyroIndex++) {
        gyroClearSavedCalibration(gyroIndex);
    }

    gyroApplySavedCalibration(&gyro.gyroSensor1, GYRO_CONFIG_USE_GYRO_1);
#ifdef USE_MULTI_GYRO
    gyroApplySavedCalibration(&gyro.gyroSensor2, GYRO_CONFIG_USE_GYRO_2);
#endif

    saveConfigAndNotify();
}

#ifdef USE_CLI
void cliPrint(const char *str);
void cliPrintLine(const char *str);
void cliPrintLinef(const char *format, ...);

static bool followCliArgEmpty(const char *cmdline)
{
    return !cmdline || !*cmdline;
}

static void followCliPrintHashLine(const char *str)
{
    cliPrint("\r\n# ");
    cliPrintLine(str);
}

static void followCliPrintErrorLine(const char *cmdName, const char *message)
{
    cliPrintLinef("###ERROR IN %s: %s###", cmdName, message);
}

static void followCliGyroCalPrintRuntime(uint8_t gyroIndex, const gyroSensor_t *gyroSensor)
{
    cliPrintLinef("# runtime gyro %d: %d,%d,%d",
        gyroIndex + 1,
        (int)gyroCalibrationValueFromZero(gyroSensor->gyroDev.gyroZero[X]),
        (int)gyroCalibrationValueFromZero(gyroSensor->gyroDev.gyroZero[Y]),
        (int)gyroCalibrationValueFromZero(gyroSensor->gyroDev.gyroZero[Z]));
}

static void followCliGyroCalPrintSaved(uint8_t gyroIndex)
{
    const gyroCalibrationConfig_t *savedCalibration = &gyroConfig()->gyroCalibration[gyroIndex];

    cliPrintLinef("# saved gyro %d: %d,%d,%d,%d",
        gyroIndex + 1,
        (int)savedCalibration->raw[X],
        (int)savedCalibration->raw[Y],
        (int)savedCalibration->raw[Z],
        (int)savedCalibration->values.calibrationCompleted);
}

static void followCliGyroCalShow(void)
{
    cliPrintLinef("# gyro calibration: %s", gyroIsCalibrationComplete() ? "complete" : "running");
    cliPrintLinef("# scale: raw gyro zero * %d", GYRO_CALIBRATION_VALUE_SCALE);

    switch (gyro.gyroToUse) {
    default:
    case GYRO_CONFIG_USE_GYRO_1:
        followCliGyroCalPrintRuntime(GYRO_CONFIG_USE_GYRO_1, &gyro.gyroSensor1);
        break;
#ifdef USE_MULTI_GYRO
    case GYRO_CONFIG_USE_GYRO_2:
        followCliGyroCalPrintRuntime(GYRO_CONFIG_USE_GYRO_2, &gyro.gyroSensor2);
        break;
    case GYRO_CONFIG_USE_GYRO_BOTH:
        followCliGyroCalPrintRuntime(GYRO_CONFIG_USE_GYRO_1, &gyro.gyroSensor1);
        followCliGyroCalPrintRuntime(GYRO_CONFIG_USE_GYRO_2, &gyro.gyroSensor2);
        break;
#endif
    }

    for (int gyroIndex = 0; gyroIndex < GYRO_CALIBRATION_CONFIG_COUNT; gyroIndex++) {
        followCliGyroCalPrintSaved(gyroIndex);
    }
}

void followCliGyroCal(const char *cmdName, char *cmdline)
{
    if (followCliArgEmpty(cmdline)) {
        followCliGyroCalShow();
        return;
    }

    char *saveptr = NULL;
    char *pch = strtok_r(cmdline, " ", &saveptr);
    if (!pch) {
        followCliGyroCalShow();
        return;
    }

    char *extra = strtok_r(NULL, " ", &saveptr);
    if (extra) {
        followCliPrintErrorLine(cmdName, "INVALID ARGUMENT COUNT");
        return;
    }

    if (strcasecmp(pch, "status") == 0 || strcasecmp(pch, "show") == 0) {
        followCliGyroCalShow();
    } else if (strcasecmp(pch, "start") == 0) {
        if (ARMING_FLAG(ARMED)) {
            followCliPrintErrorLine(cmdName, "ARMED");
            return;
        }

        gyroStartCalibration(false);
        followCliPrintHashLine("gyro calibration started");
    } else if (strcasecmp(pch, "save") == 0) {
        if (ARMING_FLAG(ARMED)) {
            followCliPrintErrorLine(cmdName, "ARMED");
            return;
        }

        if (!gyroSaveCalibration()) {
            followCliPrintErrorLine(cmdName, "CALIBRATION NOT COMPLETE");
            return;
        }

        followCliPrintHashLine("gyro calibration saved");
    } else if (strcasecmp(pch, "reset") == 0) {
        if (ARMING_FLAG(ARMED)) {
            followCliPrintErrorLine(cmdName, "ARMED");
            return;
        }

        gyroResetSavedCalibration();
        followCliPrintHashLine("gyro calibration reset");
    } else {
        followCliPrintErrorLine(cmdName, "PARSING FAILED");
    }
}
#else
void followCliGyroCal(const char *cmdName, char *cmdline)
{
    UNUSED(cmdName);
    UNUSED(cmdline);
}
#endif
